// SPDX-License-Identifier: GPL-2.0-only
/*
 * ARM64 DRTM Secure Launch — EFI stub component. Builds DRTM_PARAMETERS
 * and issues DRTM_DYNAMIC_LAUNCH when the cmdline has "drtm=on". The SMC
 * does not return on success: D-CRTM measures the image and ERETs to
 * sl_entry. Copyright (c) 2025-2026, NVIDIA Corporation.
 */

#include <linux/efi.h>
#include <linux/libfdt.h>
#include <linux/psci.h>
#include <uapi/linux/psci.h>
#include <asm/drtm.h>
#include <asm/cputype.h>
#include <asm/efi.h>
#include <asm/sections.h>
#include <asm/sysreg.h>

#include "efistub.h"

/* DRTM SMC IDs — duplicated here for EFI stub isolation */
#define SL_DRTM_SMC_FEATURES		0xC4000111UL
#define SL_DRTM_SMC_DYNAMIC_LAUNCH	0xC4000114UL
#define SL_DRTM_PAGE_SIZE		0x1000
#define SL_ROUND_UP_PAGE(x)		(((x) + SL_DRTM_PAGE_SIZE - 1) & \
					 ~(SL_DRTM_PAGE_SIZE - 1ULL))

/*
 * Preamble<->DLME DTB-PA convention slot (mirrors SL_DLME_DTB_SLOT_OFFSET
 * in arch/arm64/include/asm/drtm.h — keep them in sync).
 */
#define SL_DLME_DTB_SLOT_OFFSET		(-8)

/* From sl_stub.S — accessible via __efistub_ alias in image-vars.h */
extern char sl_entry[];
#ifdef CONFIG_ARM64_SECURE_LAUNCH_FAULT_INJECT
extern char sl_test_ap_entry[];
#endif

/*
 * DRTM Parameters (DEN0113 v1.2 §3.13 / Table 9)
 * Struct must be packed — passed directly to TF-A via SMC.
 */
struct sl_drtm_params {
	__le16	revision;
	__le16	reserved;
	__le32	launch_features;
	__le64	dlme_region_address;
	__le64	dlme_region_size;
	__le64	dlme_image_start;
	__le64	dlme_entry_point_offset;
	__le64	dlme_image_size;
	__le64	dlme_data_offset;
	__le64	nw_dce_region_address;
	__le64	nw_dce_region_size;
	__le64	mem_prot_table_address;
	__le64	mem_prot_table_size;
} __packed;

#ifdef CONFIG_ARM64_SECURE_LAUNCH_FAULT_INJECT
#define SL_EFI_PROCESSOR_AS_BSP	BIT(0)
#define SL_EFI_PROCESSOR_ENABLED	BIT(1)
#define SL_TEST_TIMEOUT_US	1000000

struct sl_efi_cpu_location {
	u32 package;
	u32 core;
	u32 thread;
};

struct sl_efi_cpu_location2 {
	u32 package;
	u32 module;
	u32 tile;
	u32 die;
	u32 core;
	u32 thread;
};

struct sl_efi_processor_info {
	u64 processor_id;
	u32 status_flag;
	struct sl_efi_cpu_location location;
	struct sl_efi_cpu_location2 extended_information;
};

struct sl_efi_mp_services;

struct sl_efi_mp_services {
	efi_status_t (__efiapi *get_number_of_processors)(
		struct sl_efi_mp_services *this, unsigned long *total,
		unsigned long *enabled);
	efi_status_t (__efiapi *get_processor_info)(
		struct sl_efi_mp_services *this, unsigned long processor,
		struct sl_efi_processor_info *info);
	void *startup_all_aps;
	void *startup_this_ap;
	void *switch_bsp;
	void *enable_disable_ap;
	void *who_am_i;
};

static efi_guid_t sl_efi_mp_services_guid =
	EFI_GUID(0x3fdda605, 0xa76e, 0x4f46, 0xad, 0x29, 0x12, 0xf4,
		 0x53, 0x1b, 0x3d, 0x08);

struct sl_test_status {
	u32 state;
	u32 reason;
	u32 flags;
	u64 target_mpidr;
	u64 scratch_pa;
	s64 drtm_rc;
	s64 detail_rc;
};

static struct sl_test_status sl_test;
#endif

static u64 sl_smc_ret4(u64 fn, u64 arg1, u64 arg2, u64 arg3)
{
	register u64 x0 __asm__("x0") = fn;
	register u64 x1 __asm__("x1") = arg1;
	register u64 x2 __asm__("x2") = arg2;
	register u64 x3 __asm__("x3") = arg3;

	asm volatile("smc #0"
		: "+r"(x0), "+r"(x1), "+r"(x2), "+r"(x3)
		:
		: "x4", "x5", "x6", "x7", "x8", "x9", "x10",
		  "x11", "x12", "x13", "x14", "x15", "x16", "x17",
		  "memory");
	return x0;
}

static u64 sl_smc_ret(u64 fn, u64 arg1)
{
	return sl_smc_ret4(fn, arg1, 0, 0);
}

static void sl_smc(u64 fn, u64 arg1)
{
	sl_smc_ret(fn, arg1);
}

/* Read CTR_EL0.DminLine and return cache line size in bytes. */
static inline unsigned int sl_dcache_line_size(void)
{
	u64 ctr;

	asm volatile("mrs %0, ctr_el0" : "=r"(ctr));
	return 4U << ((ctr >> 16) & 0xfU);
}

/*
 * Clean (cvac) a range to PoC at cache-line granularity so TF-A sees
 * the stub's freshly-written value from EL3.
 */
static inline void sl_dc_cvac_range(unsigned long start, unsigned long len)
{
	unsigned int line = sl_dcache_line_size();
	unsigned long mask = (unsigned long)line - 1UL;
	unsigned long end = start + len;
	unsigned long addr;

	start &= ~mask;
	for (addr = start; addr < end; addr += line)
		asm volatile("dc cvac, %0" : : "r"(addr) : "memory");
}

/*
 * DLME data reserve (from DRTM_FEATURES) and whether D-CRTM advertised
 * DRTM support. Consumed by arm64-stub.c and the launch gate in fdt.c.
 */
unsigned long sl_dlme_data_reserve;
bool sl_drtm_available;

/*
 * Query DRTM_FEATURES (DEN0113 v1.2 Table 6, feature 0x2) for the minimum
 * DLME data size. Best-effort: failure leaves DRTM unavailable (normal
 * boot). Called before ExitBootServices.
 */
void efi_slaunch_get_dlme_data_size(void)
{
	register u64 x0 __asm__("x0") = SL_DRTM_SMC_FEATURES;
	register u64 x1 __asm__("x1") = (1ULL << 63) | 0x2;
	register u64 x2 __asm__("x2") = 0;
	register u64 x3 __asm__("x3") = 0;
	u32 min_pages;

	asm volatile("smc #0"
		: "+r"(x0), "+r"(x1), "+r"(x2), "+r"(x3)
		:
		: "x4", "x5", "x6", "x7", "x8", "x9", "x10",
		  "x11", "x12", "x13", "x14", "x15", "x16", "x17",
		  "memory");

	/* DRTM_FEATURES success is x0 > 0; x1[31:0] = min DLME data pages. */
	if ((s64)x0 <= 0)
		return;
	min_pages = (u32)(x1 & 0xFFFFFFFF);
	if (min_pages == 0)
		return;

	sl_dlme_data_reserve = (unsigned long)min_pages * SL_DRTM_PAGE_SIZE;
	sl_drtm_available = true;
	efi_info("DRTM: min DLME data size %lu KB (%u pages)\n",
		 sl_dlme_data_reserve / 1024, min_pages);
}

/*
 * Zero the PE/COFF Optional Header ImageBase on the relocated kernel
 * buffer (pre-EBS): LoadImage patched it with the load PA, diverging the
 * D-CRTM-measured bytes from the on-disk Image. Pre-EBS because
 * efi_remap_image() marked the header RO; flip RW, zero, restore RO.
 */
void efi_slaunch_scrub_imagebase(unsigned long kernel_addr)
{
	efi_guid_t guid = EFI_MEMORY_ATTRIBUTE_PROTOCOL_GUID;
	efi_memory_attribute_protocol_t *memattr;
	efi_status_t status;
	u32 e_lfanew;
	volatile u64 *image_base_ptr;
	unsigned long page_base;

	if (!kernel_addr)
		return;

	e_lfanew = *(volatile u32 *)((char *)kernel_addr + 0x3c);
	image_base_ptr = (volatile u64 *)((char *)kernel_addr +
					  e_lfanew + 4 + 20 + 0x18);
	page_base = (unsigned long)image_base_ptr & ~(SL_DRTM_PAGE_SIZE - 1UL);

	status = efi_bs_call(locate_protocol, &guid, NULL, (void **)&memattr);
	if (status != EFI_SUCCESS) {
		efi_warn("DRTM: no EFI_MEMORY_ATTRIBUTE_PROTOCOL; "
			 "skipping PE ImageBase scrub\n");
		return;
	}

	status = memattr->clear_memory_attributes(memattr, page_base,
						  SL_DRTM_PAGE_SIZE,
						  EFI_MEMORY_RO);
	if (status != EFI_SUCCESS) {
		efi_warn("DRTM: clear EFI_MEMORY_RO failed for PE header page: 0x%lx\n",
			 status);
		return;
	}

	*image_base_ptr = 0;
	sl_dc_cvac_range((unsigned long)image_base_ptr, 8);
	asm volatile("dsb sy" : : : "memory");

	status = memattr->set_memory_attributes(memattr, page_base,
						SL_DRTM_PAGE_SIZE,
						EFI_MEMORY_RO);
	if (status != EFI_SUCCESS)
		efi_warn("DRTM: restore EFI_MEMORY_RO on PE header page failed: 0x%lx\n",
			 status);

	efi_info("DRTM: PE ImageBase zeroed at 0x%lx\n",
		 (unsigned long)image_base_ptr);
}

/*
 * Token-aware cmdline match: true iff `tok` is a standalone
 * whitespace-delimited word in `cmdline`, not a substring of another
 * option (so "drtm=on" does not match "nodrtm=on" or "root=...drtm=on").
 */
static bool sl_cmdline_token(const char *cmdline, const char *tok)
{
	size_t toklen = strlen(tok);
	const char *p = cmdline;

	while ((p = strstr(p, tok)) != NULL) {
		bool start_ok = (p == cmdline) || p[-1] == ' ' || p[-1] == '\t';
		bool end_ok = p[toklen] == '\0' || p[toklen] == ' ' ||
			      p[toklen] == '\t';

		if (start_ok && end_ok)
			return true;
		p += toklen;
	}
	return false;
}

bool efi_slaunch_enabled(const char *cmdline)
{
	if (!cmdline)
		return false;
	/*
	 * A detected DRTM launch forces EFI runtime services off in
	 * slaunch_setup() (via disable_runtime), so no efi=noruntime token is
	 * needed here; drtm=on alone requests the launch.
	 */
	return sl_cmdline_token(cmdline, "drtm=on");
}

/* Canonical converted command line, recorded once by the stub entry. */
static const char *sl_cmdline;

/*
 * Record the stub's canonical converted command line for the DRTM gates.
 * Reusing it avoids a second efi_convert_cmdline(), which would measure
 * the EFI LoadOptions a second time (duplicate PCR 9 event).
 */
void efi_slaunch_set_cmdline(const char *cmdline)
{
	sl_cmdline = cmdline;
}

/*
 * True iff the recorded command line requests a DRTM launch. Gates the
 * DRTM_FEATURES probe: an SMC faults on a platform with no EL3 monitor,
 * so it must not be issued on boots that never asked for a launch.
 */
bool efi_slaunch_requested(void)
{
	return efi_slaunch_enabled(sl_cmdline);
}

#ifdef CONFIG_ARM64_SECURE_LAUNCH_FAULT_INJECT
static const char *sl_cmdline_value(const char *key)
{
	size_t keylen = strlen(key);
	const char *p = sl_cmdline;

	if (!p)
		return NULL;

	while ((p = strstr(p, key)) != NULL) {
		if ((p == sl_cmdline || p[-1] == ' ' || p[-1] == '\t') &&
		    p[keylen] != '\0' && p[keylen] != ' ' && p[keylen] != '\t')
			return p + keylen;
		p += keylen;
	}
	return NULL;
}

bool efi_slaunch_test_requested(void)
{
	return sl_cmdline &&
		sl_cmdline_token(sl_cmdline, "slaunch_inject=secondary_pe_on");
}

static s64 sl_psci_affinity_info(u64 mpidr)
{
	return (s64)sl_smc_ret4(PSCI_0_2_FN64_AFFINITY_INFO, mpidr, 0, 0);
}

static void sl_test_set_failure(u32 reason, s64 detail)
{
	sl_test.state = SL_TEST_STATE_FAIL;
	sl_test.reason = reason;
	sl_test.detail_rc = detail;
}

static void sl_test_record_to_fdt(struct sl_test_fdt_record *record)
{
	record->version = cpu_to_fdt32(SL_TEST_RECORD_VERSION);
	record->state = cpu_to_fdt32(sl_test.state);
	record->reason = cpu_to_fdt32(sl_test.reason);
	record->flags = cpu_to_fdt32(sl_test.flags);
	record->target_mpidr = cpu_to_fdt64(sl_test.target_mpidr);
	record->scratch_pa = cpu_to_fdt64(sl_test.scratch_pa);
	record->drtm_rc = cpu_to_fdt64((u64)sl_test.drtm_rc);
	record->detail_rc = cpu_to_fdt64((u64)sl_test.detail_rc);
}

int efi_slaunch_test_add_fdt_record(void *fdt, int chosen)
{
	struct sl_test_fdt_record record;

	if (!efi_slaunch_test_requested())
		return 0;

	sl_test_record_to_fdt(&record);
	return fdt_setprop(fdt, chosen, SL_TEST_FDT_PROP, &record,
			   sizeof(record));
}

static void sl_test_update_fdt_record(void *fdt)
{
	struct sl_test_fdt_record record;
	int chosen;

	chosen = fdt_path_offset(fdt, "/chosen");
	if (chosen < 0)
		return;

	sl_test_record_to_fdt(&record);
	if (fdt_setprop_inplace(fdt, chosen, SL_TEST_FDT_PROP, &record,
				sizeof(record)))
		return;

	sl_dc_cvac_range((unsigned long)fdt, fdt_totalsize(fdt));
	asm volatile("dsb sy" : : : "memory");
}

static bool sl_test_parse_target(u64 *target)
{
	const char *value = sl_cmdline_value("slaunch_inject_mpidr=");
	char *end;
	u64 mpidr;

	if (!value)
		return false;

	mpidr = simple_strtoull(value, &end, 0);
	if (end == value || (*end && *end != ' ' && *end != '\t') ||
	    (mpidr & ~MPIDR_HWID_BITMASK)) {
		sl_test_set_failure(SL_TEST_REASON_SETUP, -EINVAL);
		return true;
	}

	*target = mpidr;
	return true;
}

static bool sl_test_wait_ready(struct sl_test_ap_control *ctrl)
{
	unsigned int i;

	for (i = 0; i < SL_TEST_TIMEOUT_US / 10; i++) {
		asm volatile("dc ivac, %0" : : "r"(ctrl) : "memory");
		asm volatile("dsb sy" : : : "memory");
		if (READ_ONCE(ctrl->ready))
			return true;
		efi_bs_call(stall, 10);
	}
	return false;
}

static bool sl_test_stop_ap(void);

void efi_slaunch_test_prepare(unsigned long kernel_addr)
{
	struct sl_efi_mp_services *mp;
	struct sl_efi_processor_info info;
	struct sl_test_ap_control *ctrl;
	efi_status_t status;
	unsigned long total, enabled, i;
	unsigned long kernel_memsize, ap_entry;
	u64 requested_mpidr = 0, current_mpidr;
	bool explicit_target, found = false;
	s64 rc;

	if (!efi_slaunch_test_requested())
		return;

	memset(&sl_test, 0, sizeof(sl_test));
	sl_test.state = SL_TEST_STATE_FAIL;
	sl_test.reason = SL_TEST_REASON_SETUP;
	sl_test.drtm_rc = DRTM_NOT_SUPPORTED;

	if (!efi_slaunch_requested() || !sl_drtm_available) {
		sl_test.detail_rc = -EOPNOTSUPP;
		return;
	}

	explicit_target = sl_test_parse_target(&requested_mpidr);
	if (sl_test.state == SL_TEST_STATE_FAIL && sl_test.detail_rc)
		return;

	status = efi_bs_call(locate_protocol, &sl_efi_mp_services_guid,
			     NULL, (void **)&mp);
	if (status != EFI_SUCCESS) {
		sl_test.detail_rc = status;
		return;
	}

	status = mp->get_number_of_processors(mp, &total, &enabled);
	if (status != EFI_SUCCESS || enabled < 2) {
		sl_test.detail_rc = status != EFI_SUCCESS ? status : -ENODEV;
		return;
	}

	asm volatile("mrs %0, mpidr_el1" : "=r"(current_mpidr));
	current_mpidr &= MPIDR_HWID_BITMASK;

	for (i = 0; i < total; i++) {
		u64 mpidr;

		status = mp->get_processor_info(mp, i, &info);
		if (status != EFI_SUCCESS ||
		    !(info.status_flag & SL_EFI_PROCESSOR_ENABLED) ||
		    (info.status_flag & SL_EFI_PROCESSOR_AS_BSP))
			continue;

		mpidr = info.processor_id & MPIDR_HWID_BITMASK;
		if (mpidr == current_mpidr ||
		    (explicit_target && mpidr != requested_mpidr))
			continue;

		sl_test.target_mpidr = mpidr;
		found = true;
		break;
	}

	if (!found) {
		sl_test.detail_rc = -ENODEV;
		return;
	}

	rc = sl_psci_affinity_info(sl_test.target_mpidr);
	if (rc != PSCI_0_2_AFFINITY_LEVEL_OFF) {
		sl_test.detail_rc = rc;
		return;
	}

	kernel_memsize = (unsigned long)(_end - _text);
	sl_test.scratch_pa = kernel_addr + SL_ROUND_UP_PAGE(kernel_memsize);
	ctrl = (struct sl_test_ap_control *)(unsigned long)sl_test.scratch_pa;
	memset(ctrl, 0, sizeof(*ctrl));
	sl_dc_cvac_range((unsigned long)ctrl, sizeof(*ctrl));
	asm volatile("dsb sy" : : : "memory");

	ap_entry = kernel_addr +
		((unsigned long)sl_test_ap_entry - (unsigned long)_text);
	rc = (s64)sl_smc_ret4(PSCI_0_2_FN64_CPU_ON, sl_test.target_mpidr,
			       ap_entry, sl_test.scratch_pa);
	if (rc != PSCI_RET_SUCCESS) {
		sl_test.detail_rc = rc;
		return;
	}
	sl_test.flags |= SL_TEST_FLAG_AP_STARTED;

	if (!sl_test_wait_ready(ctrl)) {
		sl_test.detail_rc = -ETIMEDOUT;
		if (!sl_test_stop_ap())
			sl_test.reason = SL_TEST_REASON_CLEANUP;
		return;
	}

	sl_test.state = SL_TEST_STATE_ARMED;
	sl_test.reason = SL_TEST_REASON_NONE;
	sl_test.flags |= SL_TEST_FLAG_AP_READY;
	sl_test.detail_rc = 0;
	efi_err("DRTM test: ARMED secondary PE MPIDR 0x%llx\n",
		sl_test.target_mpidr);
}

static bool sl_test_stop_ap(void)
{
	struct sl_test_ap_control *ctrl;
	u64 deadline;
	s64 rc;

	if (!(sl_test.flags & SL_TEST_FLAG_AP_STARTED))
		return true;

	ctrl = (struct sl_test_ap_control *)(unsigned long)sl_test.scratch_pa;
	WRITE_ONCE(ctrl->release, 1);
	sl_dc_cvac_range((unsigned long)&ctrl->release, sizeof(ctrl->release));
	asm volatile("dsb sy; sev" : : : "memory");

	deadline = read_sysreg(cntvct_el0) + read_sysreg(cntfrq_el0);
	do {
		rc = sl_psci_affinity_info(sl_test.target_mpidr);
		if (rc == PSCI_0_2_AFFINITY_LEVEL_OFF) {
			sl_test.flags |= SL_TEST_FLAG_AP_OFF;
			return true;
		}
		asm volatile("yield");
	} while ((s64)(deadline - read_sysreg(cntvct_el0)) > 0);

	asm volatile("dc ivac, %0" : : "r"(ctrl) : "memory");
	asm volatile("dsb sy" : : : "memory");
	sl_test.detail_rc = READ_ONCE(ctrl->cpu_off_rc);
	if (!sl_test.detail_rc)
		sl_test.detail_rc = -ETIMEDOUT;
	sl_test.flags |= SL_TEST_FLAG_QUARANTINED;
	return false;
}

void efi_slaunch_test_cancel(void)
{
	if (!efi_slaunch_test_requested() ||
	    !(sl_test.flags & SL_TEST_FLAG_AP_STARTED))
		return;

	if (!sl_test_stop_ap())
		efi_err("DRTM test: failed to stop secondary PE 0x%llx\n",
			sl_test.target_mpidr);
}
#else
bool efi_slaunch_test_requested(void)
{
	return false;
}

void efi_slaunch_test_prepare(unsigned long kernel_addr) { }

void efi_slaunch_test_cancel(void) { }

int efi_slaunch_test_add_fdt_record(void *fdt, int chosen)
{
	return 0;
}
#endif

/*
 * TF-A requires DRTM_PARAMETERS to be 4KB-aligned; we are past
 * ExitBootServices so cannot allocate — use a static buffer.
 */
static struct sl_drtm_params sl_params __aligned(SL_DRTM_PAGE_SIZE);

static struct sl_drtm_params *sl_build_drtm_params(unsigned long kernel_addr,
						    unsigned long fdt_addr)
{
	struct sl_drtm_params *params = &sl_params;
	unsigned long image_size, kernel_memsize;
	unsigned long dlme_data_offset;
	unsigned long sl_entry_offset;

	/*
	 * Store DTB PA just below the DLME data area (dlme_data_offset - 8);
	 * sl_entry finds it via X0 + X1 - 8. Direct physical write avoids
	 * EFI-stub symbol resolution, which can fail under PIC/GOT.
	 */

	/* Compute sl_entry offset from kernel image base */
	sl_entry_offset = (unsigned long)sl_entry - (unsigned long)_text;

	/*
	 * DLME region layout: [_text.._edata] measured image, [_edata.._end]
	 * BSS, then D-CRTM-populated DLME data after _end. FDT is separate
	 * (wherever efi_allocate_pages() put it).
	 */
	image_size = (unsigned long)(_edata - _text);
	kernel_memsize = (unsigned long)(_end - _text);
	dlme_data_offset = SL_ROUND_UP_PAGE(kernel_memsize) + SL_DLME_DTB_SLOT_GAP;

	/*
	 * Write DTB PA into the Preamble->DLME slot at (kernel_addr +
	 * dlme_data_offset + SL_DLME_DTB_SLOT_OFFSET); sl_entry reads it via
	 * X0 + X1 + SL_DLME_DTB_SLOT_OFFSET after the D-CRTM ERET.
	 */
	*(volatile u64 *)(kernel_addr + dlme_data_offset +
			  SL_DLME_DTB_SLOT_OFFSET) = fdt_addr;

	/* Build DRTM_PARAMETERS */
	params->revision = cpu_to_le16(DRTM_PARAMS_REVISION);
	params->reserved = cpu_to_le16(0);
	/* bits[5:3]=0: complete DMA protection. bit 7: request Secure-interrupt
	 * disable for the launch window (DEN0113 Table 9); the DLME re-enables
	 * post-launch via DRTM_ENABLE_SECURE_INTERRUPTS. */
	params->launch_features = cpu_to_le32(DRTM_LAUNCH_FEAT_MEM_PROT_ALL | DRTM_LAUNCH_FEAT_SEC_INT_DISABLE);
	params->dlme_region_address = cpu_to_le64(kernel_addr);
	params->dlme_region_size = cpu_to_le64(dlme_data_offset + sl_dlme_data_reserve);
	params->dlme_image_start = cpu_to_le64(0);
	params->dlme_entry_point_offset = cpu_to_le64(sl_entry_offset);
	params->dlme_image_size = cpu_to_le64(image_size);
	params->dlme_data_offset = cpu_to_le64(dlme_data_offset);
	params->nw_dce_region_address = cpu_to_le64(0);
	params->nw_dce_region_size = cpu_to_le64(0);
	/* Complete DMA protection: table must be zero (DEN0113 v1.2 Table 9). */
	params->mem_prot_table_address = cpu_to_le64(0);
	params->mem_prot_table_size = cpu_to_le64(0);

	/*
	 * Clean to DRAM what the D-CRTM reads after the SMC: the params
	 * struct and the DTB PA slot (outside TF-A's own DLME flush).
	 */
	sl_dc_cvac_range((unsigned long)params, sizeof(*params));
	sl_dc_cvac_range(kernel_addr + dlme_data_offset +
			 SL_DLME_DTB_SLOT_OFFSET, sizeof(u64));
	/*
	 * NOTE: the PE ImageBase scrub happens pre-EBS in
	 * efi_slaunch_scrub_imagebase(); the memory-attribute protocol used
	 * to unprotect the header page is unreachable after boot services exit.
	 */
	asm volatile("dsb sy" : : : "memory");
	return params;
}

void __noreturn efi_slaunch_drtm(unsigned long kernel_addr,
				 unsigned long fdt_addr)
{
	struct sl_drtm_params *params;

	params = sl_build_drtm_params(kernel_addr, fdt_addr);

	/*
	 * DRTM_DYNAMIC_LAUNCH — does not return on success.
	 * D-CRTM: measures kernel, populates DLME data, ERETs to sl_entry
	 */
	sl_smc(SL_DRTM_SMC_DYNAMIC_LAUNCH, (u64)params);

	/*
	 * The launch SMC returns only on failure (success ERETs to sl_entry).
	 * Boot services are gone here, so we cannot print; the pre-EBS notice
	 * in efi_boot_kernel() flagged that reaching this halt means the
	 * dynamic launch failed.
	 */
	for (;;)
		asm volatile("wfi");
}

#ifdef CONFIG_ARM64_SECURE_LAUNCH_FAULT_INJECT
void efi_slaunch_test_run(unsigned long kernel_addr, unsigned long fdt_addr)
{
	struct sl_drtm_params *params;
	s64 rc;

	if (!efi_slaunch_test_requested())
		return;

	if (sl_test.state != SL_TEST_STATE_ARMED) {
		sl_test_update_fdt_record((void *)fdt_addr);
		return;
	}

	rc = sl_psci_affinity_info(sl_test.target_mpidr);
	if (rc != PSCI_0_2_AFFINITY_LEVEL_ON) {
		sl_test_set_failure(SL_TEST_REASON_POST_EBS, rc);
		sl_test_stop_ap();
		sl_test_update_fdt_record((void *)fdt_addr);
		return;
	}

	/* Make the ARMED breadcrumb visible if firmware incorrectly launches. */
	sl_test_update_fdt_record((void *)fdt_addr);
	params = sl_build_drtm_params(kernel_addr, fdt_addr);
	sl_test.drtm_rc = (s64)sl_smc_ret(SL_DRTM_SMC_DYNAMIC_LAUNCH,
					  (u64)params);

	if (sl_test.drtm_rc == DRTM_SECONDARY_PE_NOT_OFF) {
		sl_test.state = SL_TEST_STATE_PASS;
		sl_test.reason = SL_TEST_REASON_NONE;
		sl_test.detail_rc = 0;
	} else {
		sl_test_set_failure(SL_TEST_REASON_LAUNCH_RETURN,
				    sl_test.drtm_rc);
	}

	if (!sl_test_stop_ap()) {
		sl_test.state = SL_TEST_STATE_FAIL;
		sl_test.reason = SL_TEST_REASON_CLEANUP;
	}

	sl_test_update_fdt_record((void *)fdt_addr);
}
#else
void efi_slaunch_test_run(unsigned long kernel_addr, unsigned long fdt_addr) { }
#endif
