// SPDX-License-Identifier: GPL-2.0-only
/*
 * VFIO support for CXL Type-2 devices.
 *
 * Copyright (c) 2026 NVIDIA Corporation & Affiliates
 */

#include <linux/cleanup.h>
#include <linux/memory-failure.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/range.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/vfio_pci_core.h>
#include <cxl/cxl.h>
#include <cxl/pci.h>

/**
 * struct vfio_cxl_state - per-device state for a vfio-cxl device
 * @cxlds: CXL device state; kept first for devm_cxl_dev_state_create()
 * @cxlmd: memory device joined to the CXL topology at bind
 * @hpa_range: host physical range of the HDM region
 * @dpa_pfn_space: HDM-region pfn range registered with memory_failure()
 * @dvsec: CXL device DVSEC config-space offset
 * @dvsec_len: length of the DVSEC body
 * @dvsec_shadow: guest view of the CXL DVSEC body, sampled at open
 * @hdm_regs: mapped HDM decoder registers, source for the open-time snapshot
 * @hdm_len: length of the HDM decoder register block
 * @hdm_shadow: guest view of the HDM decoder registers, sampled at open
 * @hdm_region_idx: vdev->region[] index of the HDM region
 * @hdm_valid: true when the decoder is in a known-good restored state and host
 *	       CPU access to the HDM range is safe; gated under memory_lock
 */
struct vfio_cxl_state {
	struct cxl_dev_state cxlds;
	struct cxl_memdev *cxlmd;
	struct range hpa_range;
	struct pfn_address_space dpa_pfn_space;
	u16 dvsec;
	u32 dvsec_len;

	u32 *dvsec_shadow;
	void __iomem *hdm_regs;
	u32 hdm_len;

	__le32 *hdm_shadow;
	int hdm_region_idx;
	bool hdm_valid;
};

static unsigned long vfio_cxl_mem_pgoff(struct vm_area_struct *vma,
					unsigned long addr)
{
	unsigned long mask = (1U << (VFIO_PCI_OFFSET_SHIFT - PAGE_SHIFT)) - 1;

	return (vma->vm_pgoff & mask) + ((addr - vma->vm_start) >> PAGE_SHIFT);
}

static vm_fault_t vfio_cxl_mem_huge_fault(struct vm_fault *vmf,
					  unsigned int order)
{
	struct vm_area_struct *vma = vmf->vma;
	struct vfio_pci_core_device *vdev = vma->vm_private_data;
	struct vfio_cxl_state *cxl = vdev->cxl;
	unsigned long addr = ALIGN_DOWN(vmf->address, PAGE_SIZE << order);
	unsigned long pfn = PHYS_PFN(cxl->hpa_range.start) +
			    vfio_cxl_mem_pgoff(vma, addr);
	vm_fault_t ret = VM_FAULT_FALLBACK;

	if (is_aligned_for_order(vma, addr, pfn, order)) {
		scoped_guard(rwsem_read, &vdev->memory_lock) {
			/*
			 * A reset or D3 transition takes memory_lock for write,
			 * revokes this mapping and clears the decoder. Do not
			 * insert a PFN for a decoder that is not in a known-good
			 * state, or the host CPU could reach a disabled decoder.
			 * vfio_pci_vmf_insert_pfn() adds the Memory-Space gate:
			 * an HDM access while the device has Memory-Space disabled
			 * aborts on the fabric as a fatal host SError, so it must
			 * not be faulted in until the guest re-enables it.
			 */
			if (cxl->hdm_valid)
				ret = vfio_pci_vmf_insert_pfn(vdev, vmf, pfn,
							      order);
			else
				ret = VM_FAULT_SIGBUS;
		}
	}

	return ret;
}

static vm_fault_t vfio_cxl_mem_fault(struct vm_fault *vmf)
{
	return vfio_cxl_mem_huge_fault(vmf, 0);
}

static const struct vm_operations_struct vfio_cxl_mem_vm_ops = {
	.fault = vfio_cxl_mem_fault,
#ifdef CONFIG_ARCH_SUPPORTS_HUGE_PFNMAP
	.huge_fault = vfio_cxl_mem_huge_fault,
#endif
};

static int vfio_cxl_mem_mmap(struct vfio_pci_core_device *vdev,
			     struct vfio_pci_region *region,
			     struct vm_area_struct *vma)
{
	unsigned long mask = (1U << (VFIO_PCI_OFFSET_SHIFT - PAGE_SHIFT)) - 1;
	u64 req_start = (vma->vm_pgoff & mask) << PAGE_SHIFT;
	u64 req_len = vma->vm_end - vma->vm_start;

	if (req_start + req_len > region->size)
		return -EINVAL;

	/*
	 * CXL.mem is coherent memory, so leave the mapping write-back cacheable;
	 * a device or non-cached mapping would break the coherence the guest and
	 * KVM depend on. The host physical range is claimed exclusively at bind,
	 * so no conflicting cacheable alias remains.
	 */
	vm_flags_set(vma, VM_IO | VM_PFNMAP | VM_DONTEXPAND | VM_DONTDUMP);
	vma->vm_ops = &vfio_cxl_mem_vm_ops;
	vma->vm_private_data = vdev;

	return 0;
}

static ssize_t vfio_cxl_mem_rw(struct vfio_pci_core_device *vdev,
			       char __user *buf, size_t count, loff_t *ppos,
			       bool iswrite)
{
	/*
	 * The HDM region advertises READ and WRITE so a VMM can derive an
	 * accessible mmap protection for it, but fd read/write is not supported.
	 * The only host-side way to reach the range for a copy is a kernel
	 * mapping of the CXL.mem host physical address (memremap, which reuses
	 * the linear map for this RAM-backed range), and a CPU access through
	 * that mapping aborts on the fabric as a fatal host SError, unlike the
	 * guest-facing mmap fault path which maps the pfn directly. Reject the
	 * transfer rather than fault the host; a consumer mmaps the region and
	 * accesses it that way.
	 */
	return -EIO;
}

/*
 * The CXL regions carry no per-region state (region->data is the shared,
 * devm-managed vfio_cxl_state), so releasing a region is a no-op. The hook is
 * still required: vfio_pci_core_disable() calls region->ops->release() for
 * every region without a NULL check.
 */
static void vfio_cxl_region_release(struct vfio_pci_core_device *vdev,
				    struct vfio_pci_region *region)
{
}

static const struct vfio_pci_regops vfio_cxl_mem_regops = {
	.rw = vfio_cxl_mem_rw,
	.mmap = vfio_cxl_mem_mmap,
	.release = vfio_cxl_region_release,
};

/*
 * Map a poisoned HDM-region pfn back to the file offset of each user mapping so
 * memory_failure() can unmap it and signal the fd holder. The region is a
 * single linear range at hpa_range.start; recover the per-vma file offset the
 * same way the fault handler derived the pfn.
 */
static int vfio_cxl_pfn_to_vma_pgoff(struct vm_area_struct *vma,
				     unsigned long pfn, pgoff_t *pgoff)
{
	struct vfio_pci_core_device *vdev;
	struct vfio_cxl_state *cxl;
	pgoff_t vma_off, pfn_off;
	unsigned long start_pfn;

	if (vma->vm_ops != &vfio_cxl_mem_vm_ops)
		return -ENOENT;

	vdev = vma->vm_private_data;
	cxl = vdev->cxl;

	start_pfn = PHYS_PFN(cxl->hpa_range.start);
	if (pfn < start_pfn ||
	    pfn >= start_pfn + (range_len(&cxl->hpa_range) >> PAGE_SHIFT))
		return -EFAULT;

	pfn_off = pfn - start_pfn;
	vma_off = vma->vm_pgoff &
		  ((1UL << (VFIO_PCI_OFFSET_SHIFT - PAGE_SHIFT)) - 1);
	/* Skip VMAs that do not map the pfn, e.g. a partial mmap of the region. */
	if (pfn_off < vma_off || pfn_off - vma_off >= vma_pages(vma))
		return -EFAULT;

	*pgoff = vma->vm_pgoff + (pfn_off - vma_off);
	return 0;
}

/*
 * The HDM region is struct-page-less device memory, so a memory error on it
 * cannot be routed through the normal page path. Register the range with
 * memory_failure() so such an error is contained to unmapping the range and a
 * SIGBUS to the fd holder instead of escalating to a host SError.
 */
static int vfio_cxl_register_pfn_space(struct vfio_pci_core_device *vdev)
{
	struct vfio_cxl_state *cxl = vdev->cxl;
	unsigned long start_pfn = PHYS_PFN(cxl->hpa_range.start);

	cxl->dpa_pfn_space.node.start = start_pfn;
	cxl->dpa_pfn_space.node.last =
		start_pfn + (range_len(&cxl->hpa_range) >> PAGE_SHIFT) - 1;
	cxl->dpa_pfn_space.mapping = vdev->vdev.inode->i_mapping;
	cxl->dpa_pfn_space.pfn_to_vma_pgoff = vfio_cxl_pfn_to_vma_pgoff;

	return register_pfn_address_space(&cxl->dpa_pfn_space);
}

static ssize_t vfio_cxl_comp_rw(struct vfio_pci_core_device *vdev,
				char __user *buf, size_t count, loff_t *ppos,
				bool iswrite)
{
	struct vfio_cxl_state *cxl = vdev->cxl;
	loff_t pos = *ppos & VFIO_PCI_OFFSET_MASK;

	/*
	 * The guest programs a GPA into this decoder and the host resolves the
	 * HPA, so the guest never drives the physical decoder. Reads come from
	 * the open-time snapshot; write emulation lands in a later change.
	 */
	if (iswrite)
		return -EINVAL;

	if (pos >= cxl->hdm_len)
		return -EINVAL;

	count = min_t(size_t, count, cxl->hdm_len - pos);
	/*
	 * The shadow mirrors the physical decoder, so BASE_LOW/HIGH carry the
	 * host HPA. That is visible only to the trusted VMM holding the fd; the
	 * VMM virtualizes the base so the guest sees its own GPA and never the
	 * host address.
	 */
	if (copy_to_user(buf, (u8 *)cxl->hdm_shadow + pos, count))
		return -EFAULT;

	*ppos += count;
	return count;
}

static const struct vfio_pci_regops vfio_cxl_comp_regops = {
	.rw = vfio_cxl_comp_rw,
	.release = vfio_cxl_region_release,
};

static void vfio_cxl_release_hpa(void *data)
{
	struct vfio_cxl_state *cxl = data;

	release_mem_region(cxl->hpa_range.start, range_len(&cxl->hpa_range));
}

static int vfio_cxl_init_device(struct vfio_pci_core_device *vdev)
{
	struct pci_dev *pdev = vdev->pdev;
	struct vfio_cxl_state *cxl;
	struct cxl_memdev *cxlmd;
	u64 hdm_size, serial;
	u16 dvsec;
	int ret;

	/* pdev->hdm is populated at PCI enumeration; defer until it is. */
	if (!pdev->hdm)
		return -EPROBE_DEFER;

	/* The guest drives one virtual decoder; multiple are unsupported. */
	if (pdev->hdm->decoder_count != 1)
		return -EOPNOTSUPP;

	hdm_size = range_len(&pdev->hdm->settings[0].hpa_range);
	if (!hdm_size)
		return -ENXIO;

	/* Interleaved decoders are unsupported. */
	if (pdev->hdm->settings[0].interleave_ways != 1)
		return -EOPNOTSUPP;

	/*
	 * The guest drives resets through the CXL Device DVSEC and polls the
	 * shadow for completion. If the host cannot service a function-scoped
	 * CXL reset (no reset DVSEC, a multifunction device, or no HDM reset
	 * support), that guest request could never complete, so refuse the
	 * device rather than advertise a reset the guest would poll on forever.
	 */
	if (!cxl_reset_capable(pdev)) {
		pci_err(pdev, "vfio-cxl: Unsupported device: host cannot service a CXL reset request\n");
		return -EOPNOTSUPP;
	}

	dvsec = pci_find_dvsec_capability(pdev, PCI_VENDOR_ID_CXL,
					  PCI_DVSEC_CXL_DEVICE);
	serial = pci_get_dsn(pdev);

	cxl = devm_cxl_dev_state_create(&pdev->dev, CXL_DEVTYPE_DEVMEM, serial,
					dvsec, struct vfio_cxl_state, cxlds,
					false);
	if (!cxl)
		return -ENOMEM;

	cxl->dvsec = dvsec;

	/*
	 * vfio-pci requests the whole component BAR when the guest opens the
	 * device. Declare the BAR owned so the CXL core maps the HDM/RAS
	 * sub-blocks without claiming them and that request does not collide.
	 */
	ret = cxl_pci_setup_regs(pdev, CXL_REGLOC_RBI_COMPONENT,
				 &cxl->cxlds.reg_map, true);
	if (ret)
		return ret;

	/*
	 * Map the HDM decoder registers to sample their programming at open.
	 * The block location comes from the enumeration cache in pdev->hdm, so
	 * this does not reach into the CXL core register map. vfio-pci owns the
	 * BAR, so map without claiming the sub-block.
	 */
	cxl->hdm_regs = devm_ioremap(&pdev->dev,
				     pci_resource_start(pdev, pdev->hdm->hdm_bar) +
				     pdev->hdm->hdm_offset, pdev->hdm->hdm_size);
	if (!cxl->hdm_regs)
		return -ENOMEM;

	cxl->hdm_len = pdev->hdm->hdm_size;

	ret = cxl_set_capacity(&cxl->cxlds, hdm_size);
	if (ret)
		return ret;

	cxlmd = devm_cxl_probe_mem(&cxl->cxlds, &cxl->hpa_range);
	if (IS_ERR(cxlmd))
		return PTR_ERR(cxlmd);

	/*
	 * Own the resolved host physical range outright, and exclusively: mark
	 * it IORESOURCE_EXCLUSIVE so /dev/mem cannot map a conflicting alias even
	 * on an IO_STRICT_DEVMEM=n kernel. Firmware that left it as System RAM
	 * would otherwise keep a cacheable alias that faults the host once the
	 * guest maps the range write-back. There is no devm form of the exclusive
	 * request, so pair it with a devm release action.
	 */
	if (!request_mem_region_exclusive(cxl->hpa_range.start,
					  range_len(&cxl->hpa_range),
					  "vfio-cxl-hdm"))
		return -EBUSY;
	ret = devm_add_action_or_reset(&pdev->dev, vfio_cxl_release_hpa, cxl);
	if (ret)
		return ret;

	cxl->cxlmd = cxlmd;
	vdev->cxl = cxl;

	/*
	 * The VFIO regions and the poison-containment pfn space are set up in
	 * open_device(): vfio_pci_core_disable() tears down all dynamic regions on
	 * close, so they must be created per open rather than once at bind.
	 */
	return 0;
}

static void vfio_cxl_release_device(struct vfio_pci_core_device *vdev)
{
	vdev->cxl = NULL;
}

static int vfio_cxl_open_device(struct vfio_pci_core_device *vdev)
{
	struct vfio_cxl_state *cxl = vdev->cxl;
	void __iomem *hdm = cxl->hdm_regs;
	struct pci_dev *pdev = vdev->pdev;
	__le32 *hdm_shadow;
	u32 hdr, *shadow;
	int i, dwords, ret;

	/*
	 * Sample the DVSEC body now rather than at bind: a low-power
	 * transition could have changed it since the device was bound.
	 */
	pci_read_config_dword(pdev, cxl->dvsec + PCI_DVSEC_HEADER1, &hdr);
	cxl->dvsec_len = PCI_DVSEC_HEADER1_LEN(hdr);
	dwords = cxl->dvsec_len / sizeof(u32);

	shadow = kcalloc(dwords, sizeof(u32), GFP_KERNEL);
	if (!shadow)
		return -ENOMEM;

	for (i = 0; i < dwords; i++)
		pci_read_config_dword(pdev, cxl->dvsec + i * sizeof(u32),
				      &shadow[i]);

	cxl->dvsec_shadow = shadow;

	dwords = cxl->hdm_len / sizeof(u32);
	hdm_shadow = kcalloc(dwords, sizeof(__le32), GFP_KERNEL);
	if (!hdm_shadow) {
		kfree(shadow);
		cxl->dvsec_shadow = NULL;
		return -ENOMEM;
	}

	for (i = 0; i < dwords; i++)
		hdm_shadow[i] = cpu_to_le32(readl(hdm + i * sizeof(u32)));

	cxl->hdm_shadow = hdm_shadow;

	/*
	 * vfio_pci_core_disable() frees all dynamic regions on close, so register
	 * them here (per open) rather than at bind. A failed first-open never
	 * reaches close_device(), so unwind on error.
	 *
	 * Advertise READ and WRITE alongside MMAP: a VMM derives the mmap
	 * protection from these flags, so without them the HDM memory is mapped
	 * PROT_NONE and a guest access faults (KVM cannot back the mapping). The
	 * flags describe the mmap protection only; fd read/write returns -EIO,
	 * because a host CPU read through a kernel mapping of the coherent
	 * CXL.mem range aborts on the fabric (see vfio_cxl_mem_rw()).
	 */
	ret = vfio_pci_core_register_dev_region(vdev, VFIO_REGION_TYPE_CXL,
						VFIO_REGION_SUBTYPE_CXL_MEM,
						&vfio_cxl_mem_regops,
						range_len(&cxl->hpa_range),
						VFIO_REGION_INFO_FLAG_READ |
						VFIO_REGION_INFO_FLAG_WRITE |
						VFIO_REGION_INFO_FLAG_MMAP, cxl);
	if (ret)
		goto err_free_shadows;

	ret = vfio_pci_core_register_dev_region(vdev, VFIO_REGION_TYPE_CXL,
						VFIO_REGION_SUBTYPE_CXL_COMP_REGS,
						&vfio_cxl_comp_regops, cxl->hdm_len,
						VFIO_REGION_INFO_FLAG_READ, cxl);
	if (ret)
		goto err_unregister_hdm;

	/*
	 * The HDM region is advertised mmap-able, so a fd holder can fault its
	 * struct-page-less device memory in from the host CPU. Register it with
	 * memory_failure() to contain a memory error. -EOPNOTSUPP means
	 * CONFIG_MEMORY_FAILURE is off, so run without containment.
	 */
	ret = vfio_cxl_register_pfn_space(vdev);
	if (ret && ret != -EOPNOTSUPP)
		goto err_unregister_comp;

	/*
	 * The decoder is firmware-committed and the shadow now mirrors it, so
	 * host access to the HDM range is safe. Open the access gate; reset and
	 * power transitions clear it until the decoder is restored.
	 */
	cxl->hdm_valid = true;

	return 0;

err_unregister_comp:
	vfio_pci_core_unregister_dev_region(vdev);
err_unregister_hdm:
	vfio_pci_core_unregister_dev_region(vdev);
err_free_shadows:
	kfree(cxl->hdm_shadow);
	cxl->hdm_shadow = NULL;
	kfree(cxl->dvsec_shadow);
	cxl->dvsec_shadow = NULL;
	return ret;
}

static void vfio_cxl_close_device(struct vfio_pci_core_device *vdev)
{
	struct vfio_cxl_state *cxl = vdev->cxl;

	cxl->hdm_valid = false;
	unregister_pfn_address_space(&cxl->dpa_pfn_space);
	kfree(cxl->hdm_shadow);
	cxl->hdm_shadow = NULL;
	kfree(cxl->dvsec_shadow);
	cxl->dvsec_shadow = NULL;
}

/* Read a 16-bit DVSEC field from the shadow; @off is DVSEC-relative. */
static u16 vfio_cxl_dvsec16(struct vfio_cxl_state *cxl, u32 off)
{
	u32 dw = cxl->dvsec_shadow[off / sizeof(u32)];

	return (dw >> (8 * (off % sizeof(u32)))) & 0xffff;
}

/*
 * Apply the CXL r4.0 8.1.3 write class for the 16-bit DVSEC register at @off.
 * Control is programmable, Status is write-1-to-clear, and Capability, Lock and
 * the Range registers stay fixed at their firmware snapshot.
 */
static u16 vfio_cxl_dvsec_field(u32 off, u16 old, u16 wval, u16 wmask)
{
	switch (off) {
	case PCI_DVSEC_CXL_CTRL:
		/*
		 * CXL.mem stays enabled for as long as the guest owns the device.
		 * The HDM decoder maps the guest window to device memory, so a
		 * store to it while CXL.mem is disabled completes on the device as
		 * an error that the host fabric reports as an SError, which is
		 * fatal. The spec does not pin down accesses to a decoder whose
		 * CXL.mem is off and many hosts SError, so ignore a guest request
		 * to clear the enable and keep the bit set.
		 */
		return ((old & ~wmask) | (wval & wmask)) | PCI_DVSEC_CXL_MEM_ENABLE;
	case PCI_DVSEC_CXL_CTRL2:
		return (old & ~wmask) | (wval & wmask);
	case PCI_DVSEC_CXL_STATUS:
	case PCI_DVSEC_CXL_STATUS2:
		return old & ~(wval & wmask);
	default:
		return old;
	}
}

/* Config accesses never cross a dword, so a single shadow entry covers them. */
static int vfio_cxl_config_read(struct vfio_pci_core_device *vdev, int pos,
				int count, __le32 *val)
{
	struct vfio_cxl_state *cxl = vdev->cxl;
	int boff = (pos - cxl->dvsec) % sizeof(u32);
	__le32 dword;

	if (pos < cxl->dvsec || pos >= cxl->dvsec + cxl->dvsec_len)
		return -ENODEV;

	dword = cpu_to_le32(cxl->dvsec_shadow[(pos - cxl->dvsec) / sizeof(u32)]);
	memcpy(val, (u8 *)&dword + boff, count);

	return count;
}

static int vfio_cxl_config_write(struct vfio_pci_core_device *vdev, int pos,
				 int count, __le32 val)
{
	struct vfio_cxl_state *cxl = vdev->cxl;
	int idx = (pos - cxl->dvsec) / sizeof(u32);
	int boff = (pos - cxl->dvsec) % sizeof(u32);
	u32 off = idx * sizeof(u32);
	__le32 le_wval = 0, le_wmask = 0;
	u32 old, wval, wmask;
	u16 lo, hi;

	if (pos < cxl->dvsec || pos >= cxl->dvsec + cxl->dvsec_len)
		return -ENODEV;

	/*
	 * Place the guest bytes and a matching byte mask at the write offset,
	 * then let the per-field class decide what actually lands in the shadow.
	 * The hardware is never touched.
	 */
	memcpy((u8 *)&le_wval + boff, &val, count);
	memset((u8 *)&le_wmask + boff, 0xff, count);
	old = cxl->dvsec_shadow[idx];
	wval = le32_to_cpu(le_wval);
	wmask = le32_to_cpu(le_wmask);

	lo = vfio_cxl_dvsec_field(off, old, wval, wmask);
	hi = vfio_cxl_dvsec_field(off + 2, old >> 16, wval >> 16, wmask >> 16);
	cxl->dvsec_shadow[idx] = lo | ((u32)hi << 16);

	return count;
}

static const struct vfio_cxl_ops vfio_cxl_ops = {
	.init_device	= vfio_cxl_init_device,
	.release_device	= vfio_cxl_release_device,
	.open_device	= vfio_cxl_open_device,
	.close_device	= vfio_cxl_close_device,
	.config_read	= vfio_cxl_config_read,
	.config_write	= vfio_cxl_config_write,
	.owner		= THIS_MODULE,
};

static int __init vfio_cxl_init(void)
{
	return vfio_pci_core_register_cxl_ops(&vfio_cxl_ops);
}

static void __exit vfio_cxl_exit(void)
{
	vfio_pci_core_unregister_cxl_ops(&vfio_cxl_ops);
}

module_init(vfio_cxl_init);
module_exit(vfio_cxl_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("VFIO support for CXL Type-2 devices");
MODULE_ALIAS("vfio-cxl");
MODULE_IMPORT_NS("CXL");
