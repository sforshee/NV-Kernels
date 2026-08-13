// SPDX-License-Identifier: GPL-2.0-only
/*
 * VFIO support for CXL Type-2 devices.
 *
 * Copyright (c) 2026 NVIDIA Corporation & Affiliates
 */

#include <linux/module.h>
#include <linux/pci.h>
#include <linux/range.h>
#include <linux/slab.h>
#include <linux/vfio_pci_core.h>
#include <cxl/cxl.h>
#include <cxl/pci.h>

/**
 * struct vfio_cxl_state - per-device state for a vfio-cxl device
 * @cxlds: CXL device state; kept first for devm_cxl_dev_state_create()
 * @cxlmd: memory device joined to the CXL topology at bind
 * @hpa_range: host physical range of the HDM region
 * @dvsec: CXL device DVSEC config-space offset
 * @dvsec_len: length of the DVSEC body
 * @dvsec_shadow: guest view of the CXL DVSEC body, sampled at open
 */
struct vfio_cxl_state {
	struct cxl_dev_state cxlds;
	struct cxl_memdev *cxlmd;
	struct range hpa_range;
	u16 dvsec;
	u32 dvsec_len;
	u32 *dvsec_shadow;
};

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

	ret = cxl_set_capacity(&cxl->cxlds, hdm_size);
	if (ret)
		return ret;

	cxlmd = devm_cxl_probe_mem(&cxl->cxlds, &cxl->hpa_range);
	if (IS_ERR(cxlmd))
		return PTR_ERR(cxlmd);

	cxl->cxlmd = cxlmd;
	vdev->cxl = cxl;

	return 0;
}

static void vfio_cxl_release_device(struct vfio_pci_core_device *vdev)
{
	vdev->cxl = NULL;
}

static int vfio_cxl_open_device(struct vfio_pci_core_device *vdev)
{
	struct vfio_cxl_state *cxl = vdev->cxl;
	struct pci_dev *pdev = vdev->pdev;
	u32 hdr, *shadow;
	int i, dwords;

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

	return 0;
}

static void vfio_cxl_close_device(struct vfio_pci_core_device *vdev)
{
	struct vfio_cxl_state *cxl = vdev->cxl;

	kfree(cxl->dvsec_shadow);
	cxl->dvsec_shadow = NULL;
}

static const struct vfio_cxl_ops vfio_cxl_ops = {
	.init_device	= vfio_cxl_init_device,
	.release_device	= vfio_cxl_release_device,
	.open_device	= vfio_cxl_open_device,
	.close_device	= vfio_cxl_close_device,
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
