.. SPDX-License-Identifier: GPL-2.0

=======================================
VFIO-PCI: CXL Type-2 device passthrough
=======================================

Overview
========

A CXL Type-2 device is an accelerator (for example a GPU) that exposes
host-managed device memory through an HDM decoder. vfio-pci alone does
not expose the HDM decoder registers or the CXL Device DVSEC, and it
does not place device memory at a guest-chosen address.

The optional ``vfio-cxl`` module provides that. It is a provider for
vfio-pci-core. vfio-pci-core does not implement CXL registers; it loads
``vfio-cxl`` when it binds a CXL device.

Address model
=============

Device memory uses three address spaces: DPA (device), HPA (host
physical), and GPA (guest physical). The host kernel assigns the device
memory a host physical range before the guest sees the device. The guest
only chooses where that memory appears in its own physical address
space, by programming a virtual endpoint HDM decoder. The guest does not
reprogram the physical decoder.

The kernel holds the HPA and does not see the GPA. The guest programs a
GPA and does not see the HPA. The VMM holds the device fd, reads the
committed base from the decoder-register shadow (the trapped component
region described below), and maps the HPA-backed region at the GPA the
guest committed. The base the guest reads back is the GPA, not the HPA.

Driver model
============

There is no separate PCI driver. vfio-pci binds the device. During bind,
vfio-pci-core detects a CXL device (``pcie_is_cxl()``), loads
``vfio-cxl`` with ``request_module()``, and calls the registered
``struct vfio_cxl_ops``. The module reference is pinned for the bind
lifetime so ``vfio-cxl`` cannot unload while a device is bound.

A non-CXL device, or a CXL device whose CXL setup fails, uses the
ordinary vfio-pci paths.

Regions
=======

``vfio-cxl`` adds two regions under ``VFIO_REGION_TYPE_CXL``:

``VFIO_REGION_SUBTYPE_CXL_MEM``
    The HDM region, backed by the fixed host physical range. It can be
    mapped with mmap. The fault handler inserts the host PFNs, including
    2 MB PMDs when the mapping is aligned. The VMM maps this region into
    guest memory at the committed GPA.

``VFIO_REGION_SUBTYPE_CXL_COMP_REGS``
    The trapped HDM decoder registers. Access is read/write only (no
    mmap) and must be dword aligned. A misaligned or out-of-range access
    returns ``-EINVAL``. The kernel serves the registers from a per-open
    shadow and runs the decoder state machine on writes. The region
    includes a ``VFIO_REGION_INFO_CAP_CXL_COMP_REGS`` capability that
    reports the component BAR and the offset of the decoder block within
    it, so the VMM can place the trapped window at the address the guest
    expects.

The rest of the component BAR is a normal vfio-pci BAR.

Guest decoder and commit
========================

The guest programs its endpoint decoder through the trapped region: it
writes a base (a GPA), a size, and then the COMMIT bit. The host has
already resolved the host physical placement, so a commit always reaches
COMMITTED in the shadow. The physical decoder is not written. A decoder
committed with LOCK_ON_COMMIT stays frozen until the device is reset.
The shadow is sampled from hardware at each open, so a reset clears the
frozen state on the next open.

The VMM observes the commit, reads the committed base, and maps the HDM
region at that GPA.

CXL Device DVSEC
================

The kernel virtualizes the CXL Device DVSEC body through the config-space
permission hooks. Reads and writes inside the DVSEC body use a per-open
shadow. A guest write stays in the shadow and does not reach hardware.
Accesses outside the DVSEC body go to the device as usual.

Reset
=====

A guest triggers a CXL reset by writing Initiate_CXL_Reset in the CXL
Device DVSEC. The kernel revokes the HDM mapping, saves and restores
config around the reset, runs the CXL reset, and writes the result into
DVSEC STATUS2 for the polling guest.

Host-side resets (the reset ioctl, an FLR through config space, and a
bus hot reset) revoke the mapping the same way and restore and re-sample
the decoder shadow afterward. Because the kernel re-samples the
firmware-committed decoder, the shadow returns to the committed state
without a new guest commit.

A guest that had decommitted an unlocked decoder therefore issues no new
commit. If the VMM dropped its mapping, it must rescan the decoder after
the DVSEC reset, the D3hot->D0 transition, and an FLR. No new commit
will arrive. A committed, locked decoder cannot be decommitted, so this
only applies to the unlocked case.

UAPI
====

``VFIO_DEVICE_FLAGS_CXL``
    Set in ``VFIO_DEVICE_GET_INFO`` flags for a CXL Type-2 device.

``VFIO_REGION_TYPE_CXL`` with ``VFIO_REGION_SUBTYPE_CXL_MEM`` /
``VFIO_REGION_SUBTYPE_CXL_COMP_REGS``
    Reported through the region-info ``VFIO_REGION_INFO_CAP_TYPE``
    capability. Userspace finds each region by scanning for the type and
    subtype.

``VFIO_REGION_INFO_CAP_CXL_COMP_REGS``
    On the component-register region, reports the component BAR index
    and the decoder-block offset within it.

The HDM decoder register layout is available to a VMM without a private
kernel header via ``uapi/cxl/cxl_regs.h``.

Scope
=====

This support covers a single, non-interleaved endpoint decoder on a
directly attached device. Multi-decoder devices, interleave, and
switch-attached topologies are not supported. The interfaces are
structured so those cases can be added later without changing the UAPI
described here.

A selftest, ``tools/testing/selftests/vfio/vfio_cxl_type2_test.c``,
exercises the interfaces above on a bound device.
