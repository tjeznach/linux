// SPDX-License-Identifier: GPL-2.0-only
/*
 * IOMMU API for RISC-V architected Ziommu implementations.
 *
 * Copyright (C) 2022-2023 Rivos Inc.
 *
 * Author: Tomasz Jeznach <tjeznach@rivosinc.com>
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/bitfield.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/compiler.h>
#include <linux/pci.h>
#include <linux/pci-ats.h>
#include <linux/init.h>
#include <linux/completion.h>
#include <linux/uaccess.h>
#include <linux/iommu.h>
#include <linux/platform_device.h>
#include <linux/dma-map-ops.h>
#include <asm/page.h>

#include "../dma-iommu.h"
#include "../iommu-sva.h"
#include "iommu.h"

#include <asm/csr.h>
#include <asm/delay.h>

/* Global IOMMU params. */
static int ddt_mode = RIO_DDTP_MODE_3LVL;
module_param(ddt_mode, int, 0644);
MODULE_PARM_DESC(ddt_mode, "Device Directory Table mode.");

static int cmdq_length = 1024;
module_param(cmdq_length, int, 0644);
MODULE_PARM_DESC(cmdq_length, "Command queue length.");

static int fltq_length = 1024;
module_param(fltq_length, int, 0644);
MODULE_PARM_DESC(fltq_length, "Fault queue length.");

static int priq_length = 1024;
module_param(priq_length, int, 0644);
MODULE_PARM_DESC(priq_length, "Page request interface queue length.");

/* TODO: Enable MSI remapping */
#define RISCV_IMSIC_BASE	0x28000000

/* 1 second */
#define RISCV_IOMMU_TIMEOUT		riscv_timebase

/* RISC-V IOMMU PPN <> PHYS address conversions, PHYS <=> PPN[53:10] */
#define phys_to_ppn(va)  (((va) >> 2) & (((1ULL << 44) - 1) << 10))
#define ppn_to_phys(pn)	 (((pn) << 2) & (((1ULL << 44) - 1) << 12))

#define iommu_domain_to_riscv(iommu_domain) \
    container_of(iommu_domain, struct riscv_iommu_domain, domain)

#define iommu_device_to_riscv(iommu_device) \
    container_of(iommu_device, struct riscv_iommu, iommu)

static void __cmd_iodir_pasid(struct riscv_iommu_command *cmd, unsigned devid,
			      unsigned pasid)
{
	cmd->request = FIELD_PREP(RIO_CMD_OP, RIO_CMD_IODIR_PDT) |
	    FIELD_PREP(RIO_IODIR_DID, devid) | RIO_IODIR_DV |
	    FIELD_PREP(RIO_IODIR_PID, pasid);
	cmd->address = 0;
}

static void __cmd_inval_vma(struct riscv_iommu_command *cmd)
{
	cmd->request = FIELD_PREP(RIO_CMD_OP, RIO_CMD_IOTINVAL_VMA);
	cmd->address = 0;
}

static void __cmd_inval_set_addr(struct riscv_iommu_command *cmd, u64 addr)
{
	cmd->request |= RIO_IOTINVAL_AV;
	cmd->address = addr;
}

static void __cmd_inval_set_pscid(struct riscv_iommu_command *cmd,
				  unsigned pscid)
{
	cmd->request |= FIELD_PREP(RIO_IOTINVAL_PSCID, pscid) |
	    RIO_IOTINVAL_PSCV;
}

static void __cmd_inval_set_gscid(struct riscv_iommu_command *cmd,
				  unsigned gscid)
{
	cmd->request |= FIELD_PREP(RIO_IOTINVAL_GSCID, gscid) | RIO_IOTINVAL_GV;
}

static void __cmd_iofence(struct riscv_iommu_command *cmd)
{
	cmd->request = FIELD_PREP(RIO_CMD_OP, RIO_CMD_IOFENCE_C);
	cmd->address = 0;
}

static void __cmd_iofence_set_av(struct riscv_iommu_command *cmd, u64 addr,
				 u32 data)
{
	cmd->request = FIELD_PREP(RIO_CMD_OP, RIO_CMD_IOFENCE_C) |
	    FIELD_PREP(RIO_IOFENCE_DATA, data) | RIO_IOFENCE_AV;
	cmd->address = addr;
}

static void __cmd_ats_pgr(struct riscv_iommu_command *cmd)
{
	cmd->request = FIELD_PREP(RIO_CMD_OP, RIO_CMD_ATS_PRGR);
	cmd->address = 0;
}

/// TODO: transition to DC management calls, teardown
/* Lookup or initialize device directory info structure. */
static struct riscv_iommu_dc *riscv_iommu_get_dc(struct riscv_iommu_device
						 *iommu, unsigned devid)
{
	const bool dc32 = !(iommu->cap & RIO_CAP_MSI_FLAT);
	unsigned depth = iommu->ddt_mode - RIO_DDTP_MODE_1LVL;
	u64 *ddt;

	if (iommu->ddt_mode == RIO_DDTP_MODE_OFF ||
	    iommu->ddt_mode == RIO_DDTP_MODE_BARE)
		return NULL;

	/* Check supported device id range. */
	if (devid >= (1 << (depth * 9 + 6 + (dc32 && depth != 2))))
		return NULL;

	for (ddt = (u64 *) iommu->ddtp; depth-- > 0;) {
		const int split = depth * 9 + 6 + dc32;
		ddt += (devid >> split) & 0x1FF;

		if (*ddt & RIO_DDTE_VALID) {
			ddt = __va(ppn_to_phys(*ddt));
		} else {
			/* Allocate next device directory level. */
			// TODO: DDTP page walk management
			unsigned long ddtp = get_zeroed_page(GFP_KERNEL);
			if (!ddtp)
				return NULL;
			*ddt = phys_to_ppn(__pa(ddtp)) | RIO_DDTE_VALID;
			ddt = (u64 *) ddtp;
		}
	}

	ddt += (devid & ((64 << dc32) - 1)) << (3 - dc32);
	return (struct riscv_iommu_dc *)ddt;
}

/* TODO: Convert into lock-less MPSC implementation. */
static bool riscv_iommu_post_sync(struct riscv_iommu_device *iommu,
				  struct riscv_iommu_command *cmd, bool sync)
{
	u32 head, tail, next, last;
	unsigned long flags;


	spin_lock_irqsave(&iommu->cq_lock, flags);
	head = riscv_iommu_readl(iommu, RIO_REG_CQH) & (iommu->cmdq.cnt - 1);
	tail = riscv_iommu_readl(iommu, RIO_REG_CQT) & (iommu->cmdq.cnt - 1);
	last = iommu->cmdq.lui;
	if (tail != last) {
		spin_unlock_irqrestore(&iommu->cq_lock, flags);
		/* TRY AGAIN */
		dev_err(iommu->dev, "IOMMU CQT: %x != %x (1st)\n", last, tail);
		spin_lock_irqsave(&iommu->cq_lock, flags);
		tail = riscv_iommu_readl(iommu, RIO_REG_CQT) & (iommu->cmdq.cnt - 1);
		last = iommu->cmdq.lui;
		if (tail != last) {
			spin_unlock_irqrestore(&iommu->cq_lock, flags);
			dev_err(iommu->dev, "IOMMU CQT: %x != %x (2nd)\n", last, tail);
			spin_lock_irqsave(&iommu->cq_lock, flags);
		}
	}

	next = (last + 1) & (iommu->cmdq.cnt - 1);
	if (next != head) {
		struct riscv_iommu_command *ptr = iommu->cmdq.base;
		ptr[last] = *cmd;
		wmb();
		riscv_iommu_writel(iommu, RIO_REG_CQT, next);
		iommu->cmdq.lui = next;
	}

	spin_unlock_irqrestore(&iommu->cq_lock, flags);

	if (sync && head != next) {
		cycles_t start_time = get_cycles();
		while (1) {
			last = riscv_iommu_readl(iommu, RIO_REG_CQH) & (iommu->cmdq.cnt - 1);
			if (head < next && last >= next)
				break;
			if (head > next && last < head && last >= next)
				break;
			if (RISCV_IOMMU_TIMEOUT < (get_cycles() - start_time)) {
				dev_err(iommu->dev, "IOFENCE TIMEOUT\n");
				return false;
			}
			cpu_relax();
		}
	}

	return next != head;
}

static bool riscv_iommu_post(struct riscv_iommu_device *iommu,
			     struct riscv_iommu_command *cmd)
{
	return riscv_iommu_post_sync(iommu, cmd, false);
}

static bool riscv_iommu_iodir_inv_all(struct riscv_iommu_device *iommu)
{
	struct riscv_iommu_command cmd = {
		.request = FIELD_PREP(RIO_CMD_OP, RIO_CMD_IODIR_DDT),
		.address = 0,
	};
	return riscv_iommu_post(iommu, &cmd);
}

static bool riscv_iommu_iodir_inv_devid(struct riscv_iommu_device *iommu,
					unsigned devid)
{
	struct riscv_iommu_command cmd = {
		.request = FIELD_PREP(RIO_CMD_OP, RIO_CMD_IODIR_DDT) |
		    FIELD_PREP(RIO_IODIR_DID, devid) | RIO_IODIR_DV,
		.address = 0,
	};
	return riscv_iommu_post(iommu, &cmd);
}

static bool riscv_iommu_iodir_inv_pasid(struct riscv_iommu_device *iommu,
					unsigned devid, unsigned pasid)
{
	struct riscv_iommu_command cmd;
	__cmd_iodir_pasid(&cmd, devid, pasid);
	return riscv_iommu_post(iommu, &cmd);
}

static bool riscv_iommu_iofence_sync(struct riscv_iommu_device *iommu)
{
	struct riscv_iommu_command cmd;
	__cmd_iofence(&cmd);
	return riscv_iommu_post_sync(iommu, &cmd, true);
}

/* TODO: move to local (private) allocators for VMID / PSCID
   allocate GSCID unique PSCID
 */
static ioasid_t iommu_sva_alloc_pscid(void)
{
	/* TODO: Provide anonymous pasid value */
	struct mm_struct mm = {
		.pasid = INVALID_IOASID,
	};

	if (iommu_sva_alloc_pasid(&mm, 1, (1 << 20) - 1))
		return INVALID_IOASID;

	return mm.pasid;
}

/* mark domain as second-stage translation */
static int riscv_iommu_enable_nesting(struct iommu_domain *iommu_domain)
{
	struct riscv_iommu_domain *domain = iommu_domain_to_riscv(iommu_domain);

	mutex_lock(&domain->lock);
	if (list_empty(&domain->endpoints))
		domain->g_stage = true;
	mutex_unlock(&domain->lock);

	return domain->g_stage ? 0 : -EBUSY;
}

static void riscv_iommu_domain_free(struct iommu_domain *iommu_domain)
{
	struct riscv_iommu_domain *domain = iommu_domain_to_riscv(iommu_domain);

	if (domain->mn.ops && iommu_domain->mm)
		mmu_notifier_unregister(&domain->mn, iommu_domain->mm);

	if (domain->pgtbl.cookie)
		free_io_pgtable_ops(&domain->pgtbl.ops);

	if (domain->pgd_root)
		free_pages((unsigned long)domain->pgd_root,
			   domain->g_stage ? 2 : 0);

	kfree(domain);
}

static int riscv_iommu_domain_finalize(struct riscv_iommu_domain *domain,
				       struct riscv_iommu_device *iommu)
{
	struct iommu_domain_geometry *geometry;

	if (domain->iommu && domain->iommu != iommu)
		return -EINVAL;
	else if (domain->iommu)
		return 0;	/* already initialized */

	geometry = &domain->domain.geometry;
	geometry->aperture_start = 0;
	geometry->aperture_end = DMA_BIT_MASK(VA_BITS);
	geometry->force_aperture = true;

	domain->iommu = iommu;
	domain->id = iommu_sva_alloc_pscid();
	if (domain->domain.type == IOMMU_DOMAIN_IDENTITY) {
		domain->mode = RIO_ATP_MODE_BARE;
		return 0;
	}

	domain->mode = satp_mode >> 60;
	domain->pgd_root = (pgd_t *) __get_free_pages(GFP_KERNEL | __GFP_ZERO,
						      domain->g_stage ? 2 : 0);

	if (!domain->pgd_root)
		return -ENOMEM;

	if (!alloc_io_pgtable_ops(RISCV_IOMMU, &domain->pgtbl.cfg, domain))
		return -ENOMEM;

	return 0;
}

static u64 riscv_iommu_domain_atp(struct riscv_iommu_domain *domain)
{
	u64 atp = FIELD_PREP(RIO_ATP_MODE, domain->mode);
	if (domain->mode != RIO_ATP_MODE_BARE)
		atp |= FIELD_PREP(RIO_ATP_PPN, virt_to_pfn(domain->pgd_root));
	if (domain->g_stage)
		atp |= FIELD_PREP(RIO_ATP_GSCID, domain->id);
	return atp;
}

static void riscv_iommu_disable_pci_ep(struct device *dev)
{
	struct pci_dev *pdev;
	struct riscv_iommu_endpoint *ep = dev_iommu_priv_get(dev);

	if (!dev_is_pci(dev))
		return;

	pdev = to_pci_dev(dev);
	if (pdev->ats_enabled)
		pci_disable_ats(pdev);

	if (pdev->pri_enabled)
		pci_disable_pri(pdev);

	if (pdev->pasid_enabled)
		pci_disable_pasid(pdev);

	ep->pasid_bits = 0;
}

static void riscv_iommu_enable_pci_ep(struct device *dev)
{
	int ret, feat, num;
	struct pci_dev *pdev;
	struct riscv_iommu_endpoint *ep = dev_iommu_priv_get(dev);

	if (!dev_is_pci(dev))
		return;

	pdev = to_pci_dev(dev);

	/* Enable PASID */
	feat = pci_pasid_features(pdev);
	num = pci_max_pasids(pdev);
	ret = pci_enable_pasid(pdev, feat);
	if (ret) {
		dev_warn(dev, "Can't enable PASID (rc: %d) cap: %u\n", ret, pdev->pasid_cap);
		return;
	}

	/* Reset the PRI state of the device */
	ret = pci_reset_pri(pdev);
	if (ret) {
		dev_warn(dev, "Can't reset PRI (rc: %d) en: %d\n", ret, pdev->pri_enabled);
		pci_disable_pasid(pdev);
		return;
	}

	/* Enable PRI */
	ret = pci_enable_pri(pdev, 32);
	if (ret) {
		dev_warn(dev, "Can't enable PRI (rc: %d)\n", ret);
		pci_disable_pasid(pdev);
		return;
	}

	/* Enable ATS */
	ret = pci_enable_ats(pdev, PAGE_SHIFT);
	if (ret) {
		dev_warn(dev, "Can't enable ATS (rc: %d)\n", ret);
		pci_disable_pri(pdev);
		pci_disable_pasid(pdev);
		return;
	}

	ep->sva_supported = 1;
	ep->pri_supported = 1;
	ep->pri_enabled = 1;
	ep->ats_enabled = 1;
	ep->pasid_enabled = 1;

	ep->pasid_feat = feat;
	ep->pasid_bits = ilog2(num);

	dev_info(dev, "PASID/ATS support enabled, %d bits\n", ep->pasid_bits);
}

static void riscv_iommu_detach_dev(struct iommu_domain *iommu_domain,
				   struct device *dev)
{
	struct riscv_iommu_domain *domain = iommu_domain_to_riscv(iommu_domain);
	struct riscv_iommu_endpoint *ep = dev_iommu_priv_get(dev);

	dev_info(dev, "domain detached, Stage: %d\n", domain->g_stage);

	riscv_iommu_disable_pci_ep(dev);
	ep->dc->tc = 0ULL;
	// ep->dc->fsc = cpu_to_le64(virt_to_pfn(ep->iommu->zero) | SATP_MODE);
	ep->dc->gatp = 0ULL;
	wmb();
	riscv_iommu_iodir_inv_devid(ep->iommu, ep->devid);
}

static int riscv_iommu_attach_dev(struct iommu_domain *iommu_domain,
				  struct device *dev)
{
	struct riscv_iommu_domain *domain = iommu_domain_to_riscv(iommu_domain);
	struct riscv_iommu_endpoint *ep = dev_iommu_priv_get(dev);
	struct riscv_iommu_dc *dc;
	struct iommu_resv_region *entry;
	int ret;
	u64 val;
	int i;

	dc = riscv_iommu_get_dc(ep->iommu, ep->devid);
	if (!dc)
		return -ENOMEM;

	// TODO: add endpoint lock ?
	mutex_lock(&domain->lock);

	/* allocate root pages, initialize io-pgtable ops, etc. */
	ret = riscv_iommu_domain_finalize(domain, ep->iommu);
	if (ret < 0) {
		dev_err(dev, "can not finalize domain: %d\n", ret);
		mutex_unlock(&domain->lock);
		return ret;
	}

	dev_info(dev, "domain id %u %s type %d attach\n", domain->id,
		 domain->g_stage ? "G-Stage" : "S-Stage", domain->domain.type);

	/* Update device context:
	   G-Stage : only if domain->g_stage
	   S-Stage : only if domain->s_stage. rely on OS not to update S-Stage while G-Stage valid.
	 */
	if (domain->g_stage) {
		dc->gatp = cpu_to_le64(riscv_iommu_domain_atp(domain));
		// trace S-Stage
		dev_info(dev, "IOMMU first-stage: %llx", dc->fsc);
		dc->fsc = 0ULL;	// pass-through for now.
	} else {
		val = FIELD_PREP(RIO_PCTA_PSCID, domain->id);
		dc->ta = cpu_to_le64(val);
		dc->fsc = cpu_to_le64(riscv_iommu_domain_atp(domain));
		wmb();
		val |= RIO_PCTA_V;
		dc->ta = cpu_to_le64(val);
	}

	/* Initialize MSI remapping */
	if (!(ep->iommu->cap & RIO_CAP_MSI_FLAT))
		goto skip_msiptp;

	/* FIXME: implement remapping device */
	val = get_zeroed_page(GFP_KERNEL);
	if (!val) {
		mutex_unlock(&domain->lock);
		return -ENOMEM;
	}

	domain->msi_root = (struct riscv_iommu_msipte *)val;

	for (i = 0; i < 256; i++) {
		domain->msi_root[i].msipte =
		    pte_val(pfn_pte
			    (phys_to_pfn(RISCV_IMSIC_BASE) + i,
			     __pgprot(_PAGE_WRITE | _PAGE_PRESENT)));
	}

	entry = iommu_alloc_resv_region(RISCV_IMSIC_BASE, PAGE_SIZE * 256, 0,
					IOMMU_RESV_SW_MSI, GFP_KERNEL);
	if (entry) {
		list_add_tail(&entry->list, &ep->regions);
	}

	val = virt_to_pfn(domain->msi_root) |
	    FIELD_PREP(RIO_DCMSI_MODE, RIO_DCMSI_MODE_FLAT);
	dc->msiptp = cpu_to_le64(val);

	/* Single page of MSIPTP, 256 IMSIC files */
	dc->msi_addr_mask = cpu_to_le64(255);
	dc->msi_addr_pattern = cpu_to_le64(RISCV_IMSIC_BASE >> 12);

 skip_msiptp:
	/* Mark device context as valid */
	wmb();
	// EN.ATS should be based on endpoint enable_ats() feature.
	dc->tc = cpu_to_le64(RIO_DCTC_EN_ATS | RIO_DCTC_VALID);
	ep->dc = dc;
	mutex_unlock(&domain->lock);
	riscv_iommu_iodir_inv_devid(ep->iommu, ep->devid);

	// enable PCI capabilities
	riscv_iommu_enable_pci_ep(dev);

	return 0;
}

static void riscv_iommu_mm_invalidate(struct mmu_notifier *mn,
    struct mm_struct *mm, unsigned long start, unsigned long end)
{
	struct riscv_iommu_command cmd;
	struct riscv_iommu_domain *domain = container_of(mn, struct riscv_iommu_domain, mn);
	/* TODO: add ATS.INVAL if needed, cleanup GSCID/PSCID passing, IOVA range flush */
	__cmd_inval_vma(&cmd);
	__cmd_inval_set_gscid(&cmd, 0);
	__cmd_inval_set_pscid(&cmd, domain->pscid);
	riscv_iommu_post(domain->iommu, &cmd);
	riscv_iommu_iofence_sync(domain->iommu);
}


static void riscv_iommu_mm_release(struct mmu_notifier *mn, struct mm_struct *mm)
{
	/* TODO: removed from notifier, cleanup PSCID mapping, flush IOTLB */
}

static const struct mmu_notifier_ops riscv_iommu_mmuops = {
	.release = riscv_iommu_mm_release,
	.invalidate_range = riscv_iommu_mm_invalidate,
};

static int riscv_iommu_set_dev_pasid(struct iommu_domain *iommu_domain,
				     struct device *dev, ioasid_t pasid)
{
	struct riscv_iommu_domain *domain = iommu_domain_to_riscv(iommu_domain);
	struct riscv_iommu_endpoint *ep = dev_iommu_priv_get(dev);
	struct riscv_iommu_dc *dc = ep->dc;
	struct riscv_iommu_pc *pc = ep->pc;
	struct mm_struct *mm;

	if (!iommu_domain || !iommu_domain->mm)
		return -EINVAL;
	if (!ep || !ep->sva_supported)
		return -ENODEV;
	if (!pc)
		pc = (struct riscv_iommu_pc *)get_zeroed_page(GFP_KERNEL);
	if (!pc)
		return -ENOMEM;

	/* register mm notifier */
	mm = iommu_domain->mm;
	domain->pscid = pasid;
	domain->iommu = ep->iommu;
	domain->mn.ops = &riscv_iommu_mmuops;

	if (mmu_notifier_register(&domain->mn, mm))
	    return -ENODEV;

	/* Use PASID for PSCID tag */
	pc[pasid].ta = cpu_to_le64(FIELD_PREP(RIO_PCTA_PSCID, pasid) | RIO_PCTA_V);
	/* TODO: get SXL value for the process, use 32 bit or SATP mode */
	pc[pasid].fsc = cpu_to_le64(virt_to_pfn(mm->pgd) | satp_mode);
	/* update DC with sva->mm */
	if (!(ep->dc->tc & RIO_DCTC_PDTV)) {
		/* migrate to PD, domain mappings moved to PASID:0 */
		pc[0].ta = dc->ta;
		pc[0].fsc = dc->fsc;
		dc->fsc = cpu_to_le64(virt_to_pfn(pc) | FIELD_PREP(RIO_ATP_MODE, RIO_PDTP_MODE_PD8));
		dc->tc = cpu_to_le64(RIO_DCTC_PDTV | RIO_DCTC_EN_ATS | RIO_DCTC_VALID | (1ULL << 32));
		ep->pc = pc;
		wmb();

		/* TODO: transition to PD steps */
		riscv_iommu_iodir_inv_devid(ep->iommu, ep->devid);
	} else {
		wmb();
		riscv_iommu_iodir_inv_pasid(ep->iommu, ep->devid, pasid);
	}

	riscv_iommu_iofence_sync(ep->iommu);

	return 0;
}

static const struct iommu_domain_ops riscv_iommu_domain_ops;

static struct iommu_domain *riscv_iommu_domain_alloc(unsigned type)
{
	struct riscv_iommu_domain *domain;

	if (type != IOMMU_DOMAIN_DMA &&
	    type != IOMMU_DOMAIN_DMA_FQ &&
	    type != IOMMU_DOMAIN_UNMANAGED &&
	    type != IOMMU_DOMAIN_IDENTITY &&
	    type != IOMMU_DOMAIN_BLOCKED &&
	    type != IOMMU_DOMAIN_SVA)
		return NULL;

	domain = kzalloc(sizeof(*domain), GFP_KERNEL);
	if (!domain)
		return NULL;

	/* based on domain type ?? */
	domain->domain.ops = &riscv_iommu_domain_ops;

	mutex_init(&domain->lock);
	INIT_LIST_HEAD(&domain->endpoints);

	return &domain->domain;
}

static int riscv_iommu_map_pages(struct iommu_domain *iommu_domain,
				 unsigned long iova, phys_addr_t phys,
				 size_t pgsize, size_t pgcount, int prot,
				 gfp_t gfp, size_t *mapped)
{
	struct riscv_iommu_domain *domain = iommu_domain_to_riscv(iommu_domain);

	if (domain->domain.type == IOMMU_DOMAIN_BLOCKED)
		return -ENODEV;

	if (domain->domain.type == IOMMU_DOMAIN_IDENTITY) {
		*mapped = pgsize * pgcount;
		return 0;
	}

	if (!domain->pgtbl.ops.map_pages)
		return -ENODEV;

	return domain->pgtbl.ops.map_pages(&domain->pgtbl.ops, iova, phys,
					   pgsize, pgcount, prot, gfp, mapped);
}

static size_t riscv_iommu_unmap_pages(struct iommu_domain *iommu_domain,
				      unsigned long iova, size_t pgsize,
				      size_t pgcount,
				      struct iommu_iotlb_gather *gather)
{
	struct riscv_iommu_domain *domain = iommu_domain_to_riscv(iommu_domain);

	if (domain->domain.type == IOMMU_DOMAIN_IDENTITY)
		return pgsize * pgcount;

	if (!domain->pgtbl.ops.unmap_pages)
		return 0;

	return domain->pgtbl.ops.unmap_pages(&domain->pgtbl.ops, iova, pgsize,
					     pgcount, gather);
}

static phys_addr_t riscv_iommu_iova_to_phys(struct iommu_domain *iommu_domain,
					    dma_addr_t iova)
{
	struct riscv_iommu_domain *domain = iommu_domain_to_riscv(iommu_domain);

	if (domain->domain.type == IOMMU_DOMAIN_IDENTITY)
		return (phys_addr_t) iova;

	if (!domain->pgtbl.ops.iova_to_phys)
		return 0;

	return domain->pgtbl.ops.iova_to_phys(&domain->pgtbl.ops, iova);
}

static void riscv_iommu_flush_iotlb_all(struct iommu_domain *iommu_domain)
{
	struct riscv_iommu_domain *domain = iommu_domain_to_riscv(iommu_domain);
	struct riscv_iommu_command cmd;

	__cmd_inval_vma(&cmd);

	if (!domain->g_stage)
		__cmd_inval_set_pscid(&cmd, domain->id);
	if (domain->g_stage)
		__cmd_inval_set_gscid(&cmd, domain->id);
	else if (domain->nested)
		__cmd_inval_set_gscid(&cmd, domain->nested->id);

	riscv_iommu_post(domain->iommu, &cmd);
	riscv_iommu_iofence_sync(domain->iommu);
}

static void riscv_iommu_iotlb_sync(struct iommu_domain *iommu_domain,
				   struct iommu_iotlb_gather *gather)
{
	riscv_iommu_flush_iotlb_all(iommu_domain);
}

static void riscv_iommu_iotlb_sync_map(struct iommu_domain *iommu_domain,
				       unsigned long iova, size_t size)
{
	riscv_iommu_flush_iotlb_all(iommu_domain);
}

static void riscv_iommu_get_resv_regions(struct device *dev,
					 struct list_head *head)
{
	struct iommu_resv_region *entry, *new_entry;
	struct riscv_iommu_endpoint *ep = dev_iommu_priv_get(dev);

	list_for_each_entry(entry, &ep->regions, list) {
		new_entry = kmemdup(entry, sizeof(*entry), GFP_KERNEL);
		if (new_entry)
			list_add_tail(&new_entry->list, head);
	}

	iommu_dma_get_resv_regions(dev, head);
}

static const struct iommu_ops riscv_iommu_ops;

static struct riscv_iommu_device *riscv_iommu_get_device(struct device *dev)
{
	struct iommu_fwspec *fwspec = dev_iommu_fwspec_get(dev);

	if (!fwspec || fwspec->ops != &riscv_iommu_ops ||
	    !fwspec->iommu_fwnode || !fwspec->iommu_fwnode->dev)
		return NULL;

	return dev_get_drvdata(fwspec->iommu_fwnode->dev);
}

static struct iommu_device *riscv_iommu_probe_device(struct device *dev)
{
	struct riscv_iommu_device *iommu;
	struct riscv_iommu_endpoint *ep, *rb_ep;
	struct rb_node **new_node, *parent_node = NULL;
	struct pci_dev *pdev = dev_is_pci(dev) ? to_pci_dev(dev) : NULL;

	iommu = riscv_iommu_get_device(dev);
	if (!iommu)
		return ERR_PTR(-ENODEV);

	/* TODO: Add support for non-PCI devices. */
	if (!pdev)
		return ERR_PTR(-EINVAL);

	ep = kzalloc(sizeof(*ep), GFP_KERNEL);
	if (!ep)
		return ERR_PTR(-ENOMEM);

	ep->dev = dev;
	ep->iommu = iommu;
	ep->devid = pci_dev_id(pdev);
	ep->domid = pci_domain_nr(pdev->bus);

	INIT_LIST_HEAD(&ep->domains);
	INIT_LIST_HEAD(&ep->regions);
	INIT_LIST_HEAD(&ep->bindings);

	/* insert into IOMMU endpoint mappings */
	mutex_lock(&iommu->eps_mutex);
	new_node = &(iommu->eps.rb_node);
	while (*new_node) {
		rb_ep = rb_entry(*new_node, struct riscv_iommu_endpoint, node);
		parent_node = *new_node;
		if (rb_ep->devid > ep->devid) {
			new_node = &((*new_node)->rb_left);
		} else if (rb_ep->devid < ep->devid) {
			new_node = &((*new_node)->rb_right);
		} else {
			dev_warn(dev, "device %u already in the tree\n", rb_ep->devid);
			break;
		}
	}

	rb_link_node(&ep->node, parent_node, new_node);
	rb_insert_color(&ep->node, &iommu->eps);
	mutex_unlock(&iommu->eps_mutex);

	dev_iommu_priv_set(dev, ep);

	return &iommu->iommu;
}

static void riscv_iommu_probe_finalize(struct device *dev)
{
	set_dma_ops(dev, NULL);
	iommu_setup_dma_ops(dev, 0, U64_MAX);
}

static void riscv_iommu_release_device(struct device *dev)
{
	struct riscv_iommu_endpoint *ep = dev_iommu_priv_get(dev);
	struct riscv_iommu_device *iommu = ep->iommu;

	/* Device must be already removed from protection domain */
	WARN_ON(ep->domain);

	mutex_lock(&iommu->eps_mutex);
	rb_erase(&ep->node, &iommu->eps);
	mutex_unlock(&iommu->eps_mutex);

	set_dma_ops(dev, NULL);
	dev_iommu_priv_set(dev, NULL);
	kfree(ep);
}

static struct iommu_group *riscv_iommu_device_group(struct device *dev)
{
	if (dev_is_pci(dev))
		return pci_device_group(dev);
	return generic_device_group(dev);
}

static int
riscv_iommu_of_xlate(struct device *dev, struct of_phandle_args *args)
{
	return iommu_fwspec_add_ids(dev, args->args, 1);
}

static bool riscv_iommu_capable(struct device *dev, enum iommu_cap cap)
{
	switch (cap) {
	case IOMMU_CAP_CACHE_COHERENCY:
	case IOMMU_CAP_PRE_BOOT_PROTECTION:
		return true;

	default:
		break;
	}

	return false;
}

static int riscv_iommu_enable_sva(struct device *dev)
{
	int ret;
	struct riscv_iommu_endpoint *ep = dev_iommu_priv_get(dev);

	if (!ep || !ep->iommu || !ep->iommu->pq_work)
		return -EINVAL;

	if (!ep->sva_supported)
		return -ENODEV;

	if (!ep->pasid_enabled || !ep->pri_enabled || !ep->ats_enabled)
		return -EINVAL;

	ret = iopf_queue_add_device(ep->iommu->pq_work, dev);
	if (ret)
		return ret;

	return iommu_register_device_fault_handler(dev, iommu_queue_iopf, dev);
}

static int riscv_iommu_disable_sva(struct device *dev)
{
	int ret;
	struct riscv_iommu_endpoint *ep = dev_iommu_priv_get(dev);

	ret = iommu_unregister_device_fault_handler(dev);
	if (!ret)
		ret = iopf_queue_remove_device(ep->iommu->pq_work, dev);

	return ret;
}

static int riscv_iommu_enable_iopf(struct device *dev)
{
	struct riscv_iommu_endpoint *ep = dev_iommu_priv_get(dev);

	if (ep && ep->pri_supported)
		return 0;

	return -EINVAL;
}

static int riscv_iommu_dev_enable_feat(struct device *dev,
				       enum iommu_dev_features feat)
{
	switch (feat) {
	case IOMMU_DEV_FEAT_IOPF:
		return riscv_iommu_enable_iopf(dev);

	case IOMMU_DEV_FEAT_SVA:
		return riscv_iommu_enable_sva(dev);

	default:
		return -ENODEV;
	}
}

static int riscv_iommu_dev_disable_feat(struct device *dev,
					enum iommu_dev_features feat)
{
	switch (feat) {
	case IOMMU_DEV_FEAT_IOPF:
		return 0;

	case IOMMU_DEV_FEAT_SVA:
		return riscv_iommu_disable_sva(dev);

	default:
		return -ENODEV;
	}
}

static int riscv_iommu_page_response(struct device *dev,
				     struct iommu_fault_event *evt,
				     struct iommu_page_response *msg)
{

	dev_info(dev, "page response received. code: %d.\n", msg->code);
	/* TODO: post PRGR message to the IOMMU */
	return 0;
}

static void riscv_iommu_remove_dev_pasid(struct device *dev, ioasid_t pasid)
{
	struct riscv_iommu_endpoint *ep = dev_iommu_priv_get(dev);
	struct riscv_iommu_command cmd;

	/* remove SVA iommu-domain */

	/* invalidate TA.V */
	ep->pc[pasid].ta = 0;
	wmb();

	/* 1. invalidate PDT entry */
	__cmd_iodir_pasid(&cmd, ep->devid, pasid);
	riscv_iommu_post(ep->iommu, &cmd);

	/* 2. invalidate all matching IOATC entries */
	__cmd_inval_vma(&cmd);
	__cmd_inval_set_gscid(&cmd, 0);
	__cmd_inval_set_pscid(&cmd, pasid);
	riscv_iommu_post(ep->iommu, &cmd);

	/* 3. Wait IOATC flush to happen */
	riscv_iommu_iofence_sync(ep->iommu);
}

static const struct iommu_domain_ops riscv_iommu_domain_ops = {
	.free = riscv_iommu_domain_free,
	.attach_dev = riscv_iommu_attach_dev,
	.set_dev_pasid = riscv_iommu_set_dev_pasid,
	.map_pages = riscv_iommu_map_pages,
	.unmap_pages = riscv_iommu_unmap_pages,
	.iova_to_phys = riscv_iommu_iova_to_phys,
	.iotlb_sync = riscv_iommu_iotlb_sync,
	.iotlb_sync_map = riscv_iommu_iotlb_sync_map,
	.flush_iotlb_all = riscv_iommu_flush_iotlb_all,
	.enable_nesting = riscv_iommu_enable_nesting,
};

static const struct iommu_ops riscv_iommu_ops = {
	.owner = THIS_MODULE,
	.pgsize_bitmap = SZ_4K,
	.capable = riscv_iommu_capable,
	.domain_alloc = riscv_iommu_domain_alloc,
	.probe_device = riscv_iommu_probe_device,
	.probe_finalize = riscv_iommu_probe_finalize,
	.release_device = riscv_iommu_release_device,
	.remove_dev_pasid = riscv_iommu_remove_dev_pasid,
	.device_group = riscv_iommu_device_group,
	.get_resv_regions = riscv_iommu_get_resv_regions,
	.of_xlate = riscv_iommu_of_xlate,
	.dev_enable_feat = riscv_iommu_dev_enable_feat,
	.dev_disable_feat = riscv_iommu_dev_disable_feat,
	.page_response = riscv_iommu_page_response,
	.default_domain_ops = &riscv_iommu_domain_ops,
};

#define Q_HEAD(q) ((q)->qbr + (RIO_REG_CQH - RIO_REG_CQB))
#define Q_TAIL(q) ((q)->qbr + (RIO_REG_CQT - RIO_REG_CQB))

static unsigned riscv_iommu_queue_consume(struct riscv_iommu_device *iommu,
					  struct riscv_iommu_queue *q,
					  unsigned *ready)
{
	u32 tail = riscv_iommu_readl(iommu, Q_TAIL(q));
	*ready = q->lui;

	BUG_ON(q->cnt <= tail);
	if (q->lui <= tail)
		return tail - q->lui;
	return q->cnt - q->lui;
}

static void riscv_iommu_queue_release(struct riscv_iommu_device *iommu,
				      struct riscv_iommu_queue *q,
				      unsigned count)
{
	q->lui = (q->lui + count) & (q->cnt - 1);
	riscv_iommu_writel(iommu, Q_HEAD(q), q->lui);
}

static u32 riscv_iommu_queue_ctrl(struct riscv_iommu_device *iommu,
				  struct riscv_iommu_queue *q, u32 val)
{
	cycles_t end_cycles = RISCV_IOMMU_TIMEOUT + get_cycles();

	riscv_iommu_writel(iommu, q->qcr, val);
	do {
		val = riscv_iommu_readl(iommu, q->qcr);
		if (!(val & RIO_CQ_BUSY))
			break;
		cpu_relax();
	} while (get_cycles() < end_cycles);

	return val;
}

static int riscv_iommu_queue_init(struct riscv_iommu_device *iommu,
				  struct riscv_iommu_queue *q, unsigned count,
				  size_t item_size, unsigned id, unsigned qbr,
				  unsigned qcr, irq_handler_t irq_fn,
				  const char *name)
{
	struct device *dev = iommu->dev;
	unsigned order = ilog2(count);

	do {
		size_t size = item_size * (1ULL << order);
		q->base = dmam_alloc_coherent(dev, size, &q->base_dma,
					      GFP_KERNEL);
		if (q->base || size < PAGE_SIZE)
			break;

		order--;
	} while (1);

	if (!q->base) {
		dev_err(dev, "failed to allocate %s queue (cnt: %u)\n",
			name, count);
		return -ENOMEM;
	}

	q->len = item_size;
	q->cnt = 1ULL << order;
	q->qbr = qbr;
	q->qcr = qcr;

	switch (FIELD_GET(RIO_CAP_IGS, iommu->cap)) {
	case RIO_IGS_MSI:
	case RIO_IGS_ANY:
		q->irq = msi_get_virq(dev, id);
		if (!q->irq) {
			dev_warn(dev, "no MSI vector %d for %s\n", id, name);
		} else if (request_threaded_irq(q->irq, NULL, irq_fn, IRQF_ONESHOT, dev_name(dev), q)) {
			dev_warn(dev, "fail to request irq %d for %s\n", q->irq, name);
			q->irq = 0;
		}
		break;

	case RIO_IGS_WSI:
		dev_err(dev, "MSI support is missing.\n");
		break;

	default:
		dev_err(dev, "invalid interrupt generation support.\n");
		break;
	}

	if (!q->irq) {
		/* polling mode not implemented. */
		return -ENODEV;
	}

	riscv_iommu_writeq(iommu, qbr, (order - 1) | phys_to_ppn(q->base_dma));
	riscv_iommu_queue_ctrl(iommu, q, RIO_CQ_EN | RIO_CQ_IE);

	return 0;
}

static void riscv_iommu_queue_free(struct riscv_iommu_device *iommu,
				   struct riscv_iommu_queue *q)
{
	size_t size = q->len * q->cnt;

	riscv_iommu_queue_ctrl(iommu, q, 0);

	if (q->base)
		dmam_free_coherent(iommu->dev, size, q->base, q->base_dma);
	if (q->irq)
		free_irq(q->irq, q);
}

/* Command queue interrupt hanlder thread function */
static irqreturn_t riscv_iommu_cmdq_handler(int irq, void *data)
{
	struct riscv_iommu_queue *q = (struct riscv_iommu_queue *)data;
	struct riscv_iommu_device *iommu;
	unsigned ctrl;

	iommu = container_of(q, struct riscv_iommu_device, cmdq);

	/* Error reporting, clear error reports if any. */
	ctrl = riscv_iommu_readl(iommu, RIO_REG_CQCSR);
	if (ctrl & (RIO_CQ_FAULT | RIO_CQ_TIMEOUT | RIO_CQ_ERROR)) {
		riscv_iommu_queue_ctrl(iommu, &iommu->cmdq, ctrl);
		dev_warn_ratelimited(iommu->dev,
			"Command queue error: fault: %d tout: %d err: %d\n",
			!!(ctrl & RIO_CQ_FAULT),
			!!(ctrl & RIO_CQ_TIMEOUT),
			!!(ctrl & RIO_CQ_ERROR));
	}

	/* Clear fault interrupt pending. */
	riscv_iommu_writel(iommu, RIO_REG_IPSR, RIO_IPSR_CQIP);

	return IRQ_HANDLED;
}

static void riscv_iommu_fault_report(struct riscv_iommu_device *iommu,
				     struct riscv_iommu_event *event)
{
	unsigned bdf, err;

	bdf = FIELD_GET(RIO_EVENT_DID, event->reason);
	err = FIELD_GET(RIO_EVENT_CAUSE, event->reason);

	dev_warn_ratelimited(iommu->dev, "Fault %d bdf: %04x:%02x.%x"
				" iova: %llx gpa: %llx\n",
			     err, PCI_BUS_NUM(bdf), PCI_SLOT(bdf),
			     PCI_FUNC(bdf), event->iova, event->phys);
}

/* Fault queue interrupt hanlder thread function */
static irqreturn_t riscv_iommu_fault_handler(int irq, void *data)
{
	struct riscv_iommu_queue *q = (struct riscv_iommu_queue *)data;
	struct riscv_iommu_device *iommu;
	struct riscv_iommu_event *events;
	unsigned cnt, len, idx, ctrl;

	iommu = container_of(q, struct riscv_iommu_device, fltq);
	events = (struct riscv_iommu_event *)q->base;

	/* Error reporting, clear error reports if any. */
	ctrl = riscv_iommu_readl(iommu, RIO_REG_FQCSR);
	if (ctrl & (RIO_FQ_FULL | RIO_FQ_FAULT)) {
		riscv_iommu_queue_ctrl(iommu, &iommu->fltq, ctrl);
		dev_warn_ratelimited(iommu->dev,
			"Fault queue error: fault: %d full: %d\n",
			!!(ctrl & RIO_FQ_FAULT), !!(ctrl & RIO_FQ_FULL));
	}

	/* Clear fault interrupt pending. */
	riscv_iommu_writel(iommu, RIO_REG_IPSR, RIO_IPSR_FQIP);

	/* Report fault events. */
	do {
		cnt = riscv_iommu_queue_consume(iommu, q, &idx);
		if (!cnt)
			break;
		for (len = 0; len < cnt; idx++, len++)
			riscv_iommu_fault_report(iommu, &events[idx]);
		riscv_iommu_queue_release(iommu, q, cnt);
	} while (1);

	return IRQ_HANDLED;
}

static struct riscv_iommu_endpoint *
riscv_iommu_find_ep(struct riscv_iommu_device *iommu, unsigned devid)
{
	struct rb_node *node;
	struct riscv_iommu_endpoint *ep;

	lockdep_assert_held(&iommu->eps_mutex);

	node = iommu->eps.rb_node;
	while (node) {
		ep = rb_entry(node, struct riscv_iommu_endpoint, node);
		if (ep->devid < devid)
			node = node->rb_right;
		else if (ep->devid > devid)
			node = node->rb_left;
		else
			return ep;
	}

	return NULL;
}

static void riscv_iommu_page_request(struct riscv_iommu_device *iommu,
				     struct riscv_iommu_page_request *req)
{
	struct iommu_fault_event event = { 0 };
	struct iommu_fault_page_request *prm = &event.fault.prm;
	struct riscv_iommu_endpoint *ep;

	/* Payload: PCI Specification, Chapter 10.4.1 Page Request Message */
	#define PRM_R    1
	#define PRM_W    2
	#define PRM_L    4
	#define PRM_M    (PRM_L | PRM_W | PRM_R)

	/* Ignore PGR Stop marker. */
	if ((req->payload & PRM_M) == PRM_L)
		return;

	event.fault.type = IOMMU_FAULT_PAGE_REQ;

	mutex_lock(&iommu->eps_mutex);
	ep = riscv_iommu_find_ep(iommu, FIELD_GET(RIO_PRR_DID, req->request));
	if (!ep) {
		/* TODO: Handle invalid page request */
		mutex_unlock(&iommu->eps_mutex);
		return;
	}

	if (req->payload & PRM_L)
		prm->flags |= IOMMU_FAULT_PAGE_REQUEST_LAST_PAGE;
	if (req->payload & PRM_W)
		prm->perm |= IOMMU_FAULT_PERM_WRITE;
	if (req->payload & PRM_R)
		prm->perm |= IOMMU_FAULT_PERM_READ;

	prm->grpid = (req->payload >> 3) & ((1U << 9) - 1);
	prm->addr = req->payload & PAGE_MASK;

	if (req->request & RIO_PRR_PV) {
		prm->flags |= IOMMU_FAULT_PAGE_REQUEST_PASID_VALID;
		/* TODO: where to find this bit */
		prm->flags |= IOMMU_FAULT_PAGE_RESPONSE_NEEDS_PASID;
		prm->pasid = FIELD_GET(RIO_PRR_PID, req->request);
	}

	iommu_report_device_fault(ep->dev, &event);
	mutex_unlock(&iommu->eps_mutex);
}

/* Page request interface queue interrupt hanlder thread function */
static irqreturn_t riscv_iommu_page_request_handler(int irq, void *data)
{
	struct riscv_iommu_queue *q = (struct riscv_iommu_queue *)data;
	struct riscv_iommu_device *iommu;
	struct riscv_iommu_page_request *requests;
	unsigned cnt, len, idx, ctrl;

	iommu = container_of(q, struct riscv_iommu_device, priq);
	requests = (struct riscv_iommu_page_request *)q->base;

	/* Error reporting, clear error reports if any. */
	ctrl = riscv_iommu_readl(iommu, RIO_REG_PQCSR);
	if (ctrl & (RIO_PQ_FULL | RIO_PQ_FAULT)) {
		riscv_iommu_queue_ctrl(iommu, &iommu->priq, ctrl);
		dev_warn_ratelimited(iommu->dev,
			"Page request queue error: fault: %d full: %d\n",
			!!(ctrl & RIO_PQ_FAULT), !!(ctrl & RIO_PQ_FULL));
	}

	/* Clear page request interrupt pending. */
	riscv_iommu_writel(iommu, RIO_REG_IPSR, RIO_IPSR_PQIP);

	/* Process page requests. */
	do {
		cnt = riscv_iommu_queue_consume(iommu, q, &idx);
		if (!cnt)
			break;
		for (len = 0; len < cnt; idx++, len++)
			riscv_iommu_page_request(iommu, &requests[idx]);
		riscv_iommu_queue_release(iommu, q, cnt);
	} while (1);

	return IRQ_HANDLED;
}

/* Wait for DDTP.BUSY to be cleared and return latest value */
static u64 riscv_iommu_get_ddtp(struct riscv_iommu_device *iommu)
{
	u64 ddtp;
	cycles_t end_cycles = RISCV_IOMMU_TIMEOUT + get_cycles();

	do {
		ddtp = riscv_iommu_readq(iommu, RIO_REG_DDTP);
		if (!(ddtp & RIO_DDTP_BUSY))
			break;
		cpu_relax();
	} while (get_cycles() < end_cycles);

	return ddtp;
}

/* Enable IOMMU Translation Mode. */
static int riscv_iommu_enable(struct riscv_iommu_device *iommu, unsigned mode)
{
	u64 ddtp;

	ddtp = riscv_iommu_get_ddtp(iommu);
	if (ddtp & RIO_DDTP_BUSY)
		return -EBUSY;

	/* Disallow state transtion from xLV to xLV. */
	switch (FIELD_GET(RIO_DDTP_MODE, ddtp)) {
	case RIO_DDTP_MODE_BARE:
	case RIO_DDTP_MODE_OFF:
		break;
	default:
		if ((mode != RIO_DDTP_MODE_BARE) && (mode != RIO_DDTP_MODE_OFF))
			return -EINVAL;
		break;
	}

	/* Verify supported DDTP.MODE values */
	switch (mode) {
	case RIO_DDTP_MODE_BARE:
	case RIO_DDTP_MODE_OFF:
		if (iommu->ddtp) {
			// TODO: teardown whole tree.
			free_pages(iommu->ddtp, 0);
			iommu->ddtp = 0;
		}
		ddtp = FIELD_PREP(RIO_DDTP_MODE, mode);
		break;
	case RIO_DDTP_MODE_1LVL:
	case RIO_DDTP_MODE_2LVL:
	case RIO_DDTP_MODE_3LVL:
		if (!iommu->ddtp)
			iommu->ddtp = get_zeroed_page(GFP_KERNEL);
		if (!iommu->ddtp)
			return -ENOMEM;
		ddtp = FIELD_PREP(RIO_DDTP_MODE, mode) |
		    phys_to_ppn(__pa(iommu->ddtp));
		break;
	default:
		return -EINVAL;
	}

	riscv_iommu_writeq(iommu, RIO_REG_DDTP, ddtp);
	ddtp = riscv_iommu_get_ddtp(iommu);

	if (ddtp & RIO_DDTP_BUSY)
		return -EBUSY;

	if (FIELD_GET(RIO_DDTP_MODE, ddtp) != mode)
		return -EINVAL;

	iommu->ddt_mode = mode;
	return 0;
}

/* Common IOMMU driver teardown code */
void riscv_iommu_remove(struct device *dev)
{
	struct riscv_iommu_device *iommu = dev_get_drvdata(dev);

	iommu_device_unregister(&iommu->iommu);
	iommu_device_sysfs_remove(&iommu->iommu);
	riscv_iommu_enable(iommu, RIO_DDTP_MODE_OFF);
	riscv_iommu_queue_free(iommu, &iommu->cmdq);
	riscv_iommu_queue_free(iommu, &iommu->fltq);
	riscv_iommu_queue_free(iommu, &iommu->priq);
	iopf_queue_free(iommu->pq_work);
	kfree(iommu);
}

/* Common IOMMU driver probe and setup code */
int riscv_iommu_probe(struct device *dev, phys_addr_t reg_phys, size_t reg_size)
{
	int ret;
	struct riscv_iommu_device *iommu;

	iommu = kzalloc(sizeof(*iommu), GFP_KERNEL);
	if (!iommu)
		return -ENOMEM;

	iommu->reg_phys = reg_phys;
	iommu->reg_size = reg_size;
	iommu->dev = dev;
	iommu->eps = RB_ROOT;

	iommu->reg = ioremap(iommu->reg_phys, iommu->reg_size);
	if (!iommu->reg) {
		dev_err(dev, "unable to map hardware register set\n");
		kfree(iommu);
		return -ENOMEM;
	}

	iommu->cap = riscv_iommu_readq(iommu, RIO_REG_CAP);
	if (FIELD_GET(RIO_CAP_REV, iommu->cap) != RIO_SPEC_DOT_VER) {
		dev_err(dev, "unsupported hardware interface revision\n");
		kfree(iommu);
		return -ENODEV;
	}

	if (iommu->cap & RIO_CAP_PD20)
		iommu->iommu.max_pasids = 1u << 20;
	else if (iommu->cap & RIO_CAP_PD17)
		iommu->iommu.max_pasids = 1u << 17;
	else if (iommu->cap & RIO_CAP_PD8)
		iommu->iommu.max_pasids = 1u << 8;

	/* Clear any pending interrupt flag. */
	riscv_iommu_writel(iommu, RIO_REG_IPSR,
			   RIO_IPSR_CQIP | RIO_IPSR_FQIP | RIO_IPSR_PQIP);

	/* Set simple 1:1 mapping for MSI vectors */
	riscv_iommu_writel(iommu, RIO_REG_IVEC, 0x3210);

	spin_lock_init(&iommu->cq_lock);
	mutex_init(&iommu->eps_mutex);

	ret = riscv_iommu_queue_init(iommu, &iommu->cmdq, cmdq_length,
				     sizeof(struct riscv_iommu_command),
				     RIO_INT_CQ, RIO_REG_CQB, RIO_REG_CQCSR,
				     riscv_iommu_cmdq_handler, "cmdq");
	if (ret)
		goto fail;

	ret = riscv_iommu_queue_init(iommu, &iommu->fltq, fltq_length,
				     sizeof(struct riscv_iommu_event),
				     RIO_INT_FQ, RIO_REG_FQB, RIO_REG_FQCSR,
				     riscv_iommu_fault_handler, "fltq");
	if (ret)
		goto fail;

	if (!(iommu->cap & RIO_CAP_ATS))
		goto no_ats;

	/* PRI functionally depends on ATS’s capabilities. */
	iommu->pq_work = iopf_queue_alloc(dev_name(dev));
	if (!iommu->pq_work) {
		dev_err(dev, "failed to allocate iopf queue\n");
		ret = -ENOMEM;
		goto fail;
	}

	ret = riscv_iommu_queue_init(iommu, &iommu->priq, priq_length,
				     sizeof(struct riscv_iommu_page_request),
				     RIO_INT_PQ, RIO_REG_PQB, RIO_REG_PQCSR,
				     riscv_iommu_page_request_handler, "priq");
	if (ret)
		goto fail;

 no_ats:
	ret = riscv_iommu_enable(iommu, ddt_mode);
	if (ret) {
		dev_err(dev, "cannot enable iommu device (%d)\n", ret);
		goto fail;
	}

	ret = riscv_iommu_sysfs_add(iommu);
	if (ret) {
		dev_err(dev, "cannot register sysfs interface (%d)\n", ret);
		goto fail;
	}

	ret = iommu_device_register(&iommu->iommu, &riscv_iommu_ops, dev);
	if (ret) {
		dev_err(dev, "cannot register iommu interface (%d)\n", ret);
		iommu_device_sysfs_remove(&iommu->iommu);
		goto fail;
	}

	dev_set_drvdata(dev, iommu);

	return 0;

 fail:
	riscv_iommu_enable(iommu, RIO_DDTP_MODE_OFF);
	riscv_iommu_queue_free(iommu, &iommu->priq);
	riscv_iommu_queue_free(iommu, &iommu->fltq);
	riscv_iommu_queue_free(iommu, &iommu->cmdq);
	iopf_queue_free(iommu->pq_work);
	iounmap(iommu->reg);
	kfree(iommu);

	return ret;
}
