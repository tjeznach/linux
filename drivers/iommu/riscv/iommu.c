// SPDX-License-Identifier: GPL-2.0-only
/*
 * IOMMU API for RISC-V architected Ziommu implementations.
 *
 * Copyright © 2022-2023 Rivos Inc.
 * Copyright © 2023 FORTH-ICS/CARV
 *
 * Authors
 *	Tomasz Jeznach <tjeznach@rivosinc.com>
 *	Nick Kossifidis <mick@ics.forth.gr>
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
#include <linux/irqdomain.h>
#include <linux/platform_device.h>
#include <linux/dma-map-ops.h>
#include <asm/page.h>

#include "../dma-iommu.h"
#include "../iommu-sva.h"
#include "iommu.h"

#include <asm/csr.h>
#include <asm/delay.h>

MODULE_DESCRIPTION("IOMMU driver for RISC-V architected Ziommu implementations");
MODULE_AUTHOR("Tomasz Jeznach <tjeznach@rivosinc.com>");
MODULE_AUTHOR("Nick Kossifidis <mick@ics.forth.gr>");
MODULE_ALIAS("riscv-iommu");
MODULE_LICENSE("GPL v2");

/* Global IOMMU params. */
static int ddt_mode = RISCV_IOMMU_DDTP_MODE_BARE;
module_param(ddt_mode, int, 0644);
MODULE_PARM_DESC(ddt_mode, "Device Directory Table mode.");

/* IOMMU PSCID allocation namespace. */
#define RISCV_IOMMU_MAX_PSCID	(1U << 20)
static DEFINE_IDA(riscv_iommu_pscids);

/* 1 second */
#define RISCV_IOMMU_TIMEOUT	riscv_timebase

/* RISC-V IOMMU PPN <> PHYS address conversions, PHYS <=> PPN[53:10] */
#define phys_to_ppn(va)  (((va) >> 2) & (((1ULL << 44) - 1) << 10))
#define ppn_to_phys(pn)	 (((pn) << 2) & (((1ULL << 44) - 1) << 12))

#define iommu_domain_to_riscv(iommu_domain) \
    container_of(iommu_domain, struct riscv_iommu_domain, domain)

#define iommu_device_to_riscv(iommu_device) \
    container_of(iommu_device, struct riscv_iommu, iommu)

static const struct iommu_domain_ops riscv_iommu_domain_ops;
static const struct iommu_ops riscv_iommu_ops;

/*
 * Register device for IOMMU tracking.
 */
static void riscv_iommu_add_device(struct riscv_iommu_device *iommu, struct device *dev)
{
	struct riscv_iommu_endpoint *ep, *rb_ep;
	struct rb_node **new_node, *parent_node = NULL;

	mutex_lock(&iommu->eps_mutex);

	ep = dev_iommu_priv_get(dev);

	new_node = &(iommu->eps.rb_node);
	while (*new_node) {
		rb_ep = rb_entry(*new_node, struct riscv_iommu_endpoint, node);
		parent_node = *new_node;
		if (rb_ep->devid > ep->devid) {
			new_node = &((*new_node)->rb_left);
		} else if (rb_ep->devid < ep->devid) {
			new_node = &((*new_node)->rb_right);
		} else {
			dev_warn(dev, "device %u already in the tree\n", ep->devid);
			break;
		}
	}

	rb_link_node(&ep->node, parent_node, new_node);
	rb_insert_color(&ep->node, &iommu->eps);

	mutex_unlock(&iommu->eps_mutex);
}

/*
 * Endpoint management
 */

static int riscv_iommu_of_xlate(struct device *dev, struct of_phandle_args *args)
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

static struct iommu_device *riscv_iommu_probe_device(struct device *dev)
{
	struct riscv_iommu_device *iommu;
	struct riscv_iommu_endpoint *ep;
	struct iommu_fwspec *fwspec;

	fwspec = dev_iommu_fwspec_get(dev);
	if (!fwspec || fwspec->ops != &riscv_iommu_ops ||
	    !fwspec->iommu_fwnode || !fwspec->iommu_fwnode->dev)
		return ERR_PTR(-ENODEV);

	iommu = dev_get_drvdata(fwspec->iommu_fwnode->dev);
	if (!iommu)
		return ERR_PTR(-ENODEV);

	if (dev_iommu_priv_get(dev))
		return &iommu->iommu;

	ep = kzalloc(sizeof(*ep), GFP_KERNEL);
	if (!ep)
		return ERR_PTR(-ENOMEM);

	mutex_init(&ep->lock);
	INIT_LIST_HEAD(&ep->domain);

	if (dev_is_pci(dev)) {
		ep->devid = pci_dev_id(to_pci_dev(dev));
		ep->domid = pci_domain_nr(to_pci_dev(dev)->bus);
	} else {
		/* TODO: Make this generic, for now hardcode domain id to 0 */
		ep->devid = fwspec->ids[0];
		ep->domid = 0;
	}

	ep->iommu = iommu;
	ep->dev = dev;

	dev_info(iommu->dev, "adding device to iommu with devid %i in domain %i\n",
		ep->devid, ep->domid);

	dev_iommu_priv_set(dev, ep);
	riscv_iommu_add_device(iommu, dev);

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

	dev_info(dev, "device with devid %i released\n", ep->devid);

	mutex_lock(&ep->lock);
	list_del(&ep->domain);
	mutex_unlock(&ep->lock);

	/* Remove endpoint from IOMMU tracking structures */
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

static void riscv_iommu_get_resv_regions(struct device *dev, struct list_head *head)
{
	iommu_dma_get_resv_regions(dev, head);
}

/*
 * Domain management
 */

static struct iommu_domain *riscv_iommu_domain_alloc(unsigned type)
{
	struct riscv_iommu_domain *domain;

	if (type != IOMMU_DOMAIN_IDENTITY &&
	    type != IOMMU_DOMAIN_BLOCKED)
		return NULL;

	domain = kzalloc(sizeof(*domain), GFP_KERNEL);
	if (!domain)
		return NULL;

	mutex_init(&domain->lock);
	INIT_LIST_HEAD(&domain->endpoints);

	domain->domain.ops = &riscv_iommu_domain_ops;
	domain->mode = RISCV_IOMMU_DC_FSC_MODE_BARE;
	domain->pscid = ida_alloc_range(&riscv_iommu_pscids, 1,
					RISCV_IOMMU_MAX_PSCID, GFP_KERNEL);

	printk("domain type %x alloc %u\n", type, domain->pscid);

	return &domain->domain;
}

static void riscv_iommu_domain_free(struct iommu_domain *iommu_domain)
{
	struct riscv_iommu_domain *domain = iommu_domain_to_riscv(iommu_domain);

	if (!list_empty(&domain->endpoints)) {
		pr_warn("IOMMU domain is not empty!\n");
	}

	if (domain->pgd_root)
		free_pages((unsigned long)domain->pgd_root, 0);

	if ((int)domain->pscid > 0)
		ida_free(&riscv_iommu_pscids, domain->pscid);

	printk("domain free %u\n", domain->pscid);

	kfree(domain);
}

static int riscv_iommu_domain_finalize(struct riscv_iommu_domain *domain,
				       struct riscv_iommu_device *iommu)
{
	struct iommu_domain_geometry *geometry;

	/* Domain assigned to another iommu */
	if (domain->iommu && domain->iommu != iommu)
		return -EINVAL;
	/* Domain already initialized */
	else if (domain->iommu)
		return 0;

	/*
	 * TODO: Before using VA_BITS and satp_mode here, verify they
	 * are supported by the iommu, through the capabilities register.
	 */

	geometry = &domain->domain.geometry;

	/*
	 * Note: RISC-V Privilege spec mandates that virtual addresses
	 * need to be sign-extended, so if (VA_BITS - 1) is set, all
	 * bits >= VA_BITS need to also be set or else we'll get a
	 * page fault. However the code that creates the mappings
	 * above us (e.g. iommu_dma_alloc_iova()) won't do that for us
	 * for now, so we'll end up with invalid virtual addresses
	 * to map. As a workaround until we get this sorted out
	 * limit the available virtual addresses to VA_BITS - 1.
	 */
	geometry->aperture_start = 0;
	geometry->aperture_end = DMA_BIT_MASK(VA_BITS - 1);
	geometry->force_aperture = true;

	domain->iommu = iommu;

	if (domain->domain.type == IOMMU_DOMAIN_IDENTITY)
		return 0;

	/* TODO: Fix this for RV32 */
	domain->mode = satp_mode >> 60;
	domain->pgd_root = (pgd_t *) __get_free_pages(GFP_KERNEL | __GFP_ZERO, 0);

	if (!domain->pgd_root)
		return -ENOMEM;

	return 0;
}

static int riscv_iommu_attach_dev(struct iommu_domain *iommu_domain, struct device *dev)
{
	struct riscv_iommu_domain *domain = iommu_domain_to_riscv(iommu_domain);
	struct riscv_iommu_endpoint *ep = dev_iommu_priv_get(dev);
	int ret;

	/* PSCID not valid */
	if ((int)domain->pscid < 0)
		return -ENOMEM;

	mutex_lock(&domain->lock);
	mutex_lock(&ep->lock);

	if (!list_empty(&ep->domain)) {
		dev_warn(dev, "endpoint already attached to a domain. dropping\n");
		list_del_init(&ep->domain);
	}

	/* allocate root pages, initialize io-pgtable ops, etc. */
	ret = riscv_iommu_domain_finalize(domain, ep->iommu);
	if (ret < 0) {
		dev_err(dev, "can not finalize domain: %d\n", ret);
		mutex_unlock(&ep->lock);
		mutex_unlock(&domain->lock);
		return ret;
	}

	if (ep->iommu->ddt_mode != RISCV_IOMMU_DDTP_MODE_BARE ||
	    domain->domain.type != IOMMU_DOMAIN_IDENTITY) {
		dev_warn(dev, "domain type %d not supported\n",
		    domain->domain.type);
		return -ENODEV;
	}

	list_add_tail(&ep->domain, &domain->endpoints);
	mutex_unlock(&ep->lock);
	mutex_unlock(&domain->lock);

	dev_info(dev, "domain type %d attached w/ PSCID %u\n",
	    domain->domain.type, domain->pscid);

	return 0;
}

static void riscv_iommu_flush_iotlb_range(struct iommu_domain *iommu_domain,
					  unsigned long *start, unsigned long *end,
					  size_t *pgsize)
{
	/* Command interface not implemented */
}

static void riscv_iommu_flush_iotlb_all(struct iommu_domain *iommu_domain)
{
	riscv_iommu_flush_iotlb_range(iommu_domain, NULL, NULL, NULL);
}

static void riscv_iommu_iotlb_sync(struct iommu_domain *iommu_domain,
				   struct iommu_iotlb_gather *gather)
{
	riscv_iommu_flush_iotlb_range(iommu_domain, &gather->start, &gather->end,
				      &gather->pgsize);
}

static void riscv_iommu_iotlb_sync_map(struct iommu_domain *iommu_domain,
				       unsigned long iova, size_t size)
{
	unsigned long end = iova + size - 1;
	/*
	 * Given we don't know the page size used by this range, we assume the
	 * smallest page size to ensure all possible entries are flushed from
	 * the IOATC.
	 */
	size_t pgsize = PAGE_SIZE;
	riscv_iommu_flush_iotlb_range(iommu_domain, &iova, &end, &pgsize);
}

static int riscv_iommu_map_pages(struct iommu_domain *iommu_domain,
				 unsigned long iova, phys_addr_t phys,
				 size_t pgsize, size_t pgcount, int prot,
				 gfp_t gfp, size_t *mapped)
{
	struct riscv_iommu_domain *domain = iommu_domain_to_riscv(iommu_domain);

	if (domain->domain.type == IOMMU_DOMAIN_IDENTITY) {
		*mapped = pgsize * pgcount;
		return 0;
	}

	return -ENODEV;
}

static size_t riscv_iommu_unmap_pages(struct iommu_domain *iommu_domain,
				      unsigned long iova, size_t pgsize,
				      size_t pgcount, struct iommu_iotlb_gather *gather)
{
	struct riscv_iommu_domain *domain = iommu_domain_to_riscv(iommu_domain);

	if (domain->domain.type == IOMMU_DOMAIN_IDENTITY)
		return pgsize * pgcount;

	return 0;
}

static phys_addr_t riscv_iommu_iova_to_phys(struct iommu_domain *iommu_domain,
					    dma_addr_t iova)
{
	struct riscv_iommu_domain *domain = iommu_domain_to_riscv(iommu_domain);

	if (domain->domain.type == IOMMU_DOMAIN_IDENTITY)
		return (phys_addr_t) iova;

	return 0;
}

/*
 * Translation mode setup
 */

static u64 riscv_iommu_get_ddtp(struct riscv_iommu_device *iommu)
{
	u64 ddtp;
	cycles_t end_cycles = RISCV_IOMMU_TIMEOUT + get_cycles();

	/* Wait for DDTP.BUSY to be cleared and return latest value */
	do {
		ddtp = riscv_iommu_readq(iommu, RISCV_IOMMU_REG_DDTP);
		if (!(ddtp & RISCV_IOMMU_DDTP_BUSY))
			break;
		cpu_relax();
	} while (get_cycles() < end_cycles);

	return ddtp;
}

static void riscv_iommu_ddt_cleanup(struct riscv_iommu_device *iommu)
{
	/* TODO: teardown whole device directory tree. */
	if (iommu->ddtp) {
		if (iommu->ddtp_in_iomem) {
			iounmap((void *)iommu->ddtp);
		} else
			free_page(iommu->ddtp);
		iommu->ddtp = 0;
	}
}

static int riscv_iommu_enable(struct riscv_iommu_device *iommu, unsigned requested_mode)
{
	struct device *dev = iommu->dev;
	u64 ddtp = 0;
	u64 ddtp_paddr = 0;
	unsigned mode = requested_mode;
	unsigned mode_readback = 0;

	ddtp = riscv_iommu_get_ddtp(iommu);
	if (ddtp & RISCV_IOMMU_DDTP_BUSY)
		return -EBUSY;

	/* Disallow state transtion from xLVL to xLVL. */
	switch (FIELD_GET(RISCV_IOMMU_DDTP_MODE, ddtp)) {
	case RISCV_IOMMU_DDTP_MODE_BARE:
	case RISCV_IOMMU_DDTP_MODE_OFF:
		break;
	default:
		if ((mode != RISCV_IOMMU_DDTP_MODE_BARE)
		    && (mode != RISCV_IOMMU_DDTP_MODE_OFF))
			return -EINVAL;
		break;
	}

 retry:
	switch (mode) {
	case RISCV_IOMMU_DDTP_MODE_BARE:
	case RISCV_IOMMU_DDTP_MODE_OFF:
		riscv_iommu_ddt_cleanup(iommu);
		ddtp = FIELD_PREP(RISCV_IOMMU_DDTP_MODE, mode);
		break;
	case RISCV_IOMMU_DDTP_MODE_1LVL:
	case RISCV_IOMMU_DDTP_MODE_2LVL:
	case RISCV_IOMMU_DDTP_MODE_3LVL:
		if (!iommu->ddtp) {
			/*
			 * We haven't initialized ddtp yet, since it's WARL make
			 * sure that we don't have a hardwired PPN field there
			 * that points to i/o memory instead.
			 */
			riscv_iommu_writeq(iommu, RISCV_IOMMU_REG_DDTP, 0);
			ddtp = riscv_iommu_get_ddtp(iommu);
			ddtp_paddr = ppn_to_phys(ddtp);
			if (ddtp_paddr) {
				dev_warn(dev, "ddtp at 0x%llx\n", ddtp_paddr);
				iommu->ddtp =
				    (unsigned long)ioremap(ddtp_paddr, PAGE_SIZE);
				iommu->ddtp_in_iomem = true;
			} else {
				iommu->ddtp = get_zeroed_page(GFP_KERNEL);
			}
		}
		if (!iommu->ddtp)
			return -ENOMEM;

		ddtp = FIELD_PREP(RISCV_IOMMU_DDTP_MODE, mode) |
		    phys_to_ppn(__pa(iommu->ddtp));

		break;
	default:
		return -EINVAL;
	}

	riscv_iommu_writeq(iommu, RISCV_IOMMU_REG_DDTP, ddtp);
	ddtp = riscv_iommu_get_ddtp(iommu);
	if (ddtp & RISCV_IOMMU_DDTP_BUSY) {
		dev_warn(dev, "timeout when setting ddtp (ddt mode: %i)\n", mode);
		return -EBUSY;
	}

	mode_readback = FIELD_GET(RISCV_IOMMU_DDTP_MODE, ddtp);
	dev_info(dev, "mode_readback: %i, mode: %i\n", mode_readback, mode);
	if (mode_readback != mode) {
		/*
		 * Mode field is WARL, an I/O MMU may support a subset of
		 * directory table levels in which case if we tried to set
		 * an unsupported number of levels we'll readback either
		 * a valid xLVL or off/bare. If we got off/bare, try again
		 * with a smaller xLVL.
		 */
		if (mode_readback < RISCV_IOMMU_DDTP_MODE_1LVL &&
		    mode > RISCV_IOMMU_DDTP_MODE_1LVL) {
			mode--;
			goto retry;
		}

		/*
		 * We tried all supported xLVL modes and still got off/bare instead,
		 * an I/O MMU must support at least one supported xLVL mode so something
		 * went very wrong.
		 */
		if (mode_readback < RISCV_IOMMU_DDTP_MODE_1LVL &&
		    mode == RISCV_IOMMU_DDTP_MODE_1LVL)
			goto fail;

		/*
		 * We tried setting off or bare and got something else back, something
		 * went very wrong since off/bare is always legal.
		 */
		if (mode < RISCV_IOMMU_DDTP_MODE_1LVL)
			goto fail;

		/*
		 * We tried setting an xLVL mode but got another xLVL mode that
		 * we don't support (e.g. a custom one).
		 */
		if (mode_readback > RISCV_IOMMU_DDTP_MODE_MAX)
			goto fail;

		/* We tried setting an xLVL mode but got another supported xLVL mode */
		mode = mode_readback;
	}

	if (mode != requested_mode)
		dev_warn(dev, "unsupported DDT mode requested (%i), using %i instead\n",
			 requested_mode, mode);

	iommu->ddt_mode = mode;
	dev_info(dev, "ddt_mode: %i\n", iommu->ddt_mode);
	return 0;

 fail:
	dev_err(dev, "failed to set DDT mode, tried: %i and got %i\n", mode,
		mode_readback);
	riscv_iommu_ddt_cleanup(iommu);
	return -EINVAL;
}

/*
 * Common I/O MMU driver probe/teardown
 */

static const struct iommu_domain_ops riscv_iommu_domain_ops = {
	.free = riscv_iommu_domain_free,
	.attach_dev = riscv_iommu_attach_dev,
	.map_pages = riscv_iommu_map_pages,
	.unmap_pages = riscv_iommu_unmap_pages,
	.iova_to_phys = riscv_iommu_iova_to_phys,
	.iotlb_sync = riscv_iommu_iotlb_sync,
	.iotlb_sync_map = riscv_iommu_iotlb_sync_map,
	.flush_iotlb_all = riscv_iommu_flush_iotlb_all,
};

static const struct iommu_ops riscv_iommu_ops = {
	.owner = THIS_MODULE,
	.pgsize_bitmap = SZ_4K | SZ_2M | SZ_512M,
	.capable = riscv_iommu_capable,
	.domain_alloc = riscv_iommu_domain_alloc,
	.probe_device = riscv_iommu_probe_device,
	.probe_finalize = riscv_iommu_probe_finalize,
	.release_device = riscv_iommu_release_device,
	.device_group = riscv_iommu_device_group,
	.get_resv_regions = riscv_iommu_get_resv_regions,
	.of_xlate = riscv_iommu_of_xlate,
	.default_domain_ops = &riscv_iommu_domain_ops,
};

void riscv_iommu_remove(struct riscv_iommu_device *iommu)
{
	iommu_device_unregister(&iommu->iommu);
	riscv_iommu_enable(iommu, RISCV_IOMMU_DDTP_MODE_OFF);
}

int riscv_iommu_init(struct riscv_iommu_device *iommu)
{
	struct device *dev = iommu->dev;
	u32 fctl = 0;
	int ret;

	iommu->eps = RB_ROOT;

	fctl = riscv_iommu_readl(iommu, RISCV_IOMMU_REG_FCTL);

#ifdef CONFIG_CPU_BIG_ENDIAN
	if (!(cap & RISCV_IOMMU_CAP_END)) {
		dev_err(dev, "IOMMU doesn't support Big Endian\n");
		return -EIO;
	} else if (!(fctl & RISCV_IOMMU_FCTL_BE)) {
		fctl |= FIELD_PREP(RISCV_IOMMU_FCTL_BE, 1);
		riscv_iommu_writel(iommu, RISCV_IOMMU_REG_FCTL, fctl);
	}
#endif

	/* Clear any pending interrupt flag. */
	riscv_iommu_writel(iommu, RISCV_IOMMU_REG_IPSR,
			   RISCV_IOMMU_IPSR_CIP |
			   RISCV_IOMMU_IPSR_FIP |
			   RISCV_IOMMU_IPSR_PMIP | RISCV_IOMMU_IPSR_PIP);
	spin_lock_init(&iommu->cq_lock);
	mutex_init(&iommu->eps_mutex);

	ret = riscv_iommu_enable(iommu, RISCV_IOMMU_DDTP_MODE_BARE);

	if (ret) {
		dev_err(dev, "cannot enable iommu device (%d)\n", ret);
		goto fail;
	}

	ret = iommu_device_register(&iommu->iommu, &riscv_iommu_ops, dev);
	if (ret) {
		dev_err(dev, "cannot register iommu interface (%d)\n", ret);
		iommu_device_sysfs_remove(&iommu->iommu);
		goto fail;
	}

	return 0;
 fail:
	riscv_iommu_enable(iommu, RISCV_IOMMU_DDTP_MODE_OFF);
	return ret;
}
