// SPDX-License-Identifier: GPL-2.0-only
/*
 * IOMMU API for RISC-V IOMMU implementations.
 *
 * Copyright © 2022-2023 Rivos Inc.
 * Copyright © 2023 FORTH-ICS/CARV
 *
 * Authors
 *	Tomasz Jeznach <tjeznach@rivosinc.com>
 *	Nick Kossifidis <mick@ics.forth.gr>
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/compiler.h>
#include <linux/debugfs.h>
#include <linux/dma-mapping.h>
#include <linux/init.h>
#include <linux/iommu.h>
#include <linux/iopoll.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/pci.h>

#include "iommu-bits.h"
#include "iommu.h"

MODULE_DESCRIPTION("Driver for RISC-V IOMMU");
MODULE_AUTHOR("Tomasz Jeznach <tjeznach@rivosinc.com>");
MODULE_AUTHOR("Nick Kossifidis <mick@ics.forth.gr>");
MODULE_ALIAS("riscv-iommu");
MODULE_LICENSE("GPL v2");

/* Timeouts in [us] */
#define RISCV_IOMMU_DDTP_TIMEOUT	50000

/* RISC-V IOMMU PPN <> PHYS address conversions, PHYS <=> PPN[53:10] */
#define phys_to_ppn(va)  (((va) >> 2) & (((1ULL << 44) - 1) << 10))
#define ppn_to_phys(pn)	 (((pn) << 2) & (((1ULL << 44) - 1) << 12))

#define dev_to_iommu(dev) \
	container_of((dev)->iommu->iommu_dev, struct riscv_iommu_device, iommu)

/* Device resource-managed allocations */
struct riscv_iommu_devres {
	unsigned long addr;
	unsigned int order;
};

static void riscv_iommu_devres_pages_release(struct device *dev, void *res)
{
	struct riscv_iommu_devres *devres = res;

	free_pages(devres->addr, devres->order);
}

static int riscv_iommu_devres_pages_match(struct device *dev, void *res, void *p)
{
	struct riscv_iommu_devres *devres = res;
	struct riscv_iommu_devres *target = p;

	return devres->addr == target->addr;
}

static unsigned long riscv_iommu_get_pages(struct riscv_iommu_device *iommu,
					   unsigned int order)
{
	struct riscv_iommu_devres *devres;
	struct page *pages;

	pages = alloc_pages_node(dev_to_node(iommu->dev),
				 GFP_KERNEL_ACCOUNT | __GFP_ZERO, order);
	if (unlikely(!pages)) {
		dev_err(iommu->dev, "Page allocation failed, order %u\n", order);
		return 0;
	}

	devres = devres_alloc(riscv_iommu_devres_pages_release,
			      sizeof(struct riscv_iommu_devres), GFP_KERNEL);

	if (unlikely(!devres)) {
		__free_pages(pages, order);
		return 0;
	}

	devres->addr = (unsigned long)page_address(pages);
	devres->order = order;

	devres_add(iommu->dev, devres);

	return devres->addr;
}

static void riscv_iommu_free_pages(struct riscv_iommu_device *iommu, unsigned long addr)
{
	struct riscv_iommu_devres devres = { .addr = addr };

	WARN_ON(devres_release(iommu->dev, riscv_iommu_devres_pages_release,
			       riscv_iommu_devres_pages_match, &devres));
}

/* Lookup and initialize device context info structure. */
static struct riscv_iommu_dc *riscv_iommu_get_dc(struct riscv_iommu_device *iommu,
						 unsigned int devid)
{
	const bool base_format = !(iommu->caps & RISCV_IOMMU_CAP_MSI_FLAT);
	unsigned int depth;
	unsigned long ddt, ptr, old, new;
	u8 ddi_bits[3] = { 0 };
	u64 *ddtp = NULL;

	/* Make sure the mode is valid */
	if (iommu->ddt_mode < RISCV_IOMMU_DDTP_MODE_1LVL ||
	    iommu->ddt_mode > RISCV_IOMMU_DDTP_MODE_3LVL)
		return NULL;

	/*
	 * Device id partitioning for base format:
	 * DDI[0]: bits 0 - 6   (1st level) (7 bits)
	 * DDI[1]: bits 7 - 15  (2nd level) (9 bits)
	 * DDI[2]: bits 16 - 23 (3rd level) (8 bits)
	 *
	 * For extended format:
	 * DDI[0]: bits 0 - 5   (1st level) (6 bits)
	 * DDI[1]: bits 6 - 14  (2nd level) (9 bits)
	 * DDI[2]: bits 15 - 23 (3rd level) (9 bits)
	 */
	if (base_format) {
		ddi_bits[0] = 7;
		ddi_bits[1] = 7 + 9;
		ddi_bits[2] = 7 + 9 + 8;
	} else {
		ddi_bits[0] = 6;
		ddi_bits[1] = 6 + 9;
		ddi_bits[2] = 6 + 9 + 9;
	}

	/* Make sure device id is within range */
	depth = iommu->ddt_mode - RISCV_IOMMU_DDTP_MODE_1LVL;
	if (devid >= (1 << ddi_bits[depth]))
		return NULL;

	/* Get to the level of the non-leaf node that holds the device context */
	for (ddtp = iommu->ddt_root; depth-- > 0;) {
		const int split = ddi_bits[depth];
		/*
		 * Each non-leaf node is 64bits wide and on each level
		 * nodes are indexed by DDI[depth].
		 */
		ddtp += (devid >> split) & 0x1FF;

		/*
		 * Check if this node has been populated and if not
		 * allocate a new level and populate it.
		 */
		do {
			ddt = READ_ONCE(*(unsigned long *)ddtp);
			if (ddt & RISCV_IOMMU_DDTE_VALID) {
				ddtp = __va(ppn_to_phys(ddt));
				break;
			}

			ptr = riscv_iommu_get_pages(iommu, 0);
			if (!ptr)
				return NULL;

			new = phys_to_ppn(__pa(ptr)) | RISCV_IOMMU_DDTE_VALID;
			old = cmpxchg_relaxed((unsigned long *)ddtp, ddt, new);

			if (old == ddt) {
				ddtp = (u64 *)ptr;
				break;
			}

			/* Race setting DDT detected, re-read and retry. */
			riscv_iommu_free_pages(iommu, ptr);
		} while (1);
	}

	/*
	 * Grab the node that matches DDI[depth], note that when using base
	 * format the device context is 4 * 64bits, and the extended format
	 * is 8 * 64bits, hence the (3 - base_format) below.
	 */
	ddtp += (devid & ((64 << base_format) - 1)) << (3 - base_format);

	return (struct riscv_iommu_dc *)ddtp;
}

/*
 * Discover supported DDT modes starting from requested value,
 * configure DDTP register with accepted mode and root DDT address.
 * Accepted iommu->ddt_mode is updated on success.
 */
static int riscv_iommu_set_ddtp_mode(struct riscv_iommu_device *iommu,
				     unsigned int ddtp_mode)
{
	struct device *dev = iommu->dev;
	struct riscv_iommu_command cmd;
	u64 ddtp, rq_ddtp;
	unsigned int mode, rq_mode = ddtp_mode;
	int rc;

	rc = readq_relaxed_poll_timeout(iommu->reg + RISCV_IOMMU_REG_DDTP,
					ddtp, !(ddtp & RISCV_IOMMU_DDTP_BUSY),
					10, RISCV_IOMMU_DDTP_TIMEOUT);
	if (rc < 0)
		return -EBUSY;

	/* Disallow state transtion from xLVL to xLVL. */
	switch (FIELD_GET(RISCV_IOMMU_DDTP_MODE, ddtp)) {
	case RISCV_IOMMU_DDTP_MODE_BARE:
	case RISCV_IOMMU_DDTP_MODE_OFF:
		break;
	default:
		if (rq_mode != RISCV_IOMMU_DDTP_MODE_BARE &&
		    rq_mode != RISCV_IOMMU_DDTP_MODE_OFF)
			return -EINVAL;
		break;
	}

	do {
		rq_ddtp = FIELD_PREP(RISCV_IOMMU_DDTP_MODE, rq_mode);
		if (rq_mode > RISCV_IOMMU_DDTP_MODE_BARE)
			rq_ddtp |= phys_to_ppn(iommu->ddt_phys);

		riscv_iommu_writeq(iommu, RISCV_IOMMU_REG_DDTP, rq_ddtp);

		rc = readq_relaxed_poll_timeout(iommu->reg + RISCV_IOMMU_REG_DDTP,
						ddtp, !(ddtp & RISCV_IOMMU_DDTP_BUSY),
						10, RISCV_IOMMU_DDTP_TIMEOUT);
		if (rc < 0) {
			dev_warn(dev, "timeout when setting ddtp (ddt mode: %u, read: %llx)\n",
				 rq_mode, ddtp);
			return -EBUSY;
		}

		/* Verify IOMMU hardware accepts new DDTP config. */
		mode = FIELD_GET(RISCV_IOMMU_DDTP_MODE, ddtp);

		if (rq_mode == mode)
			break;

		/* Hardware mandatory DDTP mode has not been accepted. */
		if (rq_mode < RISCV_IOMMU_DDTP_MODE_1LVL && rq_ddtp != ddtp) {
			dev_warn(dev, "DDTP update failed hw: %llx vs %llx\n", ddtp, rq_ddtp);
			return -EINVAL;
		}

		/*
		 * Mode field is WARL, an IOMMU may support a subset of
		 * directory table levels in which case if we tried to set
		 * an unsupported number of levels we'll readback either
		 * a valid xLVL or off/bare. If we got off/bare, try again
		 * with a smaller xLVL.
		 */
		if (mode < RISCV_IOMMU_DDTP_MODE_1LVL &&
		    rq_mode > RISCV_IOMMU_DDTP_MODE_1LVL) {
			dev_dbg(dev, "DDTP hw mode %u vs %u\n", mode, rq_mode);
			rq_mode--;
			continue;
		}

		/*
		 * We tried all supported modes and IOMMU hardware failed to
		 * accept new settings, something went very wrong since off/bare
		 * and at least one xLVL must be supported.
		 */
		dev_warn(dev, "DDTP hw mode %u, failed to set %u\n", mode, ddtp_mode);
		return -EINVAL;
	} while (1);

	iommu->ddt_mode = mode;
	if (mode != ddtp_mode)
		dev_warn(dev, "DDTP failover to %u mode, requested %u\n",
			 mode, ddtp_mode);

	return 0;
}

static int riscv_iommu_ddt_alloc(struct riscv_iommu_device *iommu)
{
	u64 ddtp;
	unsigned int mode;

	riscv_iommu_readq_timeout(iommu, RISCV_IOMMU_REG_DDTP,
				  ddtp, !(ddtp & RISCV_IOMMU_DDTP_BUSY),
				  10, RISCV_IOMMU_DDTP_TIMEOUT);

	if (ddtp & RISCV_IOMMU_DDTP_BUSY)
		return -EBUSY;

	/*
	 * It is optional for the hardware to report a fixed address for device
	 * directory root page when DDT.MODE is OFF or BARE.
	 */
	mode = FIELD_GET(RISCV_IOMMU_DDTP_MODE, ddtp);
	if (mode != RISCV_IOMMU_DDTP_MODE_BARE && mode != RISCV_IOMMU_DDTP_MODE_OFF) {
		/* Use WARL to discover hardware fixed DDT PPN */
		riscv_iommu_writeq(iommu, RISCV_IOMMU_REG_DDTP,
				   FIELD_PREP(RISCV_IOMMU_DDTP_MODE, mode));
		riscv_iommu_readl_timeout(iommu, RISCV_IOMMU_REG_DDTP,
					  ddtp, !(ddtp & RISCV_IOMMU_DDTP_BUSY),
					  10, RISCV_IOMMU_DDTP_TIMEOUT);
		if (ddtp & RISCV_IOMMU_DDTP_BUSY)
			return -EBUSY;

		iommu->ddt_phys = ppn_to_phys(ddtp);
		if (iommu->ddt_phys)
			iommu->ddt_root = devm_ioremap(iommu->dev, iommu->ddt_phys, PAGE_SIZE);
		if (iommu->ddt_root)
			memset(iommu->ddt_root, 0, PAGE_SIZE);
	}

	if (!iommu->ddt_root) {
		iommu->ddt_root = (u64 *)riscv_iommu_get_pages(iommu, 0);
		iommu->ddt_phys = __pa(iommu->ddt_root);
	}

	if (!iommu->ddt_root)
		return -ENOMEM;

	return 0;
}

// lockdep_assert_held(&group->mutex);
static int riscv_iommu_attach_domain(struct device *dev,
				     struct iommu_domain *domain)
{
	struct riscv_iommu_device *iommu = dev_to_iommu(dev);
	struct riscv_iommu_endpoint *ep = dev_iommu_priv_get(dev);
	const bool was_attached = ep->attached;
	struct riscv_iommu_dc *dc;
	u64 atp, ta, tc;

	if (!domain && !was_attached)
		return 0;

	/* Find DC for the endpoint */
	dc = riscv_iommu_get_dc(iommu, ep->devid);
	if (!dc)
		return -ENODEV;

	if (!domain) {
		WRITE_ONCE(dc->tc, 0);
		ep->attached = false;
	} else {
		ta = 0;
		if (domain->type == IOMMU_DOMAIN_IDENTITY)
			atp = FIELD_PREP(RISCV_IOMMU_DC_FSC_MODE, RISCV_IOMMU_DC_FSC_MODE_BARE);
		else
			return -ENODEV;
		WRITE_ONCE(dc->fsc, atp);
		/* Prevent incomplete PC state being observable */
		smp_wmb();
		WRITE_ONCE(dc->ta, ta);
	}

	/* Transition from NOT attached -> attached */
	if (!was_attached) {
		/* Configure 2nd stage translation to identity mapping */
		dc->iohgatp = FIELD_PREP(RISCV_IOMMU_DC_IOHGATP_MODE,
					 RISCV_IOMMU_DC_IOHGATP_MODE_BARE);

		/* Configure translation context. */
		tc = RISCV_IOMMU_DC_TC_V;

		/* Prevent incomplete DC state being observable */
		smp_wmb();
		WRITE_ONCE(dc->tc, tc);

		/* mark as attached */
		ep->attached = true;
	}

	return 0;
}

static int riscv_iommu_attach_identity_domain(struct iommu_domain *domain,
					      struct device *dev)
{
	struct riscv_iommu_device *iommu = dev_to_iommu(dev);

	/* Global pass-through already enabled, do nothing. */
	if (iommu->ddt_mode == RISCV_IOMMU_DDTP_MODE_BARE)
		return 0;

	return riscv_iommu_attach_domain(dev, domain);
}

static struct iommu_domain riscv_iommu_identity_domain = {
	.type = IOMMU_DOMAIN_IDENTITY,
	.ops = &(const struct iommu_domain_ops) {
		.attach_dev = riscv_iommu_attach_identity_domain,
	}
};

static int riscv_iommu_device_domain_type(struct device *dev)
{
	return IOMMU_DOMAIN_IDENTITY;
}

static struct iommu_group *riscv_iommu_device_group(struct device *dev)
{
	if (dev_is_pci(dev))
		return pci_device_group(dev);
	return generic_device_group(dev);
}

static int riscv_iommu_of_xlate(struct device *dev, struct of_phandle_args *args)
{
	return iommu_fwspec_add_ids(dev, args->args, 1);
}

static struct iommu_device *riscv_iommu_probe_device(struct device *dev)
{
	struct iommu_fwspec *fwspec = dev_iommu_fwspec_get(dev);
	struct riscv_iommu_device *iommu;
	struct pci_dev *pdev = dev_is_pci(dev) ? to_pci_dev(dev) : NULL;
	struct riscv_iommu_endpoint *ep;
	unsigned int devid;

	/* Early bus-scan exit, will retry. */
	if (!fwspec || !fwspec->iommu_fwnode)
		return ERR_PTR(-ENODEV);

	/* Pending IOMMU driver initialization, will retry */
	if (!fwspec->iommu_fwnode->dev)
		return ERR_PTR(-ENODEV);

	iommu = dev_get_drvdata(fwspec->iommu_fwnode->dev);
	if (!iommu)
		return ERR_PTR(-ENODEV);

	if (pdev)
		devid = pci_dev_id(pdev) | pci_domain_nr(pdev->bus) << 16;
	else if (fwspec->num_ids)
		devid = fwspec->ids[0];
	else
		return ERR_PTR(-ENODEV);

	ep = kzalloc(sizeof(*ep), GFP_KERNEL_ACCOUNT);
	if (!ep)
		return ERR_PTR(-ENOMEM);

	ep->devid = devid;
	ep->dev = dev;

	dev_iommu_priv_set(dev, ep);

	return &iommu->iommu;
}

static void riscv_iommu_probe_finalize(struct device *dev)
{
	iommu_setup_dma_ops(dev, 0, U64_MAX);
}

static void riscv_iommu_release_device(struct device *dev)
{
	struct riscv_iommu_endpoint *ep = dev_iommu_priv_get(dev);

	riscv_iommu_attach_domain(dev, NULL);
	dev_iommu_priv_set(dev, NULL);
	kfree(ep);
}

static const struct iommu_ops riscv_iommu_ops = {
	.owner = THIS_MODULE,
	.pgsize_bitmap = SZ_4K | SZ_2M | SZ_1G,
	.of_xlate = riscv_iommu_of_xlate,
	.identity_domain = &riscv_iommu_identity_domain,
	.def_domain_type = riscv_iommu_device_domain_type,
	.device_group = riscv_iommu_device_group,
	.probe_device = riscv_iommu_probe_device,
	.probe_finalize = riscv_iommu_probe_finalize,
	.release_device = riscv_iommu_release_device,
};

void riscv_iommu_remove(struct riscv_iommu_device *iommu)
{
	riscv_iommu_set_ddtp_mode(iommu, RISCV_IOMMU_DDTP_MODE_OFF);
	iommu_device_unregister(&iommu->iommu);
	iommu_device_sysfs_remove(&iommu->iommu);
	riscv_iommu_debugfs_remove(iommu);
}

static int riscv_iommu_init_check(struct riscv_iommu_device *iommu)
{
	u64 ddtp;

	/* Hardware must be configured in OFF | BARE mode at system initialization. */
	riscv_iommu_readq_timeout(iommu, RISCV_IOMMU_REG_DDTP,
				  ddtp, !(ddtp & RISCV_IOMMU_DDTP_BUSY),
				  10, RISCV_IOMMU_DDTP_TIMEOUT);
	if (FIELD_GET(RISCV_IOMMU_DDTP_MODE, ddtp) > RISCV_IOMMU_DDTP_MODE_BARE)
		return -EBUSY;

	/* Configure accesses to in-memory data structures for CPU-native byte order. */
	if (IS_ENABLED(CONFIG_CPU_BIG_ENDIAN) != !!(iommu->fctl & RISCV_IOMMU_FCTL_BE)) {
		if (!(iommu->caps & RISCV_IOMMU_CAP_END))
			return -EINVAL;
		riscv_iommu_writel(iommu, RISCV_IOMMU_REG_FCTL,
				   iommu->fctl ^ RISCV_IOMMU_FCTL_BE);
		iommu->fctl = riscv_iommu_readl(iommu, RISCV_IOMMU_REG_FCTL);
		if (IS_ENABLED(CONFIG_CPU_BIG_ENDIAN) != !!(iommu->fctl & RISCV_IOMMU_FCTL_BE))
			return -EINVAL;
	}

	dma_set_mask_and_coherent(iommu->dev,
				  DMA_BIT_MASK(FIELD_GET(RISCV_IOMMU_CAP_PAS, iommu->caps)));

	return 0;
}

int riscv_iommu_init(struct riscv_iommu_device *iommu)
{
	int rc;

	rc = riscv_iommu_init_check(iommu);
	if (rc)
		return dev_err_probe(iommu->dev, rc, "unexpected device state\n");

	riscv_iommu_debugfs_setup(iommu);

	rc = riscv_iommu_ddt_alloc(iommu);
	if (WARN(rc, "cannot allocate device directory\n"))
		goto err_init;

	rc = riscv_iommu_set_ddtp_mode(iommu, RISCV_IOMMU_DDTP_MODE_MAX);
	if (WARN(rc, "cannot enable iommu device\n"))
		goto err_init;

	rc = iommu_device_sysfs_add(&iommu->iommu, NULL, NULL, "riscv-iommu@%s",
				    dev_name(iommu->dev));
	if (WARN(rc, "cannot register sysfs interface\n"))
		goto err_sysfs;

	rc = iommu_device_register(&iommu->iommu, &riscv_iommu_ops, iommu->dev);
	if (WARN(rc, "cannot register iommu interface\n"))
		goto err_iommu;

	return 0;

err_iommu:
	iommu_device_sysfs_remove(&iommu->iommu);
err_sysfs:
	riscv_iommu_set_ddtp_mode(iommu, RISCV_IOMMU_DDTP_MODE_OFF);
err_init:
	riscv_iommu_debugfs_remove(iommu);
	return rc;
}
