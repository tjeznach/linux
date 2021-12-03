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
#include <linux/dma-mapping.h>
#include <linux/init.h>
#include <linux/iommu.h>
#include <linux/kernel.h>
#include <linux/module.h>

#include "iommu-bits.h"
#include "iommu.h"

MODULE_DESCRIPTION("Driver for RISC-V IOMMU");
MODULE_AUTHOR("Tomasz Jeznach <tjeznach@rivosinc.com>");
MODULE_AUTHOR("Nick Kossifidis <mick@ics.forth.gr>");
MODULE_ALIAS("riscv-iommu");
MODULE_LICENSE("GPL v2");

/* Timeouts in [us] */
#define RISCV_IOMMU_DDTP_TIMEOUT       50000

void riscv_iommu_remove(struct riscv_iommu_device *iommu)
{
	iommu_device_sysfs_remove(&iommu->iommu);
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
	/*
	 * Placeholder for a complete IOMMU device initialization.
	 * For now, only bare minimum: enable global identity mapping mode and register sysfs.
	 */
	riscv_iommu_writeq(iommu, RISCV_IOMMU_REG_DDTP,
			   FIELD_PREP(RISCV_IOMMU_DDTP_MODE, RISCV_IOMMU_DDTP_MODE_BARE));

	rc = iommu_device_sysfs_add(&iommu->iommu, NULL, NULL, "riscv-iommu@%s",
				    dev_name(iommu->dev));
	if (WARN(rc, "cannot register sysfs interface\n"))
		goto err_sysfs;

	return 0;

err_sysfs:
	return rc;
}
