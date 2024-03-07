// SPDX-License-Identifier: GPL-2.0-only
/*
 * RISC-V IOMMU as a platform device
 *
 * Copyright © 2023 FORTH-ICS/CARV
 * Copyright © 2023 Rivos Inc.
 *
 * Authors
 *	Nick Kossifidis <mick@ics.forth.gr>
 *	Tomasz Jeznach <tjeznach@rivosinc.com>
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>

#include "iommu-bits.h"
#include "iommu.h"

static int riscv_iommu_platform_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct riscv_iommu_device *iommu = NULL;
	struct resource *res = NULL;
	int vec;

	iommu = devm_kzalloc(dev, sizeof(*iommu), GFP_KERNEL);
	if (!iommu)
		return -ENOMEM;

	iommu->dev = dev;
	iommu->reg = devm_platform_get_and_ioremap_resource(pdev, 0, &res);
	if (IS_ERR(iommu->reg))
		return dev_err_probe(dev, PTR_ERR(iommu->reg),
				     "could not map register region\n");

	dev_set_drvdata(dev, iommu);

	/* Check device reported capabilities / features. */
	iommu->caps = riscv_iommu_readq(iommu, RISCV_IOMMU_REG_CAP);
	iommu->fctl = riscv_iommu_readl(iommu, RISCV_IOMMU_REG_FCTL);

	/* For now we only support WSI */
	switch (FIELD_GET(RISCV_IOMMU_CAP_IGS, iommu->caps)) {
	case RISCV_IOMMU_CAP_IGS_WSI:
	case RISCV_IOMMU_CAP_IGS_BOTH:
		break;
	default:
		return dev_err_probe(dev, -ENODEV,
				     "unable to use wire-signaled interrupts\n");
	}

	iommu->irqs_count = platform_irq_count(pdev);
	if (iommu->irqs_count <= 0)
		return dev_err_probe(dev, -ENODEV,
				     "no IRQ resources provided\n");

	for (vec = 0; vec < iommu->irqs_count; vec++)
		iommu->irqs[vec] = platform_get_irq(pdev, vec);

	/* Enable wire-signaled interrupts, fctl.WSI */
	if (!(iommu->fctl & RISCV_IOMMU_FCTL_WSI)) {
		iommu->fctl ^= RISCV_IOMMU_FCTL_WSI;
		riscv_iommu_writel(iommu, RISCV_IOMMU_REG_FCTL, iommu->fctl);
	}

	return riscv_iommu_init(iommu);
};

static void riscv_iommu_platform_remove(struct platform_device *pdev)
{
	riscv_iommu_remove(dev_get_drvdata(&pdev->dev));
};

static void riscv_iommu_platform_shutdown(struct platform_device *pdev)
{
	return;
};

static const struct of_device_id riscv_iommu_of_match[] = {
	{.compatible = "riscv,iommu",},
	{},
};

MODULE_DEVICE_TABLE(of, riscv_iommu_of_match);

static struct platform_driver riscv_iommu_platform_driver = {
	.driver = {
		   .name = "riscv,iommu",
		   .of_match_table = riscv_iommu_of_match,
		   .suppress_bind_attrs = true,
		   },
	.probe = riscv_iommu_platform_probe,
	.remove_new = riscv_iommu_platform_remove,
	.shutdown = riscv_iommu_platform_shutdown,
};

module_driver(riscv_iommu_platform_driver, platform_driver_register,
	      platform_driver_unregister);
