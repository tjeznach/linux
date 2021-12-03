#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/compiler.h>
#include <linux/pci.h>
#include <linux/init.h>
#include <linux/iommu.h>

#include "iommu.h"

#define DRV_NAME       "riscv-iommu-pci"
#define DRV_VERSION    "0.0.9"

/* Rivos Inc. assigned PCI Vendor and Device IDs */
#ifndef PCI_VENDOR_ID_RIVOS
#define PCI_VENDOR_ID_RIVOS             0x1efd
#endif

#ifndef PCI_DEVICE_ID_RIVOS_IOMMU
#define PCI_DEVICE_ID_RIVOS_IOMMU       0xedf1
#endif

/* RISCV IOMMU as a PCIe device */

static int riscv_iommu_pci_iomap_probe(struct pci_dev *pdev)
{
	phys_addr_t reg_phys;
	size_t reg_size;
	int ret;

	ret = pci_request_mem_regions(pdev, DRV_NAME);
	if (ret < 0)
		return ret;

	if (!(pci_resource_flags(pdev, 0) & IORESOURCE_MEM))
		return -ENODEV;

	reg_size = pci_resource_len(pdev, 0);
	if (reg_size < RIO_REG_SIZE)
		return -ENODEV;

	reg_phys = pci_resource_start(pdev, 0);
	if (!reg_phys)
		return -ENODEV;

	ret = pci_alloc_irq_vectors(pdev, 1, RIO_INT_COUNT, PCI_IRQ_MSIX);
	if (ret < 0)
		return ret;

	return riscv_iommu_probe(&pdev->dev, reg_phys, reg_size);
}

static int riscv_iommu_pci_probe(struct pci_dev *pdev,
				 const struct pci_device_id *ent)
{
	int ret;

	ret = pci_enable_device_mem(pdev);
	if (ret < 0)
		return ret;

	pci_set_master(pdev);

	ret = riscv_iommu_pci_iomap_probe(pdev);
	if (ret < 0) {
		pci_free_irq_vectors(pdev);
		pci_clear_master(pdev);
		pci_release_regions(pdev);
		pci_disable_device(pdev);
		return ret;
	}

	return 0;
}

static void riscv_iommu_pci_remove(struct pci_dev *pdev)
{
	riscv_iommu_remove(&pdev->dev);
	pci_free_irq_vectors(pdev);
	pci_clear_master(pdev);
	pci_release_regions(pdev);
	pci_disable_device(pdev);
}

static int riscv_iommu_suspend(struct device *dev)
{
	/* TODO: Silence IOMMU translations. */
	return 0;
}

static int riscv_iommu_resume(struct device *dev)
{
	/* TODO: Restore IOMMU state. */
	return 0;
}

static DEFINE_SIMPLE_DEV_PM_OPS(riscv_iommu_pm_ops,
				riscv_iommu_suspend, riscv_iommu_resume);

static const struct pci_device_id riscv_iommu_pci_tbl[] = {
	{PCI_VENDOR_ID_RIVOS, PCI_DEVICE_ID_RIVOS_IOMMU,
	 PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0},
	{0,}
};

MODULE_DEVICE_TABLE(pci, riscv_iommu_pci_tbl);

static const struct of_device_id riscv_iommu_of_match[] = {
	{.compatible = "riscv,pci-iommu",},
	{},
};

MODULE_DEVICE_TABLE(of, riscv_iommu_of_match);

static struct pci_driver riscv_iommu_pci_driver = {
	.name = DRV_NAME,
	.id_table = riscv_iommu_pci_tbl,
	.probe = riscv_iommu_pci_probe,
	.remove = riscv_iommu_pci_remove,
	.driver = {
		   .pm = pm_sleep_ptr(&riscv_iommu_pm_ops),
		   .of_match_table = riscv_iommu_of_match,
		   },
};

static int __init riscv_iommu_init_module(void)
{
	return pci_register_driver(&riscv_iommu_pci_driver);
}

static void __exit riscv_iommu_cleanup_module(void)
{
	pci_unregister_driver(&riscv_iommu_pci_driver);
}

module_init(riscv_iommu_init_module);
module_exit(riscv_iommu_cleanup_module);
