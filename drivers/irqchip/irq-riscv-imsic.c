// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2021 Western Digital Corporation or its affiliates.
 * Copyright (C) 2022 Ventana Micro Systems Inc.
 */

#define pr_fmt(fmt) "riscv-imsic: " fmt
#include <linux/bitmap.h>
#include <linux/cpu.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iommu.h>
#include <linux/irq.h>
#include <linux/irqchip.h>
#include <linux/irqchip/chained_irq.h>
#include <linux/irqchip/riscv-imsic.h>
#include <linux/irqdomain.h>
#include <linux/module.h>
#include <linux/msi.h>
#include <linux/of_address.h>
#include <linux/pci.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>
#include <linux/smp.h>
#include <asm/hwcap.h>

#define IMSIC_DISABLE_EIDELIVERY		0
#define IMSIC_ENABLE_EIDELIVERY			1
#define IMSIC_DISABLE_EITHRESHOLD		1
#define IMSIC_ENABLE_EITHRESHOLD		0

/*
 * The IMSIC driver uses 1 IPI for ID synchronization and
 * arch/riscv/kernel/smp.c require 6 IPIs so we fix the
 * total number of IPIs to 8.
 */
#define IMSIC_NR_IPI				8

#define imsic_csr_write(__c, __v)		\
do {						\
	csr_write(CSR_ISELECT, __c);		\
	csr_write(CSR_IREG, __v);		\
} while (0)

#define imsic_csr_read(__c)			\
({						\
	unsigned long __v;			\
	csr_write(CSR_ISELECT, __c);		\
	__v = csr_read(CSR_IREG);		\
	__v;					\
})

#define imsic_csr_set(__c, __v)			\
do {						\
	csr_write(CSR_ISELECT, __c);		\
	csr_set(CSR_IREG, __v);			\
} while (0)

#define imsic_csr_clear(__c, __v)		\
do {						\
	csr_write(CSR_ISELECT, __c);		\
	csr_clear(CSR_IREG, __v);		\
} while (0)

struct imsic_priv {
	/* Global configuration common for all HARTs */
	struct imsic_global_config global;

	/* Global state of interrupt identities */
	raw_spinlock_t ids_lock;
	unsigned long *ids_used_bimap;
	unsigned long *ids_enabled_bimap;
	unsigned int *ids_target_cpu;

	/* Mask for connected CPUs */
	struct cpumask lmask;

	/* IPI interrupt identity and synchronization */
	u32 ipi_id;
	int ipi_virq;
	struct irq_desc *ipi_lsync_desc;

	/* IRQ domains */
	struct irq_domain *base_domain;
	struct irq_domain *pci_domain;
	struct irq_domain *plat_domain;
};

static struct imsic_priv *imsic;
static int imsic_parent_irq;

const struct imsic_global_config *imsic_get_global_config(void)
{
	return (imsic) ? &imsic->global : NULL;
}
EXPORT_SYMBOL_GPL(imsic_get_global_config);

static int imsic_cpu_page_phys(unsigned int cpu,
			       unsigned int guest_index,
			       phys_addr_t *out_msi_pa)
{
	struct imsic_global_config *global;
	struct imsic_local_config *local;

	global = &imsic->global;
	local = per_cpu_ptr(global->local, cpu);

	if (BIT(global->guest_index_bits) <= guest_index)
		return -EINVAL;

	if (out_msi_pa)
		*out_msi_pa = local->msi_pa +
			      (guest_index * IMSIC_MMIO_PAGE_SZ);

	return 0;
}

static int imsic_get_cpu(const struct cpumask *mask_val, bool force,
			 unsigned int *out_target_cpu)
{
	struct cpumask amask;
	unsigned int cpu;

	cpumask_and(&amask, &imsic->lmask, mask_val);

	if (force)
		cpu = cpumask_first(&amask);
	else
		cpu = cpumask_any_and(&amask, cpu_online_mask);

	if (cpu >= nr_cpu_ids)
		return -EINVAL;

	if (out_target_cpu)
		*out_target_cpu = cpu;

	return 0;
}

static void imsic_id_set_target(unsigned int id, unsigned int target_cpu)
{
	unsigned long flags;

	raw_spin_lock_irqsave(&imsic->ids_lock, flags);
	imsic->ids_target_cpu[id] = target_cpu;
	raw_spin_unlock_irqrestore(&imsic->ids_lock, flags);
}

static unsigned int imsic_id_get_target(unsigned int id)
{
	unsigned int ret;
	unsigned long flags;

	raw_spin_lock_irqsave(&imsic->ids_lock, flags);
	ret = imsic->ids_target_cpu[id];
	raw_spin_unlock_irqrestore(&imsic->ids_lock, flags);

	return ret;
}

static void __imsic_eix_update(unsigned long base_id,
			       unsigned long num_id, bool pend, bool val)
{
	unsigned long i, isel, ireg;
	unsigned long id = base_id, last_id = base_id + num_id;

	while (id < last_id) {
		isel = id / BITS_PER_LONG;
		isel *= BITS_PER_LONG / IMSIC_EIPx_BITS;
		isel += (pend) ? IMSIC_EIP0 : IMSIC_EIE0;

		ireg = 0;
		for (i = id & (__riscv_xlen - 1);
		     (id < last_id) && (i < __riscv_xlen); i++) {
			ireg |= BIT(i);
			id++;
		}

		/*
		 * The IMSIC EIEx and EIPx registers are indirectly
		 * accessed via using ISELECT and IREG CSRs so we
		 * need to access these CSRs without getting preempted.
		 *
		 * All existing users of this function call this
		 * function with local IRQs disabled so we don't
		 * need to do anything special here.
		 */
		if (val)
			imsic_csr_set(isel, ireg);
		else
			imsic_csr_clear(isel, ireg);
	}
}

#define __imsic_id_enable(__id)		\
	__imsic_eix_update((__id), 1, false, true)
#define __imsic_id_disable(__id)	\
	__imsic_eix_update((__id), 1, false, false)

static void imsic_ids_local_sync(void)
{
	int i;
	unsigned long flags;

	raw_spin_lock_irqsave(&imsic->ids_lock, flags);
	for (i = 1; i <= imsic->global.nr_ids; i++) {
		if (imsic->ipi_id == i)
			continue;

		if (test_bit(i, imsic->ids_enabled_bimap))
			__imsic_id_enable(i);
		else
			__imsic_id_disable(i);
	}
	raw_spin_unlock_irqrestore(&imsic->ids_lock, flags);
}

static void imsic_ids_local_delivery(bool enable)
{
	if (enable) {
		imsic_csr_write(IMSIC_EITHRESHOLD, IMSIC_ENABLE_EITHRESHOLD);
		imsic_csr_write(IMSIC_EIDELIVERY, IMSIC_ENABLE_EIDELIVERY);
	} else {
		imsic_csr_write(IMSIC_EIDELIVERY, IMSIC_DISABLE_EIDELIVERY);
		imsic_csr_write(IMSIC_EITHRESHOLD, IMSIC_DISABLE_EITHRESHOLD);
	}
}

#ifdef CONFIG_SMP
static irqreturn_t imsic_ids_sync_handler(int irq, void *data)
{
	imsic_ids_local_sync();
	return IRQ_HANDLED;
}

static void imsic_ids_remote_sync(void)
{
	struct cpumask amask;

	/*
	 * We simply inject ID synchronization IPI to all target CPUs
	 * except current CPU. The ipi_send_mask() implementation of
	 * IPI mux will inject ID synchronization IPI only for CPUs
	 * that have enabled it so offline CPUs won't receive IPI.
	 * An offline CPU will unconditionally synchronize IDs through
	 * imsic_starting_cpu() when the CPU is brought up.
	 */
	cpumask_andnot(&amask, &imsic->lmask, cpumask_of(smp_processor_id()));
	__ipi_send_mask(imsic->ipi_lsync_desc, &amask);
}
#else
#define imsic_ids_remote_sync()
#endif

static int imsic_ids_alloc(unsigned int order)
{
	int ret;
	unsigned long flags;

	raw_spin_lock_irqsave(&imsic->ids_lock, flags);
	ret = bitmap_find_free_region(imsic->ids_used_bimap,
				      imsic->global.nr_ids + 1, order);
	raw_spin_unlock_irqrestore(&imsic->ids_lock, flags);

	return ret;
}

static void imsic_ids_free(unsigned int base_id, unsigned int order)
{
	unsigned long flags;

	raw_spin_lock_irqsave(&imsic->ids_lock, flags);
	bitmap_release_region(imsic->ids_used_bimap, base_id, order);
	raw_spin_unlock_irqrestore(&imsic->ids_lock, flags);
}

static int __init imsic_ids_init(void)
{
	int i;
	struct imsic_global_config *global = &imsic->global;

	raw_spin_lock_init(&imsic->ids_lock);

	/* Allocate used bitmap */
	imsic->ids_used_bimap = bitmap_zalloc(global->nr_ids + 1, GFP_KERNEL);
	if (!imsic->ids_used_bimap)
		return -ENOMEM;

	/* Allocate enabled bitmap */
	imsic->ids_enabled_bimap = bitmap_zalloc(global->nr_ids + 1,
						GFP_KERNEL);
	if (!imsic->ids_enabled_bimap) {
		kfree(imsic->ids_used_bimap);
		return -ENOMEM;
	}

	/* Allocate target CPU array */
	imsic->ids_target_cpu = kcalloc(global->nr_ids + 1,
				       sizeof(unsigned int), GFP_KERNEL);
	if (!imsic->ids_target_cpu) {
		bitmap_free(imsic->ids_enabled_bimap);
		bitmap_free(imsic->ids_used_bimap);
		return -ENOMEM;
	}
	for (i = 0; i <= global->nr_ids; i++)
		imsic->ids_target_cpu[i] = UINT_MAX;

	/* Reserve ID#0 because it is special and never implemented */
	bitmap_set(imsic->ids_used_bimap, 0, 1);

	return 0;
}

static void __init imsic_ids_cleanup(void)
{
	kfree(imsic->ids_target_cpu);
	bitmap_free(imsic->ids_enabled_bimap);
	bitmap_free(imsic->ids_used_bimap);
}

#ifdef CONFIG_SMP
static void imsic_ipi_send(unsigned int cpu)
{
	struct imsic_local_config *local =
				per_cpu_ptr(imsic->global.local, cpu);

	writel(imsic->ipi_id, local->msi_va);
}

static void imsic_ipi_starting_cpu(void)
{
	/* Enable IPIs for current CPU. */
	__imsic_id_enable(imsic->ipi_id);

	/* Enable virtual IPI used for IMSIC ID synchronization */
	enable_percpu_irq(imsic->ipi_virq, 0);
}

static void imsic_ipi_dying_cpu(void)
{
	/*
	 * Disable virtual IPI used for IMSIC ID synchronization so
	 * that we don't receive ID synchronization requests.
	 */
	disable_percpu_irq(imsic->ipi_virq);
}

static int __init imsic_ipi_domain_init(void)
{
	int virq;

	/* Allocate interrupt identity for IPIs */
	virq = imsic_ids_alloc(get_count_order(1));
	if (virq < 0)
		return virq;
	imsic->ipi_id = virq;

	/* Create IMSIC IPI multiplexing */
	virq = ipi_mux_create(IMSIC_NR_IPI, imsic_ipi_send);
	if (virq <= 0) {
		imsic_ids_free(imsic->ipi_id, get_count_order(1));
		return (virq < 0) ? virq : -ENOMEM;
	}
	imsic->ipi_virq = virq;

	/* First vIRQ is used for IMSIC ID synchronization */
	virq = request_percpu_irq(imsic->ipi_virq, imsic_ids_sync_handler,
				  "riscv-imsic-lsync", imsic->global.local);
	if (virq) {
		imsic_ids_free(imsic->ipi_id, get_count_order(1));
		return virq;
	}
	irq_set_status_flags(imsic->ipi_virq, IRQ_HIDDEN);
	imsic->ipi_lsync_desc = irq_to_desc(imsic->ipi_virq);

	/* Set vIRQ range */
	riscv_ipi_set_virq_range(imsic->ipi_virq + 1, IMSIC_NR_IPI - 1, true);

	return 0;
}

static void __init imsic_ipi_domain_cleanup(void)
{
	if (imsic->ipi_lsync_desc)
		free_percpu_irq(imsic->ipi_virq, imsic->global.local);
	imsic_ids_free(imsic->ipi_id, get_count_order(1));
}
#else
static void imsic_ipi_starting_cpu(void)
{
}

static void imsic_ipi_dying_cpu(void)
{
}

static int __init imsic_ipi_domain_init(void)
{
	/* Clear the IPI id because we are not using IPIs */
	imsic->ipi_id = 0;
	return 0;
}

static void __init imsic_ipi_domain_cleanup(void)
{
}
#endif

static void imsic_irq_mask(struct irq_data *d)
{
	unsigned long flags;

	raw_spin_lock_irqsave(&imsic->ids_lock, flags);
	bitmap_clear(imsic->ids_enabled_bimap, d->hwirq, 1);
	__imsic_id_disable(d->hwirq);
	raw_spin_unlock_irqrestore(&imsic->ids_lock, flags);

	imsic_ids_remote_sync();
}

static void imsic_irq_unmask(struct irq_data *d)
{
	unsigned long flags;

	raw_spin_lock_irqsave(&imsic->ids_lock, flags);
	bitmap_set(imsic->ids_enabled_bimap, d->hwirq, 1);
	__imsic_id_enable(d->hwirq);
	raw_spin_unlock_irqrestore(&imsic->ids_lock, flags);

	imsic_ids_remote_sync();
}

static void imsic_irq_compose_msi_msg(struct irq_data *d,
				      struct msi_msg *msg)
{
	struct msi_desc *desc = irq_data_get_msi_desc(d);
	phys_addr_t msi_addr;
	unsigned int cpu;
	int err;

	cpu = imsic_id_get_target(d->hwirq);
	if (WARN_ON(cpu == UINT_MAX))
		return;

	err = imsic_cpu_page_phys(cpu, 0, &msi_addr);
	if (WARN_ON(err))
		return;

	err = iommu_dma_select_msi(desc, msi_addr);
	if (WARN_ON(err))
		return;

	msg->address_hi = upper_32_bits(msi_addr);
	msg->address_lo = lower_32_bits(msi_addr);
	msg->data = d->hwirq;
	iommu_dma_compose_msi_msg(desc, msg);
}

#ifdef CONFIG_SMP
static int imsic_irq_set_affinity(struct irq_data *d,
				  const struct cpumask *mask_val,
				  bool force)
{
	unsigned int target_cpu;
	int rc;

	rc = imsic_get_cpu(mask_val, force, &target_cpu);
	if (rc)
		return rc;

	imsic_id_set_target(d->hwirq, target_cpu);
	irq_data_update_effective_affinity(d, cpumask_of(target_cpu));

	return IRQ_SET_MASK_OK;
}
#endif

static struct irq_chip imsic_irq_base_chip = {
	.name			= "RISC-V IMSIC-BASE",
	.irq_mask		= imsic_irq_mask,
	.irq_unmask		= imsic_irq_unmask,
#ifdef CONFIG_SMP
	.irq_set_affinity	= imsic_irq_set_affinity,
#endif
	.irq_compose_msi_msg	= imsic_irq_compose_msi_msg,
	.flags			= IRQCHIP_SKIP_SET_WAKE |
				  IRQCHIP_MASK_ON_SUSPEND,
};

static int imsic_irq_domain_alloc(struct irq_domain *domain,
				  unsigned int virq,
				  unsigned int nr_irqs,
				  void *args)
{
	msi_alloc_info_t *info = args;
	phys_addr_t msi_addr;
	int i, hwirq, err = 0;
	unsigned int cpu;

	/* Map MSI address of all CPUs */
	for_each_cpu(cpu, &imsic->lmask) {
		err = imsic_cpu_page_phys(cpu, 0, &msi_addr);
		if (err)
			return err;

		err = iommu_dma_prepare_msi(info->desc, msi_addr);
		if (err)
			return err;
	}

	err = imsic_get_cpu(&imsic->lmask, false, &cpu);
	if (err)
		return err;

	hwirq = imsic_ids_alloc(get_count_order(nr_irqs));
	if (hwirq < 0)
		return hwirq;

	for (i = 0; i < nr_irqs; i++) {
		imsic_id_set_target(hwirq + i, cpu);
		irq_domain_set_info(domain, virq + i, hwirq + i,
				    &imsic_irq_base_chip, imsic,
				    handle_simple_irq, NULL, NULL);
		irq_set_noprobe(virq + i);
		irq_set_affinity(virq + i, &imsic->lmask);
		/*
		 * IMSIC does not implement irq_disable() so Linux interrupt
		 * subsystem will take a lazy approach for disabling an IMSIC
		 * interrupt. This means IMSIC interrupts are left unmasked
		 * upon system suspend and interrupts are not processed
		 * immediately upon system wake up. To tackle this, we disable
		 * the lazy approach for all IMSIC interrupts.
		 */
		irq_set_status_flags(virq + i, IRQ_DISABLE_UNLAZY);
	}

	return 0;
}

static void imsic_irq_domain_free(struct irq_domain *domain,
				  unsigned int virq,
				  unsigned int nr_irqs)
{
	struct irq_data *d = irq_domain_get_irq_data(domain, virq);

	imsic_ids_free(d->hwirq, get_count_order(nr_irqs));
	irq_domain_free_irqs_parent(domain, virq, nr_irqs);
}

static const struct irq_domain_ops imsic_base_domain_ops = {
	.alloc		= imsic_irq_domain_alloc,
	.free		= imsic_irq_domain_free,
};

#ifdef CONFIG_RISCV_IMSIC_PCI

static void imsic_pci_mask_irq(struct irq_data *d)
{
	pci_msi_mask_irq(d);
	irq_chip_mask_parent(d);
}

static void imsic_pci_unmask_irq(struct irq_data *d)
{
	pci_msi_unmask_irq(d);
	irq_chip_unmask_parent(d);
}

static struct irq_chip imsic_pci_irq_chip = {
	.name			= "RISC-V IMSIC-PCI",
	.irq_mask		= imsic_pci_mask_irq,
	.irq_unmask		= imsic_pci_unmask_irq,
	.irq_eoi		= irq_chip_eoi_parent,
};

static struct msi_domain_ops imsic_pci_domain_ops = {
};

static struct msi_domain_info imsic_pci_domain_info = {
	.flags	= (MSI_FLAG_USE_DEF_DOM_OPS | MSI_FLAG_USE_DEF_CHIP_OPS |
		   MSI_FLAG_PCI_MSIX | MSI_FLAG_MULTI_PCI_MSI),
	.ops	= &imsic_pci_domain_ops,
	.chip	= &imsic_pci_irq_chip,
};

#endif

static struct irq_chip imsic_plat_irq_chip = {
	.name			= "RISC-V IMSIC-PLAT",
};

static struct msi_domain_ops imsic_plat_domain_ops = {
};

static struct msi_domain_info imsic_plat_domain_info = {
	.flags	= (MSI_FLAG_USE_DEF_DOM_OPS | MSI_FLAG_USE_DEF_CHIP_OPS),
	.ops	= &imsic_plat_domain_ops,
	.chip	= &imsic_plat_irq_chip,
};

static int __init imsic_irq_domains_init(struct fwnode_handle *fwnode)
{
	/* Create Base IRQ domain */
	imsic->base_domain = irq_domain_create_tree(fwnode,
					&imsic_base_domain_ops, imsic);
	if (!imsic->base_domain) {
		pr_err("Failed to create IMSIC base domain\n");
		return -ENOMEM;
	}
	irq_domain_update_bus_token(imsic->base_domain, DOMAIN_BUS_NEXUS);

#ifdef CONFIG_RISCV_IMSIC_PCI
	/* Create PCI MSI domain */
	imsic->pci_domain = pci_msi_create_irq_domain(fwnode,
						&imsic_pci_domain_info,
						imsic->base_domain);
	if (!imsic->pci_domain) {
		pr_err("Failed to create IMSIC PCI domain\n");
		irq_domain_remove(imsic->base_domain);
		return -ENOMEM;
	}
#endif

	/* Create Platform MSI domain */
	imsic->plat_domain = platform_msi_create_irq_domain(fwnode,
						&imsic_plat_domain_info,
						imsic->base_domain);
	if (!imsic->plat_domain) {
		pr_err("Failed to create IMSIC platform domain\n");
		if (imsic->pci_domain)
			irq_domain_remove(imsic->pci_domain);
		irq_domain_remove(imsic->base_domain);
		return -ENOMEM;
	}

	return 0;
}

/*
 * To handle an interrupt, we read the TOPEI CSR and write zero in one
 * instruction. If TOPEI CSR is non-zero then we translate TOPEI.ID to
 * Linux interrupt number and let Linux IRQ subsystem handle it.
 */
static void imsic_handle_irq(struct irq_desc *desc)
{
	struct irq_chip *chip = irq_desc_get_chip(desc);
	irq_hw_number_t hwirq;
	int err;

	chained_irq_enter(chip, desc);

	while ((hwirq = csr_swap(CSR_TOPEI, 0))) {
		hwirq = hwirq >> TOPEI_ID_SHIFT;

		if (hwirq == imsic->ipi_id) {
#ifdef CONFIG_SMP
			ipi_mux_process();
#endif
			continue;
		}

		err = generic_handle_domain_irq(imsic->base_domain, hwirq);
		if (unlikely(err))
			pr_warn_ratelimited(
				"hwirq %lu mapping not found\n", hwirq);
	}

	chained_irq_exit(chip, desc);
}

static int imsic_starting_cpu(unsigned int cpu)
{
	/* Enable per-CPU parent interrupt */
	enable_percpu_irq(imsic_parent_irq,
			  irq_get_trigger_type(imsic_parent_irq));

	/* Setup IPIs */
	imsic_ipi_starting_cpu();

	/*
	 * Interrupts identities might have been enabled/disabled while
	 * this CPU was not running so sync-up local enable/disable state.
	 */
	imsic_ids_local_sync();

	/* Enable local interrupt delivery */
	imsic_ids_local_delivery(true);

	return 0;
}

static int imsic_dying_cpu(unsigned int cpu)
{
	/* Cleanup IPIs */
	imsic_ipi_dying_cpu();

	return 0;
}

static int __init imsic_get_parent_hartid(struct fwnode_handle *fwnode,
					  u32 index, unsigned long *hartid)
{
	int rc;
	struct fwnode_reference_args parent;

	rc = fwnode_property_get_reference_args(fwnode,
			"interrupts-extended", "#interrupt-cells",
			0, index, &parent);
	if (rc)
		return rc;

	/*
	 * Skip interrupts other than external interrupts for
	 * current privilege level.
	 */
	if (parent.args[0] != RV_IRQ_EXT)
		return -EINVAL;

	return riscv_fw_parent_hartid(parent.fwnode, hartid);
}

static int __init imsic_get_mmio_resource(struct fwnode_handle *fwnode,
					  u32 index, struct resource *res)
{
	/*
	 * Currently, only OF fwnode is support so extend this function
	 * for other types of fwnode for ACPI support.
	 */
	if (!is_of_node(fwnode))
		return -EINVAL;
	return of_address_to_resource(to_of_node(fwnode), index, res);
}

static int __init imsic_init(struct fwnode_handle *fwnode)
{
	int rc, cpu;
	phys_addr_t base_addr;
	struct irq_domain *domain;
	void __iomem **mmios_va = NULL;
	struct resource res, *mmios = NULL;
	struct imsic_local_config *local;
	struct imsic_global_config *global;
	unsigned long reloff, hartid;
	u32 i, j, index, nr_parent_irqs, nr_handlers = 0, num_mmios = 0;

	/*
	 * Only one IMSIC instance allowed in a platform for clean
	 * implementation of SMP IRQ affinity and per-CPU IPIs.
	 *
	 * This means on a multi-socket (or multi-die) platform we
	 * will have multiple MMIO regions for one IMSIC instance.
	 */
	if (imsic) {
		pr_err("%pfwP: already initialized hence ignoring\n",
			fwnode);
		return -ENODEV;
	}

	if (!riscv_isa_extension_available(NULL, SxAIA)) {
		pr_err("%pfwP: AIA support not available\n", fwnode);
		return -ENODEV;
	}

	imsic = kzalloc(sizeof(*imsic), GFP_KERNEL);
	if (!imsic)
		return -ENOMEM;
	global = &imsic->global;

	global->local = alloc_percpu(typeof(*(global->local)));
	if (!global->local) {
		rc = -ENOMEM;
		goto out_free_priv;
	}

	/* Find number of parent interrupts */
	nr_parent_irqs = 0;
	while (!imsic_get_parent_hartid(fwnode, nr_parent_irqs, &hartid))
		nr_parent_irqs++;
	if (!nr_parent_irqs) {
		pr_err("%pfwP: no parent irqs available\n", fwnode);
		rc = -EINVAL;
		goto out_free_local;
	}

	/* Find number of guest index bits in MSI address */
	rc = fwnode_property_read_u32_array(fwnode, "riscv,guest-index-bits",
					    &global->guest_index_bits, 1);
	if (rc)
		global->guest_index_bits = 0;
	i = BITS_PER_LONG - IMSIC_MMIO_PAGE_SHIFT;
	if (i < global->guest_index_bits) {
		pr_err("%pfwP: guest index bits too big\n", fwnode);
		rc = -EINVAL;
		goto out_free_local;
	}

	/* Find number of HART index bits */
	rc = fwnode_property_read_u32_array(fwnode, "riscv,hart-index-bits",
					    &global->hart_index_bits, 1);
	if (rc) {
		/* Assume default value */
		global->hart_index_bits = __fls(nr_parent_irqs);
		if (BIT(global->hart_index_bits) < nr_parent_irqs)
			global->hart_index_bits++;
	}
	i = BITS_PER_LONG - IMSIC_MMIO_PAGE_SHIFT - global->guest_index_bits;
	if (i < global->hart_index_bits) {
		pr_err("%pfwP: HART index bits too big\n", fwnode);
		rc = -EINVAL;
		goto out_free_local;
	}

	/* Find number of group index bits */
	rc = fwnode_property_read_u32_array(fwnode, "riscv,group-index-bits",
					    &global->group_index_bits, 1);
	if (rc)
		global->group_index_bits = 0;
	i = BITS_PER_LONG - IMSIC_MMIO_PAGE_SHIFT -
	    global->guest_index_bits - global->hart_index_bits;
	if (i < global->group_index_bits) {
		pr_err("%pfwP: group index bits too big\n", fwnode);
		rc = -EINVAL;
		goto out_free_local;
	}

	/*
	 * Find first bit position of group index.
	 * If not specified assumed the default APLIC-IMSIC configuration.
	 */
	rc = fwnode_property_read_u32_array(fwnode, "riscv,group-index-shift",
					    &global->group_index_shift, 1);
	if (rc)
		global->group_index_shift = IMSIC_MMIO_PAGE_SHIFT * 2;
	i = global->group_index_bits + global->group_index_shift - 1;
	if (i >= BITS_PER_LONG) {
		pr_err("%pfwP: group index shift too big\n", fwnode);
		rc = -EINVAL;
		goto out_free_local;
	}

	/* Find number of interrupt identities */
	rc = fwnode_property_read_u32_array(fwnode, "riscv,num-ids",
					    &global->nr_ids, 1);
	if (rc) {
		pr_err("%pfwP: number of interrupt identities not found\n",
			fwnode);
		goto out_free_local;
	}
	if ((global->nr_ids < IMSIC_MIN_ID) ||
	    (global->nr_ids >= IMSIC_MAX_ID) ||
	    ((global->nr_ids & IMSIC_MIN_ID) != IMSIC_MIN_ID)) {
		pr_err("%pfwP: invalid number of interrupt identities\n",
			fwnode);
		rc = -EINVAL;
		goto out_free_local;
	}

	/* Find number of guest interrupt identities */
	if (fwnode_property_read_u32_array(fwnode, "riscv,num-guest-ids",
					   &global->nr_guest_ids, 1))
		global->nr_guest_ids = global->nr_ids;
	if ((global->nr_guest_ids < IMSIC_MIN_ID) ||
	    (global->nr_guest_ids >= IMSIC_MAX_ID) ||
	    ((global->nr_guest_ids & IMSIC_MIN_ID) != IMSIC_MIN_ID)) {
		pr_err("%pfwP: invalid number of guest interrupt identities\n",
			fwnode);
		rc = -EINVAL;
		goto out_free_local;
	}

	/* Compute base address */
	rc = imsic_get_mmio_resource(fwnode, 0, &res);
	if (rc) {
		pr_err("%pfwP: first MMIO resource not found\n", fwnode);
		rc = -EINVAL;
		goto out_free_local;
	}
	global->base_addr = res.start;
	global->base_addr &= ~(BIT(global->guest_index_bits +
				   global->hart_index_bits +
				   IMSIC_MMIO_PAGE_SHIFT) - 1);
	global->base_addr &= ~((BIT(global->group_index_bits) - 1) <<
			       global->group_index_shift);

	/* Find number of MMIO register sets */
	while (!imsic_get_mmio_resource(fwnode, num_mmios, &res))
		num_mmios++;

	/* Allocate MMIO resource array */
	mmios = kcalloc(num_mmios, sizeof(*mmios), GFP_KERNEL);
	if (!mmios) {
		rc = -ENOMEM;
		goto out_free_local;
	}

	/* Allocate MMIO virtual address array */
	mmios_va = kcalloc(num_mmios, sizeof(*mmios_va), GFP_KERNEL);
	if (!mmios_va) {
		rc = -ENOMEM;
		goto out_iounmap;
	}

	/* Parse and map MMIO register sets */
	for (i = 0; i < num_mmios; i++) {
		rc = imsic_get_mmio_resource(fwnode, i, &mmios[i]);
		if (rc) {
			pr_err("%pfwP: unable to parse MMIO regset %d\n",
				fwnode, i);
			goto out_iounmap;
		}

		base_addr = mmios[i].start;
		base_addr &= ~(BIT(global->guest_index_bits +
				   global->hart_index_bits +
				   IMSIC_MMIO_PAGE_SHIFT) - 1);
		base_addr &= ~((BIT(global->group_index_bits) - 1) <<
			       global->group_index_shift);
		if (base_addr != global->base_addr) {
			rc = -EINVAL;
			pr_err("%pfwP: address mismatch for regset %d\n",
				fwnode, i);
			goto out_iounmap;
		}

		mmios_va[i] = ioremap(mmios[i].start, resource_size(&mmios[i]));
		if (!mmios_va[i]) {
			rc = -EIO;
			pr_err("%pfwP: unable to map MMIO regset %d\n",
				fwnode, i);
			goto out_iounmap;
		}
	}

	/* Initialize interrupt identity management */
	rc = imsic_ids_init();
	if (rc) {
		pr_err("%pfwP: failed to initialize interrupt management\n",
		       fwnode);
		goto out_iounmap;
	}

	/* Configure handlers for target CPUs */
	for (i = 0; i < nr_parent_irqs; i++) {
		rc = imsic_get_parent_hartid(fwnode, i, &hartid);
		if (rc) {
			pr_warn("%pfwP: hart ID for parent irq%d not found\n",
				fwnode, i);
			continue;
		}

		cpu = riscv_hartid_to_cpuid(hartid);
		if (cpu < 0) {
			pr_warn("%pfwP: invalid cpuid for parent irq%d\n",
				fwnode, i);
			continue;
		}

		/* Find MMIO location of MSI page */
		index = num_mmios;
		reloff = i * BIT(global->guest_index_bits) *
			 IMSIC_MMIO_PAGE_SZ;
		for (j = 0; num_mmios; j++) {
			if (reloff < resource_size(&mmios[j])) {
				index = j;
				break;
			}

			/*
			 * MMIO region size may not be aligned to
			 * BIT(global->guest_index_bits) * IMSIC_MMIO_PAGE_SZ
			 * if holes are present.
			 */
			reloff -= ALIGN(resource_size(&mmios[j]),
			BIT(global->guest_index_bits) * IMSIC_MMIO_PAGE_SZ);
		}
		if (index >= num_mmios) {
			pr_warn("%pfwP: MMIO not found for parent irq%d\n",
				fwnode, i);
			continue;
		}

		cpumask_set_cpu(cpu, &imsic->lmask);

		local = per_cpu_ptr(global->local, cpu);
		local->msi_pa = mmios[index].start + reloff;
		local->msi_va = mmios_va[index] + reloff;

		nr_handlers++;
	}

	/* If no CPU handlers found then can't take interrupts */
	if (!nr_handlers) {
		pr_err("%pfwP: No CPU handlers found\n", fwnode);
		rc = -ENODEV;
		goto out_ids_cleanup;
	}

	/* Find parent domain and register chained handler */
	domain = irq_find_matching_fwnode(riscv_get_intc_hwnode(),
					  DOMAIN_BUS_ANY);
	if (!domain) {
		pr_err("%pfwP: Failed to find INTC domain\n", fwnode);
		rc = -ENOENT;
		goto out_ids_cleanup;
	}
	imsic_parent_irq = irq_create_mapping(domain, RV_IRQ_EXT);
	if (!imsic_parent_irq) {
		pr_err("%pfwP: Failed to create INTC mapping\n", fwnode);
		rc = -ENOENT;
		goto out_ids_cleanup;
	}
	irq_set_chained_handler(imsic_parent_irq, imsic_handle_irq);

	/* Initialize IPI domain */
	rc = imsic_ipi_domain_init();
	if (rc) {
		pr_err("%pfwP: Failed to initialize IPI domain\n", fwnode);
		goto out_ids_cleanup;
	}

	/* Initialize IRQ and MSI domains */
	rc = imsic_irq_domains_init(fwnode);
	if (rc) {
		pr_err("%pfwP: Failed to initialize IRQ and MSI domains\n",
		       fwnode);
		goto out_ipi_domain_cleanup;
	}

	/*
	 * Setup cpuhp state (must be done after setting imsic_parent_irq)
	 *
	 * Don't disable per-CPU IMSIC file when CPU goes offline
	 * because this affects IPI and the masking/unmasking of
	 * virtual IPIs is done via generic IPI-Mux
	 */
	cpuhp_setup_state(CPUHP_AP_ONLINE_DYN,
			  "irqchip/riscv/imsic:starting",
			  imsic_starting_cpu, imsic_dying_cpu);

	/* We don't need MMIO arrays anymore so let's free-up */
	kfree(mmios_va);
	kfree(mmios);

	pr_info("%pfwP:  hart-index-bits: %d,  guest-index-bits: %d\n",
		fwnode, global->hart_index_bits, global->guest_index_bits);
	pr_info("%pfwP: group-index-bits: %d, group-index-shift: %d\n",
		fwnode, global->group_index_bits, global->group_index_shift);
	pr_info("%pfwP: mapped %d interrupts for %d CPUs at %pa\n",
		fwnode, global->nr_ids, nr_handlers, &global->base_addr);
	if (imsic->ipi_id)
		pr_info("%pfwP: providing IPIs using interrupt %d\n",
			fwnode, imsic->ipi_id);

	return 0;

out_ipi_domain_cleanup:
	imsic_ipi_domain_cleanup();
out_ids_cleanup:
	imsic_ids_cleanup();
out_iounmap:
	for (i = 0; i < num_mmios; i++) {
		if (mmios_va[i])
			iounmap(mmios_va[i]);
	}
	kfree(mmios_va);
	kfree(mmios);
out_free_local:
	free_percpu(imsic->global.local);
out_free_priv:
	kfree(imsic);
	imsic = NULL;
	return rc;
}

static int __init imsic_dt_init(struct device_node *node,
				struct device_node *parent)
{
	return imsic_init(&node->fwnode);
}
IRQCHIP_DECLARE(riscv_imsic, "riscv,imsics", imsic_dt_init);
