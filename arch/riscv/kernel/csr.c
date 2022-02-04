// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2022 Rivos Inc.
 * Author: Tomasz Jeznach <tjeznach@rivosinc.com>
 */

/*
 * Risc-V CSR access device
 *
 * This device is accessed by lseek() to the appropriate register number
 * and then read/write in chunks of 8 bytes.  A larger size means multiple
 * reads or writes of the same register.
 *
 * This driver uses /dev/cpu/%d/csr where %d is the minor number, and on
 * multiprocessor system will direct the access to hart %d.
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/module.h>

#include <linux/types.h>
#include <linux/errno.h>
#include <linux/fcntl.h>
#include <linux/init.h>
#include <linux/poll.h>
#include <linux/smp.h>
#include <linux/major.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/cpu.h>
#include <linux/notifier.h>
#include <linux/uaccess.h>
#include <linux/gfp.h>

/* Reuse MSR_MAJOR number for architecture specific register access. */
#define CSR_MAJOR MSR_MAJOR

static struct class *csr_class;
static enum cpuhp_state cpuhp_csr_state;

struct csr_info {
	u64 value;
	u32 index;
	struct completion done;
};

/* Code generator for fixed asm csr_{read/write} calls. */
#define _NX(n, m) m(n)
#define _N0(n, m) _NX((n), m) _NX(((n)+1), m)
#define _N1(n, m) _N0((n), m) _N0(((n)+2), m)
#define _N2(n, m) _N1((n), m) _N1(((n)+4), m)
#define _N3(n, m) _N2((n), m) _N2(((n)+8), m)
#define _N4(n, m) _N3((n), m) _N3(((n)+16), m)
#define _N5(n, m) _N4((n), m) _N4(((n)+32), m)
#define _N6(n, m) _N5((n), m) _N5(((n)+64), m)
#define _N7(n, m) _N6((n), m) _N6(((n)+128), m)
#define _N8(n, m) _N7((n), m) _N7(((n)+256), m)
#define _N9(n, m) _N8((n), m) _N8(((n)+512), m)

static void __csr_read_on_cpu(void *info)
{
#define _CSR_RD(n) case n: rv->value = csr_read(n); break;
	struct csr_info *rv = info;
	switch (rv->index) {
		_N9(0, _CSR_RD)
	}
#undef _CSR_RD
	complete(&rv->done);
}

static int csr_read_on_cpu(unsigned cpu, u32 index, u64 * val)
{
	struct csr_info rv;
	call_single_data_t csd;
	int err;

	INIT_CSD(&csd, __csr_read_on_cpu, &rv);

	init_completion(&rv.done);
	rv.index = index;
	rv.value = 0;

	err = smp_call_function_single_async(cpu, &csd);
	if (!err) {
		wait_for_completion(&rv.done);
		*val = rv.value;
	}

	return err;
}

static ssize_t file_csr_read(struct file *file, char __user * buf,
			     size_t count, loff_t * ppos)
{
	u64 __user *tmp = (u64 __user *) buf;
	u64 val;
	u32 reg = *ppos;
	int cpu = iminor(file_inode(file));
	int err = 0;
	ssize_t bytes = 0;

	if (count % 8)
		return -EINVAL;	/* Invalid chunk size */

	for (; count; count -= 8) {
		err = csr_read_on_cpu(cpu, reg, &val);
		if (err)
			break;
		if (copy_to_user(tmp, &val, sizeof(val))) {
			err = -EFAULT;
			break;
		}

		tmp++;
		bytes += 8;
	}

	return bytes ? bytes : err;
}

static void __csr_write_on_cpu(void *info)
{
#define _CSR_WR(n) case n: csr_write(n, rv->value); break;
	struct csr_info *rv = info;
	switch (rv->index) {
		_N9(0, _CSR_WR)
	}
#undef _CSR_WR
	complete(&rv->done);
}

static int csr_write_on_cpu(unsigned cpu, u32 index, u64 value)
{
	struct csr_info rv;
	call_single_data_t csd;
	int err;

	INIT_CSD(&csd, __csr_write_on_cpu, &rv);

	init_completion(&rv.done);
	rv.index = index;
	rv.value = value;

	err = smp_call_function_single_async(cpu, &csd);
	if (!err)
		wait_for_completion(&rv.done);

	return err;
}

static ssize_t file_csr_write(struct file *file, const char __user * buf,
			      size_t count, loff_t * ppos)
{
	u64 __user *tmp = (u64 __user *) buf;
	u64 val;
	u32 reg = *ppos;
	int cpu = iminor(file_inode(file));
	int err = 0;
	ssize_t bytes = 0;

	if (count % 8)
		return -EINVAL;	/* Invalid chunk size */

	for (; count; count -= 8) {
		if (copy_from_user(&val, tmp, 8)) {
			err = -EFAULT;
			break;
		}

		err = csr_write_on_cpu(cpu, reg, val);
		if (err)
			break;

		tmp++;
		bytes += 8;
	}

	return bytes ? bytes : err;
}

static int file_csr_open(struct inode *inode, struct file *file)
{
	unsigned int cpu = iminor(file_inode(file));

	if (!capable(CAP_SYS_RAWIO))
		return -EPERM;

	if (cpu >= nr_cpu_ids || !cpu_online(cpu))
		return -ENXIO;

	return 0;
}

/*
 * File operations we support
 */
static const struct file_operations csr_fops = {
	.owner = THIS_MODULE,
	.llseek = no_seek_end_llseek,
	.read = file_csr_read,
	.write = file_csr_write,
	.open = file_csr_open,
};

static int csr_device_create(unsigned int cpu)
{
	struct device *dev;

	dev = device_create(csr_class, NULL, MKDEV(CSR_MAJOR, cpu), NULL,
			    "csr%d", cpu);
	return PTR_ERR_OR_ZERO(dev);
}

static int csr_device_destroy(unsigned int cpu)
{
	device_destroy(csr_class, MKDEV(CSR_MAJOR, cpu));
	return 0;
}

static char *csr_devnode(struct device *dev, umode_t * mode)
{
	return kasprintf(GFP_KERNEL, "cpu/%u/csr", MINOR(dev->devt));
}

static int __init csr_init(void)
{
	int err;

	if (__register_chrdev(CSR_MAJOR, 0, NR_CPUS, "cpu/csr", &csr_fops)) {
		pr_err("unable to get major %d for msr\n", MSR_MAJOR);
		return -EBUSY;
	}
	csr_class = class_create(THIS_MODULE, "csr");
	if (IS_ERR(csr_class)) {
		err = PTR_ERR(csr_class);
		goto out_chrdev;
	}
	csr_class->devnode = csr_devnode;

	err = cpuhp_setup_state(CPUHP_AP_ONLINE_DYN, "riscv/csr:online",
				csr_device_create, csr_device_destroy);
	if (err < 0)
		goto out_class;

	cpuhp_csr_state = err;
	return 0;

 out_class:
	class_destroy(csr_class);
 out_chrdev:
	__unregister_chrdev(CSR_MAJOR, 0, NR_CPUS, "cpu/csr");
	return err;
}

module_init(csr_init);

static void __exit csr_exit(void)
{
	cpuhp_remove_state(cpuhp_csr_state);
	class_destroy(csr_class);
	__unregister_chrdev(CSR_MAJOR, 0, NR_CPUS, "cpu/csr");
}

module_exit(csr_exit);

MODULE_AUTHOR("Tomasz Jeznach <tjeznach@rivosinc.com>");
MODULE_DESCRIPTION("Risc-V generic CSR driver");
MODULE_LICENSE("GPL v2");
