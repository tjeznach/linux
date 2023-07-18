// SPDX-FileCopyrightText: Copyright (c) 2022 by Rivos Inc.
// Confidential and proprietary, see LICENSE for details.
// SPDX-License-Identifier: LicenseRef-Rivos-Internal-Only

#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>
#include <errno.h>
#include <unistd.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/param.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/syscall.h>
#include <linux/memfd.h>
#include <time.h>

/* Not all libcs provide 'memfd_create' today: */
static int wrap_memfd_create(const char *__name, unsigned int __flags)
{
	return syscall(SYS_memfd_create, __name, __flags);
}

int main(int argn, char *argv[])
{
	/*
	 * Adding debug printf generates network traffic, with additional IOTLB cache polution
	 * and uncorrelated IOTLB cache invalidation requests from network stack.
	 * Keep test as I/O quied as possible to limit IOMMU map/unmap not related to the test.
	 */
	const int use_print = 0;
	/*
	 * Page request interface implementation for qemu-edu device is simplified, and relies
	 * on timely processing of the page-request. Ensure memory used by the test buffer is
	 *  populated and locked to work around this limitation.
	 */
	const int use_mlock = 0;

	/*
	 * Use MAP_HUGETLB for buffer mappings.
	 * Enable and allocate huge pages with, eg. 'echo 8 > /proc/sys/vm/nr_hugepages'
	 */
	const int use_hugetlb = 1;

	const size_t len = 256;
	const size_t pgs = getpagesize();
	const size_t mfd_cnt = 64;

	size_t dst_len, src_len;
	int mfd, rot;
	int fd, ret;
	int try_hugetlb;
	char ref[len];
	char *src, *dst, *dst_prev, *src_prev;
	int count = 1;

	/* QEMU EDU bounce buffer location */
	const size_t tmp = 0x40000;

	if (argn < 2) {
		printf("usage %s /dev/qemu-edu\n", argv[0]);
		return -1;
	}

	if (argn > 2)
		count = atoi(argv[2]);

	srand((unsigned int)time(NULL));

	/* qemu-edu device node */
	fd = open(argv[1], O_RDWR);
	if (fd < 0) {
		printf("Can not open %s, err: %d\n", argv[1], fd);
		return -1;
	}

	/* pool of physical pages for mapping pseudo-randomization */
	mfd = wrap_memfd_create("buffer", MFD_CLOEXEC | 0x10U /* MFD_EXEC */);
	if (mfd < 0) {
		printf("Can not create memfd, err: %d\n", mfd);
		return -1;
	}
	ftruncate(mfd, pgs * mfd_cnt);

	src_prev = NULL;
	dst_prev = NULL;
	try_hugetlb = use_hugetlb;
	rot = 0;

retry:
	/* reuse src and dst virtual addresses if already assigned in prev iteration. */
	src_len = pgs;
	src = mmap(src_prev, src_len, PROT_EXEC | PROT_READ | PROT_WRITE, MAP_SHARED, mfd, rot * pgs);
	rot = (rot + 1) % mfd_cnt;

	if (try_hugetlb) {
		dst_len = 2 << 20; // assume 2M page.
		dst = mmap(dst_prev, dst_len, PROT_EXEC | PROT_READ | PROT_WRITE,
			   MAP_PRIVATE | MAP_ANONYMOUS | MAP_HUGETLB, -1, 0);
	}
	if (!try_hugetlb || dst == MAP_FAILED) {
		dst_len = pgs;
		dst = mmap(dst_prev, dst_len, PROT_EXEC | PROT_READ | PROT_WRITE,
			   MAP_SHARED, mfd, rot * pgs);
	}

	rot = (rot + 1) % mfd_cnt;

	memset(ref, 0xa5, len);
	snprintf(ref, len, "Lorem Ipsum @ %x", rand());

	memcpy(src, ref, len);

	if (use_mlock)
		mlock(src, len);
	ret = pwrite(fd, src, len, tmp);
	if (use_mlock)
		munlock(src, len);

	if (ret < 0)
		printf("Can not write, error %d\n", ret);

	if (use_mlock)
		mlock(dst, len);
	ret = pread(fd, dst, len, tmp);
	if (use_mlock)
		munlock(dst, len);

	if (ret < 0)
		printf("Can not read, error %d\n", ret);

	if (memcmp(src, ref, len)) {
		printf("ERROR\nsrc: %p  - %s\nref: %p  - %s\n", src, (char *)src, ref, (char *)ref);
		return -1;
	}

	if (memcmp(dst, ref, len)) {
		printf("ERROR\ndst: %p  - %s\nref: %p  - %s\n", dst, (char *)dst, ref, (char *)ref);
		return -1;
	}

	if (use_print)
		printf("src: %p  - %s\ndst: %p  - %s\n", src, (char *)src, dst, (char *)dst);

	madvise(src, src_len, MADV_DONTNEED);
	munmap(src, src_len);

	madvise(dst, dst_len, MADV_DONTNEED);
	munmap(dst, dst_len);

	src_prev = src;
	dst_prev = dst;
	try_hugetlb = !try_hugetlb && use_hugetlb;

	if (--count > 0)
		goto retry;

	close(fd);

	return 0;
}
