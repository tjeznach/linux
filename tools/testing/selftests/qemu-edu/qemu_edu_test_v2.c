// SPDX-FileCopyrightText: Copyright (c) 2022 by Rivos Inc.
// Confidential and proprietary, see LICENSE for details.
// SPDX-License-Identifier: LicenseRef-Rivos-Internal-Only

/* Building upon qemu_edu_test, this test copies random data between
 * randomised offsets within source/destination buffers to maximise
 * byte-misalignment.
 */

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
#include <time.h>
#include <assert.h>

#define VERSION			"0.222"
#define DEFAULT_BUF_PAGES	12

/* Utilities */

static void hexdump(uint64_t *buf, size_t len /* assumes multiple of 8! */)
{
	for (size_t offs = 0; offs < (len/8); offs++) {
		if ((offs & 15) == 0)
			printf(" %16p: ", &buf[offs]);
		printf("%016lx ", buf[offs]);
		if ((offs & 15) == 15)
			printf("\n");
	}
}

/* Method:
 * - Map two pages, src/dest
 * - generate a block of data (varying-size)
 * - use the device to read from one, write to another
 * - compare
 */

static void help(char *n)
{
	printf("qemu-edu-test-v2 " VERSION "\n");
	printf("Syntax:\n"
	       "\t%s\t[options]\n"
	       "\t\t-l <loops>\t Number of test loops\n"
	       "\t\t-p <pages>\t Number of pages to allocate (default %d)\n"
	       "\t\t-d <device>\t Use specified device node\n"
	       "\t\t-s <seed>\t Set rand() seed\n"
	       "\t\t-S <size>\t Maximum block size (default is pagesize)\n"
	       "\t\t-v \t\t Verbose\n"
	       "\t\t-r \t\t Reuse buffer (else re-mapped)\n"
	       "\t\t-R <n>\t\t Remap every n loops\n", n, DEFAULT_BUF_PAGES);
}

static void map_buffers(const size_t host_buffer_len, int buf_pages,
			void **rdat, void **src, void **srcref, void **dst,
			void **dstref, int tweak)
{
	/* Allocate buffers with alternating madvise for hugepages
	 * based on tweak (e.g. loop count): try to involve DMA to
	 * both hugepage and non-hugepage mappings by making src one
	 * and dst the other.
	 */
	*rdat = mmap(0, host_buffer_len * buf_pages, PROT_READ | PROT_WRITE,
		     MAP_ANON | MAP_PRIVATE, -1, 0);

	*src = mmap(0, host_buffer_len * buf_pages, PROT_READ | PROT_WRITE,
		    MAP_ANON | MAP_PRIVATE, -1, 0);
	madvise(*src, host_buffer_len * buf_pages,
		(tweak & 1) ? MADV_HUGEPAGE : MADV_NOHUGEPAGE);

	*srcref = mmap(0, host_buffer_len * buf_pages, PROT_READ | PROT_WRITE,
		       MAP_ANON | MAP_PRIVATE, -1, 0);

	*dst = mmap(0, host_buffer_len * buf_pages, PROT_READ | PROT_WRITE,
		    MAP_ANON | MAP_PRIVATE, -1, 0);
	madvise(*dst, host_buffer_len * buf_pages,
		(tweak & 1) ? MADV_NOHUGEPAGE : MADV_HUGEPAGE);

	*dstref = mmap(0, host_buffer_len * buf_pages, PROT_READ | PROT_WRITE,
		       MAP_ANON | MAP_PRIVATE, -1, 0);

	assert(*rdat != MAP_FAILED && *src != MAP_FAILED && *srcref != MAP_FAILED &&
	       *dst != MAP_FAILED && *dstref != MAP_FAILED);

	/* Secondly, touch SOME of the pages to ensure they're
	 * demand-allocated.  This means that when we remap, we're
	 * more likely to get different physical pages rather than
	 * reallocating the ones just freed.  But, it also means most
	 * will remain demand-allocated.
	 */
	for (int i = 0; i < buf_pages; i += 3) {
		*(uint64_t *)(*src + (i*host_buffer_len/8)) = 0;
		*(uint64_t *)(*dst + (i*host_buffer_len/8)) = 0;
	}
}

static void unmap_buffers(const size_t host_buffer_len, int buf_pages, void *rdat,
			  void *src, void *srcref, void *dst, void *dstref)
{
	munmap(rdat, host_buffer_len * buf_pages);
	munmap(src, host_buffer_len * buf_pages);
	munmap(srcref, host_buffer_len * buf_pages);
	munmap(dst, host_buffer_len * buf_pages);
	munmap(dstref, host_buffer_len * buf_pages);
}

int main(int argc, char *argv[])
{
	const size_t host_buffer_len = getpagesize();
	unsigned int buf_pages = DEFAULT_BUF_PAGES;
	unsigned int blocklen_min = 32;
	unsigned int blocklen_limit = ~0; // Clamped below
	void *src, *srcref, *dst, *dstref, *rdat;
	int count = 1;
	int reuse = 0;
	int remap_period = (int)(buf_pages*1.5);
	int verbose = 0;
	char *dev = "/dev/qemu-edu";
	int chr;

	while ((chr = getopt(argc, argv, "d:l:s:rhvR:p:S:")) != -1) {
		switch (chr) {
		case 'd':
			dev = optarg;
			break;

		case 'l':
			count = atoi(optarg);
			break;

		case 's':
			srand(atoi(optarg));
			break;

		case 'r':
			reuse = 1;
			break;

		case 'R':
			remap_period = atoi(optarg);
			break;

		case 'v':
			verbose = 1;
			break;

		case 'p':
			buf_pages = atoi(optarg);
			break;

		case 'S':
			blocklen_limit = atoi(optarg);
			break;

		case 'h':
		default:
			help(argv[0]);
			return 1;
			break;
		}
	}

	if (blocklen_limit > host_buffer_len-blocklen_min)
		blocklen_limit = host_buffer_len-blocklen_min;

#ifndef TEST_BUILD
	/*
	 * Page request interface implementation for qemu-edu device
	 * is simplified, and relies on timely processing of the
	 * page-request. Ensure memory used by the test buffer is
	 * populated and locked to work around this limitation.
	 */
	const int use_mlock = 0; /* FIXME: Support this */

	int fd = -1;
	int ret;
	/* QEMU EDU bounce buffer location */
	const size_t dev_offset = 0x40000;

	/* qemu-edu device node */
	fd = open(dev, O_RDWR);
	if (fd < 0) {
		printf("Cannot open %s, err: %d\n", dev, fd);
		return -1;
	}
#endif

	printf("qemu-edu-test-v2 " VERSION "\n");

	int map_tweak = 0;
	/* Buffers: rdat is a bunch of random data; src is updated
	 * with it (and read by the device).  dst is a buffer to which
	 * the device writes, and ref is prepared to look like dst
	 * should look after the transfer.
	 */
	map_buffers(host_buffer_len, buf_pages, &rdat, &src, &srcref, &dst, &dstref, map_tweak++);

	printf("Mapped: rdat %p, src %p, srcref %p, dst %p, dstref %p\n",
	       rdat, src, srcref, dst, dstref);

	/* Prepare random reference buffer (changed slightly each loop) */
	for (int i = 0; i < host_buffer_len/4; i++) {
		((uint32_t *)rdat)[i] = rand();
	}

	printf("Running %d loops", count);
	if (reuse)
		printf(", reusing buffers");
	else
		printf(", remapping every %d loops", remap_period);
	printf(":\n");

	for (int loop = 0; loop < count; loop++) {
		/* A given test/loop transfers a random block, between 32
		 * bytes and max buffer size:
		 */
		int blocklen = 32 + (rand() % blocklen_limit);
		/* It also has a random offset within the host buffer, between
		 * 0 and (host_buffer_len-size), for device read data:
		 */
		int blockoffs_r = rand() % (1 + host_buffer_len - blocklen);
		/* ...and a different offset for device write data: */
		int blockoffs_w = rand() % (1 + host_buffer_len - blocklen);
		/* And choose random data from a random place (a little faster
		 * than recreating every loop)
		 */
		int randoffs = rand() % (1 + host_buffer_len - blocklen);

		int page_offs = reuse ? 0 : host_buffer_len * (loop % buf_pages);
		void *randblock = rdat + page_offs + randoffs;
		void *from = src + page_offs + blockoffs_r;
		void *to = dst + page_offs + blockoffs_w;
		void *oracle = dstref + page_offs + blockoffs_w;

		if (verbose)
			printf("loop %d: %d from %" PRIxPTR " to %" PRIxPTR "\n",
			       loop, blocklen, from, to);

		/* Update the 'from' data with some randomness */
		memcpy(from, randblock, blocklen);

		/* Take a copy of the src to leave it as a reference, so we
		 * can check the src buffer didn't change:
		 */
		memcpy(srcref, src, host_buffer_len);

		/* Create the oracle (AKA what "to" should become) */
		memcpy(oracle, from, blocklen);

#ifdef TEST_BUILD
		memcpy(to, from, blocklen);
#else
		/* Perform, effectively, a memcpy 'from' -> 'to' using the device: */

		/* Write the source buffer to device memory (read from host) */
		if (use_mlock)
			mlock(from, blocklen);
		ret = pwrite(fd, from, blocklen, dev_offset);
		if (use_mlock)
			munlock(from, blocklen);
		if (ret < 0) {
			printf("Cannot write, error %d\n", ret);
		}

		/* Read back from device memory into dest buffer (write from host) */
		if (use_mlock)
			mlock(to, blocklen);
		ret = pread(fd, to, blocklen, dev_offset);
		if (use_mlock)
			munlock(to, blocklen);
		if (ret < 0) {
			printf("Cannot read, error %d\n", ret);
		}
#endif

		int diff = 0;
		/* Compare entire 'from' buffer; it should NOT have changed: */
		if (memcmp(src, srcref, host_buffer_len)) {
			printf("ERROR\nRead-side buffer changed! (loop %d)\n\n", loop);
			printf("src:\n");
			hexdump(src, host_buffer_len);
			printf("srcref:\n");
			hexdump(srcref, host_buffer_len);
			diff = 1;
		}

		/* Check the entire 'to' buffer matches the reference version: */
		if (memcmp(dst, dstref, host_buffer_len)) {
			printf("ERROR\nDestination buffer doesn't match reference! (loop %d)\n\n",
			       loop);
			printf("dst:\n");
			hexdump(dst, host_buffer_len);
			printf("dstref:\n");
			hexdump(dstref, host_buffer_len);
			diff = 1;
		}

		if (diff) {
			printf("Errors during transfer: data 0x%x from %p to %p:\n",
			       blocklen, from, to);
			printf("Original buffer (pre-transfer):\n");
			hexdump(srcref, host_buffer_len);
			printf("\nExpected result after transfer:\n");
			hexdump(oracle, blocklen);
			printf("\nResult was:\n");
			hexdump(to, blocklen);
			return -1;
		}

		/* Mess with the random data a little: */
		((uint32_t *)rdat)[rand() & ((host_buffer_len/4)-1)] = rand();

		if (!reuse && (loop % remap_period) == 0) {
			void *ordat = rdat;
			void *osrc = src;
			void *osrcref = srcref;
			void *odst = dst;
			void *odstref = dstref;
			/* Every few loops, remap the buffers to (hopefully) get
			 * new phys addresses.  Note some pages are used twice, as
			 * that's also interesting.
			 */
			map_buffers(host_buffer_len, buf_pages, &rdat, &src, &srcref,
				    &dst, &dstref, map_tweak++);
			if (verbose)
				printf("Remapped: rdat %p, src %p, srcref %p, dst %p, dstref %p\n",
				       rdat, src, srcref, dst, dstref);
			unmap_buffers(host_buffer_len, buf_pages, ordat, osrc,
				      osrcref, odst, odstref);
		}
	}

#ifndef TEST_BUILD
	close(fd);
#endif

	return 0;
}
