/*
 * writespi.c
 *
 * Copyright (C) 2016 Dream Property GmbH, Germany
 *                    http://www.dream-multimedia-tv.de/
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#define _POSIX_C_SOURCE 199309L
#include <fcntl.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/param.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <time.h>
#include <unistd.h>

#define ARRAY_SIZE(x)		(sizeof((x)) / sizeof(*(x)))

#define BCM_PHYSICAL_OFFSET	0x10000000
#define BCM_CHIP_FAMILY_ID	0x00404000

#define HIF_MSPI_BCM73625	0x00413200
#define HIF_MSPI_BCM7435	0x0041d400
#define HIF_MSPI_SIZE		0x188
#define HIF_SPI_INTR2_BCM73625	0x00411d00
#define HIF_SPI_INTR2_BCM7435	0x0041bd00
#define HIF_SPI_INTR2_SIZE	0x30

#define GIO_BCM73625		0x00406500
#define GIO_SIZE		0xa0

#define SPI_SECTOR_SIZE	4096
#define SPI_PAGE_SIZE	256

enum spi_cmd {
	SPI_CMD_WRSR		= 0x01,
	SPI_CMD_PP		= 0x02,
	SPI_CMD_READ		= 0x03,
	SPI_CMD_WRDI		= 0x04,
	SPI_CMD_RDSR		= 0x05,
	SPI_CMD_WREN		= 0x06,
	SPI_CMD_FAST_READ	= 0x0b,
	SPI_CMD_SE		= 0x20,
	SPI_CMD_CE		= 0x60,
	SPI_CMD_GBLK		= 0x7e,
	SPI_CMD_GBULK		= 0x98,
	SPI_CMD_RDID		= 0x9f,
	SPI_CMD_RDP		= 0xab,
	SPI_CMD_DP		= 0xb9,
	SPI_CMD_BE		= 0xd8,
};

enum manuf_id {
	MANUF_ID_EON = 0x1c,
	MANUF_ID_WINBOND = 0xef,
};

#define SPI_RDSR_WIP		(1 << 0)
#define SPI_RDSR_WEL		(1 << 1)
#define SPI_RDSR_BP0		(1 << 2)
#define SPI_RDSR_BP1		(1 << 3)
#define SPI_RDSR_BP2		(1 << 4)
#define SPI_RDSR_BP3		(1 << 5)
#define SPI_RDSR_SRWD		(1 << 7)

#define KiB(x)	((x) * 1024)
#define MiB(x)	((x) * 1024 * 1024)

struct flash_part {
	char *name;
	bool rdonly;
	unsigned int size;
};

static const struct flash_part flash_map[] = {
	{ "fsbl", true, MiB(1) },
	{ "ssbl", false, MiB(1) },
	{ "nvram", true, KiB(256) },
	{ "ssblconf", true, KiB(256) },
	{ "reserved", true, KiB(512) },
	{ "kernel", false, MiB(13) },
};

static inline void cpu_relax(void)
{
#if defined(__mips__)
	__asm__ volatile("" ::: "memory");
#endif
}

static volatile unsigned char *mmio_spi;
static volatile unsigned long *hif_spi_intr;
static int devmem = -1;

static volatile void *ioremap(unsigned long phys, size_t length)
{
	unsigned long mask = sysconf(_SC_PAGESIZE) - 1;
	unsigned long addr = phys & ~mask;
	unsigned long offset = phys & mask;
	unsigned char *mem;

	mem = mmap(NULL, length + offset, PROT_READ | PROT_WRITE, MAP_SHARED, devmem, addr);
	if (mem == MAP_FAILED) {
		perror("mmap");
		return NULL;
	}

	return &mem[offset];
}

static void iounmap(volatile void *virt, size_t length)
{
	unsigned long mask = sysconf(_SC_PAGESIZE) - 1;
	unsigned long offset = (unsigned long)virt & mask;

	munmap((void *)virt - offset, length + offset);
}

static void spi_write_bytes(const unsigned char *out, unsigned char *in, size_t len)
{
	size_t i;

	while (len) {
		size_t tr = MIN(len, 16);

		len -= tr;

		/* set tx ram */
		for (i = 0; i < tr; i++) {
			mmio_spi[0x40 + (i * 8)] = *out++; // TXRAM

			if ((len == 0) && (i == (tr - 1)))
				mmio_spi[0x140 + (i * 4)] = 0;
			else
				mmio_spi[0x140 + (i * 4)] = (1 << 7);
		}

		mmio_spi[0x14] = tr - 1; // ENDQP (end queue pointer)
		mmio_spi[0x10] = 0;    // NEWQP (queue start)

		mmio_spi[0x18] = 0xC0; // SPE - spi enable (start transfer)
		while (mmio_spi[0x18] & 0x40)
			cpu_relax();

		if (in != NULL)
			for (i = 0; i < tr; i++)
				*in++ = mmio_spi[0xc4 + (i * 8)]; // RXRAM
	}
}

static bool dm520_setup_gpio(void)
{
	volatile unsigned long *mmio_gpio;

	mmio_gpio = ioremap(BCM_PHYSICAL_OFFSET + GIO_BCM73625, GIO_SIZE);
	if (mmio_gpio == NULL)
		return false;

	/* set SPI_SELECT output */
	mmio_gpio[0x48/4] &= ~(1<<21);  //IO_DIR set output
	/* select onboard spirom (in any case) */
	mmio_gpio[0x44/4] |= (1<<21);   //IO_DATA

	iounmap(mmio_gpio, GIO_SIZE);
	return true;
}

static unsigned long hif_spi_intr2_enable;

static bool spi_init(void)
{
	volatile unsigned long *family_id;
	unsigned long chip_id;
	unsigned long hif_mspi;
	unsigned long hif_spi_intr2;
	unsigned long hif_spi_intr2_disable;
	unsigned char spcr;

	family_id = ioremap(BCM_PHYSICAL_OFFSET + BCM_CHIP_FAMILY_ID, 4);
	if (family_id == NULL)
		return false;

	chip_id = *family_id;
	chip_id = (chip_id >> 28 ? chip_id >> 16 : chip_id >> 8);
	iounmap(family_id, 4);

	switch (chip_id) {
	case 0x73625:
		hif_mspi = HIF_MSPI_BCM73625;
		hif_spi_intr2 = HIF_SPI_INTR2_BCM73625;
		hif_spi_intr2_disable = 0x3f;
		hif_spi_intr2_enable = 0x3f;
		spcr = 5;
		if (!dm520_setup_gpio())
			return false;
		break;
	case 0x7435:
		hif_mspi = HIF_MSPI_BCM7435;
		hif_spi_intr2 = HIF_SPI_INTR2_BCM7435;
		hif_spi_intr2_disable = 0x7f;
		hif_spi_intr2_enable = 0x20;
		spcr = 6;
		break;
	default:
		fprintf(stderr, "Unsupported chip: %#lx\n", chip_id);
		return false;
	}

	mmio_spi = ioremap(BCM_PHYSICAL_OFFSET + hif_mspi, HIF_MSPI_SIZE);
	if (mmio_spi == NULL)
		return false;

	hif_spi_intr = ioremap(BCM_PHYSICAL_OFFSET + hif_spi_intr2, HIF_SPI_INTR2_SIZE);
	if (hif_spi_intr == NULL)
		return false;

	/* disable kernel irq handler */
	hif_spi_intr[0x10/4] = hif_spi_intr2_disable;

	mmio_spi[0x180] = 1;
	mmio_spi[0x20] = 0;

			/* set baudrate */
	mmio_spi[0] = spcr;          // SPCR, 8 = 1.6Mhz
	mmio_spi[4] = (1<<7);     // SPCR0_MSB, spi master, 8 bits, clock polarity/phase

	return true;
}

static void spi_exit(void)
{
	/* reenable kernel irq handler */
	if (hif_spi_intr) {
		hif_spi_intr[0x14/4] = 0x20;
		iounmap(hif_spi_intr, HIF_SPI_INTR2_SIZE);
	}

	if (mmio_spi)
		iounmap(mmio_spi, HIF_MSPI_SIZE);
}

static int spirom_read_status(void)
{
	unsigned char cmd[] = { SPI_CMD_RDSR, 0 };

	spi_write_bytes(cmd, cmd, sizeof(cmd));

	return cmd[1];
}

static bool spirom_wait_ready(unsigned int to)
{
	const struct timespec req = {
		.tv_sec = 0,
		.tv_nsec = 1000000,
	};

	while (spirom_read_status() & SPI_RDSR_WIP) {
		if (to-- == 0) {
			fprintf(stderr, "timeout!\n");
			return false;
		}
		nanosleep(&req, NULL);
	}

	return true;
}

static bool spirom_write_enable(void)
{
	const unsigned char cmd = SPI_CMD_WREN;

	if (!spirom_wait_ready(1000))
		return false;

	spi_write_bytes(&cmd, NULL, sizeof(cmd));
	return spirom_read_status() & SPI_RDSR_WEL;
}

static bool spirom_reset(void)
{
	const struct timespec req = {
		.tv_sec = 0,
		.tv_nsec = 100000,
	};
	unsigned char cmd;

	if (!spirom_wait_ready(1000))
		return false;

	cmd = 0x66;
	spi_write_bytes(&cmd, NULL, sizeof(cmd));
	nanosleep(&req, NULL);

	cmd = 0x99;
	spi_write_bytes(&cmd, NULL, sizeof(cmd));
	nanosleep(&req, NULL);

	return true;
}

static bool spirom_read_id(void)
{
	unsigned char cmd[] = { SPI_CMD_RDID, 0, 0, 0 };
	unsigned char mem_type;
	unsigned char mem_density;

	if (!spirom_wait_ready(1000))
		return false;

	spi_write_bytes(cmd, cmd, sizeof(cmd));

	printf("Manufacturer ID: %02x\n", cmd[1]);
	printf("Memory type    : %02x\n", cmd[2]);
	printf("Memory density : %02x\n", cmd[3]);

	switch (cmd[1]) {
	case MANUF_ID_EON:
		mem_type = 0x70;
		mem_density = 0x18;
		break;
	case MANUF_ID_WINBOND:
		mem_type = 0x40;
		mem_density = 0x18;
		break;
	default:
		fprintf(stderr, "wrong manufacturer id\n");
		return false;
	}

	if (cmd[2] != mem_type) {
		fprintf(stderr, "wrong memory type\n");
		return false;
	}

	if (cmd[3] != mem_density) {
		fprintf(stderr, "wrong memory density\n");
		return false;
	}

	printf("read id ok\n");
	return true;
}

static bool spirom_sector_erase(unsigned int addr)
{
	const unsigned char cmd[] = {
		SPI_CMD_SE,
		addr >> 16,
		addr >> 8,
		addr,
	};

	if (!spirom_write_enable()) {
		fprintf(stderr, "write enable failed\n");
		return false;
	}

	if (!spirom_wait_ready(1000))
		return false;

	spi_write_bytes(cmd, NULL, sizeof(cmd));
	return spirom_wait_ready(1000);
}

static bool spirom_page_program(unsigned int addr, const unsigned char *page, size_t count)
{
	unsigned char cmd[4 + SPI_PAGE_SIZE];

	if (count > SPI_PAGE_SIZE)
		return false;

	cmd[0] = SPI_CMD_PP;
	cmd[1] = addr >> 16;
	cmd[2] = addr >> 8;
	cmd[3] = addr;
	memcpy(&cmd[4], page, count);
	if (count < SPI_PAGE_SIZE)
		memset(&cmd[count + 4], 0xff, SPI_PAGE_SIZE - count);

	if (!spirom_write_enable()) {
		fprintf(stderr, "write enable failed\n");
		return false;
	}

	if (!spirom_wait_ready(1000))
		return false;

	spi_write_bytes(cmd, NULL, sizeof(cmd));
	return spirom_wait_ready(90000);
}

static bool spirom_lock(void)
{
	const unsigned char cmd = SPI_CMD_GBLK;

	if (!spirom_write_enable()) {
		fprintf(stderr, "write enable failed\n");
		return false;
	}

	if (!spirom_wait_ready(1000))
		return false;

	spi_write_bytes(&cmd, NULL, sizeof(cmd));
	return true;
}

static bool spirom_unlock(void)
{
	const unsigned char cmd = SPI_CMD_GBULK;

	if (!spirom_write_enable()) {
		fprintf(stderr, "write enable failed\n");
		return false;
	}

	if (!spirom_wait_ready(1000))
		return false;

	spi_write_bytes(&cmd, NULL, sizeof(cmd));
	return true;
}

static bool spirom_read(unsigned int addr, unsigned char *buf, unsigned int len)
{
	unsigned char cmd[len + 4];

	cmd[0] = SPI_CMD_READ;
	cmd[1] = addr >> 16;
	cmd[2] = addr >> 8;
	cmd[3] = addr;
	memset(&cmd[4], 0, len);

	if (!spirom_wait_ready(1000))
		return false;

	spi_write_bytes(cmd, cmd, sizeof(cmd));
	memcpy(buf, &cmd[4], len);
	return true;
}

static const size_t bits_per_long = sizeof(unsigned long) * 8;

static unsigned long *bitmap_alloc(size_t nbits)
{
	return calloc((nbits + bits_per_long - 1) / bits_per_long, sizeof(unsigned long));
}

static void bitmap_set(unsigned long *bitmap, unsigned long nr)
{
	bitmap[nr / bits_per_long] |= (1 << (nr % bits_per_long));
}

static bool bitmap_test(unsigned long *bitmap, unsigned long nr)
{
	return bitmap[nr / bits_per_long] & (1 << (nr % bits_per_long));
}

static void bitmap_free(unsigned long *bitmap)
{
	free(bitmap);
}

static bool write_file(int fd, size_t start, size_t size)
{
	unsigned char sector0[SPI_SECTOR_SIZE];
	unsigned char sector1[SPI_SECTOR_SIZE];
	unsigned char *mem = MAP_FAILED;
	unsigned long *need_erase = NULL;
	unsigned long *need_write = NULL;
	size_t nsectors, nchanged = 0;
	const unsigned char *src;
	size_t i, pos, end;
	bool ret = false;

	memset(sector0, 0x00, SPI_SECTOR_SIZE);
	memset(sector1, 0xff, SPI_SECTOR_SIZE);

	mem = mmap(NULL, size, PROT_READ, MAP_SHARED, fd, 0);
	if (mem == MAP_FAILED) {
		perror("mmap");
		goto err;
	}

	end = start + size;
	nsectors = (size + SPI_SECTOR_SIZE - 1) / SPI_SECTOR_SIZE;
	need_erase = bitmap_alloc(nsectors);
	need_write = bitmap_alloc(nsectors);

	printf("comparing %zu sectors...\n", nsectors);

	src = mem;
	pos = start;
	for (i = 0; i < nsectors; i++) {
		unsigned char sector[SPI_SECTOR_SIZE];
		size_t len = MIN(end - pos, SPI_SECTOR_SIZE);

		spirom_read(pos, sector, len);
		if (memcmp(sector, src, len)) {
			if (memcmp(src, sector0, len) && memcmp(sector, sector1, len))
				bitmap_set(need_erase, i);
			if (memcmp(src, sector1, len))
				bitmap_set(need_write, i);
			nchanged++;
		}

		src += SPI_SECTOR_SIZE;
		pos += SPI_SECTOR_SIZE;
	}

	if (nchanged == 0) {
		printf("nothing to do\n");
		ret = true;
		goto err;
	}

	if (!spirom_unlock()) {
		fprintf(stderr, "unlock failed\n");
		goto err;
	}

	printf("erasing %zu sectors...\n", nchanged);

	pos = start;
	for (i = 0; i < nsectors; i++) {
		if (bitmap_test(need_erase, i)) {
			if (!spirom_sector_erase(pos)) {
				fprintf(stderr, "sector erase failed\n");
				goto err;
			}
		}

		pos += SPI_SECTOR_SIZE;
	}

	printf("writing %zu pages...\n", nchanged * SPI_SECTOR_SIZE / SPI_PAGE_SIZE);

	src = mem;
	pos = start;
	for (i = 0; i < nsectors; i++) {
		if (bitmap_test(need_write, i)) {
			size_t len = MIN(end - pos, SPI_SECTOR_SIZE);
			size_t offset;
			for (offset = 0; offset < len; offset += SPI_PAGE_SIZE) {
				size_t plen = MIN(len - offset, SPI_PAGE_SIZE);
				spirom_page_program(pos + offset, &src[offset], plen);
			}
		}

		src += SPI_SECTOR_SIZE;
		pos += SPI_SECTOR_SIZE;
	}

	printf("verifying %zu sectors...\n", nchanged);

	src = mem;
	pos = start;
	for (i = 0; i < nsectors; i++) {
		if (bitmap_test(need_erase, i) || bitmap_test(need_write, i)) {
			unsigned char sector[SPI_SECTOR_SIZE];
			size_t len = MIN(end - pos, SPI_SECTOR_SIZE);

			spirom_read(pos, sector, len);
			if (memcmp(sector, src, len)) {
				fprintf(stderr, "failed at %#zx\n", pos);
				goto err;
			}
		}

		src += SPI_SECTOR_SIZE;
		pos += SPI_SECTOR_SIZE;
	}

	if (!spirom_lock())
		fprintf(stderr, "lock failed\n");

	printf("flash done\n");
	ret = true;
err:
	bitmap_free(need_write);
	bitmap_free(need_erase);
	if (mem != MAP_FAILED)
		munmap(mem, size);
	return ret;
}

int main(int argc, char **argv)
{
	const char *partname = "kernel";
	const struct flash_part *part = NULL;
	unsigned int offset;
	unsigned int i;
	struct stat st;
	int ret = 1;
	int fd;

	if (argc < 2) {
		fprintf(stderr, "usage: %s <filename> [<partition>]\n", argv[0]);
		return 1;
	}

	fd = open(argv[1], O_RDONLY);
	if (fd < 0) {
		perror(argv[1]);
		return 1;
	}

	if (fstat(fd, &st) < 0) {
		perror("fstat");
		goto err;
	}

	if (argc >= 3)
		partname = argv[2];

	offset = 0;
	for (i = 0; i < ARRAY_SIZE(flash_map); i++) {
		if (!strcmp(flash_map[i].name, partname)) {
			part = &flash_map[i];
			break;
		}
		offset += flash_map[i].size;
	}

	if (part == NULL) {
		fprintf(stderr, "Invalid partition: %s\n", partname);
		goto err;
	}

	if (part->rdonly) {
		fprintf(stderr, "Sorry, writing to this partition is not allowed.\n");
		goto err;
	}

	if (part->size < st.st_size) {
		fprintf(stderr, "Input file is too big (max. %u bytes)\n", part->size);
		goto err;
	}

	signal(SIGHUP, SIG_IGN);
	signal(SIGINT, SIG_IGN);
	signal(SIGTERM, SIG_IGN);

	devmem = open("/dev/mem", O_RDWR | O_SYNC);
	if (devmem < 0) {
		perror("/dev/mem");
		goto err;
	}

	if (!spi_init()) {
		fprintf(stderr, "init failed\n");
		goto err;
	}

	if (!spirom_reset()) {
		fprintf(stderr, "reset failed\n");
		goto err;
	}

	if (!spirom_read_id()) {
		fprintf(stderr, "read id failed\n");
		goto err;
	}

	if (write_file(fd, offset, st.st_size))
		ret = 0;
err:
	spi_exit();

	if (devmem >= 0)
		close(devmem);

	close(fd);
	return ret;
}
