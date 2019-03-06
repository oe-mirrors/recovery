/*
 * io.c
 *
 * Copyright (C) 2019 Dream Property GmbH, Germany
 *                    https://dreambox.de/
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

#include <endian.h>
#include <fcntl.h>
#include <glob.h>
#include <limits.h>
#include <stdint.h>
#include <stdio.h>
#include <sys/mman.h>
#include <unistd.h>
#include "io.h"

static int devmem = -1;
static unsigned int users;

volatile void *ioremap(unsigned long phys, size_t length)
{
	unsigned long mask = sysconf(_SC_PAGESIZE) - 1;
	unsigned long addr = phys & ~mask;
	unsigned long offset = phys & mask;
	unsigned char *mem;

	if (devmem < 0) {
		devmem = open("/dev/mem", O_RDWR | O_SYNC);
		if (devmem < 0) {
			perror("/dev/mem");
			return NULL;
		}
	}

	mem = mmap(NULL, length + offset, PROT_READ | PROT_WRITE, MAP_SHARED, devmem, addr);
	if (mem == MAP_FAILED) {
		perror("mmap");
		return NULL;
	}

	users++;
	return &mem[offset];
}

void iounmap(volatile void *virt, size_t length)
{
	unsigned long mask = sysconf(_SC_PAGESIZE) - 1;
	unsigned long offset = (unsigned long)virt & mask;

	munmap((void *)virt - offset, length + offset);

	users--;
	if (users == 0) {
		close(devmem);
		devmem = -1;
	}
}

static bool io_read_binfile(const char *filename, void *buf, size_t count)
{
	ssize_t ret;
	int fd;

	fd = open(filename, O_RDONLY);
	if (fd < 0) {
		perror(filename);
		return false;
	}

	ret = read(fd, buf, count);
	if (ret < 0)
		perror("read");
	close(fd);

	return (size_t)ret == count;
}

static bool io_read_dt_reg(const char *path, const char *filename, uint64_t *addr, uint64_t *size)
{
	char joined[PATH_MAX];
	uint64_t buf[2];

	snprintf(joined, PATH_MAX, "%s/%s", path, filename);
	if (!io_read_binfile(joined, buf, sizeof(buf)))
		return false;

	*addr = be64toh(buf[0]);
	*size = be64toh(buf[1]);
	return true;
}

bool detect_soc(unsigned int *pchip_id)
{
	volatile unsigned long *mem;
	unsigned int chip_id;

	glob_t gl;
	glob("/sys/firmware/devicetree/base/soc/aobus@*", 0, NULL, &gl);
	if (gl.gl_pathc > 0) {
		uint64_t aobus_addr, aobus_size;
		uint64_t cpuver_offs, cpuver_size;

		if (!io_read_dt_reg(gl.gl_pathv[0], "reg", &aobus_addr, &aobus_size))
			return false;
		if (!io_read_dt_reg(gl.gl_pathv[0], "cpu_version/reg", &cpuver_offs, &cpuver_size))
			return false;
		if (aobus_size < cpuver_offs + cpuver_size || cpuver_size < 4)
			return false;

		mem = ioremap(aobus_addr + cpuver_offs, cpuver_size);
		if (mem == NULL) {
			fprintf(stderr, "Failed to map chip family register!\n");
			return false;
		}

		chip_id = *mem;
		iounmap(mem, cpuver_size);

		if (chip_id == 0x29400a02) {
			*pchip_id = chip_id;
			return true;
		}
	} if (access("/sys/devices/platform/brcmstb", F_OK) == 0) {
		mem = ioremap(BCM_PHYSICAL_OFFSET + BCM_CHIP_FAMILY_ID, 4);
		if (mem == NULL) {
			fprintf(stderr, "Failed to map chip family register!\n");
			return false;
		}

		chip_id = *mem;
		chip_id = (chip_id >> 28 ? chip_id >> 16 : chip_id >> 8);
		iounmap(mem, 4);

		if (chip_id == 0x73625 ||
		    chip_id == 0x7435 ||
		    chip_id == 0x7439) {
			*pchip_id = chip_id;
			return true;
		}
	}

	fprintf(stderr, "Unsupported SoC: %#x\n", chip_id);
	return false;
}
