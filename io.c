/*
 * io.c
 *
 * Copyright (C) 2016 Dream Property GmbH, Germany
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

#include <fcntl.h>
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

bool detect_soc(unsigned int *pchip_id)
{
	volatile unsigned long *family_id;
	unsigned int chip_id;

	family_id = ioremap(BCM_PHYSICAL_OFFSET + BCM_CHIP_FAMILY_ID, 4);
	if (family_id == NULL) {
		fprintf(stderr, "Failed to map chip family register!\n");
		return false;
	}

	chip_id = *family_id;
	chip_id = (chip_id >> 28 ? chip_id >> 16 : chip_id >> 8);
	iounmap(family_id, 4);

	if (chip_id == 0x73625 ||
	    chip_id == 0x7435 ||
	    chip_id == 0x7439) {
		*pchip_id = chip_id;
		return true;
	}

	fprintf(stderr, "Unsupported SoC: %#x\n", chip_id);
	return false;
}
