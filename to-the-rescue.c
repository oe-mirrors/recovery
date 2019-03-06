/*
 * to-the-rescue.c
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

#define _POSIX_C_SOURCE 199309L
#include <fcntl.h>
#include <stdio.h>
#include <signal.h>
#include <string.h>
#include <sys/types.h>
#include <unistd.h>
#include "io.h"

#define AON_ARRAY_BCM73625	0x00408200
#define AON_ARRAY_BCM7439	0x00410200

static bool proc(void)
{
	const char rescue[] = "rescue";
	int fd;

	fd = open("/proc/stb/fp/boot_mode", O_WRONLY);
	if (fd < 0)
		return false;

	write(fd, rescue, strlen(rescue));
	close(fd);
	return true;
}

static bool aon(void)
{
	unsigned int chip_id;
	unsigned long addr;
	volatile int *array;
	unsigned int index;
	size_t size;

	if (!detect_soc(&chip_id))
		return false;

	switch (chip_id) {
	case 0x73625:
		addr = BCM_PHYSICAL_OFFSET + AON_ARRAY_BCM73625;
		index = 120;
		size = 0x1000;
		break;
	case 0x7439:
		addr = BCM_PHYSICAL_OFFSET + AON_ARRAY_BCM7439;
		index = 244;
		size = 0x1000;
		break;
	case 0x29400a02:
		addr = 0xff6345e4;
		index = 0;
		size = 4;
		break;
	default:
		return false;
	}

	array = ioremap(addr, size);
	if (array == NULL)
		return false;

	array[index] = 0x12E5C00E;
	iounmap(array, size);
	return true;
}

int main(void)
{
	char *argv[2];

	if (!(proc() || aon())) {
		printf("Sorry, manual intervention required! Please execute 'reboot' manually,\n"
		       "then press the front panel power button and keep it pressed while your\n"
		       "Dreambox restarts, until the countdown on the front panel display has\n"
		       "elapsed.\n");
		return 1;
	}

	argv[0] = "reboot";
	argv[1] = NULL;
	return !!execve("/sbin/reboot", argv, NULL);
}
