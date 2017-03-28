/*
 * autoflashd.c
 *
 * Copyright (C) 2017 Dream Property GmbH, Germany
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

#define _POSIX_C_SOURCE 200809L
#include <fcntl.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/epoll.h>
#include <sys/mount.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <unistd.h>
#include <libudev.h>

static void handle_image(const char *path)
{
	pid_t pid;
	int status;

	printf("%s(%s)\n", __func__, path);

	pid = fork();
	if (pid == -1) {
		perror("fork");
		return;
	}

	if (pid == 0) {
		const char fifo[] = "/run/recovery-ui.fifo";
		struct stat st;
		int fd;

		if (stat(fifo, &st) < 0 || !S_ISFIFO(st.st_mode)) {
			unlink(fifo);
			if (mkfifo(fifo, 0600) < 0)
				perror("mkfifo");
		}

		fd = open(fifo, O_WRONLY);
		if (fd >= 0) {
			close(STDOUT_FILENO);
			close(STDERR_FILENO);
			dup2(fd, STDOUT_FILENO);
			dup2(fd, STDERR_FILENO);
		}

		execlp("flash-tarball", "flash-tarball", path, (char *)NULL);
		perror("execlp");
		exit(1);
	}

	if (waitpid(pid, &status, 0) < 0) {
		perror("waitpid");
		return;
	}

	if (WIFEXITED(status))
		printf("Child exited with code %d\n", WEXITSTATUS(status));
	else if (WIFSIGNALED(status))
		fprintf(stderr, "Child received signal %d\n", WTERMSIG(status));
}

static void handle_mountpoint(const char *mountpoint)
{
	const char modelpath[] = "/proc/stb/info/model";
	char path[FILENAME_MAX];
	char machine[16];
	ssize_t ret;
	int fd;

	fd = open(modelpath, O_RDONLY | O_CLOEXEC);
	if (fd < 0) {
		perror(modelpath);
		return;
	}

	ret = read(fd, machine, sizeof(machine));
	if (ret < 0) {
		perror("read");
		return;
	}
	if (ret == 0)
		return;
	if (machine[ret - 1] != '\n')
		return;
	machine[ret - 1] = '\0';

	snprintf(path, sizeof(path), "%s/dreambox-image-%s.tar.xz", mountpoint, machine);
	if (access(path, R_OK) == 0)
		handle_image(path);
}

static void handle_mount(const char *source, const char *target, const char *fstype)
{
	printf("%s(%s, %s, %s)\n", __func__, source, target, fstype);

	if (mount(source, target, fstype, MS_RDONLY, NULL) < 0) {
		perror("mount");
		return;
	}

	handle_mountpoint(target);

	umount2(target, MNT_DETACH | UMOUNT_NOFOLLOW);
}

static void handle_device(struct udev_device *device)
{
	const char *action, *devnode, *label, *type;
	char template[] = "/tmp/autoflash.XXXXXX";

	action = udev_device_get_action(device);
	if (action == NULL || strcmp(action, "add"))
		return;

	devnode = udev_device_get_devnode(device);
	if (devnode == NULL || strncmp(devnode, "/dev/", 5))
		return;

	type = udev_device_get_property_value(device, "ID_FS_TYPE");
	if (type == NULL)
		return;

	label = udev_device_get_property_value(device, "ID_FS_LABEL");
	if (label && !strcmp(label, "DREAMFLASH")) {
		const char *dir = mkdtemp(template);
		handle_mount(devnode, dir, type);
		rmdir(dir);
	}
}

static void do_trigger(struct udev *udev, const char *action)
{
	struct udev_enumerate *enumerate;
	struct udev_list_entry *list_entry;
	size_t len = strlen(action);
	char path[FILENAME_MAX];
	int fd;

	enumerate = udev_enumerate_new(udev);
	if (enumerate == NULL)
		return;

	udev_enumerate_add_match_subsystem(enumerate, "block");
	udev_enumerate_scan_devices(enumerate);

	udev_list_entry_foreach(list_entry, udev_enumerate_get_list_entry(enumerate)) {
		snprintf(path, FILENAME_MAX, "%s/uevent", udev_list_entry_get_name(list_entry));
		fd = open(path, O_WRONLY | O_CLOEXEC);
		if (fd >= 0) {
			write(fd, action, len);
			close(fd);
		}
	}

	udev_enumerate_unref(enumerate);
}

static void do_monitor(struct udev_monitor *monitor)
{
	struct udev_device *device;
	struct epoll_event ev;
	int epoll;
	int fd;

	udev_monitor_filter_add_match_subsystem_devtype(monitor, "block", NULL);
	fd = udev_monitor_get_fd(monitor);
	udev_monitor_enable_receiving(monitor);

	epoll = epoll_create1(EPOLL_CLOEXEC);
	ev.events = EPOLLIN;
	ev.data.fd = fd;
	epoll_ctl(epoll, EPOLL_CTL_ADD, fd, &ev);

	do_trigger(udev_monitor_get_udev(monitor), "add");

	for (;;) {
		if (epoll_wait(epoll, &ev, 1, -1) < 0) {
			perror("epoll_wait");
			break;
		}

		if (ev.events & EPOLLIN) {
			device = udev_monitor_receive_device(monitor);
			if (device != NULL)
				handle_device(device);
		}
	}

	epoll_ctl(epoll, EPOLL_CTL_DEL, fd, NULL);
	close(epoll);
}

int main(void)
{
	struct udev *udev;
	struct udev_monitor *monitor;

	udev = udev_new();
	if (udev == NULL)
		return 1;

	monitor = udev_monitor_new_from_netlink(udev, "udev");
	if (monitor == NULL)
		return 1;

	do_monitor(monitor);

	udev_monitor_unref(monitor);
	udev_unref(udev);
	return 0;
}
