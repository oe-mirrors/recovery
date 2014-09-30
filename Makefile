#!/usr/bin/make -f

prefix ?= /usr/local
exec_prefix ?= $(prefix)
sbindir ?= $(exec_prefix)/sbin

TARGETS := backup-tarball flash-kernel flash-rescue flash-ssbl flash-tarball help librecovery recovery run-recovery select-boot-source to-the-rescue

default:

install:
	install -d $(DESTDIR)$(sbindir)
	install -m 755 $(TARGETS) $(DESTDIR)$(sbindir)
