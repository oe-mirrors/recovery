#!/usr/bin/make -f

prefix ?= /usr/local
exec_prefix ?= $(prefix)
sbindir ?= $(exec_prefix)/sbin

TARGETS := flash-kernel flash-rescue flash-tarball recovery run-recovery select-boot-source

default:

install:
	install -d $(DESTDIR)$(sbindir)
	install -m 755 $(TARGETS) $(DESTDIR)$(sbindir)
