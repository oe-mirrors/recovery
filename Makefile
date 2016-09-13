#!/usr/bin/make -f

prefix ?= /usr/local
exec_prefix ?= $(prefix)
sbindir ?= $(exec_prefix)/sbin

override CFLAGS := $(CFLAGS) -Wall -std=c99
override CPPFLAGS := $(CPPFLAGS) -DNDEBUG

SCRIPTS := backup-tarball flash-kernel flash-rescue flash-tarball help librecovery recovery run-recovery to-the-rescue
TARGETS := writespi

default: $(TARGETS)

writespi: writespi.c

clean:
	$(RM) $(TARGETS)

install: $(TARGETS)
	install -d $(DESTDIR)$(sbindir)
	install -m 755 $(SCRIPTS) $(TARGETS) $(DESTDIR)$(sbindir)
