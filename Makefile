#!/usr/bin/make -f

prefix ?= /usr/local
exec_prefix ?= $(prefix)
sbindir ?= $(exec_prefix)/sbin

override CFLAGS := $(CFLAGS) -Wall -std=c99
override CPPFLAGS := $(CPPFLAGS) -DNDEBUG

SCRIPTS := backup-settings backup-tarball flash-fsbl flash-kernel flash-rescue flash-tarball help librecovery recovery run-recovery restore-settings select-boot-source
TARGETS := to-the-rescue writespi

default: $(TARGETS)

to-the-rescue: io.o

writespi: io.o

clean:
	$(RM) $(wildcard $(TARGETS) *.o)

install: $(TARGETS)
	install -d $(DESTDIR)$(sbindir)
	install -m 755 $(SCRIPTS) $(TARGETS) $(DESTDIR)$(sbindir)
	ln -nsf writespi $(DESTDIR)$(sbindir)/readspi
