#!/usr/bin/make -f
#
# Copyright (C) 2017 Dream Property GmbH, Germany
#                    https://dreambox.de/
#

prefix ?= /usr/local
exec_prefix ?= $(prefix)
sbindir ?= $(exec_prefix)/sbin
sysconfdir ?= $(prefix)/etc
localstatedir ?= $(prefix)/var
runstatedir ?= $(localstatedir)/run

autoflashd_CFLAGS := $(shell pkg-config --cflags libudev)
autoflashd_LIBS := $(shell pkg-config --libs libudev)

override CFLAGS := $(CFLAGS) $(autoflashd_CFLAGS) -Wall -std=c99
override CPPFLAGS := $(CPPFLAGS) -DNDEBUG

SCRIPTS := backup-settings backup-tarball flash-fsbl flash-kernel \
           flash-rescue flash-ssbl flash-tarball help librecovery \
           restore-settings run-recovery select-boot-source update-rescue
TARGETS := autoflashd recovery to-the-rescue writespi
INITSCRIPT := autoflashd.init

default: $(TARGETS) $(INITSCRIPT)

autoflashd: autoflashd.o
	$(LINK.c) $^ $(LOADLIBES) $(LDLIBS) $($(@)_LIBS) -o $@

autoflashd.init: autoflashd.init.in Makefile
	sed -e 's,@sbindir@,$(sbindir),g' \
	    -e 's,@runstatedir@,$(runstatedir),g' \
	    < $< > $@

recovery: flash-tarball librecovery recovery.head recovery.tail
	cat recovery.head > $@
	tar --owner root --group root -cz flash-tarball librecovery | base64 >> $@
	cat recovery.tail >> $@
	chmod 755 recovery

to-the-rescue: io.o

writespi: io.o

clean:
	$(RM) $(wildcard $(TARGETS) $(INITSCRIPT) *.o)

install: $(TARGETS)
	install -d $(DESTDIR)$(sysconfdir)/init.d
	install -m 755 $(INITSCRIPT) $(DESTDIR)$(sysconfdir)/init.d/autoflashd
	install -d $(DESTDIR)$(sbindir)
	install -m 755 $(SCRIPTS) $(TARGETS) $(DESTDIR)$(sbindir)
	install -d $(DESTDIR)$(sysconfdir)/recovery/backup-hooks.d
	install -m 755 backup-hooks.d/*.sh $(DESTDIR)$(sysconfdir)/recovery/backup-hooks.d
	ln -nsf writespi $(DESTDIR)$(sbindir)/readspi
