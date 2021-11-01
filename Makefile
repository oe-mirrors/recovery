#!/usr/bin/make -f
#
# Copyright (C) 2019 Dream Property GmbH, Germany
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
override CPPFLAGS := $(CPPFLAGS) -D_DEFAULT_SOURCE -DNDEBUG

SCRIPTS := apt-diff backup-settings backup-tarball \
           fbscaled flash-fsbl flash-kernel \
           flash-rescue flash-tarball help librecovery \
           restore-settings run-recovery \
           update-autoexec update-rescue
TARGETS := autoflashd recovery to-the-rescue
INITSCRIPTS := init.d/autoflashd init.d/fbscaled

default: $(TARGETS) $(INITSCRIPTS)

autoflashd: autoflashd.o
	$(LINK.c) $^ $(LOADLIBES) $(LDLIBS) $($(@)_LIBS) -o $@

init.d/%: %.init.in Makefile
	mkdir -p $(@D)
	sed -e 's,@sbindir@,$(sbindir),g' \
	    -e 's,@runstatedir@,$(runstatedir),g' \
	    < $< > $@

recovery: flash-tarball librecovery recovery.head recovery.tail
	cat recovery.head > $@
	tar --owner root --group root -cz flash-tarball librecovery | base64 >> $@
	cat recovery.tail >> $@
	chmod 755 recovery

to-the-rescue: io.o

clean:
	$(RM) $(wildcard $(TARGETS) $(INITSCRIPTS) *.o)

install: $(TARGETS) $(INITSCRIPTS)
	install -d $(DESTDIR)$(sysconfdir)/init.d
	install -m 755 $(INITSCRIPTS) $(DESTDIR)$(sysconfdir)/init.d
	install -d $(DESTDIR)$(sbindir)
	install -m 755 $(SCRIPTS) $(TARGETS) $(DESTDIR)$(sbindir)
	install -d $(DESTDIR)$(sysconfdir)/recovery/backup-hooks.d
	install -m 755 backup-hooks.d/*.sh $(DESTDIR)$(sysconfdir)/recovery/backup-hooks.d
