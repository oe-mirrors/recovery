#!/usr/bin/make -f

prefix ?= /usr/local
exec_prefix ?= $(prefix)
sbindir ?= $(exec_prefix)/sbin

override CFLAGS := $(CFLAGS) -Wall -std=c99
override CPPFLAGS := $(CPPFLAGS) -DNDEBUG

SCRIPTS := backup-settings backup-tarball flash-fsbl flash-kernel \
           flash-rescue flash-ssbl flash-tarball help librecovery \
           restore-settings run-recovery select-boot-source update-rescue
TARGETS := recovery to-the-rescue writespi

default: $(TARGETS)

recovery: flash-tarball librecovery recovery.head recovery.tail
	cat recovery.head > $@
	tar --owner root --group root -cz flash-tarball librecovery | base64 >> $@
	cat recovery.tail >> $@
	chmod 755 recovery

to-the-rescue: io.o

writespi: io.o

clean:
	$(RM) $(wildcard $(TARGETS) *.o)

install: $(TARGETS)
	install -d $(DESTDIR)$(sbindir)
	install -m 755 $(SCRIPTS) $(TARGETS) $(DESTDIR)$(sbindir)
	ln -nsf writespi $(DESTDIR)$(sbindir)/readspi
