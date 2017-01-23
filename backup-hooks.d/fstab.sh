#!/bin/sh
if [ -f "${SYSROOT}/etc/fstab" -a -d "${SYSROOT}/etc/enigma2" ]; then
	grep -v '^[a-zA-Z0-9_-]\+\s' "${SYSROOT}/etc/fstab" >"${SYSROOT}/etc/enigma2/fstab.bak"
fi
