#!/bin/sh

# disable warning in case of initramdisk generation
[ -n "${DESTDIR}" ] && exec : 2>/dev/null

if [ -c /dev/rtc1 ]; then
	hwclock --hctosys -f /dev/rtc1 || true
else
	hwclock --hctosys || true
fi
