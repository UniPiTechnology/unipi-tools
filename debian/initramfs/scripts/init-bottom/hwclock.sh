#!/bin/sh

if [ -c /dev/rtc1 ]; then
	hwclock --hctosys -f /dev/rtc1
else
	hwclock --hctosys
fi
