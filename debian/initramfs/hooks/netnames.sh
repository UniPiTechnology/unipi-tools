#!/bin/sh

mkdir -p ${DESTDIR}/etc/systemd/network
touch ${DESTDIR}/etc/systemd/network/99-default.link

. /usr/share/initramfs-tools/hook-functions
copy_exec /sbin/mke2fs
