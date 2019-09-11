#!/bin/sh

mkdir -p ${DESTDIR}/etc/systemd/network
touch ${DESTDIR}/etc/systemd/network/99-default.link

. /usr/share/initramfs-tools/hook-functions
rm ${DESTDIR}/usr/sbin/mke2fs
copy_exec /sbin/mke2fs
copy_exec /sbin/fdisk
copy_exec /sbin/sfdisk
copy_exec /sbin/mkfs.ext4


