#!/bin/sh

. /usr/share/initramfs-tools/hook-functions
copy_exec /sbin/fsck
copy_exec /sbin/fsck.ext4
copy_exec /sbin/logsave
