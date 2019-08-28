#!/bin/sh

# exit if Axon platform
grep -q 'Hardware[[:blank:]]*:[[:blank:]]*BCM' /proc/cpuinfo || exit 0

unset IS_REAL_SYSTEM
unset DO_MOUNT

if [ "$1" = "--noreboot" ]; then
    ## called from package install
    MNTDIR=/boot
    # check if running from "real" Neuron system
    grep -q '^/dev/mmcblk[[:digit:]]p1 /boot ' /proc/mounts && IS_REAL_SYSTEM=1
else
    ## called from initramdisk
    IS_REAL_SYSTEM=1
    DO_MOUNT=1
    MNTDIR=/tmp/boot
fi
    
if [ "${IS_REAL_SYSTEM}" = "1" ]; then
  [ -d /sys/firmware/devicetree/base/soc/i2c@7e804000/24c01@57 ] && NEURONEE=1
  [ -d /sys/firmware/devicetree/base/soc/i2c@7e804000/24c02@50 ] && UNIPIEE=1
  [ -d /sys/firmware/devicetree/base/soc/i2c@7e804000/mcp7941x@6f ] && RTC=1
  [ -d /sys/firmware/devicetree/base/soc/spi@7e204000/neuronspi@0 ] && NEURONDRV=1
  grep -q okay /sys/firmware/devicetree/base/soc/spi@7e204000/status && SPI=1
  grep -q okay /sys/firmware/devicetree/base/soc/i2c@7e804000/status && I2C=1
  [ -r /sys/class/i2c-dev/i2c-1/subsystem/i2c-1/device/1-0050/eeprom ] && IS_UNIPI1=1

  if [ -n "$I2C" -a -n "$NEURONEE" -a -n "$UNIPIEE" ]; then
    if [ -n "$IS_UNIPI1" ]; then
        if [ -z "$NEURONDRV" ]; then
            echo OK
            exit 0
        fi
    else
        if [ -n "$NEURONDRV" ]; then
            echo OK
            exit 0
        fi
    fi
  fi
fi

if [ "${DO_MOUNT}" = "1" ]; then
  ##mount boot for running in initramdisk
  mkdir -p ${MNTDIR}
  mount /dev/mmcblk0p1  ${MNTDIR}
fi

(
[ -z "$I2C" ] && echo "dtparam=i2c_arm=on"
[ -z "$NEURONEE" ] && echo "dtoverlay=neuronee"
[ -z "$RTC" ] && echo "dtoverlay=i2c-rtc,mcp7941x"
[ -z "$UNIPIEE" ] && echo "dtoverlay=unipiee"
if [ -n "$IS_UNIPI1" ]; then
  sed 's/^[[:blank:]]*\(dtoverlay[[:blank:]]*=[[:blank:]]*neuron-spi-new\)/#\1/'  ${MNTDIR}/config.txt
else
  [ -z "$NEURONDRV" ] && echo "dtoverlay=neuron-spi-new"
  sed '/^[[:blank:]#;]*dtoverlay[[:blank:]]*=[[:blank:]]*neuron-spi-new/d' ${MNTDIR}/config.txt
fi
) >${MNTDIR}/config.new

rm -f ${MNTDIR}/config.old 2>/dev/null
mv ${MNTDIR}/config.txt ${MNTDIR}/config.old
mv ${MNTDIR}/config.new ${MNTDIR}/config.txt

if [ "${DO_MOUNT}" = "1" ]; then
  ## umount boot
  umount ${MNTDIR}
  rmdir  ${MNTDIR}
fi
sync

## reboot
[ -z "$1" ] && reboot -f -n
exit 0
