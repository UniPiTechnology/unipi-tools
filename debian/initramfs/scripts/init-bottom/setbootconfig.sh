#!/bin/sh

[ -d /sys/firmware/devicetree/base/soc/i2c@7e804000/24c01@57 ] && NEURONEE=1
[ -d /sys/firmware/devicetree/base/soc/i2c@7e804000/24c02@50 ] && UNIPIEE=1
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

##mount boot
mkdir /tmp/boot
mount /dev/mmcblk0p1 /tmp/boot
(
#[ "`tail -c1 /tmp/boot/config.txt`" != "" ] && echo ""
[ -z "$I2C" ] && echo "dtparam=i2c_arm=on"
[ -z "$NEURONEE" ] && echo "dtoverlay=neuronee"
[ -z "$UNIPIEE" ] && echo "dtoverlay=unipiee"
if [ -n "$IS_UNIPI1" ]; then
  sed 's/^[[:blank:]]*\(dtoverlay[[:blank:]]*=[[:blank:]]*neuron-spi-new\)/#\1/' /tmp/boot/config.txt
else
  [ -z "$NEURONDRV" ] && echo "dtoverlay=neuron-spi-new"
  sed '/^[[:blank:]#;]*dtoverlay[[:blank:]]*=[[:blank:]]*neuron-spi-new/d' /tmp/boot/config.txt
fi
) >/tmp/boot/config.new

rm /tmp/boot/config.old 2>/dev/null
mv /tmp/boot/config.txt /tmp/boot/config.old
mv /tmp/boot/config.new /tmp/boot/config.txt

## umount boot
umount /tmp/boot
sync
## reboot
[ -z "$1" ] && reboot -f -n
