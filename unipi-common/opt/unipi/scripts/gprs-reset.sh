#!/bin/sh

#
# Generate reset for GPRS modul via pin 18
#

PIN=18

echo ${PIN} >/sys/class/gpio/export
echo out > /sys/class/gpio/gpio${PIN}/direction
echo 1 > /sys/class/gpio/gpio${PIN}/value
echo 0 > /sys/class/gpio/gpio${PIN}/value
sleep 1
echo 1 > /sys/class/gpio/gpio${PIN}/value
sleep 1
