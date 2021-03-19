#!/bin/sh

#
# Generate power-on for NB module via pin 19
#

PWR_KEY_PIN=19
RESET_PIN=18

echo ${RESET_PIN} > /sys/class/gpio/export
echo out > /sys/class/gpio/gpio${RESET_PIN}/direction
echo 0 > /sys/class/gpio/gpio${RESET_PIN}/value
echo ${PWR_KEY_PIN} > /sys/class/gpio/export
echo out > /sys/class/gpio/gpio${PWR_KEY_PIN}/direction
echo 0 > /sys/class/gpio/gpio${PWR_KEY_PIN}/value
echo 1 > /sys/class/gpio/gpio${PWR_KEY_PIN}/value
sleep 1
echo 0 > /sys/class/gpio/gpio${PWR_KEY_PIN}/value
sleep 1
