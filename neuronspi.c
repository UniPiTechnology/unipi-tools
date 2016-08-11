/*
 * SPI communication with UniPi Neuron family controllers
 *
 * Copyright (c) 2016  Faster CZ, ondra@faster.cz
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License.
 *
 */

#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <getopt.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>
#include <string.h>
#include "armspi.h"



int main(int argc, char *argv[])
{
    arm_handle* arm = malloc(sizeof(arm_handle));

    arm_init(arm, "/dev/spidev0.1", 12000000);

    idle_op(arm);

    uint16_t buffer[10];
    buffer[0] = 10;
    buffer[1] = 1;
    write_regs(arm, 7, 2, buffer);

    int i;
    //for (i=0; i<1000; i++) {
    read_regs(arm, 0, 10, buffer);
    //}

    for (i=0; i<10; i++) {
       printf("%5d,",buffer[i]);
    }
    printf("\n");

    read_regs(arm, 1000, 5, buffer);
    for (i=0; i<5; i++) {
       printf("%5d,",buffer[i]);
    }
    printf("\n");

    buffer[0]=0x3;
    write_bits(arm, 1, 2, (uint8_t*)buffer);

    read_bits(arm, 0, 16, (uint8_t*)buffer);
    printf("%5x\n",buffer[0]);

    close(arm->fd);
    return 0;
}
