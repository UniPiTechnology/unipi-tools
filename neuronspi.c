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

    arm_init(arm, "/dev/spidev0.1", 12000000, 0, NULL);

    idle_op(arm);

    uint16_t buffer[128];
    buffer[0] = 10;
    buffer[1] = 1;
    //write_regs(arm, 7, 2, buffer);

    int i, n;
    //for (i=0; i<10000; i++) {
        n = read_regs(arm, 0, 10, buffer);
    //}

    printf("cnt reg=%d\n",n);
    for (i=0; i<10; i++) {
       printf("%5d,",buffer[i]);
    }
    printf("\n");

    read_regs(arm, 1000, 7, buffer);
    for (i=0; i<7; i++) {
       printf("%5d,",buffer[i]);
    }
    printf("\n");

    
    //for (i=0; i<10000; i++) {
    //    if (read_regs(arm, 0, 20, buffer) <= 0 ) { printf("ERR reg 0 iter=%d\n",i); }
        /*if (read_regs(arm, 0, 50, buffer) <= 0 ) { printf("ERR reg 0 iter=%d\n",i); }
    //    if (read_regs(arm, 1000, 35, buffer) <= 0 ) { printf("ERR reg 1000 iter=%d\n",i); }*/
    //} 
    
    
    buffer[0] = 0x10;
    for (i=0; i<10000; i++) {
        //buffer[0] ^= 0x0f;
        buffer[0] >>= 1;
        write_regs(arm, 20, 1, buffer);
        if (buffer[0] == 0) buffer[0] = 0x10;
        usleep(50000);
    }
    
    buffer[0]=0x3;
    //write_bits(arm, 1, 2, (uint8_t*)buffer);
    // n = write_bit(arm, 1002, 1);
    //printf("b1 =%d\n",n);
    //n = write_bit(arm, 10, 1);
    //printf("b10 =%d\n",n);

    //n = read_bits(arm, 0, 16, (uint8_t*)buffer);
    //printf("cnt =%d  %5x \n",n, buffer[0]);

#if 0
/*  Test of pn_513 */
    buffer[0] = 0x1002;
    n = write_regs(arm, 100, 1, buffer);
    
    char s[256] = "\x00UU\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\xff\x03\xfd\xd4\x14\x01\x17\x00";
    uint8_t r[256]; 
    n = write_string(arm,0,s,26);
    printf("write %d\n",n);
    usleep(100000);
    n = read_string(arm,0, r, sizeof(r));
    printf("read %d\n",n);
    for (i=0; i<n; i++) {
       printf("%02x ",r[i]);
    }
    printf("\n");
#endif

    close(arm->fd);
    return 0;
}

