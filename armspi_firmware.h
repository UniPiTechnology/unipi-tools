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
#ifndef __armspi_h
#define __armspi_h

#include <stdint.h>
#include <linux/spi/spidev.h>
#include "armutil.h"


#define ARM_PAGE_SIZE      1024
#define ARM_FIRMWARE_KEY   0xAA99FF33
typedef struct {
    uint32_t  address;
    uint8_t   data[ARM_PAGE_SIZE];
    uint16_t  crc;
} __attribute__((packed)) arm_comm_firmware;



void* start_firmware(arm_handle* arm);
int send_firmware(void* ctx, uint8_t* data, size_t datalen, uint32_t start_address);
void finish_firmware(void*  ctx);


#endif
