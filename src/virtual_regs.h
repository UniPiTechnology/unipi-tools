/*
 * SPI communication with UniPi Neuron and Axon families of controllers
 *
 * Copyright (c) 2016  Faster CZ, ondra@faster.cz
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License.
 *
 */
#ifndef __virtual_regs_h
#define __virtual_regs_h

#include <stdint.h>
#include "armutil.h"
#include "armspi.h"

#define OFFSET_V_REGS 3000
#define OFFSET_PV_REGS 4000
#define OFFSET_PV_STORSTAT_GROUP 0
#define OFFSET_PV_LTE_GROUP 100
#define OFFSET_PV_SYSSTAT_GROUP 200

#define VIRTUAL_COILS_NANOPI 1
#define VIRTUAL_COILS_ZULU 2

int read_virtual_regs(arm_handle* arm, uint16_t reg, uint8_t cnt, uint16_t* result);
int write_virtual_regs(arm_handle* arm, uint16_t reg, uint8_t cnt, uint16_t* values);
int read_virtual_bits(arm_handle* arm, uint16_t reg, uint16_t cnt, uint8_t* result);
int write_virtual_bit(arm_handle* arm, uint16_t reg, uint8_t value, uint8_t do_lock);
int write_virtual_bits(arm_handle* arm, uint16_t reg, uint16_t cnt, uint8_t* values);
void monitor_virtual_regs(arm_handle* arm, uint16_t reg, uint16_t* result);

void monitor_virtual_coils(arm_handle* arm, uint16_t reg, uint8_t* values, uint16_t cnt, int platform);

int read_pure_virtual_regs(uint16_t reg, uint8_t cnt, uint16_t* result);


#endif
