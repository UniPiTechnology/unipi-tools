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

#ifndef __nb_modbus_h
#define __nb_modbus_h

#include "modbus.h"
#include "armspi.h"

#define MAX_ARMS 3

typedef struct {
    modbus_t* ctx;
    arm_handle* arm[MAX_ARMS];
} nb_modbus_t;

nb_modbus_t*  nb_modbus_new_tcp(const char *ip_address, int port);
void nb_modbus_free(nb_modbus_t*  nb_ctx);
int nb_modbus_reqlen(uint8_t* data, uint8_t size);
int nb_modbus_reply(nb_modbus_t *nb_ctx, uint8_t *req, int req_length); 
int add_arm(nb_modbus_t*  nb_ctx, uint8_t index, const char *device, int speed, const char* gpio);

#endif
