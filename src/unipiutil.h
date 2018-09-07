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
#ifndef __unipiutil_h
#define __unipiutil_h

#include <sys/types.h>

char* get_unipi_name(void);
uint32_t get_unipi_serial(void);


#endif
