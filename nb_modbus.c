/*
 * Non-blocking version Modbus/Tcp - server(slave) version only
 *
 * Copyright (c) 2016  Faster CZ, ondra@faster.cz
 * Copyright © 2001-2011 Stéphane Raimbault <stephane.raimbault@gmail.com>
 *
 * SPDX-License-Identifier: LGPL-2.1+
 *
 * This library implements the Modbus protocol.
 * http://libmodbus.org/
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdarg.h>
#include <errno.h>
#include <limits.h>
#include <time.h>
#include <unistd.h>

#include <config.h>

#include "modbus.h"
#include "modbus-private.h"
#include "modbus-tcp-private.h"

#include "nb_modbus.h"
#include "armspi.h"


/* Internal use */
#define MSG_LENGTH_UNDEFINED -1

/* Max between RTU and TCP max adu length (so TCP) */
#define MAX_MESSAGE_LENGTH 260


int nb_modbus_reqlen(uint8_t* data, uint8_t size)
{
    if (size < 6) return 0;
    int len = (data[4] << 8) + data[5] + 6;
    if (size < len) return 0;
    return len;
}


/* Build the exception response */
static int nb_response_exception(modbus_t *ctx, int exception_code, uint8_t *rsp,
                              const char* template, ...)
{
    int rsp_length;
    
    /* Print debug message */
    if (ctx->debug) {
        va_list ap;

        va_start(ap, template);
        vfprintf(stderr, template, ap);
        va_end(ap);
    }

    /* Flush if required */
    //if (to_flush) {
    //    _sleep_response_timeout(ctx);
    //    modbus_flush(ctx);
    //}

    int offset = ctx->backend->header_length;
    /* Build exception response */
    rsp[offset] = rsp[offset] + 0x80;
    rsp_length = _MODBUS_TCP_PRESET_RSP_LENGTH;
    rsp[rsp_length++] = exception_code;

    /* Substract the header length to the message length */
    int mbap_length = rsp_length - 6;

    rsp[4] = mbap_length >> 8;
    rsp[5] = mbap_length & 0x00FF;

    return rsp_length;
}


/* Send a response to the received request.
   Analyses the request and constructs a response.

   If an error occurs, this function construct the response
   accordingly.
*/
int nb_modbus_reply(nb_modbus_t *nb_ctx, uint8_t *req, int req_length) //, arm_handle* arm)
{
    int offset;
    int slave;
    int function;
    uint16_t address;
    uint8_t* rsp = req;
    arm_handle* arm;
    int rsp_length = 0;

    if (nb_ctx == NULL) {
        errno = EINVAL;
        return -1;
    }

    offset = nb_ctx->ctx->backend->header_length;
    slave = req[offset - 1];
    function = req[offset];
    address = (req[offset + 1] << 8) + req[offset + 2];
    rsp_length = _MODBUS_TCP_PRESET_RSP_LENGTH;
    if (slave == 0) {
        if (address < 1000) {
            slave = address / 100 + 1;
            address = address % 100;
        } else if (address < 2000) {
            slave = (address-1000) / 100 + 1;
            address = (address-1000) % 100;
        } else if (address < 3000) {
            slave = (address-2000) / 100 + 1;
            address = (address-2000) % 100;
        }
    }    
    if (slave < MAX_ARMS) {
        arm = nb_ctx->arm[slave-1];
    } else {
        return nb_response_exception(
            nb_ctx->ctx, MODBUS_EXCEPTION_GATEWAY_TARGET, rsp, 
                    "Illegal slave address 0x%0X\n", slave);
    }
    
    switch (function) {
    case MODBUS_FC_READ_COILS:
    case MODBUS_FC_READ_DISCRETE_INPUTS: {
        int nb = (req[offset + 3] << 8) + req[offset + 4];
        if (nb < 1 || MODBUS_MAX_READ_BITS < nb) {
            rsp_length = nb_response_exception(
                nb_ctx->ctx, MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE, rsp,
                "Illegal nb of values %d in read_bits (max %d)\n", nb, MODBUS_MAX_READ_BITS);
        } else {
            rsp[rsp_length++] = (nb / 8) + ((nb % 8) ? 1 : 0);
            int n = read_bits(arm, address, nb, rsp+rsp_length); 
            if (n >= nb) {
                rsp_length += (nb / 8) + ((nb % 8) ? 1 : 0);
            } else {
                rsp_length = nb_response_exception(
                    nb_ctx->ctx, MODBUS_EXCEPTION_ILLEGAL_DATA_ADDRESS, rsp, 
                    "Illegal data address 0x%0X in read_bits\n", address);
            }
        }
        break;
    }

    case MODBUS_FC_READ_HOLDING_REGISTERS:
    case MODBUS_FC_READ_INPUT_REGISTERS: {
        int nb = (req[offset + 3] << 8) + req[offset + 4];
        if (nb < 1 || MODBUS_MAX_READ_REGISTERS < nb) {
            rsp_length = nb_response_exception(
                nb_ctx->ctx, MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE, rsp, 
                "Illegal nb of values %d in read_register (max %d)\n", nb, MODBUS_MAX_READ_REGISTERS);
        } else {
            int i;
            uint8_t c;

            rsp[rsp_length++] = nb << 1;
            int n = read_regs(arm, address, nb, (uint16_t*) (rsp+rsp_length));
            if (n == nb) {
                for (i = address; i < address + nb; i++) {
                    c = rsp[rsp_length++];
                    rsp[rsp_length-1] = rsp[rsp_length];
                    rsp[rsp_length++] = c;
                }
            } else {
                rsp_length = nb_response_exception(
                    nb_ctx->ctx, MODBUS_EXCEPTION_ILLEGAL_DATA_ADDRESS, rsp,
                    "Illegal data address 0x%0X in read_register\n", address);
            }
        }
        break;
    }
    case MODBUS_FC_WRITE_SINGLE_COIL: {
        int data = (req[offset + 3] << 8) + req[offset + 4];

        if ((data != 0xFF00) && (data != 0x0)) {
            rsp_length = nb_response_exception(
                nb_ctx->ctx, MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE, rsp, FALSE,
                "Illegal data value 0x%0X in write_bit request at address %0X\n", data, address);
        } else {
            int n = write_bit(arm, address, data ? 1 : 0);
            if (n == 1) {
                rsp_length += 4; // = req_length;
            } else {
                rsp_length = nb_response_exception(
                    nb_ctx->ctx, MODBUS_EXCEPTION_SLAVE_OR_SERVER_FAILURE, rsp,
                    "Illegal data address 0x%0X in write_bit\n", address);
            }
        }
        break;
    }
    case MODBUS_FC_WRITE_SINGLE_REGISTER: {
        uint16_t data = (req[offset + 3] << 8) + req[offset + 4];

        int n = write_regs(arm, address, 1, &data);
        if (n == 1) {
            rsp_length += 4; // = req_length;
        } else {
            rsp_length = nb_response_exception(
                    nb_ctx->ctx, MODBUS_EXCEPTION_ILLEGAL_DATA_ADDRESS, rsp,
                    "Illegal data address 0x%0X in write_single_register\n", address);
        }
        break;
    }
    case MODBUS_FC_WRITE_MULTIPLE_COILS: {
        int nb = (req[offset + 3] << 8) + req[offset + 4];

        if (nb < 1 || MODBUS_MAX_WRITE_BITS < nb) {
            rsp_length = nb_response_exception(
                nb_ctx->ctx, MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE, rsp,
                "Illegal number of values %d in write_bits (max %d)\n", nb, MODBUS_MAX_WRITE_BITS);
        } else if (address < 0 ) {
        } else {
            /* 6 = byte count */
            int n = write_bits(arm, address, nb, rsp+rsp_length + 5);
            if ( n == nb ) {
                rsp_length += 4;
            } else {
                rsp_length = nb_response_exception(
                    nb_ctx->ctx, MODBUS_EXCEPTION_ILLEGAL_DATA_ADDRESS, rsp,
                    "Illegal data address 0x%0X in write_bits\n",  address);
            }
        }
    }
        break;
    case MODBUS_FC_WRITE_MULTIPLE_REGISTERS: {
        int nb = (req[offset + 3] << 8) + req[offset + 4];

        if (nb < 1 || MODBUS_MAX_WRITE_REGISTERS < nb) {
            rsp_length = nb_response_exception(
                nb_ctx->ctx, MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE, rsp,
                "Illegal number of values %d in write_registers (max %d)\n",
                nb, MODBUS_MAX_WRITE_REGISTERS);
        } else {
            int i, j;
            uint8_t c;   
            for (i = 0, j = rsp_length+5; i < nb; i++, j += 2) {
                c = rsp[j];
                rsp[j] = rsp[j+1];
                rsp[j+1] = c;
            }

            int n = write_regs(arm, address, nb, (uint16_t*)(rsp + rsp_length + 5));
            if (n == nb) {
                rsp_length += 4; // = req_length;
            } else {
                rsp_length = nb_response_exception(
                    nb_ctx->ctx, MODBUS_EXCEPTION_ILLEGAL_DATA_ADDRESS, rsp,
                    "Illegal data address 0x%0X in write_registers\n", address);
            }
        }
    }
        break;
    case MODBUS_FC_REPORT_SLAVE_ID: {
        int str_len;
        int byte_count_pos;

        /* Skip byte count for now */
        byte_count_pos = rsp_length++;
        rsp[rsp_length++] = _REPORT_SLAVE_ID;
        /* Run indicator status to ON */
        rsp[rsp_length++] = 0xFF;
        /* LMB + length of LIBMODBUS_VERSION_STRING */
        str_len = 3 + strlen(LIBMODBUS_VERSION_STRING);
        memcpy(rsp + rsp_length, "LMB" LIBMODBUS_VERSION_STRING, str_len);
        rsp_length += str_len;
        rsp[byte_count_pos] = rsp_length - byte_count_pos - 1;
    }
        break;

    default:
        rsp_length = nb_response_exception(
            nb_ctx->ctx, MODBUS_EXCEPTION_ILLEGAL_FUNCTION, rsp, 
            "Unknown Modbus function code: 0x%0X\n", function);
        break;
    }

    /* Substract the header length to the message length */
    int mbap_length = rsp_length - 6;

    rsp[4] = mbap_length >> 8;
    rsp[5] = mbap_length & 0x00FF;

    return rsp_length;
}


nb_modbus_t*  nb_modbus_new_tcp(const char *ip_address, int port)
{
    modbus_t* ctx = modbus_new_tcp(ip_address, port);
    if (ctx == NULL) return NULL;
    
    nb_modbus_t* nb_ctx = calloc(1, sizeof(nb_modbus_t));
    if (nb_ctx == NULL) {
        modbus_free(ctx);
        return NULL;
    }
    nb_ctx->ctx = ctx;
}


void nb_modbus_free(nb_modbus_t*  nb_ctx)
{
    if (nb_ctx != NULL) {
        int i;
        modbus_free(nb_ctx->ctx);
        for (i=0; i<MAX_ARMS; i++) {
            if (nb_ctx->arm[i] != NULL) {
                close(nb_ctx->arm[i]->fd);
                if (nb_ctx->arm[i]->fdint >= 0) 
                    close(nb_ctx->arm[i]->fdint);
                free(nb_ctx->arm[i]);
            }
        }
        free(nb_ctx);
    }
}


int add_arm(nb_modbus_t*  nb_ctx, uint8_t index, const char *device, int speed, const char* gpio)
{
    if (index >= MAX_ARMS) 
        return -1;

    arm_handle* arm = calloc(1, sizeof(arm_handle));

    if (arm == NULL) 
        return -1;

    arm_init(arm, device, speed, index, gpio);
    nb_ctx->arm[index] = arm;
}
