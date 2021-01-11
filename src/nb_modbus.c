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
#include <fcntl.h>
#include <sys/types.h>
#include <sys/mman.h>
#include <sys/stat.h>

#include "nb_modbus.h"
#include "armspi.h"
#include "armutil.h"
#include "virtual_regs.h"

#include <modbus/modbus-version.h>
#if LIBMODBUS_VERSION_CHECK(3,1,4) != 1
//Library_error "YOU NEED libmodbus version min 3.1.4"
#endif

int deferred_op = DFR_NONE;
arm_handle*  deferred_arm;


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
    if (verbose > 1) {
        va_list ap;

        va_start(ap, template);
        vfprintf(stderr, template, ap);                
        va_end(ap);
    }
    //int offset = ctx->backend->header_length;
    int offset = _MODBUS_TCP_HEADER_LENGTH;
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
int nb_modbus_reply(nb_modbus_t *nb_ctx, uint8_t *req, int req_length, int broadcast_address)
{
    int offset;
    int slave;
    int function;
    int n;
    uint16_t address;
    uint8_t* rsp = req;
    arm_handle* arm;
    int rsp_length = 0;

    if (nb_ctx == NULL) {
        errno = EINVAL;
        return -1;
    }

    //offset = nb_ctx->ctx->backend->header_length;
    offset = _MODBUS_TCP_HEADER_LENGTH;
    slave = req[offset - 1];
    if (slave == broadcast_address) {
    	slave = 0;
    }
    function = req[offset];
    address = (req[offset + 1] << 8) + req[offset + 2];

    rsp_length = _MODBUS_TCP_PRESET_RSP_LENGTH;
    if (slave == 0) {
        if (address < 1000) {
            slave = address / 100 + 1;
            address = address % 100;
        } else if (address < 2000) {
            slave = (address-1000) / 100 + 1;
            address = (address-1000) % 100 + 1000;
        } else if (address < 3000) {
            slave = (address-2000) / 100 + 1;
            address = (address-2000) % 100 + 2000;
        } else {
            slave = 1;
        }
    }
    if (slave <= MAX_ARMS) {
        arm = nb_ctx->arm[slave-1];
    } else {
        arm = NULL;
    }
    /*if (arm == NULL) {
        return nb_response_exception(
            nb_ctx->ctx, MODBUS_EXCEPTION_GATEWAY_TARGET, rsp,
                    "Illegal slave address 0x%0X\n", slave);
    }*/

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
            } else if (n < 0) {
            	rsp_length = nb_response_exception(
                        nb_ctx->ctx, MODBUS_EXCEPTION_SLAVE_OR_SERVER_FAILURE, rsp,
                        "Illegal data value 0x%0X in read_bits\n", address);
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
            if ((address >= 3000) && (address < 4000)) {
                n = read_virtual_regs(arm, address, nb, (uint16_t*) (rsp+rsp_length));
            } else {
                n = read_regs(arm, address, nb, (uint16_t*) (rsp+rsp_length));
            }
            if (n == nb) {
                for (i = address; i < address + nb; i++) {
                    c = rsp[rsp_length++];
                    rsp[rsp_length-1] = rsp[rsp_length];
                    rsp[rsp_length++] = c;
                }
            } else if (n < 0) {
            	rsp_length = nb_response_exception(
                        nb_ctx->ctx, MODBUS_EXCEPTION_SLAVE_OR_SERVER_FAILURE, rsp,
                        "Illegal data value 0x%0X in read_registers\n", address);
            } else {
                rsp_length = nb_response_exception(
                    nb_ctx->ctx, MODBUS_EXCEPTION_ILLEGAL_DATA_ADDRESS, rsp,
                    "Illegal data address 0x%0X in read_registers\n", address);
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
        } else if((address >= 1004) && (address <= 1006)) {
            rsp_length = nb_response_exception(
                nb_ctx->ctx, MODBUS_EXCEPTION_ILLEGAL_DATA_ADDRESS, rsp,
                "Illegal data address 0x%0X in write_coil\n", address);
        } else {
            n = write_bit(arm, address, data ? 1 : 0, 0);
            if (arm && arm->has_virtual_coils && (address == 1001)) {
                data = data ? 1 : 0;
                monitor_virtual_coils(arm, address, (uint8_t*)(&data), 1); // monitoring coil changes
            }
            if (n == 1) {
                rsp_length += 4; // = req_length;
            } else if (n < 0) {
            	rsp_length = nb_response_exception(
                        nb_ctx->ctx, MODBUS_EXCEPTION_SLAVE_OR_SERVER_FAILURE, rsp,
                        "Illegal data value 0x%0X in write_coil\n", address);
            } else {
                rsp_length = nb_response_exception(
                    nb_ctx->ctx, MODBUS_EXCEPTION_ILLEGAL_DATA_ADDRESS, rsp,
                    "Illegal data address 0x%0X in write_coil\n", address);
            }
        }
        break;
    }
    case MODBUS_FC_WRITE_SINGLE_REGISTER: {
        uint16_t data = (req[offset + 3] << 8) + req[offset + 4];

        if ((address >= 3000) && (address < 4000)) {
            n = write_virtual_regs(arm, address, 1, &data);
        } else {
            n = write_regs(arm, address, 1, &data);
        }
        if (n == 1) {
            rsp_length += 4; // = req_length;
            if ((address == 1019) || (address==1024)) {    // monitoring register changes
                monitor_virtual_regs(arm, address, &data); 
            }
        } else if (n < 0) {
        	rsp_length = nb_response_exception(
                    nb_ctx->ctx, MODBUS_EXCEPTION_SLAVE_OR_SERVER_FAILURE, rsp,
                    "Illegal data value 0x%0X in write_single_register\n", address);
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
                "Illegal number of values %d in write_coils (max %d)\n", nb, MODBUS_MAX_WRITE_BITS);
        } else if (address < 0 || ((address <= 1006) && (address + nb > 1004))) {
            rsp_length = nb_response_exception(
                nb_ctx->ctx, MODBUS_EXCEPTION_ILLEGAL_DATA_ADDRESS, rsp,
                "Illegal data address 0x%0X in write_coils\n", address);
        } else {
            /* 6 = byte count */
            n = write_bits(arm, address, nb, rsp+rsp_length + 5);
            if ( n == nb ) {
                if (arm && arm->has_virtual_coils && (address <= 1001) && (address+nb > 1001)) {
                    monitor_virtual_coils(arm, address, rsp+rsp_length + 5, nb); // monitoring coil changes
                }
                rsp_length += 4;
            } else if (n < 0) {
            	rsp_length = nb_response_exception(
                        nb_ctx->ctx, MODBUS_EXCEPTION_SLAVE_OR_SERVER_FAILURE, rsp,
                        "Illegal data value 0x%0X in write_coils\n", address);
            } else {
                rsp_length = nb_response_exception(
                    nb_ctx->ctx, MODBUS_EXCEPTION_ILLEGAL_DATA_ADDRESS, rsp,
                    "Illegal data address 0x%0X in write_coils\n",  address);
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

            if ((address >= 3000) && (address < 4000)) {
                n = write_virtual_regs(arm, address, nb, (uint16_t*)(rsp + rsp_length + 5));
            } else {
                n = write_regs(arm, address, nb, (uint16_t*)(rsp + rsp_length + 5));
            }
            if (n == nb) {
                if ((address <= 1019)&&(address+nb>1019)) {
                    monitor_virtual_regs(arm, 1019, (uint16_t*)(rsp + rsp_length + 5 + (1019-address))); // monitoring register changes
                }
                if ((address <= 1024)&&(address+nb>1024)) {
                    monitor_virtual_regs(arm, 1024, (uint16_t*)(rsp + rsp_length + 5 + (1024-address))); // monitoring register changes
                }
                rsp_length += 4; // = req_length;
            } else if (n < 0) {
            	rsp_length = nb_response_exception(
                        nb_ctx->ctx, MODBUS_EXCEPTION_SLAVE_OR_SERVER_FAILURE, rsp,
                        "Illegal data value in write_registers %d\n", address);
            } else {
                rsp_length = nb_response_exception(
                    nb_ctx->ctx, MODBUS_EXCEPTION_ILLEGAL_DATA_ADDRESS, rsp,
                    "Illegal data address %d in write_registers\n", address);
            }
        }
    }
        break;
    case MODBUS_FC_REPORT_SLAVE_ID: {
        int str_len;
        int byte_count_pos;

        /* Skip byte count for now */
        byte_count_pos = rsp_length++;
        rsp[rsp_length++] = _REPORT_MB_SLAVE_ID;
        /* Run indicator status to ON */
        rsp[rsp_length++] = 0xFF;
        /* LMB + length of LIBMODBUS_VERSION_STRING */
        str_len = 3 + strlen(LIBMODBUS_VERSION_STRING);
        memcpy(rsp + rsp_length, "SPI" LIBMODBUS_VERSION_STRING, str_len);
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


nb_modbus_t* nb_modbus_new_tcp(const char *ip_address, int port)
{
    modbus_t* ctx = modbus_new_tcp(ip_address, port);
    if (ctx == NULL) return NULL;

    nb_modbus_t* nb_ctx = calloc(1, sizeof(nb_modbus_t));
    if (nb_ctx == NULL) {
        modbus_free(ctx);
        return NULL;
    }
    nb_ctx->ctx = ctx;
    return nb_ctx;
}


void nb_modbus_free(nb_modbus_t*  nb_ctx)
{
    if (nb_ctx != NULL) {
        int i;
        modbus_free(nb_ctx->ctx);
        for (i=0; i<MAX_ARMS; i++) {
            if (nb_ctx->arm[i] != NULL) {
                close(nb_ctx->arm[i]->fd);
                free(nb_ctx->arm[i]);
            }
        }
        free(nb_ctx);
    }
}


int add_arm(nb_modbus_t*  nb_ctx, uint8_t index, const char *device, int speed)
{
    if (index >= MAX_ARMS) 		// Too many devices
        return -1;

    arm_verbose = verbose;
    arm_handle* arm = calloc(1, sizeof(arm_handle));	// Allocate and zero-init the arm_handle struct

    if (arm == NULL) // Allocation failed
        return -1;

    if (arm_init(arm, device, speed, index) == 0) {
        nb_ctx->arm[index] = arm;
    } else {
        free(arm);
        return -1;
    }
}


int load_fw(char *path, uint8_t* prog_data, const size_t len)
{
    FILE* fd;
    int red, i;
    fd = fopen(path, "rb");
    if (!fd) {
        printf("error opening firmware file \"%s\"\n", path);
        return -1;
    }
    memset(prog_data, 0xff, len);

    red = fread(prog_data, 1, MAX_FW_SIZE, fd);
    //printf("Bytes 58: %d,59: %d,60: %d,61: %d,62: %d,63: %d,64: %d\n", prog_data[58], prog_data[59], prog_data[60], prog_data[61], prog_data[62], prog_data[63]);
    fclose(fd);
    return red;
}

#define MAX_R2000  64
int arm_firmware_do(arm_handle* arm, const char* fwdir, int overwrite)
{

    uint8_t *prog_data;   // buffer containing firmware
    uint8_t* rw_data;     // buffer containing firmware rw data
    int prog_data_len;
    int rw_data_len;
    uint16_t buffer[MAX_R2000];
    int n2000;


    rw_data = load_fw_file(&arm->bv, fwdir, TRUE, &rw_data_len);
    if (rw_data == NULL) {
        return -1;
    }
    prog_data = load_fw_file(&arm->bv, fwdir, FALSE, &prog_data_len);
    if (prog_data == NULL) {
        free(rw_data);
        return -1;
    }

    if (!overwrite) {
        n2000 = read_regs(arm, 2000, MAX_R2000, buffer);
        vprintf("N2000 = %d\n", n2000);
        if (n2000 > 1) {
            n2000 = 2*(n2000);
            if (n2000 >= rw_data_len) n2000 = rw_data_len-2;
            memcpy(rw_data, buffer, n2000);
            overwrite = 1;
        }
    }
    start_firmware(arm);
    send_firmware(arm, prog_data, prog_data_len, 0);
/*
    vprintf("Sending nvram file %s length=%d\n", fwname_rw, rw_data_len);
*/
    if (overwrite)  {
        vprintf("\n");
        send_firmware(arm, rw_data, rw_data_len, 0xe000);
    }
    finish_firmware(arm);
    free(prog_data);
    free(rw_data);
    // Reload version
    uint16_t configregs[5];
    if (read_regs(arm, 1000, 5, configregs) == 5)
        parse_version(&arm->bv, configregs);
}

int arm_firmware(arm_handle* arm, const char* fwdir)
{
    /* Check version */
    uint32_t fwver;

    if (fwver=check_firmware_update(&arm->bv, fwdir)) {
        vprintf("PLEASE UPDATE FIRMWARE TO %d.%d - to proceed, execute fwspi -P -U\n", fwver>>8, fwver & 0xff);
    }

    if (fwver=check_new_rw_version(&arm->bv, fwdir)) {
        vprintf("NEW firmware=%d.%d found\n", fwver>>8, fwver & 0xff);
        return arm_firmware_do(arm, fwdir, FALSE);
    }
}
