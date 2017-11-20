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
#include <sys/mman.h>

#include "nb_modbus.h"
#include "armspi.h"
#include "armutil.h"

int verbose = 0;
int deferred_op = DFR_NONE;
arm_handle*  deferred_arm;

#define vprintf( ... ) if (verbose > 0) printf( __VA_ARGS__ )
#define vvprintf( ... ) if (verbose > 1) printf( __VA_ARGS__ )

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

    //offset = nb_ctx->ctx->backend->header_length;
    offset = _MODBUS_TCP_HEADER_LENGTH;
    slave = req[offset - 1];
    if (slave == 255) {
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
            int n = read_regs(arm, address, nb, (uint16_t*) (rsp+rsp_length));
            if (n == nb) {
                for (i = address; i < address + nb; i++) {
                    c = rsp[rsp_length++];
                    rsp[rsp_length-1] = rsp[rsp_length];
                    rsp[rsp_length++] = c;
                }
            } else if (n < 0) {
            	rsp_length = nb_response_exception(
                        nb_ctx->ctx, MODBUS_EXCEPTION_SLAVE_OR_SERVER_FAILURE, rsp,
                        "Illegal data value 0x%0X in read_bits\n", address);
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
            int n;
            if (address == 1004) { // exception for firmware
                //arm_firmware(arm, nb_ctx->fwdir, data ? 1 : 0);
                deferred_op = DFR_OP_FIRMWARE;
                deferred_arm = arm;
                n = 1;
            } else
                n = write_bit(arm, address, data ? 1 : 0, 0);
            if (n == 1) {
                rsp_length += 4; // = req_length;
            } else if (n < 0) {
            	rsp_length = nb_response_exception(
                        nb_ctx->ctx, MODBUS_EXCEPTION_SLAVE_OR_SERVER_FAILURE, rsp,
                        "Illegal data value 0x%0X in read_bits\n", address);
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
        } else if (n < 0) {
        	rsp_length = nb_response_exception(
                    nb_ctx->ctx, MODBUS_EXCEPTION_SLAVE_OR_SERVER_FAILURE, rsp,
                    "Illegal data value 0x%0X in read_bits\n", address);
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
            } else if (n < 0) {
            	rsp_length = nb_response_exception(
                        nb_ctx->ctx, MODBUS_EXCEPTION_SLAVE_OR_SERVER_FAILURE, rsp,
                        "Illegal data value 0x%0X in read_bits\n", address);
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
            } else if (n < 0) {
            	rsp_length = nb_response_exception(
                        nb_ctx->ctx, MODBUS_EXCEPTION_SLAVE_OR_SERVER_FAILURE, rsp,
                        "Illegal data value 0x%0X in read_bits\n", address);
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
    if (index >= MAX_ARMS) 		// Too many devices
        return -1;

    arm_verbose = verbose;
    arm_handle* arm = calloc(1, sizeof(arm_handle));	// Allocate and zero-init the arm_handle struct

    if (arm == NULL) // Allocation failed
        return -1;

    if (arm_init(arm, device, speed, index, gpio) == 0) {
        nb_ctx->arm[index] = arm;
    } else {
        free(arm);
        return -1;
    }
}


char* _firmware_name(arm_handle* arm, const char* fwdir, const char* ext)
{
    const char* armname = arm_name(arm->bv.hw_version);
    char* fwname = malloc(strlen(fwdir) + strlen(armname) + strlen(ext) + 2);
    strcpy(fwname, fwdir);
    if (strlen(fwname) && (fwname[strlen(fwname)-1] != '/')) strcat(fwname, "/");
    strcat(fwname, armname);
    strcat(fwname, ext);
    return fwname;
}


int arm_flash_file(void* fwctx, const char* fwname)
{
    /* Firmware programming */
    int fd, ret;
    void * data;
    int min_len = 1024;

    if((fd = open(fwname, O_RDONLY)) >= 0) {
        struct stat st;
        if((ret=fstat(fd,&st)) >= 0) {
            size_t len_file = st.st_size;
            vprintf("Sending firmware file %s length=%d\n", fwname, len_file);
            if (len_file > min_len) { 
                if((data=mmap(NULL, len_file, PROT_READ,MAP_PRIVATE, fd, 0)) != MAP_FAILED) {
                    send_firmware(fwctx, (uint8_t*) data, len_file, 0);
                    munmap(data, len_file);
                } else {
                    vprintf("Error mapping firmware file %s to memory\n", fwname);
                }
            } else {
                vprintf("Damaged firmware file %s\n", fwname);
            }
        }
        close(fd);
    } else {
        vprintf("Error opening firmware file %s\n", fwname);
    }
}


int arm_flash_rw_file(void* fwctx, const char* fwname, int overwrite, int n2000, uint16_t* buffer)
{
    /* Nvram programming */
    int fd, ret;
    void * data;
    int min_len = 6;

    if((fd = open(fwname, O_RDONLY)) >= 0) {
        struct stat st;
        if((ret=fstat(fd,&st)) >= 0) {
            size_t len_file = st.st_size;
            vprintf("Sending nvram file %s length=%d\n", fwname, len_file);
            if ((len_file > min_len)&&(len_file < 2*128 /*1024*/)) { 
                if ((n2000 > 0)|| overwrite) {
                    if((data=mmap(NULL, len_file, PROT_READ,MAP_PRIVATE, fd, 0)) != MAP_FAILED) {
                        if (overwrite) {
                            send_firmware(fwctx, (uint8_t*) data, len_file,0xe000);
                        } else {
                            memcpy(buffer+n2000-1, ((uint8_t*) data) + 2*(n2000-1), len_file - 2*(n2000-1));
                            send_firmware(fwctx, (uint8_t*) buffer, len_file,0xe000);
                        }
                        munmap(data, len_file);
                    } else {
                        vprintf("Error mapping nvram file %s to memory\n", fwname);
                    }
                } else {
                    vprintf("Can't read original nvram\n");
                }
            } else {
                vprintf("Damaged nvram file %s\n", fwname);
            }
        }
        close(fd);
    } else {
        vprintf("Error opening nvram file %s\n", fwname);
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
    printf("Bytes 58: %d,59: %d,60: %d,61: %d,62: %d,63: %d,64: %d\n", prog_data[58], prog_data[59], prog_data[60], prog_data[61], prog_data[62], prog_data[63]);
    fclose(fd);
    return red;
}

int arm_firmware(arm_handle* arm, const char* fwdir, int overwrite)
{
    /* Check version */
    uint8_t *prog_data;   // buffer containing firmware
    uint8_t* rw_data;     // buffer containing firmware rw data
    char* fwname = firmware_name(arm->bv.hw_version, arm->bv.base_hw_version,
                                 fwdir, ".rw");

    int fd, ret;
    int min_len = 6;
    uint32_t fwver = 0;
    uint16_t buffer[64];
    void * data;
    int n2000 = read_regs(arm, 2000, 64, buffer);

    vprintf("N2000 = %d\n", n2000);
    vprintf("Checking firmware file: %s\n", fwname);
    if((fd = open(fwname, O_RDONLY)) >= 0) {
        if (lseek(fd, - 4, SEEK_END) >= 0) {
            if (read(fd, &fwver, 4) == 4) {
                if (fwver & 0xff000000) fwver = fwver >> 16;
            } else fwver = 0; 
        }
        close(fd);
    }
    free(fwname);
    vprintf("SW ver: %04x\n", fwver);
    if (fwver > arm->bv.sw_version) {
    	char* fwname_rw = firmware_name(arm->bv.hw_version, arm->bv.base_hw_version,
                               fwdir, ".rw");
    	char* fwname_bin = firmware_name(arm->bv.hw_version, arm->bv.base_hw_version,
                               fwdir, ".bin");
        prog_data = malloc(MAX_FW_SIZE);
        rw_data = malloc(MAX_FW_SIZE);




        int red = load_fw(fwname_bin, prog_data, MAX_FW_SIZE);
        int rwlen = load_fw(fwname_rw, rw_data, MAX_RW_SIZE);


        void * fwctx = start_firmware(arm);
        if (fwctx == NULL)
            return -1;

        send_firmware(fwctx, prog_data, red, 0);
        if((fd = open(fwname_rw, O_RDONLY)) >= 0) {
            struct stat st;
            if((ret=fstat(fd,&st)) >= 0) {
                size_t len_file = st.st_size;
                vprintf("Sending nvram file %s length=%d\n", fwname_rw, len_file);
                if ((len_file > min_len)&&(len_file < 2*128 /*1024*/)) {
                    if ((n2000 > 0)|| overwrite) {
                        if((data=mmap(NULL, len_file, PROT_READ,MAP_PRIVATE, fd, 0)) != MAP_FAILED) {
                            if (overwrite) {
                                send_firmware(fwctx, (uint8_t*) data, len_file,0xe000);
                            } else {
                                memcpy(buffer+n2000-1, ((uint8_t*) data) + 2*(n2000-1), len_file - 2*(n2000-1));
                                send_firmware(fwctx, (uint8_t*) buffer, len_file,0xe000);
                            }
                            munmap(data, len_file);
                        } else {
                            vprintf("Error mapping nvram file %s to memory\n", fwname_rw);
                        }
                    } else {
                        vprintf("Can't read original nvram\n");
                    }
                } else {
                    vprintf("Damaged nvram file %s\n", fwname_rw);
                }
            }
            close(fd);
        } else {
            vprintf("Error opening nvram file %s\n", fwname_rw);
        }

        finish_firmware(fwctx);
        free(prog_data);
        free(rw_data);
        // Reload version
        uint16_t configregs[5];
        if (read_regs(arm, 1000, 5, configregs) == 5)
            parse_version(&arm->bv, configregs);
    }
}
