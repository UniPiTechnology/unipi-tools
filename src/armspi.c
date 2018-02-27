/*
 * SPI communication with UniPi Neuron family controllers
 *
 * Copyright (c) 2016  Faster CZ, ondra@faster.cz
 * Copyright (c) 2007  MontaVista Software, Inc.
 * Copyright (c) 2007  Anton Vorontsov <avorontsov@ru.mvista.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License.
 *
 * Cross-compile with cross-gcc -I/path/to/cross-kernel/include
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
#include "armutil.h"
#include "spicrc.h"

// brain/modbus_prot.h
#define ARM_OP_READ_BIT   1
#define ARM_OP_READ_REG   4
#define ARM_OP_WRITE_BIT  5
#define ARM_OP_WRITE_REG  6
#define ARM_OP_WRITE_BITS 15

#define ARM_OP_WRITE_CHAR  65
#define ARM_OP_WRITE_STR   100
#define ARM_OP_READ_STR    101

#define ARM_OP_IDLE        0xfa


// !!!! on RPI 2,3 doesn't work transfer longer then 94 bytes. Must be divided into chunks
//#define _MAX_SPI_RX  94
#define _MAX_SPI_RX  64
//#define _MAX_SPI_RX  256


#define ac_header(buf) ((arm_comm_header*)buf)
#define ach_header(buf) ((arm_comm_chr_header*)buf)
#define acs_header(buf) ((arm_comm_str_header*)buf)

#define IDLE_PATTERN 0x0e5500fa
// hodnota 240 znaku by pravdepodobne mela byt spise 
//    255 - sizeof(arm_comm_header) = 251 -> 250(even number) characters
// vyzkouset jestli neni problem v firmware
#define SPI_STR_MAX 240
#define NSS_PAUSE_DEFAULT  10

//static int be_quiet = 0;
int arm_verbose = 0;
int nss_pause = NSS_PAUSE_DEFAULT;


int one_phase_op(arm_handle* arm, uint8_t op, uint16_t reg, uint8_t value, uint8_t do_lock)
{
    int ret;
    if (arm_verbose>1) printf("Neuron TCP Server: One Phase Op\n");
    arm->tx1.op = op;
    arm->tx1.len = value;
    arm->tx1.reg = reg;
    arm->tx1.crc = SpiCrcString((uint8_t*)&arm->tx1, SIZEOF_HEADER, 0);

    uint32_t total = SIZEOF_HEADER + CRC_SIZE;
    uint8_t char_package[total + 10];
    memset(char_package, 0, sizeof(char_package));
    memcpy(char_package+10, &arm->tx1, SIZEOF_HEADER + CRC_SIZE);
    char_package[0] = (uint8_t)arm->index;
    char_package[3] = 1;
    char_package[7] = do_lock;
    //memcpy(char_package+16, &arm->tx2, tr_len2 + CRC_SIZE);
    ret = write(arm->fd, char_package, total+10);
    if (ret == total+10) ret = read(arm->fd, char_package, total+10);
    memcpy(&arm->rx1, char_package, SNIPLEN1);
    if (ret < 1) {
        if (arm_verbose) printf("Can't send one-phase spi message");
        return -1;
    }
    uint16_t crc = SpiCrcString(char_package, SIZEOF_HEADER, 0);
    if (arm_verbose>1) printf("One Phase Op: %x %x\n", crc, arm->rx1.crc);
    if (crc != arm->rx1.crc) {
        if (arm_verbose) printf("Bad crc in one-phase operation");
        return -1;
    }

    if ((*((uint32_t*)&arm->rx1) & 0xffff00ff) == IDLE_PATTERN) {
    	if (arm_verbose>1) printf("One Phase Op successful!\n");
        return 0;
    }
    if (arm->rx1.op == ARM_OP_WRITE_CHAR) { 
        // we received character from UART, resolved by kernel
        return 0;
    }
    if (ret == total) {
    	if (arm_verbose>1) printf("One Phase Op successful!\n");
    	return 0;
    }
    if (arm_verbose) printf("Unexpected reply in one-phase operation");
    return -1;
}

char errmsg[256];
int two_phase_op(arm_handle* arm, uint8_t op, uint16_t reg, uint16_t len2)
{
    int ret;
    uint16_t tr_len2;
    uint16_t crc;
    uint16_t delay_usecs = 25; // set delay after first phase
    if (arm_verbose>1) printf("Neuron TCP Server: Two Phase Op\n");
    // Prepare chunk1
    if (arm == NULL) {
    	if (arm_verbose>1) printf("Invalid Arm Device\n");
    	return -1;
    }
    arm->tx1.op = op;
    arm->tx1.reg = reg;
    arm->tx1.len = len2 & 0xff;        //set len in chunk1 to length of chunk2 (without crc)
    //arm->tr[1].delay_usecs = 25;              // set delay after first phase
    if (arm_verbose>1) printf("Neuron TCP Server: Two Phase Op1-b\n");
    if (op != ARM_OP_WRITE_STR) {
        ac_header(arm->tx2)->op  = op;  // op and reg in chunk2 is the same
        ac_header(arm->tx2)->reg = reg;
        if (len2 > 60) {
            delay_usecs += (len2-60)/2;  // add more delay
        }
    }

    tr_len2 = (len2 & 1) ? len2+1 : len2;         //transaction length must be even

    if (arm_verbose>1) printf("Neuron TCP Server: Two Phase Op 2\n");
    ac_header(arm->rx2)->op  = op;                // 'destroy' content of receiving buffer
    uint32_t total = SIZEOF_HEADER + CRC_SIZE + tr_len2 + CRC_SIZE;
    uint8_t char_package[total + 10];

    crc = SpiCrcString(&arm->tx1, SIZEOF_HEADER, 0);
    arm->tx1.crc = crc;                           // crc of first phase
    memset(char_package, 0, sizeof(char_package));
    memcpy(char_package+10, &arm->tx1, SIZEOF_HEADER + CRC_SIZE);

    crc = SpiCrcString(&arm->tx2, tr_len2, crc);   // crc of second phase
    ((uint16_t*)arm->tx2)[tr_len2>>1] = crc;
    memcpy(char_package+16, &arm->tx2, tr_len2 + CRC_SIZE);
    char_package[0] = (uint8_t)arm->index;

    char_package[3] = 1;
    char_package[6] = delay_usecs;
    *((uint16_t *)&char_package[1]) = reg;
    ret = write(arm->fd, char_package, total+10);
    if (arm_verbose>1) printf("WRITE RET:%d TOT:%d\n", ret, total);
    if (ret == total+10) ret = read(arm->fd, char_package, total+10);

    if (ret < 1) {
        if (arm_verbose) printf("Can't send two-phase spi message\n");
        return ret;
    }
    if (arm_verbose>1) printf("Read %d from /dev/neuronspi: %x %x %x %x %x %x\n", ret, char_package[0], char_package[1], char_package[2], char_package[3], char_package[4], char_package[5]);

    crc = SpiCrcString(char_package, SIZEOF_HEADER, 0);
    memcpy(&arm->rx1, char_package, SNIPLEN1);

    if (crc != arm->rx1.crc) {
    	if (arm_verbose) printf("Bad 1.crc in two phase operation %x %x\n", crc, arm->rx1.crc);
        return -1;
    }
    if (arm_verbose>1) printf("Read phase 1 finished: %x %x\n", crc, arm->rx1.crc);
    crc = SpiCrcString(&(char_package[SNIPLEN1]), tr_len2, crc);
    memcpy(arm->rx2,&(char_package[SNIPLEN1]), tr_len2);
    if (arm->rx1.op == ARM_OP_WRITE_CHAR) { 
        // we received a character from UART
        // queue_uart(arm->uart_q, ach_header(&arm->rx1)->ch1, ach_header(&arm->rx1)->len);
        if (((uint16_t*)char_package)[tr_len2+SNIPLEN1>>1] != crc) {
        	if (arm_verbose) printf("Bad 2.crc in two phase operation %x %x\n", crc, ((uint16_t*)arm->rx2)[tr_len2>>1]);
            return -2;
        }
        return 0;
    }
    if (((uint16_t*)char_package)[tr_len2+SNIPLEN1>>1] != crc) {
        if (arm_verbose) printf("Bad 2.crc in two phase operation\n");
        return -3;
    }
    if (arm_verbose>1) printf("Read phase 2 finished: %x %x\n", crc, ((uint16_t*)char_package)[tr_len2+SNIPLEN1>>1]);

    if ((*((uint32_t*)&arm->rx1) & 0xffff00ff) == IDLE_PATTERN) {
    	if (arm_verbose>1) printf("Read successful!\n");
        return 0;
    }

    if (ret == total) {
    	if (arm_verbose>1) printf("Read successful!\n");
    	return 0;
    }

    if (arm_verbose) printf(errmsg,"Unexpected reply in two phase operation %02x %02x %04x %04x\n",
            arm->rx1.op, arm->rx1.len, arm->rx1.reg, arm->rx1.crc);
    return -1;
}

int idle_op(arm_handle* arm, uint8_t do_lock)
{
    int backup = arm_verbose;
    arm_verbose = 0;
    int n = one_phase_op(arm, ARM_OP_IDLE, 0x0e55, 0, do_lock);
    arm_verbose = backup;;
    return n;
}

int read_regs(arm_handle* arm, uint16_t reg, uint8_t cnt, uint16_t* result)
{
    uint16_t len2 = SIZEOF_HEADER + sizeof(uint16_t) * cnt;
    int ret = two_phase_op(arm, ARM_OP_READ_REG, reg, len2);
    if (ret < 0) {
        return ret;
    }
    
    if ((ac_header(arm->rx2)->op != ARM_OP_READ_REG) || 
        (ac_header(arm->rx2)->len > cnt) ||
        (ac_header(arm->rx2)->reg != reg)) {
            if (arm_verbose) printf("Unexpected reply in READ_REG");
            return -1;
    }
    cnt =  ac_header(arm->rx2)->len;
    
    memmove(result, arm->rx2+SIZEOF_HEADER, cnt * sizeof(uint16_t));
    if (arm_verbose>1) printf("CNT: %d %d %x %x %x %x %x\n", cnt, ret, result[0], result[1], result[2], result[3], result[4]);
    return cnt;
}

int write_regs(arm_handle* arm, uint16_t reg, uint8_t cnt, uint16_t* values)
{
    if (cnt > 126) {
        if (arm_verbose) printf("Too many registers in WRITE_REG");
        return -1;
    }
    uint16_t len2 = SIZEOF_HEADER + sizeof(uint16_t) * cnt;
    if (arm == NULL) {
    	if (arm_verbose>1) printf("Invalid Arm Device\n");
    	return -1;
    }
    ac_header(arm->tx2)->len = cnt;
    memmove(arm->tx2 + SIZEOF_HEADER, values, cnt * sizeof(uint16_t));

    int ret = two_phase_op(arm, ARM_OP_WRITE_REG, reg, len2);
    if (ret < 0) {
        return ret;
    }

    if (ac_header(arm->rx2)->op != ARM_OP_WRITE_REG) {
        if (arm_verbose) printf("Unexpected reply in WRITE_REG");
        return -1;
    }
    cnt =  ac_header(arm->rx2)->len;
    return cnt;
}

int read_bits(arm_handle* arm, uint16_t reg, uint16_t cnt, uint8_t* result)
{
    uint16_t len2 = SIZEOF_HEADER + (((cnt+15) >> 4) << 1);  // trunc to 16bit in bytes
    if (len2 > 256){
        if (arm_verbose) printf("Too many registers in READ_BITS");
        return -1;
    }
    int ret = two_phase_op(arm, ARM_OP_READ_BIT, reg, len2);
    if (ret < 0) {
        return ret;
    }

    if ((ac_header(arm->rx2)->op != ARM_OP_READ_BIT) || 
        (ac_header(arm->rx2)->reg != reg)) {
            if (arm_verbose) printf("Unexpected reply in READ_BIT");
            return -1;
    }
    cnt = ac_header(arm->rx2)->len;
    memmove(result, arm->rx2+SIZEOF_HEADER, ((cnt+7) >> 3));    // trunc to 8 bit
    return cnt;
}

int write_bit(arm_handle* arm, uint16_t reg, uint8_t value, uint8_t do_lock)
{
    int ret = one_phase_op(arm, ARM_OP_WRITE_BIT, reg, !(!value), do_lock);
    if (ret < 0) {
        return ret;
    }
    return 1;
}

int write_bits(arm_handle* arm, uint16_t reg, uint16_t cnt, uint8_t* values)
{
    uint16_t len2 = SIZEOF_HEADER + (((cnt+15) >> 4) << 1);  // trunc to 16bit in bytes
    if (len2 > 256) {
        if (arm_verbose) printf("Too many registers in WRITE_BITS");
        return -1;
    }

    ac_header(arm->tx2)->len = cnt;
    memmove(arm->tx2 + SIZEOF_HEADER, values, ((cnt+7) >> 3));

    int ret = two_phase_op(arm, ARM_OP_WRITE_BITS, reg, len2);
    if (ret < 0) {
        return ret;
    }

    if (ac_header(arm->rx2)->op != ARM_OP_WRITE_BITS) {
        if (arm_verbose) printf("Unexpected reply in WRITE_REG");
        return -1;
    }
    if (cnt > ac_header(arm->rx2)->len)
       cnt = ac_header(arm->rx2)->len;
    return cnt;
}


#define START_SPI_SPEED 5000000
int arm_init(arm_handle* arm, const char* device, uint32_t speed, int index)
{
    arm->fd = open(device, O_RDWR);

    if (arm->fd < 0) {
        if (arm_verbose) printf("Cannot open device");
        return -1;
    }

    arm->index = index;

    int i;
    /* Load firmware and hardware versions */
    arm->bv.sw_version = 0;
    int backup = arm_verbose;
    arm_verbose = 0;
    uint16_t configregs[5];
    int outp_l = 0;
    if (read_regs(arm, 1000, 5, configregs) == 5) {
        parse_version(&arm->bv, configregs);
        uint16_t buf[1024];
        memset(&buf[0], 0, 1024*2);
        outp_l = read_regs(arm, 0, 50, buf);
        if (backup>1) printf("Config-regs: %x %x %x %x %x\n", configregs[0], configregs[1], configregs[2], configregs[3], configregs[4]);
    }
    //arm_version(arm);
    if (speed == 0) {
        speed = get_board_speed(&arm->bv);
        //set_spi_speed(arm->fd, speed);
        if (read_regs(arm, 1000, 5, configregs) != 5) {
            speed = START_SPI_SPEED;
        }
    }
    if (arm_verbose>1) printf("ARM Init finished pt1:%x\n", arm->bv.sw_version);
    arm_verbose = backup;
    if (arm->bv.sw_version) {
        if (arm_verbose) 
            printf("Board on %s firmware=%d.%d  hardware=%d.%d (%s) (spi %dMHz)\n", device,
                SW_MAJOR(arm->bv.sw_version), SW_MINOR(arm->bv.sw_version),
                HW_BOARD(arm->bv.hw_version), HW_MAJOR(arm->bv.hw_version),
                arm_name(arm->bv.hw_version), speed / 1000000);
    } else {
        close(arm->fd);
        return -1;
    }

    if (arm_verbose>1) printf("ARM Init finished!\n");

    return 0;
}

/***************************************************************************************/


uint32_t firmware_op(arm_handle* arm, uint32_t address, uint8_t* tx_data, int tx_len)
{
    uint16_t crc;
    int ret;
    uint32_t rx_result;
    uint8_t char_package[sizeof(arm_comm_firmware) + 10];
    
    memset(char_package, 0, 10);									// package header
    memcpy(char_package+10, &address, sizeof(address));				// firmware page address
    if (tx_len > 0) {
        memcpy(char_package+10+ sizeof(address), tx_data, 
                                      tx_len);						// firmware data
    }
    memset(char_package+10+ sizeof(address)+tx_len, 0xff, 
                                      ARM_PAGE_SIZE-tx_len);		// empty firmware data
    crc = SpiCrcString((uint8_t*)(char_package+10), 
                                      ARM_PAGE_SIZE+sizeof(address), 0);// calculate crc from address + data
    memcpy(char_package+10+ sizeof(address)+ARM_PAGE_SIZE, &crc, 
                                      sizeof(crc));						// set crc
    char_package[0] = (uint8_t)arm->index;
    char_package[3] = 0;
    char_package[7] = ((uint8_t)arm->index+1);
    if (arm_verbose>1) printf("FW-OP send package len:%d: %x %x %x %x \t%x %x %x %x %x %x\n", sizeof(arm_comm_firmware), char_package[10], char_package[11], char_package[12], char_package[13], char_package[14], char_package[15], char_package[16], char_package[17], char_package[18], char_package[19], char_package[20]);

    ret = write(arm->fd, char_package, sizeof(arm_comm_firmware) + 10);
    if (ret != sizeof(arm_comm_firmware) + 10) {
    	if (arm_verbose) printf("FW-OP invalid length written: %d, exp: %d\n", ret, sizeof(arm_comm_firmware) + 10);
        return 0xffffffff;
    }    
   	ret = read(arm->fd, char_package, sizeof(arm_comm_firmware) + 10);
    if (ret != sizeof(arm_comm_firmware) + 10) {
    	if (arm_verbose) printf("FW-OP invalid length read: %d, exp: %d\n", ret, sizeof(arm_comm_firmware) + 10);
        return 0xffffffff;
    }

    if (arm_verbose>1) printf("FW-OP recv package len:%d: %x %x %x %x \t%x %x %x %x %x %x\n", sizeof(arm_comm_firmware), char_package[0], char_package[1], char_package[2], char_package[3], char_package[4], char_package[5], char_package[16], char_package[17], char_package[18], char_package[19], char_package[20]);
    crc = SpiCrcString((uint8_t*)(char_package), sizeof(arm_comm_firmware), 0);// calculate crc INCLUDING crc
    if (crc != 0) {
    	if (arm_verbose) printf("FW-OP bad crc RET:%d CRC(0):%x \n", ret, crc);
        return 0xffffffff;
    }
    memcpy(&rx_result, char_package, sizeof(rx_result));				// result from last fw operation
    return rx_result;
}


void start_firmware(arm_handle* arm)
{
    int prog_bit = 1004;
    if (arm->bv.sw_version <= 0x400) prog_bit = 104;
    write_bit(arm, prog_bit, 1, (arm->index) + 1);                                                   // start programming in ARM
    usleep(100000);
}


void finish_firmware(arm_handle* arm)
{
    uint32_t rx_result;

    // Finish transfer
    rx_result = firmware_op(arm, ARM_FIRMWARE_KEY, NULL, 0);
    if (rx_result != ARM_FIRMWARE_KEY) {
        printf("UNKNOWN ERROR\nREBOOTING...\n");
    } else {
        printf("REBOOTING...\n");
    }
    idle_op(arm, 255);// Unlock operation
    usleep(100000);
}

int send_firmware(arm_handle* arm, uint8_t* data, size_t datalen, uint32_t start_address)
{
    uint32_t rx_result;

    int prev_addr = -1;
    int len = datalen;
    uint32_t address = start_address;
    if (arm_verbose>1) printf("Starting to send at %x\n", start_address);
    while (len >= 0) {
        if (len >= ARM_PAGE_SIZE) {
            rx_result = firmware_op(arm, address, data + (address - start_address), ARM_PAGE_SIZE);
            len = len - ARM_PAGE_SIZE;
        } else if (len != 0) {
            rx_result = firmware_op(arm, address, data + (address - start_address), len);
            len = 0;
        } else {
            address = 0xF400;   // read-only page; operation is performed only for last page confirmation
            rx_result = firmware_op(arm, address, NULL, 0);
            len = -1;
        }
        if (rx_result != ARM_FIRMWARE_KEY) {
        	if (arm_verbose) printf("Address %x does not equal %x\n", rx_result, ARM_FIRMWARE_KEY);
            if ((address == prev_addr)||(prev_addr == -1)) { 
                // double error or start error
                usleep(100000);
                break;
            }
            address = prev_addr;
            len = datalen - (address - start_address);
            usleep(100000);
            continue;
        }
        if (prev_addr != -1) {
        	if (arm_verbose) printf("%04x OK\n", prev_addr);
        }
        usleep(100000);
        prev_addr = address;
        address = address + ARM_PAGE_SIZE;
    }
    if (arm_verbose>1) printf("Finished sending at %x\n", address);
    return 0;
}

