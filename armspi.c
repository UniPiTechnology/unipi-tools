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


// on RPI 2 doesn't work transfer longer then 94 bytes. Must be divided into chunks
#define _MAX_SPI_RX  94


#define ac_header(buf) ((arm_comm_header*)buf)
#define ach_header(buf) ((arm_comm_chr_header*)buf)
#define acs_header(buf) ((arm_comm_str_header*)buf)

#define IDLE_PATTERN 0x0e5500fa
// hodnota 240 znaku by pravdepodobne mela byt spise 
//    255 - sizeof(arm_comm_header) = 251 -> 250(sude cislo) znaku 
// vyzkouset jestli neni problem v firmware
#define SPI_STR_MAX 240


static void pabort(const char *s)
{
    perror(s);
    //abort();
}


uint8_t get_spi_mode(int fd)
{
    uint8_t mode;

    int ret = ioctl(fd, SPI_IOC_RD_MODE, &mode);
    if (ret >= 0) {
        return mode;
    }
    return ret;
}

uint32_t get_spi_speed(int fd)
{
    uint32_t speed;
    int ret = ioctl(fd, SPI_IOC_RD_MAX_SPEED_HZ, &speed);
    if (ret >= 0) {
        return speed;
    }
    return ret;
    //SPI_IOC_RD_BITS_PER_WORD,
    //SPI_IOC_RD_LSB_FIRST,
    //SPI_IOC_WR_MODE32,
}

void set_spi_mode(int fd, uint8_t mode)
{
    int ret = ioctl(fd, SPI_IOC_WR_MODE, &mode);
    if (ret < 0) {
        pabort("Cannot set mode");
    }
}

void set_spi_speed(int fd, uint32_t speed)
{
    int ret = ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
    if (ret < 0) {
        pabort("Cannot set speed" );
    }
}

void queue_uart(uart_queue* queue, uint8_t chr1, uint8_t len)
{
    queue->remain = (len==0) ? 255 : len - 1;  // len==0 means 256 byte in remote queue
    if (queue->index < MAX_LOCAL_QUEUE_LEN) {
        queue->buffer[queue->index++] = chr1;
    } else {
        queue->overflow++;
    }
}

int one_phase_op(arm_handle* arm, uint8_t op, uint16_t reg, uint8_t value)
{
    int ret;
    arm->tx1.op = op;
    arm->tx1.len = value;
    arm->tx1.reg = reg;
    arm->tx1.crc = SpiCrcString((uint8_t*)&arm->tx1, SIZEOF_HEADER, 0);

    arm->tr[1].delay_usecs = 0;
    ret = ioctl(arm->fd, SPI_IOC_MESSAGE(2), arm->tr);
    if (ret < 1) {
        pabort("Can't send one-phase spi message");
        return -1;
    }
    uint16_t crc = SpiCrcString((uint8_t*)&arm->rx1, SIZEOF_HEADER,0);
    if (crc != arm->rx1.crc) {
        pabort("Bad crc in one-phase operation");
        return -1;
    }

    if ((*((uint32_t*)&arm->rx1) & 0xffff00ff) == IDLE_PATTERN) { 
        return 0;
    }
    if (arm->rx1.op == ARM_OP_WRITE_CHAR) { 
        // we received character from UART
        // doplnit adresaci uartu &arm->uart_q[0..4]
        queue_uart(arm->uart_q, ach_header(&arm->rx1)->ch1, ach_header(&arm->rx1)->len);
        return 0;
    }
    pabort("Unexpcted reply in one-phase operation");
    return -1;
}

int two_phase_op(arm_handle* arm, uint8_t op, uint16_t reg, uint16_t len2)
{
    int ret;
    uint16_t tr_len2;
    uint16_t crc;
    // Prepare chunk1
    arm->tx1.op = op;
    arm->tx1.reg = reg;
    arm->tx1.len = len2 & 0xff;        //set len in chunk1 to length of chunk2 (without crc)
    if (op != ARM_OP_WRITE_STR) {
       ac_header(arm->tx2)->op  = op;  // op and reg in chunk2 is the same
       ac_header(arm->tx2)->reg = reg;
    }
    tr_len2 = (len2 & 1) ? len2+1 : len2;         //transaction length must be even
    crc = SpiCrcString((uint8_t*)&arm->tx1, SIZEOF_HEADER, 0);
    arm->tx1.crc = crc;                           // crc of first phase
    crc = SpiCrcString(arm->tx2, tr_len2, crc);   // crc of second phase
    ((uint16_t*)arm->tx2)[tr_len2>>1] = crc;

    ac_header(arm->rx2)->op  = op;                // 'destroy' content of receiving buffer
    arm->tr[1].delay_usecs = 25;                  // set delay after first phase

    uint32_t total = tr_len2 + CRC_SIZE;

    if (total <= _MAX_SPI_RX) {
        arm->tr[2].len = total;
        ret = ioctl(arm->fd, SPI_IOC_MESSAGE(3), arm->tr);
    } else if (total <= (2*_MAX_SPI_RX)) {
        arm->tr[2].len = _MAX_SPI_RX;
        arm->tr[3].len = total - _MAX_SPI_RX;
        ret = ioctl(arm->fd, SPI_IOC_MESSAGE(4), arm->tr);
    } else {
        arm->tr[2].len = _MAX_SPI_RX;
        arm->tr[3].len = _MAX_SPI_RX;
        arm->tr[4].len = total - (2*_MAX_SPI_RX);
        ret = ioctl(arm->fd, SPI_IOC_MESSAGE(5), arm->tr);
    }

    //printf("ret2=%d\n", ret);
    if (ret < 1) {
        pabort("can't send two-phase spi message");
        return ret;
    }

    //printf("rx1=%x\n", *((uint32_t*)&arm->rx1));
    crc = SpiCrcString((uint8_t*)&arm->rx1, SIZEOF_HEADER, 0);
    if (crc != arm->rx1.crc) {
        pabort("Bad 1.crc in two phase operation");
        return -1;
    }

    crc = SpiCrcString(arm->rx2, tr_len2, crc);

    if (arm->rx1.op == ARM_OP_WRITE_CHAR) { 
        // we received character from UART
        // doplnit adreaci uartu!
        queue_uart(arm->uart_q, ach_header(&arm->rx1)->ch1, ach_header(&arm->rx1)->len);
        if (((uint16_t*)arm->rx2)[tr_len2>>1] != crc) {
            pabort("Bad 2.crc in two phase operation");
            return -1;
        }
        return 0;
    }
    if (((uint16_t*)arm->rx2)[tr_len2>>1] != crc) {
        pabort("Bad 2.crc in two phase operation");
        return -1;
    }
    if ((*((uint32_t*)&arm->rx1) & 0xffff00ff) == IDLE_PATTERN) {
        return 0;
    }
    pabort("Unexpcted reply in two phase operation");
    return -1;
}

int idle_op(arm_handle* arm)
{
    return one_phase_op(arm, ARM_OP_IDLE, 0x0e55, 0);
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
            pabort("Unexpected reply in READ_REG");
            return -1;
    }
    cnt =  ac_header(arm->rx2)->len;
    memmove(result, arm->rx2+SIZEOF_HEADER, cnt * sizeof(uint16_t));
    return cnt;
}

int write_regs(arm_handle* arm, uint16_t reg, uint8_t cnt, uint16_t* values)
{
    if (cnt > 126) {
        pabort("Too many registers in WRITE_REG");
        return -1;
    }

    uint16_t len2 = SIZEOF_HEADER + sizeof(uint16_t) * cnt;

    ac_header(arm->tx2)->len = cnt;
    memmove(arm->tx2 + SIZEOF_HEADER, values, cnt * sizeof(uint16_t));

    int ret = two_phase_op(arm, ARM_OP_WRITE_REG, reg, len2);
    if (ret < 0) {
        return ret;
    }

    if (ac_header(arm->rx2)->op != ARM_OP_WRITE_REG) {
        pabort("Unexpcted reply in WRITE_REG");
        return -1;
    }
    cnt =  ac_header(arm->rx2)->len;
    return cnt;
}

int read_bits(arm_handle* arm, uint16_t reg, uint16_t cnt, uint8_t* result)
{
    uint16_t len2 = SIZEOF_HEADER + (((cnt+15) >> 4) << 1);  // trunc to 16bit in bytes
    if (len2 > 256){
        pabort("Too many registers in READ_BITS");
        return -1;
    }
    int ret = two_phase_op(arm, ARM_OP_READ_BIT, reg, len2);
    if (ret < 0) {
        return ret;
    }

    if ((ac_header(arm->rx2)->op != ARM_OP_READ_BIT) || 
        (ac_header(arm->rx2)->reg != reg)) {
            pabort("Unexpcted reply in READ_BIT");
            return -1;
    }
    cnt = ac_header(arm->rx2)->len;
    memmove(result, arm->rx2+SIZEOF_HEADER, ((cnt+7) >> 3));    // trunc to 8 bit
    return cnt;
}

int write_bit(arm_handle* arm, uint16_t reg, uint8_t value)
{
    int ret = one_phase_op(arm, ARM_OP_WRITE_BIT, reg, !(!value));
    if (ret < 0) {
        return ret;
    }
    return 1;
}

int write_bits(arm_handle* arm, uint16_t reg, uint16_t cnt, uint8_t* values)
{
    uint16_t len2 = SIZEOF_HEADER + (((cnt+15) >> 4) << 1);  // trunc to 16bit in bytes
    if (len2 > 256) {
        pabort("Too many registers in WRITE_BITS");
        return -1;
    }

    ac_header(arm->tx2)->len = cnt;
    memmove(arm->tx2 + SIZEOF_HEADER, values, ((cnt+7) >> 3));

    int ret = two_phase_op(arm, ARM_OP_WRITE_BITS, reg, len2);
    if (ret < 0) {
        return ret;
    }

    if (ac_header(arm->rx2)->op != ARM_OP_WRITE_BITS) {
        pabort("Unexpcted reply in WRITE_REG");
        return -1;
    }
    if (cnt > ac_header(arm->rx2)->len)
       cnt = ac_header(arm->rx2)->len;
    return cnt;
}


int write_char(arm_handle* arm, uint8_t uart, uint8_t c)
{
    int ret = one_phase_op(arm, ARM_OP_WRITE_CHAR, uart, c);
    if (ret < 0) {
        return ret;
    }
    return 1;
}

int write_string(arm_handle* arm, uint8_t uart, uint8_t* str, int len)
{

    if ((len > 256) || (len<=0)) {
        pabort("Bad string length(1..256)");
        return -1;
    }
    uint16_t len2 = len;

    memmove(arm->tx2, str, len2);

    int ret = two_phase_op(arm, ARM_OP_WRITE_STR, uart, len2);

    if (ac_header(arm->rx2)->op != ARM_OP_WRITE_STR) {
        pabort("Unexpcted reply in WRITE_STR");
        return -1;
    }
    return ac_header(arm->rx2)->len;
    //return cnt;
}

int read_string(arm_handle* arm, uint8_t uart, uint8_t* str, int cnt)
{
    if (uart > 0) {
        pabort("Bad parameter uart");
        return -1;
    }
    uart_queue* queue = &arm->uart_q[uart];
    uint16_t len2 = cnt;

    if (len2 % 2) len2++;
    if (len2 > SPI_STR_MAX) len2 = SPI_STR_MAX;
    len2 = len2 + SIZEOF_HEADER;

    int ret =  two_phase_op(arm, ARM_OP_READ_STR, uart, len2);
    if (ret < 0) {
        return ret;
    }

    if (ac_header(arm->rx2)->op != ARM_OP_READ_STR) {
        //if (arm_rx2_str.len > cnt):
        pabort("Unexpcted reply in READ_STR");
        return -1;
    }
    uint16_t rcnt = acs_header(arm->rx2)->len;    // length of received string
    queue->remain = acs_header(arm->rx2)->remain; // remains in remote queue
    // join uart_queue and rcnt chars
    int n = queue->index < cnt ? queue->index : cnt;
    if (n > 0) {
        memmove(str, queue->buffer, n);
        if (n < queue->index) {  // str is too short for import local queue
            memmove(queue->buffer, queue->buffer + n, queue->index - n);
            queue->index = queue->index + rcnt;
            if (queue->index + rcnt > MAX_LOCAL_QUEUE_LEN) 
                rcnt = MAX_LOCAL_QUEUE_LEN - queue->index;
            memmove(queue->buffer+queue->index, arm->rx2 + SIZEOF_HEADER, rcnt);
            queue->index = queue->index + rcnt;
            return n;
        }
        queue->index = 0;
        cnt = cnt - n;
    }
    int n2 = rcnt < cnt ? rcnt : cnt;
    memmove(str+n, arm->rx2 + SIZEOF_HEADER, n2);
    if (rcnt > cnt) { // str is too short,  move rest of string to local queue
        queue->index = rcnt - cnt;
        if (queue->index > MAX_LOCAL_QUEUE_LEN) queue->index = MAX_LOCAL_QUEUE_LEN;
        memmove(queue->buffer, arm->rx2 + SIZEOF_HEADER+cnt, queue->index);
    }
    return n+n2;
}


int arm_init(arm_handle* arm, char* device, uint32_t speed)
{
    arm->fd = open(device, O_RDWR);

    if (arm->fd < 0) {
        pabort("Cannot open device");
        return -1;
    }
    //set_spi_mode(fd,0);
    set_spi_speed(arm->fd, speed);

    int i;
    for (i=0; i< 4; i++) {
       arm->uart_q[i].remain = 0;
       arm->uart_q[i].index = 0;
    }
    // Prepare transactional structure
    memset(arm->tr, 0, sizeof(arm->tr));
    arm->tr[0].delay_usecs = 5;    // starting pause between NSS and SCLK
    arm->tr[1].tx_buf = (unsigned long) &arm->tx1;
    arm->tr[1].rx_buf = (unsigned long) &arm->rx1;
    arm->tr[1].len = SNIPLEN1;
    arm->tr[2].tx_buf = (unsigned long) arm->tx2;
    arm->tr[2].rx_buf = (unsigned long) arm->rx2;
    arm->tr[3].tx_buf = (unsigned long) arm->tx2 + _MAX_SPI_RX;
    arm->tr[3].rx_buf = (unsigned long) arm->rx2 + _MAX_SPI_RX;
    arm->tr[4].tx_buf = (unsigned long) arm->tx2 + (_MAX_SPI_RX*2);
    arm->tr[4].rx_buf = (unsigned long) arm->rx2 + (_MAX_SPI_RX*2);
    return 0;
}


