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
int arm_verbose = 1;
int nss_pause = NSS_PAUSE_DEFAULT;
static void pabort(const char *s)
{
    if (arm_verbose > 0) perror(s);
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
    queue->remain = (len==0) ? 255 : len - 1;  // len==0 means 256 bytes in remote queue
    if (queue->index < MAX_LOCAL_QUEUE_LEN) {
        queue->buffer[queue->index++] = chr1;
    } else {
        queue->overflow++;
    }
}

int one_phase_op(arm_handle* arm, uint8_t op, uint16_t reg, uint8_t value, uint8_t do_lock)
{
    int ret;
    if (arm_verbose) printf("Neuron TCP Server: One Phase Op\n");
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
    //int fd = open("/dev/neuronspi", O_RDWR);
    ret = write(arm->fd, char_package, total+10);
    if (ret == total+10) ret = read(arm->fd, char_package, total+10);
    //close(fd);
    memcpy(&arm->rx1, char_package, SNIPLEN1);
    if (ret < 1) {
        pabort("Can't send one-phase spi message");
        return -1;
    }
    uint16_t crc = SpiCrcString(char_package, SIZEOF_HEADER, 0);
    if (arm_verbose) printf("One Phase Op: %x %x\n", crc, arm->rx1.crc);
    if (crc != arm->rx1.crc) {
        pabort("Bad crc in one-phase operation");
        return -1;
    }

    if ((*((uint32_t*)&arm->rx1) & 0xffff00ff) == IDLE_PATTERN) {
    	if (arm_verbose) printf("One Phase Op successful!\n");
        return 0;
    }
    if (arm->rx1.op == ARM_OP_WRITE_CHAR) { 
        // we received character from UART
        // doplnit adresaci uartu &arm->uart_q[0..4]
        queue_uart(arm->uart_q, ach_header(&arm->rx1)->ch1, ach_header(&arm->rx1)->len);
        return 0;
    }
    if (ret == total) {
    	if (arm_verbose) printf("One Phase Op successful!\n");
    	return 0;
    }
    pabort("Unexpected reply in one-phase operation");
    return -1;
}

char errmsg[256];
int two_phase_op(arm_handle* arm, uint8_t op, uint16_t reg, uint16_t len2)
{
    int ret;
    uint16_t tr_len2;
    uint16_t crc;
    if (arm_verbose) printf("Neuron TCP Server: Two Phase Op\n");
    // Prepare chunk1
    if (arm == NULL) {
    	if (arm_verbose) printf("Invalid Arm Device\n");
    	return -1;
    }
    arm->tx1.op = op;
    arm->tx1.reg = reg;
    arm->tx1.len = len2 & 0xff;        //set len in chunk1 to length of chunk2 (without crc)
    arm->tr[1].delay_usecs = 25;              // set delay after first phase
    if (arm_verbose) printf("Neuron TCP Server: Two Phase Op1-b\n");
    if (op != ARM_OP_WRITE_STR) {
        ac_header(arm->tx2)->op  = op;  // op and reg in chunk2 is the same
        ac_header(arm->tx2)->reg = reg;
        if (len2 > 60) {
            arm->tr[1].delay_usecs += (len2-60)/2;  // add more delay
        }
    }

    tr_len2 = (len2 & 1) ? len2+1 : len2;         //transaction length must be even

    if (arm_verbose) printf("Neuron TCP Server: Two Phase Op 2\n");
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
    char_package[6] = arm->tr[1].delay_usecs;
    *((uint16_t *)&char_package[1]) = reg;
    ret = write(arm->fd, char_package, total+10);
    if (arm_verbose) printf("WRITE RET:%d TOT:%d\n", ret, total);
    if (ret == total+10) ret = read(arm->fd, char_package, total+10);

    if (ret < 1) {
        pabort("Can't send two-phase spi message\n");
        return ret;
    }
    if (arm_verbose) printf("Read %d from /dev/neuronspi: %x %x %x %x %x %x\n", ret, char_package[0], char_package[1], char_package[2], char_package[3], char_package[4], char_package[5]);

    crc = SpiCrcString(char_package, SIZEOF_HEADER, 0);
    memcpy(&arm->rx1, char_package, SNIPLEN1);

    if (crc != arm->rx1.crc) {
    	if (arm_verbose) printf("Bad 1.crc in two phase operation %x %x\n", crc, arm->rx1.crc);
        pabort("Bad 1.crc in two phase operation\n");
        return -1;
    }
    if (arm_verbose) printf("Read phase 1 finished: %x %x\n", crc, arm->rx1.crc);
    crc = SpiCrcString(&(char_package[SNIPLEN1]), tr_len2, crc);
    memcpy(arm->rx2,&(char_package[SNIPLEN1]), tr_len2);
    if (arm->rx1.op == ARM_OP_WRITE_CHAR) { 
        // we received a character from UART
        queue_uart(arm->uart_q, ach_header(&arm->rx1)->ch1, ach_header(&arm->rx1)->len);
        if (((uint16_t*)char_package)[tr_len2+SNIPLEN1>>1] != crc) {
        	if (arm_verbose) printf("Bad 2.crc in two phase operation %x %x\n", crc, ((uint16_t*)arm->rx2)[tr_len2>>1]);
            pabort("Bad 2.crc in two phase operation\n");
            return -2;
        }
        return 0;
    }
    if (((uint16_t*)char_package)[tr_len2+SNIPLEN1>>1] != crc) {
        pabort("Bad 2.crc in two phase operation\n");
        return -3;
    }
    if (arm_verbose) printf("Read phase 2 finished: %x %x\n", crc, ((uint16_t*)char_package)[tr_len2+SNIPLEN1>>1]);

    if ((*((uint32_t*)&arm->rx1) & 0xffff00ff) == IDLE_PATTERN) {
    	if (arm_verbose) printf("Read successful!\n");
        return 0;
    }

    if (ret == total) {
    	if (arm_verbose) printf("Read successful!\n");
    	return 0;
    }

    sprintf(errmsg,"Unexpected reply in two phase operation %02x %02x %04x %04x\n",
            arm->rx1.op, arm->rx1.len, arm->rx1.reg, arm->rx1.crc);
    pabort(errmsg);
    return -1;
}

int idle_op(arm_handle* arm)
{
    int backup = arm_verbose;
    arm_verbose = 0;
    int n = one_phase_op(arm, ARM_OP_IDLE, 0x0e55, 0, 0);
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
    /*
    if ((ac_header(arm->rx2)->op != ARM_OP_READ_REG) || 
        (ac_header(arm->rx2)->len > cnt) ||
        (ac_header(arm->rx2)->reg != reg)) {
            pabort("Unexpected reply in READ_REG");
            return -1;
    }
    cnt =  ac_header(arm->rx2)->len;
    */
    memmove(result, arm->rx2+SIZEOF_HEADER, cnt * sizeof(uint16_t));
    if (arm_verbose) printf("CNT: %d %d %x %x %x %x %x\n", cnt, ret, result[0], result[1], result[2], result[3], result[4]);
    return cnt;
}

int write_regs(arm_handle* arm, uint16_t reg, uint8_t cnt, uint16_t* values)
{
    if (cnt > 126) {
        pabort("Too many registers in WRITE_REG");
        return -1;
    }
    uint16_t len2 = SIZEOF_HEADER + sizeof(uint16_t) * cnt;
    if (arm == NULL) {
    	if (arm_verbose) printf("Invalid Arm Device\n");
    	return -1;
    }
    ac_header(arm->tx2)->len = cnt;
    memmove(arm->tx2 + SIZEOF_HEADER, values, cnt * sizeof(uint16_t));

    int ret = two_phase_op(arm, ARM_OP_WRITE_REG, reg, len2);
    if (ret < 0) {
        return ret;
    }

    if (ac_header(arm->rx2)->op != ARM_OP_WRITE_REG) {
        pabort("Unexpected reply in WRITE_REG");
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
            pabort("Unexpected reply in READ_BIT");
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
        pabort("Unexpected reply in WRITE_REG");
        return -1;
    }
    if (cnt > ac_header(arm->rx2)->len)
       cnt = ac_header(arm->rx2)->len;
    return cnt;
}


int write_char(arm_handle* arm, uint8_t uart, uint8_t c)
{
    int ret = one_phase_op(arm, ARM_OP_WRITE_CHAR, uart, c, 0);
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
        pabort("Unexpected reply in WRITE_STR");
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
        if (arm_verbose) printf("Error read str %d %d\n", cnt, len2);
        return ret;
    }

    if (ac_header(arm->rx2)->op != ARM_OP_READ_STR) {
        //if (arm_rx2_str.len > cnt):
        pabort("Unexpected reply in READ_STR");
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
            queue->index -= n;
            if (queue->index + rcnt > MAX_LOCAL_QUEUE_LEN) 
                rcnt = MAX_LOCAL_QUEUE_LEN - queue->index;
            memmove(queue->buffer+queue->index, arm->rx2 + SIZEOF_HEADER, rcnt);
            queue->index += rcnt;
            return n;
        }
        queue->index = 0;
        cnt -= n;
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

int read_qstring(arm_handle* arm, uint8_t uart, uint8_t* str, int cnt)
{
    if (uart > 0) {
        pabort("Bad parameter uart");
        return -1;
    }
    uart_queue* queue = &arm->uart_q[uart];
    
    // join uart_queue and rcnt chars
    int n = queue->index < cnt ? queue->index : cnt;
    if (n > 0) {
        memmove(str, queue->buffer, n);
        if (n < queue->index) {  // str is too short for import local queue
            memmove(queue->buffer, queue->buffer + n, queue->index - n);
            queue->index -= n;
            return n;
        }
        queue->index = 0;
        return n;
    }
    return 0;
}


//const char* GPIO_INT[] = { "27", "23", "22" };
#define START_SPI_SPEED 5000000
int arm_init(arm_handle* arm, const char* device, uint32_t speed, int index, const char* gpio)
{
    arm->fd = open(device, O_RDWR);

    if (arm->fd < 0) {
        pabort("Cannot open device");
        return -1;
    }

    //set_spi_mode(fd,0);
    //if (speed==0) {
    //    set_spi_speed(arm->fd, START_SPI_SPEED);
    //} else {
    //    set_spi_speed(arm->fd, speed);
    //}
    arm->index = index;

    int i;
    for (i=0; i< 4; i++) {
       arm->uart_q[i].masterpty = -1;
       arm->uart_q[i].remain = 0;
       arm->uart_q[i].index = 0;
    }
    // Prepare transactional structure
    memset(arm->tr, 0, sizeof(arm->tr));
    arm->tr[0].delay_usecs = nss_pause;    // starting pause between NSS and SCLK
    arm->tr[1].tx_buf = (unsigned long) &arm->tx1;
    arm->tr[1].rx_buf = (unsigned long) &arm->rx1;
    arm->tr[1].len = SNIPLEN1;
    arm->tr[2].tx_buf = (unsigned long) arm->tx2;
    arm->tr[2].rx_buf = (unsigned long) arm->rx2;
    arm->tr[3].tx_buf = (unsigned long) arm->tx2 + _MAX_SPI_RX;
    arm->tr[3].rx_buf = (unsigned long) arm->rx2 + _MAX_SPI_RX;
    arm->tr[4].tx_buf = (unsigned long) arm->tx2 + (_MAX_SPI_RX*2);
    arm->tr[4].rx_buf = (unsigned long) arm->rx2 + (_MAX_SPI_RX*2);
    arm->tr[5].tx_buf = (unsigned long) arm->tx2 + (_MAX_SPI_RX*3);
    arm->tr[5].rx_buf = (unsigned long) arm->rx2 + (_MAX_SPI_RX*3);
    arm->tr[6].tx_buf = (unsigned long) arm->tx2 + (_MAX_SPI_RX*4);
    arm->tr[6].rx_buf = (unsigned long) arm->rx2 + (_MAX_SPI_RX*4);
    /* Load firmware and hardware versions */
    int backup = arm_verbose;
    arm_verbose = 0;
    uint16_t configregs[5];
    int outp_l = 0;
    if (read_regs(arm, 1000, 5, configregs) == 5) {
        parse_version(&arm->bv, configregs);
        uint16_t buf[1024];
        memset(&buf[0], 0, 1024*2);
        outp_l = read_regs(arm, 0, 50, buf);
    }
    if (arm_verbose) printf("Config-regs: %x %x %x %x %x\n", configregs[0], configregs[1], configregs[2], configregs[3], configregs[4]);
    //arm_version(arm);
    if (speed == 0) {
        speed = get_board_speed(&arm->bv);
        set_spi_speed(arm->fd, speed);
        if (read_regs(arm, 1000, 5, configregs) != 5) {
            set_spi_speed(arm->fd, START_SPI_SPEED);
            speed = START_SPI_SPEED;
        }
    }
    printf("ARM Init finished pt1:%x\n", arm->bv.sw_version);
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

    /* Open fdint for interrupt catcher */
    arm->fdint = -1;
    if (arm_verbose) printf("ARM Init finished!\n");

    if ((gpio == NULL)||(strlen(gpio) == 0)||(arm->bv.int_mask_register<=0)) return 0;

    int fdx = open("/sys/class/gpio/export", O_WRONLY);
    if (fdx < 0) return 0;
    write(fdx, gpio, strlen(gpio));
    close(fdx);

    char gpiobuf[256];
    sprintf(gpiobuf, "/sys/class/gpio/gpio%s/edge", gpio);
    fdx = open(gpiobuf, O_WRONLY);
    if (fdx < 0) return 0;
    write(fdx, "rising", 6);
    close(fdx);

    sprintf(gpiobuf, "/sys/class/gpio/gpio%s/value", gpio);
    arm->fdint = open(gpiobuf, O_RDONLY);
    if (arm->fdint < 0) return 0;

    return 0;
}

/***************************************************************************************/

typedef struct {
    arm_handle* arm;
    struct spi_ioc_transfer* tr;
    arm_comm_firmware* tx;
    arm_comm_firmware* rx;
} Tfirmware_context;

int firmware_op(arm_handle* arm, arm_comm_firmware* tx, arm_comm_firmware* rx, int tr_len, struct spi_ioc_transfer* tr, uint8_t do_lock)
{
	printf ("Bytes 54:%x, 55:%x, 56: %x, 57: %x, 58: %x, 59: %x, 60: %x, 61: %x, 62: %x, 63: %x\n", tx->data[54], tx->data[55], tx->data[56], tx->data[57], tx->data[58], tx->data[59], tx->data[60], tx->data[61], tx->data[62], tx->data[63]);
    tx->crc = SpiCrcString((uint8_t*)tx, sizeof(arm_comm_firmware) - sizeof(tx->crc), 0);
    uint8_t char_package[sizeof(arm_comm_firmware) + 10];
    memset(char_package, 0, sizeof(arm_comm_firmware) + 10);
    memcpy((&char_package[10]), tx, sizeof(arm_comm_firmware));
    char_package[0] = (uint8_t)arm->index;
    char_package[3] = 0;
    char_package[7] = do_lock;
    printf("FW-OP package len:%d: %x %x %x %x %x \t %x %x %x %x %x\n", sizeof(arm_comm_firmware), char_package[10], char_package[11], char_package[12], char_package[13], char_package[14], char_package[15], char_package[16], char_package[17], char_package[18], char_package[19], char_package[20]);
    int ret = write(arm->fd, char_package, sizeof(arm_comm_firmware) + 10);
    if (ret == sizeof(arm_comm_firmware) + 10) {
    	ret = read(arm->fd, char_package, sizeof(arm_comm_firmware) + 10);
    	memcpy(rx, char_package, sizeof(arm_comm_firmware) + 10);
    } else {
    	printf("Invalid length written: %d, exp: %d\n", ret, sizeof(arm_comm_firmware) + 10);
        pabort("Invalid length written");
        return -1;
    }

    //int ret = ioctl(arm->fd, SPI_IOC_MESSAGE(tr_len), tr);
    if (ret < 1) {
    	printf("Can't send firmware-op spi message\n");
        pabort("Can't send firmware-op spi message");
        return -2;
    }
    uint16_t crc = SpiCrcString((uint8_t*)rx, sizeof(arm_comm_firmware) - sizeof(rx->crc),0);
    //printf("a=%0x d=%x crc=%x\n", rx->address, rx->data[0], rx->crc);
    if (crc != rx->crc && do_lock != 255) {
    	printf("Bad crc in firmware operation RET:%d CRC:%x RX-CRC:%x\n", ret, crc, rx->crc);
        pabort("Bad crc in firmware operation");
        return -3;
    }
    return 0;
}

void* start_firmware(arm_handle* arm)
{
    Tfirmware_context* fwctx = calloc(1, sizeof(Tfirmware_context));
    if (fwctx == NULL) return NULL;
    fwctx->arm = arm;
    /* Alloc Tx Rx buffers */
    fwctx->tx = calloc(1, sizeof(arm_comm_firmware)+2);
    if (fwctx->tx == NULL) {
        free(fwctx);
        return NULL;
    }
    fwctx->rx = calloc(1, sizeof(arm_comm_firmware)+2);
    if (fwctx->rx == NULL) { 
        free(fwctx->tx); 
        free(fwctx);
        return NULL; 
    }
    /* Transaction array */
    int i;
    int tr_len = ((sizeof(arm_comm_firmware) - 1) / _MAX_SPI_RX) + 2;               // Transaction array length 
    fwctx->tr = calloc(tr_len, sizeof(struct spi_ioc_transfer));  // Alloc transaction array
    if (fwctx->tr == NULL) {
        free(fwctx->rx); 
        free(fwctx->tx); 
        free(fwctx);
        return NULL; 
    } 
    fwctx->tr[0].delay_usecs = 5;                                                          // first transaction has no data
    for (i=0; i < tr_len-1; i++) {
        fwctx->tr[i+1].len = _MAX_SPI_RX;
        fwctx->tr[i+1].tx_buf = (unsigned long) fwctx->tx + (_MAX_SPI_RX*i);
        fwctx->tr[i+1].rx_buf = (unsigned long) fwctx->rx + (_MAX_SPI_RX*i);
    }
    fwctx->tr[tr_len-1].len = ((sizeof(arm_comm_firmware) - 1) % _MAX_SPI_RX) + 1;       // last transaction is shorter

    int prog_bit = 1004;
    if (arm->bv.sw_version <= 0x400) prog_bit = 104;
    write_bit(arm, prog_bit, 1, (arm->index) + 1);                                                   // start programming in ARM
    usleep(100000);
    return (void*) fwctx;
}


void finish_firmware(void*  ctx)
{
    Tfirmware_context* fwctx = (Tfirmware_context*) ctx;
    int tr_len = ((sizeof(arm_comm_firmware) - 1) / _MAX_SPI_RX) + 2;               // Transaction array length 
    int idle_resp = 0;

    fwctx->tx->address = ARM_FIRMWARE_KEY;  // finish transfer
    firmware_op(fwctx->arm, fwctx->tx, fwctx->rx, tr_len, fwctx->tr, (fwctx->arm->index) + 1);
    if (fwctx->rx->address != ARM_FIRMWARE_KEY) {
        printf("UNKNOWN ERROR\nREBOOTING...\n");
    } else {
        printf("REBOOTING...\n");
    }
    firmware_op(fwctx->arm, fwctx->tx, fwctx->rx, tr_len, fwctx->tr, 255);

    // dealloc
    //free(fwctx->tr);
    //free(fwctx->rx);
    //free(fwctx->tx);
    //free(fwctx);
    usleep(100000);
}

int send_firmware(void* ctx, uint8_t* data, size_t datalen, uint32_t start_address)
{
    Tfirmware_context* fwctx = (Tfirmware_context*) ctx;
    int tr_len = ((sizeof(arm_comm_firmware) - 1) / _MAX_SPI_RX) + 2;               // Transaction array length 

    int prev_addr = -1;
    int len = datalen;
    uint32_t address = start_address;
    printf("Starting to send at %x\n", start_address);
    while (len >= 0) {
    	printf("Sending len = %d\n", len);
        fwctx->tx->address = address;
        if (len >= ARM_PAGE_SIZE) {
            memcpy(fwctx->tx->data, data + (address - start_address), ARM_PAGE_SIZE);  // read page from file
            len = len - ARM_PAGE_SIZE;
        } else if (len != 0) {
        	memset(fwctx->tx->data+len, 0xff, ARM_PAGE_SIZE - len);
            memcpy(fwctx->tx->data, data + (address - start_address), len);            // read  page (part) from file
            printf("MEMCPY LEN: %d", len);
            len = 0;
        } else {
            address = 0xF400;   // read-only page; operation is performed only for last page confirmation
            len = -1;
        }
        firmware_op(fwctx->arm, fwctx->tx, fwctx->rx, tr_len, fwctx->tr, (fwctx->arm->index) + 1);
        if (fwctx->rx->address != ARM_FIRMWARE_KEY) {
        	printf("Address %x does not equal %x\n", fwctx->rx->address, ARM_FIRMWARE_KEY);
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
    printf("Finished sending at %x\n", address);
    return 0;
}

int _send_firmware(arm_handle* arm, uint8_t* data, size_t datalen, uint32_t start_address)
{
    /* Alloc Tx Rx buffers */
    arm_comm_firmware* tx = calloc(1, sizeof(arm_comm_firmware)+2);
    if (tx == NULL) return -1;
    arm_comm_firmware* rx = calloc(1, sizeof(arm_comm_firmware)+2);
    if (rx == NULL) { 
        free(tx); 
        return -1; 
    }
    /* Transaction array */
    int i;
    int tr_len = ((sizeof(arm_comm_firmware) - 1) / _MAX_SPI_RX) + 2;               // Transaction array length 
    struct spi_ioc_transfer* tr = calloc(tr_len, sizeof(struct spi_ioc_transfer));  // Alloc transaction array
    if (tr == NULL) {
        free(rx); 
        free(tx); 
        return -1; 
    } 
    tr[0].delay_usecs = 5;                                                          // first transaction has no data
    for (i=0; i < tr_len-1; i++) {
        tr[i+1].len = _MAX_SPI_RX;
        tr[i+1].tx_buf = (unsigned long) tx + (_MAX_SPI_RX*i);
        tr[i+1].rx_buf = (unsigned long) rx + (_MAX_SPI_RX*i);
    }
    tr[tr_len-1].len = ((sizeof(arm_comm_firmware) - 1) % _MAX_SPI_RX) + 1;       // last transaction is shorter

    int prog_bit = 1004;
    if (arm->bv.sw_version <= 0x400) prog_bit = 104;
    write_bit(arm, prog_bit, 1, (arm->index) + 1);                                                   // start programming in ARM
    usleep(100000);

    int prev_addr = -1;
    int len = datalen;
    uint32_t address = start_address;

    while (len >= 0) {
        tx->address = address;
        if (len >= ARM_PAGE_SIZE) {
            memcpy(tx->data, data + (address-start_address), ARM_PAGE_SIZE);  // read page from file
            len = len - ARM_PAGE_SIZE;
        } else if (len != 0) {
            memcpy(tx->data, data + (address-start_address), len);            // read  page (part) from file
            memset(tx->data+len, 0xff, ARM_PAGE_SIZE-len);  
            len = 0;
        } else {
            address = 0xF400;   // read-only page; operation is performed only for last page confirmation
            len = -1;
        }
        firmware_op(arm, tx, rx, tr_len, tr, (arm->index) + 1);
        if (rx->address != ARM_FIRMWARE_KEY) {
            if ((address == prev_addr)||(prev_addr == -1)) { 
                // double error or start error
                usleep(100000);
                break;
            }
            address = prev_addr;
            len = datalen - (address-start_address);
            usleep(100000);
            continue;
        }
        if (prev_addr != -1) if (arm_verbose) printf("%04x OK\n", prev_addr);
        usleep(100000);
        prev_addr = address;
        address = address + ARM_PAGE_SIZE;
    } 

    tx->address = ARM_FIRMWARE_KEY;  // finish transfer
    firmware_op(arm, tx, rx, tr_len, tr, (arm->index) + 1);
    if (rx->address != ARM_FIRMWARE_KEY) {
        if (arm_verbose) printf("UNKNOWN ERROR\nREBOOTING...\n");
    } else {
        if (arm_verbose) printf("REBOOTING...\n");
    }
    firmware_op(arm, tx, rx, tr_len, tr, 255);

    // dealloc
    free(tr); 
    free(rx); 
    free(tx);
    usleep(100000);
    return 0;
}
