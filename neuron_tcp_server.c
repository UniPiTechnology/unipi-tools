/*
 * SPI communication with UniPi Neuron family controllers
 * using standard libmodbus framework
 *
 * Copyright (c) 2016  Faster CZ, ondra@faster.cz
 * Copyright © 2009-2010 Stéphane Raimbault <stephane.raimbault@gmail.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include <errno.h>
#include <signal.h>

#include <modbus.h>

#if defined(_WIN32)
#include <ws2tcpip.h>
#else
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#endif

#include "armspi.h"

#define NB_CONNECTION    5

modbus_t *ctx = NULL;
int server_socket;
modbus_mapping_t *mb_mapping;

static void close_sigint(int dummy)
{
    close(server_socket);
    modbus_free(ctx);
    modbus_mapping_free(mb_mapping);

    exit(dummy);
}

int modbus_to_spi(uint8_t* query, modbus_mapping_t* mb_mapping, arm_handle* arm1/*, uint16_t reg_offset*/)
{
    int i;
    uint8_t len = query[5];
    uint8_t slave_id = query[6];
    uint8_t op = query[7];
    uint16_t reg = MODBUS_GET_INT16_FROM_INT8(query, 8); //query[9] + (query[8] << 8);
    uint16_t cnt = MODBUS_GET_INT16_FROM_INT8(query, 10); //query[11] + (query[10] << 8);
    uint16_t c;

    switch (op) {
        case 1: {
            /* read coil */
            // printf("slave=%d l=%d op=%d: READ_COIL b%d(%d)", slave_id, len, op, reg, cnt);
            if (cnt > mb_mapping->nb_bits)
                cnt = mb_mapping->nb_bits;
            c = read_bits(arm1, reg, cnt, mb_mapping->tab_input_bits);
            modbus_set_bits_from_bytes(mb_mapping->tab_bits, 0, cnt, mb_mapping->tab_input_bits);
            mb_mapping->start_bits = reg;
            break;
          }
        case 2: {
            /* read bit */
            // printf("slave=%d l=%d op=%d: READ_BIT b%d(%d)", slave_id, len, op, reg, cnt);
            // read_bits(arm, reg, cnt, &result);
            if (cnt > mb_mapping->nb_input_bits)
                cnt = mb_mapping->nb_input_bits;
            c = read_bits(arm1, reg, cnt, mb_mapping->tab_bits);
            modbus_set_bits_from_bytes(mb_mapping->tab_input_bits, 0, cnt, mb_mapping->tab_bits);
            mb_mapping->start_input_bits = reg;
            break;
          }
        case 3: {
            /* read holding registers */
            //printf("slave=%d l=%d op=%d: READ_HOLD_REG r%d(%d)", slave_id, len, op, reg, cnt);
            if (cnt > mb_mapping->nb_registers)
                cnt = mb_mapping->nb_registers;
            c = read_regs(arm1, reg, cnt, mb_mapping->tab_registers);
            mb_mapping->start_registers = reg;
            break;
          }
        case 4: {
            /* read input registers */
            //printf("slave=%d l=%d op=%d: READ_INP_REG  r%d(%d)", slave_id, len, op, reg, cnt);
            if (cnt > mb_mapping->nb_input_registers) 
                cnt = mb_mapping->nb_input_registers;
            c = read_regs(arm1, reg, cnt, mb_mapping->tab_input_registers);
            mb_mapping->start_input_registers = reg;
            break;
          }
        case 5: {
            /* write single coil */
            //printf("slave=%d l=%d op=%d: WRITE_COIL c%d = %d\n", slave_id, len, op, reg, (cnt==0xff00)?1:0);
            c = write_bit(arm1, reg, (cnt==0xff00)?1:0);
            mb_mapping->start_bits = reg;
            break;
          }
        case 6: {
            /* write single register */
            //printf("slave=%d l=%d op=%d: WRITE_REG  r%d = %d\n", slave_id, len, op, reg, cnt);
            c = write_regs(arm1, reg, 1, &cnt);
            mb_mapping->start_registers = reg;
            break;
          }
        case 15: {
            /* write multi coil */
            uint8_t blen = query[12];
            // ToDo: check max length 
            c = write_bits(arm1, reg, cnt, &query[13]);
            //printf("slave=%d l=%d op=%d: WRITE_COILS c%d(%d) = ", slave_id, len, op, reg, cnt);
            //for (i=0; i<blen; i++) printf("%02x ", query[i+13]);
            //printf("\n");
            mb_mapping->start_bits = reg;
            break;
          }
        case 16: {
            uint8_t blen = query[12];
            //printf("slave=%d l=%d op=%d: WRITE_REGS r%d(%d) = ", slave_id, len, op, reg, cnt);
            for (i=0; i<(blen>>1); i++) {
                //printf("%d ", query[2*i+14]+(query[3*i+13] << 8));
                mb_mapping->tab_registers[i] = MODBUS_GET_INT16_FROM_INT8(query, 13+2*i);
                //mb_mapping->tab_registers[i+1] = query[i+13];
                //c = query[i+14]; query[i+14] = query[i+13]; query[i+13] = c;
            }
            printf("\n");
            //c = write_regs(arm1, reg, cnt, (uint16_t*)(&query[i+13]));
            c = write_regs(arm1, reg, cnt, mb_mapping->tab_registers);
            mb_mapping->start_registers = reg;
            break;
          }
        case 23: {
            uint16_t regw = query[13] + (query[12] << 8);
            uint16_t cntw = query[15] + (query[14] << 8);
            printf("slave=%d l=%d op=%d: WR_RD__REGS r%d(%d) r%d(%d) = ", slave_id, len, op, reg, cnt, regw, cntw);
            uint8_t blen = query[16];
            for (i=0; i<blen; i++, i++) printf("%d ", query[i+18]+(query[i+17] << 8));
            printf("\n");
            break;
          }
        default: {
            printf("slave=%d l=%d op=%d: ", slave_id, len, op);
            for (i=0; i<len-2; i++) {
                printf("%02x ", query[i+8]);
            }
            printf("\n");
         }
    }
    //printf("\n");
    return 0;
}



int main(void)
{
    int master_socket;
    int rc;
    fd_set refset;
    fd_set rdset;

    arm_handle* arm1 = malloc(sizeof(arm_handle));
    arm_init(arm1, "/dev/spidev0.1", 12000000);

    /* Maximum file descriptor number */
    int fdmax;

//    ctx = modbus_new_tcp("127.0.0.1", 5020);
    ctx = modbus_new_tcp("0.0.0.0", 5020);

    int header_len = modbus_get_header_length(ctx);
    printf("header_len=%d\n", header_len);

    mb_mapping = modbus_mapping_new(MODBUS_MAX_READ_BITS, MODBUS_MAX_READ_BITS,
                                    MODBUS_MAX_READ_REGISTERS, MODBUS_MAX_READ_REGISTERS);
//    mb_mapping = modbus_mapping_new(1100, 1100,
//                                    1100, 1100);
    if (mb_mapping == NULL) {
        fprintf(stderr, "Failed to allocate the mapping: %s\n",
                modbus_strerror(errno));
        modbus_free(ctx);
        return -1;
    }

    server_socket = modbus_tcp_listen(ctx, NB_CONNECTION);

    signal(SIGINT, close_sigint);

    /* Clear the reference set of socket */
    FD_ZERO(&refset);
    /* Add the server socket */
    FD_SET(server_socket, &refset);

    /* Keep track of the max file descriptor */
    fdmax = server_socket;

    for (;;) {
        rdset = refset;
        if (select(fdmax+1, &rdset, NULL, NULL, NULL) == -1) {
            perror("Server select() failure.");
            close_sigint(1);
        }

        /* Run through the existing connections looking for data to be
         * read */
        for (master_socket = 0; master_socket <= fdmax; master_socket++) {

            if (FD_ISSET(master_socket, &rdset)) {
                if (master_socket == server_socket) {
                    /* A client is asking a new connection */
                    socklen_t addrlen;
                    struct sockaddr_in clientaddr;
                    int newfd;

                    /* Handle new connections */
                    addrlen = sizeof(clientaddr);
                    memset(&clientaddr, 0, sizeof(clientaddr));
                    newfd = accept(server_socket, (struct sockaddr *)&clientaddr, &addrlen);
                    if (newfd == -1) {
                        perror("Server accept() error");
                        continue;
                    }
                    FD_SET(newfd, &refset);

                    if (newfd > fdmax) {  /* Keep track of the maximum */
                        fdmax = newfd;
                    }
                    printf("New connection from %s:%d on socket %d\n",
                               inet_ntoa(clientaddr.sin_addr), clientaddr.sin_port, newfd);
                } else {
                    /* An already connected master has sent a new query */
                    uint8_t query[MODBUS_TCP_MAX_ADU_LENGTH];

                    modbus_set_socket(ctx, master_socket);
                    rc = modbus_receive(ctx, query);
                    if (rc != -1) {
                        modbus_to_spi(query, mb_mapping, arm1);
                        modbus_reply(ctx, query, rc, mb_mapping);
                    } else {
                        /* Connection closed by the client, end of server */
                        printf("Connection closed on socket %d\n", master_socket);
                        close(master_socket);

                        /* Remove from reference set */
                        FD_CLR(master_socket, &refset);

                        if (master_socket == fdmax) {
                            fdmax--;
                        }
                    }
                }
            }
        }
    }

    return 0;
}
