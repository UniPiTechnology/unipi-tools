/*OC
 * SPI communication with UniPi Neuron family controllers
 *
 * Copyright (c) 2016  Faster CZ, ondra@faster.cz
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License.
 *
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
#include <sys/mman.h>

#include "armspi.h"

#define MAX_WRITE_REGS 16
#define MAX_READ_REGS 16
uint16_t write_buf[MAX_WRITE_REGS+1];
uint16_t read_struct[2] = {0,0};
uint16_t read_buf[MAX_READ_REGS+1];

static struct option long_options[] = {
  {"verbose", no_argument,       0, 'v'},
  {"nsspause", required_argument, 0, 'n'},
  {"spidev", required_argument, 0, 's'},
  {"bauds",required_argument, 0, 'b'},
  {"read", required_argument, 0, 's'},
  {"write", required_argument, 0, 's'},
  {0, 0, 0, 0}
};

static void print_usage(const char *progname)
{
  printf("usage: %s [-s spidev] --b baudrate] [-v[v]] [-w reg,val[,val,val...] [-r reg,count]\n", progname);
  int i;
  for (i=0; ; i++) {
      if (long_options[i].name == NULL)  return;
      printf("  --%s%s\t %s\n", long_options[i].name, 
                                long_options[i].has_arg?"=...":"",
                                "");
  }
}

int parse_slist(char * option, char** results, int max_count)//, int maxlen)
{
    int i = 0;
    int len;
    char* p = option;
    char* np;

    for (i=0; i<max_count; i++) results[i] = NULL;
    i = 0;
    while (p != NULL) {
        if (i >= max_count) return 0;
        np = strchr(p, ',');
        if (np != NULL) {
            len = np - p;
            np++;
        } else {
            len = strlen(p);
        }
        results[i] = malloc(len+1);
        if (! results[i]) {
            printf("Error allocate string\n");
            abort();
        }
        strncpy(results[i],p,len);
        results[i][len+1] = '\0';
        p = np; i++;
    }
    return i;
}

int parse_ilist(char * option, uint16_t* results, int max_count)
{
    int i = 0;
    int len;
    char* p = option;
    char* np;

    for (i=0; i<max_count; i++) results[i] = 0;
    i = 0;
    while (p != NULL) {
        if (i >= max_count) return 0;
        np = strchr(p, ',');
        if (np != NULL) {
            np++;
        }
        results[i] = atoi(p);
        p = np; i++;
    }
    return i;
}

int main(int argc, char *argv[])
{
    int s, nss;
    int efd, n;
    int verbose = 0;
    int to_write = 0;
    char*  spi_device = "/dev/spidev0.0";
    int spi_speed = 8000000;

     // Options
    int c;
    while (1) {
       int option_index = 0;
       c = getopt_long(argc, argv, "vb:s:w:r:n:", long_options, &option_index);
       if (c == -1) {
           if (optind < argc)  {
               printf ("non-option ARGV-element: %s\n", argv[optind]);
               exit(EXIT_FAILURE);
            }
            break;
       }

       switch (c) {
       case 'v':
           verbose++;
           break;
       case 'b':
           spi_speed = atoi(optarg);
           //if (parse_ilist(optarg, spi_speed) == 0) {
           //    printf("Bad bauds count(1-3) (%s))\n", optarg);
           //    exit(EXIT_FAILURE);
           //}
           break;
       case 's':
           spi_device = strdup(optarg);
           break;
       case 'w':
           to_write = parse_ilist(optarg, write_buf, MAX_WRITE_REGS+1);
           if (to_write < 2) {
               printf("Write_reg option must be in form register,value[,value,value..]\n");
               exit(EXIT_FAILURE);
           }
           break;
       case 'r':
           n = parse_ilist(optarg, read_struct, 2);
           if (n < 1) {
               printf("Read_reg option must be in form register[,count]\n");
               exit(EXIT_FAILURE);
           }
           if (n == 1) read_struct[1] = 1;
           if (read_struct[1] > MAX_READ_REGS) { 
               printf("YOu can read max %d registers\n", MAX_READ_REGS);
               exit(EXIT_FAILURE);
           }
           break;
       case 'n':
           nss = atoi(optarg);
           if (nss <= 0) {
               printf("Nss pause must be non-zero integer (given %s)\n", optarg);
               exit(EXIT_FAILURE);
           } else {
               nss_pause = nss;
           }
           break;
       default:
           print_usage(argv[0]);
           exit(EXIT_FAILURE);
           break;
       }
    }


    arm_handle* arm = malloc(sizeof(arm_handle));

    arm_init(arm, spi_device, spi_speed, 0, NULL);

    idle_op(arm);
    if (to_write) {
       n = write_regs(arm, write_buf[0], to_write-1, write_buf+1);
    }

    if (read_struct[1] > 0) {
       int i;
       n = read_regs(arm, read_struct[0], read_struct[1], read_buf);
       /*printf("%5d:",read_struct[0]);
       for (i=0; i<n; i++) printf("%5d,",read_buf[i]);
       printf("\n");
       printf("%5d:",read_struct[0]);
       for (i=0; i<n; i++) printf(" %04x",read_buf[i]);
       printf("\n");*/
       for (i=0; i<n; i++) printf("%5d:  %04x  %5d\n",read_struct[0]+i, read_buf[i], read_buf[i]);

    }

    close(arm->fd);
    return 0;
}

