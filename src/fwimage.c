
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <errno.h>
#include <unistd.h>

#include "armutil.h"
#include "fwimage.h"
#include "fwconfig.h"

T_image_header* load_image_header(Tboard_version* bv)
{
    FILE* fd;
    char* fwname;
    uint16_t sw_version_bak;
    T_image_header* header = NULL;

    sw_version_bak = bv->sw_version;
    bv->sw_version = 0x600;
    fwname = firmware_name(bv, firmwaredir, ".img");

    if ((fd = fopen(fwname, "rb"))!=NULL) {
        header = malloc(sizeof(T_image_header));
        if (fread(header, 1, sizeof(*header), fd) != sizeof(*header)) {
            free(header);
            header = NULL;
        } 
        fclose(fd);
    }
    free(fwname);
    bv->sw_version = sw_version_bak;
    return header;
}

uint16_t get_image_version(Tboard_version* bv)
{
    uint16_t sw_version = 0;
    T_image_header* header = load_image_header(bv);
    if (header) {
		sw_version = header->swversion;
		free(header);
	}
	return sw_version;
}

int load_image(char* fname, T_image_header *header, void* prog_data, void* bootloader, void* rw_data)
{
    FILE* fd;

	vprintf_1("Loading image: %s\n", fname);
	if ((fd = fopen(fname, "rb")) == NULL) {
		eprintf("Cannot open file %s\n", fname);
		return -1;
	}
	if (fread(header, 1, sizeof(*header), fd) != sizeof(*header)) {
		eprintf("Cannot read header of image %s\n", fname);
		fclose(fd);
		return -1;
	}
	if (header->firmware_length > MAX_FW_SIZE) {
		eprintf("Firmware length > max %d\n", MAX_FW_SIZE);
		fclose(fd);
		return -1;
	}
	if (header->bootloader_length > MAX_BL_SIZE) {
		eprintf("Booloader length > max %d\n", MAX_BL_SIZE);
		fclose(fd);
		return -1;
	}
	if (header->rwdata_length > MAX_RW_SIZE) {
		eprintf("RW data length > max %d\n", MAX_RW_SIZE);
		fclose(fd);
		return -1;
	}
    //int x=fseek(fd, IMAGE_HEADER_LENGTH, SEEK_SET);
	//printf("%d %d\n", IMAGE_HEADER_LENGTH, x);
	if (fseek(fd, IMAGE_HEADER_LENGTH, SEEK_SET) < 0) {
		eprintf("Cannot seek to firmware in image %s\n", fname);
		fclose(fd);
		return -1;
	}

	if (prog_data != NULL) {
		if (fread(prog_data, 1, header->firmware_length, fd) != header->firmware_length) {
			eprintf("Cannot read firmware from image %s\n", fname);
			fclose(fd);
			return -1;
		}
	} else {
		if (fseek(fd, header->firmware_length, SEEK_CUR) < 0) {
	    	eprintf("Cannot seek to bootloader in image %s\n", fname);
			fclose(fd);
	    	return -1;
		}
	}
	if (bootloader != NULL) {
		if (fread(bootloader, 1, header->bootloader_length, fd) != header->bootloader_length) {
			eprintf("Cannot read bootloader from image %s\n", fname);
			fclose(fd);
			return -1;
		}
 	} else {
		if (fseek(fd, header->bootloader_length, SEEK_CUR) < 0) {
	    	eprintf("Cannot seek to rwdata in image %s\n", fname);
			fclose(fd);
	    	return -1;
		}
	}

	if (rw_data != NULL) {
		if (fread(rw_data, 1, header->rwdata_length, fd) != header->rwdata_length) {
			eprintf("Cannot read rwdata from image %s\n", fname);
			fclose(fd);
			return -1;
		}
	}
	return 0;
}




#define USART_CR1_M0     (uint32_t) 0x00001000   
#define USART_CR1_PS     (uint32_t) 0x00000200 
#define USART_CR1_PCE    (uint32_t) 0x00000400 
#define USART_CR2_STOP2  (uint32_t) 0x20000000

#define BRR_UHIGH 417   //  115200
#define BRR_HIGH 2500   // 19200

uint32_t bl_uart   		= 0;
uint32_t bl_uart_parity = 0;
uint32_t bl_uart_brr    = BRR_HIGH;


int setup_boot_context(int device_id, int baud, char parity, int stopbit)
{
	bl_uart = device_id;
    switch (parity) {
		case 'N': bl_uart_parity = 0; break;
		case 'E': bl_uart_parity = USART_CR1_M0 | USART_CR1_PCE; break;
		case 'O': bl_uart_parity = USART_CR1_M0 | USART_CR1_PCE | USART_CR1_PS; break;
		default: {
			eprintf("Incompatible parity setting %c\n", parity);
			return -1;
		}
	}
    switch (stopbit) {
		case 1: break;
		case 2: bl_uart_parity |= USART_CR2_STOP2; break;
		default: {
			eprintf("Incompatible stopbit setting %c\n", stopbit);
			return -1;
		}
	}
	switch (baud) {
		case  2400: bl_uart_brr = BRR_HIGH << 3; break;
		case  4800: bl_uart_brr = BRR_HIGH << 2; break;
		case  9600: bl_uart_brr = BRR_HIGH << 1; break;
		case 19200: bl_uart_brr = BRR_HIGH; break;
		case 38400: bl_uart_brr = BRR_HIGH >> 1; break;
		case 57600: bl_uart_brr = BRR_UHIGH << 1; break;
		case 115200: bl_uart_brr = BRR_UHIGH; break;
		default: {
			eprintf("Unsupported baudrate  %d\n", baud);
			return -1;
		}
	}
	return 0;
}

void patch_first_page(T_image_header *header, uint8_t* prog_data)
{
	*(uint32_t*)&(prog_data[header->status]) = 0xffffffff;
	*(uint32_t*)&(prog_data[header->bl_uart]) = bl_uart;
	*(uint32_t*)&(prog_data[header->bl_uart_brr]) = bl_uart_brr;
	*(uint32_t*)&(prog_data[header->bl_uart_parity]) = bl_uart_parity;
}

void patch_first_page_downgrade(T_image_header *header, uint8_t* prog_data)
{
	*(uint32_t*)&(prog_data[header->main_address]) = *(uint32_t*)&(prog_data[4]);
	*(uint32_t*)&(prog_data[header->status]) = 0xffff0000;
	*(uint32_t*)&(prog_data[header->bl_uart]) = bl_uart;
	*(uint32_t*)&(prog_data[header->bl_uart_brr]) = bl_uart_brr;
	*(uint32_t*)&(prog_data[header->bl_uart_parity]) = bl_uart_parity;
}

