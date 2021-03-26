#ifndef __FWIMAGE_H
#define __FWIMAGE_H

typedef struct __attribute__((__packed__)) {
    uint32_t swversion;
    uint32_t hwversion;
    uint32_t firmware_length;
    uint32_t bootloader_length;
    uint32_t bootloader_start;
    uint32_t rwdata_length;
    uint32_t rwdata_start;
    uint32_t main_address;
    uint32_t status;
    uint32_t bl_uart;
    uint32_t bl_uart_brr;
    uint32_t bl_uart_parity;
} T_image_header;

#define IMAGE_HEADER_LENGTH 256
//#define MAX_RW_SIZE 0x1000
#define MAX_BL_SIZE 0x1000

int setup_boot_context(int device_id, int baud, char parity, int stopbit);
void patch_first_page(T_image_header *header, uint8_t* prog_data);
void patch_first_page_downgrade(T_image_header *header, uint8_t* prog_data);

int load_image(char* fname, T_image_header *header, void* prog_data, void* bootloader, void* rw_data);
T_image_header* load_image_header(Tboard_version* bv);
uint16_t get_image_version(Tboard_version* bv);


int load_bin(Tboard_version* bv, T_image_header *header, void* prog_data, void* rw_data);
uint16_t get_bin_version(Tboard_version* bv);

#endif
