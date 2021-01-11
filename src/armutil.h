#ifndef __ARMUTIL_H
#define __ARMUTIL_H

#ifdef __cplusplus
 extern "C" {
#endif

#define vprintf( ... ) if (verbose > 0) printf( __VA_ARGS__ )
#define vvprintf( ... ) if (verbose > 1) printf( __VA_ARGS__ )
extern int verbose;

/* Hardware constants */
#define PAGE_SIZE   1024
#define REG_SIZE    64

#define MAX_FW_SIZE (64*PAGE_SIZE)
#define MAX_RW_SIZE (PAGE_SIZE)
#define MIN_FW_SIZE (2*PAGE_SIZE)
#define MIN_RW_SIZE (8)

#define RW_START_PAGE ((0xE000) / PAGE_SIZE)


#define IS_CALIB(hw)  (((hw) & 0x8) != 0)
#define HW_BOARD(hw)  ((hw) >> 8)
#define HW_MAJOR(hw)  (((hw) & 0xf0) >> 4)
#define HW_MINOR(hw)  ((hw) & 0x07)

#define SW_MAJOR(sw)  ((sw) >> 8)
#define SW_MINOR(sw)  ((sw) & 0xff)

typedef struct {
  uint16_t sw_version;
  uint16_t hw_version;
  uint16_t base_hw_version;
  uint8_t di_count;
  uint8_t do_count;
  uint8_t ai_count;
  uint8_t ao_count;
  uint8_t uart_count;
  // ------- not in register block 1000+
  uint16_t uled_count;
  uint16_t int_mask_register;
} Tboard_version;

typedef struct {
  uint8_t board;
  uint8_t subver;
  const char*  name;
} Tboards_map;



#ifndef __EXTENSION_BOARDS_MAP
#define __EXTENSION_BOARDS_MAP
#define EXTENSION_COUNT 8
typedef struct {
	uint8_t board;
	uint8_t ext_board;
	const char* product;
} Textension_map;
#endif

int parse_version(Tboard_version* bv, uint16_t *r1000);
const char* arm_name(uint16_t hw_version);//int sw_version, int hw_version);
char* firmware_name(Tboard_version* bv, const char* fwdir, const char* ext);
void print_upboards(int filter);
int upboard_exists(int board);
int check_compatibility(int hw_base, int upboard);
int get_board_speed(Tboard_version* bv);

uint32_t check_new_rw_version(Tboard_version* bv, const char* fwdir);
uint8_t* load_fw_file(Tboard_version* bv, const char* fwdir, int rw, int* datalen);
uint32_t check_firmware_update(Tboard_version* bv, const char* fwdir);

Textension_map* get_extension_map(int board);


#endif
