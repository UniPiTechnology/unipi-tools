/**********************
 *
 * Programming utility via ModBus
 *
 * Michal Petrilak 2016
 * Miroslav Ondra  2017
 *
 **********************/

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <getopt.h>
#include <errno.h>
#include <modbus/modbus.h>
#include <unistd.h>

#include "armutil.h"
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include "unipiutil.h"

/* Hardware constants */
#define PAGE_SIZE   1024            
#define REG_SIZE    64

#define MAX_FW_SIZE (64*PAGE_SIZE)
#define MAX_RW_SIZE (PAGE_SIZE)
#define RW_START_PAGE ((0xE000) / PAGE_SIZE)

/* Default parameters */
char* PORT = NULL;
int   BAUD = 19200;
int   DEVICE_ID = 15;
#ifdef OS_WIN32
char* firmwaredir = "./fw";
#else
char* firmwaredir = "/opt/unipi/firmware";
#endif
const char* version_string = "Version " PROJECT_VER;

int upboard;
//int verbose = 0;
int do_verify = 0;
int do_prog   = 0;
int do_resetrw= 0;
int do_calibrate= 0;
int do_final= 0;
int do_auto = 0;
int do_upgrade = 0;

#define vprintf_1(f, args...)	if (verbose>=1) printf(f, ##args)
#define vprintf_2(f, args...)	if (verbose>=2) printf(f, ##args)
#define eprintf(f, args...)	fprintf(stderr, f, ##args)

int load_fw(char *path, uint8_t* prog_data, const size_t len)
{
    FILE* fd;
    int read_n, i;
    fd = fopen(path, "rb");
    struct stat finfo;
#ifdef OS_WIN32
    fstat(fd->_file, &finfo);
    off_t filesize = finfo.st_size;
#endif
    if (!fd) {
        eprintf("error opening firmware file \"%s\"\n", path);
        return -1;
    }
    memset(prog_data, 0xff, len);

    read_n = fread(prog_data, 1, MAX_FW_SIZE, fd);
#ifdef OS_WIN32
    if (!read_n) {
    	for (int i = 0; i < filesize; i++) {
    		prog_data[i] = fgetc(fd);
    	}
    	read_n = filesize;
    }
    printf("READ: %d %x %d\n", read_n, prog_data[0], filesize);
#endif
    fclose(fd);
    return read_n;
}


int verify(modbus_t *ctx, uint8_t* prog_data, uint8_t* rw_data, int last_prog_page, int last_page)
{
    uint16_t* pd;
    int ret, chunk, page;
    uint16_t val, reg;

            //modbus_set_response_timeout(ctx, 2, 999999);
            pd = (uint16_t*) prog_data;
            for (page=0; page < last_page; page++) {
                vprintf_1("Verifying page %.2d ...", page);
                fflush(stdout);
                if (modbus_write_register(ctx, 0x7705, page) != 1) {   // set page address in the target device
                    eprintf("Verifying failed: %s\n", modbus_strerror(errno));
                    break;
                }
                for (chunk=0; chunk < 8; chunk++) {
                    if (modbus_write_registers(ctx, 0x7700+chunk, REG_SIZE, pd) != REG_SIZE) {; // send chunk of data
                        eprintf("Sending data failed: %s\n", modbus_strerror(errno));
                    }
                    pd += REG_SIZE;
                }
                if (modbus_read_registers(ctx, 0x7707, 1, &val) == 1) {
                    if (val == 0x100) {
                        vprintf_1(" OK\n");
                    } else {
                        vprintf_1(" NOT OK. errors = %d.\n", 0x100-val);
                    }
                } else {
                    eprintf("Verifying failed: %s\n", modbus_strerror(errno));
                    break;
                }
                if (page == last_prog_page-1) {
                    page = RW_START_PAGE-1;
                    pd = (uint16_t*) rw_data;
                }
            }
}

int flashit(modbus_t *ctx, uint8_t* prog_data, uint8_t* rw_data, int last_prog_page, int last_page)
{
    uint16_t* pd;
    int ret, chunk, page;
            // Programming
            //modbus_set_response_timeout(ctx, 1, 0);
            page = 0;
            int errors = 0;
            while (page < last_page) {
				if (!verbose) printf(".");
                vprintf_1("Programming page %.2d ...", page);
                fflush(stdout);
                if (page < last_prog_page) {
                    pd = (uint16_t*) (prog_data + page*PAGE_SIZE);
                } else {
                    pd = (uint16_t*) (rw_data + ((page-RW_START_PAGE)*PAGE_SIZE));
                }
                if (modbus_write_register(ctx, 0x7705, page) == 1) {   // set page address in the target device
                    for (chunk=0; chunk < 8; chunk++) {
                    	int retval = modbus_write_registers(ctx, 0x7700+chunk, REG_SIZE, pd);
                        if (retval == -1) { // send chunk of data (64*2 B)
                            errors++;
                        }
                        vprintf_2("Finished programming chunk %d, ret: %d err: %d\n", chunk, retval, errors);
                        pd += REG_SIZE;
                    }
                    if (modbus_write_register(ctx, 0x7707, 1) == 1) {  // write page to flash
                        vprintf_1("OK.\n");
                        page++;
                        if (page == last_prog_page) {
                            page = RW_START_PAGE;
                        }
                        //sleep(1);
                    } else {
                        errors++;
                        vprintf_1(" Trying again.\n");
                        eprintf("Flashing page failed: %s\n", modbus_strerror(errno));
                    }
                } else {
                    errors++;
                    vprintf_1(" Trying again.\n");
                }
                if (errors > 200) break;
            }
    
}



static struct option long_options[] = {
  {"verbose", no_argument,      0, 'v'},
  {"verify", no_argument,       0, 'V'},
  {"programm", no_argument,     0, 'P'},
  {"upgrade", no_argument,      0, 'U'},
  {"resetrw", no_argument,      0, 'R'},
  {"calibrate", no_argument,    0, 'C'},
  {"final", required_argument,  0, 'F'},
  {"port",  required_argument,  0, 'p'},
  {"baud",  required_argument,  0, 'b'},
  {"parity",required_argument,  0, 'r'},
  {"timeout",required_argument, 0, 't'},
  {"unit",    required_argument,0, 'u'},
  {"dir", required_argument,    0, 'd'},
  {"auto", no_argument,    		0, 'a'},
  {0, 0, 0, 0}
};

void print_usage(char *argv0)
{
    printf("\nUtility for Programming UniPi devices via ModBus RTU\n");
    printf("%s [-vVPRC] -p <port> [-u <mb address>] [-b <baudrate>] [-d <firmware dir>] [-F <upper board id>]\n", argv0);
    printf("\n");
    printf("--port <port>\t\t /dev/extcomm/1/0 or COM3\n");
    printf("--unit <mb address>\t default 15\n");
    printf("--baud <baudrate>\t default 19200\n");
    printf("--parity N|E|O\t default N\n");
    printf("--timeout in ms\t default 800\n");
    printf("--dir <firmware dir>\t default /opt/unipi/firmware\n");
    printf("--verbose\t show more messages\n");
    printf("--verify\t compare flash with file\n");
    printf("--programm\t write firmware to flash\n");
    printf("--resetrw\t check/rewrite also rw settings\n");
    printf("--calibrate\t write calibrating firmware to flash\n");
    printf("--final <upper board id or ?>\t write final firmware over calibrating\n");
    printf("--auto\t write new firmware\n");
    printf("\n");
}

int main(int argc, char **argv)
{
    uint8_t *prog_data;   // buffer containing firmware
    uint8_t* rw_data;     // buffer containing firmware rw data
    uint16_t* pd;
    int ret, chunk, page;
    uint16_t val, reg;
    modbus_t *ctx;
    FILE* fdx;
    uint32_t new_fwver;
	char parity = 'N';
	uint32_t timeout_ms = 800;
    uint32_t fw_upgrade;
    
    // Parse command line options
    int c;
    char *endptr;
    while (1) {
       int option_index = 0;
       c = getopt_long(argc, argv, "vVPRUCp:b:u:d:F:t:", long_options, &option_index);
       if (c == -1) {
           if (optind < argc)  {
               eprintf ("non-option ARGV-element: %s\n", argv[optind]);
               exit(EXIT_FAILURE);
            }
            break;
       }

       switch (c) {
       case 'v':
           verbose++;
           break;
       case 'V':
           do_verify = 1;
           break;
       case 'P':
           do_prog = 1;
           break;
       case 'R':
           do_resetrw = 1;
           break;
       case 'U':
           do_upgrade = 1;
           break;
       case 'C':
           do_calibrate = 1; do_prog = 1; do_resetrw = 1;
           break;
       case 'F':
           upboard = strtol(optarg, &endptr, 10);
           if ((endptr==optarg) || (!upboard_exists(upboard))) {
               printf("Available upper board ids:\n");
               print_upboards(-1);
               exit(EXIT_FAILURE);
           }
           do_final = 1; do_prog = 1; do_resetrw = 1;
           break;
       case 'p':
           PORT = strdup(optarg);
           break;
       case 'b':
           BAUD = atoi(optarg);
           if (BAUD==0) {
               eprintf("Baud must be non-zero integer (given %s)\n", optarg);
               exit(EXIT_FAILURE);
           }
           break;
       case 'r':
           parity = optarg[0];
           if (parity!='N' && parity != 'E' && parity != 'O') {
               eprintf("Parity must be N or E or O(given %s)\n", optarg);
               exit(EXIT_FAILURE);
           }
           break;
       case 'u':
           DEVICE_ID = atoi(optarg);
           if (DEVICE_ID==0) {
               eprintf("Unit must be non-zero integer (given %s)\n", optarg);
               exit(EXIT_FAILURE);
           }
           break;
       case 'd':
           firmwaredir = strdup(optarg);
           break;

       case 'a':
           do_auto=1;
           break;

       case 't':
           timeout_ms = atoi(optarg);
           if (timeout_ms <= 0) {
               eprintf("Timeout must be greater than zero integer (given %s)\n", optarg);
               exit(EXIT_FAILURE);
           }
           break;
       default:
           print_usage(argv[0]);
           exit(EXIT_FAILURE);
           break;
       }
    }

    if (verbose)
    	   printf("Fwserial: %s\n", version_string);

    if (PORT == NULL) {
        eprintf("Port device must be specified\n", optarg);
        print_usage(argv[0]);
        exit(EXIT_FAILURE);
    }

    // Open port
    ctx = modbus_new_rtu(PORT , BAUD, parity, 8, parity=='N'? 2:1);

    if (ctx == NULL) {
        eprintf("Unable to create the libmodbus context\n");
        return -1;
    }
    if ( verbose > 1) modbus_set_debug(ctx,verbose-1);
    modbus_set_slave(ctx, DEVICE_ID);

    if (modbus_connect(ctx) == -1) {
        eprintf("Connection failed: %s\n", modbus_strerror(errno));
        modbus_free(ctx);
        return -1;
    }

    // get FW & HW version
    uint16_t r1000[5];
    uint16_t r1022[128];
    Tboard_version bv;
    //int hw_version, sw_version, base_version;

    modbus_set_response_timeout(ctx, 0, timeout_ms*1000);
    //printf("Modbus timeout set\n");

    modbus_read_registers(ctx, 1000, 22, r1022);
    if (modbus_read_registers(ctx, 1000, 5, r1000) == 5) {
        parse_version(&bv, r1000);
        printf("Boardset:   %3d %-30s (v%d.%d%s)\n",
               HW_BOARD(bv.hw_version),  arm_name(bv.hw_version),
               HW_MAJOR(bv.hw_version), HW_MINOR(bv.hw_version),
               IS_CALIB(bv.hw_version)?" CAL":"");
        printf("Baseboard:  %3d %-30s (v%d.%d)\n",
               HW_BOARD(bv.base_hw_version),  arm_name(bv.base_hw_version),
               HW_MAJOR(bv.base_hw_version), HW_MINOR(bv.base_hw_version));
        printf("Firmware: v%d.%d\n", SW_MAJOR(bv.sw_version), SW_MINOR(bv.sw_version));
        if (fw_upgrade=check_firmware_upgrade(&bv, firmwaredir)) {
            vprintf("PLEASE UPGRADE FIRMWARE TO %d.%d - to proceed, execute fwspi -P -U\n", fw_upgrade>>8, fw_upgrade & 0xff);
        }
    } else {
        eprintf("Read version failed: %s\n", modbus_strerror(errno));
        modbus_free(ctx);
        return -1;
    }


    if (do_auto) { 
		if (new_fwver=check_new_rw_version(&bv, firmwaredir)) {
        	printf("NEW firmware=%d.%d found\n", new_fwver>>8, new_fwver & 0xff);
			do_prog=1;
    	}
	}

    //modbus_set_response_timeout(ctx, 0, 800000);
    if (do_prog || do_verify) {
        // FW manipulation
        if (do_calibrate) {
            bv.hw_version = bv.base_hw_version | 0x8;
            do_resetrw = 1;
        } else if (do_final) {
            if (!IS_CALIB(bv.hw_version)) {
                eprintf("Only calibrating version can be reprogrammed to final\n");
                modbus_free(ctx);
                return -1;
            }
            bv.hw_version = check_compatibility(bv.base_hw_version, upboard);
            if (bv.hw_version == 0) {
                eprintf("Incompatible base and upper boards. Use one of:\n");
                print_upboards(bv.base_hw_version);
                modbus_free(ctx);
                return -1;
            }
        } else if (do_upgrade) {
            bv.sw_version = (uint16_t)0x0600;
        }
        // load firmware file
        char* fwname = firmware_name(&bv, firmwaredir, ".bin");
        prog_data = malloc(MAX_FW_SIZE);
        vprintf_1("Opening firmware file: %s\n", fwname);
        int red = load_fw(fwname, prog_data, MAX_FW_SIZE);
        int rwred = RW_START_PAGE;
        free(fwname);
        if (red <= 0) {
            if (red == 0) {
                eprintf("Firmware file is empty!\n");
            } 
            free(prog_data);
            modbus_free(ctx);
            return -1;
        }
        red = (red + (PAGE_SIZE - 1)) / PAGE_SIZE;
        vprintf_1("Program pages: %d\n", red);
        int rwlen = 0;
        if (do_resetrw) {
            // load rw consts file
            rw_data = malloc(MAX_RW_SIZE);
            char* rwname = firmware_name(&bv, firmwaredir, ".rw");
            vprintf_2("Opening RW settings file: %s\n", rwname);
            rwlen = load_fw(rwname, rw_data, MAX_RW_SIZE);
            free(rwname);
            // calc page count of firmware file
            rwred += ((rwlen + (PAGE_SIZE - 1)) / PAGE_SIZE);
            vprintf_1("Final page: %d\n", rwred);
        }
        
        // init FW programmer
        if (modbus_write_bit(ctx, 1006, 1) != 1) {
            eprintf("Program mode setting failed: %s\n", modbus_strerror(errno));
            modbus_free(ctx);
            return -1;
        }
        if (do_prog || do_calibrate) {
            flashit(ctx,prog_data, rw_data, red, rwred);
        }
        if (do_verify) { 
            verify(ctx,prog_data, rw_data, red, rwred);
        }
        modbus_write_register(ctx, 0x7707, 3); // reboot
        // upgrade
        if (do_upgrade) {
            usleep(200000);
            vprintf_1("Upgrading.\n");
            modbus_write_register(ctx, 0x7707, 5); // upgrade_firmware_copy_struct
            usleep(200000);
            flashit(ctx,prog_data, rw_data, red, rwred);
            if (do_verify) { 
                verify(ctx,prog_data, rw_data, red, rwred);
            }
            modbus_write_register(ctx, 0x7707, 3); // reboot
        }
        free(prog_data);
    }
    modbus_free(ctx);
    return 0;
}
