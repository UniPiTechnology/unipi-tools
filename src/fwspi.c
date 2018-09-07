/**********************
 *
 * Programming utility via SPI
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
#include <unistd.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/mman.h>

#include "armspi.h"
#include "armutil.h"
#include "nb_modbus.h"


/* Default parameters */
char* PORT = NULL;
char* INDEX = NULL;
int   BAUD = 8000000;
#ifdef OS_WIN32
char* firmwaredir = "./fw"
#else
char* firmwaredir = "/opt/fw";
#endif
int device_index;
int upboard;
//int verbose = 0;
int do_verify = 0;
int do_prog   = 0;
int do_resetrw= 0;
int do_calibrate= 0;
int do_final= 0;
int do_auto= 0;


#define vprintf( ... ) if (verbose > 0) printf( __VA_ARGS__ )
#define vvprintf( ... ) if (verbose > 1) printf( __VA_ARGS__ )


static struct option long_options[] = {
  {"auto", no_argument,         0, 'a'},
  {"verbose", no_argument,      0, 'v'},
  {"programm", no_argument,     0, 'P'},
  {"resetrw", no_argument,      0, 'R'},
  {"calibrate", no_argument,    0, 'C'},
  {"final", required_argument,  0, 'F'},
  {"spidev",  no_argument,      0, 's'},
  {"baud",  required_argument,  0, 'b'},
  {"dir", required_argument,    0, 'd'},
  {"index", required_argument,	0, 'i'},
  {0, 0, 0, 0}
};

void print_usage(char *argv0)
{
    printf("\nUtility for Programming UniPi devices via ModBus RTU\n");
    printf("%s [-v] -a [-s <spidevice>] [ -i <index>] [-b <baudrate>] [-d <firmware dir>]\n", argv0);
    printf("%s [-vPRC] [-s <spidevice>] -i <index> [-b <baudrate>] [-d <firmware dir>] [-F <upper board id>]\n", argv0);
    printf("\n");
    printf("--auto \t\t autoupdate firmware\n");
    printf("--index <index>\t\t [0...n] device index\n");
    printf("--spidev <spidev>\t\t /dev/unipispi \n");
    printf("--baud <baudrate>\t default 10000000\n");
    printf("--dir <firmware dir>\t default /opt/fw\n");
    printf("--verbose\t show more messages\n");
    printf("--programm\t write firmware to flash\n");
    printf("--resetrw\t check/rewrite also rw settings\n");
    printf("--calibrate\t write calibrating firmware to flash\n");
    printf("--final <upper board id or ?>\t write final firmware over calibrating\n");
    printf("\n");
}

int main(int argc, char **argv)
{
    uint8_t *prog_data;   // buffer containing firmware
    uint8_t* rw_data;     // buffer containing firmware rw data
    uint16_t* pd;
    int ret, chunk, page;
    int max_device_index;
    uint16_t val, reg;
    arm_handle *arm;
    FILE* fdx;
    
    // Parse command line options
    int c;
    char *endptr;
    while (1) {
       int option_index = 0;
       c = getopt_long(argc, argv, "avPRCs:b:d:F:i:", long_options, &option_index);
       if (c == -1) {
           if (optind < argc)  {
               printf ("non-option ARGV-element: %s\n", argv[optind]);
               exit(EXIT_FAILURE);
            }
            break;
       }

       switch (c) {
       case 'a':
           do_auto=1;
           break;
       case 'v':
           verbose++;
           break;
       case 'P':
           do_prog = 1;
           break;
       case 'R':
           do_resetrw = 1;
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
       case 's':
           PORT = strdup(optarg);
           break;
       case 'b':
           BAUD = atoi(optarg);
           if (BAUD==0) {
               printf("Baud must be non-zero integer (given %s)\n", optarg);
               exit(EXIT_FAILURE);
           }
           break;
       case 'd':
           firmwaredir = strdup(optarg);
           break;
       case 'i':
    	   INDEX = strdup(optarg);
    	   device_index = atoi(INDEX);
    	   break;
       default:
           print_usage(argv[0]);
           exit(EXIT_FAILURE);
           break;
       }
    }
    if ((INDEX == NULL) && (do_auto == 0)) {
    	printf("Device index must be specified\n", optarg);
        print_usage(argv[0]);
        exit(EXIT_FAILURE);
    }
    if (PORT == NULL) PORT = "/dev/unipispi";

    if (verbose > 0) arm_verbose = verbose;

    arm = malloc(sizeof(arm_handle));
    // autoupdate 
    if (do_auto) {
        if (INDEX == NULL) {
            max_device_index = 2;
            device_index = 0;
        } else {
            max_device_index = device_index;
        }
        for (;device_index <= max_device_index; device_index++) {
    		//arm = calloc(1,sizeof(arm_handle));
            if (arm_init(arm, PORT , BAUD, device_index) >= 0) {
                arm_firmware(arm, firmwaredir);
                close(arm->fd);
            }
		}
        free(arm);
		if (verbose) printf("Firmware autoupdate finished\n");
		return 0;
    }

    //arm = malloc(sizeof(arm_handle));
    if ( arm_init(arm, PORT , BAUD, device_index) < 0) {
        fprintf(stderr, "Unable to create the arm[%d] context\n", device_index);
        free(arm);
        return -1;
    }

    Tboard_version *bv = &(arm->bv);
    printf("Boardset:   %3d %-30s (v%d.%d%s)\n",
               HW_BOARD(bv->hw_version), arm_name(bv->hw_version),
               HW_MAJOR(bv->hw_version), HW_MINOR(bv->hw_version),
               IS_CALIB(bv->hw_version)?" CAL":"");
    printf("Baseboard:  %3d %-30s (v%d.%d)\n",
               HW_BOARD(bv->base_hw_version),  arm_name(bv->base_hw_version),
               HW_MAJOR(bv->base_hw_version), HW_MINOR(bv->base_hw_version));
    printf("Firmware: v%d.%d\n", SW_MAJOR(bv->sw_version), SW_MINOR(bv->sw_version));

    
    if (do_prog) {
        // FW manipulation
        if (do_calibrate) {
            // combine base_board + MAJ/MIN of hw_version + CAL
            arm->bv.hw_version = (arm->bv.base_hw_version & 0xff00) |\
                                 (arm->bv.hw_version & 0xff | 0x8);
            do_resetrw = 1;
        } else if (do_final) {
            if (!(arm->bv.hw_version & 0x8)) {
                fprintf(stderr, "Only calibrating version can be reprogrammed to final\n");
                close(arm->fd);
                free(arm);
                return -1;
            }
            arm->bv.hw_version = check_compatibility(arm->bv.base_hw_version, upboard);
            if (arm->bv.hw_version == 0) {
                fprintf(stderr, "Incompatible base and upper boards. Use one of:\n");
                print_upboards(arm->bv.base_hw_version);
                close(arm->fd);
                free(arm);
                return -1;
            }
        }
        if (do_prog || do_calibrate) {
            arm_firmware_do(arm, firmwaredir, do_resetrw);
        }
    }
    close(arm->fd);
    free(arm);
    return 0;
}
