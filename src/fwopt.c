
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
#include "unipiutil.h"
#include "fwconfig.h"


int upboard;
int do_verify = 0;
int do_prog   = 0;
int do_resetrw= 0;
int do_calibrate= 0;
int do_final= 0;
int do_auto= 0;
int do_upgrade = 0;
int do_downgrade = 0;



static struct option long_options[] = {

  {"auto", no_argument,    		0, 'a'},
  {"verbose", no_argument,      0, 'v'},
  {"programm", no_argument,     0, 'P'},
  {"upgrade", no_argument,      0, 'U'},
  {"downgrade", no_argument,    0, 'D'},
  {"resetrw", no_argument,      0, 'R'},
  {"calibrate", no_argument,    0, 'C'},
  {"final", required_argument,  0, 'F'},
  {"verify", no_argument,       0, 'V'},
  {"dir", required_argument,    0, 'd'},
  {"baud",  required_argument,  0, 'b'},
#ifdef FWSERIAL
  {"port",  required_argument,  0, 'p'},
  {"parity",required_argument,  0, 'r'},
  {"timeout",required_argument, 0, 't'},
  {"unit",    required_argument,0, 'u'},
#endif
#ifdef FWSPI
  {"spidev",  no_argument,      0, 's'},
  {"index", required_argument,	0, 'i'},
#endif
  {0, 0, 0, 0}
};

void print_usage(char *argv0)
{
#ifdef FWSERIAL
    printf("\nUtility for Programming UniPi devices via ModBus RTU\n");
    printf("%s [-vVPRC] -p <port> [-u <mb address>] [-b <baudrate>] [-d <firmware dir>] [-F <upper board id>]\n", argv0);
    printf("\n");
    printf("--port <port>\t\t /dev/extcomm/1/0 or COM3\n");
    printf("--unit <mb address>\t default 15\n");
    printf("--baud <baudrate>\t default 19200\n");
    printf("--parity N|E|O\t default N\n");
    printf("--timeout in ms\t default 800\n");
    printf("--verify\t compare flash with file\n");
#endif
#ifdef FWSPI
    printf("\nUtility for Programming UniPi devices via SPI\n");
    printf("%s [-v] -a [-s <spidevice>] [ -i <index>] [-b <baudrate>] [-d <firmware dir>]\n", argv0);
    printf("%s [-vPRC] [-s <spidevice>] -i <index> [-b <baudrate>] [-d <firmware dir>] [-F <upper board id>]\n", argv0);
    printf("\n");
    printf("--index <index>\t\t [0...n] device index\n");
    printf("--spidev <spidev>\t\t /dev/unipispi \n");
    printf("--baud <baudrate>\t default 10000000\n");
#endif
    printf("--dir <firmware dir>\t default /opt/unipi/firmware\n");
    printf("--verbose\t show more messages\n");
    printf("--programm\t write firmware to flash\n");
    printf("--upgrade\t upgrade firmware from 5.x to 6.x\n");
    printf("--resetrw\t check/rewrite also rw settings\n");
    printf("--calibrate\t write calibrating firmware to flash\n");
    printf("--final <upper board id or ?>\t write final firmware over calibrating\n");
    printf("--auto \t\t autoupdate firmware\n");
    printf("\n");
}

#ifdef FWSERIAL
char* shortopt = "vVPRUDCp:b:u:d:F:t:";
#endif
#ifdef FWSPI
char* shortopt = "avPRUCs:b:d:F:i:";
#endif

int parseopt(int argc, char **argv)
{
    // Parse command line options
    int c;
    char *endptr;
    while (1) {
       int option_index = 0;
       c = getopt_long(argc, argv, shortopt, long_options, &option_index);
       if (c == -1) {
           if (optind < argc)  {
               printf ("non-option ARGV-element: %s\n", argv[optind]);
               return 1;
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
        case 'U':
           do_upgrade = 1;
           do_downgrade = 0;
           break;
        case 'D':
           if (! do_upgrade) do_downgrade = do_prog = 1;
           break;
       case 'C':
           do_calibrate = 1; do_prog = 1; do_resetrw = 1;
           break;
       case 'F':
           upboard = strtol(optarg, &endptr, 10);
           if ((endptr==optarg) || (!upboard_exists(upboard))) {
               printf("Available upper board ids:\n");
               print_upboards(-1);
               return 1;
           }
           do_final = 1; do_prog = 1; do_resetrw = 1;
           break;
       case 'b':
           com_options.BAUD = atoi(optarg);
           if (com_options.BAUD==0) {
               printf("Baud must be non-zero integer (given %s)\n", optarg);
               return 1;
           }
           break;
       case 'd':
           firmwaredir = strdup(optarg);
           break;
#ifdef FWSPI
       case 's':
           com_options.PORT = strdup(optarg);
           break;
       case 'i':
    	   com_options.DEVICE_ID = atoi(optarg);
    	   break;
#endif
#ifdef FWSERIAL
       case 'V':
           do_verify = 1;
           break;
       case 'p':
           com_options.PORT = strdup(optarg);
           break;
       case 'r':
           com_options.parity = optarg[0];
           if (com_options.parity!='N' && com_options.parity != 'E' && com_options.parity != 'O') {
               eprintf("Parity must be N or E or O(given %s)\n", optarg);
               return 1;
           }
           com_options.stopbit = com_options.parity=='N'? 2:1;
           break;
       case 'u':
           com_options.DEVICE_ID = atoi(optarg);
           if (com_options.DEVICE_ID==0) {
               eprintf("Unit must be non-zero integer (given %s)\n", optarg);
               return 1;
           }
           break;
       case 't':
           com_options.timeout_ms = atoi(optarg);
           if (com_options.timeout_ms <= 0) {
               eprintf("Timeout must be greater than zero integer (given %s)\n", optarg);
               return 1;
           }
           break;
#endif
       default:
           print_usage(argv[0]);
           return 1;
           break;
       }
    }

    if ((do_upgrade || do_downgrade) && (do_calibrate || do_final)) {
        eprintf("Cannot combine upgrade with -C or -F\n");
        return 1;
    }

    if (com_options.PORT == NULL) {
        eprintf("Port device must be specified\n");
        print_usage(argv[0]);
        return 1;
    }
    if ((com_options.DEVICE_ID) < 0 && ! do_auto) {
        eprintf("Device index must be defined\n");
        print_usage(argv[0]);
        return 1;
	}
    return 0;
}
