
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>


typedef struct __attribute__ ((packed)) {
    uint16_t   signature;
    uint8_t   ver1[2];
    uint32_t  serial;
    uint8_t   flags[2];
    char      model[6];
    //uint8_t   fill[2];
} unipiversion;


unipiversion global_unipi_version;
char  unipi_name[sizeof(global_unipi_version.model)+1];
int unipi_loaded = 0;

int read_unipi_eprom( unipiversion *ver)
{
   int res;
   int i;

   unipi_loaded = 1;
   int f = open("/sys/bus/i2c/devices/1-0057/eeprom", O_RDONLY);
   if (f < 0) {
      f = open("/sys/bus/i2c/devices/0-0057/eeprom", O_RDONLY);
      if (f < 0) {
         return 1;
      }
   }
   res = lseek(f, 0x60, 0);
   if (res < 0) goto err;
   res = read(f,ver,sizeof(unipiversion));
   if (res < 0) goto err;
   if (ver->signature != 0x55fa) goto err;
   close(f);
   for (i=0; i<6; i++) { 
       if (ver->model[i]==0xff) {
          ver->model[i] = '\0';
          break;
       }
   }
   return 0;
err:
   close(f);
   return 1;
}

char* get_unipi_name(void)
{
	if (! unipi_loaded) {
		if (read_unipi_eprom(&global_unipi_version) == 0) {
			strncpy(unipi_name, global_unipi_version.model, sizeof(global_unipi_version.model));
			unipi_name[sizeof(global_unipi_version.model)] = '\0';
		} else {
			unipi_name[0] = '\0';
		}
	}
	return unipi_name;
}

uint32_t get_unipi_serial(void)
{
	if (! unipi_loaded) {
		if (read_unipi_eprom(&global_unipi_version) != 0) global_unipi_version.serial = 0;
	}
	return global_unipi_version.serial;
}

