
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
    char      model[4];
    uint8_t   fill[2];
} unipiversion;

int main(int argc, char** argv)
{
   int res;
   int do_set = 0;
   int i;
   unipiversion ver;
   char hostname[256];

   if (argc > 1) {
      if (gethostname(hostname, sizeof(hostname))) return 1;
      //printf("%s\n", hostname);
      i = 1;
      while (i < argc) {
          if (strcmp(hostname, argv[i]) == 0) {
              do_set = 1;
              break;
          }
          i++;
      }
      if (do_set == 0) return 0;
   }

   int f = open("/sys/bus/i2c/devices/1-0057/eeprom", O_RDONLY);
   if (f < 0) {
      f = open("/sys/bus/i2c/devices/0-0057/eeprom", O_RDONLY);
      if (f < 0) {
         return 1;
      }
   }
   res = lseek(f, 0x60, 0);
   if (res < 0) goto err;
   res = read(f,&ver,sizeof(ver));
   if (res < 0) goto err;
   close(f);
   if (ver.signature != 0x55fa) goto err;
   ver.fill[0] = 0;
   sprintf(hostname, "%s-sn%d", ver.model, ver.serial);
   //printf();
   if (do_set) {
       sethostname(hostname,strlen(hostname));
   } else {
       printf("%s\n", hostname);
   }
   return 0;
err:
   close(f);
   return 1;
}