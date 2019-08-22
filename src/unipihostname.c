
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "unipiutil.h"

int main(int argc, char** argv)
{
   int res;
   int do_set = 0;
   int i;
   char hostname[256];
   char *unipi_model;

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

   unipi_model = get_unipi_name();
   if (unipi_model[0]=='\0') return 1;

   sprintf(hostname, "%s-sn%d", unipi_model, get_unipi_serial());
   //printf();
   if (do_set) {
       sethostname(hostname,strlen(hostname));
   } else {
       printf("%s\n", hostname);
   }
   return 0;
}
