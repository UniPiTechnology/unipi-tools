
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <errno.h>
#include <unistd.h>
#include "fwconfig.h"

/* Default parameters */
struct comopt_struct com_options = {
#ifdef FWSPI
	.PORT = "/dev/unipispi",
	.BAUD = 6000000,
	.DEVICE_ID = -1,
#endif
#ifdef FWSERIAL
	.PORT = NULL,
	.BAUD = 19200,
	.DEVICE_ID = 15,
	.parity = 'N',
	.timeout_ms = 800,
#endif
};

#ifdef OS_WIN32
char* firmwaredir = "./fw"
#else
char* firmwaredir = "/opt/unipi/firmware";
#endif

