#ifndef __FWCONFIG_H
#define __FWCONFIG_H

//#define FWSERIAL 1
//#define FWSPI 1

struct comopt_struct {
	char* PORT;
	int   BAUD;
	int   DEVICE_ID;
	char  parity;
	int   stopbit;
	int   timeout_ms;
};

/* Default parameters */
extern struct comopt_struct com_options;
extern char* firmwaredir;

#endif
