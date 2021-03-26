#ifndef __FWDRIVER_H
#define __FWDRIVER_H

#include "armutil.h"
#include "fwconfig.h"

struct page_description {
	uint32_t flash_addr;
	uint8_t* data;
	int errors;
};


struct driver {
	void* (*open)(struct comopt_struct*);
	void (*reopen)(void*, struct comopt_struct*);
	void (*close)(void*);
	Tboard_version*(*identify)(void*);
	int  (*start)(void*);
	int  (*run)(void*);
	int  (*confirm)(void*);
	int  (*reboot)(void*);
	int  (*flash)(void*, struct page_description *, int, int);
};

extern struct driver driver;

#endif
