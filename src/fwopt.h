#ifndef __FWOPT_H
#define __FWOPT_H

#include "fwconfig.h"

extern int upboard;
extern int do_verify, do_prog, do_resetrw, do_calibrate, do_final, do_auto, do_upgrade, do_downgrade;

int parseopt(int argc, char **argv);

#endif
