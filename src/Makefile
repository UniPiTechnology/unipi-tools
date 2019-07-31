#
#       !!!! Do NOT edit this makefile with an editor which replace tabs by spaces !!!!   
#
##############################################################################################
#
# On command line:
#
# make all = Create project
#
# make clean = Clean project files.
#
# To rebuild project do "make clean" and "make all".
#
# Included originally in the yagarto projects. Original Author : Michael Fischer
# Modified to suit our purposes by Hussam Al-Hertani
# Use at your own risk!!!!!
##############################################################################################
# Start of default section
#
CC   = $(CCPREFIX)gcc -g
CP   = $(CCPREFIX)objcopy
AS   = $(CCPREFIX)gcc -x assembler-with-cpp
INSTALL=install
 
# standard version
#MODBUS_LIBDIR = /usr/lib/modbus
#INCDIRS = /usr/include/modbus

# But we need libmodbus version >= 3.1.4 
#   - older versions listen on all interface 
#   - there was problem with tty name length
MODBUSDIR = ../libmodbus
MODBUS_LIBDIR = $(MODBUSDIR)/src/.libs
INCDIRS       = $(MODBUSDIR)/src\
                $(MODBUSDIR)

LIBDIRS += $(MODBUS_LIBDIR)

DSTDIR = .
#DSTDIR = ../bin

# List all user libraries here
LIBS = modbus m

# SYSTEMROOT should be defined on windows, so we use it to detect the OS
ifdef SYSTEMROOT
  CC += -D OS_WIN32
  LDFLAGS2 = $(MODBUS_LIBDIR)/libmodbus.dll.a
  LIBDIRS += c:\MinGW\bin
  INCDIRS += gtk/include/gtk-3.0\
             gtk/include/glib-2.0\
             gtk/lib/glib-2.0/include\
             gtk/include/pango-1.0\
             gtk/include/cairo\
             gtk/include/gdk-pixbuf-2.0\
             gtk/include/atk-1.0
  LIBS    += gtk-3-0\
             glib-2.0-0\
             gobject-2.0-0
endif

#list of src files to include in build process
SPISRC = armspi.c
SPISRC += spicrc.c
SPISRC += armutil.c
SPISRC += unipiutil.c
SRC = $(SPISRC) nb_modbus.c virtual_regs.c

BINFILES = unipi_tcp_server fwspi fwserial unipihost unipicheck
 
# Define optimisation level here
#OPT = -Ofast
#OPT = -Os

INCDIR  = $(patsubst %,-I%, $(INCDIRS))
LIBDIR  = $(patsubst %,-L%, $(LIBDIRS))
LIB     = $(patsubst %,-l%, $(LIBS))

OBJS  = $(SRC:.c=.o)
SPIOBJS  = $(SPISRC:.c=.o)

LDFLAGS = $(LIBDIR) $(LIB)

#
# makefile rules
#
all: $(BINFILES)

%.o: %.c
	$(CC) -c $(LDFLAGS) -I . $(INCDIR) $< -o $@


.PHONY: fwspi unipi_tcp_server fwserial

unipi_tcp_server: unipi_tcp_server.o $(OBJS) 
	$(CC) unipi_tcp_server.o $(OBJS) $(LDFLAGS) -o $@
	@chmod +x $@

fwspi:  fwspi.o $(OBJS)
	$(CC) fwspi.o $(OBJS) $(LDFLAGS) -o $@
	@chmod +x $@
	
fwserial: fwserial.o armutil.o
	$(CC) $+ $(LDFLAGS) -o $@
	@chmod +x $@

unipihost:  unipihost.o unipiutil.o
	$(CC) unipihost.o unipiutil.o $(LDFLAGS) -o $@
	@chmod +x $@

unipicheck:  unipicheck.o unipiutil.o
	$(CC) unipicheck.o unipiutil.o -lm -o $@
	@chmod +x $@
	
bandwidth-client: bandwidth-client.o
	$(CC) bandwidth-client.o $(LDFLAGS) -o $@


# SYSTEMROOT should be defined on windows, so we use it to detect the OS
ifdef SYSTEMROOT
win32_serial.o: win32_serial.c
	$(CC) -Wno-deprecated-declarations -c $(LDFLAGS) -I . $(INCDIR)  $< -o $@
	
fwserial-win: win32_serial.o armutil.o
	$(CC) $+ $(LDFLAGS2) $(LDFLAGS) -o neuron_fw_utility
endif

install:
	$(INSTALL) -D $(BINFILES) -t $(DESTDIR)/opt/unipi/tools


clean:
	-rm -f *.o
	-rm -f $(BINFILES)
#	-rm -rf $(SRC:.c=.lst)
#	-rm -f ../bin/*

# *** EOF ***
