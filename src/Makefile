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

DSTDIR = ../bin

# List all user libraries here
LIBS = util modbus

#LDFLAGS2 = -Llibmodbus-3.1.4/src/.libs -l modbus 

# SYSTEMROOT should be defined on windows, so we use it to detect the OS
ifdef SYSTEMROOT
#LDFLAGS2 = -Llibmodbus-3.1.4/src/.libs libmodbus-3.1.4/src/.libs/libmodbus.dll.a
#DFLAGS2  = -Ilibmodbus-3.1.4/src
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
SRC = $(SPISRC) nb_modbus.c

 
# Define optimisation level here
#OPT = -Ofast
#OPT = -Os

INCDIR  = $(patsubst %,-I%, $(INCDIRS))
LIBDIR  = $(patsubst %,-L%, $(LIBDIRS))
LIB     = $(patsubst %,-l%, $(LIBS))

OBJS  = $(SRC:.c=.o)
SPIOBJS  = $(SPISRC:.c=.o)

#CPFLAGS = $(MCFLAGS) $(OPT) -g -gdwarf-3 -mthumb   -fomit-frame-pointer -Wall -Wstrict-prototypes -fverbose-asm -Wa,-ahlms=$(<:.c=.lst) $(DEFS)
#LDFLAGS = $(MCFLAGS) -g -gdwarf-3 -mthumb -nostartfiles -T$(LINKER_SCRIPT) -Wl,-Map=$(PROJECT).map,--cref,--no-warn-mismatch $(LIBDIR) $(LIB)
LDFLAGS = $(LIBDIR) $(LIB)

#
# makefile rules
#
all: $(OBJS) $(DSTDIR)/neuron_tcp_server $(DSTDIR)/fwspi $(DSTDIR)/fwserial

%.o: %.c
	$(CC) -c $(LDFLAGS) -I . $(INCDIR) $< -o $@

$(DSTDIR)/neuron_tcp_server: neuron_tcp_server.o $(OBJS) 
	$(CC) neuron_tcp_server.o $(OBJS) $(LDFLAGS) -o $@
	@chmod +x $@

$(DSTDIR)/fwspi:  fwspi.o $(OBJS)
	$(CC) fwspi.o $(OBJS) $(LDFLAGS) -o $@
	@chmod +x $@
	
#fwserial.o: fwserial.c
#	$(CC) -c $(DFLAGS2)  $< -o $@

$(DSTDIR)/fwserial: fwserial.o armutil.o
	$(CC) $+ $(LDFLAGS) -o $@
	@chmod +x $@

# SYSTEMROOT should be defined on windows, so we use it to detect the OS
ifdef SYSTEMROOT
win32_serial.o: win32_serial.c
	$(CC) -Wno-deprecated-declarations -c $(LDFLAGS) -I . $(INCDIR)  $< -o $@
	
fwserial-win: win32_serial.o armutil.o
	$(CC) $+ $(LDFLAGS2) $(LDFLAGS) -o neuron_fw_utility
endif

bandwidth-client: bandwidth-client.o
	$(CC) bandwidth-client.o $(LDFLAGS) -o $@

.PHONY: fwspi neuron_tcp_server fwserial

neuron_tcp_server:
	@rm -f $(DSTDIR)/$@
	$(MAKE) $(DSTDIR)/$@

fwspi:
	@rm -f $(DSTDIR)/$@
	$(MAKE) $(DSTDIR)/$@

fwserial:
	@rm -f $(DSTDIR)/$@
	$(MAKE) $(DSTDIR)/$@


clean:
	-rm -rf $(OBJS) $(SPIOBJS) neuron_tcp_server.o fwspi.o fwserial.o win32_serial.o bandwidth-client.o
	-rm -rf $(PROJECT).elf
	-rm -rf $(PROJECT).map
	-rm -rf $(PROJECT).hex
	-rm -rf $(PROJECT).bin
	-rm -rf $(PROJECT).rw
	-rm -f ../bin/*
	-rm -rf $(SRC:.c=.lst)

#install:
#	mkdir -p $(DESTDIR)/opt/neurontcp
#	install -m 0755 neuron_tcp_server $(DESTDIR)/opt/neurontcp
# *** EOF ***
