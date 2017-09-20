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
 
# List all C defines here
#DDEFS = -DSTM32F0XX -DUSE_STDPERIPH_DRIVER
# -Dhw_v_0_4
#
# Define project name and Ram/Flash mode here
PROJECT        = neuron_tcp_server




# List C source files here
LIBSDIRS    = libmodbus-3.1.4/src/.libs
#CORELIBDIR = $(LIBSDIRS)/CMSIS/Include

LDFLAGS2 = -Llibmodbus-3.1.4/src/.libs libmodbus-3.1.4/src/.libs/libmodbus.so
ifdef SYSTEMROOT
LDFLAGS2 = -Llibmodbus-3.1.4/src/.libs libmodbus-3.1.4/src/.libs/libmodbus.dll.a
endif
DFLAGS2  = -Ilibmodbus-3.1.4/src

LDFLAGS3 = -L c:\MinGW\bin
DFLAGS3 = -Igtk/include/gtk-3.0 -Igtk/include/glib-2.0 -Igtk/lib/glib-2.0/include -Igtk/include/pango-1.0 -Igtk/include/cairo -Igtk/include/gdk-pixbuf-2.0 -Igtk/include/atk-1.0

#list of src files to include in build process

#SRC =  neuronmb.c
SPISRC = armspi.c
SPISRC += spicrc.c
SPISRC += armutil.c
SRC = $(SPISRC) nb_modbus.c armpty.c

# List all directories here
#INCDIRS = /usr/local/include/modbus
INCDIRS = libmodbus-3.1.4/src\
          libmodbus-3.1.4
          $(CORELIBDIR) \
          $(STMSPINCDDIR) \

# List the user directory to look for the libraries here
LIBDIRS += $(LIBSDIRS)
 
# List all user libraries here
LIBS = modbus util
# Define optimisation level here
#OPT = -Ofast
#OPT = -Os
 

INCDIR  = $(patsubst %,-I%, $(INCDIRS))
LIBDIR  = $(patsubst %,-L%, $(LIBDIRS))
LIB     = $(patsubst %,-l%, $(LIBS))
##reference only flags for run from ram...not used here
##DEFS    = $(DDEFS) $(UDEFS) -DRUN_FROM_FLASH=0 -DVECT_TAB_SRAM

#DEFS    = $(DDEFS) -DRUN_FROM_FLASH=1 -DHSE_VALUE=12000000

.PHONY += neuronspi

OBJS  = $(SRC:.c=.o)
SPIOBJS  = $(SPISRC:.c=.o)

#CPFLAGS = $(MCFLAGS) $(OPT) -g -gdwarf-3 -mthumb   -fomit-frame-pointer -Wall -Wstrict-prototypes -fverbose-asm -Wa,-ahlms=$(<:.c=.lst) $(DEFS)
#LDFLAGS = $(MCFLAGS) -g -gdwarf-3 -mthumb -nostartfiles -T$(LINKER_SCRIPT) -Wl,-Map=$(PROJECT).map,--cref,--no-warn-mismatch $(LIBDIR) $(LIB)
LDFLAGS = $(LIBDIR) $(LIB)

# SYSTEMROOT should be defined on windows, so we use it to detect the OS
ifdef SYSTEMROOT
   CC += -D OS_WIN32
endif

#
# makefile rules
#

all: $(OBJS) $(PROJECT) neuronspi fwspi fwserial

%.o: %.c
	$(CC) -c $(CPFLAGS) -I . $(INCDIR) $< -o $@

$(PROJECT): $(PROJECT).o $(OBJS)
	$(CC) $(PROJECT).o $(OBJS) $(LDFLAGS) -o $@

fwspi:  fwspi.o $(SPIOBJS)
	$(CC) fwspi.o $(SPIOBJS) -o $@

fwserial.o: fwserial.c
	$(CC) -c $(DFLAGS2)  $< -o $@

fwserial: fwserial.o armutil.o
	$(CC) $+ $(LDFLAGS2) $(DFLAGS2) -o fwserial
	chmod +x fwserial

# SYSTEMROOT should be defined on windows, so we use it to detect the OS
ifdef SYSTEMROOT
win32_serial.o: win32_serial.c
	$(CC) -Wno-deprecated-declarations -c $(DFLAGS2) $(DFLAGS3)  $< -o $@
	
fwserial-win: win32_serial.o armutil.o
	$(CC) $+ $(LDFLAGS2) $(LDFLAGS3) $(DFLAGS2) $(DFLAGS3) -o neuron_fw_utility -lgtk-3-0 -lglib-2.0-0 -lgobject-2.0-0
endif

neuronspi:
	$(shell cp neuronspi.c /root/kernel/neuron_spi/) 

bandwidth-client: bandwidth-client.o $(OBJS)
	$(CC) bandwidth-client.o $(OBJS) $(PKGC_FLAGS) $(LDFLAGS) -o $@

clean:
	-rm -rf $(OBJS) $(SPIOBJS) $(PROJECT).o armspi.o armutil.o neuronspi.o bandwidth-client.o win32_serial.o
	-rm -rf $(PROJECT).elf
	-rm -rf $(PROJECT).map
	-rm -rf $(PROJECT).hex
	-rm -rf $(PROJECT).bin
	-rm -rf $(PROJECT).rw
	-rm -rf $(SRC:.c=.lst)
	-rm -rf $(ASRC:.s=.lst)

install:
	mkdir -p $(DESTDIR)/opt/neurontcp
	install -m 0755 neuron_tcp_server $(DESTDIR)/opt/neurontcp
# *** EOF ***
