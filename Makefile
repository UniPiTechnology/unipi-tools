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
CC   = $(CCPREFIX)gcc
CP   = $(CCPREFIX)objcopy
AS   = $(CCPREFIX)gcc -x assembler-with-cpp
 
# List all C defines here
#DDEFS = -DSTM32F0XX -DUSE_STDPERIPH_DRIVER
# -Dhw_v_0_4
#
# Define project name and Ram/Flash mode here
PROJECT        = neuron_tcp_server
 
# List C source files here
LIBSDIRS    = libmodbus-master/src/.libs
CORELIBDIR = $(LIBSDIRS)/CMSIS/Include

#list of src files to include in build process

#SRC =  neuronmb.c
SRC = armspi.c
SRC += spicrc.c

# List all directories here
INCDIRS = libmodbus-master/src
#          $(CORELIBDIR) \
#          $(STMSPINCDDIR) \

# List the user directory to look for the libraries here
LIBDIRS += $(LIBSDIRS)
 
# List all user libraries here
LIBS = modbus
 
# Define optimisation level here
#OPT = -Ofast
#OPT = -Os
 

INCDIR  = $(patsubst %,-I%, $(INCDIRS))
LIBDIR  = $(patsubst %,-L%, $(LIBDIRS))
LIB     = $(patsubst %,-l%, $(LIBS))
##reference only flags for run from ram...not used here
##DEFS    = $(DDEFS) $(UDEFS) -DRUN_FROM_FLASH=0 -DVECT_TAB_SRAM

#DEFS    = $(DDEFS) -DRUN_FROM_FLASH=1 -DHSE_VALUE=12000000

OBJS  = $(SRC:.c=.o)

#CPFLAGS = $(MCFLAGS) $(OPT) -g -gdwarf-3 -mthumb   -fomit-frame-pointer -Wall -Wstrict-prototypes -fverbose-asm -Wa,-ahlms=$(<:.c=.lst) $(DEFS)
#LDFLAGS = $(MCFLAGS) -g -gdwarf-3 -mthumb -nostartfiles -T$(LINKER_SCRIPT) -Wl,-Map=$(PROJECT).map,--cref,--no-warn-mismatch $(LIBDIR) $(LIB)
LDFLAGS = $(LIBDIR) $(LIB)

#
# makefile rules
#

all: $(OBJS) $(PROJECT) neuronspi

%.o: %.c
	$(CC) -c $(CPFLAGS) -I . $(INCDIR) $< -o $@

$(PROJECT): $(PROJECT).o $(OBJS)
	$(CC) $(PROJECT).o $(OBJS) $(LDFLAGS) -o $@

neuronspi: neuronspi.o $(OBJS)
	$(CC) neuronspi.o $(OBJS) -o $@

clean:
	-rm -rf $(OBJS)
	-rm -rf $(PROJECT).elf
	-rm -rf $(PROJECT).map
	-rm -rf $(PROJECT).hex
	-rm -rf $(PROJECT).bin
	-rm -rf $(PROJECT).rw
	-rm -rf $(SRC:.c=.lst)
	-rm -rf $(ASRC:.s=.lst)
# *** EOF ***
