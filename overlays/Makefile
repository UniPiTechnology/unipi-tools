
###
### devicetree overlays for i2c ans spi devices installed in Neuron PLC
###

DTSS = ds2482-overlay.dts neuronee-overlay.dts neuron-spi-overlay.dts neuron-spi-new-overlay.dts unipiee-overlay.dts
DTBS = $(DTSS:-overlay.dts=.dtbo)

LINUX_DIR_PATH = /lib/modules/$(shell uname -r)/build
ifdef CCPREFIX
    CCPAR = CROSS_COMPILE=${CCPREFIX}
endif
ifdef ARCH
    CCPAR += ARCH=${ARCH}
endif

PWD := $(shell pwd)

dtbo-y += $(DTBS)

targets += $(dtbo-y)
always  := $(dtbo-y)


all:
	make $(CCPAR) -C $(LINUX_DIR_PATH) M=$(PWD)

clean:
	make $(CCPAR) -C $(LINUX_DIR_PATH) M=$(PWD) clean


#%.dtbo: %.dts
#	dtc $< -o $@
##	dtc -@ $< -o $@ ## this is correct, but not supported by some version of dtc


#all: $(DTBS)

#clean:
#	@rm *.dtbo
