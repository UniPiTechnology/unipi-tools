
DTSS = ds2482.dts neuronee.dts neuron-spi.dts neuron-spi-new.dts unipiee.dts
DTBS = $(DTSS:.dts=.dtbo)



%.dtbo: %.dts
	dtc -@ $< -o $@

#	dtc -@ $< -o $@ ## this is correct, but not supported by some version of dtc


all: $(DTBS)

clean:
	@rm *.dtbo
