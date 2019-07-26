
DTSS = ds2482.dts neuronee.dts neuron-spi.dts neuron-spi-new.dts unipiee.dts
DTBS = $(DTSS:.dts=.dtbo)



%.dtbo: %.dts
	dtc -@ $< > $@


all: $(DTBS)

clean:
	@rm *.dtbo
