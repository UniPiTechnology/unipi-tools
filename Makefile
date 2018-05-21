
BINFILES = neuron_tcp_server fwspi fwserial neuronhost
#ETCFILES = dnsmasq.conf  hostapd.conf  interfaces  
#SYSFILES = neurontcp.service neuronhost.service
INSTALL=install
HOST = $(shell ${CC} -dumpmachine | sed 's/-.*//')

BINPATH := $(BINFILES:%=bin/%)
#ETCPATH := $(ETCFILES:%=etc/%)
#SYSPATH := $(SYSFILES:%=systemd/%)

all: libmodbusx
	cd src; make; cd ..

libmodbusx:
	@echo ${CFLAGS}
	@echo ${LDFLAGS}
	@if [ -d libmodbus ]; then \
		cd libmodbus;\
		if ! git pull; then \
		  echo "****** Damaged libmodbus directory. Remove it!" >&2; exit 1;\
		fi; \
	 else\
		git clone git://github.com/stephane/libmodbus;\
	 fi
	@cd libmodbus;\
	 #export xCFLAGS="-g -O2 -fstack-protector-all";\
	 ./autogen.sh;\
	 ac_cv_func_malloc_0_nonnull=yes ./configure --host=${HOST} --enable-static --enable-shared=no --disable-tests;\
	 make clean; make;


clean:
	cd src; make clean; cd ..

install-bin:	$(BINPATH)
	@$(INSTALL) -d $(DESTDIR)/opt/neuron-bin
	@cp $(BINPATH) $(DESTDIR)/opt/neuron-bin

#install-etc:	$(ETCPATH)
#	@$(INSTALL) -d $(DESTDIR)/opt/unipiap/etc
#	@cp $(ETCPATH) $(DESTDIR)/opt/unipiap/etc

#install-sys:	$(SYSPATH)
#	@$(INSTALL) -d $(DESTDIR)/etc/systemd/system
#	@cp $(SYSPATH) $(DESTDIR)/etc/systemd/system


install: install-bin 


##	 ac_cv_func_malloc_0_nonnull=yes ./configure --host=armv7 --enable-static --enable-shared=no --disable-tests;\
##	 ac_cv_func_malloc_0_nonnull=yes ./configure --host=${HOST} --enable-static --enable-shared=no --disable-tests;\
