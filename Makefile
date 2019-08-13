BINFILES = unipi_tcp_server fwspi fwserial unipihost unipicheck
HOST = $(shell ${CC} -dumpmachine | sed 's/-.*//')
INSTALL = install

#BINPATH := $(BINFILES:%=src/%)


#ifeq (armhf,$(filter armhf,$(ARCH) $(DEB_TARGET_ARCH)))
#overlaysd: overlays/*.dts
#	cd overlays; make; cd ..
#else
#overlaysd: 
#endif
.PHONY: all libmodbusx

all: libmodbusx
	cd src; make; cd ..
	if [ "$(ARCH)" = "arm" -o "$(DEB_TARGET_ARCH)" = "armhf" ]; then \
	  cd overlays; make LINUX_DIR_PATH=${LINUX_DIR_PATH}; cd ../.. ; \
	fi

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
	@rm -rf libmodbus

install:
	$(INSTALL) -D $(BINFILES:%=src/%) -t $(DESTDIR)/opt/unipi/tools
	$(INSTALL) -D src/unipi-target.map -t $(DESTDIR)/opt/unipi/data
	if [ "$(ARCH)" = "arm" -o "$(DEB_TARGET_ARCH)" = "armhf" ]; then $(INSTALL) -D overlays/*.dtbo -t $(DESTDIR)/boot/overlays ; fi

mr-proper:
	@make clean
	@rm -rf libmodbus
	@dh_clean
