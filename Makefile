
BINFILES = unipi_tcp_server fwspi fwserial unipihost
HOST = $(shell ${CC} -dumpmachine | sed 's/-.*//')
INSTALL = install

#BINPATH := $(BINFILES:%=src/%)

all: libmodbusx
	cd src; make; cd ..
	if [ "$(ARCH)" = "arm" -o "$(DEB_TARGET_ARCH)" = "armhf" ]; then cd unipi-common/dts; make; cd ../.. ; fi

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
	$(INSTALL) -D $(BINFILES:%=src/%) -t $(DESTDIR)/opt/unipi/bin
	if [ "$(ARCH)" = "arm" -o "$(DEB_TARGET_ARCH)" = "armhf" ]; then $(INSTALL) -D unipi-common/dts/*.dtbo -t $(DESTDIR)/boot/overlays ; fi

mr-proper:
	@make clean
	@rm -rf libmodbus
	@dh_clean
