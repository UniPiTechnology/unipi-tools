/**********************
 *
 * Programming utility via ModBus
 *
 * Michal Petrilak 2016
 * Miroslav Ondra  2017
 *
 **********************/

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <getopt.h>
#include <errno.h>
#include <modbus/modbus.h>
#include <unistd.h>

#include "armutil.h"
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include "unipiutil.h"
#include "fwimage.h"
#include "fwopt.h"
#include "fwdriver.h"

const char* version_string = "Version " PROJECT_VER;
#ifdef FWSERIAL
const char* program_name = "fwserial";
#endif
#ifdef FWSPI
const char* program_name = "fwspi";
#endif

#define MAX_PAGES 64

int upgrade_bootloader(Tboard_version *bv, void* channel)
{
	T_image_header header;
	char* fwname;
	int ret;
	uint8_t *prog_data = malloc(MAX_FW_SIZE);
	uint8_t *bootloader = malloc(MAX_BL_SIZE);
	uint8_t *rw_data = NULL;
	struct page_description *pd_array = calloc(sizeof(struct page_description), MAX_PAGES);

	// force 6.00 version
	bv->sw_version = (uint16_t)0x0600;
	fwname = firmware_name(bv, firmwaredir, ".img");
	ret = load_image(fwname, &header, prog_data, bootloader, rw_data);
	free(fwname);
	if (ret != 0) goto err;

	printf("Upgrading  bootloader...\n");
	ret = -1;
	if (driver.start(channel) != 0) goto err;

	// write first page + booloader
	patch_first_page(&header, prog_data);

	// prepare page description array
	pd_array[0].flash_addr = 0;
	pd_array[0].data = prog_data;

	int n=1;
	int offset = 0;
	while (offset < header.bootloader_length) {
		pd_array[n].flash_addr = header.bootloader_start+offset;
		pd_array[n].data = bootloader + offset;
        offset += PAGE_SIZE;
        n++;
	}

	// write bootloader
	vprintf_1("Sending %d pages.\n", n);
	if (driver.flash(channel, pd_array, n, TRUE) != 0) goto err;

	// reboot
	driver.run(channel);
	if (!verbose) printf("\n");
	printf("Reboot board...\n");
	usleep(200000);

	if ((bv = driver.identify(channel)) == NULL)
		goto err;

	if (SW_MAJOR(bv->sw_version) < 6) {
		eprintf("Unsuccessful upgrade: %s.\n", modbus_strerror(errno));
		goto err;
	}
	ret = 0;
err:
	free(prog_data);
	if (bootloader) free(bootloader);
	if (rw_data) free(rw_data);
	free(pd_array);
	return ret;
}

int upload_firmware(Tboard_version *bv, void* channel, int do_verify, int do_resetrw)
{
	T_image_header header;
	char* fwname;
	int ret;
	int n, offset;
	uint8_t *prog_data = malloc(MAX_FW_SIZE);
	uint8_t *bootloader = NULL;
	uint8_t *rw_data = malloc(MAX_RW_SIZE);
	struct page_description *pd_array = calloc(sizeof(struct page_description),MAX_PAGES);

	if (SW_MAJOR(bv->sw_version) < 6) {
		ret = load_bin(bv, &header, prog_data, rw_data);
	} else {
		fwname = firmware_name(bv, firmwaredir, ".img");
		ret = load_image(fwname, &header, prog_data, bootloader, rw_data);
		free(fwname);
	}
	if (ret != 0) goto err;

	if (driver.start(channel) != 0) goto err;

	// prepare page description array
	n=0;
	offset = 0;
	while (offset < (header.firmware_length)) {
		pd_array[n].flash_addr = offset;
		pd_array[n].data = prog_data + offset;
        offset += PAGE_SIZE;
        n++;
	}
	if (do_resetrw) {
		// write rw data
		offset = 0;
		while (offset < header.rwdata_length) {
			pd_array[n].flash_addr = header.rwdata_start+offset;
			pd_array[n].data = rw_data + offset;
	        offset += PAGE_SIZE;
    	    n++;
		}
	}
	vprintf_1("Sending %d pages.\n", n);
	if (driver.flash(channel, pd_array, n, do_verify) != 0) goto err;

	if (!verbose) printf("\n");
	// try to run new firmware
	driver.run(channel);
	usleep(200000);
	// confirm firmware
	if ((driver.confirm(channel)!=0) && do_resetrw) {
        com_options.BAUD=19200;  com_options.parity='N', com_options.stopbit=1;
	    driver.reopen(channel, &com_options);
	    driver.confirm(channel);
    }
	// reboot
	driver.reboot(channel);
	usleep(200000);
	if ((bv = driver.identify(channel)) == NULL)
		goto err;
	ret = 0;
err:
	free(prog_data);
	if (bootloader) free(bootloader);
	if (rw_data) free(rw_data);
	free(pd_array);
	return ret;
}

int auto_update(void)
{
    void *channel;
    Tboard_version *bv;
	uint16_t image_version;
	int verbose0;
    int device_index = com_options.DEVICE_ID;
	int max_device_index = com_options.DEVICE_ID;

    if (device_index == -1) {
        device_index = 0;
        max_device_index = 2;
    }

    for (;device_index <= max_device_index; device_index++) {
		com_options.DEVICE_ID = device_index;
		verbose0 = verbose;
		verbose = -1;
		channel=driver.open(&com_options);
		verbose = verbose0;
		if (channel != NULL) {
			if ((bv = driver.identify(channel)) != NULL){
				if (SW_MAJOR(bv->sw_version) < 6) 
					image_version = get_bin_version(bv);
				else 
					image_version = get_image_version(bv);
				if (bv->sw_version < image_version) {
					printf("New firmware %d.%d for device id=%d fw=%d.%d\n", SW_MAJOR(image_version), SW_MINOR(image_version),\
							device_index, SW_MAJOR(bv->sw_version), SW_MINOR(bv->sw_version));
					upload_firmware(bv, channel, 0, 0);
				}
			}
			driver.close(channel);
		}
	}
	exit(0);
	return 0;
}


int main(int argc, char **argv)
{
    void *channel;
    Tboard_version *bv;
    T_image_header *header = NULL;

    // Parse command line options
    if (parseopt(argc, argv)!= 0) exit(EXIT_FAILURE);

    if (verbose) printf("%s: %s\n", program_name, version_string);

    if (do_auto) {
        auto_update();
    }

    // Open port
    channel=driver.open(&com_options);
    if (channel == NULL) 
        exit(EXIT_FAILURE);

    // get FW & HW version
    if ((bv = driver.identify(channel)) == NULL) goto err;

    printf("Boardset:   %3d %-30s (v%d.%d%s)\n",
               HW_BOARD(bv->hw_version),  arm_name(bv->hw_version),
               HW_MAJOR(bv->hw_version), HW_MINOR(bv->hw_version),
               IS_CALIB(bv->hw_version)?" CAL":"");
    printf("Baseboard:  %3d %-30s (v%d.%d)\n",
               HW_BOARD(bv->base_hw_version),  arm_name(bv->base_hw_version),
               HW_MAJOR(bv->base_hw_version), HW_MINOR(bv->base_hw_version));
    if (SW_MINOR(bv->sw_version)!=0) {
        printf("Firmware: v%d.%d\n", SW_MAJOR(bv->sw_version), SW_MINOR(bv->sw_version));
    } else {
        printf("WARNING! Bootloader only (v%d.0). UPDATE FIRMWARE!\n", SW_MAJOR(bv->sw_version));
    }
    header=load_image_header(bv);
    if (header && (SW_MAJOR(bv->sw_version) < 6)) {
        if (!do_upgrade)
            printf("PLEASE UPGRADE FIRMWARE TO %d.%d - to proceed, execute %s -P -U\n", 
                                SW_MAJOR(header->swversion), SW_MINOR(header->swversion), program_name);
    } else {
        if (do_upgrade) {
            eprintf("Cannot do upgrade. Try normal update.\n");
            do_upgrade = 0;
        }
    }

    if (do_upgrade) {
#ifdef FWSERIAL
        if (setup_boot_context(com_options.DEVICE_ID, com_options.BAUD, com_options.parity, com_options.stopbit) != 0) {
            goto err;
        }
#endif
        if (upgrade_bootloader(bv, channel) != 0) {
            goto err;
        }
    } else if (do_downgrade) {
#ifdef FWSERIAL
        if (setup_boot_context(com_options.DEVICE_ID, com_options.BAUD, com_options.parity, com_options.stopbit) != 0) {
            goto err;
        }
#endif
        bv->sw_version = 0x500;
    }

    if (do_prog) {
        // FW manipulation
        if (do_calibrate) {
            bv->hw_version = bv->base_hw_version | 0x8;
            do_resetrw = 1;
        } else if (do_final) {
            if (!IS_CALIB(bv->hw_version)) {
                eprintf("Only calibrating version can be reprogrammed to final\n");
                goto err;
            }
            bv->hw_version = check_compatibility(bv->base_hw_version, upboard);
            if (bv->hw_version == 0) {
                eprintf("Incompatible base and upper boards. Use one of:\n");
                print_upboards(bv->base_hw_version);
                goto err;
            }
            do_resetrw = 1;
        }
        upload_firmware(bv, channel, do_verify, do_resetrw);
    }
    driver.close(channel);
    return 0;
err:
    driver.close(channel);
    return 1;
}
