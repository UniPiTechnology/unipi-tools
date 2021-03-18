
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <errno.h>
#include <unistd.h>

#include "armutil.h"
#include "fwimage.h"
#include "fwconfig.h"


int load_bin(Tboard_version* bv, T_image_header *header, void* prog_data, void* rw_data)
{
    FILE* fd;
	char* fname;
    int read_n;
	T_image_header *header6;
#ifdef OS_WIN32
    struct stat finfo;
    int i;
    off_t filesize;
#endif

	fname = firmware_name(bv, firmwaredir, ".bin");
	vprintf_1("Loading firmware bin: %s\n", fname);
	if ((fd = fopen(fname, "rb")) == NULL) {
		eprintf("Cannot open file %s\n", fname);
		goto err1;
	}
#ifdef OS_WIN32
    fstat(fd->_file, &finfo);
    off_t filesize = finfo.st_size;
#endif
    memset(prog_data, 0xff, MAX_FW_SIZE);
    read_n = fread(prog_data, 1, MAX_FW_SIZE, fd);
#ifdef OS_WIN32
    if (!read_n) {
    	for (int i = 0; i < filesize; i++) {
    		prog_data[i] = fgetc(fd);
    	}
    	read_n = filesize;
    }
    vprintf_2("READ: %d %x %d\n", read_n, prog_data[0], filesize);
#endif
	if (read_n <= 0) {
		eprintf("Cannot read firmware %s\n", fname);
		goto err;
	}
	header->firmware_length = read_n;
    fclose(fd);
	free(fname);

	fname = firmware_name(bv, firmwaredir, ".rw");
	vprintf_1("Loading firmware rw: %s\n", fname);
	if ((fd = fopen(fname, "rb")) == NULL) {
		eprintf("Cannot open file %s\n", fname);
		goto err1;
	}
#ifdef OS_WIN32
    fstat(fd->_file, &finfo);
    off_t filesize = finfo.st_size;
#endif
    memset(rw_data, 0xff, MAX_RW_SIZE);
    read_n = fread(rw_data, 1, MAX_RW_SIZE, fd);
#ifdef OS_WIN32
    if (!read_n) {
    	for (int i = 0; i < filesize; i++) {
    		rw_data[i] = fgetc(fd);
    	}
    	read_n = filesize;
    }
    vprintf_2("READ: %d %x %d\n", read_n, rw_data[0], filesize);
#endif
	if (read_n <= 0) {
		eprintf("Cannot read firmware %s\n", fname);
		goto err;
	}
	header->rwdata_length = read_n;
	header->rwdata_start = 0xe000;
    fclose(fd);
	free(fname);

	if (bv->sw_version == 0x500) {
		// downgrade to 5.x
		header6 = load_image_header(bv);
		if (header6) { 
			patch_first_page_downgrade(header6, prog_data);
			free(header6);
		}
	}
	return 0;
err:
    fclose(fd);
err1:
	free(fname);
	return -1;
}


/*******************
    - file firmware version is written in last four bytes in .rw file
*/
uint16_t get_bin_version(Tboard_version* bv)
{
	FILE* fd;
	char* fwname;
	uint32_t sw_version = 0;

	fwname = firmware_name(bv, firmwaredir, ".rw");

 	if ((fd = fopen(fwname, "rb"))!=NULL) {
    	// old firmware has version in .rw file 
		if (fseek(fd, -4, SEEK_END) >= 0) {
			if (fread(&sw_version, 1, 4, fd) == 4) {
				if (sw_version & 0xff000000) sw_version = sw_version >> 16;
			}
	    }
        fclose(fd);
	}
	free(fwname);
	return (sw_version & 0xffff);
}

