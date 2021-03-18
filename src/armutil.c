
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>

#include "armutil.h"
#include "fwimage.h"

int verbose = 0;

typedef struct {
  uint8_t board;
  uint8_t baseboard;
  uint8_t upboard;
  const char*   name;
} Tcompatibility_map;

#define UP_COUNT 9
Tboards_map up_boards[] = {
    {0, 0, ""},
    {1, 16, "P-11DiR485-1"},
    {2, 16, "U-14Ro-1"},
    {3, 16, "U-14Di-1"},
    {4, 16, "P-6Di5Ro-1"},
    {5, 16, "U-6Di5Ro-1"},
    {6, 16, "P-R485Di4Ro5-1"},
    {7, 16, "U-R485Di4Ro5-1"},
    {13,16, "B-485-1"},
};

Textension_map extension_boards[] = {
	{1, 4, "xS10-CAL"},
	{2, 5, "xS40-CAL"},
	{3, 6, "xS30-CAL"},
	{4, 4, "xS10"},
	{5, 5, "xS40"},
	{6, 6, "xS30"},
	{11, 12, "xS50-CAL"},
	{12, 12, "xS50"},
	{16, 16, "X-1Ir"},
	{17, 17, "MM-8OW"},
	{21, 21, "MM-8PT"}
};

Textension_map* get_extension_map(int board) {
    int i;
    for (i=0; i<EXTENSION_COUNT; i++) {
            if (extension_boards[i].board == board) {
                return extension_boards + i;
            }
    }
    return NULL;
}

Tboards_map* get_umap(int board)
{
    int i;
    for (i=0; i<UP_COUNT; i++) {
            if (up_boards[i].board == board) {
                return up_boards + i;
            }
    }
    return NULL;
}

#define HW_COUNT 22
Tcompatibility_map compatibility_map[HW_COUNT] = {
    {0,  0, 0, "B-1000",},
    {1,  1, 0, "E-8Di8Ro",},
    {2,  2, 0, "E-14Ro",},
    {3,  3, 0, "E-16Di",},
    {4,  1, 1, "E-8Di8Ro_P-11DiR485", },        //"E-8Di8Ro_P-11DiR485"
    {5,  2, 1, "E-14Ro_P-11DiR485",},         //"E-14Ro_P-11DiR485"
    {6,  3, 1, "E-16Di_P-11DiR485",},         // "E-16Di_P-11DiR485"
    {7,  2, 2, "E-14Ro_U-14Ro",},         //"E-14Ro_U-14Ro"
    {8,  3, 2, "E-16Di_U-14Ro",},         //"E-16Di_U-14Ro"
    {9,  2, 3, "E-14Ro_U-14Di",},         //"E-14Ro_U-14Di"
    {10, 3, 3, "E-16Di_U-14Di",},         //"E-16Di_U-14Di"
    {11, 11,0, "E-4Ai4Ao"},
    {12, 11,4, "E-4Ai4Ao_P-6Di5Ro",},         //"E-4Ai4Ao_P-6Di5Ro"},
    {13, 0,13, "B-485"},
    {14, 14,0, "E-4Light"},
    {15, 11,5, "E-4Ai4Ao_U-6Di5Ro"},
    {16, 16,0, "X-1Ir"},
    {17, 17,0, "MM-8OW"},
    {18, 11,6, "E-4Ai4Ao_P-R485Di4Ro5",},         //"E-4Ai4Ao_P-4Di5Ro"},
    {19, 11,7, "E-4Ai4Ao_U-R485Di4Ro5"},
    {20,  1,6, "E-8Di8Ro_P-R485Di4Ro5", },     
    {21, 21,0, "MM-8PT"},
};

Tcompatibility_map* get_map(int board)
{
    //uint8_t board = hw_version >> 8;
    int i;
    for (i=0; i<HW_COUNT; i++) {
        if (compatibility_map[i].board == board) {
            return compatibility_map + i;
        }
    }
        /*if (board < HW_COUNT) {
            return hwnames[board];
        }*/
    return NULL;
}


const char* arm_name(uint16_t hw_version)
{
    Tcompatibility_map* map = get_map(HW_BOARD(hw_version));
    if (map == NULL)
        return "UNKNOWN BOARD";
    return map->name;
}

char* _firmware_name(Tboard_version* bv, const char* fwdir, const char* ext, int use_base_revision)
{
    uint8_t calibrate = IS_CALIB(bv->hw_version);
    uint8_t board_revision = HW_MAJOR(bv->hw_version);
    uint8_t used_board_revision = use_base_revision ? HW_MAJOR(bv->base_hw_version) : board_revision;
    Tcompatibility_map* map = get_map(HW_BOARD(bv->hw_version));
    if (map  == NULL) return NULL;
    
    if (SW_MAJOR(bv->sw_version) <= 5)
    {
        if (map->baseboard == map->board) {
            const char* armname = map->name;
            char* fwname = malloc(strlen(fwdir) + strlen(armname) + strlen(ext) + 2 + 4);
            strcpy(fwname, fwdir);
            if (strlen(fwname) && (fwname[strlen(fwname)-1] != '/')) strcat(fwname, "/");
            sprintf(fwname+strlen(fwname), "%s-%d%s%s", armname, used_board_revision, calibrate?"C":"", ext);
            return fwname;

        } else {
            Tcompatibility_map* basemap = get_map(HW_BOARD(bv->base_hw_version));
            if (basemap == NULL) return NULL;
            //uint8_t base_version = HW_MAJOR(hw_base);
            if (basemap->board != map->baseboard) {
                // Incorrent parameters
                return NULL;
            }
            const char* basename = basemap->name;
            Tboards_map* umap = get_umap(map->upboard);
            const char* uname = umap->name;
            char* fwname = malloc(strlen(fwdir) + strlen(basename) + strlen(uname) + strlen(ext) + 2 + 4 + +1 + 4);
            strcpy(fwname, fwdir);
            if (strlen(fwname) && (fwname[strlen(fwname)-1] != '/')) strcat(fwname, "/");
            sprintf(fwname+strlen(fwname), "%s-%d_%s%s%s", basename, used_board_revision, uname, calibrate?"C":"", ext);
            return fwname;
        }
    }
    else
    {
        char* fwname = malloc(strlen(fwdir) + strlen(ext) + 7);
        strcpy(fwname, fwdir);
        if (strlen(fwname) && (fwname[strlen(fwname)-1] != '/')) strcat(fwname, "/");
        sprintf(fwname+strlen(fwname), "%02d-%d%s%s", map->board, used_board_revision, calibrate?"C":"", ".img");
        return fwname;
    }
}


char* firmware_name(Tboard_version* bv, const char* fwdir, const char* ext)
{
	char * fname = _firmware_name(bv, fwdir, ext, 1);
    FILE* fd = fopen(fname, "r");
    if (fd != NULL) {
        fclose(fd);
        return fname;
    }
    free(fname);
	return _firmware_name(bv, fwdir, ext, 0);
}

int check_compatibility(int hw_base, int upboard)
{
    uint8_t board = hw_base >> 8;
    int i;
    for (i=0; i<HW_COUNT; i++) {
        if ((compatibility_map[i].baseboard == board) && (compatibility_map[i].upboard == upboard)) {
            Tboards_map* umap = get_umap(upboard);
            if (umap->subver == 0) {
                return (compatibility_map[i].board << 8) | (hw_base & 0xff);
            } else {
                return (compatibility_map[i].board << 8) | (umap->subver & 0xff);
            }
        }
    }
    return 0;
}

int get_board_speed(Tboard_version* bv)
{
    // E-4Ai4Ao* - used Digital Isolator on SPI - speed max 8MHz
    if (HW_BOARD(bv->base_hw_version) == 11) return 8000000;
    // Default speed 12MHz
    return 12000000;
}

void print_upboards(int filter)
{
    int i;
    for (i=0; i<UP_COUNT; i++) {
        if ((filter == -1) || (check_compatibility(filter,up_boards[i].board)))
            printf("%3d - %s\n", up_boards[i].board, up_boards[i].name);
    }
}

int upboard_exists(int board) 
{
    return get_umap(board) != NULL;
}

int parse_version(Tboard_version* bv, uint16_t *r1000)
{
    bv->sw_version = r1000[0];
    bv->hw_version = r1000[3];
    bv->base_hw_version = r1000[4];

    bv->di_count    = (r1000[1]       ) >> 8;
    bv->do_count   = (r1000[1] & 0xff);
    bv->ai_count   = (r1000[2]       ) >> 8;
    bv->ao_count   = (r1000[2] & 0xff) >> 4;
    bv->uart_count = (r1000[2] & 0x0f);
    bv->uled_count = 0;
    bv->int_mask_register = 1007;
    if (SW_MAJOR(bv->sw_version) < 4) {
        bv->hw_version = (SW_MINOR(bv->sw_version) & 0xff) << 4 \
                       | (SW_MINOR(bv->sw_version) & 0x0f);
        bv->sw_version = bv->sw_version & 0xff00;
        bv->int_mask_register = 1003;
    } else {
        if ((bv->sw_version < 0x0403)) {  // devel version
           bv->int_mask_register = 1004;
        }
        if (HW_BOARD(bv->hw_version) == 0) {
            if (bv->sw_version != 0x0400)
                bv->uled_count = 4;
        }
    }
    if ((HW_BOARD(bv->base_hw_version) == 0x0b) && (HW_MAJOR(bv->base_hw_version) <= 1)) bv->int_mask_register = 0;   // 4Ai4Ao has not interrupt
    return 0;
}

/*******************
 function checks if firmware in file is newer then firmware in Tboard_version
    - file firmware version is written in last four bytes in .rw file
    - return 0 or file firmware version
*/
/*
uint32_t check_new_rw_version(Tboard_version* bv, const char* fwdir)
{
    FILE* fd;
    char* fwname;
    T_image_header header;
    uint32_t fwver;
    uint32_t ret = 0;

    fwname = firmware_name(bv, fwdir, ".rw");

    if ((fd = fopen(fwname, "rb"))!=NULL) {
	if (SW_MAJOR(bv->sw_version) <= 5) {
	    // old firmware has version in .rw file 
	    if (fseek(fd, -4, SEEK_END) >= 0) {
        	if (fread(&fwver, 1, 4, fd) == 4) {
    	    	    if (fwver & 0xff000000) fwver = fwver >> 16;
            	    if (fwver > bv->sw_version) ret = fwver;
		}
	    }
	} else {
	    // 6.xx has version in image header
	    if (fread(&header, 1, sizeof(header), fd) == sizeof(header)) {
        	if (header.swversion > bv->sw_version) ret = header.swversion;
    	    } 
	}
        fclose(fd);
    }
    free(fwname);
    return ret;
}
*/

/*******************
 function load firmware file into new allocated memory
    - rw (bool) if rw==0 load .bin file else load .rw file
    - return pointer to new buffer containing data from file
    - in case of error returns NULL
    - in datalen is returned length of data
*/
/*
uint8_t* load_fw_file(Tboard_version* bv, const char* fwdir, int rw, int* datalen)
{
    FILE* fd;
    char* fwname;
    //int red, i;
    uint8_t* data;
    size_t maxdatalen;

    fwname = firmware_name(bv, fwdir, rw ? ".rw" : ".bin" );

    fd = fopen(fwname, "rb");
    if (!fd) {
        printf("LOAD_FW: Error opening firmware file \"%s\"\n", fwname);
        free(fwname);
        return NULL;
    }
    maxdatalen = rw ? MAX_RW_SIZE : MAX_FW_SIZE;
    data = malloc(maxdatalen);
    memset(data, 0xff, maxdatalen);

    *datalen = fread(data, 1, maxdatalen, fd);
    //printf("Bytes 58: %d,59: %d,60: %d,61: %d,62: %d,63: %d,64: %d\n", prog_data[58], prog_data[59], prog_data[60], prog_data[61], prog_data[62], prog_data[63]);

    if (*datalen < (rw ? MIN_RW_SIZE : MIN_FW_SIZE)) {
        if (*datalen < 0) {
            printf("LOAD_FW: Error reading firmware file \"%s\"\n", fwname);
        } else {
            printf("LOAD_FW: Firmware file \"%s\" is short(%dB)\n", fwname, *datalen);
        }
        free(data);
        data = NULL;
    }
    free(fwname);
    fclose(fd);
    return data;
}

*/
