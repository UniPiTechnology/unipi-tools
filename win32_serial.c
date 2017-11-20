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
#include <modbus.h>
#include <unistd.h>
#include <gtk/gtk.h>

#include "armutil.h"
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>


/* Hardware constants */
#define PAGE_SIZE   1024
#define REG_SIZE    64

#define MAX_FW_SIZE (64*PAGE_SIZE)
#define MAX_RW_SIZE (PAGE_SIZE)
#define RW_START_PAGE ((0xE000) / PAGE_SIZE)

/* Default parameters */
char* active_port = NULL;
int   active_baud = 19200;
const int   DEVICE_ID = 15;
#ifdef OS_WIN32
char* firmwaredir = "./fw";
#else
char* firmwaredir = "/opt/fw";
#endif
int upboard;
int verbose = 0;
int do_verify = 0;
int do_prog   = 1;
int do_resetrw= 0;
int do_calibrate= 0;
int do_final= 0;



GtkApplication *app;

GtkWidget *dialog;
GtkWidget *dialog_content_area;
GtkWidget *dialog_label;

//GtkWidget *dialog_ok_button;
//GtkWidget *dialog_button_hbox;
//GtkWidget *dialog_button_vbox;

GtkWidget *button_alignment;
GtkWidget *flash_button;
GtkWidget *button_hbox;

GtkWidget *main_window;
GtkWidget *window_vbox;

GtkWidget *dialog_window;
GtkWidget *dialog_window_vbox;


GtkWidget *label_hbox;
GtkWidget *output_label;

GtkWidget *port_label;
GtkWidget *port_combo_box;
GtkWidget *baud_combo_box;
GtkWidget *verify_checkbox;
GtkWidget *verify_label;
GtkWidget *settings_hbox;

uint8_t *prog_data;   // buffer containing firmware
uint8_t* rw_data;     // buffer containing firmware rw data
uint16_t* pd;
int ret, chunk, page;
uint16_t val, reg;
modbus_t *ctx;
FILE* fdx;

char box_entries[255][64];
char output_string[1024];

int verify(modbus_t *ctx, uint8_t* prog_data, uint8_t* rw_data, int last_prog_page, int last_page, int print_errors)
{
    uint16_t* pd;
    int ret, chunk, page;
    uint16_t val, reg;

	modbus_set_response_timeout(ctx, 2, 999999);
	pd = (uint16_t*) prog_data;
	printf("!!! WARNING: OPERATION IN PROGRESS, DO NOT EXIT THIS WINDOW UNTIL IT IS FINISHED !!!\n");
	printf("\tTO RECOVER FROM PREMATURE TERMINATION POWER-CYCLE YOUR TARGET DEVICE\n");
	printf("Pages to verify: %.2d\n", last_page);
	for (page=0; page < last_page; page++) {
		if (page > 0) printf("DO NOT EXIT THIS WINDOW UNTIL THE OPERATION IS FINISHED !!!\n");
		if (page > 0) printf("TO RECOVER POWER-CYCLE YOUR TARGET DEVICE\n");
		printf("Verifying page %.2d out of %.2d ...", page + 1, last_page);
		fflush(stdout);
		if (modbus_write_register(ctx, 0x7705, page) != 1) {   // set page address in Neuron
			fprintf(stderr, "Verifying failed: %s\n", modbus_strerror(errno));
			break;
		}
		for (chunk=0; chunk < 8; chunk++) {
			if (modbus_write_registers(ctx, 0x7700+chunk, REG_SIZE, pd) != REG_SIZE) {; // send chunk of data
				fprintf(stderr, "Sending data failed: %s\n", modbus_strerror(errno));
			}
			pd += REG_SIZE;
		}
		if (modbus_read_registers(ctx, 0x7707, 1, &val) == 1) {
			if (val == 0x100) {
				if (print_errors) printf(" OK\n");
			} else {
				if (page == 1) {
					if (print_errors) printf("OK\n");
				} else {
					if (print_errors) printf(" NOT OK. errors = %d.\n", 0x100-val);
				}
			}
		} else {
			snprintf(output_string, sizeof(output_string), "Verification Failed!!: %s\n", modbus_strerror(errno));
			gtk_label_set_text(GTK_LABEL(output_label), output_string);
			return -1;
		}
		if (page == last_prog_page-1) {
			page = RW_START_PAGE-1;
			pd = (uint16_t*) rw_data;
		}
	}
	snprintf(output_string, sizeof(output_string), "Verification Passed!\n");
	gtk_label_set_text(GTK_LABEL(output_label), output_string);
}

int flashit(modbus_t *ctx, uint8_t* prog_data, uint8_t* rw_data, int last_prog_page, int last_page)
{
	uint16_t* pd;
	int ret, chunk, page;
	// Programming
	modbus_set_response_timeout(ctx, 1, 0);
	page = 0;
	int errors = 0;
	printf("!!! WARNING: OPERATION IN PROGRESS, DO NOT EXIT THIS WINDOW UNTIL IT IS FINISHED !!!\n");
	printf("\tTO RECOVER FROM PREMATURE TERMINATION POWER-CYCLE YOUR TARGET DEVICE\n");
	printf("Pages to write: %.2d\n", last_prog_page + 1);
	while (page < last_page) {
		if (page > 0) printf("DO NOT EXIT THIS WINDOW UNTIL THE OPERATION IS FINISHED !!!\n");
		if (page > 0) printf("TO RECOVER POWER-CYCLE YOUR TARGET DEVICE\n");
		if (page <= last_prog_page) {
			printf("Programming page %.2d out of %.2d ...\n", page + 1, last_prog_page + 2);
		} else {
			printf("Programming page %.2d out of %.2d ...\n", last_prog_page + 2, last_prog_page + 2);
		}
		fflush(stdout);
		if (page < last_prog_page + 1) {
			pd = (uint16_t*) (prog_data + page*PAGE_SIZE);
		} else {
			pd = (uint16_t*) (rw_data + ((page-RW_START_PAGE)*PAGE_SIZE));
		}
		if (modbus_write_register(ctx, 0x7705, page) == 1) {   // set page address in Neuron
			for (chunk=0; chunk < 8; chunk++) {
				int retval = modbus_write_registers(ctx, 0x7700+chunk, REG_SIZE, pd);
				if (retval == -1) { // send chunk of data (64*2 B)
					errors++;
				}
				fprintf(stderr, "Written Chunk %d, R:%d E:%d\n", chunk, retval, errors);
				pd += REG_SIZE;
			}
			if (modbus_write_register(ctx, 0x7707, 1) == 1) {  // write page to flash
				printf("Page Written OK\n");
				if (page == last_prog_page) {
					page = RW_START_PAGE;
				} else {
					page++;
				}

			} else {
				errors++;
				printf(" Trying again\n");
				fprintf(stderr, "Flashing page failed: %s\n", modbus_strerror(errno));
			}
		} else {
			errors++;
			printf(" Trying again\n");
		}
		if (errors > 200) {
			snprintf(output_string, sizeof(output_string), "Flashing Not Successful!! Power-cycle your target device");
			gtk_label_set_text(GTK_LABEL(output_label), output_string);
			return -1;
		}
	}
	snprintf(output_string, sizeof(output_string), "Flashing Successful!! Power-cycle your target device");
	gtk_label_set_text(GTK_LABEL(output_label), output_string);
}

int load_fw(char *path, uint8_t* prog_data, const size_t len)
{
    FILE* fd;
    int read_n, i;
    fd = fopen(path, "rb");
    struct stat finfo;
    fstat(fd->_file, &finfo);
    off_t filesize = finfo.st_size;
    if (!fd) {
        printf("error opening firmware file \"%s\"\n", path);
        return -1;
    }
    memset(prog_data, 0xff, len);

    read_n = fread(prog_data, 1, MAX_FW_SIZE, fd);
    if (!read_n) {
    	for (int i = 0; i < filesize; i++) {
    		prog_data[i] = fgetc(fd);
    	}
    	read_n = filesize;
    }
    printf("READ: %d %x %d\n", read_n, prog_data[0], filesize);
    fclose(fd);
    return read_n;
}

void close_window(GtkWindow *caller) {
	exit(0);
}

void select_port(GtkComboBox *caller) {
/*    if (PORT == NULL) {
        printf("Port device must be specified\n", optarg);
        print_usage(argv[0]);
        exit(EXIT_FAILURE);
    } */
    // Open port
	gint i = gtk_combo_box_get_active(caller);
	//fprintf(stderr, "Unable to create the libmodbus context%s\n", i);
	//strcpy(active_port, box_entries[0]);
    active_port = box_entries[i];
	ctx = modbus_new_rtu(active_port, active_baud, 'N', 8, 1);
    modbus_set_response_timeout(ctx, 0, 800000);
    if (ctx == NULL) {
        gtk_label_set_text(GTK_LABEL(output_label), "ERROR: Unable to create modbus context (wrong port?)");
    	//fprintf(stderr, "Unable to create the libmodbus context\n");
        return;
    }
    if ( verbose > 1) modbus_set_debug(ctx,verbose-1);
    modbus_set_slave(ctx, DEVICE_ID);

    if (modbus_connect(ctx) == -1) {
        //fprintf(stderr, "Connection failed: %s\n", modbus_strerror(errno));
    	gtk_label_set_text(GTK_LABEL(output_label), "ERROR: Connection failed (wrong port?)");
    	modbus_close(ctx);
    	modbus_free(ctx);
        return;
    }

    // get FW & HW version
    uint16_t r1000[7];
    Tboard_version bv;
    //int hw_version, sw_version, base_version;
    if (modbus_read_registers(ctx, 1000, 7, r1000) == 7) {
        parse_version(&bv, r1000);
        const char * product_name;
        Textension_map* product_name_p = get_extension_map(HW_BOARD(bv.hw_version));
        if (product_name_p == NULL) {
        	product_name = product_name_p->product;
        }

        snprintf(output_string, sizeof(output_string), "Board:  %s\nE-Board S/N:  (%d)\nBoardset:   %3d %-30s (v%d.%d%s)\nBaseboard:  %3d %-30s (v%d.%d)\nFirmware: v%d.%d\n",
        		get_extension_map(HW_BOARD(bv.hw_version))->product, (r1000[6] << 16) + r1000[5],
        		HW_BOARD(bv.hw_version),  arm_name(bv.hw_version),
               HW_MAJOR(bv.hw_version), HW_MINOR(bv.hw_version),
               IS_CALIB(bv.hw_version)?" CAL":"",
               HW_BOARD(bv.base_hw_version),  arm_name(bv.base_hw_version),
               HW_MAJOR(bv.base_hw_version), HW_MINOR(bv.base_hw_version),
			   SW_MAJOR(bv.sw_version), SW_MINOR(bv.sw_version));
        gtk_label_set_text(GTK_LABEL(output_label), output_string);
    } else {
        snprintf(output_string, sizeof(output_string), "Read version failed: %s\n", modbus_strerror(errno));
        gtk_label_set_text(GTK_LABEL(output_label), output_string);
    }
    modbus_close(ctx);
    modbus_free(ctx);
}

void select_baud_rate(GtkComboBox *caller) {
    // Open port
	gint i = gtk_combo_box_get_active(caller);
	//fprintf(stderr, "Unable to create the libmodbus context%s\n", i);
	//strcpy(active_port, box_entries[0]);
    if (i) {
    	active_baud = 9600;
    } else {
    	active_baud = 19200;
    }
	ctx = modbus_new_rtu(active_port, active_baud, 'N', 8, 1);
    modbus_set_response_timeout(ctx, 0, 800000);
    if (ctx == NULL) {
        gtk_label_set_text(GTK_LABEL(output_label), "ERROR: Unable to create modbus context (wrong port?)");
    	//fprintf(stderr, "Unable to create the libmodbus context\n");
        return;
    }
    if ( verbose > 1) modbus_set_debug(ctx,verbose-1);
    modbus_set_slave(ctx, DEVICE_ID);

    if (modbus_connect(ctx) == -1) {
        //fprintf(stderr, "Connection failed: %s\n", modbus_strerror(errno));
    	gtk_label_set_text(GTK_LABEL(output_label), "ERROR: Connection failed (wrong port?)");
    	modbus_close(ctx);
    	modbus_free(ctx);
        return;
    }

    // get FW & HW version
    uint16_t r1000[7];
    Tboard_version bv;
    //int hw_version, sw_version, base_version;
    if (modbus_read_registers(ctx, 1000, 7, r1000) == 57) {
        parse_version(&bv, r1000);
        snprintf(output_string, sizeof(output_string), "Board:  %s\nE-Board S/N:  (%d)\nBoardset:  %3d %-30s (v%d.%d%s)\nBaseboard:  %3d %-30s (v%d.%d)\nFirmware:  v%d.%d\n",
        		get_extension_map(HW_BOARD(bv.hw_version))->product, (r1000[6] << 16) + r1000[5],
        	   HW_BOARD(bv.hw_version),  arm_name(bv.hw_version),
               HW_MAJOR(bv.hw_version), HW_MINOR(bv.hw_version),
               IS_CALIB(bv.hw_version)?" CAL":"",
               HW_BOARD(bv.base_hw_version),  arm_name(bv.base_hw_version),
               HW_MAJOR(bv.base_hw_version), HW_MINOR(bv.base_hw_version),
			   SW_MAJOR(bv.sw_version), SW_MINOR(bv.sw_version));
        gtk_label_set_text(GTK_LABEL(output_label), output_string);
    } else {
        snprintf(output_string, sizeof(output_string), "Read version failed: %s\n", modbus_strerror(errno));
        gtk_label_set_text(GTK_LABEL(output_label), output_string);
    }
    modbus_close(ctx);
    modbus_free(ctx);
}

void switch_mode_verify(GtkToggleButton *caller) {
	gboolean verification_set = gtk_toggle_button_get_active(caller);
	if (verification_set == TRUE) {
		do_verify = 1;
		do_prog = 0;
		gtk_button_set_label(GTK_BUTTON(flash_button), "Verify");
	} else {
		do_verify = 0;
		do_prog = 1;
		gtk_button_set_label(GTK_BUTTON(flash_button), "Flash");
	}
}

void flash_button_pressed(GtkButton *caller) {
	do_resetrw = 1;
	// FW manipulation
	ctx = modbus_new_rtu(active_port, active_baud, 'N', 8, 1);
    modbus_set_response_timeout(ctx, 0, 800000);
    if (ctx == NULL) {
        gtk_label_set_text(GTK_LABEL(output_label), "ERROR: Unable to create modbus context (wrong port?)");
    	//fprintf(stderr, "Unable to create the libmodbus context\n");
        return;
    }
    if ( verbose > 1) modbus_set_debug(ctx,verbose-1);
    modbus_set_slave(ctx, DEVICE_ID);

    if (modbus_connect(ctx) == -1) {
        //fprintf(stderr, "Connection failed: %s\n", modbus_strerror(errno));
    	gtk_label_set_text(GTK_LABEL(output_label), "ERROR: Connection failed (wrong port?)");
    	modbus_close(ctx);
    	modbus_free(ctx);
        return;
    }

    // get FW & HW version
    uint16_t r1000[5];
    Tboard_version bv;
    //int hw_version, sw_version, base_version;
    if (modbus_read_registers(ctx, 1000, 5, r1000) == 5) {
        parse_version(&bv, r1000);
        printf("Board:  %s\nE-Board S/N:  (%d)\nBoardset:   %3d %-30s (v%d.%d%s)\nBaseboard:  %3d %-30s (v%d.%d)\nFirmware: v%d.%d\n",
        	   get_extension_map(HW_BOARD(bv.hw_version))->product, (r1000[6] << 16) + r1000[5],
			   HW_BOARD(bv.hw_version),  arm_name(bv.hw_version),
               HW_MAJOR(bv.hw_version), HW_MINOR(bv.hw_version),
               IS_CALIB(bv.hw_version)?" CAL":"",
               HW_BOARD(bv.base_hw_version),  arm_name(bv.base_hw_version),
               HW_MAJOR(bv.base_hw_version), HW_MINOR(bv.base_hw_version),
			   SW_MAJOR(bv.sw_version), SW_MINOR(bv.sw_version));
        //gtk_label_set_text(GTK_LABEL(output_label), output_string);
    } else {
        snprintf(output_string, sizeof(output_string), "Read version failed: %s\n", modbus_strerror(errno));
        gtk_label_set_text(GTK_LABEL(output_label), output_string);
    	modbus_close(ctx);
    	modbus_free(ctx);
    	return;
    }
    gtk_widget_hide(main_window);
	if (IS_CALIB(bv.hw_version)) {
		bv.hw_version = check_compatibility(bv.base_hw_version, upboard);
		if (bv.hw_version == 0) {
			gtk_label_set_text(GTK_LABEL(output_label), "Incompatible base and upper boards.\n");
			//print_upboards(bv.base_hw_version);
	    	modbus_close(ctx);
	    	modbus_free(ctx);
	    	gtk_widget_show_all(main_window);
			return;
		}
		uint16_t ver_temp = bv.hw_version;
		bv.hw_version &= 0x00FF;
		bv.hw_version |= ((get_extension_map(HW_BOARD(ver_temp))->ext_board) << 8);
	}

	// load firmware file
	char* fwname = firmware_name(bv.hw_version, bv.base_hw_version, firmwaredir, ".bin");
	prog_data = malloc(MAX_FW_SIZE);
	printf("Opening firmware file: %s\n", fwname);
	gtk_label_set_text(GTK_LABEL(output_label), output_string);
	int red = load_fw(fwname, prog_data, MAX_FW_SIZE);
	int rwred = RW_START_PAGE;
	free(fwname);
	if (red <= 0) {
		if (red == 0) {
			gtk_label_set_text(GTK_LABEL(output_label), "Firmware file is empty!\n");
		}
		free(prog_data);
    	modbus_close(ctx);
    	modbus_free(ctx);
    	gtk_widget_show_all(main_window);
		return;
	}
	red = (red + (PAGE_SIZE - 1)) / PAGE_SIZE;
	snprintf(output_string, sizeof(output_string), "Program pages: %d\n", red);
	gtk_label_set_text(GTK_LABEL(output_label), output_string);

	if (do_resetrw) {
		// load rw consts file
		rw_data = malloc(MAX_RW_SIZE);
		char* rwname = firmware_name(bv.hw_version, bv.base_hw_version, firmwaredir, ".rw");
		printf("Opening RW settings file: %s\n", rwname);
		int rwlen = load_fw(rwname, rw_data, MAX_RW_SIZE);
		free(rwname);
		//printf("arrrgh\n");
		// calc page count of firmware file
		rwred += ((rwlen + (PAGE_SIZE - 1)) / PAGE_SIZE);
		printf("Final page: %d, READ: %d, rwlen: %d\n", rwred, red, rwlen);
	}

	// init FW programmer
	if (modbus_write_bit(ctx, 1006, 1, 0) != 1) {
		snprintf(output_string, sizeof(output_string), "Program mode setting failed: %s\n", modbus_strerror(errno));
		gtk_label_set_text(GTK_LABEL(output_label), output_string);
    	modbus_close(ctx);
    	modbus_free(ctx);
    	gtk_widget_show_all(main_window);
		return;
	}
	verify(ctx,prog_data, rw_data, red, rwred, do_verify);
	if (do_prog) {
		flashit(ctx,prog_data, rw_data, red, rwred);
	}
	modbus_write_register(ctx, 0x7707, 3); // reboot
	free(prog_data);
	free(rw_data);
	modbus_close(ctx);
	modbus_free(ctx);
	gtk_widget_show_all(main_window);
}




static struct option long_options[] = {
  {"verbose", no_argument,      0, 'v'},
  {"verify", no_argument,       0, 'V'},
  {"programm", no_argument,     0, 'P'},
  {"resetrw", no_argument,      0, 'R'},
  {"calibrate", no_argument,    0, 'C'},
  {"final", required_argument,  0, 'F'},
  {"port",  no_argument,        0, 'p'},
  {"baud",  required_argument,  0, 'b'},
  {"unit",    required_argument,0, 'u'},
  {"dir", required_argument,    0, 'd'},
  {0, 0, 0, 0}
};

void print_usage(char *argv0)
{
    printf("\nUtility for Programming Neuron via ModBus RTU\n");
    printf("%s [-vVPRC] -p <port> [-u <mb address>] [-b <baudrate>] [-d <firmware dir>] [-F <upper board id>]\n", argv0);
    printf("\n");
    printf("--port <port>\t\t /dev/extcomm/1/0 or COM3\n");
    printf("--unit <mb address>\t default 15\n");
    printf("--baud <baudrate>\t default 19200\n");
    printf("--dir <firmware dir>\t default /opt/fw\n");
    printf("--verbose\t show more messages\n");
    printf("--verify\t compare flash with file\n");
    printf("--programm\t write firmware to flash\n");
    printf("--resetrw\t check/rewrite also rw settings\n");
    printf("--calibrate\t write calibrating firmware to flash\n");
    printf("--final <upper board id or ?>\t write final firmware over calibrating\n");
    printf("\n");
}


void setup_gui(int argc, char **argv) {
    gtk_init (&argc, &argv);

    main_window = gtk_window_new(GTK_WINDOW_TOPLEVEL);

    dialog_window = gtk_window_new(GTK_WINDOW_TOPLEVEL);
    //gtk_window_set_decorated(GTK_WINDOW(dialog_window), FALSE);
    //gtk_window_set_default_size(GTK_WINDOW(dialog_window), 400, 280);
    //gtk_window_set_resizable(GTK_WINDOW(dialog_window), FALSE);
    //gtk_widget_set_size_request(dialog_window, 400, 280);
    dialog = gtk_dialog_new_with_buttons("PLEASE READ - IMPORTANT INFORMATION", GTK_WINDOW(main_window), GTK_DIALOG_MODAL, "I Understand", GTK_RESPONSE_DELETE_EVENT, NULL);
    dialog_content_area = gtk_dialog_get_content_area (GTK_DIALOG (dialog));
    dialog_label = gtk_label_new("In order to correctly flash an updated firmware to your UniPi Neuron extension device, ensure the following:\n\n\t - Have a properly configured (1 stop bit, no parity, 19200/9600 baud) serial adapter device plugged into your computer\n\t - Turn on the termination resistors (pin 1) on your Neuron extension and use a twisted-pair cable to connect it to your PC adapter\n\t - Set your Neuron Serial device to the default address (address 15)\n\n Note also the following:\n\n\t - New firmware is only loaded into the Neuron upon restart (power cycle)\n\t - Following an unsuccessful flashing session you need to power cycle your Neuron as well (no new firmware will be written)\n\t - This utility operates in 19200 baud mode by default, to switch to 9600 select \"9600 baud\" in the drop-down menu\n\n!!! To recover from an unfinished flashing session please power-cycle your Neuron extension !!!\n\nUNIPI TECHNOLOGIES ACCEPTS NO LIABILITY ARISING FROM INCORRECT USE OF THIS TOOL\n");

    button_hbox = gtk_hbox_new(FALSE, 15);
    label_hbox = gtk_hbox_new(FALSE, 15);
    window_vbox = gtk_vbox_new(FALSE, 5);
    flash_button = gtk_button_new_with_label("Flash");
    output_label = gtk_label_new("Status: ready");
    port_label = gtk_label_new("Choose your port label:");
    port_combo_box = gtk_combo_box_text_new();
    baud_combo_box = gtk_combo_box_text_new();
    settings_hbox = gtk_hbox_new(FALSE, 10);
    verify_label = gtk_label_new("Verify Only: ");
    verify_checkbox = gtk_check_button_new ();

    button_alignment = gtk_alignment_new(1.0, 0.5, 0.5, 0.5);

    gtk_widget_set_margin_left(dialog_label, 30);
    gtk_widget_set_margin_right(dialog_label, 30);
    gtk_widget_set_margin_top(dialog_label, 20);
    //gtk_widget_set_margin_bottom(dialog_label, 10);
    gtk_container_add(GTK_CONTAINER(dialog_content_area), dialog_label);
    gtk_window_set_resizable(GTK_WINDOW(dialog), FALSE);
    gtk_window_set_position(GTK_WINDOW(dialog), GTK_WIN_POS_CENTER);
 //   g_signal_connect(dialog, "destroy", G_CALLBACK(close_dialog))

    gtk_window_set_position(GTK_WINDOW(main_window), GTK_WIN_POS_CENTER);
    gtk_window_set_default_size(GTK_WINDOW(main_window), 600, 280);
    gtk_window_set_resizable(GTK_WINDOW(main_window), FALSE);
    gtk_widget_set_size_request(main_window, 600, 280);
    gtk_container_set_border_width(GTK_CONTAINER(main_window), 15);
    gtk_window_set_title(GTK_WINDOW(main_window), "Neuron Serial Port Firmware Flasher v.1.2");

    gtk_widget_set_margin_left(flash_button, 85);
    gtk_widget_set_margin_top(flash_button, 20);
    gtk_widget_set_size_request(flash_button, 400, 180);
    g_signal_connect(flash_button, "clicked", G_CALLBACK(flash_button_pressed), NULL);
    gtk_box_pack_start(GTK_BOX(button_hbox), flash_button, FALSE, FALSE, 0);


    gtk_widget_set_size_request(output_label, 400, 60);
    gtk_misc_set_alignment (GTK_MISC (output_label), 0.0, 0.5);
    gtk_widget_set_margin_left(output_label, 85);
    gtk_box_pack_start(GTK_BOX(label_hbox), output_label, FALSE, FALSE, 0);

    for (char i = 0; i < 12; i++) {
    	strcpy(box_entries[i], "COM");
    	itoa(i, &box_entries[i][3], 10);
    	strcpy(&(box_entries[i][6]), "/0");
        gtk_combo_box_text_insert(GTK_COMBO_BOX_TEXT(port_combo_box), i, NULL, box_entries[i]);
    	//strcat(box_entries[i], "/n");
    }
    gtk_combo_box_set_active(GTK_COMBO_BOX(port_combo_box), 0);
    g_signal_connect(G_OBJECT(port_combo_box), "changed", G_CALLBACK(select_port), NULL);
    gtk_box_pack_start(GTK_BOX(settings_hbox), port_label, FALSE, FALSE, 0);
    gtk_box_pack_start(GTK_BOX(settings_hbox), port_combo_box, FALSE, FALSE, 0);

    gtk_combo_box_text_insert(GTK_COMBO_BOX_TEXT(baud_combo_box), 0, NULL, "19200 baud");
    gtk_combo_box_text_insert(GTK_COMBO_BOX_TEXT(baud_combo_box), 1, NULL, "9600 baud");
    gtk_combo_box_set_active(GTK_COMBO_BOX(baud_combo_box), 0);
    g_signal_connect(G_OBJECT(baud_combo_box), "changed", G_CALLBACK(select_baud_rate), NULL);
    gtk_box_pack_start(GTK_BOX(settings_hbox), baud_combo_box, FALSE, FALSE, 0);

    gtk_misc_set_alignment(GTK_MISC(verify_label), 0.5f, 0.5f);
    gtk_widget_set_valign(verify_checkbox, GTK_ALIGN_END);
    g_signal_connect(G_OBJECT(verify_checkbox), "toggled", G_CALLBACK(switch_mode_verify), NULL);
    gtk_box_pack_end(GTK_BOX(settings_hbox), verify_checkbox, FALSE, FALSE, 0);
    gtk_widget_set_margin_right(verify_checkbox, 78);
    gtk_box_pack_end(GTK_BOX(settings_hbox), verify_label, FALSE, FALSE, 0);

    gtk_box_pack_start(GTK_BOX(window_vbox), settings_hbox, FALSE, FALSE, 0);
    gtk_box_pack_start(GTK_BOX(window_vbox), button_hbox, FALSE, FALSE, 0);
    gtk_box_pack_start(GTK_BOX(window_vbox), label_hbox, FALSE, FALSE, 0);

    gtk_container_add(GTK_CONTAINER(main_window), window_vbox);

    gtk_widget_show_all(GTK_WIDGET(dialog));
    //gtk_widget_show_all(dialog_window);
    gtk_dialog_run(GTK_DIALOG(dialog));
    gtk_widget_destroy (GTK_WIDGET(dialog));
    select_port(GTK_COMBO_BOX(port_combo_box));

    g_signal_connect_after(G_OBJECT(main_window), "delete-event", G_CALLBACK(close_window), NULL);
    gtk_widget_show_all(main_window);


    gtk_main();
    //gtk_dialog_run (dialog);
}

int main(int argc, char **argv)
{
	// Parse command line options
	int c;
	char *endptr;
	//while (1) {
	int option_index = 0;
	c = getopt_long(argc, argv, "vVPRCp:b:u:d:F:", long_options, &option_index);
	if (c == -1) {
	   if (optind < argc)  {
		   printf ("non-option ARGV-element: %s\n", argv[optind]);
		   exit(EXIT_FAILURE);
		}
	}
/*
       switch (c) {
       case 'v':
           verbose++;
           break;
       case 'V':
           do_verify = 1;
           break;
       case 'P':
           do_prog = 1;
           break;
       case 'R':
           do_resetrw = 1;
           break;
       case 'C':
           do_calibrate = 1; do_prog = 1; do_resetrw = 1;
           break;
       case 'F':
           upboard = strtol(optarg, &endptr, 10);
           if ((endptr==optarg) || (!upboard_exists(upboard))) {
               printf("Available upper board ids:\n");
               print_upboards(-1);
               exit(EXIT_FAILURE);
           }
           do_final = 1; do_prog = 1; do_resetrw = 1;
           break;
       case 'p':
           PORT = strdup(optarg);
           break;
       case 'b':
           BAUD = atoi(optarg);
           if (BAUD==0) {
               printf("Baud must be non-zero integer (given %s)\n", optarg);
               exit(EXIT_FAILURE);
           }
           break;
       case 'u':
           DEVICE_ID = atoi(optarg);
           if (DEVICE_ID==0) {
               printf("Unit must be non-zero integer (given %s)\n", optarg);
               exit(EXIT_FAILURE);
           }
           break;
       case 'd':
           firmwaredir = strdup(optarg);
           break;

       default:
           print_usage(argv[0]);
           exit(EXIT_FAILURE);
           break;
       }
    }
*/
    //}
    setup_gui(argc, argv);

/*
    if (do_prog || do_verify) {
        // FW manipulation
        if (do_calibrate) {
            bv.hw_version = bv.base_hw_version | 0x8;
            do_resetrw = 1;
        } else if (do_final) {
            if (!IS_CALIB(bv.hw_version)) {
                fprintf(stderr, "Only calibrating version can be reprogrammed to final\n");
                modbus_free(ctx);
                return -1;
            }
            bv.hw_version = check_compatibility(bv.base_hw_version, upboard);
            if (bv.hw_version == 0) {
                fprintf(stderr, "Incompatible base and upper boards. Use one of:\n");
                print_upboards(bv.base_hw_version);
                modbus_free(ctx);
                return -1;
            }
        }
        // load firmware file
        char* fwname = firmware_name(bv.hw_version, bv.base_hw_version, firmwaredir, ".bin");
        prog_data = malloc(MAX_FW_SIZE);
        if (verbose) printf("Opening firmware file: %s\n", fwname);
        int red = load_fw(fwname, prog_data, MAX_FW_SIZE);
        int rwred = RW_START_PAGE;
        free(fwname);
        if (red <= 0) {
            if (red == 0) {
                fprintf(stderr, "Firmware file is empty!\n");
            }
            free(prog_data);
            modbus_free(ctx);
            return -1;
        }
        red = (red + (PAGE_SIZE - 1)) / PAGE_SIZE;
        if (verbose) printf("Program pages: %d\n", red);

        if (do_resetrw) {
            // load rw consts file
            rw_data = malloc(MAX_RW_SIZE);
            char* rwname = firmware_name(bv.hw_version, bv.base_hw_version, firmwaredir, ".rw");
            if (verbose) printf("Opening RW settings file: %s\n", rwname);
            int rwlen = load_fw(rwname, rw_data, MAX_RW_SIZE);
            free(rwname);
            // calc page count of firmware file
            rwred += ((rwlen + (PAGE_SIZE - 1)) / PAGE_SIZE);
            if (verbose) printf("Final page: %d\n", rwred);
        }

        // init FW programmer
        if (modbus_write_bit(ctx, 1006, 1, 0) != 1) {
            fprintf(stderr, "Program mode setting failed: %s\n", modbus_strerror(errno));
            modbus_free(ctx);
            return -1;
        }
        /*
        if (modbus_read_registers(ctx, 1000, 5, version) == 5) {
        if (modbus_read_registers(ctx, 1000, 5, r1000) == 5) {
            parse_version(&xbv, r1000);
            if (xbv.hw_version != 0xffff) {
                printf("Boot boardset:   %3d (v%d.%d%s)\n", xhw_version >> 8, (xhw_version & 0xff)>>4, xhw_version & 0x7, (xhw_version & 0x8)?" CAL":"");
            } else {
                printf("Boot boardset:   Undefined\n");
            }
            printf("Boot baseboard:  %3d (v%d.%d)\n", xbase_version>> 8, (xbase_version & 0xff)>>4, xbase_version & 0x7);
            printf("Boot firmware:  v%d.%d\n", xsw_version >> 8, xsw_version & 0xff);
        }

        if (do_prog || do_calibrate) {
            flashit(ctx,prog_data, rw_data, red, rwred);
        }
        if (do_verify) {
            verify(ctx,prog_data, rw_data, red, rwred);
        }
        modbus_write_register(ctx, 0x7707, 3); // reboot
        free(prog_data);
    }
    */
    modbus_free(ctx);
    gtk_widget_destroy (GTK_WIDGET (dialog));
    return 0;
}
