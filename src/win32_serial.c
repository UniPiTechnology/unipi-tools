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
#include "win32_serial.h"
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
int	active_baud = 19200;
int	active_address = 15;
int active_stopbits = 1;
char active_parity = 'N';
#ifdef OS_WIN32
char* firmwaredir = "./fw";
#else
char* firmwaredir = "/opt/unipi/firmware";
#endif
int upboard;

int do_verify = 0;
int do_prog   = 1;
int do_connect = 1;
int do_resetrw = 0;
int do_calibrate = 0;
int do_final = 0;

GtkApplication *app;

GtkWidget *dialog;
GtkWidget *dialog_content_area;
GtkWidget *dialog_label;

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
GtkWidget *settings_label;

GtkWidget *port_combo_box;
GtkWidget *baud_combo_box;
GtkWidget *address_combo_box;
GtkWidget *parity_combo_box;
GtkWidget *stop_bit_combo_box;

GtkWidget *verify_checkbox;
GtkWidget *verify_label;

GtkWidget *port_hbox;
GtkWidget *settings_hbox;

uint8_t *prog_data;   // buffer containing firmware
uint8_t* rw_data;     // buffer containing firmware rw data
uint16_t* pd;
int ret, chunk, page;
uint16_t val, reg;
modbus_t *ctx;
FILE* fdx;

char box_entries[255][64];
char parity_entries[2] = {
	'N','E'
};
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
		if (modbus_write_register(ctx, 0x7705, page) != 1) {   // set page address in the target device
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
			snprintf(output_string, sizeof(output_string), "Verification failed!!: %s\n", modbus_strerror(errno));
			gtk_label_set_text(GTK_LABEL(output_label), output_string);
			return -1;
		}
		if (page == last_prog_page-1) {
			page = RW_START_PAGE-1;
			pd = (uint16_t*) rw_data;
		}
	}
	snprintf(output_string, sizeof(output_string), "Verification passed!\n");
	gtk_label_set_text(GTK_LABEL(output_label), output_string);
}

int flashit(modbus_t *ctx, uint8_t* prog_data, uint8_t* rw_data, int last_prog_page, int last_page)
{
	uint16_t* pd;
	int ret, chunk, page;
	int retval = -1;
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
		if (modbus_write_register(ctx, 0x7705, page) == 1) {   // set page address in the target device
			for (chunk=0; chunk < 8; chunk++) {
				retval = modbus_write_registers(ctx, 0x7700+chunk, REG_SIZE, pd);
				while (retval == -1 && errors < 255) { // send chunk of data (64*2 B)
					retval = modbus_write_registers(ctx, 0x7700+chunk, REG_SIZE, pd);
					errors++;
				}
				fprintf(stderr, "Written Chunk %d, R:%d E:%d\n", chunk, retval, errors);
				pd += REG_SIZE;
			}
			retval = modbus_write_register(ctx, 0x7707, 1);
			if (retval == 1) {
				printf("Page Written OK\n");
				if (page == last_prog_page) {
					page = RW_START_PAGE;
				} else {
					page++;
				}

			} else {
				errors++;
				printf(" Trying again\n");
				fprintf(stderr, "Flashing page failed A: %s\n", modbus_strerror(errno));
				while (retval != 1) {  // write page to flash
					retval = modbus_write_register(ctx, 0x7707, 1);
					if (retval != 1) {
						printf("Page Written OK\n");
						if (page == last_prog_page) {
							page = RW_START_PAGE;
						} else {
							page++;
						}

					} else {
						errors++;
						printf(" Trying again A\n");
						fprintf(stderr, "Flashing page failed B: %s\n", modbus_strerror(errno));
					}
				}
			}
		} else {
			errors++;
			printf(" Trying again B\n");
		}
		if (errors > 255) {
			snprintf(output_string, sizeof(output_string), "Flashing not successful!! Power-cycle your target device");
			gtk_label_set_text(GTK_LABEL(output_label), output_string);
			do_connect = 1;
			gtk_button_set_label(GTK_BUTTON(flash_button), "Connect");
			return -1;
		}
	}
	snprintf(output_string, sizeof(output_string), "Flashing successful!! Power-cycle your target device");
	gtk_label_set_text(GTK_LABEL(output_label), output_string);
	do_connect = 1;
	gtk_button_set_label(GTK_BUTTON(flash_button), "Connect");
	return 0;
}

int load_fw(char *path, uint8_t* prog_data, const size_t len)
{
    FILE* fd;
    int read_n, i;
    fd = fopen(path, "rb");
    if (!fd) {
        printf("Error opening firmware file \"%s\"\n", path);
        return -1;
    }
    struct stat finfo;
    fstat(fd->_file, &finfo);
    off_t filesize = finfo.st_size;
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
    // Open port
	gint i = gtk_combo_box_get_active(caller);
    active_port = box_entries[i];
	do_connect = 1;
	gtk_label_set_text(GTK_LABEL(output_label), " ");
	gtk_button_set_label(GTK_BUTTON(flash_button), "Connect");
}

void select_parity(GtkComboBox *caller) {
    // Open port
	gint i = gtk_combo_box_get_active(caller);
    active_parity = parity_entries[i];
	do_connect = 1;
	gtk_label_set_text(GTK_LABEL(output_label), " ");
	gtk_button_set_label(GTK_BUTTON(flash_button), "Connect");
}

void select_address(GtkComboBox *caller) {
	gint i = gtk_combo_box_get_active(caller);
	switch (i) {
	case 0: {
		active_address = 1;
		break;
	}
	case 1: {
		active_address = 2;
		break;
	}
	case 2: {
		active_address = 3;
		break;
	}
	case 3: {
		active_address = 4;
		break;
	}
	case 4: {
		active_address = 5;
		break;
	}
	case 5: {
		active_address = 6;
		break;
	}
	case 6: {
		active_address = 7;
		break;
	}
	case 7: {
		active_address = 8;
		break;
	}
	case 8: {
		active_address = 9;
		break;
	}
	case 9: {
		active_address = 10;
		break;
	}
	case 10: {
		active_address = 11;
		break;
	}
	case 11: {
		active_address = 12;
		break;
	}
	case 12: {
		active_address = 13;
		break;
	}
	case 13: {
		active_address = 14;
		break;
	}
	case 14: {
		active_address = 15;
		break;
	}
	default: {
		active_address = 15;
		break;
	}
	}
	do_connect = 1;
	gtk_label_set_text(GTK_LABEL(output_label), " ");
	gtk_button_set_label(GTK_BUTTON(flash_button), "Connect");
}

void select_baud_rate(GtkComboBox *caller) {
	gint i = gtk_combo_box_get_active(caller);
	switch (i) {
	case 0: {
		active_baud = 1200;
		break;
	}
	case 1: {
		active_baud = 2400;
		break;
	}
	case 2: {
		active_baud = 4800;
		break;
	}
	case 3: {
		active_baud = 9600;
		break;
	}
	case 4: {
		active_baud = 19200;
		break;
	}
	case 5: {
		active_baud = 38400;
		break;
	}
	case 6: {
		active_baud = 57600;
		break;
	}
	case 7: {
		active_baud = 115200;
		break;
	}
	default: {
		active_baud = 19200;
		break;
	}
	}
	do_connect = 1;
	gtk_label_set_text(GTK_LABEL(output_label), " ");
	gtk_button_set_label(GTK_BUTTON(flash_button), "Connect");
}

void select_stop_bit_count(GtkComboBox *caller) {
	gint i = gtk_combo_box_get_active(caller);
	switch (i) {
	case 0: {
		active_stopbits = 1;
		break;
	}
	case 1: {
		active_stopbits = 2;
		break;
	}
	default: {
		active_stopbits = 1;
		break;
	}
	}
	do_connect = 1;
	gtk_label_set_text(GTK_LABEL(output_label), " ");
	gtk_button_set_label(GTK_BUTTON(flash_button), "Connect");
}

void test_connection_settings(void) {
	ctx = modbus_new_rtu(active_port, active_baud, active_parity, 8, active_stopbits);
    modbus_set_response_timeout(ctx, 0, 800000);
    if (ctx == NULL) {
        gtk_label_set_text(GTK_LABEL(output_label), "ERROR: Unable to create ModBus context (wrong port?)");
        return;
    }
    modbus_set_slave(ctx, active_address);

    if (modbus_connect(ctx) == -1) {
    	gtk_label_set_text(GTK_LABEL(output_label), "ERROR: Connection failed (wrong port?)");
    	modbus_close(ctx);
    	modbus_free(ctx);
        return;
    }
    // get FW & HW version
    uint16_t r1000[7];
    Tboard_version bv;
    if (modbus_read_registers(ctx, 1000, 7, r1000) == 7) {
        parse_version(&bv, r1000);
        snprintf(output_string, sizeof(output_string), "Product:\t%s\nE-Board S/N:\t(%d)\nBoardset:\t\t\t%3d\t%-35s - (v%d.%d%s)\nBaseboard:\t\t%3d\t%-35s - (v%d.%d)\nFirmware:\t\tv%d.%d\n",
        		get_extension_map(HW_BOARD(bv.hw_version))->product, (r1000[6] << 16) + r1000[5],
        	   HW_BOARD(bv.hw_version),  arm_name(bv.hw_version),
               HW_MAJOR(bv.hw_version), HW_MINOR(bv.hw_version),
               IS_CALIB(bv.hw_version)?" CAL":"",
               HW_BOARD(bv.base_hw_version),  arm_name(bv.base_hw_version),
               HW_MAJOR(bv.base_hw_version), HW_MINOR(bv.base_hw_version),
			   SW_MAJOR(bv.sw_version), SW_MINOR(bv.sw_version));
        gtk_label_set_text(GTK_LABEL(output_label), output_string);
        do_connect = 0;
        if (do_verify) {
        	gtk_button_set_label(GTK_BUTTON(flash_button), "Verify");
        } else {
        	gtk_button_set_label(GTK_BUTTON(flash_button), "Flash");
        }
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
		if (!do_connect) {
			gtk_button_set_label(GTK_BUTTON(flash_button), "Verify");
		}
	} else {
		do_verify = 0;
		do_prog = 1;
		if (!do_connect) {
			gtk_button_set_label(GTK_BUTTON(flash_button), "Flash");
		}
	}
}

void flash_button_pressed(GtkButton *caller) {
	if (do_connect) {
		test_connection_settings();
		return;
	}
	do_resetrw = 1;
	// FW manipulation
	ctx = modbus_new_rtu(active_port, active_baud, active_parity, 8, active_stopbits);
    modbus_set_response_timeout(ctx, 0, 800000);
    if (ctx == NULL) {
        gtk_label_set_text(GTK_LABEL(output_label), "ERROR: Unable to create ModBus context (wrong port?)");
        return;
    }
    modbus_set_slave(ctx, active_address);

    if (modbus_connect(ctx) == -1) {
    	gtk_label_set_text(GTK_LABEL(output_label), "ERROR: Connection failed (wrong port?)");
    	modbus_close(ctx);
    	modbus_free(ctx);
        return;
    }

    // get FW & HW version
    uint16_t r1000[5];
    Tboard_version bv;
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
	char* fwname = firmware_name(&bv, firmwaredir, ".bin");
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
		char* rwname = firmware_name(&bv, firmwaredir, ".rw");
		printf("Opening RW settings file: %s\n", rwname);
		int rwlen = load_fw(rwname, rw_data, MAX_RW_SIZE);
		free(rwname);
		// calc page count of firmware file
		rwred += ((rwlen + (PAGE_SIZE - 1)) / PAGE_SIZE);
		printf("Final page: %d, READ: %d, rwlen: %d\n", rwred, red, rwlen);
	}

	// init FW programmer
	if (modbus_write_bit(ctx, 1006, 1) != 1) {
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

void setup_gui(int argc, char **argv) {
    gtk_init (&argc, &argv);

    main_window = gtk_window_new(GTK_WINDOW_TOPLEVEL);

    dialog_window = gtk_window_new(GTK_WINDOW_TOPLEVEL);
    dialog = gtk_dialog_new_with_buttons("PLEASE READ TY VOLE - IMPORTANT INFORMATION", GTK_WINDOW(main_window), GTK_DIALOG_MODAL, "I Understand", GTK_RESPONSE_DELETE_EVENT, NULL);
    dialog_content_area = gtk_dialog_get_content_area (GTK_DIALOG (dialog));
    dialog_label = gtk_label_new("In order to correctly flash an updated firmware to your UniPi extension device, ensure the following:\n\n\t - Have a properly configured (1 stop bit, no parity, 19200/9600 baud) serial adapter device plugged into your computer\n\t - Turn on the termination resistors (pin 1) on your UniPi extension and use a twisted-pair cable to connect it to your PC adapter\n\t - Set your UniPi Serial device to the default address (address 15)\n\n Note also the following:\n\n\t - New firmware is only loaded into the UniPi device upon restart (power cycle)\n\t - Following an unsuccessful flashing session you need to power cycle your UniPi device as well (no new firmware will be written)\n\t - This utility operates in 19200 baud mode by default, to switch to 9600 select \"9600 baud\" in the drop-down menu\n\n!!! To recover from an unfinished flashing session please power-cycle your UniPi extension !!!\n\nUNIPI TECHNOLOGIES ACCEPTS NO LIABILITY ARISING FROM INCORRECT USE OF THIS TOOL\n");

    button_hbox = gtk_hbox_new(FALSE, 15);
    label_hbox = gtk_hbox_new(FALSE, 15);
    window_vbox = gtk_vbox_new(FALSE, 10);
    flash_button = gtk_button_new_with_label("Connect");
    output_label = gtk_label_new("Status: ready");
    port_label = gtk_label_new("Port settings:");
    port_combo_box = gtk_combo_box_text_new();
    address_combo_box = gtk_combo_box_text_new();
    settings_label = gtk_label_new("TERMIOS settings:");
    baud_combo_box = gtk_combo_box_text_new();
    stop_bit_combo_box = gtk_combo_box_text_new();
    parity_combo_box = gtk_combo_box_text_new();
    port_hbox = gtk_hbox_new(FALSE, 10);
    settings_hbox = gtk_hbox_new(FALSE, 10);
    verify_label = gtk_label_new("Verify Only: ");
    verify_checkbox = gtk_check_button_new ();

    button_alignment = gtk_alignment_new(1.0, 0.5, 0.5, 0.5);

    gtk_widget_set_margin_left(dialog_label, 30);
    gtk_widget_set_margin_right(dialog_label, 30);
    gtk_widget_set_margin_top(dialog_label, 20);
    gtk_container_add(GTK_CONTAINER(dialog_content_area), dialog_label);
    gtk_window_set_resizable(GTK_WINDOW(dialog), FALSE);
    gtk_window_set_position(GTK_WINDOW(dialog), GTK_WIN_POS_CENTER);

    gtk_window_set_position(GTK_WINDOW(main_window), GTK_WIN_POS_CENTER);
    gtk_window_set_default_size(GTK_WINDOW(main_window), 400, 320);
    gtk_window_set_resizable(GTK_WINDOW(main_window), FALSE);
    gtk_widget_set_size_request(main_window, 400, 320);
    gtk_container_set_border_width(GTK_CONTAINER(main_window), 10);
    gtk_window_set_title(GTK_WINDOW(main_window), "UniPi Serial Port Firmware Flasher v.1.3");

    gtk_widget_set_margin_left(flash_button, 10);
    gtk_widget_set_margin_top(flash_button, 10);
    gtk_widget_set_size_request(flash_button, 385, 140);
    g_signal_connect(flash_button, "clicked", G_CALLBACK(flash_button_pressed), NULL);
    gtk_box_pack_start(GTK_BOX(button_hbox), flash_button, FALSE, FALSE, 0);

    gtk_widget_set_size_request(output_label, 385, 60);
    gtk_misc_set_alignment (GTK_MISC (output_label), 0.0, 0.5);
    gtk_widget_set_margin_left(output_label, 10);
    gtk_box_pack_start(GTK_BOX(label_hbox), output_label, FALSE, FALSE, 0);

    for (char i = 0; i < 12; i++) {
    	strcpy(box_entries[i], "COM");
    	itoa(i, &box_entries[i][3], 10);
    	strcpy(&(box_entries[i][6]), "/0");
        gtk_combo_box_text_insert(GTK_COMBO_BOX_TEXT(port_combo_box), i, NULL, box_entries[i]);
    }
    gtk_combo_box_set_active(GTK_COMBO_BOX(port_combo_box), 0);
    g_signal_connect(G_OBJECT(port_combo_box), "changed", G_CALLBACK(select_port), NULL);


    gtk_combo_box_text_insert(GTK_COMBO_BOX_TEXT(baud_combo_box), 0, NULL, "1200 baud");
    gtk_combo_box_text_insert(GTK_COMBO_BOX_TEXT(baud_combo_box), 1, NULL, "2400 baud");
    gtk_combo_box_text_insert(GTK_COMBO_BOX_TEXT(baud_combo_box), 2, NULL, "4800 baud");
    gtk_combo_box_text_insert(GTK_COMBO_BOX_TEXT(baud_combo_box), 3, NULL, "9600 baud");
    gtk_combo_box_text_insert(GTK_COMBO_BOX_TEXT(baud_combo_box), 4, NULL, "19200 baud");
    gtk_combo_box_text_insert(GTK_COMBO_BOX_TEXT(baud_combo_box), 5, NULL, "38400 baud");
    gtk_combo_box_text_insert(GTK_COMBO_BOX_TEXT(baud_combo_box), 6, NULL, "57600 baud");
    gtk_combo_box_text_insert(GTK_COMBO_BOX_TEXT(baud_combo_box), 7, NULL, "115200 baud");
    gtk_combo_box_set_active(GTK_COMBO_BOX(baud_combo_box), 4);
    g_signal_connect(G_OBJECT(baud_combo_box), "changed", G_CALLBACK(select_baud_rate), NULL);

    gtk_combo_box_text_insert(GTK_COMBO_BOX_TEXT(address_combo_box), 0, NULL, "UnitID 1");
    gtk_combo_box_text_insert(GTK_COMBO_BOX_TEXT(address_combo_box), 1, NULL, "UnitID 2");
    gtk_combo_box_text_insert(GTK_COMBO_BOX_TEXT(address_combo_box), 2, NULL, "UnitID 3");
    gtk_combo_box_text_insert(GTK_COMBO_BOX_TEXT(address_combo_box), 3, NULL, "UnitID 4");
    gtk_combo_box_text_insert(GTK_COMBO_BOX_TEXT(address_combo_box), 4, NULL, "UnitID 5");
    gtk_combo_box_text_insert(GTK_COMBO_BOX_TEXT(address_combo_box), 5, NULL, "UnitID 6");
    gtk_combo_box_text_insert(GTK_COMBO_BOX_TEXT(address_combo_box), 6, NULL, "UnitID 7");
    gtk_combo_box_text_insert(GTK_COMBO_BOX_TEXT(address_combo_box), 7, NULL, "UnitID 8");
    gtk_combo_box_text_insert(GTK_COMBO_BOX_TEXT(address_combo_box), 8, NULL, "UnitID 9");
    gtk_combo_box_text_insert(GTK_COMBO_BOX_TEXT(address_combo_box), 9, NULL, "UnitID 10");
    gtk_combo_box_text_insert(GTK_COMBO_BOX_TEXT(address_combo_box), 10, NULL, "UnitID 11");
    gtk_combo_box_text_insert(GTK_COMBO_BOX_TEXT(address_combo_box), 11, NULL, "UnitID 12");
    gtk_combo_box_text_insert(GTK_COMBO_BOX_TEXT(address_combo_box), 12, NULL, "UnitID 13");
    gtk_combo_box_text_insert(GTK_COMBO_BOX_TEXT(address_combo_box), 13, NULL, "UnitID 14");
    gtk_combo_box_text_insert(GTK_COMBO_BOX_TEXT(address_combo_box), 14, NULL, "UnitID 15");
    gtk_combo_box_set_active(GTK_COMBO_BOX(address_combo_box), 14);
    g_signal_connect(G_OBJECT(address_combo_box), "changed", G_CALLBACK(select_address), NULL);

    gtk_combo_box_text_insert(GTK_COMBO_BOX_TEXT(parity_combo_box), 0, NULL, "N");
    gtk_combo_box_text_insert(GTK_COMBO_BOX_TEXT(parity_combo_box), 1, NULL, "E");
    gtk_combo_box_set_active(GTK_COMBO_BOX(parity_combo_box), 0);
    g_signal_connect(G_OBJECT(parity_combo_box), "changed", G_CALLBACK(select_parity), NULL);

    gtk_combo_box_text_insert(GTK_COMBO_BOX_TEXT(stop_bit_combo_box), 0, NULL, "One SB");
    gtk_combo_box_text_insert(GTK_COMBO_BOX_TEXT(stop_bit_combo_box), 1, NULL, "Two SBs");
    gtk_combo_box_set_active(GTK_COMBO_BOX(stop_bit_combo_box), 0);
    g_signal_connect(G_OBJECT(stop_bit_combo_box), "changed", G_CALLBACK(select_stop_bit_count), NULL);

    gtk_misc_set_alignment(GTK_MISC(verify_label), 0.5f, 0.5f);
    gtk_widget_set_valign(verify_checkbox, GTK_ALIGN_END);
    gtk_widget_set_margin_right(verify_checkbox, 10);
    g_signal_connect(G_OBJECT(verify_checkbox), "toggled", G_CALLBACK(switch_mode_verify), NULL);

    gtk_widget_set_margin_left(port_label, 10);
    gtk_box_pack_start(GTK_BOX(port_hbox), port_label, FALSE, FALSE, 0);
    gtk_box_pack_start(GTK_BOX(port_hbox), port_combo_box, FALSE, FALSE, 0);
    gtk_box_pack_start(GTK_BOX(port_hbox), address_combo_box, FALSE, FALSE, 0);
    gtk_box_pack_start(GTK_BOX(port_hbox), verify_label, FALSE, FALSE, 0);
    gtk_box_pack_start(GTK_BOX(port_hbox), verify_checkbox, FALSE, FALSE, 0);

    gtk_widget_set_margin_left(settings_label, 10);
    gtk_box_pack_start(GTK_BOX(settings_hbox), settings_label, FALSE, FALSE, 0);
    gtk_box_pack_start(GTK_BOX(settings_hbox), baud_combo_box, FALSE, FALSE, 0);
    gtk_box_pack_start(GTK_BOX(settings_hbox), parity_combo_box, FALSE, FALSE, 0);
    gtk_box_pack_start(GTK_BOX(settings_hbox), stop_bit_combo_box, FALSE, FALSE, 0);

    gtk_box_pack_start(GTK_BOX(window_vbox), port_hbox, FALSE, FALSE, 0);
    gtk_box_pack_start(GTK_BOX(window_vbox), settings_hbox, FALSE, FALSE, 0);
    gtk_box_pack_start(GTK_BOX(window_vbox), button_hbox, FALSE, FALSE, 0);
    gtk_box_pack_start(GTK_BOX(window_vbox), label_hbox, FALSE, FALSE, 0);

    gtk_container_add(GTK_CONTAINER(main_window), window_vbox);

    gtk_widget_show_all(GTK_WIDGET(dialog));
    gtk_dialog_run(GTK_DIALOG(dialog));
    gtk_widget_destroy (GTK_WIDGET(dialog));
    select_port(GTK_COMBO_BOX(port_combo_box));

    g_signal_connect_after(G_OBJECT(main_window), "delete-event", G_CALLBACK(close_window), NULL);
    gtk_widget_show_all(main_window);


    gtk_main();
}

int main(int argc, char **argv)
{
	// Parse command line options
	int c;
    setup_gui(argc, argv);
    modbus_free(ctx);
    gtk_widget_destroy (GTK_WIDGET (dialog));
    return 0;
}
