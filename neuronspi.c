/*
 * UniPi Neuron tty serial driver - Copyright (C) 2017 UniPi Technologies
 * Author: Tomas Knot <tomasknot@gmail.com>
 *
 *  Based on the SC16IS7xx driver by Jon Ringle <jringle@gridpoint.com>,
 *  which was in turn based on max310x.c, by Alexander Shiyan <shc_work@mail.ru>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt
#define NEURONSPI_SCHED_REQUIRED 0 // Older kernels do not require sched/types to be specifically imported

#include <linux/bitops.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/regmap.h>
#include <linux/serial_core.h>
#include <linux/serial.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/spi/spi.h>
#include <linux/uaccess.h>
#include <asm/termbits.h>

#if NEURONSPI_SCHED_REQUIRED > 1
#include <uapi/linux/sched/types.h>
#endif

/***************
 * Definitions *
 ***************/

#define NEURONSPI_MAX_DEVS				3
#define NEURONSPI_MAX_UART				128
#define NEURONSPI_BUFFER_MAX			1152
#define NEURONSPI_HEADER_LENGTH 		10
#define NEURONSPI_FIRST_MESSAGE_LENGTH	6
#define NEURONSPI_EDGE_DELAY			10
#define NEURONSPI_B_PER_WORD 			8
#define NEURONSPI_DEFAULT_FREQ			500000
#define NEURONSPI_COMMON_FREQ			12000000
#define NEURONSPI_SLOWER_FREQ			8000000
#define NEURONSPI_MAX_TX				64
#define NEURONSPI_MAX_BAUD				115200
#define NEURONSPI_FIFO_SIZE				(256)
#define NEURONSPI_DETAILED_DEBUG		0

#define NEURONSPI_NAME "neuronspi"
#define NEURON_DEVICE_NAME "neuron"
#define NEURON_DEVICE_CLASS "neuron"
#define NEURON_MAX_MB_BUFFER_LEN 260
#define MODBUS_TCP_HEADER_LENGTH 7
#define PORT_NEURONSPI	184

#define STRICT_RESERVING

#define NEURONSPI_NO_INTERRUPT_MODELS_LEN 3
const uint16_t NEURONSPI_NO_INTERRUPT_MODELS[NEURONSPI_NO_INTERRUPT_MODELS_LEN] = {
		0xb10, 0xc10, 0xf10
};

#define NEURONSPI_SLOWER_MODELS_LEN 3
const uint16_t NEURONSPI_SLOWER_MODELS[NEURONSPI_SLOWER_MODELS_LEN] = {
		0xb10, 0xc10, 0xf10
};

#define NEURONSPI_PROBE_MESSAGE_LEN				22
const u8 NEURONSPI_PROBE_MESSAGE[NEURONSPI_PROBE_MESSAGE_LEN] = {
		0x04, 0x0e, 0xe8, 0x03, 0xa0, 0xdd,
		0x04, 0x00, 0xe8, 0x03,	0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00,	0x12, 0x16
};

#define NEURONSPI_UART_PROBE_MESSAGE_LEN			6
const u8 NEURONSPI_UART_PROBE_MESSAGE[NEURONSPI_UART_PROBE_MESSAGE_LEN] = {
		0xfa, 0x00, 0x55, 0x0e, 0xb6, 0x0a
};

#define NEURONSPI_SPI_UART_SHORT_MESSAGE_LEN		6
const u8 NEURONSPI_SPI_UART_SHORT_MESSAGE[NEURONSPI_SPI_UART_SHORT_MESSAGE_LEN] = {
		0x41, 0x00, 0x00, 0x00, 0x00, 0x00
};

#define NEURONSPI_SPI_UART_LONG_MESSAGE_LEN			8
const u8 NEURONSPI_SPI_UART_LONG_MESSAGE[NEURONSPI_SPI_UART_LONG_MESSAGE_LEN] = {
		0x64, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00
};

#define NEURONSPI_SPI_UART_READ_MESSAGE_LEN			14
const u8 NEURONSPI_SPI_UART_READ_MESSAGE[NEURONSPI_SPI_UART_READ_MESSAGE_LEN] = {
		0x65, 0x06, 0x00, 0x00, 0x44, 0x69,
		0x65, 0x03, 0x00, 0x00, 0x00, 0x05,
		0x6a, 0x0c
};

#define NEURONSPI_SPI_IRQ_SET_MESSAGE_LEN			14
const u8 NEURONSPI_SPI_IRQ_SET_MESSAGE[NEURONSPI_SPI_UART_READ_MESSAGE_LEN] = {
		0x06, 0x06, 0xef, 0x03, 0x00, 0x00,
		0x06, 0x01, 0xef, 0x03, 0x05, 0x00,
		0x00, 0x00
};

#define NEURONSPI_SPI_UART_GET_CFLAG_MESSAGE_LEN 	16
const u8 NEURONSPI_SPI_UART_GET_CFLAG_MESSAGE[NEURONSPI_SPI_UART_GET_CFLAG_MESSAGE_LEN] = {
		0x04, 0x08, 0xf4, 0x01, 0x00, 0x00,
		0x04, 0x02, 0xf4, 0x01, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00
};

#define NEURONSPI_SPI_UART_SET_CFLAG_MESSAGE_LEN	16
const u8 NEURONSPI_SPI_UART_SET_CFLAG_MESSAGE[NEURONSPI_SPI_UART_SET_CFLAG_MESSAGE_LEN] = {
		0x06, 0x08, 0xf4, 0x01, 0x00, 0x00,
		0x06, 0x02, 0xf4, 0x01, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00
};

#define NEURONSPI_CRC16TABLE_LEN					256
const uint16_t NEURONSPI_CRC16TABLE[NEURONSPI_CRC16TABLE_LEN] = {
    0,  1408,  3968,  2560,  7040,  7680,  5120,  4480, 13184, 13824, 15360,
14720, 10240, 11648, 10112,  8704, 25472, 26112, 27648, 27008, 30720, 32128,
30592, 29184, 20480, 21888, 24448, 23040, 19328, 19968, 17408, 16768, 50048,
50688, 52224, 51584, 55296, 56704, 55168, 53760, 61440, 62848, 65408, 64000,
60288, 60928, 58368, 57728, 40960, 42368, 44928, 43520, 48000, 48640, 46080,
45440, 37760, 38400, 39936, 39296, 34816, 36224, 34688, 33280, 33665, 34305,
35841, 35201, 38913, 40321, 38785, 37377, 45057, 46465, 49025, 47617, 43905,
44545, 41985, 41345, 57345, 58753, 61313, 59905, 64385, 65025, 62465, 61825,
54145, 54785, 56321, 55681, 51201, 52609, 51073, 49665, 16385, 17793, 20353,
18945, 23425, 24065, 21505, 20865, 29569, 30209, 31745, 31105, 26625, 28033,
26497, 25089,  9089,  9729, 11265, 10625, 14337, 15745, 14209, 12801,  4097,
 5505,  8065,  6657,  2945,  3585,  1025,   385,   899,  1539,  3075,  2435,
 6147,  7555,  6019,  4611, 12291, 13699, 16259, 14851, 11139, 11779,  9219,
 8579, 24579, 25987, 28547, 27139, 31619, 32259, 29699, 29059, 21379, 22019,
23555, 22915, 18435, 19843, 18307, 16899, 49155, 50563, 53123, 51715, 56195,
56835, 54275, 53635, 62339, 62979, 64515, 63875, 59395, 60803, 59267, 57859,
41859, 42499, 44035, 43395, 47107, 48515, 46979, 45571, 36867, 38275, 40835,
39427, 35715, 36355, 33795, 33155, 32770, 34178, 36738, 35330, 39810, 40450,
37890, 37250, 45954, 46594, 48130, 47490, 43010, 44418, 42882, 41474, 58242,
58882, 60418, 59778, 63490, 64898, 63362, 61954, 53250, 54658, 57218, 55810,
52098, 52738, 50178, 49538, 17282, 17922, 19458, 18818, 22530, 23938, 22402,
20994, 28674, 30082, 32642, 31234, 27522, 28162, 25602, 24962,  8194,  9602,
12162, 10754, 15234, 15874, 13314, 12674,  4994,  5634,  7170,  6530,  2050,
 3458,  1922,   514
};


static const struct of_device_id neuronspi_id_match[] = {
		{.compatible = "unipi,neuron"},
		{.compatible = NEURONSPI_NAME},
		{}
};
MODULE_DEVICE_TABLE(of, neuronspi_id_match);

#define NEURON_INT_RX_NOT_EMPTY 			0x1
#define NEURON_INT_TX_FINISHED  			0x2
#define NEURON_INT_RX_MODBUS    			0x4
#define NEURON_INT_DI_CHANGED   			0x8
#define NEURON_INT_ID_MASK      			0x0f
#define NEURON_INT_NO_INT_BIT   			0x0f

#define NEURONSPI_RECONF_MD					(1 << 0)
#define NEURONSPI_RECONF_IER				(1 << 1)
#define NEURONSPI_RECONF_RS485				(1 << 2)

#define MODBUS_MAX_READ_BITS               2000
#define MODBUS_MAX_WRITE_BITS              1968
#define MODBUS_MAX_READ_REGISTERS          125
#define MODBUS_MAX_WRITE_REGISTERS         123
#define MODBUS_MAX_WR_WRITE_REGISTERS      121
#define MODBUS_MAX_WR_READ_REGISTERS       125


/*******************
 * Data structures *
 *******************/

struct neuronspi_devtype
{
	u8	name[10];
	int32_t	nr_gpio;
	int32_t	nr_uart;
};

struct neuronspi_port
{
	struct uart_port			port;
	u8							line;
	struct kthread_work			tx_work;
	struct kthread_work			rx_work;
	struct kthread_work			irq_work;
	u32							flags;
	u8							ier_clear;
	u8							buf[NEURONSPI_FIFO_SIZE];
	struct neuronspi_uart_data *parent;
	u8							dev_index;
	u8							dev_port;
};

struct neuronspi_uart_data
{
	const struct neuronspi_devtype	*devtype;
	struct kthread_worker			kworker;
	struct task_struct				*kworker_task;
	struct neuronspi_port			*p;
	u8								p_count;
};

static struct neuronspi_uart_data* neuronspi_uart_glob_data;

static unsigned long neuronspi_lines;

static struct uart_driver* neuronspi_uart;

// Instantiated once
struct neuronspi_char_driver
{
	int32_t major_number;
	u8 *message;
	u16 message_size;
	u32 open_counter;
	struct class* driver_class;
	struct device* dev;
};

// Instantiated once per SPI device
struct neuronspi_driver_data
{
	struct spi_driver *spi_driver;
	struct neuronspi_char_driver *char_driver;
	struct uart_driver *serial_driver;
	struct neuronspi_uart_data *uart_data;
	struct task_struct *poll_thread;
	struct mutex device_lock;
	u8 *send_buf;
	u8 *recv_buf;
	u8 *first_probe_reply;
	u8 *second_probe_reply;
	u8 reserved_device;
	u8 uart_count;
	u8 uart_read;
	u8 *uart_buf;
	u8 spi_index;
	u8 slower_model;
	u8 no_irq;
	int32_t neuron_index;
};

struct mutex neuronspi_master_mutex;

struct neuronspi_file_data
{
	struct spi_device** spi_device;
	struct mutex 		lock;
	u8 					*send_buf;
	u8 					*recv_buf;
	uint32_t			message_len;
};

struct neuronspi_direct_acc
{
	void __iomem		*vaddr;
	u32					size;
};

static struct neuronspi_char_driver neuronspi_cdrv =
{
	.dev = NULL
};

/*************
 * Functions *
 *************/

static int neuronspi_open (struct inode *, struct file *);
static int neuronspi_release (struct inode *, struct file *);
static ssize_t neuronspi_read (struct file *, char *, size_t, loff_t *);
static ssize_t neuronspi_write (struct file *, const char *, size_t, loff_t *);
static int32_t char_register_driver(void);
static int32_t char_unregister_driver(void);

static irqreturn_t neuronspi_spi_irq(int32_t irq, void *dev_id);
static int32_t neuronspi_spi_probe(struct spi_device *spi);
static int32_t neuronspi_spi_remove(struct spi_device *spi);
void neuronspi_spi_send_message(struct spi_device *spi_dev, u8 *send_buf, u8 *recv_buf, int32_t len, int32_t freq, int32_t delay, int32_t send_header);
static uint16_t neuronspi_spi_crc(u8* inputstring, int32_t length, uint16_t initval);
static int32_t neuronspi_spi_uart_write(struct spi_device *spi, u8 *send_buf, u8 length, u8 uart_index);
void neuronspi_spi_uart_read(struct spi_device* spi_dev, u8 *send_buf, u8 *recv_buf, int32_t len, u8 uart_index);
void neuronspi_spi_set_irqs(struct spi_device* spi_dev, uint16_t to);

static void neuronspi_uart_start_tx(struct uart_port *port);
static void neuronspi_uart_stop_tx(struct uart_port *port);
static void neuronspi_uart_stop_rx(struct uart_port *port);
static void neuronspi_uart_set_termios(struct uart_port *port, struct ktermios *termios, struct ktermios *old);
static uint32_t neuronspi_uart_tx_empty(struct uart_port *port);
static void neuronspi_uart_break_ctl(struct uart_port *port, int break_state);
static void neuronspi_uart_shutdown(struct uart_port *port);
static int32_t neuronspi_uart_startup(struct uart_port *port);
static int32_t neuronspi_uart_request_port(struct uart_port *port);
static int32_t neuronspi_uart_alloc_line(void);
static void neuronspi_uart_set_mctrl(struct uart_port *port, uint32_t mctrl);
static uint32_t neuronspi_uart_get_mctrl(struct uart_port *port);
static const char *neuronspi_uart_type(struct uart_port *port);
static void neuronspi_uart_null_void(struct uart_port *port);
static void neuronspi_uart_config_port(struct uart_port *port, int flags);
static int32_t neuronspi_uart_verify_port(struct uart_port *port, struct serial_struct *s);
static void neuronspi_uart_pm(struct uart_port *port, uint32_t state,  uint32_t oldstate);
static int32_t neuronspi_uart_poll(void *data);
static int32_t neuronspi_uart_probe(struct spi_device* dev, u8 device_index);
static int32_t neuronspi_uart_remove(struct neuronspi_uart_data *u_data);
static void neuronspi_uart_power(struct uart_port *port, int32_t on);
static int32_t neuronspi_uart_config_rs485(struct uart_port *port, struct serial_rs485 *rs485);
void neuronspi_spi_uart_set_cflag(struct spi_device* spi_dev, u8 port, uint32_t to);
uint32_t neuronspi_spi_uart_get_cflag(struct spi_device* spi_dev, u8 port);

static void neuronspi_uart_fifo_write(struct neuronspi_port *port, u8 to_send);
static void neuronspi_uart_fifo_read(struct uart_port *port, uint32_t rxlen);

static void neuronspi_uart_rx_proc(struct kthread_work *ws);
static void neuronspi_uart_tx_proc(struct kthread_work *ws);
static void neuronspi_uart_ist(struct kthread_work *ws);
static void neuronspi_uart_handle_tx(struct neuronspi_port *port);
static void neuronspi_uart_handle_rx(struct neuronspi_port *port, uint32_t rxlen, uint32_t iir);
static void neuronspi_uart_handle_irq(struct neuronspi_uart_data *uart_data, int32_t portno);

static struct spinlock* neuronspi_spi_w_spinlock;
static u8 neuronspi_spi_w_flag = 1;
static struct spi_device* neuronspi_s_dev[NEURONSPI_MAX_DEVS];// = { NULL, NULL, NULL };

/***********************
 * Function structures *
 ***********************/

// Host driver struct
static struct spi_driver neuronspi_spi_driver =
{
	.driver =
	{
		.name		= NEURONSPI_NAME,
		.of_match_table	= of_match_ptr(neuronspi_id_match),
	},
	.probe		= neuronspi_spi_probe,
	.remove		= neuronspi_spi_remove,
};

static struct file_operations file_ops =
{
	.open = neuronspi_open,
	.read = neuronspi_read,
	.write = neuronspi_write,
	.release = neuronspi_release
};

static const struct uart_ops neuronspi_uart_ops =
{
	.tx_empty		= neuronspi_uart_tx_empty,
	.set_mctrl		= neuronspi_uart_set_mctrl,
	.get_mctrl		= neuronspi_uart_get_mctrl,
	.stop_tx		= neuronspi_uart_stop_tx,
	.start_tx		= neuronspi_uart_start_tx,
	.stop_rx		= neuronspi_uart_stop_rx,
	.break_ctl		= neuronspi_uart_break_ctl,
	.startup		= neuronspi_uart_startup,
	.shutdown		= neuronspi_uart_shutdown,
	.set_termios	= neuronspi_uart_set_termios,
	.type			= neuronspi_uart_type,
	.request_port	= neuronspi_uart_request_port,
	.release_port	= neuronspi_uart_null_void,
	.config_port	= neuronspi_uart_config_port,
	.verify_port	= neuronspi_uart_verify_port,
	.pm				= neuronspi_uart_pm,
};

// These defines need to be at the end
#define to_neuronspi_uart_data(p,e)  ((container_of((p), struct neuronspi_uart_data, e)))
#define to_neuronspi_port(p,e)	((container_of((p), struct neuronspi_port, e)))
#define to_uart_port(p,e)	((container_of((p), struct uart_port, e)))

/***********************
 * End of Data section *
 ***********************/

static int neuronspi_open (struct inode *inode_p, struct file *file_p)
{
	struct neuronspi_file_data *f_internal_data;
	if (neuronspi_s_dev == NULL || file_p == NULL || inode_p == NULL) {
		return -1;
	}
	neuronspi_cdrv.open_counter += 1;
	f_internal_data = kzalloc(sizeof(*f_internal_data), GFP_KERNEL);
	f_internal_data->recv_buf = kzalloc(NEURONSPI_BUFFER_MAX, GFP_KERNEL);
	f_internal_data->send_buf = kzalloc(NEURONSPI_BUFFER_MAX, GFP_KERNEL);
	f_internal_data->spi_device = neuronspi_s_dev;
	mutex_init(&f_internal_data->lock);
	file_p->private_data = f_internal_data;
	return 0;
}

static int neuronspi_release (struct inode *inode_p, struct file *file_p)
{
	struct neuronspi_file_data *f_internal_data;
	if (file_p == NULL) {
		return -1;
	}
	f_internal_data = (struct neuronspi_file_data*)file_p->private_data;
	f_internal_data->spi_device = NULL;
	kfree(f_internal_data->recv_buf);
	f_internal_data->recv_buf = NULL;
	kfree(f_internal_data->send_buf);
	f_internal_data->send_buf = NULL;
	kfree(f_internal_data);
	file_p->private_data = NULL;
	neuronspi_cdrv.open_counter -= 1;
	return 0;
}

static ssize_t neuronspi_read (struct file *file_p, char *buffer, size_t len, loff_t *offset)
{
    int32_t result = 0;
	u8 device_index = 0;
	uint32_t frequency = NEURONSPI_COMMON_FREQ;
	struct neuronspi_file_data* private_data;
	struct spi_device* spi_driver_data;
	struct neuronspi_driver_data* driver_data;
	// Sanity checking
	if (buffer == NULL) return -7; // Invalid read buffer
    if (len == 0) return result; // Empty read
    if (file_p == NULL) {
    	printk(KERN_DEBUG "NEURONSPI: File Pointer is NULL\n");
    	return -8;
    }
    if (file_p->private_data == NULL) {
    	printk(KERN_DEBUG "NEURONSPI: No Private Data\n");
    	return -1;	// No private data
    }
    private_data = (struct neuronspi_file_data*) file_p->private_data;
    if (private_data == NULL) return -4;
    device_index = private_data->send_buf[0];
    spi_driver_data = private_data->spi_device[device_index];	// Get private (driver) data from FP
    if (spi_driver_data == NULL) return -2;
    driver_data = spi_get_drvdata(spi_driver_data);
    if (driver_data == NULL) return -2;
    if (driver_data->spi_driver == NULL) return -2;	// Invalid private data
    if (driver_data->first_probe_reply[0] == 0) return -3; // No device present
    if (driver_data->slower_model) {
    	frequency = NEURONSPI_SLOWER_FREQ;
    }
    mutex_lock(&(private_data->lock));
    if (private_data->recv_buf == NULL) {
    	mutex_unlock(&(private_data->lock));
    	return -10;
    }
#if NEURONSPI_DETAILED_DEBUG > 0
    printk(KERN_INFO "NEURONSPI: Device read %d DEV:%s%d DRV:%s%d\n", private_data->message_len, (spi_driver_data->dev.of_node->name), (spi_driver_data->chip_select), (driver_data->spi_driver->driver.name), (device_index));
#endif
    if ((((int32_t)len) == private_data->message_len + 10)) {
    	memcpy(buffer, private_data->recv_buf, len);
    	result = len;
    } else {
    	result = simple_read_from_buffer(buffer, len, offset, private_data->recv_buf, len);
    }
    memset(private_data->recv_buf, 0, NEURONSPI_BUFFER_MAX);
	mutex_unlock(&(private_data->lock));
	return result;
}



static ssize_t neuronspi_write (struct file *file_p, const char *buffer, size_t len, loff_t *w_offset)
{
	u8 device_index = 0;
	int32_t result = 0;
	uint32_t frequency = NEURONSPI_COMMON_FREQ;
	int32_t transmit_len = len - NEURONSPI_HEADER_LENGTH;
	int32_t send_header = 0;
	int32_t delay = 25;
	struct neuronspi_file_data* private_data;
	struct spi_device* spi_driver_data;
	struct neuronspi_driver_data* driver_data;
#if NEURONSPI_DETAILED_DEBUG > 0
	printk(KERN_INFO "NEURONSPI: LENGTH:%d\n", len);
#endif
	// Sanity checking
	if (buffer == NULL) {
		return 0; // Void write
	}
    if (len == 0) {
    	return result; // Empty write
    }
    if (file_p == NULL) {
    	return -12;
    }
    if (file_p->private_data == NULL) {
    	printk(KERN_DEBUG "NEURONSPI: No Private Data\n");
    	return -1;	// No private data
    }
    // Read packet header and initialise private data (dependent on each other)
    device_index = buffer[0];
    if (device_index > NEURONSPI_MAX_DEVS - 1) return -2;
    private_data = (struct neuronspi_file_data*) file_p->private_data;
    spi_driver_data = private_data->spi_device[device_index];	// Get private (driver) data from FP
    if (spi_driver_data == NULL) return -2;
    driver_data = spi_get_drvdata(spi_driver_data);
    if (driver_data == NULL) return -2;
    if (driver_data->spi_driver == NULL) return -2;	// Invalid private data
    if (driver_data->first_probe_reply[0] == 0) return -3; // Device not present
    send_header = buffer[3];
    if (buffer[4]) {	// Frequency setting
    	frequency = (buffer[4] << 8 | buffer[5]) * 1000;
    }
    if (buffer[6]) {	// Delay setting
    	delay = buffer[6];
    }
    if (buffer[7]) {	// Device reservation
    	if (buffer[7] == 255) { // Unlock device
    		driver_data->reserved_device = 0;
    	} else if ((driver_data->reserved_device) && buffer[7] != driver_data->reserved_device) {
    		return -4;				// Another device reserved
    	} else if (!driver_data->reserved_device) {
    		driver_data->reserved_device = buffer[7];	// Reserve the device
    	}
#ifdef STRICT_RESERVING
    } else if (driver_data->reserved_device) {
    	return -5;			// Device reserved
    }
    if (driver_data->slower_model) {
    	frequency = NEURONSPI_SLOWER_FREQ;
    }
#else
	} else if (device_index == (driver_data->reserved_device - 1)) {
		return -5;			// Device reserved
	}
	if (driver_data->slower_model) {
		frequency = NEURONSPI_SLOWER_FREQ;
	}
#endif
    mutex_lock(&(private_data->lock));
    memset(private_data->send_buf, 0, NEURONSPI_BUFFER_MAX );
    memcpy(private_data->send_buf, buffer, len);
    memset(private_data->recv_buf, 0, NEURONSPI_BUFFER_MAX );
    private_data->message_len = transmit_len;
    spin_lock(neuronspi_spi_w_spinlock);
    neuronspi_spi_w_flag = 1;
    spin_unlock(neuronspi_spi_w_spinlock);
    neuronspi_spi_send_message(spi_driver_data, &private_data->send_buf[NEURONSPI_HEADER_LENGTH], private_data->recv_buf, transmit_len, frequency, delay, send_header);
    mutex_unlock(&private_data->lock);
    return len;
}

static uint16_t neuronspi_spi_crc(u8* inputstring, int32_t length, uint16_t initval)
{
    int32_t i;
    uint16_t result = initval;
    for (i=0; i<length; i++) {
        result = (result >> 8) ^ NEURONSPI_CRC16TABLE[(result ^ inputstring[i]) & 0xff];
    }
    return result;
}

static int32_t neuronspi_spi_uart_write(struct spi_device *spi, u8 *send_buf, u8 length, u8 uart_index)
{
	u8 *message_buf;
	u8 *recv_buf;
	int32_t transmit_len, i;
	struct neuronspi_driver_data *d_data = spi_get_drvdata(spi);
	uint16_t crc1, crc2;
	uint32_t frequency = NEURONSPI_COMMON_FREQ;
#if NEURONSPI_DETAILED_DEBUG > 0
	printk(KERN_INFO "NEURONSPI: UART SPI Write, dev:%d, len:%d\n", uart_index, length);
#endif
	if (length == 0) {
		return -1;
	}
	if (length == 1) {
		transmit_len = 6;
		message_buf = kzalloc(transmit_len, GFP_KERNEL);
		memcpy(message_buf, NEURONSPI_SPI_UART_SHORT_MESSAGE, NEURONSPI_SPI_UART_SHORT_MESSAGE_LEN);
		message_buf[1] = send_buf[0];
		message_buf[3] = uart_index;
		crc1 = neuronspi_spi_crc(message_buf, 4, 0);
		memcpy(&message_buf[4], &crc1, 2);
	} else {
		transmit_len = 6 + length + 2;
		message_buf = kzalloc(transmit_len, GFP_KERNEL);
		memcpy(message_buf, NEURONSPI_SPI_UART_LONG_MESSAGE, NEURONSPI_SPI_UART_LONG_MESSAGE_LEN);
		message_buf[1] = length;
		message_buf[3] = uart_index;
		crc1 = neuronspi_spi_crc(message_buf, 4, 0);
		memcpy(&message_buf[4], &crc1, 2);
		for (i = 0; i < length; i++) {
			message_buf[6 + i] = send_buf[i];
		}
		crc2 = neuronspi_spi_crc(&message_buf[6], length, crc1);
		memcpy(&message_buf[6+length], &crc2, 2);
	}
	recv_buf = kzalloc(transmit_len, GFP_KERNEL);
	if (d_data->slower_model) {
		frequency = NEURONSPI_SLOWER_FREQ;
	}
	if (!d_data->reserved_device) {
		neuronspi_spi_send_message(spi, message_buf, recv_buf, transmit_len, frequency, 65, 1);
	}
	kfree(message_buf);
	kfree(recv_buf);
	return 0;
}


void neuronspi_spi_uart_read(struct spi_device* spi, u8 *send_buf, u8 *recv_buf, int32_t len, u8 uart_index)
{
	int32_t transmit_len;
	struct neuronspi_driver_data *d_data = spi_get_drvdata(spi);
	uint16_t crc1, crc2;
	int32_t frequency = NEURONSPI_COMMON_FREQ;
	if (d_data->slower_model) {
		frequency = NEURONSPI_SLOWER_FREQ;
	}
#if NEURONSPI_DETAILED_DEBUG > 0
	printk(KERN_INFO "NEURONSPI: UART SPI Read, cs:%d, len:%d\n", uart_index, len);
#endif
	if (len <= 2) {
		memcpy(send_buf, NEURONSPI_SPI_UART_READ_MESSAGE, NEURONSPI_SPI_UART_READ_MESSAGE_LEN);
		transmit_len = NEURONSPI_SPI_UART_READ_MESSAGE_LEN;
	} else {
		memcpy(send_buf, NEURONSPI_SPI_UART_READ_MESSAGE, NEURONSPI_SPI_UART_READ_MESSAGE_LEN);
		if (len < 100) {
			len = (len * 2) + 1;
		} else {
			len = 201;
		}
		transmit_len = 5 + len + 6;	// Header (-1 for the byte there) + 4 bytes in second part + 2 bytes of CRC
		send_buf[1] = len + 3;	// Length of second part (len + 4 - 1)

		crc1 = neuronspi_spi_crc(send_buf, 4, 0);
		memcpy(&send_buf[4], &crc1, 2);
		send_buf[7] = len;
		crc2 = neuronspi_spi_crc(&send_buf[6], len + 3, crc1);
		memcpy(&send_buf[len + 9], &crc2, 2);
#if NEURONSPI_DETAILED_DEBUG > 0
		printk(KERN_INFO "NEURONSPI: UART Device Read len:%d %100ph\n", transmit_len, send_buf);
#endif
	}
	if (!d_data->reserved_device) {
		neuronspi_spi_send_message(spi, send_buf, recv_buf, transmit_len, frequency, 65, 1);
	}
}

void neuronspi_spi_set_irqs(struct spi_device* spi_dev, uint16_t to)
{
	u8 *message_buf;
	u8 *recv_buf;
	uint16_t crc1, crc2;
	struct neuronspi_driver_data *d_data = spi_get_drvdata(spi_dev);
	int32_t frequency = NEURONSPI_COMMON_FREQ;
	if (d_data->slower_model) {
		frequency = NEURONSPI_SLOWER_FREQ;
	}
#if NEURONSPI_DETAILED_DEBUG > 0
	printk(KERN_INFO "NEURONSPI: SPI IRQ Set, Dev-CS:%d, to:%d\n", spi_dev->chip_select, to);
#endif
	message_buf = kzalloc(NEURONSPI_SPI_IRQ_SET_MESSAGE_LEN, GFP_KERNEL);
	recv_buf = kzalloc(NEURONSPI_SPI_IRQ_SET_MESSAGE_LEN, GFP_KERNEL);
	memcpy(message_buf, NEURONSPI_SPI_IRQ_SET_MESSAGE, NEURONSPI_SPI_IRQ_SET_MESSAGE_LEN);
	crc1 = neuronspi_spi_crc(message_buf, 4, 0);
	memcpy(&message_buf[4], &crc1, 2);
	crc2 = neuronspi_spi_crc(&message_buf[6], NEURONSPI_SPI_IRQ_SET_MESSAGE_LEN - 8, crc1);
	memcpy(&message_buf[NEURONSPI_SPI_IRQ_SET_MESSAGE_LEN - 2], &crc2, 2);
	if (!d_data->reserved_device) {
		neuronspi_spi_send_message(spi_dev, message_buf, recv_buf, NEURONSPI_SPI_IRQ_SET_MESSAGE_LEN, frequency, 65, 1);
	}
	kfree(message_buf);
	kfree(recv_buf);
}

void neuronspi_spi_uart_set_cflag(struct spi_device* spi_dev, u8 port, uint32_t to)
{
	u8 *message_buf;
	u8 *recv_buf;
	uint16_t crc1, crc2;
	struct neuronspi_driver_data *d_data = spi_get_drvdata(spi_dev);
	int32_t frequency = NEURONSPI_COMMON_FREQ;
	if (d_data->slower_model) {
		frequency = NEURONSPI_SLOWER_FREQ;
	}
#if NEURONSPI_DETAILED_DEBUG > 0
	printk(KERN_INFO "NEURONSPI: SPI TERMIOS Set, Dev-CS:%d, to:%x\n", spi_dev->chip_select, to);
#endif
	message_buf = kzalloc(NEURONSPI_SPI_UART_SET_CFLAG_MESSAGE_LEN, GFP_KERNEL);
	recv_buf = kzalloc(NEURONSPI_SPI_UART_SET_CFLAG_MESSAGE_LEN, GFP_KERNEL);
	memcpy(message_buf, NEURONSPI_SPI_UART_SET_CFLAG_MESSAGE, NEURONSPI_SPI_UART_SET_CFLAG_MESSAGE_LEN);
	crc1 = neuronspi_spi_crc(message_buf, 4, 0);
	memcpy(&message_buf[4], &crc1, 2);
	memcpy(&message_buf[10], &to, 4);
	crc2 = neuronspi_spi_crc(&message_buf[6], NEURONSPI_SPI_UART_SET_CFLAG_MESSAGE_LEN - 8, crc1);
	memcpy(&message_buf[NEURONSPI_SPI_UART_SET_CFLAG_MESSAGE_LEN - 2], &crc2, 2);
	if (!d_data->reserved_device) {
		neuronspi_spi_send_message(spi_dev, message_buf, recv_buf, NEURONSPI_SPI_UART_SET_CFLAG_MESSAGE_LEN, frequency, 65, 1);
	}
	kfree(message_buf);
	kfree(recv_buf);
}

static int32_t neuronspi_spi_watchdog(void *data)
{
	int32_t *cycle = (int32_t *) data;
	while (!kthread_should_stop()) {
		msleep(*cycle);
		spin_lock(neuronspi_spi_w_spinlock);
		if (neuronspi_spi_w_flag == 0) {
			panic_timeout = -1;
			panic("SPI Watchdog Failure\n");
		} else {
			neuronspi_spi_w_flag = 0;
		}
		spin_unlock(neuronspi_spi_w_spinlock);
	}
	return 0;
}

uint32_t neuronspi_spi_uart_get_cflag(struct spi_device* spi_dev, u8 port)
{
	u8 *message_buf;
	u8 *recv_buf;
	uint16_t crc1, crc2, ret;
	struct neuronspi_driver_data *d_data = spi_get_drvdata(spi_dev);
	int32_t frequency = NEURONSPI_COMMON_FREQ;
	if (d_data->slower_model) {
		frequency = NEURONSPI_SLOWER_FREQ;
	}
	message_buf = kzalloc(NEURONSPI_SPI_UART_GET_CFLAG_MESSAGE_LEN, GFP_KERNEL);
	recv_buf = kzalloc(NEURONSPI_SPI_UART_GET_CFLAG_MESSAGE_LEN, GFP_KERNEL);
	memcpy(message_buf, NEURONSPI_SPI_UART_GET_CFLAG_MESSAGE, NEURONSPI_SPI_UART_GET_CFLAG_MESSAGE_LEN);
	crc1 = neuronspi_spi_crc(message_buf, 4, 0);
	memcpy(&message_buf[4], &crc1, 2);
	crc2 = neuronspi_spi_crc(&message_buf[6], NEURONSPI_SPI_UART_GET_CFLAG_MESSAGE_LEN - 8, crc1);
	memcpy(&message_buf[NEURONSPI_SPI_UART_GET_CFLAG_MESSAGE_LEN - 2], &crc2, 2);
	if (!d_data->reserved_device) {
		neuronspi_spi_send_message(spi_dev, message_buf, recv_buf, NEURONSPI_SPI_UART_GET_CFLAG_MESSAGE_LEN, frequency, 65, 1);
	}
	ret = ((uint32_t*)recv_buf)[5];
#if NEURONSPI_DETAILED_DEBUG > 0
	printk(KERN_INFO "NEURONSPI: SPI TERMIOS Get, Dev-CS:%d, val:%x\n", spi_dev->chip_select, ret);
#endif
	kfree(message_buf);
	kfree(recv_buf);
	return ret;
}


void neuronspi_spi_send_message(struct spi_device* spi_dev, u8 *send_buf, u8 *recv_buf, int32_t len, int32_t freq, int32_t delay, int32_t send_header)
{
	int32_t i = 0;
	uint16_t recv_crc1 = 0;
	uint16_t recv_crc2 = 0;
	uint16_t packet_crc = 0;
	int32_t trans_count = (len / NEURONSPI_MAX_TX) + 3;	// number of transmissions
	struct spi_message s_msg;
	struct neuronspi_driver_data *d_data;
    struct spi_transfer* s_trans;
	mutex_lock(&neuronspi_master_mutex);
    s_trans = kzalloc(sizeof(struct spi_transfer) * trans_count, GFP_KERNEL);
#if NEURONSPI_DETAILED_DEBUG > 1
    printk(KERN_INFO "NEURONSPI: SPI Master Write, len:%d,\n %100ph\n", len, send_buf);
#endif
	if (!send_header) {
		trans_count -= 1;	// one less transmission as the header is omitted
	}
    spi_message_init(&s_msg);
    for (i = 0; i < trans_count; i++) {
    	memset(&(s_trans[i]), 0, sizeof(s_trans[i]));
    	s_trans[i].delay_usecs = 0;
    	s_trans[i].bits_per_word = 8;
    	s_trans[i].speed_hz = freq;
    	if (i == 0) {
    		s_trans[i].delay_usecs = NEURONSPI_EDGE_DELAY;
    	} else if (i == 1) {
		    s_trans[i].tx_buf = send_buf;
		    s_trans[i].rx_buf = recv_buf;
    		if (send_header) {
    		    s_trans[i].delay_usecs = delay;
    		    s_trans[i].len = NEURONSPI_FIRST_MESSAGE_LENGTH;
    		} else {
    			// If len is more than NEURONSPI_MAX_TX * i, then chunk len is NEURONSPI_MAX_TX, otherwise it's the remainder
    			s_trans[i].len = (len - (NEURONSPI_MAX_TX * i)) > 0 ? NEURONSPI_MAX_TX : len;
    		}
    	} else {
    		if (send_header) {
    			s_trans[i].tx_buf = &(send_buf[NEURONSPI_FIRST_MESSAGE_LENGTH + (NEURONSPI_MAX_TX * (i - 2))]);
    			s_trans[i].rx_buf = &(recv_buf[NEURONSPI_FIRST_MESSAGE_LENGTH + (NEURONSPI_MAX_TX * (i - 2))]);
    			s_trans[i].len = ((NEURONSPI_MAX_TX * (i - 1)) - len) < 0 ? NEURONSPI_MAX_TX : (len - (NEURONSPI_MAX_TX * (i - 2)));
    		} else {
    			s_trans[i].tx_buf = &(send_buf[NEURONSPI_MAX_TX * (i - 1)]);
    			s_trans[i].rx_buf = &(recv_buf[NEURONSPI_MAX_TX * (i - 1)]);
    			s_trans[i].len = ((NEURONSPI_MAX_TX * i) - len) < 0 ? NEURONSPI_MAX_TX : (len - (NEURONSPI_MAX_TX * (i - 1)));
    		}
    		// If len is more than NEURONSPI_MAX_TX * i (+ optionally header), then chunk len is NEURONSPI_MAX_TX (+ optionally header), otherwise it's the remainder
    	}
    	spi_message_add_tail(&(s_trans[i]), &s_msg);
    }
    spi_sync(spi_dev, &s_msg);
    for (i = 0; i < trans_count; i++) {
    	spi_transfer_del(&(s_trans[i]));
    }
#if NEURONSPI_DETAILED_DEBUG > 1
    printk(KERN_INFO "NEURONSPI: SPI Master Read - %d:\n\t%100ph\n\t%100ph\n\t%100ph\n\t%100ph\n", len,recv_buf, &recv_buf[64], &recv_buf[128], &recv_buf[192]);
#endif
    d_data = spi_get_drvdata(spi_dev);
    if (d_data != NULL && !d_data->reserved_device) {
		recv_crc1 = neuronspi_spi_crc(recv_buf, 4, 0);
		memcpy(&packet_crc, &recv_buf[4], 2);
#if NEURONSPI_DETAILED_DEBUG > 1
		printk(KERN_INFO "NEURONSPI: SPI CRC1: %x\t COMPUTED CRC1:%x\n", packet_crc, recv_crc1);
#endif
		if (recv_crc1 == packet_crc) {
		// Signal the UART to issue character reads
#if NEURONSPI_DETAILED_DEBUG > 1
		printk(KERN_INFO "NEURONSPI: SPI CRC1 Correct");
#endif
			if (recv_buf[0] == 0x41) {
				d_data->uart_buf[0] = recv_buf[3];
#if NEURONSPI_DETAILED_DEBUG > 0
				printk(KERN_INFO "NEURONSPI: Reading UART data for device %d\n", d_data->neuron_index);
#endif
				for (i = 0; i < d_data->uart_data->p_count; i++) {
					if (d_data->uart_data->p[i].dev_index == d_data->neuron_index) {
						neuronspi_uart_handle_rx(&d_data->uart_data->p[i], 1, 1);
					}
				}
				if (!(d_data->uart_read) && (d_data->uart_count)) {
					d_data->uart_read = recv_buf[2];
					for (i = 0; i < d_data->uart_data->p_count; i++) {
#if NEURONSPI_DETAILED_DEBUG > 0
					printk(KERN_INFO "NEURONSPI: UART Buffer:%d, UART Local Port Count:%d, UART Global Port Count:%d\n", d_data->uart_read, d_data->uart_count,  d_data->uart_data->p_count);
#endif
						if (d_data->uart_data->p[i].dev_index == d_data->neuron_index && !d_data->reserved_device) {
							kthread_queue_work(&d_data->uart_data->kworker, &d_data->uart_data->p[i].rx_work);
						}
					}
				}
			}
		}
#if NEURONSPI_DETAILED_DEBUG > 0
		else {
			printk(KERN_INFO "NEURONSPI: SPI CRC1 Not Correct");
		}
#endif
		recv_crc2 = neuronspi_spi_crc(&recv_buf[6], len - 8, recv_crc1);
		memcpy(&packet_crc, &recv_buf[len - 2], 2);
#if NEURONSPI_DETAILED_DEBUG > 1
		printk(KERN_INFO "NEURONSPI: SPI CRC2: %x\t COMPUTED CRC2:%x\n", packet_crc, recv_crc2);
#endif
		if (recv_crc2 != packet_crc) {
#if NEURONSPI_DETAILED_DEBUG > 0
			printk(KERN_INFO "NEURONSPI: SPI CRC2 Not Correct");
#endif
			recv_buf[0] = 0;
		}
    } else {

    }
    mutex_unlock(&neuronspi_master_mutex);
    kfree(s_trans);
}

static void neuronspi_uart_fifo_read(struct uart_port *port, uint32_t rxlen)
{
	struct neuronspi_port *s = to_neuronspi_port(port,port);
	struct neuronspi_driver_data *d_data = spi_get_drvdata(neuronspi_s_dev[s->dev_index]);
#if NEURONSPI_DETAILED_DEBUG > 0
	printk(KERN_INFO "NEURONSPI: FIFO Read len:%d\n", rxlen);
#endif
    memcpy(s->buf, d_data->uart_buf, rxlen);
}

static void neuronspi_uart_fifo_write(struct neuronspi_port *port, u8 to_send)
{
	int32_t i;
#if NEURONSPI_DETAILED_DEBUG > 0
	printk(KERN_INFO "NEURONSPI: FIFO Write to_send:%d\n", to_send);
#endif
	for (i = 0; i < to_send; i++) {
#if NEURONSPI_DETAILED_DEBUG > 0
		printk(KERN_INFO "NEURONSPI: UART Char Send: %x\n", port->buf[i]);
#endif
	}
    neuronspi_spi_uart_write(neuronspi_s_dev[port->dev_index], port->buf, to_send, port->dev_port);
}

static int32_t neuronspi_uart_alloc_line(void)
{
	int32_t i;
	BUILD_BUG_ON(NEURONSPI_MAX_DEVS > BITS_PER_LONG);

	for (i = 0; i < NEURONSPI_MAX_DEVS; i++)
		if (!test_and_set_bit(i, &neuronspi_lines))
			break;

	return i;
}

static void neuronspi_uart_power(struct uart_port *port, int32_t on)
{
    /* Do nothing */
}

static void neuronspi_uart_handle_rx(struct neuronspi_port *port, uint32_t rxlen, uint32_t iir)
{
	uint32_t ch, flag, bytes_read, i;
	while (rxlen) {

		neuronspi_uart_fifo_read(&port->port, rxlen);
		bytes_read = rxlen;

		port->port.icount.rx++;
		flag = TTY_NORMAL;

		for (i = 0; i < bytes_read; ++i) {
#if NEURONSPI_DETAILED_DEBUG > 0
			printk(KERN_INFO "NEURONSPI: UART Insert Char:%x\n", port->buf[i]);
#endif
			ch = port->buf[i];
			if (uart_handle_sysrq_char(port, ch))
				continue;

			uart_insert_char(&port->port, 0, 0, ch, flag);
		}
		rxlen -= bytes_read;
	}

	tty_flip_buffer_push(&port->port.state->port);
}

static void neuronspi_uart_handle_tx(struct neuronspi_port *port)
{
	uint32_t txlen, to_send, i;
	struct spi_device *spi;
	struct neuronspi_driver_data *d_data;
	struct circ_buf *xmit;

	spi = neuronspi_s_dev[port->dev_index];
	d_data = spi_get_drvdata(spi);
	xmit = &port->port.state->xmit;

	if (unlikely(port->port.x_char)) {
		neuronspi_spi_uart_write(spi, &port->port.x_char, 1, port->dev_port);
		port->port.icount.tx++;
		port->port.x_char = 0;
		return;
	}

	if (uart_circ_empty(xmit) || uart_tx_stopped(&port->port)) {
		return;
	}


	/* Get length of data pending in circular buffer */
	to_send = uart_circ_chars_pending(xmit);
	if (likely(to_send)) {
		/* Limit to size of TX FIFO */
		txlen = NEURONSPI_FIFO_SIZE;
		to_send = (to_send > txlen) ? txlen : to_send;

		/* Add data to send */
		port->port.icount.tx += to_send;

		/* Convert to linear buffer */
		for (i = 0; i < to_send; ++i) {
			port->buf[i] = xmit->buf[xmit->tail];
			xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE - 1);
		}

		neuronspi_uart_fifo_write(port, to_send);
	}


	if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
		uart_write_wakeup(&port->port);
}

static void neuronspi_uart_handle_irq(struct neuronspi_uart_data *uart_data, int32_t portno)
{
	struct neuronspi_port *n_port = &uart_data->p[portno];
	struct spi_device *spi = neuronspi_s_dev[n_port->dev_index];

	u8 *send_buf = kzalloc(NEURONSPI_UART_PROBE_MESSAGE_LEN, GFP_KERNEL);
	u8 *recv_buf = kzalloc(NEURONSPI_UART_PROBE_MESSAGE_LEN, GFP_KERNEL);
	memcpy(send_buf, NEURONSPI_UART_PROBE_MESSAGE, NEURONSPI_UART_PROBE_MESSAGE_LEN);
	neuronspi_spi_send_message(spi, send_buf, recv_buf, NEURONSPI_UART_PROBE_MESSAGE_LEN, NEURONSPI_DEFAULT_FREQ, 25, 1);
	kfree(send_buf);
	kfree(recv_buf);
}

static void neuronspi_uart_ist(struct kthread_work *ws)
{
	struct neuronspi_port *p = to_neuronspi_port(ws, irq_work);
	neuronspi_uart_handle_irq(p->parent, p->line);
}

static irqreturn_t neuronspi_spi_irq(int32_t irq, void *dev_id)
{
	int32_t i;
	struct spi_device *spi;
	struct neuronspi_driver_data *d_data;
	struct neuronspi_uart_data *u_data;
	spi = (struct spi_device *)dev_id;
	d_data = spi_get_drvdata(spi);
#if NEURONSPI_DETAILED_DEBUG > 0
	printk(KERN_INFO "NEURONSPI: SPI IRQ\n");
#endif
	if (d_data->uart_count) {
		u_data = d_data->uart_data;
		for (i = 0; i < u_data->p_count; i++) {
			if (u_data->p[i].dev_index == d_data->neuron_index) {
				kthread_queue_work(&u_data->kworker, &u_data->p[i].irq_work);
			}

		}
	}
	return IRQ_HANDLED;
}

static void neuronspi_uart_tx_proc(struct kthread_work *ws)
{
	struct neuronspi_port *port = to_neuronspi_port(ws, tx_work);

	if ((port->port.rs485.flags & SER_RS485_ENABLED) &&
	    (port->port.rs485.delay_rts_before_send > 0)) {
		msleep(port->port.rs485.delay_rts_before_send);
	}

	neuronspi_uart_handle_tx(port);
}


static void neuronspi_uart_rx_proc(struct kthread_work *ws)
{
	int32_t end_flag = 0;
	int32_t read_count = 0;
	struct neuronspi_port *n_port = to_neuronspi_port(ws, rx_work);
	struct spi_device *spi = neuronspi_s_dev[n_port->dev_index];
	struct neuronspi_driver_data *d_data = spi_get_drvdata(spi);

	u8 *send_buf = kzalloc(NEURONSPI_BUFFER_MAX, GFP_KERNEL);
	u8 *recv_buf = kzalloc(NEURONSPI_BUFFER_MAX, GFP_KERNEL);

	mutex_lock(&neuronspi_master_mutex);
	read_count = d_data->uart_read;
	mutex_unlock(&neuronspi_master_mutex);

	while (!end_flag) {
		memset(recv_buf, 0, NEURONSPI_BUFFER_MAX);
		neuronspi_spi_uart_read(spi, send_buf, recv_buf, read_count, n_port->dev_port);

		if (recv_buf[6] == 0x65 && recv_buf[7] > 0) {
			mutex_lock(&neuronspi_master_mutex);
			memcpy(&d_data->uart_buf[0], &recv_buf[10], recv_buf[7]);
			neuronspi_uart_handle_rx(n_port, recv_buf[7], 1);
			read_count = recv_buf[9];
			mutex_unlock(&neuronspi_master_mutex);
		} else if (recv_buf[0] != 0x41) {
			mutex_lock(&neuronspi_master_mutex);
			d_data->uart_read = 0;
			end_flag = 1;
			mutex_unlock(&neuronspi_master_mutex);
		}
	}
	kfree(recv_buf);
	kfree(send_buf);
}


static void neuronspi_uart_stop_tx(struct uart_port *port)
{
    // ToDo : create new opcode / coil?
}

static void neuronspi_uart_stop_rx(struct uart_port *port)
{
    // ToDo : create new opcode / coil?
}

static void neuronspi_uart_start_tx(struct uart_port *port)
{
	struct neuronspi_port *n_port = to_neuronspi_port(port,port);
#if NEURONSPI_DETAILED_DEBUG > 0
	printk(KERN_INFO "NEURONSPI: Start TX\n");
#endif
	kthread_queue_work(&n_port->parent->kworker, &n_port->tx_work);
}

static int32_t neuronspi_uart_poll(void *data)
{
	struct neuronspi_driver_data *d_data = (struct neuronspi_driver_data*) data;
	struct neuronspi_uart_data *u_data;
	int32_t i;
	while (!kthread_should_stop()) {
		usleep_range(2000,8000);
		if (d_data->uart_count) {
			u_data = d_data->uart_data;
			for (i = 0; i < u_data->p_count; i++) {
				if (u_data->p[i].dev_index == d_data->neuron_index) {
					kthread_queue_work(&u_data->kworker, &u_data->p[i].irq_work);
				}

			}
		}
	}
	return 0;
}

static uint32_t neuronspi_uart_tx_empty(struct uart_port *port)
{
#if NEURONSPI_DETAILED_DEBUG > 0
	printk(KERN_INFO "NEURONSPI: UART TX Empty\n");
#endif

	return TIOCSER_TEMT;
}

static uint32_t neuronspi_uart_get_mctrl(struct uart_port *port)
{
	/* DCD and DSR are not wired and CTS/RTS is handled automatically
	 * so just indicate DSR and CAR asserted
	 */
#if NEURONSPI_DETAILED_DEBUG > 0
	printk(KERN_DEBUG "NEURONSPI: UART MCTRL Get\n");
#endif
	return TIOCM_DSR | TIOCM_CAR;
}

static void neuronspi_uart_set_mctrl(struct uart_port *port, uint32_t mctrl)
{
}

static void neuronspi_uart_break_ctl(struct uart_port *port, int break_state)
{

}

static void neuronspi_uart_set_termios(struct uart_port *port, struct ktermios *termios, struct ktermios *old)
{
	int32_t baud;
	struct neuronspi_port *n_port;
	n_port = to_neuronspi_port(port, port);
#if NEURONSPI_DETAILED_DEBUG > 0
	printk(KERN_DEBUG "NEURONSPI: TERMIOS Set, p:%d, c_cflag:%x", port->line, termios->c_cflag);
#endif
	neuronspi_spi_uart_set_cflag(neuronspi_s_dev[n_port->dev_index], n_port->dev_port, termios->c_cflag);

	baud = uart_get_baud_rate(port, termios, old, 2400, 115200);
	uart_update_timeout(port, termios->c_cflag, baud);
}

static int32_t neuronspi_uart_config_rs485(struct uart_port *port, struct serial_rs485 *rs485)
{
	port->rs485 = *rs485;
	return 0;
}

// Initialise the module
static int32_t neuronspi_uart_startup(struct uart_port *port)
{
	struct neuronspi_port *n_port = to_neuronspi_port(port, port);

	struct spi_device *spi = neuronspi_s_dev[n_port->dev_index];
	struct neuronspi_driver_data *d_data = spi_get_drvdata(spi);
	neuronspi_spi_set_irqs(spi, 0x5);
	if (d_data->poll_thread != NULL) {
		wake_up_process(d_data->poll_thread);
	} else if (d_data->no_irq) {
		d_data->poll_thread = kthread_create(neuronspi_uart_poll, (void *)d_data, "UART_poll_thread");
	}
	neuronspi_uart_power(port, 1);

	// TODO: /* Reset FIFOs*/
#if NEURONSPI_DETAILED_DEBUG > 0
	printk(KERN_DEBUG "NEURONSPI: UART Startup\n");
#endif
	return 0;
}

static void neuronspi_uart_shutdown(struct uart_port *port)
{
    neuronspi_uart_power(port, 0);
}

static const char* neuronspi_uart_type(struct uart_port *port)
{
	return port->type == PORT_NEURONSPI ? "NEURONSPI_NAME" : NULL;
}

static int32_t neuronspi_uart_request_port(struct uart_port *port)
{
#if NEURONSPI_DETAILED_DEBUG > 0
	printk(KERN_DEBUG "NEURONSPI: Requested port %d\n", port->line);
#endif
	return 0;
}

static void neuronspi_uart_config_port(struct uart_port *port, int flags)
{
	if (flags & UART_CONFIG_TYPE)
		port->type = PORT_NEURONSPI;
}

static int32_t neuronspi_uart_verify_port(struct uart_port *port,
				 struct serial_struct *s)
{
	if ((s->type != PORT_UNKNOWN) && (s->type != PORT_NEURONSPI))
		return -EINVAL;
	if (s->irq != port->irq)
		return -EINVAL;

	return 0;
}

static void neuronspi_uart_pm(struct uart_port *port, uint32_t state, uint32_t oldstate)
{
	neuronspi_uart_power(port, (state == UART_PM_STATE_ON) ? 1 : 0);
}

static void neuronspi_uart_null_void(struct uart_port *port)
{
	/* Do nothing */
}

static int32_t neuronspi_uart_probe(struct spi_device* dev, u8 device_index)
{
	struct neuronspi_driver_data* driver_data = spi_get_drvdata(dev);
	struct sched_param sched_param = { .sched_priority = MAX_RT_PRIO / 2 };
	int32_t i, j, ret, new_uart_count;
	struct neuronspi_uart_data *uart_data = driver_data->uart_data;

	if (uart_data->p == NULL) {
		uart_data->p = kzalloc(sizeof(struct neuronspi_port[NEURONSPI_MAX_UART]), GFP_KERNEL);
		for (i = 0; i < NEURONSPI_MAX_UART; i++) {
			uart_data->p[i].parent = uart_data;
		}
#if NEURONSPI_DETAILED_DEBUG > 0
		printk(KERN_DEBUG "NEURONSPI: Allocated port structure for %d potential UART devices\n", NEURONSPI_MAX_UART);
#endif
	}

	new_uart_count = driver_data->uart_count + uart_data->p_count;

	// Initialise port data
	for (i = uart_data->p_count; i < new_uart_count; i++) {
		uart_data->p[i].dev_index = device_index;
		uart_data->p[i].dev_port = i - uart_data->p_count;
		uart_data->p[i].line		= i;
		uart_data->p[i].port.dev	= &(dev->dev);
		uart_data->p[i].port.irq	= dev->irq;
		uart_data->p[i].port.type	= PORT_SC16IS7XX;
		uart_data->p[i].port.fifosize	= NEURONSPI_FIFO_SIZE;
		uart_data->p[i].port.flags	= UPF_FIXED_TYPE | UPF_LOW_LATENCY;
		uart_data->p[i].port.iotype	= UPIO_PORT;
		uart_data->p[i].port.uartclk	= 9800;
		uart_data->p[i].port.rs485_config = neuronspi_uart_config_rs485;
		uart_data->p[i].port.ops	= &neuronspi_uart_ops;
		uart_data->p[i].port.line	= neuronspi_uart_alloc_line();
		spin_lock_init(&uart_data->p[i].port.lock);
		if (uart_data->p[i].port.line >= NEURONSPI_MAX_DEVS) {
			ret = -ENOMEM;
		}
		kthread_init_work(&(uart_data->p[i].tx_work), neuronspi_uart_tx_proc);
		kthread_init_work(&(uart_data->p[i].rx_work), neuronspi_uart_rx_proc);
		kthread_init_work(&(uart_data->p[i].irq_work), neuronspi_uart_ist);
		uart_add_one_port(driver_data->serial_driver, &uart_data->p[i].port);
		printk(KERN_INFO "NEURONSPI: Added UART port %d\n", i);
	}

	// For ports on multiple SPI devices renumber the ports to correspond to SPI chip-select numbering
	if (uart_data->p_count) {
		// First remove all existing ports
		for (i = 0; i < new_uart_count; i++) {
			uart_remove_one_port(driver_data->serial_driver, &uart_data->p[i].port);
			clear_bit(uart_data->p[i].port.line, &neuronspi_lines);
			kthread_flush_worker(&uart_data->kworker);
		}
		// Now add the ports in the correct order
		for (i = 0; i < NEURONSPI_MAX_DEVS; i++) {
			if (neuronspi_s_dev[i] != NULL) {
				driver_data = spi_get_drvdata(neuronspi_s_dev[i]);
#if NEURONSPI_DETAILED_DEBUG > 0
				printk(KERN_DEBUG "NEURONSPI: Renumber not NULL %d UC:%d\n", i, driver_data->uart_count);
#endif
				if (driver_data->uart_count) {
					for (j = 0; j < new_uart_count; j++) {
						if (uart_data->p[j].dev_index == i) {
							uart_data->p[j].port.dev	= &(neuronspi_s_dev[i]->dev);
							uart_data->p[j].port.irq	= neuronspi_s_dev[i]->irq;
							uart_data->p[j].port.type	= PORT_SC16IS7XX;
							uart_data->p[j].port.fifosize	= NEURONSPI_FIFO_SIZE;
							uart_data->p[j].port.flags	= UPF_FIXED_TYPE | UPF_LOW_LATENCY;
							uart_data->p[j].port.iotype	= UPIO_PORT;
							uart_data->p[j].port.uartclk	= 9800;
							uart_data->p[j].port.rs485_config = neuronspi_uart_config_rs485;
							uart_data->p[j].port.ops	= &neuronspi_uart_ops;
							uart_data->p[j].port.line	= neuronspi_uart_alloc_line();
							uart_add_one_port(driver_data->serial_driver, &uart_data->p[j].port);
#if NEURONSPI_DETAILED_DEBUG > 0
							printk(KERN_DEBUG "NEURONSPI: Added UART port %d\n", j);
#endif
						}
					}
				}
			}
		}
	}

	uart_data->p_count = new_uart_count;
	if (uart_data->kworker_task == NULL) {
#if NEURONSPI_DETAILED_DEBUG > 0
		printk(KERN_DEBUG "NEURONSPI: KWorker Task is NULL\n");
#endif

		kthread_init_worker(&uart_data->kworker);

		uart_data->kworker_task = kthread_run(kthread_worker_fn, &uart_data->kworker,
						  "neuronspi");
		if (IS_ERR(uart_data->kworker_task)) {
			ret = PTR_ERR(uart_data->kworker_task);
		}
		sched_setscheduler(uart_data->kworker_task, SCHED_FIFO, &sched_param);
	}
	return ret;
}


static int32_t neuronspi_uart_remove(struct neuronspi_uart_data *u_data)
{
	struct neuronspi_driver_data *d_data;
	struct spi_device *spi;
	int32_t i;

	for (i = 0; i < NEURONSPI_MAX_DEVS; i++) {
		if (!(neuronspi_s_dev[i] == NULL)) {
			spi = neuronspi_s_dev[i];
			d_data = spi_get_drvdata(spi);
			if (d_data->poll_thread != NULL) {
				kthread_stop(d_data->poll_thread);
			}
		}
	}
	for (i = 0; i < u_data->p_count; i++) {
		uart_remove_one_port(neuronspi_uart, &u_data->p[i].port);
		clear_bit(u_data->p[i].port.line, &neuronspi_lines);
		neuronspi_uart_power(&u_data->p[i].port, 0);
	}

	kthread_flush_worker(&u_data->kworker);
	return 0;
}

static int32_t neuronspi_spi_probe(struct spi_device *spi)
{
	const struct neuronspi_devtype *devtype;
	struct neuronspi_driver_data *n_spi;
	int32_t ret, i, index, no_irq = 0;
	u8 uart_count = 0;
	n_spi = kzalloc(sizeof *n_spi, GFP_KERNEL);
	if (!n_spi)
		return -ENOMEM;
	printk(KERN_INFO "NEURONSPI: Neuronspi Probe Started\n");
	if (n_spi == NULL || spi == NULL) {
		return -8;
	}
#if NEURONSPI_DETAILED_DEBUG > 0
	printk(KERN_DEBUG "NEURONSPI: Chip Max Hz-%d\n",spi->master->max_speed_hz);
#endif
	/* Setup SPI bus */
	spi->bits_per_word	= 8;
	spi->mode		= spi->mode ? : SPI_MODE_0;
	spi->max_speed_hz	= spi->max_speed_hz ? : 12000000;
	ret = spi_setup(spi);
	n_spi->neuron_index = spi->chip_select - 1;
	n_spi->reserved_device = 0;
	if (ret)
		return ret;
#if NEURONSPI_DETAILED_DEBUG > 0
	printk(KERN_DEBUG "NEURONSPI: Chip Max Hz-%d %d\n", spi->master->max_speed_hz, spi->max_speed_hz);
#endif
	if (spi->dev.of_node) {
		const struct of_device_id *of_id =
			of_match_device(neuronspi_id_match, &spi->dev);
		if (!of_id) {
			printk(KERN_DEBUG "NEURONSPI: Probe %s does not match a device!\n", *(&spi->dev.of_node->full_name));
			return -ENODEV;
		}
		of_property_read_u32_array(spi->dev.of_node, "neuron-board-index", &(n_spi->neuron_index), 1);
		//print_device_tree_node(spi->dev.of_node->parent, 0);
		devtype = (struct neuronspi_devtype *)of_id->data;
#if NEURONSPI_DETAILED_DEBUG > 0
		printk(KERN_INFO "DEVICE TREE NODE FOUND %d\n", n_spi->neuron_index);
#endif
	} else {
		const struct spi_device_id *id_entry = spi_get_device_id(spi);
		devtype = (struct neuronspi_devtype *)id_entry->driver_data;
	}

	// We perform an initial probe of registers 1000-1004 to identify the device, using a premade message
	n_spi->recv_buf = kzalloc(NEURONSPI_BUFFER_MAX, GFP_KERNEL);
	n_spi->send_buf = kzalloc(NEURONSPI_BUFFER_MAX, GFP_KERNEL);
	n_spi->first_probe_reply = kzalloc(NEURONSPI_PROBE_MESSAGE_LEN, GFP_KERNEL);	// allocate space for initial probe
	n_spi->second_probe_reply = kzalloc(NEURONSPI_PROBE_MESSAGE_LEN, GFP_KERNEL); // allocate space for uart probe
	n_spi->spi_driver = &neuronspi_spi_driver;

	memcpy(n_spi->send_buf, &NEURONSPI_PROBE_MESSAGE, NEURONSPI_PROBE_MESSAGE_LEN);
	neuronspi_spi_send_message(spi, n_spi->send_buf, n_spi->first_probe_reply, NEURONSPI_PROBE_MESSAGE_LEN, NEURONSPI_DEFAULT_FREQ, 25, 1);

	// Throw away the first message - the associated SPI Master is sometimes not properly initialised at this point
	memcpy(n_spi->send_buf, &NEURONSPI_PROBE_MESSAGE, NEURONSPI_PROBE_MESSAGE_LEN);
	memset(n_spi->first_probe_reply, 0, NEURONSPI_PROBE_MESSAGE_LEN);
	neuronspi_spi_send_message(spi, n_spi->send_buf, n_spi->first_probe_reply, NEURONSPI_PROBE_MESSAGE_LEN, NEURONSPI_DEFAULT_FREQ, 25, 1);

	if (n_spi->first_probe_reply[0] != 0) {
		uart_count = n_spi->first_probe_reply[14] & 0x0f;
		printk(KERN_INFO "NEURONSPI: Probe detected Neuron Board %d v%d.%d on CS %d, Uart count: %d - reg1000: %x, reg1001: %x, reg1002: %x, reg1003: %x, reg1004: %x\n",  n_spi->neuron_index, n_spi->first_probe_reply[11],  n_spi->first_probe_reply[10], spi->chip_select, uart_count, n_spi->first_probe_reply[11] << 8 | n_spi->first_probe_reply[10], n_spi->first_probe_reply[13] << 8 | n_spi->first_probe_reply[12], n_spi->first_probe_reply[15] << 8 | n_spi->first_probe_reply[14], n_spi->first_probe_reply[17] << 8 | n_spi->first_probe_reply[16], n_spi->first_probe_reply[19] << 8 | n_spi->first_probe_reply[18]);
	} else {
		ret = -5;
		kfree(n_spi);
		printk(KERN_INFO "NEURONSPI: Probe did not detect a valid Neuron device on CS %d\n", spi->chip_select);
		return ret;
	}
	for (i = 0; i < NEURONSPI_SLOWER_MODELS_LEN; i++) {
		if (NEURONSPI_SLOWER_MODELS[i] == (n_spi->first_probe_reply[17] << 8 | n_spi->first_probe_reply[16])) {
			n_spi->slower_model = 1;
		}
	}
	if (!(neuronspi_cdrv.major_number)) { // Register character device if it doesn't exist
		ret = char_register_driver();
		if (ret) {
			pr_err("Failed to register the neuronspi character driver, ERR:%d\n", ret);
		}
	}
	if (uart_count && neuronspi_uart == NULL) {	// Register UART if not registered
		neuronspi_uart = kzalloc(sizeof(struct uart_driver), GFP_KERNEL);
		neuronspi_uart->owner		= THIS_MODULE;
		neuronspi_uart->dev_name	= "ttyNS";
		neuronspi_uart->driver_name = "ttyNS";
		neuronspi_uart->nr	= NEURONSPI_MAX_UART;
		ret = uart_register_driver(neuronspi_uart);
		if (ret) {
			pr_err("Failed to register the neuronspi uart driver, ERR:%d\n", ret);
		} else {
#if NEURONSPI_DETAILED_DEBUG > 0
			printk(KERN_DEBUG "NEURONSPI: UART driver registered successfully!\n");
#endif
		}
		if (neuronspi_uart_glob_data != NULL) {
			pr_err("Uart data already allocated!\n");
		} else {
			neuronspi_uart_glob_data = kzalloc(sizeof(struct neuronspi_uart_data), GFP_KERNEL);
#if NEURONSPI_DETAILED_DEBUG > 0
			printk(KERN_DEBUG "NEURONSPI: UART driver data allocated successfully!\n");
#endif
		}

	}
	if (neuronspi_uart_glob_data != NULL) {
		n_spi->uart_data = neuronspi_uart_glob_data;
	}
	n_spi->char_driver = &neuronspi_cdrv;
	if (uart_count) {
		n_spi->serial_driver = neuronspi_uart;
	} else {
		n_spi->serial_driver = NULL;
	}
	n_spi->uart_count = uart_count;
	index = n_spi->neuron_index;
#if NEURONSPI_DETAILED_DEBUG > 0
	printk(KERN_DEBUG "NEURONSPI: CHIP SELECT %d\n", spi->chip_select);
#endif
	neuronspi_s_dev[index] = spi;
	spi_set_drvdata(neuronspi_s_dev[index], n_spi);
	n_spi->spi_index = index;
#if NEURONSPI_DETAILED_DEBUG > 0
	printk(KERN_DEBUG "NEURONSPI: SPI IRQ: %d", spi->irq);
#endif
	if (uart_count) {
#if NEURONSPI_DETAILED_DEBUG > 0
		printk(KERN_DEBUG "NEURONSPI: UART registration 1\n");
#endif
		n_spi->uart_buf = kzalloc(NEURONSPI_FIFO_SIZE, GFP_KERNEL);
		neuronspi_uart_probe(spi, n_spi->spi_index);
#if NEURONSPI_DETAILED_DEBUG > 0
		printk(KERN_DEBUG "NEURONSPI: UART PROBE MCTRL:%d\n", neuronspi_spi_uart_get_cflag(spi, 0));
#endif
	}
#if NEURONSPI_DETAILED_DEBUG > 0
	printk(KERN_DEBUG "NEURONSPI: UART registration\n");
#endif
	neuronspi_spi_set_irqs(spi, 0x5);
	for (i = 0; i < NEURONSPI_NO_INTERRUPT_MODELS_LEN; i++) {
		if (NEURONSPI_NO_INTERRUPT_MODELS[i] == (n_spi->first_probe_reply[17] << 8 | n_spi->first_probe_reply[16])) {
			no_irq = 1;
		}
	}
	n_spi->poll_thread = NULL;
	if (!no_irq) {
		n_spi->no_irq = 0;
		ret = devm_request_irq(&(spi->dev), spi->irq, neuronspi_spi_irq, 0x81, dev_name(&(spi->dev)), spi);
#if NEURONSPI_DETAILED_DEBUG > 0
		printk(KERN_DEBUG "NEURONSPI: IRQ registration, ret:%d\n", ret);
#endif
	} else {
		n_spi->no_irq = 1;
#if NEURONSPI_DETAILED_DEBUG > 0
		printk(KERN_DEBUG "NEURONSPI: NO IRQ ON THIS MODEL !!\n");
#endif
	}

	return ret;
}

static int32_t neuronspi_spi_remove(struct spi_device *spi)
{
	struct neuronspi_driver_data *n_spi = spi_get_drvdata(spi);
	kfree(n_spi->send_buf);
	n_spi->send_buf = NULL;
	kfree(n_spi->recv_buf);
	n_spi->recv_buf = NULL;
	if (n_spi->uart_buf) {
		kfree(n_spi->uart_buf);
		n_spi->uart_buf = NULL;
	}
	kfree(n_spi);
	return 0;
}


static int32_t char_register_driver(void)
{
	int32_t ret = 0;
	// Character device registration
#if NEURONSPI_DETAILED_DEBUG > 0
	printk(KERN_DEBUG "NEURONSPI: Initialising Character Device\n");
#endif
	neuronspi_cdrv.major_number = register_chrdev(0, NEURONSPI_NAME, &file_ops);
	if (neuronspi_cdrv.major_number < 0){
	   printk(KERN_ALERT "NEURONSPI: failed to register a major number\n");
	   return neuronspi_cdrv.major_number;
	}
#if NEURONSPI_DETAILED_DEBUG > 0
	printk(KERN_DEBUG "NEURONSPI: registered correctly with major number %d\n", neuronspi_cdrv.major_number);
#endif

	// Character class registration
	neuronspi_cdrv.driver_class = class_create(THIS_MODULE, NEURONSPI_NAME);
	if (IS_ERR(neuronspi_cdrv.driver_class)) {
		unregister_chrdev(neuronspi_cdrv.major_number, NEURONSPI_NAME);
		printk(KERN_ALERT "NEURONSPI: Failed to register device class\n");
		return PTR_ERR(neuronspi_cdrv.driver_class);
	}
#if NEURONSPI_DETAILED_DEBUG > 0
	printk(KERN_DEBUG "NEURONSPI: device class registered correctly\n");
#endif

	// Device driver registration
	neuronspi_cdrv.dev = device_create(neuronspi_cdrv.driver_class, (struct device*)NULL, MKDEV(neuronspi_cdrv.major_number, 0), NULL, NEURONSPI_NAME);
	if (IS_ERR(neuronspi_cdrv.dev)) {
		class_destroy(neuronspi_cdrv.driver_class);
	    	unregister_chrdev(neuronspi_cdrv.major_number, NEURONSPI_NAME);
	    	printk(KERN_ALERT "NEURONSPI: Failed to create the device\n");
	    	return PTR_ERR(neuronspi_cdrv.dev);
	}
#if NEURONSPI_DETAILED_DEBUG > 0
	printk(KERN_DEBUG "NEURONSPI: device class created correctly\n");
#endif
	return ret;
}

static int32_t char_unregister_driver(void)
{
	device_destroy(neuronspi_cdrv.driver_class, MKDEV(neuronspi_cdrv.major_number, 0));     // Destroy the device
	class_unregister(neuronspi_cdrv.driver_class);                          				// Unregister the class
	class_destroy(neuronspi_cdrv.driver_class);                             				// Destroy the class
	unregister_chrdev(neuronspi_cdrv.major_number, NEURONSPI_NAME);             			// Unregister the major number
	printk(KERN_INFO "NEURONSPI: Device unloaded successfully\n");
	return 0;
}

/*********************
 * Final definitions *
 *********************/

MODULE_ALIAS("spi:neuronspi");

static int32_t __init neuronspi_init(void)
{
	int32_t ret = 0;
	neuronspi_spi_w_spinlock = kzalloc(sizeof(struct spinlock), GFP_KERNEL);
	spin_lock_init(neuronspi_spi_w_spinlock);
	mutex_init(&neuronspi_master_mutex);
	memset(&neuronspi_s_dev, 0, sizeof(neuronspi_s_dev));
	ret = spi_register_driver(&neuronspi_spi_driver);
	if (ret < 0) {
		pr_err("Failed to init neuronspi spi --> %d\n", ret);
		return ret;
	} else {
		printk(KERN_INFO "NEURONSPI: SPI Driver Registered\n");
	}
	return ret;
}
module_init(neuronspi_init);

static void __exit neuronspi_exit(void)
{
	if (neuronspi_cdrv.open_counter == 0) {
		char_unregister_driver();
	}
	if (neuronspi_uart != NULL) {
		neuronspi_uart_remove(neuronspi_uart_glob_data);
		uart_unregister_driver(neuronspi_uart);
		kfree(neuronspi_uart);
		kfree(neuronspi_uart_glob_data);
	}
	mutex_lock(&neuronspi_master_mutex);
	spi_unregister_driver(&neuronspi_spi_driver);
	mutex_unlock(&neuronspi_master_mutex);
	kfree(neuronspi_spi_w_spinlock);
	printk(KERN_INFO "NEURONSPI: SPI Driver Unregistered\n");
}
module_exit(neuronspi_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Tomas Knot <knot@faster.cz>");
MODULE_DESCRIPTION("UniPi Neuron driver");
