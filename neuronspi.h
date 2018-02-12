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

#ifndef NEURONSPI_H_
#define NEURONSPI_H_

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt
#define NEURONSPI_SCHED_REQUIRED 0 // Older kernels do not require sched/types to be specifically imported

#define NEURONSPI_MAJOR_VERSIONSTRING "Master Version 1.0:20:12:2017"

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
#include <linux/leds.h>
#include <linux/uaccess.h>
#include <asm/termbits.h>

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
#define NEURONSPI_MAX_TX				62
#define NEURONSPI_MAX_BAUD				115200
#define NEURONSPI_FIFO_SIZE				(256)
#define NEURONSPI_DETAILED_DEBUG		0
#define NEURONSPI_LAST_TRANSFER_DELAY	40

#define NEURONSPI_NAME "neuronspi"
#define NEURON_DEVICE_NAME "neuron"
#define NEURON_DEVICE_CLASS "neuron"
#define NEURON_MAX_MB_BUFFER_LEN 260
#define MODBUS_TCP_HEADER_LENGTH 7
#define PORT_NEURONSPI	184

#define STRICT_RESERVING

#define NEURONSPI_GET_COIL_READ_PHASE2_BYTE_LENGTH(X)	((((X) + 15) >> 4) << 1)

#define NEURONSPI_LED_BRAIN_MODEL					0x10

#define NEURONSPI_NO_INTERRUPT_MODELS_LEN 			3
const uint16_t NEURONSPI_NO_INTERRUPT_MODELS[NEURONSPI_NO_INTERRUPT_MODELS_LEN] = {
		0xb10, 0xc10, 0xf10
};

#define NEURONSPI_SLOWER_MODELS_LEN 				3
const uint16_t NEURONSPI_SLOWER_MODELS[NEURONSPI_SLOWER_MODELS_LEN] = {
		0xb10, 0xc10, 0xf10
};

#define NEURONSPI_PROBE_MESSAGE_LEN					22
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

#define NEURONSPI_SPI_UART_GET_LDISC_MESSAGE_LEN 	16
const u8 NEURONSPI_SPI_UART_GET_LDISC_MESSAGE[NEURONSPI_SPI_UART_GET_LDISC_MESSAGE_LEN] = {
		0x04, 0x08, 0xf6, 0x01, 0x00, 0x00,
		0x04, 0x02, 0xf6, 0x01, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00
};

#define NEURONSPI_SPI_UART_SET_CFLAG_MESSAGE_LEN	16
const u8 NEURONSPI_SPI_UART_SET_CFLAG_MESSAGE[NEURONSPI_SPI_UART_SET_CFLAG_MESSAGE_LEN] = {
		0x06, 0x08, 0xf4, 0x01, 0x00, 0x00,
		0x06, 0x02, 0xf4, 0x01, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00
};

#define NEURONSPI_SPI_LED_SET_MESSAGE_LEN			6
const u8 NEURONSPI_SPI_LED_SET_MESSAGE[NEURONSPI_SPI_LED_SET_MESSAGE_LEN] = {
		0x05, 0x00, 0x08, 0x00, 0x00, 0x00
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

/*
#define NEURONSPI_BOARDTABLE_LEN		16
const uint16_t NEURONSPI_BOARDTABLE[5][NEURONSPI_BOARDTABLE_LEN] = {
		{0,		0,	0,	21,	32},			// B_1000
		{1, 	1,	0,	0,	0},				// E-8Di8Ro
		{2, 	2,	0,	0,	0},				// E-14Ro
		{3, 	3,	0,	0,	0},				// E-16Di
		{4, 	1,	1,	0,	0},				// E-8Di8Ro_P-11DiR485
		{5, 	2,	1,	0,	0},				// E-14Ro_P-11DiR485
		{6, 	3,	1,	0,	0},				// E-16Di_P-11DiR485
		{7, 	2,	2,	0,	0},				// E-14Ro_U-14Ro
		{8, 	3,	2,	0,	0},				// E-16Di_U-14Ro
		{9,		2,	3,	0,	0},				// E-14Ro_U-14Di
		{10,	3,	3,	0,	0},				// E-16Di_U-14Di
		{11,	11,	0,	0,	0},				// E-4Ai4Ao
		{12,	11,	4,	0,	0},				// E-4Ai4Ao_P-6Di5Ro
		{13,	0,	13,	0,	0},				// B-485
		{14,	14,	0,	0,	0},				// E-4Dali
		{15,	11,	5,	0,	0}				// E-4Ai4Ao_U-6Di5Ro
};
*/

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

#define MODBUS_FIRST_DATA_BYTE				10

#define MODBUS_MAX_READ_BITS                2000
#define MODBUS_MAX_WRITE_BITS               1968
#define MODBUS_MAX_READ_REGISTERS           125
#define MODBUS_MAX_WRITE_REGISTERS          123
#define MODBUS_MAX_WR_WRITE_REGISTERS       121
#define MODBUS_MAX_WR_READ_REGISTERS        125


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
	struct neuronspi_uart_data 	*parent;
	u8							dev_index;
	u8							dev_port;
	u8							parmrk_enabled;
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
	struct neuronspi_led_driver *led_driver;
	struct kthread_worker	primary_worker;
	struct task_struct 		*primary_worker_task;
	struct regmap *reg_map;
	struct task_struct *poll_thread;
	struct mutex device_lock;
	u8 led_count;
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
	uint32_t ideal_frequency;
};

// Instantiated once per LED
struct neuronspi_led_driver
{
	struct led_classdev	ldev;
	struct spi_device	*spi;
	struct kthread_work	led_work;
	int					id;
	int					brightness;
	char				name[sizeof("neuron:green:uled-x1")];
	spinlock_t			lock;
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
void neuronspi_spi_led_set_brightness(struct spi_device* spi_dev, enum led_brightness brightness, int id);

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
int	neuronspi_uart_ioctl (struct uart_port *port, unsigned int ioctl_code, unsigned long ioctl_arg);
static void neuronspi_uart_set_ldisc(struct uart_port *port, struct ktermios *kterm);
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

static void neuronspi_led_set_brightness(struct led_classdev *ldev, enum led_brightness brightness);

int neuronspi_regmap_hw_gather_write(void *context, const void *reg, size_t reg_size, const void *val, size_t val_size);
int neuronspi_regmap_hw_read(void *context, const void *reg_buf, size_t reg_size, void *val_buf, size_t val_size);
int neuronspi_regmap_hw_reg_read(void *context, unsigned int reg, unsigned int *val);
int neuronspi_regmap_hw_reg_write(void *context, unsigned int reg, unsigned int val);
int neuronspi_regmap_hw_write(void *context, const void *data, size_t count);

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
		.name			= NEURONSPI_NAME,
		.of_match_table	= of_match_ptr(neuronspi_id_match),
	},
	.probe				= neuronspi_spi_probe,
	.remove				= neuronspi_spi_remove,
};

static struct file_operations file_ops =
{
	.open 				= neuronspi_open,
	.read 				= neuronspi_read,
	.write 				= neuronspi_write,
	.release 			= neuronspi_release,
	.owner				= THIS_MODULE
};

static const struct uart_ops neuronspi_uart_ops =
{
	.tx_empty			= neuronspi_uart_tx_empty,
	.set_mctrl			= neuronspi_uart_set_mctrl,
	.get_mctrl			= neuronspi_uart_get_mctrl,
	.stop_tx			= neuronspi_uart_stop_tx,
	.start_tx			= neuronspi_uart_start_tx,
	.stop_rx			= neuronspi_uart_stop_rx,
	.break_ctl			= neuronspi_uart_break_ctl,
	.startup			= neuronspi_uart_startup,
	.shutdown			= neuronspi_uart_shutdown,
	.set_termios		= neuronspi_uart_set_termios,
	.set_ldisc			= neuronspi_uart_set_ldisc,
	.type				= neuronspi_uart_type,
	.request_port		= neuronspi_uart_request_port,
	.release_port		= neuronspi_uart_null_void,
	.config_port		= neuronspi_uart_config_port,
	.verify_port		= neuronspi_uart_verify_port,
	.pm					= neuronspi_uart_pm,
};

static const struct regmap_bus neuronspi_regmap_bus =
{
	.fast_io 					= 0,
	.write 						= neuronspi_regmap_hw_write,
	.gather_write 				= neuronspi_regmap_hw_gather_write,
	.reg_write					= neuronspi_regmap_hw_reg_write,
	.read						= neuronspi_regmap_hw_read,
	.reg_read					= neuronspi_regmap_hw_reg_read,
	.reg_format_endian_default  = REGMAP_ENDIAN_NATIVE,
	.val_format_endian_default  = REGMAP_ENDIAN_NATIVE,
	.max_raw_read				= 200,								// CRC and other overhead not included
	.max_raw_write				= 200,								// CRC and other overhead not included
};

static const struct regmap_config neuronspi_regmap_config_default =
{
		.name 					= "Neuronspi Regmap",
		.reg_bits				= 16,
		.reg_stride				= 0,
		.pad_bits				= 0,
		.val_bits				= 16,
		.max_register			= 65535,
		.cache_type				= REGCACHE_RBTREE,
		.use_single_rw			= 0,
		.can_multi_write		= 1,
};

// These defines need to be at the end
#define to_neuronspi_uart_data(p,e)  ((container_of((p), struct neuronspi_uart_data, e)))
#define to_neuronspi_port(p,e)	((container_of((p), struct neuronspi_port, e)))
#define to_led_driver(p,e)	((container_of((p), struct neuronspi_led_driver, e)))
#define to_uart_port(p,e)	((container_of((p), struct uart_port, e)))


/***********************
 * Inline Functions    *
 ***********************/

__always_inline uint16_t neuronspi_spi_crc(u8* inputstring, int32_t length, uint16_t initval)
{
    int32_t i;
    uint16_t result = initval;
    for (i=0; i<length; i++) {
        result = (result >> 8) ^ NEURONSPI_CRC16TABLE[(result ^ inputstring[i]) & 0xff];
    }
    return result;
}

__always_inline size_t neuronspi_spi_compose_single_coil_write(uint16_t start, uint8_t **buf_inp, uint8_t **buf_outp, uint8_t data)
{
	uint16_t crc1;
	*buf_outp = kzalloc(6, GFP_KERNEL);
	*buf_inp = kzalloc(6, GFP_KERNEL);
	(*buf_inp)[0] = 0x05;
	(*buf_inp)[1] = data;
	(*buf_inp)[2] = start & 0xFF;
	(*buf_inp)[3] = start >> 8;
	crc1 = neuronspi_spi_crc(*buf_inp, 4, 0);
	memcpy(&(*buf_inp)[4], &crc1, 2);
	return 6;
}

__always_inline size_t neuronspi_spi_compose_single_coil_read(uint16_t start, uint8_t **buf_inp, uint8_t **buf_outp)
{
	uint16_t crc1, crc2;
	// Allocate enough space for 2 headers (8) + 2 CRCs (4) + actual content, which is a 16-bit aligned bit field
	*buf_outp = kzalloc(14, GFP_KERNEL);
	*buf_inp = kzalloc(14, GFP_KERNEL);
	// Read multiple coils Modbus op
	(*buf_inp)[0] = 0x01;
	// Phase 2 length (content + 1x header)
	(*buf_inp)[1] = 0x06;
	// Read index / start address
	(*buf_inp)[2] = start & 0xFF;
	(*buf_inp)[3] = start >> 8;
	// Compute CRC for the first part and copy it into the input buffer
	crc1 = neuronspi_spi_crc(*buf_inp, 4, 0);
	memcpy(&(*buf_inp)[4], &crc1, 2);
	// Copy the part one header into part two
	memcpy(&(*buf_inp)[6], *buf_inp, 4);
	// Compute CRC of the second part and copy it into the buffer
	crc2 = neuronspi_spi_crc(&(*buf_inp)[6], 6, crc1);
	memcpy(&(*buf_inp)[12], &crc2, 2);
	return 14;
}

__always_inline size_t neuronspi_spi_compose_multiple_coil_write(uint8_t number, uint16_t start, uint8_t **buf_inp, uint8_t **buf_outp, uint8_t *data)
{
	uint16_t crc1, crc2;
	// Allocate enough space for 2 headers (8) + 2 CRCs (4) + actual content, which is a 16-bit aligned bit field
	*buf_outp = kzalloc(12 + NEURONSPI_GET_COIL_READ_PHASE2_BYTE_LENGTH(number), GFP_KERNEL);
	*buf_inp = kzalloc(12 + NEURONSPI_GET_COIL_READ_PHASE2_BYTE_LENGTH(number), GFP_KERNEL);
	// Read multiple coils Modbus op
	(*buf_inp)[0] = 0x0F;
	// Phase 2 length (content + 1x header)
	(*buf_inp)[1] = 4 + NEURONSPI_GET_COIL_READ_PHASE2_BYTE_LENGTH(number);
	// Read index / start address
	(*buf_inp)[2] = start & 0xFF;
	(*buf_inp)[3] = start >> 8;
	/// Compute CRC for the first part and copy it into the input buffer
	crc1 = neuronspi_spi_crc(*buf_inp, 4, 0);
	memcpy(&(*buf_inp)[4], &crc1, 2);
	// Copy the part one header into part two
	memcpy(&(*buf_inp)[6], *buf_inp, 4);
	// Change the length of part two into the actual number of coils to write
	(*buf_inp)[7] = number;
	// Copy data into part two
	memcpy(&(*buf_inp)[10], data, NEURONSPI_GET_COIL_READ_PHASE2_BYTE_LENGTH(number));
	// Compute CRC of the second part and copy it into the buffer
	crc2 = neuronspi_spi_crc(&(*buf_inp)[6], 4 + NEURONSPI_GET_COIL_READ_PHASE2_BYTE_LENGTH(number), crc1);
	memcpy(&(*buf_inp)[10 + NEURONSPI_GET_COIL_READ_PHASE2_BYTE_LENGTH(number)], &crc2, 2);
	return 12 + NEURONSPI_GET_COIL_READ_PHASE2_BYTE_LENGTH(number);
}

__always_inline size_t neuronspi_spi_compose_multiple_coil_read(uint8_t number, uint16_t start, uint8_t **buf_inp, uint8_t **buf_outp)
{
	uint16_t crc1, crc2;
	// Allocate enough space for 2 headers (8) + 2 CRCs (4) + actual content, which is a 16-bit aligned bit field
	*buf_outp = kzalloc(12 + NEURONSPI_GET_COIL_READ_PHASE2_BYTE_LENGTH(number), GFP_KERNEL);
	*buf_inp = kzalloc(12 + NEURONSPI_GET_COIL_READ_PHASE2_BYTE_LENGTH(number), GFP_KERNEL);
	// Read multiple coils Modbus op
	(*buf_inp)[0] = 0x01;
	// Phase 2 length (content + 1x header)
	(*buf_inp)[1] = 4 + NEURONSPI_GET_COIL_READ_PHASE2_BYTE_LENGTH(number);
	// Read index / start address
	(*buf_inp)[2] = start & 0xFF;
	(*buf_inp)[3] = start >> 8;
	/// Compute CRC for the first part and copy it into the input buffer
	crc1 = neuronspi_spi_crc(*buf_inp, 4, 0);
	memcpy(&(*buf_inp)[4], &crc1, 2);
	// Copy the part one header into part two
	memcpy(&(*buf_inp)[6], *buf_inp, 4);
	// Compute CRC of the second part and copy it into the buffer
	crc2 = neuronspi_spi_crc(&(*buf_inp)[6], 4 + NEURONSPI_GET_COIL_READ_PHASE2_BYTE_LENGTH(number), crc1);
	memcpy(&(*buf_inp)[10 + NEURONSPI_GET_COIL_READ_PHASE2_BYTE_LENGTH(number)], &crc2, 2);
	return 12 + NEURONSPI_GET_COIL_READ_PHASE2_BYTE_LENGTH(number);
}

__always_inline size_t neuronspi_spi_compose_single_register_write(uint16_t start, uint8_t **buf_inp, uint8_t **buf_outp, uint16_t data)
{
	uint16_t crc1, crc2;
	// Allocate enough space for 2 headers (8) + 2 CRCs (4) + actual content, which is a 16-bit aligned bit field
	*buf_outp = kzalloc(14, GFP_KERNEL);
	*buf_inp = kzalloc(14, GFP_KERNEL);
	// Read multiple coils Modbus op
	(*buf_inp)[0] = 0x06;
	// Phase 2 length (content + 1x header)
	(*buf_inp)[1] = 0x06;
	// Read index / start address
	(*buf_inp)[2] = start & 0xFF;
	(*buf_inp)[3] = start >> 8;
	/// Compute CRC for the first part and copy it into the input buffer
	crc1 = neuronspi_spi_crc(*buf_inp, 4, 0);
	memcpy(&(*buf_inp)[4], &crc1, 2);
	// Copy the part one header into part two
	memcpy(&(*buf_inp)[6], *buf_inp, 4);
	(*buf_inp)[7] = 0x01;
	memcpy(&(*buf_inp)[10], &data, 2);
	crc2 = neuronspi_spi_crc(&(*buf_inp)[6], 6, crc1);
	memcpy(&(*buf_inp)[12], &crc2, 2);
	return 14;
}

__always_inline size_t neuronspi_spi_compose_single_register_read(uint16_t start, uint8_t **buf_inp, uint8_t **buf_outp)
{
	uint16_t crc1, crc2;
	// Allocate enough space for 2 headers (8) + 2 CRCs (4) + actual content, which is a 16-bit aligned bit field
	*buf_outp = kzalloc(14, GFP_KERNEL);
	*buf_inp = kzalloc(14, GFP_KERNEL);
	// Read multiple coils Modbus op
	(*buf_inp)[0] = 0x03;
	// Phase 2 length (content + 1x header)
	(*buf_inp)[1] = 0x06;
	// Read index / start address
	(*buf_inp)[2] = start & 0xFF;
	(*buf_inp)[3] = start >> 8;
	/// Compute CRC for the first part and copy it into the input buffer
	crc1 = neuronspi_spi_crc(*buf_inp, 4, 0);
	memcpy(&(*buf_inp)[4], &crc1, 2);
	// Copy the part one header into part two
	memcpy(&(*buf_inp)[6], *buf_inp, 4);
	(*buf_inp)[7] = 0x01;
	crc2 = neuronspi_spi_crc(&(*buf_inp)[6], 6, crc1);
	memcpy(&(*buf_inp)[12], &crc2, 2);
	return 14;
}

__always_inline size_t neuronspi_spi_compose_multiple_register_write(uint8_t number, uint16_t start, uint8_t **buf_inp, uint8_t **buf_outp, uint8_t *data)
{
	uint16_t crc1, crc2;
	// Allocate enough space for 2 headers (8) + 2 CRCs (4) + actual content, which is a 16-bit aligned bit field
	*buf_outp = kzalloc(12 + (number * 2), GFP_KERNEL);
	*buf_inp = kzalloc(12 + (number * 2), GFP_KERNEL);
	// Read multiple coils Modbus op
	(*buf_inp)[0] = 0x10;
	// Phase 2 length (content + 1x header)
	(*buf_inp)[1] = 4 + (number * 2);
	// Read index / start address
	(*buf_inp)[2] = start & 0xFF;
	(*buf_inp)[3] = start >> 8;
	/// Compute CRC for the first part and copy it into the input buffer
	crc1 = neuronspi_spi_crc(*buf_inp, 4, 0);
	memcpy(&(*buf_inp)[4], &crc1, 2);
	// Copy the part one header into part two
	memcpy(&(*buf_inp)[6], *buf_inp, 4);
	(*buf_inp)[7] = number;
	// Copy the data in
	memcpy(&(*buf_inp)[10], data, number * 2);
	crc2 = neuronspi_spi_crc(&(*buf_inp)[6], 4 + (number * 2), crc1);
	memcpy(&(*buf_inp)[10 + (number * 2)], &crc2, 2);
	return 12 + (number * 2);
}

__always_inline size_t neuronspi_spi_compose_multiple_register_read(uint8_t number, uint16_t start, uint8_t **buf_inp, uint8_t **buf_outp)
{
	uint16_t crc1, crc2;
	// Allocate enough space for 2 headers (8) + 2 CRCs (4) + actual content, which is a 16-bit aligned bit field
	*buf_outp = kzalloc(12 + (number * 2), GFP_KERNEL);
	*buf_inp = kzalloc(12 + (number * 2), GFP_KERNEL);
	// Read multiple coils Modbus op
	(*buf_inp)[0] = 0x03;
	// Phase 2 length (content + 1x header)
	(*buf_inp)[1] = 4 + (number * 2);
	// Read index / start address
	(*buf_inp)[2] = start & 0xFF;
	(*buf_inp)[3] = start >> 8;
	/// Compute CRC for the first part and copy it into the input buffer
	crc1 = neuronspi_spi_crc(*buf_inp, 4, 0);
	memcpy(&(*buf_inp)[4], &crc1, 2);
	// Copy the part one header into part two
	memcpy(&(*buf_inp)[6], *buf_inp, 4);
	(*buf_inp)[7] = number;
	crc2 = neuronspi_spi_crc(&(*buf_inp)[6], 4 + (number * 2), crc1);
	memcpy(&(*buf_inp)[10 + (number * 2)], &crc2, 2);
	return 12 + (number * 2);
}

#endif /* NEURONSPI_H_ */
