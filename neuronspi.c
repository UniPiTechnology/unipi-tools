/*
 * SC16IS7xx tty serial driver - Copyright (C) 2014 GridPoint
 * Author: Jon Ringle <jringle@gridpoint.com>
 *
 *  Based on max310x.c, by Alexander Shiyan <shc_work@mail.ru>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

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

#define NEURONSPI_NAME					"neuronspi"
#define NEURONSPI_MAX_DEVS				3
#define NEURONSPI_BUFFER_MAX			1152
#define NEURONSPI_HEADER_LENGTH 		10
#define NEURONSPI_FIRST_MESSAGE_LENGTH	6
#define NEURONSPI_EDGE_DELAY			10
#define NEURONSPI_B_PER_WORD 			8
#define NEURONSPI_DEFAULT_FREQ			500000
#define NEURONSPI_COMMON_FREQ			12000000
#define NEURONSPI_MAX_TX				64


const char NEURONSPI_PROBE_MESSAGE[22] = {
		0x04, 0x0e, 0xe8, 0x03, 0xa0, 0xdd, 0x04, 0x00, 0xe8, 0x03,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x12, 0x16
};
#define NEURONSPI_PROBE_MESSAGE_LEN 	22

const char NEURONSPI_UART_PROBE_MESSAGE[22] = {

};
#define NEURONSPI_UART_PROBE_MESSAGE_LEN 	22

static const struct of_device_id neuronspi_id_match[] = {
		{.compatible = "unipi,neuron"},
		{.compatible = "neuronspi"},
		{}
};
MODULE_DEVICE_TABLE(of, neuronspi_id_match);

#define NEURON_FIFO_SIZE		(256)
#define NEURON_DEVICE_NAME "neuronspi"
#define NEURON_DEVICE_CLASS "ebb"
#define NEURON_MAX_MB_BUFFER_LEN 260
#define MODBUS_TCP_HEADER_LENGTH 7

/* SC16IS7XX register definitions */
#define SC16IS7XX_RHR_REG		(0x00) /* RX FIFO */
#define SC16IS7XX_THR_REG		(0x00) /* TX FIFO */
#define SC16IS7XX_IER_REG		(0x01) /* Interrupt enable */
#define SC16IS7XX_IIR_REG		(0x02) /* Interrupt Identification */
#define SC16IS7XX_FCR_REG		(0x02) /* FIFO control */
#define SC16IS7XX_LCR_REG		(0x03) /* Line Control */
#define SC16IS7XX_MCR_REG		(0x04) /* Modem Control */
#define SC16IS7XX_LSR_REG		(0x05) /* Line Status */
#define SC16IS7XX_MSR_REG		(0x06) /* Modem Status */
#define SC16IS7XX_SPR_REG		(0x07) /* Scratch Pad */
#define SC16IS7XX_TXLVL_REG		(0x08) /* TX FIFO level */
#define SC16IS7XX_RXLVL_REG		(0x09) /* RX FIFO level */
#define SC16IS7XX_IODIR_REG		(0x0a) /* I/O Direction
						* - only on 75x/76x
						*/
#define SC16IS7XX_IOSTATE_REG		(0x0b) /* I/O State
						* - only on 75x/76x
						*/
#define SC16IS7XX_IOINTENA_REG		(0x0c) /* I/O Interrupt Enable
						* - only on 75x/76x
						*/
#define SC16IS7XX_IOCONTROL_REG		(0x0e) /* I/O Control
						* - only on 75x/76x
						*/
#define SC16IS7XX_EFCR_REG		(0x0f) /* Extra Features Control */

/* TCR/TLR Register set: Only if ((MCR[2] == 1) && (EFR[4] == 1)) */
#define SC16IS7XX_TCR_REG		(0x06) /* Transmit control */
#define SC16IS7XX_TLR_REG		(0x07) /* Trigger level */

/* Special Register set: Only if ((LCR[7] == 1) && (LCR != 0xBF)) */
#define SC16IS7XX_DLL_REG		(0x00) /* Divisor Latch Low */
#define SC16IS7XX_DLH_REG		(0x01) /* Divisor Latch High */

/* Enhanced Register set: Only if (LCR == 0xBF) */
#define SC16IS7XX_EFR_REG		(0x02) /* Enhanced Features */
#define SC16IS7XX_XON1_REG		(0x04) /* Xon1 word */
#define SC16IS7XX_XON2_REG		(0x05) /* Xon2 word */
#define SC16IS7XX_XOFF1_REG		(0x06) /* Xoff1 word */
#define SC16IS7XX_XOFF2_REG		(0x07) /* Xoff2 word */

/* IER register bits */
#define SC16IS7XX_IER_RDI_BIT		(1 << 0) /* Enable RX data interrupt */
#define SC16IS7XX_IER_THRI_BIT		(1 << 1) /* Enable TX holding register
						  * interrupt */
#define SC16IS7XX_IER_RLSI_BIT		(1 << 2) /* Enable RX line status
						  * interrupt */
#define SC16IS7XX_IER_MSI_BIT		(1 << 3) /* Enable Modem status
						  * interrupt */

/* IER register bits - write only if (EFR[4] == 1) */
#define SC16IS7XX_IER_SLEEP_BIT		(1 << 4) /* Enable Sleep mode */
#define SC16IS7XX_IER_XOFFI_BIT		(1 << 5) /* Enable Xoff interrupt */
#define SC16IS7XX_IER_RTSI_BIT		(1 << 6) /* Enable nRTS interrupt */
#define SC16IS7XX_IER_CTSI_BIT		(1 << 7) /* Enable nCTS interrupt */

/* FCR register bits */
#define SC16IS7XX_FCR_FIFO_BIT		(1 << 0) /* Enable FIFO */
#define SC16IS7XX_FCR_RXRESET_BIT	(1 << 1) /* Reset RX FIFO */
#define SC16IS7XX_FCR_TXRESET_BIT	(1 << 2) /* Reset TX FIFO */
#define SC16IS7XX_FCR_RXLVLL_BIT	(1 << 6) /* RX Trigger level LSB */
#define SC16IS7XX_FCR_RXLVLH_BIT	(1 << 7) /* RX Trigger level MSB */

/* FCR register bits - write only if (EFR[4] == 1) */
#define SC16IS7XX_FCR_TXLVLL_BIT	(1 << 4) /* TX Trigger level LSB */
#define SC16IS7XX_FCR_TXLVLH_BIT	(1 << 5) /* TX Trigger level MSB */

/* IIR register bits */
#define SC16IS7XX_IIR_NO_INT_BIT	(1 << 0) /* No interrupts pending */
#define SC16IS7XX_IIR_ID_MASK		0x3e     /* Mask for the interrupt ID */
#define SC16IS7XX_IIR_THRI_SRC		0x02     /* TX holding register empty */
#define SC16IS7XX_IIR_RDI_SRC		0x04     /* RX data interrupt */
#define SC16IS7XX_IIR_RLSE_SRC		0x06     /* RX line status error */
#define SC16IS7XX_IIR_RTOI_SRC		0x0c     /* RX time-out interrupt */
#define SC16IS7XX_IIR_MSI_SRC		0x00     /* Modem status interrupt
						  * - only on 75x/76x
						  */
#define SC16IS7XX_IIR_INPIN_SRC		0x30     /* Input pin change of state
						  * - only on 75x/76x
						  */
#define SC16IS7XX_IIR_XOFFI_SRC		0x10     /* Received Xoff */
#define SC16IS7XX_IIR_CTSRTS_SRC	0x20     /* nCTS,nRTS change of state
						  * from active (LOW)
						  * to inactive (HIGH)
						  */
/* LCR register bits */
#define SC16IS7XX_LCR_LENGTH0_BIT	(1 << 0) /* Word length bit 0 */
#define SC16IS7XX_LCR_LENGTH1_BIT	(1 << 1) /* Word length bit 1
						  *
						  * Word length bits table:
						  * 00 -> 5 bit words
						  * 01 -> 6 bit words
						  * 10 -> 7 bit words
						  * 11 -> 8 bit words
						  */
#define SC16IS7XX_LCR_STOPLEN_BIT	(1 << 2) /* STOP length bit
						  *
						  * STOP length bit table:
						  * 0 -> 1 stop bit
						  * 1 -> 1-1.5 stop bits if
						  *      word length is 5,
						  *      2 stop bits otherwise
						  */
#define SC16IS7XX_LCR_PARITY_BIT	(1 << 3) /* Parity bit enable */
#define SC16IS7XX_LCR_EVENPARITY_BIT	(1 << 4) /* Even parity bit enable */
#define SC16IS7XX_LCR_FORCEPARITY_BIT	(1 << 5) /* 9-bit multidrop parity */
#define SC16IS7XX_LCR_TXBREAK_BIT	(1 << 6) /* TX break enable */
#define SC16IS7XX_LCR_DLAB_BIT		(1 << 7) /* Divisor Latch enable */
#define SC16IS7XX_LCR_WORD_LEN_5	(0x00)
#define SC16IS7XX_LCR_WORD_LEN_6	(0x01)
#define SC16IS7XX_LCR_WORD_LEN_7	(0x02)
#define SC16IS7XX_LCR_WORD_LEN_8	(0x03)
#define SC16IS7XX_LCR_CONF_MODE_A	SC16IS7XX_LCR_DLAB_BIT /* Special
								* reg set */
#define SC16IS7XX_LCR_CONF_MODE_B	0xBF                   /* Enhanced
								* reg set */

/* MCR register bits */
#define SC16IS7XX_MCR_DTR_BIT		(1 << 0) /* DTR complement
						  * - only on 75x/76x
						  */
#define SC16IS7XX_MCR_RTS_BIT		(1 << 1) /* RTS complement */
#define SC16IS7XX_MCR_TCRTLR_BIT	(1 << 2) /* TCR/TLR register enable */
#define SC16IS7XX_MCR_LOOP_BIT		(1 << 4) /* Enable loopback test mode */
#define SC16IS7XX_MCR_XONANY_BIT	(1 << 5) /* Enable Xon Any
						  * - write enabled
						  * if (EFR[4] == 1)
						  */
#define SC16IS7XX_MCR_IRDA_BIT		(1 << 6) /* Enable IrDA mode
						  * - write enabled
						  * if (EFR[4] == 1)
						  */
#define SC16IS7XX_MCR_CLKSEL_BIT	(1 << 7) /* Divide clock by 4
						  * - write enabled
						  * if (EFR[4] == 1)
						  */

/* LSR register bits */
#define SC16IS7XX_LSR_DR_BIT		(1 << 0) /* Receiver data ready */
#define SC16IS7XX_LSR_OE_BIT		(1 << 1) /* Overrun Error */
#define SC16IS7XX_LSR_PE_BIT		(1 << 2) /* Parity Error */
#define SC16IS7XX_LSR_FE_BIT		(1 << 3) /* Frame Error */
#define SC16IS7XX_LSR_BI_BIT		(1 << 4) /* Break Interrupt */
#define SC16IS7XX_LSR_BRK_ERROR_MASK	0x1E     /* BI, FE, PE, OE bits */
#define SC16IS7XX_LSR_THRE_BIT		(1 << 5) /* TX holding register empty */
#define SC16IS7XX_LSR_TEMT_BIT		(1 << 6) /* Transmitter empty */
#define SC16IS7XX_LSR_FIFOE_BIT		(1 << 7) /* Fifo Error */

/* MSR register bits */
#define SC16IS7XX_MSR_DCTS_BIT		(1 << 0) /* Delta CTS Clear To Send */
#define SC16IS7XX_MSR_DDSR_BIT		(1 << 1) /* Delta DSR Data Set Ready
						  * or (IO4)
						  * - only on 75x/76x
						  */
#define SC16IS7XX_MSR_DRI_BIT		(1 << 2) /* Delta RI Ring Indicator
						  * or (IO7)
						  * - only on 75x/76x
						  */
#define SC16IS7XX_MSR_DCD_BIT		(1 << 3) /* Delta CD Carrier Detect
						  * or (IO6)
						  * - only on 75x/76x
						  */
#define SC16IS7XX_MSR_CTS_BIT		(1 << 0) /* CTS */
#define SC16IS7XX_MSR_DSR_BIT		(1 << 1) /* DSR (IO4)
						  * - only on 75x/76x
						  */
#define SC16IS7XX_MSR_RI_BIT		(1 << 2) /* RI (IO7)
						  * - only on 75x/76x
						  */
#define SC16IS7XX_MSR_CD_BIT		(1 << 3) /* CD (IO6)
						  * - only on 75x/76x
						  */
#define SC16IS7XX_MSR_DELTA_MASK	0x0F     /* Any of the delta bits! */

/*
 * TCR register bits
 * TCR trigger levels are available from 0 to 60 characters with a granularity
 * of four.
 * The programmer must program the TCR such that TCR[3:0] > TCR[7:4]. There is
 * no built-in hardware check to make sure this condition is met. Also, the TCR
 * must be programmed with this condition before auto RTS or software flow
 * control is enabled to avoid spurious operation of the device.
 */
#define SC16IS7XX_TCR_RX_HALT(words)	((((words) / 4) & 0x0f) << 0)
#define SC16IS7XX_TCR_RX_RESUME(words)	((((words) / 4) & 0x0f) << 4)

/*
 * TLR register bits
 * If TLR[3:0] or TLR[7:4] are logical 0, the selectable trigger levels via the
 * FIFO Control Register (FCR) are used for the transmit and receive FIFO
 * trigger levels. Trigger levels from 4 characters to 60 characters are
 * available with a granularity of four.
 *
 * When the trigger level setting in TLR is zero, the SC16IS740/750/760 uses the
 * trigger level setting defined in FCR. If TLR has non-zero trigger level value
 * the trigger level defined in FCR is discarded. This applies to both transmit
 * FIFO and receive FIFO trigger level setting.
 *
 * When TLR is used for RX trigger level control, FCR[7:6] should be left at the
 * default state, that is, '00'.
 */
#define SC16IS7XX_TLR_TX_TRIGGER(words)	((((words) / 4) & 0x0f) << 0)
#define SC16IS7XX_TLR_RX_TRIGGER(words)	((((words) / 4) & 0x0f) << 4)

/* IOControl register bits (Only 750/760) */
#define SC16IS7XX_IOCONTROL_LATCH_BIT	(1 << 0) /* Enable input latching */
#define SC16IS7XX_IOCONTROL_GPIO_BIT	(1 << 1) /* Enable GPIO[7:4] */
#define SC16IS7XX_IOCONTROL_SRESET_BIT	(1 << 3) /* Software Reset */

/* EFCR register bits */
#define SC16IS7XX_EFCR_9BIT_MODE_BIT	(1 << 0) /* Enable 9-bit or Multidrop
						  * mode (RS485) */
#define SC16IS7XX_EFCR_RXDISABLE_BIT	(1 << 1) /* Disable receiver */
#define SC16IS7XX_EFCR_TXDISABLE_BIT	(1 << 2) /* Disable transmitter */
#define SC16IS7XX_EFCR_AUTO_RS485_BIT	(1 << 4) /* Auto RS485 RTS direction */
#define SC16IS7XX_EFCR_RTS_INVERT_BIT	(1 << 5) /* RTS output inversion */
#define SC16IS7XX_EFCR_IRDA_MODE_BIT	(1 << 7) /* IrDA mode
						  * 0 = rate upto 115.2 kbit/s
						  *   - Only 750/760
						  * 1 = rate upto 1.152 Mbit/s
						  *   - Only 760
						  */

/* EFR register bits */
#define SC16IS7XX_EFR_AUTORTS_BIT	(1 << 6) /* Auto RTS flow ctrl enable */
#define SC16IS7XX_EFR_AUTOCTS_BIT	(1 << 7) /* Auto CTS flow ctrl enable */
#define SC16IS7XX_EFR_XOFF2_DETECT_BIT	(1 << 5) /* Enable Xoff2 detection */
#define SC16IS7XX_EFR_ENABLE_BIT	(1 << 4) /* Enable enhanced functions
						  * and writing to IER[7:4],
						  * FCR[5:4], MCR[7:5]
						  */
#define SC16IS7XX_EFR_SWFLOW3_BIT	(1 << 3) /* SWFLOW bit 3 */
#define SC16IS7XX_EFR_SWFLOW2_BIT	(1 << 2) /* SWFLOW bit 2
						  *
						  * SWFLOW bits 3 & 2 table:
						  * 00 -> no transmitter flow
						  *       control
						  * 01 -> transmitter generates
						  *       XON2 and XOFF2
						  * 10 -> transmitter generates
						  *       XON1 and XOFF1
						  * 11 -> transmitter generates
						  *       XON1, XON2, XOFF1 and
						  *       XOFF2
						  */
#define SC16IS7XX_EFR_SWFLOW1_BIT	(1 << 1) /* SWFLOW bit 2 */
#define SC16IS7XX_EFR_SWFLOW0_BIT	(1 << 0) /* SWFLOW bit 3
						  *
						  * SWFLOW bits 3 & 2 table:
						  * 00 -> no received flow
						  *       control
						  * 01 -> receiver compares
						  *       XON2 and XOFF2
						  * 10 -> receiver compares
						  *       XON1 and XOFF1
						  * 11 -> receiver compares
						  *       XON1, XON2, XOFF1 and
						  *       XOFF2
						  */

/* Misc definitions */
#define SC16IS7XX_REG_SHIFT		2




#define NEURON_INT_RX_NOT_EMPTY 0x1
#define NEURON_INT_TX_FINISHED  0x2
#define NEURON_INT_RX_MODBUS    0x4
#define NEURON_INT_DI_CHANGED   0x8
#define NEURON_INT_ID_MASK      0x0f
#define NEURON_INT_NO_INT_BIT   0x0f

// from modbus_h
/* Modbus function codes */
#define MODBUS_FC_READ_COILS                0x01
#define MODBUS_FC_READ_DISCRETE_INPUTS      0x02
#define MODBUS_FC_READ_HOLDING_REGISTERS    0x03
#define MODBUS_FC_READ_INPUT_REGISTERS      0x04
#define MODBUS_FC_WRITE_SINGLE_COIL         0x05
#define MODBUS_FC_WRITE_SINGLE_REGISTER     0x06
#define MODBUS_FC_READ_EXCEPTION_STATUS     0x07
#define MODBUS_FC_WRITE_MULTIPLE_COILS      0x0F
#define MODBUS_FC_WRITE_MULTIPLE_REGISTERS  0x10
#define MODBUS_FC_REPORT_SLAVE_ID           0x11
#define MODBUS_FC_MASK_WRITE_REGISTER       0x16
#define MODBUS_FC_WRITE_AND_READ_REGISTERS  0x17

#define MODBUS_MAX_READ_BITS              2000
#define MODBUS_MAX_WRITE_BITS             1968
#define MODBUS_MAX_READ_REGISTERS          125
#define MODBUS_MAX_WRITE_REGISTERS         123
#define MODBUS_MAX_WR_WRITE_REGISTERS      121
#define MODBUS_MAX_WR_READ_REGISTERS       125

/* EMPTY DEFINITIONS ToDo */
#define SPI_READ_STR(a,b,c) {}
#define SPI_WRITE_STR(a,b,c) {}
#define SPI_WRITE(a,b) {}
#define SPI_WRITE_CHAR(a) {}
#define SPI_READ_REG(a,b) {}
#define SPI_READ_BIT(a,b) {}
#define SPI_WRITE_COIL(a,b) {}
#define SPI_IDLE() {}


struct neuronspi_devtype {
	char	name[10];
	int	nr_gpio;
	int	nr_uart;
};

#define SC16IS7XX_RECONF_MD		(1 << 0)
#define SC16IS7XX_RECONF_IER		(1 << 1)
#define SC16IS7XX_RECONF_RS485		(1 << 2)

struct neuronspi_one_config {
	unsigned int			flags;
	u8				ier_clear;
};

struct neuronspi_one {
	struct uart_port		port;
	u8				line;
	struct kthread_work		tx_work;
	struct kthread_work		reg_work;
	struct neuronspi_one_config	config;
};

struct neuronspi_uart_port {
	const struct neuronspi_devtype	*devtype;
	struct regmap			*regmap;
	struct clk			*clk;
#ifdef CONFIG_GPIOLIB
	struct gpio_chip		gpio;
#endif
	unsigned char			buf[NEURON_FIFO_SIZE];
	struct kthread_worker		kworker;
	struct task_struct		*kworker_task;
	struct kthread_work		irq_work;
	struct neuronspi_one		p[0];
};

static unsigned long neuronspi_lines;

struct uart_driver neuronspi_unreg_uart = {
		.owner		= THIS_MODULE,
		.dev_name	= "ttySC",
		.nr		= NEURONSPI_MAX_DEVS,
};

static struct uart_driver* neuronspi_uart;


struct neuronspi_char_driver {
	int major_number;
	char *message;
	short message_size;
	int open_counter;
	struct class* driver_class;
	struct class* dev_name;
};

struct neuronspi_driver_data {
	//struct spi_master *master;
	struct spi_driver *spi_driver;
	struct neuronspi_char_driver *char_driver;
	struct uart_driver *serial_driver;
	struct mutex lock;
	char *send_buf;
	char *recv_buf;
	char *first_probe_reply;
	char *second_probe_reply;
	char reserved_device;
};

struct mutex neuronspi_master_mutex;

struct neuronspi_file_data {
	struct spi_device**  spi_device;
	struct mutex lock;
	char *send_buf;
	char *recv_buf;
	int message_len;
};

struct neuronspi_direct_acc {
	void __iomem		*vaddr;
	u32			size;
};

/*
struct neuronspi_spi {
	struct spi_master *master;
	void __iomem *base;
	struct clk *clk;
	const struct neuronspi_spi_dev *devdata;
	struct neuronspi_direct_acc *direct_access;
};
*/

struct neuronspi_spi_dev {
	unsigned long		max_hz;
	unsigned int		min_divisor;
	unsigned int		max_divisor;
	u32			prescale_mask;
	bool			is_errata_50mhz_ac;
};

static int neuronspi_open (struct inode *, struct file *);
static int neuronspi_release (struct inode *, struct file *);
static ssize_t neuronspi_read (struct file *, char *, size_t, loff_t *);
static ssize_t neuronspi_write (struct file *, const char *, size_t, loff_t *);
static int char_register_driver(void);
static int char_unregister_driver(void);
static int neuronspi_spi_probe(struct spi_device *spi);
static int neuronspi_spi_remove(struct spi_device *spi);
void neuronspi_spi_send_message(struct spi_device* spi_dev, char *send_buf, char *recv_buf, int len, int freq, int delay, int send_header);

static void neuronspi_start_tx(struct uart_port *port);
static void neuronspi_stop_tx(struct uart_port *port);
static void neuronspi_start_tx(struct uart_port *port);
static void neuronspi_stop_rx(struct uart_port *port);
static void neuronspi_set_termios(struct uart_port *port, struct ktermios *termios, struct ktermios *old);
static unsigned int neuronspi_tx_empty(struct uart_port *port);
static void neuronspi_tx_proc(struct kthread_work *ws);
static void neuronspi_break_ctl(struct uart_port *port, int break_state);
static void neuronspi_shutdown(struct uart_port *port);
static int neuronspi_startup(struct uart_port *port);
static int neuronspi_request_port(struct uart_port *port);
static int neuronspi_alloc_line(void);
static irqreturn_t neuronspi_irq(int irq, void *dev_id);
static void neuronspi_set_mctrl(struct uart_port *port, unsigned int mctrl);
static unsigned int neuronspi_get_mctrl(struct uart_port *port);
static const char *neuronspi_type(struct uart_port *port);
static void neuronspi_null_void(struct uart_port *port);
static void neuronspi_config_port(struct uart_port *port, int flags);
static int neuronspi_verify_port(struct uart_port *port, struct serial_struct *s);
static void neuronspi_pm(struct uart_port *port, unsigned int state, unsigned int oldstate);


static struct spi_device* neuronspi_s_dev[NEURONSPI_MAX_DEVS];
static struct neuronspi_char_driver neuronspi_cdrv = {
	.driver_class = "ebb",
	.dev_name = "neuronspi"
};

// Host driver struct
static struct spi_driver neuronspi_spi_driver = {
	.driver = {
		.name		= NEURONSPI_NAME,
		.of_match_table	= of_match_ptr(neuronspi_id_match),
	},
	.probe		= neuronspi_spi_probe,
	.remove		= neuronspi_spi_remove,
};

static struct file_operations file_ops = {
	.open = neuronspi_open,
	.read = neuronspi_read,
	.write = neuronspi_write,
	.release = neuronspi_release
};

static const struct uart_ops neuronspi_uart_ops = {
	.tx_empty		= neuronspi_tx_empty,
	.set_mctrl		= neuronspi_set_mctrl,
	.get_mctrl		= neuronspi_get_mctrl,
	.stop_tx		= neuronspi_stop_tx,
	.start_tx		= neuronspi_start_tx,
	.stop_rx		= neuronspi_stop_rx,
	.break_ctl		= neuronspi_break_ctl,
	.startup		= neuronspi_startup,
	.shutdown		= neuronspi_shutdown,
	.set_termios	= neuronspi_set_termios,
	.type			= neuronspi_type,
	.request_port	= neuronspi_request_port,
	.release_port	= neuronspi_null_void,
	.config_port	= neuronspi_config_port,
	.verify_port	= neuronspi_verify_port,
	.pm				= neuronspi_pm,
};

static char *turn_buf;

#define to_neuronspi_port(p,e)	((container_of((p), struct neuronspi_port, e)))
#define to_neuronspi_one(p,e)	((container_of((p), struct neuronspi_one, e)))

static int neuronspi_open (struct inode *inode_p, struct file *file_p) {
	//printk(KERN_INFO "NEURONSPI: Device opened successfully\n");
	struct neuronspi_file_data *f_internal_data;
	f_internal_data = kzalloc(sizeof(*f_internal_data), GFP_KERNEL);
	f_internal_data->recv_buf = kzalloc(NEURONSPI_BUFFER_MAX, GFP_KERNEL);
	f_internal_data->send_buf = kzalloc(NEURONSPI_BUFFER_MAX, GFP_KERNEL);
	f_internal_data->spi_device = neuronspi_s_dev;
	mutex_init(&f_internal_data->lock);
	file_p->private_data = f_internal_data;
//	printk(KERN_INFO "NEURONSPI: Device opened\n");
	return 0;
}

static int neuronspi_release (struct inode *inode_p, struct file *file_p) {
	//printk(KERN_INFO "NEURONSPI: Device closed successfully\n");
	//kfree(file_p->private_data);
	struct neuronspi_file_data *f_internal_data = (struct neuronspi_file_data*)file_p->private_data;
	f_internal_data->spi_device = NULL;
	kfree(f_internal_data->recv_buf);
	f_internal_data->recv_buf = NULL;
	kfree(f_internal_data->send_buf);
	f_internal_data->send_buf = NULL;
	kfree(f_internal_data);
	file_p->private_data = NULL;
//	printk(KERN_INFO "NEURONSPI: Device closed\n");
	return 0;
}

static ssize_t neuronspi_read (struct file *file_p, char *buffer, size_t len, loff_t *offset) {
	//const char* msg = "Neuronspi\n";
    int result = 0;
	char device_index = 0;
	int frequency = NEURONSPI_COMMON_FREQ;
	int transmit_len = len - NEURONSPI_HEADER_LENGTH;
	struct neuronspi_file_data* private_data;
	struct spi_device* spi_driver_data;
	struct neuronspi_driver_data* driver_data;
	// Sanity checking
	if (buffer == NULL) return -7; // Invalid read buffer
    if (len == 0) return result; // Empty read
    if (file_p == NULL) {
    	printk(KERN_INFO "NEURONSPI: File Pointer is NULL\n");
    	return -8;
    }
    if (file_p->private_data == NULL) {
    	printk(KERN_INFO "NEURONSPI: No Private Data\n");
    	return -1;	// No private data
    }
	printk(KERN_INFO "NEURONSPI: Read %d Stage 1\n", len);
    private_data = (struct spi_file_data*) file_p->private_data;
    if (private_data == NULL) return -4;
    device_index = private_data->send_buf[0];
    spi_driver_data = private_data->spi_device[device_index];	// Get private (driver) data from FP
    driver_data = spi_get_drvdata(spi_driver_data);
    if (driver_data->spi_driver == NULL) return -2;	// Invalid private data
    if (driver_data->first_probe_reply[0] == 0) return -3; // No device present
//    printk(KERN_INFO "NEURONSPI: Read Stage 1b\n");
//	printk(KERN_INFO "NEURONSPI: Read Stage 2\n");
    mutex_lock(&(private_data->lock));
    if (private_data->recv_buf == NULL) {
    	mutex_unlock(&(private_data->lock));
    	return -10;
    }
	//printk(KERN_INFO "NEURONSPI: Read Stage 3\n");
    printk(KERN_INFO "NEURONSPI: Device read %d DEV:%s%d DRV:%s%d\n", private_data->message_len, (spi_driver_data->dev.of_node->name), (spi_driver_data->chip_select), (driver_data->spi_driver->driver.name), (device_index));
    //printk(KERN_INFO "NEURONSPI: Device read len:%d %100ph\n", transmit_len, &(private_data->recv_buf[NEURONSPI_HEADER_LENGTH]));
    if ((((int)len) == private_data->message_len + 10)) {
    	memcpy(buffer, private_data->recv_buf, len);
    	result = len;
    } else {
    	result = simple_read_from_buffer(buffer, len, offset, private_data->recv_buf, len);
    }
    //
	printk(KERN_INFO "NEURONSPI: Read Stage %d 4\n", result);
    memset(private_data->recv_buf, 0, NEURONSPI_BUFFER_MAX);
	mutex_unlock(&(private_data->lock));
	return result;
}



static ssize_t neuronspi_write (struct file *file_p, const char *buffer, size_t len, loff_t *w_offset) {
	char device_index = 0;
	int result = 0;
	int frequency = NEURONSPI_COMMON_FREQ;
	int transmit_len = len - NEURONSPI_HEADER_LENGTH;
	int send_header = 0;
	int delay = 25;
	struct neuronspi_file_data* private_data;
	struct spi_device* spi_driver_data;
	struct neuronspi_driver_data* driver_data;
	printk(KERN_INFO "NEURONSPI: LENGTH:%d\n", len);
	// Sanity checking
	if (buffer == NULL) return 0; // Void write
    if (len == 0) return result; // Empty write
    if (file_p->private_data == NULL) {
    	printk(KERN_INFO "NEURONSPI: No Private Data\n");
    	return -1;	// No private data
    }
    // Read packet header and initiliase private data (dependent on each other)
    device_index = buffer[0];
    if (device_index > NEURONSPI_MAX_DEVS - 1) return -2;
    private_data = (struct spi_file_data*) file_p->private_data;
    spi_driver_data = private_data->spi_device[device_index];	// Get private (driver) data from FP
    driver_data = spi_get_drvdata(spi_driver_data);
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
    } else if (device_index == (driver_data->reserved_device - 1)) {
    	return -5;			// Device reserved
    }
    mutex_lock(&(private_data->lock));
    memset(private_data->send_buf, 0, sizeof(&(private_data->send_buf)) );
    memcpy(private_data->send_buf, buffer, len);
    memset(private_data->recv_buf, 0, sizeof(&(private_data->recv_buf)) );
    private_data->message_len = transmit_len;
    //printk(KERN_INFO "NEURONSPI: Device write DEV:%s%d DRV:%s%d\n", (spi_driver_data->dev.of_node->name), (spi_driver_data->chip_select), (driver_data->spi_driver->driver.name), (device_index));
    printk(KERN_INFO "NEURONSPI: Device write len:%d %100ph\n", transmit_len, &(private_data->send_buf[NEURONSPI_HEADER_LENGTH]));
    neuronspi_spi_send_message(spi_driver_data, &private_data->send_buf[NEURONSPI_HEADER_LENGTH], private_data->recv_buf, transmit_len, frequency, delay, send_header);
    //printk(KERN_INFO "NEURONSPI: Device write finished, read: len%d - %100ph\n", transmit_len, private_data->recv_buf);

    mutex_unlock(&private_data->lock);
    return len;
}

void neuronspi_callback(void *context) {
	printk(KERN_INFO "\n");
}

void neuronspi_spi_send_message(struct spi_device* spi_dev, char *send_buf, char *recv_buf, int len, int freq, int delay, int send_header) {
	int i = 0;
	int cnt = 3;
	int trans_count = (len / NEURONSPI_MAX_TX) + 3;	// number of transmissions
	struct spi_message s_msg;
	mutex_lock(&neuronspi_master_mutex);
	if (!send_header) {
		trans_count -= 1;	// one less transmission as the header is omitted
	}
    struct spi_transfer s_trans[trans_count];
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
    			// if len is more than NEURONSPI_MAX_TX * i, then chunk len is NEURONSPI_MAX_TX, otherwise it's the remainder
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
    		// if len is more than NEURONSPI_MAX_TX * i (+ optionally header), then chunk len is NEURONSPI_MAX_TX (+ optionally header), otherwise it's the remainder

    	}
    	spi_message_add_tail(&(s_trans[i]), &s_msg);
    }
    /*
    s_trans[0].delay_usecs = NEURONSPI_EDGE_DELAY;
    s_trans[1].delay_usecs = delay;
    s_trans[1].len = NEURONSPI_FIRST_MESSAGE_LENGTH;
    s_trans[1].tx_buf = send_buf;
    s_trans[1].rx_buf = recv_buf;
	s_trans[2].len = len;
	s_trans[2].tx_buf = &(send_buf[NEURONSPI_FIRST_MESSAGE_LENGTH]);
	s_trans[2].rx_buf = &(recv_buf[NEURONSPI_FIRST_MESSAGE_LENGTH]);
	if (len > NEURONSPI_FIRST_MESSAGE_LENGTH + 64) {
		cnt++;
		s_trans[2].len = 64;
		s_trans[3].len = len - 64;
		s_trans[3].tx_buf = &(send_buf[NEURONSPI_FIRST_MESSAGE_LENGTH + 64]);
		s_trans[3].rx_buf = &(recv_buf[NEURONSPI_FIRST_MESSAGE_LENGTH + 64]);
	}
	if (len > NEURONSPI_FIRST_MESSAGE_LENGTH + 128) {
		cnt++;
		s_trans[3].len = 64;
		s_trans[4].len = len - 128;
		s_trans[4].tx_buf = &(send_buf[NEURONSPI_FIRST_MESSAGE_LENGTH + 128]);
		s_trans[4].rx_buf = &(recv_buf[NEURONSPI_FIRST_MESSAGE_LENGTH + 128]);
	}
	if (len > NEURONSPI_FIRST_MESSAGE_LENGTH + 192) {
		cnt++;
		s_trans[4].len = 64;
		s_trans[5].len = len - 192;
		s_trans[5].tx_buf = &(send_buf[NEURONSPI_FIRST_MESSAGE_LENGTH + 192]);
		s_trans[5].rx_buf = &(recv_buf[NEURONSPI_FIRST_MESSAGE_LENGTH + 192]);
	}
	if (len > NEURONSPI_FIRST_MESSAGE_LENGTH + 256) {
		cnt++;
		s_trans[5].len = 64;
		s_trans[6].len = len - 256;
		s_trans[6].tx_buf = &(send_buf[NEURONSPI_FIRST_MESSAGE_LENGTH + 256]);
		s_trans[6].rx_buf = &(recv_buf[NEURONSPI_FIRST_MESSAGE_LENGTH + 256]);
	}
	*/

    //s_trans_w.cs_change = 1;
    /*
    for (i = 0; i < trans_count; i++) {
    	spi_message_add_tail(&(s_trans[i]), &s_msg);
    }
*/
    //spi_message_add_tail(&s_trans_w, &s_msg);
    //spi_write_then_read(private_data, txbuf, n_tx, rxbuf, n_rx);
    spi_sync(spi_dev, &s_msg);
    for (i = 0; i < trans_count; i++) {
    	spi_transfer_del(&(s_trans[i]));
    }
    //if (len > 64)
    printk(KERN_INFO "LONG MESSAGE - %d:\n\t%100ph\n\t%100ph\n\t%100ph\n\t%100ph\n", len,recv_buf, &recv_buf[64], &recv_buf[128], &recv_buf[192]);
    mutex_unlock(&neuronspi_master_mutex);
}

/*
static int neuronspi_line(struct uart_port *port)
{
	struct neuronspi_one *one = to_neuronspi_one(port, port);

	return one->line;
}
*/
/*
static u8 neuronspi_port_read(struct uart_port *port, u8 reg)
{
	struct neuronspi_port *s = dev_get_drvdata(port->dev);
	unsigned int val = 0;
	const u8 line = neuronspi_line(port);

	regmap_read(s->regmap, (reg << SC16IS7XX_REG_SHIFT) | line, &val);

	return val;
}

static void neuronspi_port_write(struct uart_port *port, u8 reg, u8 val)
{
	struct neuronspi_port *s = dev_get_drvdata(port->dev);
	const u8 line = neuronspi_line(port);

	regmap_write(s->regmap, (reg << SC16IS7XX_REG_SHIFT) | line, val);
}
*/
static void neuronspi_fifo_read(struct uart_port *port, unsigned int rxlen)
{
	//struct neuronspi_port *s = dev_get_drvdata(port->dev);
	//const u8 line = neuronspi_line(port);
	//u8 addr = line;

    SPI_READ_STR(line, s->buf, rxlen);
}

static void neuronspi_fifo_write(struct uart_port *port, u8 to_send)
{
	//struct neuronspi_port *s = dev_get_drvdata(port->dev);
	//const u8 line = neuronspi_line(port);
	//u8 addr = line;
    SPI_WRITE_STR(line, s->buf, to_send);
}

/*
static void neuronspi_port_update(struct uart_port *port, u8 reg,
				  u8 mask, u8 val)
{
	struct neuronspi_port *s = dev_get_drvdata(port->dev);
	const u8 line = neuronspi_line(port);

	regmap_update_bits(s->regmap, (reg << SC16IS7XX_REG_SHIFT) | line,
			   mask, val);
}
*/
static int neuronspi_alloc_line(void)
{
	int i;

	BUILD_BUG_ON(NEURONSPI_MAX_DEVS > BITS_PER_LONG);

	for (i = 0; i < NEURONSPI_MAX_DEVS; i++)
		if (!test_and_set_bit(i, &neuronspi_lines))
			break;

	return i;
}

static void neuronspi_power(struct uart_port *port, int on)
{
    /* Do nothing */
}


static const struct neuronspi_devtype b1000_1_devtype = {
	.name		= "spi",
//	.nr_gpio	= 8,
//	.nr_uart	= 1,
};

static bool neuronspi_regmap_volatile(struct device *dev, unsigned int reg)
{
    return true;
/*	switch (reg >> SC16IS7XX_REG_SHIFT) {
	case SC16IS7XX_RHR_REG:
	case SC16IS7XX_IIR_REG:
	case SC16IS7XX_LSR_REG:
	case SC16IS7XX_MSR_REG:
	case SC16IS7XX_TXLVL_REG:
	case SC16IS7XX_RXLVL_REG:
	case SC16IS7XX_IOSTATE_REG:
		return true;
	default:
		break;
	}
	return false;
     */ 
}

static bool neuronspi_regmap_precious(struct device *dev, unsigned int reg)
{
	return true;
/*	switch (reg >> SC16IS7XX_REG_SHIFT) {
	case SC16IS7XX_RHR_REG:
	default:
		break;
	}
	return false;*/
}

static int neuronspi_set_baud(struct uart_port *port, int baud)
{
	//struct neuronspi_port *s = dev_get_drvdata(port->dev);
	//u8 lcr;
	u8 prescaler = 0;
	unsigned long clk = port->uartclk, div = clk / 16 / baud;

	if (div > 0xffff) {
		prescaler = SC16IS7XX_MCR_CLKSEL_BIT;
		div /= 4;
	}


    SPI_WRITE(100, baud_mode)

	return baud;
}

static void neuronspi_handle_rx(struct uart_port *port, unsigned int rxlen,
				unsigned int iir)
{
	struct neuronspi_uart_port *s = dev_get_drvdata(port->dev);
	//unsigned int lsr = 0;
	unsigned int ch, flag, bytes_read, i;
	//bool read_lsr = (iir == SC16IS7XX_IIR_RLSE_SRC) ? true : false;

/*	if (unlikely(rxlen >= sizeof(s->buf))) {
		dev_warn_ratelimited(port->dev,
				     "ttySC%i: Possible RX FIFO overrun: %d\n",
				     port->line, rxlen);
		port->icount.buf_overrun++;
		rxlen = sizeof(s->buf);
	}
*/
	while (rxlen) {
		/* Only read lsr if there are possible errors in FIFO */
		//if (read_lsr) {
		//	lsr = neuronspi_port_read(port, SC16IS7XX_LSR_REG);
		//	if (!(lsr & SC16IS7XX_LSR_FIFOE_BIT))
		//		read_lsr = false; /* No errors left in FIFO */
		//} else
		//	lsr = 0;

		//if (read_lsr) {
		//	s->buf[0] = neuronspi_port_read(port, SC16IS7XX_RHR_REG);
		//	bytes_read = 1;
		//} else {
			neuronspi_fifo_read(port, rxlen);
			bytes_read = rxlen;
		//}

		//lsr &= SC16IS7XX_LSR_BRK_ERROR_MASK;

		port->icount.rx++;
		flag = TTY_NORMAL;
        /*
		if (unlikely(lsr)) {
			if (lsr & SC16IS7XX_LSR_BI_BIT) {
				port->icount.brk++;
				if (uart_handle_break(port))
					continue;
			} else if (lsr & SC16IS7XX_LSR_PE_BIT)
				port->icount.parity++;
			else if (lsr & SC16IS7XX_LSR_FE_BIT)
				port->icount.frame++;
			else if (lsr & SC16IS7XX_LSR_OE_BIT)
				port->icount.overrun++;

			lsr &= port->read_status_mask;
			if (lsr & SC16IS7XX_LSR_BI_BIT)
				flag = TTY_BREAK;
			else if (lsr & SC16IS7XX_LSR_PE_BIT)
				flag = TTY_PARITY;
			else if (lsr & SC16IS7XX_LSR_FE_BIT)
				flag = TTY_FRAME;
			else if (lsr & SC16IS7XX_LSR_OE_BIT)
				flag = TTY_OVERRUN;
		}
        */
		for (i = 0; i < bytes_read; ++i) {
			ch = s->buf[i];
			if (uart_handle_sysrq_char(port, ch))
				continue;

			//if (lsr & port->ignore_status_mask)
			//	continue;

			uart_insert_char(port, 0/*lsr*/, 0/*SC16IS7XX_LSR_OE_BIT*/, ch,
					 flag);
		}
		rxlen -= bytes_read;
	}

	tty_flip_buffer_push(&port->state->port);
}

static void neuronspi_handle_tx(struct uart_port *port)
{
	struct neuronspi_uart_port *s = dev_get_drvdata(port->dev);
	struct circ_buf *xmit = &port->state->xmit;
	unsigned int txlen, to_send, i;

	if (unlikely(port->x_char)) {
		SPI_WRITE_CHAR(port->x_char);
		port->icount.tx++;
		port->x_char = 0;
		return;
	}

	if (uart_circ_empty(xmit) || uart_tx_stopped(port))
		return;

	/* Get length of data pending in circular buffer */
	to_send = uart_circ_chars_pending(xmit);
	if (likely(to_send)) {
		/* Limit to size of TX FIFO */
		//txlen = SPI_READ_REG(txcount, port->);
		to_send = (to_send > txlen) ? txlen : to_send;

		/* Add data to send */
		port->icount.tx += to_send;

		/* Convert to linear buffer */
		for (i = 0; i < to_send; ++i) {
			s->buf[i] = xmit->buf[xmit->tail];
			xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE - 1);
		}

		neuronspi_fifo_write(port, to_send);
	}

	if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
		uart_write_wakeup(port);
}

static void neuronspi_port_irq(struct neuronspi_uart_port *s, int portno)
{
	struct uart_port *port = &s->p[portno].port;

	do {
		unsigned int interrupt_status, rxlen;
        //interrupt_status, rxlen = SPI_IDLE();
		if (interrupt_status & NEURON_INT_NO_INT_BIT) // ignore some bits
			break;
        interrupt_status &= NEURON_INT_ID_MASK;       // check only some bits
		switch (interrupt_status) {
		case NEURON_INT_RX_NOT_EMPTY:
		case NEURON_INT_RX_MODBUS:
			//rxlen =  //neuronspi_port_read(port, SC16IS7XX_RXLVL_REG);
			if (rxlen)
				neuronspi_handle_rx(port, rxlen, interrupt_status);
			break;
        
		case NEURON_INT_DI_CHANGED:
			break;
		case NEURON_INT_TX_FINISHED:
			neuronspi_handle_tx(port);
			break;
		default:
			dev_err_ratelimited(port->dev,
					    "ttySC%i: Unexpected interrupt: %x",
					    port->line, interrupt_status);
			break;
		}
	} while (1);
}

static void neuronspi_ist(struct kthread_work *ws)
{
	/*struct neuronspi_uart_port *s = to_neuronspi_port(ws, irq_work);
	int i;

	for (i = 0; i < s->devtype->nr_uart; ++i)
		neuronspi_port_irq(s, i);
		*/
}

static irqreturn_t neuronspi_irq(int irq, void *dev_id)
{
	struct neuronspi_uart_port *s = (struct neuronspi_port *)dev_id;

	kthread_queue_work(&s->kworker, &s->irq_work);

	return IRQ_HANDLED;
}

static void neuronspi_tx_proc(struct kthread_work *ws)
{
	struct uart_port *port = &(to_neuronspi_one(ws, tx_work)->port);
/*
	if ((port->rs485.flags & SER_RS485_ENABLED) &&
	    (port->rs485.delay_rts_before_send > 0))
		msleep(port->rs485.delay_rts_before_send);
*/
	neuronspi_handle_tx(port);
}

/*
static void neuronspi_reconf_rs485(struct uart_port *port)
{
    const u32 mask = SC16IS7XX_EFCR_AUTO_RS485_BIT |
			 SC16IS7XX_EFCR_RTS_INVERT_BIT;
	u32 efcr = 0;
	struct serial_rs485 *rs485 = &port->rs485;
	unsigned long irqflags;

	spin_lock_irqsave(&port->lock, irqflags);
	if (rs485->flags & SER_RS485_ENABLED) {
		efcr |=	SC16IS7XX_EFCR_AUTO_RS485_BIT;

		if (rs485->flags & SER_RS485_RTS_AFTER_SEND)
			efcr |= SC16IS7XX_EFCR_RTS_INVERT_BIT;
	}
	spin_unlock_irqrestore(&port->lock, irqflags);

	neuronspi_port_update(port, SC16IS7XX_EFCR_REG, mask, efcr);
}
*/ 

/*
static void neuronspi_reg_proc(struct kthread_work *ws)
{
	struct neuronspi_one *one = to_neuronspi_one(ws, reg_work);
	struct neuronspi_one_config config;
	unsigned long irqflags;

	spin_lock_irqsave(&one->port.lock, irqflags);
	config = one->config;
	memset(&one->config, 0, sizeof(one->config));
	spin_unlock_irqrestore(&one->port.lock, irqflags);

	if (config.flags & SC16IS7XX_RECONF_MD)
		neuronspi_port_update(&one->port, SC16IS7XX_MCR_REG,
				      SC16IS7XX_MCR_LOOP_BIT,
				      (one->port.mctrl & TIOCM_LOOP) ?
				      SC16IS7XX_MCR_LOOP_BIT : 0);

	if (config.flags & SC16IS7XX_RECONF_IER)
		neuronspi_port_update(&one->port, SC16IS7XX_IER_REG,
				      config.ier_clear, 0);

	if (config.flags & SC16IS7XX_RECONF_RS485)
		neuronspi_reconf_rs485(&one->port);
}
*/

/*
static void neuronspi_ier_clear(struct uart_port *port, u8 bit)
{
	struct neuronspi_port *s = dev_get_drvdata(port->dev);
	struct neuronspi_one *one = to_neuronspi_one(port, port);

	one->config.flags |= SC16IS7XX_RECONF_IER;
	one->config.ier_clear |= bit;
	queue_kthread_work(&s->kworker, &one->reg_work);
}
*/

static void neuronspi_stop_tx(struct uart_port *port)
{
    // ToDo : create new opcode / coil?
//	neuronspi_ier_clear(port, SC16IS7XX_IER_THRI_BIT);
}

static void neuronspi_stop_rx(struct uart_port *port)
{
    // ToDo : create new opcode / coil?
//	neuronspi_ier_clear(port, SC16IS7XX_IER_RDI_BIT);
}

static void neuronspi_start_tx(struct uart_port *port)
{
	struct neuronspi_uart_port *s = dev_get_drvdata(port->dev);
	struct neuronspi_one *one = to_neuronspi_one(port, port);

	kthread_queue_work(&s->kworker, &one->tx_work);
}

static unsigned int neuronspi_tx_empty(struct uart_port *port)
{
	unsigned int lsr;

	SPI_READ_REG(111, &lsr); //neuronspi_port_read(port, SC16IS7XX_LSR_REG); ToDo!!!

	return (lsr) ? TIOCSER_TEMT : 0;
}

static unsigned int neuronspi_get_mctrl(struct uart_port *port)
{
	/* DCD and DSR are not wired and CTS/RTS is handled automatically
	 * so just indicate DSR and CAR asserted
	 */
	return TIOCM_DSR | TIOCM_CAR;
}

static void neuronspi_set_mctrl(struct uart_port *port, unsigned int mctrl)
{
/*	struct neuronspi_port *s = dev_get_drvdata(port->dev);
	struct neuronspi_one *one = to_neuronspi_one(port, port);

	one->config.flags |= SC16IS7XX_RECONF_MD;
	queue_kthread_work(&s->kworker, &one->reg_work);*/
}

static void neuronspi_break_ctl(struct uart_port *port, int break_state)
{
	/*neuronspi_port_update(port, SC16IS7XX_LCR_REG,
			      SC16IS7XX_LCR_TXBREAK_BIT,
			      break_state ? SC16IS7XX_LCR_TXBREAK_BIT : 0);*/
}

static void neuronspi_set_termios(struct uart_port *port,
				  struct ktermios *termios,
				  struct ktermios *old)
{
	//struct neuronspi_port *s = dev_get_drvdata(port->dev);
	unsigned int lcr;//, flow = 0;
	int baud;

	/* Mask termios capabilities we don't support */
	termios->c_cflag &= ~CMSPAR;

	/* Word size */
	switch (termios->c_cflag & CSIZE) {
	case CS5:
		lcr = SC16IS7XX_LCR_WORD_LEN_5;
		break;
	case CS6:
		lcr = SC16IS7XX_LCR_WORD_LEN_6;
		break;
	case CS7:
		lcr = SC16IS7XX_LCR_WORD_LEN_7;
		break;
	case CS8:
		lcr = SC16IS7XX_LCR_WORD_LEN_8;
		break;
	default:
		lcr = SC16IS7XX_LCR_WORD_LEN_8;
		termios->c_cflag &= ~CSIZE;
		termios->c_cflag |= CS8;
		break;
	}

	/* Parity */
	if (termios->c_cflag & PARENB) {
		/*lcr |= SC16IS7XX_LCR_PARITY_BIT;*/
		if (!(termios->c_cflag & PARODD)) {
			/*lcr |= SC16IS7XX_LCR_EVENPARITY_BIT;*/
            
        }
	}

	/* Stop bits */
	//if (termios->c_cflag & CSTOPB)
	//	lcr |= SC16IS7XX_LCR_STOPLEN_BIT; /* 2 stops */

	/* Set read status mask */
	//port->read_status_mask = SC16IS7XX_LSR_OE_BIT;
	//if (termios->c_iflag & INPCK)
	//	port->read_status_mask |= SC16IS7XX_LSR_PE_BIT |
	//				  SC16IS7XX_LSR_FE_BIT;
	//if (termios->c_iflag & (BRKINT | PARMRK))
	//	port->read_status_mask |= SC16IS7XX_LSR_BI_BIT;

	/* Set status ignore mask */
	port->ignore_status_mask = 0;
	//if (termios->c_iflag & IGNBRK)
	//	port->ignore_status_mask |= SC16IS7XX_LSR_BI_BIT;
	//if (!(termios->c_cflag & CREAD))
	//	port->ignore_status_mask |= SC16IS7XX_LSR_BRK_ERROR_MASK;

	//neuronspi_port_write(port, SC16IS7XX_LCR_REG,
	//		     SC16IS7XX_LCR_CONF_MODE_B);

	/* Configure flow control */
	//regcache_cache_bypass(s->regmap, true);
	//neuronspi_port_write(port, SC16IS7XX_XON1_REG, termios->c_cc[VSTART]);
	//neuronspi_port_write(port, SC16IS7XX_XOFF1_REG, termios->c_cc[VSTOP]);
	//if (termios->c_cflag & CRTSCTS)
	//	flow |= SC16IS7XX_EFR_AUTOCTS_BIT |
	//		SC16IS7XX_EFR_AUTORTS_BIT;
	//if (termios->c_iflag & IXON)
	//	flow |= SC16IS7XX_EFR_SWFLOW3_BIT;
	//if (termios->c_iflag & IXOFF)
	//	flow |= SC16IS7XX_EFR_SWFLOW1_BIT;

	//neuronspi_port_write(port, SC16IS7XX_EFR_REG, flow);
	//regcache_cache_bypass(s->regmap, false);

	/* Update LCR register */
	//neuronspi_port_write(port, SC16IS7XX_LCR_REG, lcr);

	/* Get baud rate generator configuration */
	baud = uart_get_baud_rate(port, termios, old,
				  port->uartclk / 16 / 4 / 0xffff,
				  port->uartclk / 16);

	/* Setup baudrate generator */
	baud = neuronspi_set_baud(port, baud);

	/* Update timeout according to new baud rate */
	uart_update_timeout(port, termios->c_cflag, baud);
}

static int neuronspi_config_rs485(struct uart_port *port,
				  struct serial_rs485 *rs485)
{
    //struct neuronspi_port *s = dev_get_drvdata(port->dev);
	struct neuronspi_one *one = to_neuronspi_one(port, port);
    /*
	if (rs485->flags & SER_RS485_ENABLED) {
		bool rts_during_rx, rts_during_tx;

		rts_during_rx = rs485->flags & SER_RS485_RTS_AFTER_SEND;
		rts_during_tx = rs485->flags & SER_RS485_RTS_ON_SEND;

		if (rts_during_rx == rts_during_tx)
			dev_err(port->dev,
				"unsupported RTS signalling on_send:%d after_send:%d - exactly one of RS485 RTS flags should be set\n",
				rts_during_tx, rts_during_rx);

		//
		// RTS signal is handled by HW, it's timing can't be influenced.
		// However, it's sometimes useful to delay TX even without RTS
		// control therefore we try to handle .delay_rts_before_send.
		//
    
		if (rs485->delay_rts_after_send)
			return -EINVAL;
	}
    */
	port->rs485 = *rs485;
	one->config.flags |= SC16IS7XX_RECONF_RS485;
	//queue_kthread_work(&s->kworker, &one->reg_work);
	return 0;
}

// Initialise the module
static int neuronspi_startup(struct uart_port *port)
{
	//struct neuronspi_port *s = dev_get_drvdata(port->dev);
	//unsigned int val;

	neuronspi_power(port, 1);

	/* Reset FIFOs*/
	/*val = SC16IS7XX_FCR_RXRESET_BIT | SC16IS7XX_FCR_TXRESET_BIT;
	neuronspi_port_write(port, SC16IS7XX_FCR_REG, val);
	udelay(5);
	neuronspi_port_write(port, SC16IS7XX_FCR_REG,
			     SC16IS7XX_FCR_FIFO_BIT);
    */
	/* Enable EFR */
    /*
	neuronspi_port_write(port, SC16IS7XX_LCR_REG,
			     SC16IS7XX_LCR_CONF_MODE_B);

	regcache_cache_bypass(s->regmap, true);
    */
	/* Enable write access to enhanced features and internal clock div */
	/*
    neuronspi_port_write(port, SC16IS7XX_EFR_REG,
			     SC16IS7XX_EFR_ENABLE_BIT);
    */
	/* Enable TCR/TLR */
	/*
    neuronspi_port_update(port, SC16IS7XX_MCR_REG,
			      SC16IS7XX_MCR_TCRTLR_BIT,
			      SC16IS7XX_MCR_TCRTLR_BIT);
    */

	/* Now, initialize the UART */
	/*
    neuronspi_port_write(port, SC16IS7XX_LCR_REG, SC16IS7XX_LCR_WORD_LEN_8);
    */
	/* Enable the Rx and Tx FIFO */
    /*
	neuronspi_port_update(port, SC16IS7XX_EFCR_REG,
			      SC16IS7XX_EFCR_RXDISABLE_BIT |
			      SC16IS7XX_EFCR_TXDISABLE_BIT,
			      0);
    */
	/* Enable RX, TX, CTS change interrupts */
    /*
	val = SC16IS7XX_IER_RDI_BIT | SC16IS7XX_IER_THRI_BIT |
	      SC16IS7XX_IER_CTSI_BIT;
	neuronspi_port_write(port, SC16IS7XX_IER_REG, val);
    */
	return 0;
}

static void neuronspi_shutdown(struct uart_port *port)
{
	struct neuronspi_uart_port *s = dev_get_drvdata(port->dev);

	/* Disable all interrupts */
    SPI_WRITE(1005,!interrupt_uart);
	/*neuronspi_port_write(port, SC16IS7XX_IER_REG, 0);*/
	/* Disable TX/RX */
	/*neuronspi_port_update(port, SC16IS7XX_EFCR_REG,
			      SC16IS7XX_EFCR_RXDISABLE_BIT |
			      SC16IS7XX_EFCR_TXDISABLE_BIT,
			      SC16IS7XX_EFCR_RXDISABLE_BIT |
			      SC16IS7XX_EFCR_TXDISABLE_BIT);

	*/
    neuronspi_power(port, 0);

	kthread_flush_worker(&s->kworker);
}

static const char *neuronspi_type(struct uart_port *port)
{
	struct neuronspi_uart_port *s = dev_get_drvdata(port->dev);

	return (port->type == PORT_SC16IS7XX) ? s->devtype->name : NULL;
}

static int neuronspi_request_port(struct uart_port *port)
{
	/* Do nothing */
	return 0;
}

static void neuronspi_config_port(struct uart_port *port, int flags)
{
	if (flags & UART_CONFIG_TYPE)
		port->type = PORT_SC16IS7XX;
}

static int neuronspi_verify_port(struct uart_port *port,
				 struct serial_struct *s)
{
	if ((s->type != PORT_UNKNOWN) && (s->type != PORT_SC16IS7XX))
		return -EINVAL;
	if (s->irq != port->irq)
		return -EINVAL;

	return 0;
}

static void neuronspi_pm(struct uart_port *port, unsigned int state,
			 unsigned int oldstate)
{
	neuronspi_power(port, (state == UART_PM_STATE_ON) ? 1 : 0);
}

static void neuronspi_null_void(struct uart_port *port)
{
	/* Do nothing */
}



#ifdef CONFIG_GPIOLIB
static int neuronspi_gpio_get(struct gpio_chip *chip, unsigned offset)
{
	unsigned int val;
	//struct neuronspi_port *s = container_of(chip, struct neuronspi_port,
	//					gpio);
	//struct uart_port *port = &s->p[0].port;

	//val = neuronspi_port_read(port, SC16IS7XX_IOSTATE_REG);

    SPI_READ_BIT(111, &val);
	return !!(val & BIT(offset));
}

static void neuronspi_gpio_set(struct gpio_chip *chip, unsigned offset, int val)
{
	//struct neuronspi_port *s = container_of(chip, struct neuronspi_port,
	//					gpio);
	//struct uart_port *port = &s->p[0].port;

	
    //neuronspi_port_update(port, SC16IS7XX_IOSTATE_REG, BIT(offset),
	//		      val ? BIT(offset) : 0);
    
    SPI_WRITE_COIL(1111,1);
}

static int neuronspi_gpio_direction_input(struct gpio_chip *chip,
					  unsigned offset)
{
	//struct neuronspi_port *s = container_of(chip, struct neuronspi_port,
	//					gpio);
	//struct uart_port *port = &s->p[0].port;

	//neuronspi_port_update(port, SC16IS7XX_IODIR_REG, BIT(offset), 0);

	return 0;
}

static int neuronspi_gpio_direction_output(struct gpio_chip *chip,
					   unsigned offset, int val)
{
	//struct neuronspi_port *s = container_of(chip, struct neuronspi_port,
	//					gpio);
	//struct uart_port *port = &s->p[0].port;

	//neuronspi_port_update(port, SC16IS7XX_IOSTATE_REG, BIT(offset),
	//		      val ? BIT(offset) : 0);
	//neuronspi_port_update(port, SC16IS7XX_IODIR_REG, BIT(offset),
	//		      BIT(offset));

	return 0;
}
#endif

static int neuronspi_uart_probe(struct device *dev,
			   const struct neuronspi_devtype *devtype,
			   struct regmap *regmap, int irq, unsigned long flags)
{
	struct sched_param sched_param = { .sched_priority = MAX_RT_PRIO / 2 };
	unsigned long freq, *pfreq = dev_get_platdata(dev);
	int i, ret;
	struct neuronspi_uart_port *s;

	if (IS_ERR(regmap))
		return PTR_ERR(regmap);

	// Alloc port structure
	s = devm_kzalloc(dev, sizeof(*s) +
			 sizeof(struct neuronspi_one) * devtype->nr_uart,
			 GFP_KERNEL);
	if (!s) {
		dev_err(dev, "Error allocating port structure\n");
		return -ENOMEM;
	}

	s->clk = devm_clk_get(dev, NULL);
	if (IS_ERR(s->clk)) {
		if (pfreq)
			freq = *pfreq;
		else
			return PTR_ERR(s->clk);
	} else {
		clk_prepare_enable(s->clk);
		freq = clk_get_rate(s->clk);
	}

	s->regmap = regmap;
	s->devtype = devtype;
	//dev_set_drvdata(dev, s);

	//kthread_init_worker(&s->kworker);
	//kthread_init_work(&s->irq_work, neuronspi_ist);
	//s->kworker_task = kthread_run(kthread_worker_fn, &s->kworker,
	//			      "neuronspi");
	//if (IS_ERR(s->kworker_task)) {
	//	ret = PTR_ERR(s->kworker_task);
	//	goto out_clk;
	//}
	//sched_setscheduler(s->kworker_task, SCHED_FIFO, &sched_param);


	for (i = 0; i < devtype->nr_uart; ++i) {
		s->p[i].line		= i;
		// Initialize port data
		s->p[i].port.dev	= dev;
		s->p[i].port.irq	= irq;
		s->p[i].port.type	= PORT_SC16IS7XX;
		s->p[i].port.fifosize	= NEURON_FIFO_SIZE;
		s->p[i].port.flags	= UPF_FIXED_TYPE | UPF_LOW_LATENCY;
		s->p[i].port.iotype	= UPIO_PORT;
		s->p[i].port.uartclk	= freq;
		s->p[i].port.rs485_config = neuronspi_config_rs485;
		s->p[i].port.ops	= &neuronspi_uart_ops;
		s->p[i].port.line	= neuronspi_alloc_line();
		if (s->p[i].port.line >= NEURONSPI_MAX_DEVS) {
			ret = -ENOMEM;
			goto out_ports;
		}

		// Disable all interrupts
        SPI_WRITE(MASK_INTERRUPT, port[1])
		//neuronspi_port_write(&s->p[i].port, SC16IS7XX_IER_REG, 0);
		// Disable TX/RX
		//neuronspi_port_write(&s->p[i].port, SC16IS7XX_EFCR_REG,
		//		     SC16IS7XX_EFCR_RXDISABLE_BIT |
		//		     SC16IS7XX_EFCR_TXDISABLE_BIT);
		// Initialize kthread work structs
		//kthread_init_work(&s->p[i].tx_work, neuronspi_tx_proc);
		//kthread_init_work(&s->p[i].reg_work, neuronspi_reg_proc);
		// Register port
		//uart_add_one_port(&neuronspi_uart, &s->p[i].port);
		// Go to suspend mode
		neuronspi_power(&s->p[i].port, 0);
	}
	return 0;
	/// Setup interrupt
	//ret = devm_request_irq(dev, irq, neuronspi_irq,
	//		       flags, dev_name(dev), s);
	if (!ret)
		return 0;

out_ports:
	for (i--; i >= 0; i--) {
		uart_remove_one_port(&neuronspi_uart, &s->p[i].port);
		clear_bit(s->p[i].port.line, &neuronspi_lines);
	}

out_thread:
	kthread_stop(s->kworker_task);

out_clk:
	if (!IS_ERR(s->clk))
		clk_disable_unprepare(s->clk);

	return ret;
}

/*
static int neuronspi_remove(struct device *dev)
{
	struct neuronspi_port *s = dev_get_drvdata(dev);
	int i;

#ifdef CONFIG_GPIOLIB
	if (s->devtype->nr_gpio)
		gpiochip_remove(&s->gpio);
#endif

	for (i = 0; i < s->devtype->nr_uart; i++) {
		uart_remove_one_port(&neuronspi_uart, &s->p[i].port);
		clear_bit(s->p[i].port.line, &neuronspi_lines);
		neuronspi_power(&s->p[i].port, 0);
	}

	kthread_flush_worker(&s->kworker);
	kthread_stop(s->kworker_task);

	if (!IS_ERR(s->clk))
		clk_disable_unprepare(s->clk);

	return 0;
}
*/
MODULE_DEVICE_TABLE(of, neuronspi_id_match);

static struct regmap_config regcfg = {
	.reg_bits = 7,
	.pad_bits = 1,
	.val_bits = 8,
	.cache_type = REGCACHE_RBTREE, //REGCACHE_BYPASS, //REGCACHE_RBTREE,
	.volatile_reg = neuronspi_regmap_volatile,
	.precious_reg = neuronspi_regmap_precious,
};
/*****
	Probe function of the child SPI driver
*/
static int neuronspi_spi_probe(struct spi_device *spi)
{
	const struct neuronspi_devtype *devtype;
	struct spi_master *master;
	struct neuronspi_driver_data *n_spi;
	unsigned long flags = 0;
	struct regmap *regmap;
	int ret;
	int i;
	n_spi = kzalloc(sizeof *n_spi, GFP_KERNEL);
	if (!n_spi)
		return -ENOMEM;
	printk(KERN_INFO "NEURONSPI: Neuronspi Probe Started\n");

	printk(KERN_INFO "NEURONSPI: Chip Max Hz-%d\n",spi->master->max_speed_hz);
	/* Setup SPI bus */
	spi->bits_per_word	= 8;
	/* only supports mode 0 on SC16IS762 */
	spi->mode		= spi->mode ? : SPI_MODE_0;
	spi->max_speed_hz	= spi->max_speed_hz ? : 12000000;
	ret = spi_setup(spi);
	if (ret)
		return ret;
	printk(KERN_INFO "NEURONSPI: Chip Max Hz-%d %d\n", spi->master->max_speed_hz, spi->max_speed_hz);
	if (spi->dev.of_node) {
		const struct of_device_id *of_id =
			of_match_device(neuronspi_id_match, &spi->dev);
		if (!of_id) {
			printk(KERN_INFO "NEURONSPI: Probe %s does not match a device!\n", *(&spi->dev.of_node->full_name));
			return -ENODEV;
		}
		devtype = (struct neuronspi_devtype *)of_id->data;
	} else {
		const struct spi_device_id *id_entry = spi_get_device_id(spi);
		devtype = (struct neuronspi_devtype *)id_entry->driver_data;
		flags = IRQF_TRIGGER_FALLING;
	}

	// We perform an initial probe of registers 1000-1004 to identify the device, using a premade message
	n_spi->recv_buf = kzalloc(NEURONSPI_BUFFER_MAX, GFP_KERNEL);
	n_spi->send_buf = kzalloc(NEURONSPI_BUFFER_MAX, GFP_KERNEL);
	n_spi->first_probe_reply = kzalloc(NEURONSPI_PROBE_MESSAGE_LEN, GFP_KERNEL);	// allocate space for initial probe
	n_spi->second_probe_reply = kzalloc(NEURONSPI_UART_PROBE_MESSAGE_LEN, GFP_KERNEL); // allocate space for uart probe
	n_spi->spi_driver = &neuronspi_spi_driver;
	memcpy(n_spi->send_buf, &NEURONSPI_PROBE_MESSAGE, NEURONSPI_PROBE_MESSAGE_LEN);
	neuronspi_spi_send_message(spi, n_spi->send_buf, n_spi->first_probe_reply, NEURONSPI_PROBE_MESSAGE_LEN, NEURONSPI_DEFAULT_FREQ, 25, 1);

	// We perform a second probe of register 1031 to determine initial serial settings
	//memset(n_spi->send_buf, 0, NEURONSPI_BUFFER_MAX);
	//memcpy(n_spi->send_buf, &NEURONSPI_UART_PROBE_MESSAGE, NEURONSPI_UART_PROBE_MESSAGE_LEN);
	//neuronspi_spi_send_message(spi, n_spi->send_buf, n_spi->second_probe_reply, NEURONSPI_UART_PROBE_MESSAGE_LEN, NEURONSPI_DEFAULT_FREQ, 25);
	//memset(n_spi->send_buf, 0, NEURONSPI_BUFFER_MAX);

	if (n_spi->first_probe_reply[0] != 0) {
		printk(KERN_INFO "NEURONSPI: Probe detected Neuron v%d.%d on CS %d - reg1000: %x, reg1001: %x, reg1002: %x, reg1003: %x, reg1004: %x\n", n_spi->first_probe_reply[11],  n_spi->first_probe_reply[10], spi->chip_select,  n_spi->first_probe_reply[11] << 8 | n_spi->first_probe_reply[10], n_spi->first_probe_reply[13] << 8 | n_spi->first_probe_reply[12], n_spi->first_probe_reply[15] << 8 | n_spi->first_probe_reply[14], n_spi->first_probe_reply[17] << 8 | n_spi->first_probe_reply[16], n_spi->first_probe_reply[19] << 8 | n_spi->first_probe_reply[18]);
	}
	if (neuronspi_uart == NULL) {	// Register UART if not registered
		neuronspi_uart = kzalloc(sizeof(struct uart_driver), GFP_KERNEL);
		neuronspi_uart->owner		= THIS_MODULE;
		neuronspi_uart->dev_name	= "ttySC";
		neuronspi_uart->nr		= NEURONSPI_MAX_DEVS,
		ret = uart_register_driver(neuronspi_uart);
		if (ret) {
			pr_err("Registering UART driver failed\n");
		} else {
			printk(KERN_INFO "NEURONSPI: Uart driver registration finished\n");
		}
	}
	if (!(neuronspi_cdrv.major_number)) { // Register character device if it doesn't exist
		ret = char_register_driver();
		if (ret) {
			pr_err("Failed to register the neuronspi character driver, ERR:%d\n", ret);
		}
	}

	n_spi->char_driver = &neuronspi_cdrv;
	n_spi->serial_driver = neuronspi_uart;

	for (i = 0; i < NEURONSPI_MAX_DEVS; i++) {	// Assign to the last empty pointer
		if (neuronspi_s_dev[NEURONSPI_MAX_DEVS - i - 1] == NULL) {
			neuronspi_s_dev[NEURONSPI_MAX_DEVS - i - 1] = spi;
			spi_set_drvdata(neuronspi_s_dev[NEURONSPI_MAX_DEVS - i - 1], n_spi);
			break;
		}

	}
	//regcfg.max_register = (0xf << SC16IS7XX_REG_SHIFT) |
	//		      (devtype->nr_uart - 1);
	//regmap = devm_regmap_init_spi(spi, &regcfg);
	//neuronspi_uart_probe(spi, devtype, regmap, spi->irq, flags);
	return ret;
}

static int neuronspi_spi_remove(struct spi_device *spi)
{
	struct neuronspi_driver_data *n_spi = spi_get_drvdata(spi);
	kfree(n_spi->send_buf);
	n_spi->send_buf = NULL;
	kfree(n_spi->recv_buf);
	n_spi->recv_buf = NULL;
	kfree(n_spi);
	return 0;// neuronspi_remove(&spi->dev);
}


static int char_register_driver(void) {
	int ret = 0;
	printk(KERN_INFO "NEURONSPI: Initialising Character Device\n");
	// Try to dynamically allocate a major number for the device -- more difficult but worth it
	neuronspi_cdrv.major_number = register_chrdev(0, NEURON_DEVICE_NAME, &file_ops);
	if (neuronspi_cdrv.major_number < 0){
	   printk(KERN_ALERT "NEURONSPI failed to register a major number\n");
	   return neuronspi_cdrv.major_number;
	}
	printk(KERN_INFO "NEURONSPI: registered correctly with major number %d\n", neuronspi_cdrv.major_number);

	// Register the device class
	neuronspi_cdrv.driver_class = class_create(THIS_MODULE, NEURON_DEVICE_CLASS);
	if (IS_ERR(neuronspi_cdrv.driver_class)){                // Check for error and clean up if there is
		unregister_chrdev(neuronspi_cdrv.major_number, NEURON_DEVICE_NAME);
		printk(KERN_ALERT "Failed to register device class\n");
		return PTR_ERR(neuronspi_cdrv.driver_class);          // Correct way to return an error on a pointer
	}
	printk(KERN_INFO "NEURONSPI: device class registered correctly\n");

	// Register the device driver
	neuronspi_cdrv.dev_name = device_create(neuronspi_cdrv.driver_class, NULL, MKDEV(neuronspi_cdrv.major_number, 0), NULL, NEURON_DEVICE_NAME);
	if (IS_ERR(neuronspi_cdrv.dev_name)){               // Clean up if there is an error
		class_destroy(neuronspi_cdrv.driver_class);           // Repeated code but the alternative is goto statements
	    	unregister_chrdev(neuronspi_cdrv.major_number, NEURON_DEVICE_NAME);
	    	printk(KERN_ALERT "Failed to create the device\n");
	    	return PTR_ERR(neuronspi_cdrv.dev_name);
	}
	printk(KERN_INFO "NEURONSPI: device class created correctly\n"); // Made it! device was initialised
	turn_buf = kzalloc(NEURONSPI_BUFFER_MAX, GFP_KERNEL);
	return ret;
}

static int char_unregister_driver(void) {
	device_destroy(neuronspi_cdrv.driver_class, MKDEV(neuronspi_cdrv.major_number, 0));     // remove the device
	class_unregister(neuronspi_cdrv.driver_class);                          // unregister the device class
	class_destroy(neuronspi_cdrv.driver_class);                             // remove the device class
	unregister_chrdev(neuronspi_cdrv.major_number, NEURON_DEVICE_NAME);             // unregister the major number
	kfree(turn_buf);
	printk(KERN_INFO "NEURONSPI: Device unloaded successfully\n");
	return 0;
}

MODULE_ALIAS("spi:neuronspi");

static int __init neuronspi_init(void)
{
	mutex_init(&neuronspi_master_mutex);
	int ret = spi_register_driver(&neuronspi_spi_driver);
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
	spi_unregister_driver(&neuronspi_spi_driver);
	uart_unregister_driver(neuronspi_uart);
	kfree(neuronspi_uart);
	char_unregister_driver();
}
module_exit(neuronspi_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Tomas Knot <knot@faster.cz>");
MODULE_DESCRIPTION("UniPi Neuron driver");
