//#undef CONFIG_GPIOLIB
//#define CONFIG_GPIOLIB

#define ENABLE_CUSTOM_IOCTLS
#define DEBUG_INTERNAL_SYSFS
//#define DEBUG_TX_EMPTY
//#define WRITER_WQ
#define LSR_ISSUE

#define SPI_HWMODE
#define I2C_HWMODE
#define PWM_MODE

#ifdef ENABLE_CUSTOM_IOCTLS
#define MULTIDROP_ENABLE
#define FLASH_RW_DRIVER
#define SAVE_CONTROL
#endif

#include <linux/errno.h>
#include <linux/slab.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/serial.h>
#include <linux/usb.h>
#include <linux/usb/serial.h>
#include <linux/serial_reg.h>
#ifdef ENABLE_CUSTOM_IOCTLS
#include <linux/version.h>
#endif
#include <linux/module.h>
#include <linux/kfifo.h>
#include <linux/gpio.h>
#include <linux/uaccess.h>

#include <linux/spi/spi.h>
//#include <linux/spi/spi_bitbang.h>
//#include "spi_gpio.h"

#define DRIVER_VER "v1.02-20231205a1"

#include <linux/platform_device.h>
#if LINUX_VERSION_CODE <= KERNEL_VERSION(4, 17, 0)
#include <linux/i2c-gpio.h>
#else
#include <linux/platform_data/i2c-gpio.h>
#endif
#include <linux/i2c.h>
#include <linux/i2c-algo-bit.h>
#include <linux/gpio/machine.h>
#include <linux/pwm.h>

//#include "f75115.h"

#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 0, 0)
#define __dynamic_dev_dbg(...)

#undef dev_err
#define dev_err(dev, text, arg...)                                          \
	printk(KERN_INFO "%s %s: " #text, dev_driver_string(dev), __func__, \
	       ##arg)

#undef dev_warn
#define dev_warn(dev, text, arg...)                                         \
	printk(KERN_INFO "%s %s: " #text, dev_driver_string(dev), __func__, \
	       ##arg)

#undef dev_info
#define dev_info(dev, text, arg...)                                         \
	printk(KERN_INFO "%s %s: " #text, dev_driver_string(dev), __func__, \
	       ##arg)
#endif

#define VERSION "$Rev: 19 $"
#define GPIO_OUTPUT_FAST_MODE

/* Serial Port register Address */
#define SERIAL_BASE_ADDRESS 0x1200
#define RECEIVE_BUFFER_REGISTER (0x00 + SERIAL_BASE_ADDRESS)
#define TRANSMIT_HOLDING_REGISTER (0x00 + SERIAL_BASE_ADDRESS)
#define DIVISOR_LATCH_LSB (0x00 + SERIAL_BASE_ADDRESS)
#define INTERRUPT_ENABLE_REGISTER (0x01 + SERIAL_BASE_ADDRESS)
#define DIVISOR_LATCH_MSB (0x01 + SERIAL_BASE_ADDRESS)
#define INTERRUPT_IDENT_REGISTER (0x02 + SERIAL_BASE_ADDRESS)
#define FIFO_CONTROL_REGISTER (0x02 + SERIAL_BASE_ADDRESS)
#define LINE_CONTROL_REGISTER (0x03 + SERIAL_BASE_ADDRESS)
#define MODEM_CONTROL_REGISTER (0x04 + SERIAL_BASE_ADDRESS)
#define LINE_STATUS_REGISTER (0x05 + SERIAL_BASE_ADDRESS)
#define MODEM_STATUS_REGISTER (0x06 + SERIAL_BASE_ADDRESS)
#define CLK_SEL_REGISTER (0x08 + SERIAL_BASE_ADDRESS)
#define CONFIG1_REGISTER (0x09 + SERIAL_BASE_ADDRESS)
#define SADDRESS_REGISTER (0x0a + SERIAL_BASE_ADDRESS)
#define SADEN_REGISTER (0x0b + SERIAL_BASE_ADDRESS)

#define IER_DMA_TX_EN BIT(7)
#define IER_DMA_RX_EN BIT(6)

#define F75115_DEF_CONF_ADDRESS_START 0x3000
#define F75115_DEF_CONF_SIZE 12

#define F75115_CUSTOM_ADDRESS_START 0x2f00
#define F75115_CUSTOM_TOTAL_SIZE 0x10
#define F75115_CUSTOM_DATA_SIZE 0x10
#define F75115_CUSTOM_MAX_IDX \
	(F75115_CUSTOM_TOTAL_SIZE / F75115_CUSTOM_DATA_SIZE)
#define F75115_CUSTOM_NO_CUSTOM_DATA (-1)
#define F75115_CUSTOM_VALID_TOKEN 0xf0
#define F75115_CONF_OFFSET 1
#define F75115_CONF_SIZE 8

#define F75115_MAX_DATA_BLOCK 64
#define F75115_MAX_BUS_RETRY 2000

/* default URB timeout for usb operations */
#define F75115_USB_MAX_RETRY 10
#define F75115_USB_TIMEOUT 1000
#define F75115_CONTROL_BYTE 0x1B
#define F75115_SET_GET_REGISTER 0xA0

#define F75115_NUM_PORT 4
#define F75115_UNUSED_PORT 0xff
#define F75115_WRITE_BUFFER_SIZE 512
#define F75115_MAX_RECEIVE_BLOCK_SIZE 512

#define IC_NAME "f75115"
#define DRIVER_DESC \
	"Fintek USB to Rasperberry pin coverter Driver(F75115-Evaluation Board)"
#define FINTEK_VENDOR_ID_1 0x1934
#define FINTEK_VENDOR_ID_2 0x2C42
#define FINTEK_DEVICE_ID 0x1222 /* RS232 1 port */
#define F75115_MAX_TX_SIZE 100
#define F75115_FIFO_SIZE 128
#define F75115_RECEIVE_BLOCK_SIZE 128

#define F75115_TOKEN_RECEIVE 0x01
#define F75115_TOKEN_WRITE 0x02
#define F75115_TOKEN_TX_EMPTY 0x03
#define F75115_TOKEN_MSR_CHANGE 0x04

#define F75115_BUS_BUSY 0x03
#define F75115_BUS_IDLE 0x04
#define F75115_BUS_READ_DATA 0x1004
#define F75115_BUS_REG_STATUS 0x1003
#define F75115_BUS_REG_START 0x1002
#define F75115_BUS_REG_END 0x1001

#define F75115_CMD_READ 0x03
#define F75115_CMD_ENABLE_WR 0x06
#define F75115_CMD_PROGRAM 0x02
#define F75115_CMD_ERASE 0x20
#define F75115_CMD_READ_STATUS 0x05

#define F75115_MEDIA_BUSY_STATUS 0x03

#define F75115_1X_RXTRIGGER 0xc3
#define F75115_8X_RXTRIGGER 0xcf

#define F75115_DEFAULT_BAUD_RATE 9600
#define F75115_MAX_BAUDRATE 115200

#define F75115_DELAY_READ_MSR 10

#define F75115_MAX_PWM_DIV (1024 - 1)
#define F75115_MAX_PWN_CNT 4

#ifdef SAVE_CONTROL
#define PORT_NOSAVE BIT(0)
#endif

#ifdef ENABLE_CUSTOM_IOCTLS
#define FINTEK_MAGIC 'F'

#ifdef MULTIDROP_ENABLE
#define FINTEK_SET_MULTI_DROP_MODE _IOW(FINTEK_MAGIC, 0, int)
#define FINTEK_GET_MULTI_DROP_MODE _IOR(FINTEK_MAGIC, 1, int)
#define FINTEK_GET_SM2_STATE _IOR(FINTEK_MAGIC, 2, int)
#endif

#ifdef FLASH_RW_DRIVER
#define FINTEK_GET_DATA _IOR(FINTEK_MAGIC, 8, struct internal_data)
#define FINTEK_SET_DATA _IOW(FINTEK_MAGIC, 9, struct internal_data)
#define FINTEK_ERASE_DATA_PAGE _IOW(FINTEK_MAGIC, 10, struct internal_data)
#endif

#define FINTEK_GET_GPIO _IOWR(FINTEK_MAGIC, 11, struct gpio_access)
#define FINTEK_SET_GPIO _IOW(FINTEK_MAGIC, 12, struct gpio_access)
#endif

#define F75115_RS232_FLAG 0x00
#define F75115_RS485_FLAG 0x03
#define F75115_RS485_1_FLAG 0x01
#define F75115_MODE_MASK 0x03
#define F75115_PORT_CONF_RS485 BIT(0)
#define F75115_PORT_CONF_RS485_INVERT BIT(1)
#define F75115_PORT_CONF_DISABLE_PORT BIT(3)
#define F75115_PORT_CONF_NOT_EXIST_PORT BIT(7)
#define F75115_PORT_UNAVAILABLE \
	(F75115_PORT_CONF_DISABLE_PORT | F75115_PORT_CONF_NOT_EXIST_PORT)

#define F75115_RS485_MODE BIT(4)
#define F75115_RS485_INVERT BIT(5)

#define F75115_PIN_SET_DEFAULT 0x01
#define F75115_PIN_SET_MAX 0x07
#define F75115_PIN_SET_MIN 0x00

#define F75115_SCL1_PIN 32
#define F75115_SDA1_PIN 33
#define F75115_SCL2_PIN 18
#define F75115_SDA2_PIN 17
#define F75115_MAX_I2C_SET 2

#define F75115_SPI_MISO_PIN 14
#define F75115_SPI_MOSI_PIN 13
#define F75115_SPI_SCK_PIN 15
#define F75115_SPI_CS0_PIN 12
#define F75115_SPI_CS1_PIN 16
#define F75115_SPI_MAX_CS_NUM 2

/*
 * Clock rate selector, always or-ed with CLKSEL_ENABLE_UART to enable
 * UART functional.
 */
#define CLKSEL_ENABLE_UART BIT(0)
#define CLKSEL_1DOT846_MHZ CLKSEL_ENABLE_UART
#define CLKSEL_18DOT46_MHZ (BIT(1) | CLKSEL_ENABLE_UART)
#define CLKSEL_24_MHZ (BIT(2) | CLKSEL_ENABLE_UART)
#define CLKSEL_14DOT77_MHZ (BIT(1) | BIT(2) | CLKSEL_ENABLE_UART)
#define CLKSEL_TX_DELAY_1BIT BIT(3)

#ifndef C_CMSPAR
#define C_CMSPAR(tty) _C_FLAG((tty), CMSPAR)
#endif

#define F75115_GPIO_BASE            0
#define F75115_GPIO00   (F75115_GPIO_BASE + 0)
#define F75115_GPIO01   (F75115_GPIO_BASE + 1)
#define F75115_GPIO02   (F75115_GPIO_BASE + 2)
#define F75115_GPIO03   (F75115_GPIO_BASE + 3)
#define F75115_GPIO04   (F75115_GPIO_BASE + 4)
#define F75115_GPIO05   (F75115_GPIO_BASE + 5)
#define F75115_GPIO06   (F75115_GPIO_BASE + 6)
#define F75115_GPIO07   (F75115_GPIO_BASE + 7)

#define F75115_GPIO10   (F75115_GPIO_BASE + 1*8 + 0)
#define F75115_GPIO11   (F75115_GPIO_BASE + 1*8 + 1)
#define F75115_GPIO12   (F75115_GPIO_BASE + 1*8 + 2)
#define F75115_GPIO13   (F75115_GPIO_BASE + 1*8 + 3)
#define F75115_GPIO14   (F75115_GPIO_BASE + 1*8 + 4)
#define F75115_GPIO15   (F75115_GPIO_BASE + 1*8 + 5)
#define F75115_GPIO16   (F75115_GPIO_BASE + 1*8 + 6)
#define F75115_GPIO17   (F75115_GPIO_BASE + 1*8 + 7)

#define F75115_GPIO20   (F75115_GPIO_BASE + 2*8 + 0)
#define F75115_GPIO21   (F75115_GPIO_BASE + 2*8 + 1)
#define F75115_GPIO22   (F75115_GPIO_BASE + 2*8 + 2)
#define F75115_GPIO23   (F75115_GPIO_BASE + 2*8 + 3)
#define F75115_GPIO24   (F75115_GPIO_BASE + 2*8 + 4)
#define F75115_GPIO25   (F75115_GPIO_BASE + 2*8 + 5)
#define F75115_GPIO26   (F75115_GPIO_BASE + 2*8 + 6)
#define F75115_GPIO27   (F75115_GPIO_BASE + 2*8 + 7)

#define F75115_GPIO30   (F75115_GPIO_BASE + 3*8 + 0)
#define F75115_GPIO31   (F75115_GPIO_BASE + 3*8 + 1)
#define F75115_GPIO32   (F75115_GPIO_BASE + 3*8 + 2)
#define F75115_GPIO33   (F75115_GPIO_BASE + 3*8 + 3)
#define F75115_GPIO34   (F75115_GPIO_BASE + 3*8 + 4)
#define F75115_GPIO35   (F75115_GPIO_BASE + 3*8 + 5)
#define F75115_GPIO36   (F75115_GPIO_BASE + 3*8 + 6)
#define F75115_GPIO37   (F75115_GPIO_BASE + 3*8 + 7)

// For SDA0 and SCL0
//#define F75115_GPIO40  (F75115_GPIO_BASE + 4*8 + 0)
//#define F75115_GPIO41  (F75115_GPIO_BASE + 4*8 + 1)
#define F75115_GPIO42   (F75115_GPIO_BASE + 4*8 + 2)
#define F75115_GPIO43   (F75115_GPIO_BASE + 4*8 + 3)
#define F75115_GPIO44   (F75115_GPIO_BASE + 4*8 + 4)
#define F75115_GPIO45   (F75115_GPIO_BASE + 4*8 + 5)
#define F75115_GPIO46   (F75115_GPIO_BASE + 4*8 + 6)
#define F75115_GPIO47   (F75115_GPIO_BASE + 4*8 + 7)

#define F75115_GPIO50   (F75115_GPIO_BASE + 5*8 + 0)
#define F75115_GPIO51   (F75115_GPIO_BASE + 5*8 + 1)
#define F75115_GPIO52   (F75115_GPIO_BASE + 5*8 + 2)
#define F75115_GPIO53   (F75115_GPIO_BASE + 5*8 + 3)
#define F75115_GPIO54   (F75115_GPIO_BASE + 5*8 + 4)
#define F75115_GPIO55   (F75115_GPIO_BASE + 5*8 + 5)
// For TXD and RXD
//#define F75115_GPIO56   (F75115_GPIO_BASE + 5*8 + 6)
//#define F75115_GPIO57   (F75115_GPIO_BASE + 5*8 + 7)

static int full_uart = 0;
module_param(full_uart, int, S_IRUGO);
MODULE_PARM_DESC(full_uart, "Full UART function PIN");

static int en_i2c = 1;
module_param(en_i2c, int, S_IRUGO);
MODULE_PARM_DESC(en_i2c, "Enable I2C function PIN (default 3, bitwise)");

static int en_spi = 0;
module_param(en_spi, int, S_IRUGO);
MODULE_PARM_DESC(en_spi, "Enable SPI function PIN");

enum uart_mode {
	uart_mode_rs422,
	uart_mode_rs232,
	uart_mode_rs485,
	uart_mode_rs485_1,
	uart_mode_rs422_term,
	uart_mode_rs232_coexist,
	uart_mode_rs485_1_term,
	uart_mode_shutdown,
	uart_mode_invalid,
};

struct gpio_access {
	unsigned int set;
	unsigned char data;
};

#ifdef FLASH_RW_DRIVER
struct internal_data {
	unsigned int address;
	unsigned int size;
	unsigned char buf[F75115_MAX_DATA_BLOCK];
};
#endif

struct f75115_pin_config_data {
	enum uart_mode force_uart_mode;
	u8 gpio_mode;
	const int address[9];
	const int offset[9];
};

/* Save for a control register and bit offset */
struct reg_value {
	const u16 reg_address;
	const u16 reg_offset;
	const u16 reg_bit;
};

/* 3 control register to configuration a output pin mode and value */
struct pin_data {
	struct reg_value port_mode_1;
	struct reg_value port_mode_0;
	struct reg_value port_io;
};

/* 3 output pins to control transceiver mode */
struct out_pin {
	struct pin_data m1;
	struct pin_data m2;
	struct pin_data m0_sd;
};

struct io_map_value {
	int product_id;
	int max_port;
	enum uart_mode mode;
	struct out_pin port[MAX_NUM_PORTS + 1];
};

static const struct io_map_value f75115_rs232_control = {
	FINTEK_DEVICE_ID,
	F75115_NUM_PORT,
	uart_mode_rs232,
	{
		/* please reference f81439 io port */
		{
			{
				{ 0x2ad5, 4, 0 },
				{ 0x2ad4, 4, 1 },
				{ 0x2a90, 4, 0 },
			},
			{
				{ 0x2ad5, 5, 0 },
				{ 0x2ad4, 5, 1 },
				{ 0x2a90, 5, 0 },
			},
			{
				{ 0x2add, 7, 0 },
				{ 0x2adc, 7, 1 },
				{ 0x2ae8, 7, 1 },
			},
		},
		{
			{
				{ 0x2add, 3, 0 },
				{ 0x2adc, 3, 1 },
				{ 0x2ae8, 3, 0 },
			},
			{
				{ 0x2add, 0, 0 },
				{ 0x2adc, 0, 1 },
				{ 0x2ae8, 0, 0 },
			},
			{
				{ 0x2add, 6, 0 },
				{ 0x2adc, 6, 1 },
				{ 0x2ae8, 6, 1 },
			},
		},
		{
			{
				{ 0x2ad3, 6, 0 },
				{ 0x2ad2, 6, 1 },
				{ 0x2a80, 6, 0 },
			},
			{
				{ 0x2add, 2, 0 },
				{ 0x2adc, 2, 1 },
				{ 0x2ae8, 2, 0 },
			},
			{
				{ 0x2ad5, 0, 0 },
				{ 0x2ad4, 0, 1 },
				{ 0x2a90, 0, 1 },
			},
		},
		{
			{
				{ 0x2ad5, 1, 0 },
				{ 0x2ad4, 1, 1 },
				{ 0x2a90, 1, 0 },
			},
			{
				{ 0x2ad5, 2, 0 },
				{ 0x2ad4, 2, 1 },
				{ 0x2a90, 2, 0 },
			},
			{
				{ 0x2ad5, 3, 0 },
				{ 0x2ad4, 3, 1 },
				{ 0x2a90, 3, 1 },
			},
		},
	},
};

static const struct io_map_value f75115_rs485_control = {
	FINTEK_DEVICE_ID,
	F75115_NUM_PORT,
	uart_mode_rs485,
	{
		/* please reference f81439 io port */
		{
			{
				{ 0x2ad5, 4, 0 },
				{ 0x2ad4, 4, 1 },
				{ 0x2a90, 4, 0 },
			},
			{
				{ 0x2ad5, 5, 0 },
				{ 0x2ad4, 5, 1 },
				{ 0x2a90, 5, 1 },
			},
			{
				{ 0x2add, 7, 0 },
				{ 0x2adc, 7, 1 },
				{ 0x2ae8, 7, 0 },
			},
		},
		{
			{
				{ 0x2add, 3, 0 },
				{ 0x2adc, 3, 1 },
				{ 0x2ae8, 3, 0 },
			},
			{
				{ 0x2add, 0, 0 },
				{ 0x2adc, 0, 1 },
				{ 0x2ae8, 0, 1 },
			},
			{
				{ 0x2add, 6, 0 },
				{ 0x2adc, 6, 1 },
				{ 0x2ae8, 6, 0 },
			},
		},
		{
			{
				{ 0x2ad3, 6, 0 },
				{ 0x2ad2, 6, 1 },
				{ 0x2a80, 6, 0 },
			},
			{
				{ 0x2add, 2, 0 },
				{ 0x2adc, 2, 1 },
				{ 0x2ae8, 2, 1 },
			},
			{
				{ 0x2ad5, 0, 0 },
				{ 0x2ad4, 0, 1 },
				{ 0x2a90, 0, 0 },
			},
		},
		{
			{
				{ 0x2ad5, 1, 0 },
				{ 0x2ad4, 1, 1 },
				{ 0x2a90, 1, 0 },
			},
			{
				{ 0x2ad5, 2, 0 },
				{ 0x2ad4, 2, 1 },
				{ 0x2a90, 2, 1 },
			},
			{
				{ 0x2ad5, 3, 0 },
				{ 0x2ad4, 3, 1 },
				{ 0x2a90, 3, 0 },
			},
		},
	},
};

static const struct io_map_value f75115_rs485_1_control = {
	FINTEK_DEVICE_ID,
	F75115_NUM_PORT,
	uart_mode_rs485_1,
	{
		/* please reference f81439 io port */
		{
			{
				{ 0x2ad5, 4, 0 },
				{ 0x2ad4, 4, 1 },
				{ 0x2a90, 4, 0 },
			},
			{
				{ 0x2ad5, 5, 0 },
				{ 0x2ad4, 5, 1 },
				{ 0x2a90, 5, 1 },
			},
			{
				{ 0x2add, 7, 0 },
				{ 0x2adc, 7, 1 },
				{ 0x2ae8, 7, 1 },
			},
		},
		{
			{
				{ 0x2add, 3, 0 },
				{ 0x2adc, 3, 1 },
				{ 0x2ae8, 3, 0 },
			},
			{
				{ 0x2add, 0, 0 },
				{ 0x2adc, 0, 1 },
				{ 0x2ae8, 0, 1 },
			},
			{
				{ 0x2add, 6, 0 },
				{ 0x2adc, 6, 1 },
				{ 0x2ae8, 6, 1 },
			},
		},
		{
			{
				{ 0x2ad3, 6, 0 },
				{ 0x2ad2, 6, 1 },
				{ 0x2a80, 6, 0 },
			},
			{
				{ 0x2add, 2, 0 },
				{ 0x2adc, 2, 1 },
				{ 0x2ae8, 2, 1 },
			},
			{
				{ 0x2ad5, 0, 0 },
				{ 0x2ad4, 0, 1 },
				{ 0x2a90, 0, 1 },
			},
		},
		{
			{
				{ 0x2ad5, 1, 0 },
				{ 0x2ad4, 1, 1 },
				{ 0x2a90, 1, 0 },
			},
			{
				{ 0x2ad5, 2, 0 },
				{ 0x2ad4, 2, 1 },
				{ 0x2a90, 2, 1 },
			},
			{
				{ 0x2ad5, 3, 0 },
				{ 0x2ad4, 3, 1 },
				{ 0x2a90, 3, 1 },
			},
		},
	},
};

static const struct io_map_value f75115_rs422_control = {
	FINTEK_DEVICE_ID,
	F75115_NUM_PORT,
	uart_mode_rs422,
	{
		/* please reference f81439 io port */
		{
			{
				{ 0x2ad5, 4, 0 },
				{ 0x2ad4, 4, 1 },
				{ 0x2a90, 4, 0 },
			},
			{
				{ 0x2ad5, 5, 0 },
				{ 0x2ad4, 5, 1 },
				{ 0x2a90, 5, 0 },
			},
			{
				{ 0x2add, 7, 0 },
				{ 0x2adc, 7, 1 },
				{ 0x2ae8, 7, 0 },
			},
		},
		{
			{
				{ 0x2add, 3, 0 },
				{ 0x2adc, 3, 1 },
				{ 0x2ae8, 3, 0 },
			},
			{
				{ 0x2add, 0, 0 },
				{ 0x2adc, 0, 1 },
				{ 0x2ae8, 0, 0 },
			},
			{
				{ 0x2add, 6, 0 },
				{ 0x2adc, 6, 1 },
				{ 0x2ae8, 6, 0 },
			},
		},
		{
			{
				{ 0x2ad3, 6, 0 },
				{ 0x2ad2, 6, 1 },
				{ 0x2a80, 6, 0 },
			},
			{
				{ 0x2add, 2, 0 },
				{ 0x2adc, 2, 1 },
				{ 0x2ae8, 2, 0 },
			},
			{
				{ 0x2ad5, 0, 0 },
				{ 0x2ad4, 0, 1 },
				{ 0x2a90, 0, 0 },
			},
		},
		{
			{
				{ 0x2ad5, 1, 0 },
				{ 0x2ad4, 1, 1 },
				{ 0x2a90, 1, 0 },
			},
			{
				{ 0x2ad5, 2, 0 },
				{ 0x2ad4, 2, 1 },
				{ 0x2a90, 2, 0 },
			},
			{
				{ 0x2ad5, 3, 0 },
				{ 0x2ad4, 3, 1 },
				{ 0x2a90, 3, 0 },
			},
		},
	},
};

static const struct io_map_value f75115_shutdown_control = {
	FINTEK_DEVICE_ID,
	F75115_NUM_PORT,
	uart_mode_shutdown,
	{
		/* please reference f81439 io port */
		{
			{
				{ 0x2ad5, 4, 0 },
				{ 0x2ad4, 4, 1 },
				{ 0x2a90, 4, 1 },
			},
			{
				{ 0x2ad5, 5, 0 },
				{ 0x2ad4, 5, 1 },
				{ 0x2a90, 5, 1 },
			},
			{
				{ 0x2add, 7, 0 },
				{ 0x2adc, 7, 1 },
				{ 0x2ae8, 7, 1 },
			},
		},
		{
			{
				{ 0x2add, 3, 0 },
				{ 0x2adc, 3, 1 },
				{ 0x2ae8, 3, 1 },
			},
			{
				{ 0x2add, 0, 0 },
				{ 0x2adc, 0, 1 },
				{ 0x2ae8, 0, 1 },
			},
			{
				{ 0x2add, 6, 0 },
				{ 0x2adc, 6, 1 },
				{ 0x2ae8, 6, 1 },
			},
		},
		{
			{
				{ 0x2ad3, 6, 0 },
				{ 0x2ad2, 6, 1 },
				{ 0x2a80, 6, 1 },
			},
			{
				{ 0x2add, 2, 0 },
				{ 0x2adc, 2, 1 },
				{ 0x2ae8, 2, 1 },
			},
			{
				{ 0x2ad5, 0, 0 },
				{ 0x2ad4, 0, 1 },
				{ 0x2a90, 0, 1 },
			},
		},
		{
			{
				{ 0x2ad5, 1, 0 },
				{ 0x2ad4, 1, 1 },
				{ 0x2a90, 1, 1 },
			},
			{
				{ 0x2ad5, 2, 0 },
				{ 0x2ad4, 2, 1 },
				{ 0x2a90, 2, 1 },
			},
			{
				{ 0x2ad5, 3, 0 },
				{ 0x2ad4, 3, 1 },
				{ 0x2a90, 3, 1 },
			},
		},
	},
};

static const struct io_map_value f75115_rs422_term_control = {
	FINTEK_DEVICE_ID,
	F75115_NUM_PORT,
	uart_mode_shutdown,
	{
		/* please reference f81439 io port */
		{
			{
				{ 0x2ad5, 4, 0 },
				{ 0x2ad4, 4, 1 },
				{ 0x2a90, 4, 1 },
			},
			{
				{ 0x2ad5, 5, 0 },
				{ 0x2ad4, 5, 1 },
				{ 0x2a90, 5, 0 },
			},
			{
				{ 0x2add, 7, 0 },
				{ 0x2adc, 7, 1 },
				{ 0x2ae8, 7, 0 },
			},
		},
		{
			{
				{ 0x2add, 3, 0 },
				{ 0x2adc, 3, 1 },
				{ 0x2ae8, 3, 1 },
			},
			{
				{ 0x2add, 0, 0 },
				{ 0x2adc, 0, 1 },
				{ 0x2ae8, 0, 0 },
			},
			{
				{ 0x2add, 6, 0 },
				{ 0x2adc, 6, 1 },
				{ 0x2ae8, 6, 0 },
			},
		},
		{
			{
				{ 0x2ad3, 6, 0 },
				{ 0x2ad2, 6, 1 },
				{ 0x2a80, 6, 1 },
			},
			{
				{ 0x2add, 2, 0 },
				{ 0x2adc, 2, 1 },
				{ 0x2ae8, 2, 0 },
			},
			{
				{ 0x2ad5, 0, 0 },
				{ 0x2ad4, 0, 1 },
				{ 0x2a90, 0, 0 },
			},
		},
		{
			{
				{ 0x2ad5, 1, 0 },
				{ 0x2ad4, 1, 1 },
				{ 0x2a90, 1, 1 },
			},
			{
				{ 0x2ad5, 2, 0 },
				{ 0x2ad4, 2, 1 },
				{ 0x2a90, 2, 0 },
			},
			{
				{ 0x2ad5, 3, 0 },
				{ 0x2ad4, 3, 1 },
				{ 0x2a90, 3, 0 },
			},
		},
	},
};

static const struct io_map_value f75115_rs232_coexist_control = {
	FINTEK_DEVICE_ID,
	F75115_NUM_PORT,
	uart_mode_shutdown,
	{
		/* please reference f81439 io port */
		{
			{
				{ 0x2ad5, 4, 0 },
				{ 0x2ad4, 4, 1 },
				{ 0x2a90, 4, 1 },
			},
			{
				{ 0x2ad5, 5, 0 },
				{ 0x2ad4, 5, 1 },
				{ 0x2a90, 5, 0 },
			},
			{
				{ 0x2add, 7, 0 },
				{ 0x2adc, 7, 1 },
				{ 0x2ae8, 7, 1 },
			},
		},
		{
			{
				{ 0x2add, 3, 0 },
				{ 0x2adc, 3, 1 },
				{ 0x2ae8, 3, 1 },
			},
			{
				{ 0x2add, 0, 0 },
				{ 0x2adc, 0, 1 },
				{ 0x2ae8, 0, 0 },
			},
			{
				{ 0x2add, 6, 0 },
				{ 0x2adc, 6, 1 },
				{ 0x2ae8, 6, 1 },
			},
		},
		{
			{
				{ 0x2ad3, 6, 0 },
				{ 0x2ad2, 6, 1 },
				{ 0x2a80, 6, 1 },
			},
			{
				{ 0x2add, 2, 0 },
				{ 0x2adc, 2, 1 },
				{ 0x2ae8, 2, 0 },
			},
			{
				{ 0x2ad5, 0, 0 },
				{ 0x2ad4, 0, 1 },
				{ 0x2a90, 0, 1 },
			},
		},
		{
			{
				{ 0x2ad5, 1, 0 },
				{ 0x2ad4, 1, 1 },
				{ 0x2a90, 1, 1 },
			},
			{
				{ 0x2ad5, 2, 0 },
				{ 0x2ad4, 2, 1 },
				{ 0x2a90, 2, 0 },
			},
			{
				{ 0x2ad5, 3, 0 },
				{ 0x2ad4, 3, 1 },
				{ 0x2a90, 3, 1 },
			},
		},
	},
};

static const struct io_map_value f75115_rs485_1_term_control = {
	FINTEK_DEVICE_ID,
	F75115_NUM_PORT,
	uart_mode_shutdown,
	{
		/* please reference f81439 io port */
		{
			{
				{ 0x2ad5, 4, 0 },
				{ 0x2ad4, 4, 1 },
				{ 0x2a90, 4, 1 },
			},
			{
				{ 0x2ad5, 5, 0 },
				{ 0x2ad4, 5, 1 },
				{ 0x2a90, 5, 1 },
			},
			{
				{ 0x2add, 7, 0 },
				{ 0x2adc, 7, 1 },
				{ 0x2ae8, 7, 0 },
			},
		},
		{
			{
				{ 0x2add, 3, 0 },
				{ 0x2adc, 3, 1 },
				{ 0x2ae8, 3, 1 },
			},
			{
				{ 0x2add, 0, 0 },
				{ 0x2adc, 0, 1 },
				{ 0x2ae8, 0, 1 },
			},
			{
				{ 0x2add, 6, 0 },
				{ 0x2adc, 6, 1 },
				{ 0x2ae8, 6, 0 },
			},
		},
		{
			{
				{ 0x2ad3, 6, 0 },
				{ 0x2ad2, 6, 1 },
				{ 0x2a80, 6, 1 },
			},
			{
				{ 0x2add, 2, 0 },
				{ 0x2adc, 2, 1 },
				{ 0x2ae8, 2, 1 },
			},
			{
				{ 0x2ad5, 0, 0 },
				{ 0x2ad4, 0, 1 },
				{ 0x2a90, 0, 0 },
			},
		},
		{
			{
				{ 0x2ad5, 1, 0 },
				{ 0x2ad4, 1, 1 },
				{ 0x2a90, 1, 1 },
			},
			{
				{ 0x2ad5, 2, 0 },
				{ 0x2ad4, 2, 1 },
				{ 0x2a90, 2, 1 },
			},
			{
				{ 0x2ad5, 3, 0 },
				{ 0x2ad4, 3, 1 },
				{ 0x2a90, 3, 0 },
			},
		},
	},
};

static const struct io_map_value *f75115_mode_control[uart_mode_invalid] = {
	&f75115_rs422_control,	      &f75115_rs232_control,
	&f75115_rs485_control,	      &f75115_rs485_1_control,
	&f75115_rs422_term_control,   &f75115_rs232_coexist_control,
	&f75115_rs485_1_term_control, &f75115_shutdown_control,
};

static unsigned int pin0_mode_table[] = { 0x2ad2, 0x2ad4, 0x2ad6, 0x2ada,
					  0x2adc };
static unsigned int pin1_mode_table[] = { 0x2ad3, 0x2ad5, 0x2ad7, 0x2adb,
					  0x2add };
static unsigned int pin_data_table[] = { 0x2a80, 0x2a90, 0x2aa0, 0x2ab0,
					 0x2ae8 };
static DEFINE_MUTEX(id_mutex);

#ifndef I2C_HWMODE
static unsigned int dev_id = 100;
#endif

static const struct usb_device_id id_table[] = {
	{ USB_DEVICE(FINTEK_VENDOR_ID_1, FINTEK_DEVICE_ID) },
	{ USB_DEVICE(FINTEK_VENDOR_ID_2, FINTEK_DEVICE_ID) },
	{} /* Terminating entry */
};

MODULE_DEVICE_TABLE(usb, id_table);

struct f75115_serial_private {
	bool is_phy_port_not_empty[F75115_NUM_PORT];
	spinlock_t tx_empty_lock;
	struct mutex change_mode_mutex;
	u8 default_conf_data[F75115_DEF_CONF_SIZE];
	u32 setting_idx;
	atomic_t port_active[F75115_NUM_PORT];

	struct platform_device i2c_device[2];

#ifndef I2C_HWMODE
	struct i2c_gpio_platform_data i2c_data[2];
#endif

	unsigned long long gpio_opendrain_mode;
	unsigned long long gpio_opendrain_changed_bit;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 15, 0)
	struct gpiod_lookup_table *i2c_table[2];
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 17, 0)
	struct gpiod_lookup_table *spi_table;
#endif

	struct f75115_pin_config_data port_pin_data;
#ifdef CONFIG_GPIOLIB
	struct gpio_chip f75115_gpio_chip;
#endif

#ifdef SPI_HWMODE
	struct spi_master *spi_master;
	struct spi_device *spi_device[F75115_SPI_MAX_CS_NUM];
	struct spi_board_info info[F75115_SPI_MAX_CS_NUM];

#else
	struct platform_device spi_device;
#endif

#ifdef GPIO_OUTPUT_FAST_MODE
	u8 gpio_data[sizeof(pin_data_table) + 1]; // add GPIO5x
#endif
	long fw_ver;
	struct pwm_chip f75115_pwm_chip;
	struct usb_serial *serial;
	unsigned int pwm_cur_clock;
	unsigned int pwm_cur_div;
	u64 pwm_cur_duty_us[F75115_MAX_PWN_CNT];
	u64 pwm_cur_en[F75115_MAX_PWN_CNT];
	struct mutex pwm_mutex;
};

struct f75115_port_private {
	u8 phy;
	u8 shadow_mcr;
	u8 shadow_lcr;
	u32 current_baud_rate;
	u32 current_baud_base;
#ifdef MULTIDROP_ENABLE
	u16 mode_9bit;
	u16 bitmask_9bit;
	u16 addr_9bit;
#endif

#ifdef SAVE_CONTROL
	int port_flag;
#endif
	spinlock_t msr_lock;
	struct mutex msr_mutex;
	u8 shadow_msr;
#ifdef LSR_ISSUE
	struct work_struct lsr_work;
#endif

#ifdef WRITER_WQ
	struct work_struct writer_work;
#endif
	struct usb_serial_port *port;
};

const static int m_pwm_freq[] = { 24000000, 12000000, 6000000,
				  4000000,  3000000,  2000000 };

static int f75115_set_mask_normal_register(struct usb_device *dev, u16 reg,
					   u8 mask, u8 data);
static int f75115_set_normal_register(struct usb_device *dev, u16 reg,
				      u8 data);
static int f75115_get_normal_register(struct usb_device *dev, u16 reg,
				      u8 *data);
static int f75115_pwm_en_param_set(struct usb_serial *serial, int idx, int en,
				   u64 duty_us);
static int f75115_pwm_div_set(struct usb_serial *serial, int pwm_div);
static int f75115_pwm_clock_set(struct usb_serial *serial, int pwm_freq);

#ifdef GPIO_OUTPUT_FAST_MODE
static bool f75115_gpio_special_func(int set, int offset)
{
	int index = (set << 4) | offset;

	switch (index) {
	// SPI
	case 14:
	case 15:
	case 16:
	case 17:
	case 20:

	// I2C0
	case 40:
	case 41:

	// I2C1
	case 21:
	case 22:
		return true;
	}

	return false;
}
#endif

static void f75115_i2c_release(struct device *dev)
{
}

#ifdef I2C_HWMODE
struct f75115_i2c_priv {
	struct i2c_adapter adapter;
	struct usb_device *dev;
	int chan;
};

static u32 f75115_i2c_func(struct i2c_adapter *adap)
{
#if 1
	return I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMUL | I2C_FUNC_PROTOCOL_MANGLING;
#else
	return I2C_FUNC_I2C | I2C_FUNC_NOSTART | I2C_FUNC_SMBUS_EMUL_ALL |
	       I2C_FUNC_10BIT_ADDR | I2C_FUNC_PROTOCOL_MANGLING;
#endif
}

static int f75111_i2c_start(struct i2c_adapter *adap)
{
	struct f75115_i2c_priv *i2c_priv = i2c_get_adapdata(adap);
	struct usb_device *dev = i2c_priv->dev;
	int reg;

	if (i2c_priv->chan == 1)
		reg = 0xd020;
	else
		reg = 0xd028;

	return f75115_set_normal_register(dev, reg, 0);
}

static int f75111_i2c_stop(struct i2c_adapter *adap)
{
	struct f75115_i2c_priv *i2c_priv = i2c_get_adapdata(adap);
	struct usb_device *dev = i2c_priv->dev;
	int reg;

	if (i2c_priv->chan == 1)
		reg = 0xd022;
	else
		reg = 0xd02a;

	return f75115_set_normal_register(dev, reg, 1);
}

static int f75115_i2c_get_ack(struct usb_device *dev, u16 reg, u8 *data)
{
	int count = F75115_USB_MAX_RETRY;
	int status;
	u8 *tmp;

	tmp = kmalloc(sizeof(u8), GFP_KERNEL);
	if (!tmp)
		return -ENOMEM;

	/*
	 * Our device maybe not reply when heavily loading,
	 * We'll retry for F75115_USB_MAX_RETRY times
	 */
	while (count--) {
		status = usb_control_msg(dev, usb_rcvctrlpipe(dev, 0),
					 F75115_SET_GET_REGISTER,
					 USB_TYPE_VENDOR | USB_DIR_IN, reg, 0,
					 tmp, sizeof(u8), F75115_USB_TIMEOUT);
		if (status > 0)
			break;

		if (status == 0)
			status = -EIO;
	}

	if (status < 0) {
		dev_err(&dev->dev, "%s ERROR reg:%x status:%i failed\n",
			__func__, reg, status);
		kfree(tmp);
		return status;
	}

	*data = *tmp;
	kfree(tmp);
	return 0;
}

static int f75115_i2c_get_data_with_ack(struct usb_device *dev, u16 reg,
					u8 *data, int is_cont)
{
	int count = F75115_USB_MAX_RETRY;
	int status;
	u8 *tmp;

	tmp = kmalloc(sizeof(u8), GFP_KERNEL);
	if (!tmp)
		return -ENOMEM;

	/*
	 * Our device maybe not reply when heavily loading,
	 * We'll retry for F75115_USB_MAX_RETRY times
	 */
	while (count--) {
		status = usb_control_msg(dev, usb_rcvctrlpipe(dev, 0),
					 F75115_SET_GET_REGISTER,
					 USB_TYPE_VENDOR | USB_DIR_IN, reg,
					 !is_cont, tmp, sizeof(u8),
					 F75115_USB_TIMEOUT);
		if (status > 0)
			break;

		if (status == 0)
			status = -EIO;
	}

	if (status < 0) {
		dev_err(&dev->dev, "%s ERROR reg:%x status:%i failed\n",
			__func__, reg, status);
		kfree(tmp);
		return status;
	}

	*data = *tmp;
	kfree(tmp);
	return 0;
}

static int f75115_get_acknak(struct i2c_adapter *adap, int *is_ack)
{
	struct f75115_i2c_priv *i2c_priv = i2c_get_adapdata(adap);
	struct usb_device *dev = i2c_priv->dev;
	int r, reg;
	u8 data;

	if (i2c_priv->chan == 1)
		reg = 0xd023;
	else
		reg = 0xd02b;

	r = f75115_i2c_get_ack(dev, reg, &data);
	if (r)
		return r;

	*is_ack = !data;

	return 0;
}

static int f75111_i2c_outb(struct i2c_adapter *adap, unsigned char c)
{
	struct f75115_i2c_priv *i2c_priv = i2c_get_adapdata(adap);
	struct usb_device *dev = i2c_priv->dev;
	int r, reg, ack;

	if (i2c_priv->chan == 1)
		reg = 0xd021;
	else
		reg = 0xd029;

	r = f75115_set_normal_register(dev, reg, c);
	if (r)
		return r;

	r = f75115_get_acknak(adap, &ack);
	if (r)
		return r;

	return ack;
}

static int f75111_i2c_inb(struct i2c_adapter *adap, u8 *data, int is_cont)
{
	struct f75115_i2c_priv *i2c_priv = i2c_get_adapdata(adap);
	struct usb_device *dev = i2c_priv->dev;
	int reg;

	if (i2c_priv->chan == 1)
		reg = 0xd021;
	else
		reg = 0xd029;

	return f75115_i2c_get_data_with_ack(dev, reg, data, is_cont);
}

static int f75111_sendbytes(struct i2c_adapter *adap, struct i2c_msg *msg)
{
	const unsigned char *temp = msg->buf;
	int count = msg->len;
	unsigned short nak_ok = msg->flags & I2C_M_IGNORE_NAK;
	int retval;
	int wrcount = 0;

	dev_info(&adap->dev, "%s: in\n", __func__);

	while (count > 0) {
		retval = f75111_i2c_outb(adap, *temp);
		dev_info(&adap->dev,
			 "%s: wrcount: %d, *temp: %xh, retval: %d\n", __func__,
			 wrcount, *temp, retval);

		/* OK/ACK; or ignored NAK */
		if ((retval > 0) || (nak_ok && (retval == 0))) {
			count--;
			temp++;
			wrcount++;

			/* A slave NAKing the master means the slave didn't like
		 * something about the data it saw.  For example, maybe
		 * the SMBus PEC was wrong.
		 */
		} else if (retval == 0) {
			dev_err(&adap->dev, "sendbytes: NAK bailout.\n");
			return -EIO;

			/* Timeout; or (someday) lost arbitration
		 *
		 * FIXME Lost ARB implies retrying the transaction from
		 * the first message, after the "winning" master issues
		 * its STOP.  As a rule, upper layer code has no reason
		 * to know or care about this ... it is *NOT* an error.
		 */
		} else {
			dev_err(&adap->dev, "sendbytes: error %d\n", retval);
			return retval;
		}
	}
	return wrcount;
}

static int f75115_bit_doAddress(struct i2c_adapter *adap, struct i2c_msg *msg)
{
	unsigned short flags = msg->flags;
	unsigned short nak_ok = msg->flags & I2C_M_IGNORE_NAK;
	unsigned char addr;
	int ret, retries;

	retries = nak_ok ? 0 : adap->retries;

	if (flags & I2C_M_TEN) {
#if 0		
		/* a ten bit address */
		addr = 0xf0 | ((msg->addr >> 7) & 0x06);
		bit_dbg(2, &adap->dev, "addr0: %d\n", addr);
		/* try extended address code...*/
		ret = try_address(adap, addr, retries);
		if ((ret != 1) && !nak_ok)  {
			dev_err(&adap->dev,
				"died at extended address code\n");
			return -ENXIO;
		}
		/* the remaining 8 bit address */
		ret = i2c_outb(adap, msg->addr & 0xff);
		if ((ret != 1) && !nak_ok) {
			/* the chip did not ack / xmission error occurred */
			dev_err(&adap->dev, "died at 2nd address code\n");
			return -ENXIO;
		}
		if (flags & I2C_M_RD) {
			bit_dbg(3, &adap->dev,
				"emitting repeated start condition\n");
			i2c_repstart(adap);
			/* okay, now switch into reading mode */
			addr |= 0x01;
			ret = try_address(adap, addr, retries);
			if ((ret != 1) && !nak_ok) {
				dev_err(&adap->dev,
					"died at repeated address code\n");
				return -EIO;
			}
		}
#endif
		return -ENOSPC;
	} else { /* normal 7bit address	*/
		addr = i2c_8bit_addr_from_msg(msg);
		if (flags & I2C_M_REV_DIR_ADDR)
			addr ^= 1;

		ret = f75111_i2c_outb(adap, addr);
		if ((ret != 1) && !nak_ok)
			return -ENXIO;
	}

	return 0;
}

static int f75115_readbytes(struct i2c_adapter *adap, struct i2c_msg *msg)
{
	int i, r;
	unsigned char *temp = msg->buf;
	int count = msg->len;
	//const unsigned flags = msg->flags;

#if 1
	for (i = 0; i < count; ++i) {
		if (i == count - 1)
			r = f75111_i2c_inb(adap, temp, false);
		else
			r = f75111_i2c_inb(adap, temp, true);
		if (r < 0)
			return r;

		temp++;
	}

	return count;
#else
	//f75111_i2c_inb

	while (count > 0) {
		inval = i2c_inb(adap);
		if (inval >= 0) {
			*temp = inval;
			rdcount++;
		} else { /* read timed out */
			break;
		}

		temp++;
		count--;

		/* Some SMBus transactions require that we receive the
		   transaction length as the first read byte. */
		if (rdcount == 1 && (flags & I2C_M_RECV_LEN)) {
			if (inval <= 0 || inval > I2C_SMBUS_BLOCK_MAX) {
				if (!(flags & I2C_M_NO_RD_ACK))
					acknak(adap, 0);
				dev_err(&adap->dev,
					"readbytes: invalid block length (%d)\n",
					inval);
				return -EPROTO;
			}
			/* The original count value accounts for the extra
			   bytes, that is, either 1 for a regular transaction,
			   or 2 for a PEC transaction. */
			count += inval;
			msg->len += inval;
		}

		dev_info(&adap->dev, "readbytes: 0x%02x %s\n", inval,
			 (flags & I2C_M_NO_RD_ACK) ? "(no ack/nak)" :
						     (count ? "A" : "NA"));

		if (!(flags & I2C_M_NO_RD_ACK)) {
			inval = acknak(adap, count);
			if (inval < 0)
				return inval;
		}
	}
	return rdcount;
#endif
	return 0;
}

static int f75115_i2c_xfer(struct i2c_adapter *adap, struct i2c_msg msgs[],
			   int num)
{
	struct i2c_msg *pmsg;
	int i, ret;
	unsigned short nak_ok;

	dev_dbg(&adap->dev, "emitting start condition\n");

	ret = f75111_i2c_start(adap);
	if (ret)
		goto bailout;

	for (i = 0; i < num; i++) {
		pmsg = &msgs[i];
		nak_ok = pmsg->flags & I2C_M_IGNORE_NAK;
		if (!(pmsg->flags & I2C_M_NOSTART)) {
			if (i) {
				if (msgs[i - 1].flags & I2C_M_STOP) {
					dev_dbg(&adap->dev,
						"emitting enforced stop/start condition\n");
					ret = f75111_i2c_stop(adap);
					if (ret)
						goto bailout;

					ret = f75111_i2c_start(adap);
					if (ret)
						goto bailout;
				} else {
					dev_dbg(&adap->dev,
						"emitting repeated start condition\n");

					//i2c_repstart(adap);
					ret = f75111_i2c_start(adap);
					if (ret)
						goto bailout;
				}
			}

			ret = f75115_bit_doAddress(adap, pmsg);
			if ((ret != 0) && !nak_ok) {
				dev_dbg(&adap->dev,
					"NAK from device addr 0x%02x msg #%d\n",
					msgs[i].addr, i);
				goto bailout;
			}
		}
		if (pmsg->flags & I2C_M_RD) {
			/* read bytes into buffer*/
			ret = f75115_readbytes(adap, pmsg);
			if (ret >= 1)
				dev_dbg(&adap->dev, "read %d byte%s\n", ret,
					ret == 1 ? "" : "s");
			if (ret < pmsg->len) {
				if (ret >= 0)
					ret = -EIO;
				goto bailout;
			}
		} else {
			/* write bytes from buffer */
			ret = f75111_sendbytes(adap, pmsg);
			if (ret >= 1)
				dev_dbg(&adap->dev, "wrote %d byte%s\n", ret,
					ret == 1 ? "" : "s");
			if (ret < pmsg->len) {
				if (ret >= 0)
					ret = -EIO;
				goto bailout;
			}
		}
	}
	ret = i;

bailout:
	dev_dbg(&adap->dev, "emitting stop condition\n");
	f75111_i2c_stop(adap);

	return ret;
}

static const struct i2c_algorithm f75115_i2c_algo = {
	.master_xfer = f75115_i2c_xfer,
	.functionality = f75115_i2c_func,
};

static int f75115_i2c_probe(struct platform_device *pdev)
{
	struct f75115_i2c_priv *i2c_priv = dev_get_platdata(&pdev->dev);
	int status;
	int max_name = 32;
	char *name = NULL;

	pr_debug("%s: %d\n", __func__, i2c_priv->chan);

	i2c_set_adapdata(&i2c_priv->adapter, i2c_priv);
	i2c_priv->adapter.owner = THIS_MODULE;
	i2c_priv->adapter.algo = &f75115_i2c_algo;
	i2c_priv->adapter.dev.parent = &pdev->dev;
	strlcpy(i2c_priv->adapter.name, pdev->name,
		sizeof(i2c_priv->adapter.name));

        name = devm_kzalloc(&pdev->dev, max_name, GFP_KERNEL);
        if (!name)
                return -ENOMEM;

        snprintf(name, max_name - 1, "/i2c@f75115%d", i2c_priv->chan);

	dev_info(&pdev->dev, "%s: name %s ok\n", __func__, name);

	i2c_priv->adapter.dev.of_node   = of_find_node_by_path(name);

	status = i2c_add_adapter(&i2c_priv->adapter);
	if (status)
		return status;

	dev_dbg(&pdev->dev, "%s: i2c bus %d ok\n", __func__, i2c_priv->chan);

	return 0;
}

static int f75115_i2c_remove(struct platform_device *pdev)
{
	struct f75115_i2c_priv *i2c_priv = dev_get_platdata(&pdev->dev);

	pr_debug("%s\n", __func__);
	i2c_del_adapter(&i2c_priv->adapter);

	return 0;
}

static struct platform_driver f75115_i2c_driver = {
	.probe = f75115_i2c_probe,
	.remove = f75115_i2c_remove,
	.driver = {
		.name = "f75115-i2c",
	},
};

static int f75115_i2c_init(struct usb_serial *serial, int gpio_start)
{
	struct f75115_serial_private *priv;
	struct f75115_i2c_priv *i2c_priv;
	int i, status;

	priv = usb_get_serial_data(serial);
	i2c_priv = devm_kzalloc(&serial->interface->dev, sizeof(*i2c_priv),
				GFP_KERNEL);

	for (i = 0; i < F75115_MAX_I2C_SET; ++i) {
		if ((en_i2c & BIT(i)) == 0)
			continue;
		
		priv->i2c_device[i].name = "f75115-i2c";
		priv->i2c_device[i].id =
			PLATFORM_DEVID_AUTO; //PLATFORM_DEVID_AUTO;
		priv->i2c_device[i].dev.release = f75115_i2c_release;
		priv->i2c_device[i].dev.parent = &serial->interface->dev;

		memset(i2c_priv, 0, sizeof(*i2c_priv));
		i2c_priv->chan = i;
		i2c_priv->dev = serial->dev;

		pr_debug("%s: i2c_priv.dev: %p", __func__, i2c_priv->dev);

		status = platform_device_add_data(&priv->i2c_device[i],
						  i2c_priv, sizeof(*i2c_priv));
		if (status) {
			dev_err(&serial->interface->dev,
				"%s: add data failed\n", __func__);
			return status;
		}

		status = platform_device_register(&priv->i2c_device[i]);
		if (status) {
			dev_err(&serial->interface->dev, "%s: add failed\n",
				__func__);
			return status;
		}
	}

	return 0;
}

static void f75115_i2c_deinit(struct usb_serial *serial, int gpio_start)
{
	struct f75115_serial_private *priv;
	int i;

	priv = usb_get_serial_data(serial);

	for (i = 0; i < F75115_MAX_I2C_SET; ++i) {
		if ((en_i2c & BIT(i)) == 0)
			continue;

		platform_device_unregister(&priv->i2c_device[i]);
	}
}

#else
static int f75115_i2c_init(struct usb_serial *serial, int gpio_start)
{
	struct f75115_serial_private *priv;
	//struct i2c_adapter *adapter;
	u8 *dev_name;
	int status;
	int i;
	int cnt = F75115_MAX_I2C_SET;

	priv = usb_get_serial_data(serial);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 15, 0)

	priv->i2c_table[0] =
		devm_kzalloc(&serial->interface->dev,
			     sizeof(struct gpiod_lookup_table) +
				     2 * sizeof(struct gpiod_lookup),
			     GFP_KERNEL);
	if (!priv->i2c_table[0])
		return -ENOMEM;

	priv->i2c_table[1] =
		devm_kzalloc(&serial->interface->dev,
			     sizeof(struct gpiod_lookup_table) +
				     2 * sizeof(struct gpiod_lookup),
			     GFP_KERNEL);
	if (!priv->i2c_table[1])
		return -ENOMEM;

	priv->i2c_data[0].udelay = 1;
	priv->i2c_data[0].timeout = 1;

	priv->i2c_data[1].udelay = 1;
	priv->i2c_data[1].timeout = 1;
#else
	priv->i2c_data[0].sda_pin = gpio_start + F75115_SDA1_PIN;
	priv->i2c_data[0].scl_pin = gpio_start + F75115_SCL1_PIN;
	priv->i2c_data[0].udelay = 1;
	priv->i2c_data[0].timeout = 1;

	priv->i2c_data[1].sda_pin = gpio_start + F75115_SDA2_PIN;
	priv->i2c_data[1].scl_pin = gpio_start + F75115_SCL2_PIN;
	priv->i2c_data[1].udelay = 1;
	priv->i2c_data[1].timeout = 1;
#endif
	for (i = 0; i < cnt; ++i) {
		mutex_lock(&id_mutex);

#if 0
		for (j = 0; j < 65536; ++j) {
			adapter = i2c_get_adapter(j);

			if (adapter) 
				i2c_put_adapter(adapter);
			else
				break;
		}

		if (j >= 65536) {
			mutex_unlock(&id_mutex);
			return -ENODEV;
		}
#endif
		priv->i2c_device[i].name = "i2c-gpio";
		priv->i2c_device[i].id = dev_id++; //PLATFORM_DEVID_AUTO;
		priv->i2c_device[i].dev.platform_data = &priv->i2c_data[i];
		priv->i2c_device[i].dev.release = f75115_i2c_release;
		priv->i2c_device[i].dev.parent = &serial->interface->dev;

		status = platform_device_register(&priv->i2c_device[i]);
		if (status) {
			dev_err(&serial->interface->dev, "%s: failed\n",
				__func__);
			mutex_unlock(&id_mutex);
			return status;
		}

		mutex_unlock(&id_mutex);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 15, 0)
		dev_name =
			devm_kzalloc(&serial->interface->dev, 32, GFP_KERNEL);
		if (!dev_name)
			return -ENOMEM;

		sprintf(dev_name, "%s.%d", priv->i2c_device[i].name,
			priv->i2c_device[i].id);
		priv->i2c_table[i]->dev_id = dev_name;

		if (i == 0) {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 8, 0)
			priv->i2c_table[i]->table[0].key =
				priv->f75115_gpio_chip.label;
#else
			priv->i2c_table[i]->table[0].chip_label =
				priv->f75115_gpio_chip.label;
#endif
			priv->i2c_table[i]->table[0].chip_hwnum =
				F75115_SDA1_PIN;
			priv->i2c_table[i]->table[0].flags = GPIO_ACTIVE_HIGH |
							     GPIO_OPEN_DRAIN;
			priv->i2c_table[i]->table[0].con_id = "sda";

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 8, 0)
			priv->i2c_table[i]->table[1].key =
				priv->f75115_gpio_chip.label;
#else
			priv->i2c_table[i]->table[1].chip_label =
				priv->f75115_gpio_chip.label;
#endif
			priv->i2c_table[i]->table[1].chip_hwnum =
				F75115_SCL1_PIN;
			priv->i2c_table[i]->table[1].flags = GPIO_ACTIVE_HIGH |
							     GPIO_OPEN_DRAIN;
			priv->i2c_table[i]->table[1].con_id = "scl";

		} else {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 8, 0)
			priv->i2c_table[i]->table[0].key =
				priv->f75115_gpio_chip.label;
#else
			priv->i2c_table[i]->table[0].chip_label =
				priv->f75115_gpio_chip.label;
#endif
			priv->i2c_table[i]->table[0].chip_hwnum =
				F75115_SDA2_PIN;
			priv->i2c_table[i]->table[0].flags = GPIO_ACTIVE_HIGH |
							     GPIO_OPEN_DRAIN;
			priv->i2c_table[i]->table[0].con_id = "sda";

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 8, 0)
			priv->i2c_table[i]->table[1].key =
				priv->f75115_gpio_chip.label;
#else
			priv->i2c_table[i]->table[1].chip_label =
				priv->f75115_gpio_chip.label;
#endif
			priv->i2c_table[i]->table[1].chip_hwnum =
				F75115_SCL2_PIN;
			priv->i2c_table[i]->table[1].flags = GPIO_ACTIVE_HIGH |
							     GPIO_OPEN_DRAIN;
			priv->i2c_table[i]->table[1].con_id = "scl";
		}

		gpiod_add_lookup_table(priv->i2c_table[i]);
#endif
	}

	return 0;
}

static void f75115_i2c_deinit(struct usb_serial *serial, int gpio_start)
{
	struct f75115_serial_private *priv;
	int i;

	priv = usb_get_serial_data(serial);

	for (i = 0; i < F75115_MAX_I2C_SET; ++i)
		platform_device_unregister(&priv->i2c_device[i]);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 15, 0)
	gpiod_remove_lookup_table(priv->i2c_table[0]);
	gpiod_remove_lookup_table(priv->i2c_table[1]);
#endif
}
#endif

#ifdef SPI_HWMODE
static int f75115_spi_cs_gpio_deinit(struct usb_serial *serial,
				     int gpio_start);
static int f75115_spi_cs_gpio_init(struct usb_serial *serial, int gpio_start);
static int f75115_spi_hw_init(struct usb_serial *serial);

/* This may be called twice for each spi dev */
static int f75115_spi_setup(struct spi_device *spi)
{
#if 1
	struct f75115_serial_private *priv;
	struct usb_serial *serial;
	struct usb_device *dev;
	int status;
	unsigned char data = 0;

	serial = spi_controller_get_devdata(spi->controller);
	priv = usb_get_serial_data(serial);
	dev = serial->dev;

	switch (spi->mode & (SPI_CPOL | SPI_CPHA)) {
	case SPI_MODE_0:
		data = BIT(3);
		break;
	case SPI_MODE_1:
		break;
	case SPI_MODE_2:
		data = BIT(4);
		break;
	case SPI_MODE_3:
		data = BIT(4) | BIT(3);
		break;
	default:
		return -EINVAL;
	}

	if (spi->max_speed_hz > 3000000)
		data |= 0x00;
	else if (spi->max_speed_hz > 1500000)
		data |= 0x01;
	else if (spi->max_speed_hz > 750000)
		data |= 0x02;
	else if (spi->max_speed_hz > 375000)
		data |= 0x03;
	else if (spi->max_speed_hz > 187500)
		data |= 0x04;
	else if (spi->max_speed_hz > 93750)
		data |= 0x05;
	else if (spi->max_speed_hz > 46875)
		data |= 0x06;
	else
		data |= 0x07;

	data |= 0xc0; // enable spi
	status = f75115_set_normal_register(dev, 0x2af1, data);
	if (status)
		return status;
#else
	struct dw_spi_chip *chip_info = NULL;
	struct chip_data *chip;

	/* Only alloc on first setup */
	chip = spi_get_ctldata(spi);
	if (!chip) {
		chip = kzalloc(sizeof(struct chip_data), GFP_KERNEL);
		if (!chip)
			return -ENOMEM;
		spi_set_ctldata(spi, chip);
	}

	/*
	 * Protocol drivers may change the chip settings, so...
	 * if chip_info exists, use it
	 */
	chip_info = spi->controller_data;

	/* chip_info doesn't always exist */
	if (chip_info) {
		if (chip_info->cs_control)
			chip->cs_control = chip_info->cs_control;

		chip->poll_mode = chip_info->poll_mode;
		chip->type = chip_info->type;
	}

	chip->tmode = SPI_TMOD_TR;
#endif
	return 0;
}

static void f75115_spi_cleanup(struct spi_device *spi)
{
#if 1

#else
	struct chip_data *chip = spi_get_ctldata(spi);

	kfree(chip);
	spi_set_ctldata(spi, NULL);
#endif
}

static void f75115_spi_set_cs(struct spi_device *spi, bool enable)
{
	struct usb_serial *serial;
	unsigned char data = 0;
	struct usb_device *dev;
	struct spi_controller *controller;
	int gpio_cs;

	controller = spi->controller;
	serial = spi_controller_get_devdata(spi->controller);
	dev = serial->dev;

	dev_dbg(&serial->interface->dev, "%s: cs: %d, enable: %d\n", __func__,
		spi->chip_select, enable);

	gpio_cs = controller->cs_gpios[spi->chip_select];
	if (enable)
		data = 1;

	gpio_set_value_cansleep(gpio_cs, data);
	//f75115_set_mask_normal_register(dev, pin_data_table[1], BIT(4), data);
}

static int f75115_spi_transfer_one(struct spi_controller *master,
				   struct spi_device *spi,
				   struct spi_transfer *transfer)
{
	struct usb_serial *serial;
	struct usb_device *dev;
	unsigned count = transfer->len;
	const u8 *tx = transfer->tx_buf;
	u8 *rx = transfer->rx_buf;
	u8 word, stat;
	unsigned int retry;
	int status;

	serial = spi_controller_get_devdata(spi->controller);
	dev = serial->dev;

	dev_dbg(&serial->interface->dev, "%s\n", __func__);

	while (count > 0) {
		retry = 100;

		if (tx)
			word = *tx++;
		else
			word = 0xff;

		// tx
		status = f75115_set_normal_register(dev, 0x2af3,
						    word); // tx data
		if (status)
			return status;

		status = f75115_set_normal_register(dev, 0x2af5,
						    BIT(3) | BIT(6)); // send
		if (status)
			return status;

		while (retry--) {
			status =
				f75115_get_normal_register(dev, 0x2af5, &stat);
			if (status)
				return status;
			else if (stat & BIT(4))
				break;
		}

		// rx
		while (retry--) {
			if (stat & BIT(1))
				break;

			f75115_get_normal_register(dev, 0x2af5, &stat);
		}

		f75115_get_normal_register(dev, 0x2af4, &word);
		f75115_set_normal_register(dev, 0x2af5, BIT(6));

		if (rx)
			*rx++ = word;

		count -= 1;
	}

	spi_finalize_current_transfer(master);

	return transfer->len - count;
}

static void f75115_spi_handle_err(struct spi_controller *master,
				  struct spi_message *msg)
{
	struct usb_serial *serial;
	int status;

	serial = spi_controller_get_devdata(master);
	dev_info(&serial->interface->dev, "%s\n", __func__);

	status = f75115_spi_hw_init(serial);
	if (status) {
		dev_warn(&serial->interface->dev, "%s: failed: %d\n", __func__,
			 status);
	}
}

static int f75115_spi_hw_init(struct usb_serial *serial)
{
	struct usb_device *dev = serial->dev;
	int status;

	status = f75115_set_normal_register(dev, 0x2af2, 0x80); // dual
	if (status)
		return status;

	status = f75115_set_normal_register(dev, 0x2af5,
					    BIT(6)); // clear status
	if (status)
		return status;

	return 0;
}

static int f75115_spi_cs_gpio_deinit(struct usb_serial *serial, int gpio_start)
{
	devm_gpio_free(&serial->interface->dev,
		       gpio_start + F75115_SPI_CS0_PIN);
	devm_gpio_free(&serial->interface->dev,
		       gpio_start + F75115_SPI_CS1_PIN);
	return 0;
}

static int f75115_spi_cs_gpio_init(struct usb_serial *serial, int gpio_start)
{
	struct f75115_serial_private *priv;
	int status;
	int i;
	int cs[F75115_SPI_MAX_CS_NUM] = { F75115_SPI_CS0_PIN,
					  F75115_SPI_CS1_PIN };

	priv = usb_get_serial_data(serial);
	priv->spi_master->cs_gpios =
		devm_kzalloc(&serial->interface->dev,
			     F75115_SPI_MAX_CS_NUM * sizeof(int), GFP_KERNEL);
	if (!priv->spi_master->cs_gpios)
		return -ENOMEM;

	for (i = 0; i < F75115_SPI_MAX_CS_NUM; ++i) {
		status = devm_gpio_request_one(&serial->interface->dev,
					       gpio_start + cs[i],
					       GPIOF_OUT_INIT_HIGH,
					       "F75115 CS");
		if (status) {
			dev_warn(&serial->interface->dev,
				 "cant alloc gpio: %d\n", cs[i]);
			return status;
		}

		priv->spi_master->cs_gpios[i] = gpio_start + cs[i];
	}

	return 0;
}
#else
static void f75115_spi_release(struct device *dev)
{
}
#endif

static int f75115_spi_init(struct usb_serial *serial, int gpio_start)
{
	struct f75115_serial_private *priv;

#ifdef SPI_HWMODE
	struct spi_master *master;
	int status;
	int i;

	if (!F75115_SPI_MAX_CS_NUM)
		return 0;

	priv = usb_get_serial_data(serial);
	master = spi_alloc_master(&serial->interface->dev,
				  sizeof(struct f75115_serial_private *));
	if (!master)
		return -ENOMEM;

	priv->spi_master = master;
	spi_controller_set_devdata(priv->spi_master, serial);

	f75115_spi_hw_init(serial);
	f75115_spi_cs_gpio_init(serial, gpio_start);

	master->bits_per_word_mask = SPI_BPW_RANGE_MASK(1, 8);
	master->mode_bits = SPI_CPHA | SPI_CPOL;
	master->bus_num = PLATFORM_DEVID_AUTO;
	master->num_chipselect = F75115_SPI_MAX_CS_NUM;
	master->setup = f75115_spi_setup;
	master->cleanup = f75115_spi_cleanup;
	master->set_cs = f75115_spi_set_cs;
	master->transfer_one = f75115_spi_transfer_one;
	master->handle_err = f75115_spi_handle_err;
	master->min_speed_hz = 46875;
	master->max_speed_hz = 6000000;

	status = spi_register_master(master);
	if (status) {
		dev_err(&master->dev, "register spi master failed: %d\n",
			status);
		return status;
	}

	for (i = 0; i < F75115_SPI_MAX_CS_NUM; ++i) {
		priv->info[i].max_speed_hz = 6000000;
#if LINUX_VERSION_CODE <= KERNEL_VERSION(5, 14, 0)
		strcpy(priv->info[i].modalias, "spidev");
#else
		strcpy(priv->info[i].modalias, "spi-petra");
#endif
		priv->info[i].chip_select = i;

		priv->spi_device[i] = spi_new_device(master, &priv->info[i]);
	}

	return 0;
#else

	struct spi_gpio_platform_data_extra *data_extra;
	int cs[] = { F75115_SPI_CS0_PIN, F75115_SPI_CS1_PIN };
	int status;
	int size;
	int i;

	if (!F75115_SPI_MAX_CS_NUM)
		return 0;

	priv = usb_get_serial_data(serial);
	size = sizeof(struct spi_gpio_platform_data_extra) +
	       sizeof(unsigned int) * F75115_SPI_MAX_CS_NUM;

	data_extra = devm_kzalloc(&serial->interface->dev, size, GFP_KERNEL);
	if (!data_extra)
		return -ENOMEM;

	data_extra->gpio_data.sck = gpio_start + F75115_SPI_SCK_PIN;
	data_extra->gpio_data.mosi = gpio_start + F75115_SPI_MOSI_PIN;
	data_extra->gpio_data.miso = gpio_start + F75115_SPI_MISO_PIN;
	data_extra->gpio_data.num_chipselect = F75115_SPI_MAX_CS_NUM;

	for (i = 0; i < F75115_SPI_MAX_CS_NUM; ++i)
		data_extra->cs[i] = gpio_start + cs[i];

	priv->spi_device.name = "f75115_spi_gpio";
	priv->spi_device.id = PLATFORM_DEVID_AUTO;
	priv->spi_device.dev.platform_data = data_extra;
	priv->spi_device.dev.release = f75115_spi_release;
	priv->spi_device.dev.parent = &serial->interface->dev;

	status = platform_device_register(&priv->spi_device);
	if (status) {
		dev_err(&serial->interface->dev, "%s: failed\n", __func__);
		return status;
	}
#endif
	return 0;
}

static void f75115_spi_deinit(struct usb_serial *serial)
{
	struct f75115_serial_private *priv;

	if (!F75115_SPI_MAX_CS_NUM)
		return;

	priv = usb_get_serial_data(serial);

#ifdef SPI_HWMODE
	spi_unregister_master(priv->spi_master);
#else
	platform_device_unregister(&priv->spi_device);
#endif

	f75115_spi_cs_gpio_deinit(serial, priv->f75115_gpio_chip.base);
}

/*
 * Get the current port index of this device. e.g., 0 is the start index of
 * this device.
 */
static int f75115_port_index(struct usb_serial_port *port)
{
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 10, 0)
	int logic = port->port_number;
#else
	int logic = port->number - port->serial->minor;
#endif
	return logic;
}

/*
 * Find logic serial port index with H/W phy index mapping
 */
static int f75115_phy_to_logic_port(struct usb_serial *serial, int phy)
{
	int count = 0, i;
	struct f75115_serial_private *serial_priv =
		usb_get_serial_data(serial);

	for (i = 0; i < phy; ++i) {
		if (serial_priv->default_conf_data[i] &
		    F75115_PORT_UNAVAILABLE)
			continue;

		++count;
	}

	dev_dbg(&serial->dev->dev, "%s: phy:%d count:%d\n", __func__, phy,
		count);
	return count;
}

static int f75115_set_normal_register_size(struct usb_device *dev, u16 reg,
					   u8 *data, int data_size)
{
	int count = F75115_USB_MAX_RETRY;
	int status = 0;
	u8 *tmp;

	tmp = kmalloc(data_size, GFP_KERNEL);
	if (!tmp)
		return -ENOMEM;

	memcpy(tmp, data, data_size);

	/*
	 * Our device maybe not reply when heavily loading,
	 * We'll retry for F75115_USB_MAX_RETRY times
	 */
	while (count--) {
		status = usb_control_msg(dev, usb_sndctrlpipe(dev, 0),
					 F75115_SET_GET_REGISTER,
					 USB_TYPE_VENDOR | USB_DIR_OUT, reg, 0,
					 tmp, data_size, F75115_USB_TIMEOUT);
		if (status > 0)
			break;

		if (status == 0)
			status = -EIO;
	}

	if (status < 0) {
		dev_err(&dev->dev, "%s ERROR reg:%x status:%i failed\n",
			__func__, reg, status);
		kfree(tmp);
		return status;
	}

	kfree(tmp);
	return 0;
}

static int f75115_get_normal_register_size(struct usb_device *dev, u16 reg,
					   u8 *data, int data_size)
{
	int count = F75115_USB_MAX_RETRY;
	int status;
	u8 *tmp;

	tmp = kmalloc(data_size, GFP_KERNEL);
	if (!tmp)
		return -ENOMEM;

	/*
	 * Our device maybe not reply when heavily loading,
	 * We'll retry for F75115_USB_MAX_RETRY times
	 */
	while (count--) {
		status = usb_control_msg(dev, usb_rcvctrlpipe(dev, 0),
					 F75115_SET_GET_REGISTER,
					 USB_TYPE_VENDOR | USB_DIR_IN, reg, 0,
					 tmp, data_size, F75115_USB_TIMEOUT);
		if (status > 0)
			break;

		if (status == 0)
			status = -EIO;
	}

	if (status < 0) {
		dev_err(&dev->dev, "%s ERROR reg:%x status:%i failed\n",
			__func__, reg, status);
		kfree(tmp);
		return status;
	}

	memcpy(data, tmp, data_size);
	kfree(tmp);
	return 0;
}

static int f75115_set_normal_register(struct usb_device *dev, u16 reg, u8 data)
{
	return f75115_set_normal_register_size(dev, reg, &data, 1);
}

static int f75115_get_normal_register(struct usb_device *dev, u16 reg,
				      u8 *data)
{
	return f75115_get_normal_register_size(dev, reg, data, 1);
}

static int f75115_set_mask_normal_register(struct usb_device *dev, u16 reg,
					   u8 mask, u8 data)
{
	int status;
	u8 tmp;

	status = f75115_get_normal_register(dev, reg, &tmp);
	if (status)
		return status;

	tmp = (tmp & ~mask) | (mask & data);

	status = f75115_set_normal_register(dev, reg, tmp);
	if (status)
		return status;

	return 0;
}

static int f75115_command_delay(struct usb_serial *usbserial)
{
	unsigned int count = F75115_MAX_BUS_RETRY;
	unsigned char tmp;
	int status;
	struct usb_device *dev = usbserial->dev;

	do {
		status = f75115_get_normal_register(dev, F75115_BUS_REG_STATUS,
						    &tmp);
		if (status)
			return status;

		if (tmp & F75115_BUS_BUSY)
			continue;

		if (tmp & F75115_BUS_IDLE)
			break;

	} while (--count);

	if (!count)
		return -EIO;

	status = f75115_set_normal_register(dev, F75115_BUS_REG_STATUS,
					    tmp & ~F75115_BUS_IDLE);
	if (status)
		return status;

	return 0;
}

static int f75115_get_normal_register_with_delay(struct usb_serial *usbserial,
						 u16 reg, u8 *data)
{
	int status;
	struct usb_device *dev = usbserial->dev;

	status = f75115_get_normal_register(dev, reg, data);
	if (status)
		return status;

	status = f75115_command_delay(usbserial);
	if (status)
		return status;

	return 0;
}

static int f75115_set_normal_register_with_delay(struct usb_serial *usbserial,
						 u16 reg, u8 data)
{
	int status;
	struct usb_device *dev = usbserial->dev;

	status = f75115_set_normal_register(dev, reg, data);
	if (status)
		return status;

	status = f75115_command_delay(usbserial);
	if (status)
		return status;

	return 0;
}

static int f75115_read_data(struct usb_serial *usbserial, u32 address,
			    u32 size, unsigned char *buf)
{
	u32 read_size, count;
	u32 block = 0;
	u16 reg_tmp;
	u8 tmp_buf[F75115_MAX_DATA_BLOCK];
	int status, offset;

	status = f75115_set_normal_register_with_delay(
		usbserial, F75115_BUS_REG_START, F75115_CMD_READ);
	if (status)
		return status;

	status = f75115_set_normal_register_with_delay(
		usbserial, F75115_BUS_REG_START, (address >> 16) & 0xff);
	if (status)
		return status;

	status = f75115_set_normal_register_with_delay(
		usbserial, F75115_BUS_REG_START, (address >> 8) & 0xff);
	if (status)
		return status;

	status = f75115_set_normal_register_with_delay(
		usbserial, F75115_BUS_REG_START, (address >> 0) & 0xff);
	if (status)
		return status;

	/* continuous read mode */
	do {
		read_size = min_t(u32, F75115_MAX_DATA_BLOCK, size);

		for (count = 0; count < read_size; ++count) {
			/* We need write F75115_BUS_REG_END when final byte */
			if ((size <= F75115_MAX_DATA_BLOCK) &&
			    (read_size == (count + 1)))
				reg_tmp = F75115_BUS_REG_END;
			else
				reg_tmp = F75115_BUS_REG_START;

			/*
			 * dummy code, force IC to generate a read
			 * pulse, the set of value 0xf1 is dont care
			 * (any value is ok)
			 */
			status = f75115_set_normal_register_with_delay(
				usbserial, reg_tmp, 0xf1);
			if (status)
				return status;

			status = f75115_get_normal_register_with_delay(
				usbserial, F75115_BUS_READ_DATA,
				&tmp_buf[count]);
			if (status)
				return status;

			offset = count + block * F75115_MAX_DATA_BLOCK;
			buf[offset] = tmp_buf[count];
		}

		size -= read_size;
		++block;
	} while (size);

	return 0;
}

/*
 * This function maybe cause IC no workable, Please take this carefully.
 *
 * The function is used to modify the configuration area of this device
 * (F75115_CUSTOM_ADDRESS_START). If wrong operation with this function, it'll
 * make the device malfuncional.
 */
static int f75115_write_data(struct usb_serial *usbserial, u32 address,
			     u32 size, unsigned char *buf)
{
	u32 count, write_size;
	u32 block = 0;
	u16 reg_tmp;
	int offset, status;

	status = f75115_set_normal_register_with_delay(
		usbserial, F75115_BUS_REG_END, F75115_CMD_ENABLE_WR);
	if (status)
		return status;

	status = f75115_set_normal_register_with_delay(
		usbserial, F75115_BUS_REG_START, F75115_CMD_PROGRAM);
	if (status)
		return status;

	status = f75115_set_normal_register_with_delay(
		usbserial, F75115_BUS_REG_START, (address >> 16) & 0xff);
	if (status)
		return status;

	status = f75115_set_normal_register_with_delay(
		usbserial, F75115_BUS_REG_START, (address >> 8) & 0xff);
	if (status)
		return status;

	status = f75115_set_normal_register_with_delay(
		usbserial, F75115_BUS_REG_START, (address >> 0) & 0xff);
	if (status)
		return status;

	do {
		write_size = min_t(u32, F75115_MAX_DATA_BLOCK, size);

		for (count = 0; count < write_size; ++count) {
			offset = count + block * F75115_MAX_DATA_BLOCK;

			if ((size <= F75115_MAX_DATA_BLOCK) &&
			    (write_size == (count + 1)))
				reg_tmp = F75115_BUS_REG_END;
			else
				reg_tmp = F75115_BUS_REG_START;

			status = f75115_set_normal_register_with_delay(
				usbserial, reg_tmp, buf[offset]);
			if (status)
				return status;
		}

		size -= write_size;
		++block;
	} while (size);

	return 0;
}

/*
 * This function maybe cause IC no workable, Please take this carefully.
 *
 * The function is used to clear the configuration area of this device
 * (F75115_CUSTOM_ADDRESS_START). If wrong operation with this function, it'll
 * make the device malfuncional.
 */
static int f75115_erase_sector(struct usb_serial *usbserial, int address)
{
	u8 current_status = 0;
	int status;
	unsigned int count = F75115_MAX_BUS_RETRY;

	status = f75115_set_normal_register_with_delay(
		usbserial, F75115_BUS_REG_END, F75115_CMD_ENABLE_WR);
	if (status)
		return status;

	status = f75115_set_normal_register_with_delay(
		usbserial, F75115_BUS_REG_START, F75115_CMD_ERASE);
	if (status)
		return status;

	status = f75115_set_normal_register_with_delay(
		usbserial, F75115_BUS_REG_START, (address >> 16) & 0xff);
	if (status)
		return status;

	status = f75115_set_normal_register_with_delay(
		usbserial, F75115_BUS_REG_START, (address >> 8) & 0xff);
	if (status)
		return status;

	status = f75115_set_normal_register_with_delay(
		usbserial, F75115_BUS_REG_END, (address >> 0) & 0xff);
	if (status)
		return status;

	while (--count) {
		status = f75115_set_normal_register_with_delay(
			usbserial, F75115_BUS_REG_START,
			F75115_CMD_READ_STATUS);
		if (status)
			return status;

		/* dummy write, any value is acceptable */
		status = f75115_set_normal_register_with_delay(
			usbserial, F75115_BUS_REG_END, 0xff);
		if (status)
			return status;

		status = f75115_get_normal_register_with_delay(
			usbserial, F75115_BUS_READ_DATA, &current_status);
		if (status)
			return status;

		if (!(F75115_MEDIA_BUSY_STATUS & current_status)) {
			dev_dbg(&usbserial->dev->dev,
				"%s: data:%x, count:%d, ok\n", __func__,
				current_status, count);
			break;
		}
	}

	return 0;
}

static int f75115_pin_init(struct usb_serial *serial)
{
	int status;
	int i;

	// force all sfr gpio input mode
#if 0	
	for (i = 0; i < 5; ++i) {
		status = f75115_set_normal_register(serial->dev,
						pin1_mode_table[i], 0xff);
		if (status)
			return status;

		status = f75115_set_normal_register(serial->dev,
						pin0_mode_table[i], 0x00);
		if (status)
			return status;

		status = f75115_set_normal_register(serial->dev,
						pin_data_table[i], 0xff);
		if (status)
			return status;
	}
#endif
	for (i = 0; i < 3; ++i) {
		status = f75115_set_normal_register(serial->dev,
						    0x120C + i * 0x10, 0x00);
		if (status)
			return status;
	}

	// force all gpio into input mode
	f75115_set_normal_register(serial->dev, 0x1620, 0x00);
	f75115_set_normal_register(serial->dev, 0x1640, 0x00);
	f75115_set_normal_register(serial->dev, 0x1660, 0x00);
	f75115_set_normal_register(serial->dev, 0x1661, 0x00);

	if (!full_uart)
		status = f75115_set_normal_register(serial->dev, 0x123C, 0x03);
	else
		status = f75115_set_normal_register(serial->dev, 0x123C, 0xff);

	return status;
}

static int f75115_prepare_write_buffer(struct usb_serial_port *port,
				       void *dest, size_t size)
{
	unsigned char *ptr = (unsigned char *)dest;
	struct f75115_port_private *port_priv = usb_get_serial_port_data(port);
	int port_num = port_priv->phy;
	struct usb_serial *serial = port->serial;

	WARN_ON(size != serial->port[0]->bulk_out_size);

	/*
	 * The block layout is fixed with 4x128 Bytes, per 128 Bytes
	 * for a port.
	 * index 0: port phy idx (e.g., 0,1,2,3)
	 * index 1: only F75115_TOKEN_WRITE
	 * index 2: serial out size
	 * index 3: fix to 0
	 * index 4~127: serial out data block
	 */
	ptr[F75115_RECEIVE_BLOCK_SIZE * 0] = 0;
	ptr[F75115_RECEIVE_BLOCK_SIZE * 1] = 1;
	ptr[F75115_RECEIVE_BLOCK_SIZE * 2] = 2;
	ptr[F75115_RECEIVE_BLOCK_SIZE * 3] = 3;
	ptr[F75115_RECEIVE_BLOCK_SIZE * port_num + 1] = F75115_TOKEN_WRITE;
	ptr[F75115_RECEIVE_BLOCK_SIZE * port_num + 3] = 0;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 33)
	ptr[F75115_RECEIVE_BLOCK_SIZE * port_num + 2] = kfifo_out_locked(
		&port->write_fifo,
		&ptr[F75115_RECEIVE_BLOCK_SIZE * port_num + 4],
		F75115_MAX_TX_SIZE, &port->lock);
#else
	ptr[F75115_RECEIVE_BLOCK_SIZE * port_num + 2] =
		kfifo_get(port->write_fifo,
			  &ptr[F75115_RECEIVE_BLOCK_SIZE * port_num + 4],
			  F75115_MAX_TX_SIZE);
#endif

	return F75115_WRITE_BUFFER_SIZE;
}

static int f75115_submit_writer(struct usb_serial_port *port, gfp_t mem_flags)
{
	struct usb_serial *serial = port->serial;
	struct f75115_port_private *port_priv = usb_get_serial_port_data(port);
	struct f75115_serial_private *serial_priv =
		usb_get_serial_data(serial);
	struct tty_struct *tty;
	struct urb *urb;
	bool cts_status = true;
	unsigned long flags;
	int updating_data, result;

	tty = tty_port_tty_get(&port->port);
	if (tty) {
		/* check H/W Flow status */
		if (C_CRTSCTS(tty)) {
			spin_lock_irqsave(&port_priv->msr_lock, flags);
			cts_status = !!(port_priv->shadow_msr & UART_MSR_CTS);
			spin_unlock_irqrestore(&port_priv->msr_lock, flags);
		}

		tty_kref_put(tty);
	}

	dev_dbg(&port->dev, "%s: check CTS: %d\n", __func__, cts_status);

	if (!cts_status)
		return 0;

	/* someone is changing setting, pause TX */
	updating_data = mutex_is_locked(&serial_priv->change_mode_mutex);
	if (updating_data)
		return 0;

	dev_dbg(&port->dev, "%s: check FIFO\n", __func__);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 33)
	/* check is any data in write_fifo */
	spin_lock_irqsave(&port->lock, flags);

	if (kfifo_is_empty(&port->write_fifo)) {
		spin_unlock_irqrestore(&port->lock, flags);
		return 0;
	}

	spin_unlock_irqrestore(&port->lock, flags);
#else
	if (!kfifo_len(port->write_fifo))
		return 0;
#endif

	dev_dbg(&port->dev, "%s: check TX_EMPTY\n", __func__);

	/* check H/W is TXEMPTY */
	spin_lock_irqsave(&serial_priv->tx_empty_lock, flags);

	if (serial_priv->is_phy_port_not_empty[port_priv->phy]) {
		spin_unlock_irqrestore(&serial_priv->tx_empty_lock, flags);
		return 0;
	}

	serial_priv->is_phy_port_not_empty[port_priv->phy] = true;
	spin_unlock_irqrestore(&serial_priv->tx_empty_lock, flags);

	dev_dbg(&port->dev, "%s: Ready to send\n", __func__);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 35)
	urb = port->write_urbs[0];
	f75115_prepare_write_buffer(port, port->bulk_out_buffers[0],
				    port->bulk_out_size);
#else
	urb = port->write_urb;
	f75115_prepare_write_buffer(port, port->bulk_out_buffer,
				    port->bulk_out_size);
#endif
	urb->transfer_buffer_length = F75115_WRITE_BUFFER_SIZE;

	result = usb_submit_urb(urb, mem_flags);
	if (result) {
		dev_err(&port->dev, "%s: submit error, result:%d\n", __func__,
			result);
		return result;
	}

	return 0;
}

static int f75115_switch_gpio_mode(struct usb_serial_port *port, u8 mode)
{
	struct f75115_port_private *port_priv = usb_get_serial_port_data(port);
	int x = port_priv->phy;
	int y;
	int val;
	int status;
	int idx = (mode > F75115_PIN_SET_MAX) ? F75115_PIN_SET_DEFAULT : mode;
	struct usb_device *dev = port->serial->dev;
	const struct io_map_value *request_mode = f75115_mode_control[idx];
	/* our EVB m0 sometime will print as SD(Shutdown) */
	const struct pin_data *pins[3] = { &request_mode->port[x].m1,
					   &request_mode->port[x].m2,
					   &request_mode->port[x].m0_sd };

	if (mode > F75115_PIN_SET_MAX)
		return -EINVAL;

	for (y = 0; y < ARRAY_SIZE(pins); ++y) {
#if 1
		val = pins[y]->port_mode_0.reg_bit ? 0xff : 0x00;
		status = f75115_set_mask_normal_register(
			dev, pins[y]->port_mode_0.reg_address,
			BIT(pins[y]->port_mode_0.reg_offset), val);
		if (status) {
			dev_err(&port->dev, "%s: failed, %d\n", __func__,
				__LINE__);
			return status;
		}

		val = pins[y]->port_mode_1.reg_bit ? 0xff : 0x00;
		status = f75115_set_mask_normal_register(
			dev, pins[y]->port_mode_1.reg_address,
			BIT(pins[y]->port_mode_1.reg_offset), val);
		if (status) {
			dev_err(&port->dev, "%s: failed, %d\n", __func__,
				__LINE__);
			return status;
		}
#endif
		val = pins[y]->port_io.reg_bit ? 0xff : 0x00;
		status = f75115_set_mask_normal_register(
			dev, pins[y]->port_io.reg_address,
			BIT(pins[y]->port_io.reg_offset), val);
		if (status) {
			dev_err(&port->dev, "%s: failed, index:%d\n", __func__,
				y);
			return status;
		}
	}

	return 0;
}

#ifdef CONFIG_GPIOLIB
static int f75115_gpio_hw_gpio_set(struct gpio_chip *chip, unsigned gpio_num,
				   bool init, int val)
{
#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 5, 0)
	struct usb_interface *intf =
		container_of(chip->dev, struct usb_interface, dev);
#else
	struct usb_interface *intf =
		container_of(chip->parent, struct usb_interface, dev);
#endif
	struct usb_serial *serial = usb_get_intfdata(intf);
	struct f75115_serial_private *serial_priv =
		usb_get_serial_data(serial);
	int set = gpio_num / 8;
	int pin = gpio_num % 8;
	int status;
	u8 tmp;

	dev_dbg(&serial->interface->dev, "%s: GPIO%d%d, init: %d, val: %d\n",
		__func__, set, pin, init, val);

	mutex_lock(&serial_priv->change_mode_mutex);

	if (!init && f75115_gpio_special_func(set, pin))
		init = true;

	if (init ||
	    (serial_priv->gpio_opendrain_changed_bit & BIT(gpio_num))) {
		serial_priv->gpio_opendrain_changed_bit =
			serial_priv->gpio_opendrain_changed_bit &
			~BIT(gpio_num);

		status = f75115_set_mask_normal_register(serial->dev, 0x1620,
							 BIT(pin), BIT(pin));
		if (status) {
			dev_err(&serial->interface->dev,
				"%s: write output failed\n", __func__);
			goto out;
		}

		if (serial_priv->gpio_opendrain_mode & BIT(gpio_num))
			tmp = 0;
		else
			tmp = BIT(pin);

		status = f75115_set_mask_normal_register(serial->dev, 0x1640,
							 BIT(pin), tmp);
		if (status) {
			dev_err(&serial->interface->dev,
				"%s: write mode failed\n", __func__);
			goto out;
		}
	}

#ifdef GPIO_OUTPUT_FAST_MODE
	tmp = serial_priv->gpio_data[set];
	tmp = (tmp & ~BIT(pin)) | (BIT(pin) & (val ? BIT(pin) : 0));
	serial_priv->gpio_data[set] = tmp;

	status = f75115_set_normal_register(serial->dev, 0x1630, tmp);
	if (status) {
		dev_err(&serial->interface->dev, "%s: set data failed\n",
			__func__);
		goto out;
	}
#else
	status = f75115_set_mask_normal_register(serial->dev, 0x1630, BIT(pin),
						 val ? BIT(pin) : 0);
	if (status) {
		dev_err(&serial->interface->dev, "%s: write pin1 failed\n",
			__func__);
		goto out;
	}
#endif

out:
	mutex_unlock(&serial_priv->change_mode_mutex);

	return status;
}

static int f75115_gpio_hw_gpio_get(struct gpio_chip *chip, unsigned gpio_num,
				   bool init)
{
#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 5, 0)
	struct usb_interface *intf =
		container_of(chip->dev, struct usb_interface, dev);
#else
	struct usb_interface *intf =
		container_of(chip->parent, struct usb_interface, dev);
#endif
	struct usb_serial *serial = usb_get_intfdata(intf);
	struct f75115_serial_private *serial_priv =
		usb_get_serial_data(serial);
	int set = gpio_num / 8;
	int pin = gpio_num % 8;
	int status;
	u8 tmp;

	dev_dbg(&serial->interface->dev, "%s: GPIO%d%d, init: %d\n", __func__,
		set, pin, init);

	mutex_lock(&serial_priv->change_mode_mutex);

	if (init) {
		status = f75115_set_mask_normal_register(serial->dev, 0x1620,
							 BIT(pin), 0);
		if (status) {
			dev_err(&serial->interface->dev,
				"%s: write input failed\n", __func__);
			goto out;
		}
	}

	status = f75115_get_normal_register(serial->dev, 0x1650, &tmp);
	if (status) {
		dev_err(&serial->interface->dev, "%s: write data failed\n",
			__func__);
		goto out;
	}

#ifdef GPIO_OUTPUT_FAST_MODE
	serial_priv->gpio_data[set] = tmp;
#endif

	status = !!(tmp & BIT(pin));

	dev_dbg(&serial->interface->dev,
		"%s: get: %d, pin: %d, init: %d, data: %d\n", __func__, set,
		pin, init, status);

out:
	mutex_unlock(&serial_priv->change_mode_mutex);

	return status;
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 8, 0)
static int f75115_gpio_hw_gpio_get_dir(struct usb_device *dev,
				       unsigned gpio_num)
{
	int set = gpio_num / 8;
	int pin = gpio_num % 8;
	int status;
	u8 data;

	if (set != 5 && set != 6)
		return -EINVAL;

	status = f75115_get_normal_register(dev, 0x1620 + (set - 5), &data);
	if (status)
		return status;

	if (data & BIT(pin))
		return GPIOF_DIR_OUT;

	return GPIOF_DIR_IN;
}
#endif

static int f75115_gpio_hw_sfr_set(struct gpio_chip *chip, unsigned gpio_num,
				  bool init, int val)
{
#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 5, 0)
	struct usb_interface *intf =
		container_of(chip->dev, struct usb_interface, dev);
#else
	struct usb_interface *intf =
		container_of(chip->parent, struct usb_interface, dev);
#endif
	struct usb_serial *serial = usb_get_intfdata(intf);
	struct f75115_serial_private *serial_priv =
		usb_get_serial_data(serial);
	u8 mode_pin0;
	u8 mode_pin1;
	u8 tmp;
	int set = gpio_num / 8;
	int pin = gpio_num % 8;
	int status;

	dev_dbg(&serial->interface->dev,
		"%s: set: %d, pin: %d, init: %d, val: %d\n", __func__, set,
		pin, init, !!val);

	mutex_lock(&serial_priv->change_mode_mutex);

	if (!init && f75115_gpio_special_func(set, pin))
		init = true;

	//pr_info("%s: init: %d, bit: %d\n", __func__, init,
	//		!!(serial_priv->gpio_opendrain_changed_bit & BIT(gpio_num)));

	if (init ||
	    (serial_priv->gpio_opendrain_changed_bit & BIT(gpio_num))) {
		serial_priv->gpio_opendrain_changed_bit =
			serial_priv->gpio_opendrain_changed_bit &
			~BIT(gpio_num);

		mode_pin0 = BIT(pin);

		if (serial_priv->gpio_opendrain_mode & BIT(gpio_num))
			mode_pin1 = BIT(pin); /* open drain */
		else
			mode_pin1 = 0; /* push pull */

		dev_dbg(&serial->interface->dev,
			"%s: init pin0: %x, pin1: %x\n", __func__, mode_pin0,
			mode_pin1);

		status = f75115_set_mask_normal_register(serial->dev,
							 pin0_mode_table[set],
							 BIT(pin), mode_pin0);
		if (status) {
			dev_err(&serial->interface->dev,
				"%s: write pin0 failed\n", __func__);
			goto out;
		}

		status = f75115_set_mask_normal_register(serial->dev,
							 pin1_mode_table[set],
							 BIT(pin), mode_pin1);
		if (status) {
			dev_err(&serial->interface->dev,
				"%s: write pin1 failed\n", __func__);
			goto out;
		}
	}

#ifdef GPIO_OUTPUT_FAST_MODE
	tmp = serial_priv->gpio_data[set];
	tmp = (tmp & ~BIT(pin)) | (BIT(pin) & (val ? BIT(pin) : 0));
	serial_priv->gpio_data[set] = tmp;

	status = f75115_set_normal_register(serial->dev, pin_data_table[set],
					    tmp);
	if (status) {
		dev_err(&serial->interface->dev, "%s: set data failed\n",
			__func__);
		goto out;
	}

#else
	status = f75115_set_mask_normal_register(serial->dev,
						 pin_data_table[set], BIT(pin),
						 val ? BIT(pin) : 0);
	if (status) {
		dev_err(&serial->interface->dev, "%s: write data failed\n",
			__func__);
		goto out;
	}
#endif
out:
	mutex_unlock(&serial_priv->change_mode_mutex);

	return status;
}

static int f75115_gpio_hw_sfr_get(struct gpio_chip *chip, unsigned gpio_num,
				  bool init)
{
#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 5, 0)
	struct usb_interface *intf =
		container_of(chip->dev, struct usb_interface, dev);
#else
	struct usb_interface *intf =
		container_of(chip->parent, struct usb_interface, dev);
#endif
	struct usb_serial *serial = usb_get_intfdata(intf);
	struct f75115_serial_private *serial_priv =
		usb_get_serial_data(serial);
	int set = gpio_num / 8;
	int pin = gpio_num % 8;
	int status;
	u8 tmp;

	mutex_lock(&serial_priv->change_mode_mutex);

	if (init) {
		dev_dbg(&serial->interface->dev,
			"%s: init pin0: %x, pin1: %x\n", __func__,
			pin0_mode_table[set], pin1_mode_table[set]);

		status = f75115_set_mask_normal_register(
			serial->dev, pin0_mode_table[set], BIT(pin), 0);
		if (status) {
			dev_err(&serial->interface->dev,
				"%s: write pin0 failed\n", __func__);
			goto out;
		}

		status = f75115_set_mask_normal_register(
			serial->dev, pin1_mode_table[set], BIT(pin), BIT(pin));
		if (status) {
			dev_err(&serial->interface->dev,
				"%s: write pin1 failed\n", __func__);
			goto out;
		}
	}

	status = f75115_get_normal_register(serial->dev, pin_data_table[set],
					    &tmp);
	if (status) {
		dev_err(&serial->interface->dev, "%s: write data failed\n",
			__func__);
		goto out;
	}

	status = !!(tmp & BIT(pin));

#ifdef GPIO_OUTPUT_FAST_MODE
	serial_priv->gpio_data[set] = tmp;
#endif

	dev_dbg(&serial->interface->dev,
		"%s: get: %d, pin: %d, init: %d, data: %d\n", __func__, set,
		pin, init, status);

out:
	mutex_unlock(&serial_priv->change_mode_mutex);

	return status;
}

static int f75115_gpio_hw_set(struct gpio_chip *chip, unsigned gpio_num,
			      bool init, int val)
{
	int set = gpio_num / 8;

	if (set == 5)
		return f75115_gpio_hw_gpio_set(chip, gpio_num, init, val);

	return f75115_gpio_hw_sfr_set(chip, gpio_num, init, val);
}

static int f75115_gpio_hw_get(struct gpio_chip *chip, unsigned gpio_num,
			      bool init)
{
	int set = gpio_num / 8;

	if (set == 5)
		return f75115_gpio_hw_gpio_get(chip, gpio_num, init);

	return f75115_gpio_hw_sfr_get(chip, gpio_num, init);
}

static int f75115_gpio_get(struct gpio_chip *chip, unsigned gpio_num)
{
#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 5, 0)
	struct usb_interface *intf =
		container_of(chip->dev, struct usb_interface, dev);
#else
	struct usb_interface *intf =
		container_of(chip->parent, struct usb_interface, dev);
#endif
	struct usb_serial *serial = usb_get_intfdata(intf);

	dev_dbg(&serial->interface->dev, "%s\n", __func__);

	return f75115_gpio_hw_get(chip, gpio_num, false);
}

static int f75115_gpio_direction_in(struct gpio_chip *chip, unsigned gpio_num)
{
#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 5, 0)
	struct usb_interface *intf =
		container_of(chip->dev, struct usb_interface, dev);
#else
	struct usb_interface *intf =
		container_of(chip->parent, struct usb_interface, dev);
#endif
	struct usb_serial *serial = usb_get_intfdata(intf);
	int status;

	dev_dbg(&serial->interface->dev, "%s\n", __func__);

	status = f75115_gpio_hw_get(chip, gpio_num, true);
	if (status < 0)
		return status;

	return 0;
}

static int f75115_gpio_direction_out(struct gpio_chip *chip, unsigned gpio_num,
				     int val)
{
#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 5, 0)
	struct usb_interface *intf =
		container_of(chip->dev, struct usb_interface, dev);
#else
	struct usb_interface *intf =
		container_of(chip->parent, struct usb_interface, dev);
#endif
	struct usb_serial *serial = usb_get_intfdata(intf);

	dev_dbg(&serial->interface->dev, "%s\n", __func__);

	return f75115_gpio_hw_set(chip, gpio_num, true, val);
}

static void f75115_gpio_set(struct gpio_chip *chip, unsigned gpio_num, int val)
{
#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 5, 0)
	struct usb_interface *intf =
		container_of(chip->dev, struct usb_interface, dev);
#else
	struct usb_interface *intf =
		container_of(chip->parent, struct usb_interface, dev);
#endif
	struct usb_serial *serial = usb_get_intfdata(intf);

	dev_dbg(&serial->interface->dev, "%s\n", __func__);

	f75115_gpio_hw_set(chip, gpio_num, false, val);
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 8, 0)
static int f75115_gpio_get_direction(struct gpio_chip *chip, unsigned offset)
{
#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 5, 0)
	struct usb_interface *intf =
		container_of(chip->dev, struct usb_interface, dev);
#else
	struct usb_interface *intf =
		container_of(chip->parent, struct usb_interface, dev);
#endif
	struct usb_serial *serial = usb_get_intfdata(intf);
	struct f75115_serial_private *serial_priv =
		usb_get_serial_data(serial);
	int set = offset / 8;
	int pin = offset % 8;
	int status;
	int mode = 0;
	u8 tmp;

	dev_dbg(&serial->interface->dev, "%s\n", __func__);

	mutex_lock(&serial_priv->change_mode_mutex);

	if (set >= 5) {
		status = f75115_gpio_hw_gpio_get_dir(serial->dev, offset);
		mutex_unlock(&serial_priv->change_mode_mutex);
		return status;

	} else {
		status = f75115_get_normal_register(
			serial->dev, pin0_mode_table[set], &tmp);
		if (status) {
			dev_err(&serial->interface->dev,
				"%s: write pin0 failed\n", __func__);
			mutex_unlock(&serial_priv->change_mode_mutex);
			return status;
		}

		mode |= !!(tmp & BIT(pin)) << 0;

		//pr_info("%s: set: %d\n", __func__, set);
		status = f75115_get_normal_register(
			serial->dev, pin1_mode_table[set], &tmp);
		if (status) {
			dev_err(&serial->interface->dev,
				"%s: write pin1 failed\n", __func__);
			mutex_unlock(&serial_priv->change_mode_mutex);
			return status;
		}

		mode |= !!(tmp & BIT(pin)) << 1;

		mutex_unlock(&serial_priv->change_mode_mutex);

		switch (mode) {
		case 1:
		case 3:
			dev_dbg(&serial->interface->dev, "%s: mode: %d, out\n",
				__func__, mode);
			return GPIOF_DIR_OUT;
		case 2:
			dev_dbg(&serial->interface->dev, "%s: mode: %d, in\n",
				__func__, mode);
			return GPIOF_DIR_IN;
		}
	}

	return -EINVAL;
}
#endif

static int f75115_gpio_request(struct gpio_chip *chip, unsigned offset)
{
#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 5, 0)
	struct usb_interface *intf =
		container_of(chip->dev, struct usb_interface, dev);
#else
	struct usb_interface *intf =
		container_of(chip->parent, struct usb_interface, dev);
#endif
	struct usb_serial *serial = usb_get_intfdata(intf);

	dev_dbg(&serial->interface->dev, "%s\n", __func__);

	return 0;
}

static void f75115_gpio_free(struct gpio_chip *chip, unsigned offset)
{
#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 5, 0)
	struct usb_interface *intf =
		container_of(chip->dev, struct usb_interface, dev);
#else
	struct usb_interface *intf =
		container_of(chip->parent, struct usb_interface, dev);
#endif
	struct usb_serial *serial = usb_get_intfdata(intf);

	dev_dbg(&serial->interface->dev, "%s\n", __func__);
}

static struct gpio_chip f75115_gpio_chip_templete = {
	.owner = THIS_MODULE,
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 8, 0)
	.get_direction = f75115_gpio_get_direction,
#endif
	.get = f75115_gpio_get,
	.direction_input = f75115_gpio_direction_in,
	.set = f75115_gpio_set,
	.direction_output = f75115_gpio_direction_out,
	.request = f75115_gpio_request,
	.free = f75115_gpio_free,
	.ngpio = 48, /* M0(SD)/M1/M2(40) & GPIO0(8) */
	.base = -1,
};

static ssize_t open_drain_mode_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	struct f75115_serial_private *serial_priv;
	struct usb_interface *intf;
	struct usb_serial *serial;
	int status;

	intf = to_usb_interface(dev);
	serial = usb_get_intfdata(intf);
	serial_priv = usb_get_serial_data(serial);

	mutex_lock(&serial_priv->change_mode_mutex);
	status = sprintf(buf, "%lld\n", serial_priv->gpio_opendrain_mode);
	mutex_unlock(&serial_priv->change_mode_mutex);

	return status;
}

static ssize_t open_drain_mode_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	struct f75115_serial_private *serial_priv;
	struct usb_interface *intf;
	struct usb_serial *serial;
	unsigned long long tmp;

	intf = to_usb_interface(dev);
	serial = usb_get_intfdata(intf);
	serial_priv = usb_get_serial_data(serial);

	mutex_lock(&serial_priv->change_mode_mutex);

	tmp = simple_strtoull(buf, NULL, 0);
	serial_priv->gpio_opendrain_changed_bit |=
		tmp ^ serial_priv->gpio_opendrain_mode;
	serial_priv->gpio_opendrain_mode = tmp;

	mutex_unlock(&serial_priv->change_mode_mutex);

	return count;
}

static DEVICE_ATTR_RW(open_drain_mode);

static int f75115_prepare_gpio(struct usb_serial *serial)
{
	struct f75115_serial_private *serial_priv =
		usb_get_serial_data(serial);
	int max_name = 32;
	char *name = NULL;
	int rc;

	name = devm_kzalloc(&serial->interface->dev, max_name, GFP_KERNEL);
	if (!name)
		return -ENOMEM;

	memcpy(&serial_priv->f75115_gpio_chip, &f75115_gpio_chip_templete,
	       sizeof(f75115_gpio_chip_templete));

	snprintf(name, max_name - 1, "%s", IC_NAME);

	serial_priv->f75115_gpio_chip.label = name;

#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 5, 0)
	serial_priv->f75115_gpio_chip.dev = &serial->interface->dev;
#else
	serial_priv->f75115_gpio_chip.parent = &serial->interface->dev;
#endif

	rc = gpiochip_add(&serial_priv->f75115_gpio_chip);
	if (rc) {
		dev_err(&serial->interface->dev,
			"%s: f75115_prepare_gpio failed:%d\n", __func__, rc);
		return rc;
	}

	rc = device_create_file(&serial->interface->dev,
				&dev_attr_open_drain_mode);
	if (rc)
		return -EPERM;


	return 0;
}

static int f75115_release_gpio(struct usb_serial *serial)
{
	struct f75115_serial_private *serial_priv =
		usb_get_serial_data(serial);
        int base = serial_priv->f75115_gpio_chip.base;

        gpio_unexport(base + F75115_GPIO52);
        gpio_unexport(base + F75115_GPIO16);
        gpio_unexport(base + F75115_GPIO30);
        gpio_unexport(base + F75115_GPIO32);
        gpio_unexport(base + F75115_GPIO34);
        gpio_unexport(base + F75115_GPIO36);
        gpio_unexport(base + F75115_GPIO20);
        gpio_unexport(base + F75115_GPIO21);
        gpio_unexport(base + F75115_GPIO24);
        gpio_unexport(base + F75115_GPIO07);
        gpio_unexport(base + F75115_GPIO03);
        gpio_unexport(base + F75115_GPIO01);
        gpio_unexport(base + F75115_GPIO10);

        gpio_free(base + F75115_GPIO52);
        gpio_free(base + F75115_GPIO16);
        gpio_free(base + F75115_GPIO30);
        gpio_free(base + F75115_GPIO32);
        gpio_free(base + F75115_GPIO34);
        gpio_free(base + F75115_GPIO36);
        gpio_free(base + F75115_GPIO20);
        gpio_free(base + F75115_GPIO21);
        gpio_free(base + F75115_GPIO24);
        gpio_free(base + F75115_GPIO07);
        gpio_free(base + F75115_GPIO03);
        gpio_free(base + F75115_GPIO01);
        gpio_free(base + F75115_GPIO10);

	device_remove_file(&serial->interface->dev, &dev_attr_open_drain_mode);
	gpiochip_remove(&serial_priv->f75115_gpio_chip);

	return 0;
}
#else
static int f75115_prepare_gpio(struct usb_serial *serial)
{
	dev_info(&serial->interface->dev, "CONFIG_GPIOLIB is not enabled\n");
	dev_info(&serial->interface->dev,
		 "The GPIOLIB interface will not register\n");
	return 0;
}

static int f75115_release_gpio(struct usb_serial *serial)
{
	return 0;
}
#endif

static int f75115_calc_baud_divisor(u32 baudrate, u32 clockrate, u32 *remain)
{
	u32 divisor, rem;

	if (!baudrate)
		return 0;

	rem = clockrate % baudrate;

	if (remain)
		*remain = rem;

	/* Round to nearest divisor */
	divisor = DIV_ROUND_CLOSEST(clockrate, baudrate);

	return divisor;
}

static int f75115_setregister(struct usb_device *dev, u8 uart, u16 reg,
			      u8 data)
{
	return f75115_set_normal_register(dev, reg + uart * 0x10, data);
}

static int f75115_getregister(struct usb_device *dev, u8 uart, u16 reg,
			      u8 *data)
{
	return f75115_get_normal_register(dev, reg + uart * 0x10, data);
}

static int f75115_set_port_config(struct usb_device *dev, u8 port_number,
				  struct usb_serial_port *port, u32 baudrate,
				  u16 lcr)
{
	struct usb_serial *serial = port->serial;
	struct f75115_port_private *port_priv = usb_get_serial_port_data(port);
	u16 device_port = port_priv->phy;
	u32 divisor = 0;
	int status;
	u8 value;
	bool is_485_mode = false;
	bool is_need_invert = false;
#ifdef MULTIDROP_ENABLE
	bool is_9bits = port_priv->mode_9bit;
#endif

	value = CLKSEL_1DOT846_MHZ;
	divisor = f75115_calc_baud_divisor(baudrate, 115200, NULL);
	port_priv->current_baud_base = 115200;

	value &= ~(F75115_RS485_MODE | F75115_RS485_INVERT);
	value |= is_485_mode ? F75115_RS485_MODE : 0;
	value |= is_need_invert ? F75115_RS485_INVERT : 0;
	value |= CLKSEL_TX_DELAY_1BIT;

#ifdef MULTIDROP_ENABLE
	if (is_9bits)
		value |= 1L << 6; /* enable 9bits */
#endif

	status = f75115_setregister(serial->dev, device_port, CLK_SEL_REGISTER,
				    value);
	if (status) {
		dev_err(&port->dev, "%s: CLK REG setting failed\n", __func__);
		return status;
	}

	if (port_priv->current_baud_rate <= 1200)
		value = F75115_1X_RXTRIGGER; /* 128 FIFO & TL: 1x */
	else
		value = F75115_8X_RXTRIGGER; /* 128 FIFO & TL: 8x */

#ifdef MULTIDROP_ENABLE
	if (is_9bits)
		value |= 1L << 4; /* enable 9bits auto addr */
#endif

	status = f75115_setregister(serial->dev, device_port, CONFIG1_REGISTER,
				    value);
	if (status) {
		dev_err(&port->dev, "%s: CONFIG1 setting failed\n", __func__);
		return status;
	}

	if (port_priv->current_baud_rate <= 1200)
		value = UART_FCR_TRIGGER_1 | UART_FCR_ENABLE_FIFO; /* TL: 1 */
	else if (port_priv->current_baud_rate >= 1152000)
		value = UART_FCR_R_TRIG_10 | UART_FCR_ENABLE_FIFO; /* TL: 8 */
	else
		value = UART_FCR_R_TRIG_11 | UART_FCR_ENABLE_FIFO; /* TL: 14 */

	status = f75115_setregister(serial->dev, device_port,
				    FIFO_CONTROL_REGISTER, value);
	if (status) {
		dev_err(&port->dev, "%s: FCR setting failed\n", __func__);
		return status;
	}

	if (baudrate) {
		value = UART_LCR_DLAB;
		status = f75115_setregister(serial->dev, device_port,
					    LINE_CONTROL_REGISTER, value);
		if (status) {
			dev_err(&port->dev, "%s: set LCR failed, %d\n",
				__func__, status);
			return status;
		}

		value = divisor & 0xFF;
		status = f75115_setregister(serial->dev, device_port,
					    DIVISOR_LATCH_LSB, value);
		if (status) {
			dev_err(&port->dev, "%s: set DLAB LSB failed, %d\n",
				__func__, status);
			return status;
		}

		value = (divisor >> 8) & 0xFF;
		status = f75115_setregister(serial->dev, device_port,
					    DIVISOR_LATCH_MSB, value);
		if (status) {
			dev_err(&port->dev, "%s: set DLAB MSB failed, %d\n",
				__func__, status);
			return status;
		}
	}

	status = f75115_setregister(serial->dev, device_port,
				    LINE_CONTROL_REGISTER, lcr);
	if (status) {
		dev_err(&port->dev, "%s: set LCR failed, %d\n", __func__,
			status);
		return status;
	}

#ifdef MULTIDROP_ENABLE
	if (is_9bits) {
		status = f75115_setregister(dev, port_number,
					    SADDRESS_REGISTER,
					    port_priv->addr_9bit);
		if (status) {
			dev_err(&port->dev, "%s: failed, %d\n", __func__,
				__LINE__);
			return status;
		}

		status = f75115_setregister(dev, port_number, SADEN_REGISTER,
					    port_priv->bitmask_9bit);
		if (status) {
			dev_err(&port->dev, "%s: failed, %d\n", __func__,
				__LINE__);
			return status;
		}
	}
#endif
	return 0;
}

static int f75115_update_mctrl(struct usb_serial_port *port, unsigned int set,
			       unsigned int clear)
{
	struct usb_device *dev = port->serial->dev;
	struct f75115_port_private *port_priv = usb_get_serial_port_data(port);
	u8 tmp;
	int status;

	mutex_lock(&port_priv->msr_mutex);

	if (((set | clear) & (TIOCM_DTR | TIOCM_RTS | TIOCM_LOOP)) == 0) {
		dev_dbg(&dev->dev, "%s -DTR|RTS not being set|cleared\n",
			__func__);
		mutex_unlock(&port_priv->msr_mutex);
		return 0; /* no change */
	}

	/* 'set' takes precedence over 'clear' */
	clear &= ~set;

	/* always enable UART_MCR_OUT2 */
	tmp = UART_MCR_OUT2 | port_priv->shadow_mcr;

	if (clear & TIOCM_DTR) {
		tmp &= ~UART_MCR_DTR;
		dev_dbg(&dev->dev, "%s: port:%d clear DTR\n", __func__,
			port_priv->phy);
	}

	if (clear & TIOCM_RTS) {
		tmp &= ~UART_MCR_RTS;
		dev_dbg(&dev->dev, "%s: port:%d clear RTS\n", __func__,
			port_priv->phy);
	}

	if (clear & TIOCM_LOOP) {
		tmp &= ~UART_MCR_LOOP;
		dev_dbg(&dev->dev, "%s: port:%d clear LOOP\n", __func__,
			port_priv->phy);
	}

	if (set & TIOCM_DTR) {
		tmp |= UART_MCR_DTR;
		dev_dbg(&dev->dev, "%s: port:%d set DTR\n", __func__,
			port_priv->phy);
	}

	if (set & TIOCM_RTS) {
		tmp |= UART_MCR_RTS;
		dev_dbg(&dev->dev, "%s: port:%d set RTS\n", __func__,
			port_priv->phy);
	}

	if (set & TIOCM_LOOP) {
		tmp |= UART_MCR_LOOP;
		dev_dbg(&dev->dev, "%s: port:%d set LOOP\n", __func__,
			port_priv->phy);
	}

	status = f75115_setregister(dev, port_priv->phy,
				    MODEM_CONTROL_REGISTER, tmp);
	if (status < 0) {
		dev_err(&port->dev, "%s- Error from MODEM_CTRL URB: %i\n",
			__func__, status);
		mutex_unlock(&port_priv->msr_mutex);
		return status;
	}

	port_priv->shadow_mcr = tmp;
	mutex_unlock(&port_priv->msr_mutex);
	return 0;
}

/*
 * This function will search the data area with token F75115_CUSTOM_VALID_TOKEN
 * for latest configuration index. If nothing found (*index = -1), the caller
 * will load default configure in F75115_DEF_CONF_ADDRESS_START section
 */
static int f75115_find_config_idx(struct usb_serial *serial, uintptr_t *index)
{
	int idx, status;
	u8 custom_data;
	int offset;

	for (idx = F75115_CUSTOM_MAX_IDX - 1; idx >= 0; --idx) {
		offset = F75115_CUSTOM_ADDRESS_START +
			 F75115_CUSTOM_DATA_SIZE * idx;
		status = f75115_read_data(serial, offset, 1, &custom_data);
		if (status) {
			dev_err(&serial->dev->dev,
				"%s: read error, idx:%d, status:%d\n",
				__func__, idx, status);
			return status;
		}

		/*
		 * if had custom setting, override
		 * 1st byte is a indicator, 0xff is empty, 0xf0 is had data
		 */

		/* found */
		if (custom_data == F75115_CUSTOM_VALID_TOKEN)
			break;
	}

	*index = idx;
	return 0;
}

static inline struct f75115_serial_private *
to_f75115_serial_private(struct pwm_chip *c)
{
	return container_of(c, struct f75115_serial_private, f75115_pwm_chip);
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 4, 0)
static int f75115_pwm_apply(struct pwm_chip *chip, struct pwm_device *pwm,
			    const struct pwm_state *state)
#else
static int f75115_pwm_apply(struct pwm_chip *chip, struct pwm_device *pwm,
			    struct pwm_state *state)
#endif
{
	struct f75115_serial_private *serial_priv;
	struct usb_serial *serial;
	struct pwm_state old_state;
	int idx = pwm->hwpwm;
	u64 cur_period;
	int r = 0;

	serial_priv = to_f75115_serial_private(chip);
	serial = serial_priv->serial;

	cur_period = 1000000000UL * (serial_priv->pwm_cur_div) /
		     serial_priv->pwm_cur_clock;
	memcpy(&old_state, state, sizeof(*state));

	pwm_set_period(pwm, cur_period);

	//dev_info(&serial->dev->dev, "%s: 222 cur_period: %d", __func__, (int) cur_period);
	mutex_lock(&serial_priv->pwm_mutex);

#if 0
	if (serial_priv->pwm_cur_period != state->period) {
		dev_info(&serial->dev->dev, "%s: %d, %lld %lld", __func__, __LINE__, serial_priv->pwm_cur_period, state->period);
		r = -EINVAL;
		goto end;
	}
#endif

	if (state->polarity != PWM_POLARITY_NORMAL) {
		//dev_info(&serial->dev->dev, "%s: %d", __func__, __LINE__);
		r = -EINVAL;
		goto end;
	}

	//dev_info(&serial->dev->dev, "%s: 111 period: %lld", __func__, state->period);

	r = f75115_pwm_en_param_set(serial, idx, state->enabled,
				    state->duty_cycle);
	if (r) {
		dev_info(&serial->dev->dev, "%s: %d", __func__, __LINE__);
		goto end;
	}

	//dev_info(&serial->dev->dev, "%s: period: %lld", __func__, state->period);
	//dev_info(&serial->dev->dev, "%s: duty_cycle: %lld", __func__, state->duty_cycle);
	//dev_info(&serial->dev->dev, "%s: polarity: %d", __func__, state->polarity);
	//dev_info(&serial->dev->dev, "%s: enabled: %d", __func__, state->enabled);

end:
	mutex_unlock(&serial_priv->pwm_mutex);
	return r;
}

static void f75115_pwm_get_state(struct pwm_chip *chip, struct pwm_device *pwm,
				 struct pwm_state *s)
{
#if 1
	struct f75115_serial_private *serial_priv;
	struct usb_serial *serial;
	int idx = pwm->hwpwm;
	u64 cur_period;
	struct pwm_state *state = (struct pwm_state *)s;

	serial_priv = to_f75115_serial_private(chip);
	serial = serial_priv->serial;

	cur_period = 1000000000UL * (serial_priv->pwm_cur_div) /
		     serial_priv->pwm_cur_clock;
	state->period = cur_period;
	state->duty_cycle = serial_priv->pwm_cur_duty_us[idx];
	state->polarity = PWM_POLARITY_NORMAL;
	state->enabled = serial_priv->pwm_cur_en[idx];

	dev_info(&serial->dev->dev, "%s", __func__);
#else
	struct pwm_lpss_chip *lpwm = to_lpwm(chip);
	unsigned long base_unit_range;
	unsigned long long base_unit, freq, on_time_div;
	u32 ctrl;

	pm_runtime_get_sync(chip->dev);

	base_unit_range = BIT(lpwm->info->base_unit_bits);

	ctrl = pwm_lpss_read(pwm);
	on_time_div = 255 - (ctrl & PWM_ON_TIME_DIV_MASK);
	base_unit = (ctrl >> PWM_BASE_UNIT_SHIFT) & (base_unit_range - 1);

	freq = base_unit * lpwm->info->clk_rate;
	do_div(freq, base_unit_range);
	if (freq == 0)
		state->period = NSEC_PER_SEC;
	else
		state->period = NSEC_PER_SEC / (unsigned long)freq;

	on_time_div *= state->period;
	do_div(on_time_div, 255);
	state->duty_cycle = on_time_div;

	state->polarity = PWM_POLARITY_NORMAL;
	state->enabled = !!(ctrl & PWM_ENABLE);

	pm_runtime_put(chip->dev);
#endif
}

static const struct pwm_ops pwm_f75115_ops = {
	.apply = f75115_pwm_apply,
	.get_state = f75115_pwm_get_state,
	.owner = THIS_MODULE,
};

static int f75115_pwm_reload(struct usb_serial *serial)
{
	struct f75115_serial_private *serial_priv;
	int r, i;

	serial_priv = usb_get_serial_data(serial);

#if 0

#else
	r = f75115_pwm_div_set(serial, 1023);
	if (r) {
		pr_info("%s: f75115_pwm_div_set failed: %d", __func__, r);
		return r;
	}

	r = f75115_pwm_clock_set(serial, 24000000);
	if (r) {
		pr_info("%s: f75115_pwm_clock_set failed: %d", __func__, r);
		return r;
	}

	for (i = 0; i < F75115_MAX_PWN_CNT; ++i) {
		r = f75115_pwm_en_param_set(serial, i, 0, 0);
		if (r) {
			pr_info("%s: idx: %d, f75115_pwm_en_param_set failed: %d",
				__func__, i, r);
			return r;
		}
	}
#endif
	return 0;
}

static int f75115_pwm_clock_set(struct usb_serial *serial, int pwm_freq)
{
	struct f75115_serial_private *serial_priv;
	u8 buf[32], pwmc;
	int r, pwm_idx;

	serial_priv = usb_get_serial_data(serial);

	for (pwm_idx = 0; pwm_idx < ARRAY_SIZE(m_pwm_freq); ++pwm_idx) {
		if (m_pwm_freq[pwm_idx] == pwm_freq)
			break;
	}

	if (pwm_idx >= ARRAY_SIZE(m_pwm_freq))
		return -EINVAL;

	r = f75115_get_normal_register_size(serial->dev, 0xe001, buf, 11);
	if (r) {
		dev_warn(&serial->dev->dev, "%s: read 0xe001 failed: %d\n",
			 __func__, r);
		return r;
	}

	pwmc = buf[0] & ~(0xe0);
	pwmc |= pwm_idx << 5;
	buf[0] = pwmc;

	//dev_info(&serial->dev->dev, "%s: pwm_idx: %d, pwmc: %xh", __func__, pwm_idx, pwmc);

	r = f75115_set_normal_register_size(serial->dev, 0xe001, buf, 11);
	if (r) {
		dev_warn(&serial->dev->dev, "%s: write 0xe001 failed: %d\n",
			 __func__, r);
		return r;
	}

	serial_priv->pwm_cur_clock = pwm_freq;
	//dev_info(&serial->dev->dev, "%s: pwm_freq: %d", __func__, pwm_freq);

	return 0;
}

static int f75115_pwm_div_set(struct usb_serial *serial, int pwm_div)
{
	struct f75115_serial_private *serial_priv;
	u8 buf[32];
	int r;

	serial_priv = usb_get_serial_data(serial);

	if (pwm_div < 2 || pwm_div > 1023)
		return -EINVAL;

	r = f75115_get_normal_register_size(serial->dev, 0xe001, buf, 11);
	if (r) {
		dev_warn(&serial->dev->dev, "%s: read 0xe001 failed: %d\n",
			 __func__, r);
		return r;
	}

	buf[9] = (pwm_div >> 8) & 0xff;
	buf[10] = (pwm_div >> 0) & 0xff;

	//dev_info(&serial->dev->dev, "%s: pwm_div: %xh", __func__, pwm_div);

	r = f75115_set_normal_register_size(serial->dev, 0xe001, buf, 11);
	if (r) {
		dev_warn(&serial->dev->dev, "%s: write 0xe001 failed: %d\n",
			 __func__, r);
		return r;
	}

	serial_priv->pwm_cur_div = pwm_div;
	dev_info(&serial->dev->dev, "%s: pwm_cur_div: %d", __func__,
		 serial_priv->pwm_cur_div);

	return 0;
}

static int f75115_pwm_en_param_set(struct usb_serial *serial, int idx, int en,
				   u64 duty_us)
{
	struct f75115_serial_private *serial_priv;
	int r, div;
	u8 buf[32];
	u64 cur_period;

	serial_priv = usb_get_serial_data(serial);
	cur_period = 1000000000UL * (serial_priv->pwm_cur_div) /
		     serial_priv->pwm_cur_clock;

	if (idx < 0 || idx >= F75115_MAX_PWN_CNT) {
		dev_err(&serial->dev->dev, "%s: %d", __func__, __LINE__);
		return -EINVAL;
	}

	if (cur_period < duty_us || !cur_period) {
		dev_err(&serial->dev->dev, "%s: line: %d", __func__, __LINE__);
		dev_err(&serial->dev->dev, "%s: duty_us: %d", __func__,
			(int)duty_us);
		dev_err(&serial->dev->dev, "%s: cur_period: %d", __func__,
			(int)cur_period);
		dev_err(&serial->dev->dev, "%s: pwm_cur_clock: %d", __func__,
			(int)serial_priv->pwm_cur_clock);
		dev_err(&serial->dev->dev, "%s: pwm_cur_div: %d", __func__,
			(int)serial_priv->pwm_cur_div);
		return -EINVAL;
	}

	r = f75115_get_normal_register_size(serial->dev, 0xe001, buf, 11);
	if (r) {
		dev_warn(&serial->dev->dev, "%s: read 0xe001 failed: %d\n",
			 __func__, r);
		return r;
	}

#if 1
	if (en)
		buf[0] |= BIT(idx);
	else
		buf[0] &= ~BIT(idx);

	div = (serial_priv->pwm_cur_div) * duty_us / cur_period;
	if (div < 0 || div > 1023) {
		dev_warn(&serial->dev->dev, "div: %d\n", div);
		return -EINVAL;
	}

	buf[2 * idx + 1] = (div >> 8) & 0xff;
	buf[2 * idx + 2] = (div >> 0) & 0xff;
#else
	buf[0] = 2;
	buf[1] = 0;
	buf[2] = 0;
	buf[3] = 0;
	buf[4] = 5;
	buf[5] = 0;
	buf[6] = 0;
	buf[7] = 0;
	buf[8] = 0;
	buf[9] = 0;
	buf[10] = 9;
#endif

	r = f75115_set_normal_register_size(serial->dev, 0xe001, buf, 11);
	if (r) {
		dev_warn(&serial->dev->dev, "%s: write 0xe001 failed: %d\n",
			 __func__, r);
		return r;
	}

	//dev_info(&serial->dev->dev, "%s: duty_us: %lld\n", __func__, duty_us);
	//dev_info(&serial->dev->dev, "%s: pwm_cur_period: %lld\n", __func__, serial_priv->pwm_cur_period);
	//dev_info(&serial->dev->dev, "%s: rate: %lld/1000\n", __func__, duty_us * 1000ULL / serial_priv->pwm_cur_period);

	serial_priv->pwm_cur_duty_us[idx] = duty_us;
	serial_priv->pwm_cur_en[idx] = en;

	return 0;
}

static ssize_t pwm_freq_show(struct device *dev, struct device_attribute *attr,
			     char *buf)
{
	struct f75115_serial_private *serial_priv;
	struct usb_interface *intf;
	struct usb_serial *serial;

	intf = to_usb_interface(dev);
	serial = usb_get_intfdata(intf);
	serial_priv = usb_get_serial_data(serial);

	return sprintf(buf, "%d\n", serial_priv->pwm_cur_clock);
}

static ssize_t pwm_freq_store(struct device *dev,
			      struct device_attribute *attr, const char *buf,
			      size_t count)
{
	struct f75115_serial_private *serial_priv;
	struct usb_interface *intf;
	struct usb_serial *serial;
	long freq;
	int i, r;

	intf = to_usb_interface(dev);
	serial = usb_get_intfdata(intf);
	serial_priv = usb_get_serial_data(serial);

	mutex_lock(&serial_priv->pwm_mutex);

	r = kstrtol(buf, 0, &freq);
	if (r)
		goto end;

	r = -EINVAL;

	for (i = 0; i < ARRAY_SIZE(m_pwm_freq); ++i) {
		if (m_pwm_freq[i] == freq) {
			r = f75115_pwm_clock_set(serial, freq);
			if (r)
				break;

			r = count;
			break;
		}
	}

end:
	mutex_unlock(&serial_priv->pwm_mutex);
	return r;
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 11, 0)
static DEVICE_ATTR_RW(pwm_freq);
#else
static DEVICE_ATTR(pwm_freq, S_IRUGO | S_IWUSR, pwm_freq_show, pwm_freq_store);
#endif

static ssize_t pwm_div_show(struct device *dev, struct device_attribute *attr,
			    char *buf)
{
	struct f75115_serial_private *serial_priv;
	struct usb_interface *intf;
	struct usb_serial *serial;

	intf = to_usb_interface(dev);
	serial = usb_get_intfdata(intf);
	serial_priv = usb_get_serial_data(serial);

	return sprintf(buf, "%d\n", serial_priv->pwm_cur_div);
}

static ssize_t pwm_div_store(struct device *dev, struct device_attribute *attr,
			     const char *buf, size_t count)
{
	struct f75115_serial_private *serial_priv;
	struct usb_interface *intf;
	struct usb_serial *serial;
	long div;
	int r;

	intf = to_usb_interface(dev);
	serial = usb_get_intfdata(intf);
	serial_priv = usb_get_serial_data(serial);

	mutex_lock(&serial_priv->pwm_mutex);

	r = kstrtol(buf, 0, &div);
	if (r)
		goto end;

	r = f75115_pwm_div_set(serial, div);
	if (r < 0)
		goto end;

	r = count;
end:
	mutex_unlock(&serial_priv->pwm_mutex);
	return r;
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 11, 0)
static DEVICE_ATTR_RW(pwm_div);
#else
static DEVICE_ATTR(pwm_div, S_IRUGO | S_IWUSR, pwm_div_show, pwm_div_store);
#endif

static int f75115_pwm_init(struct usb_serial *serial)
{
	struct f75115_serial_private *serial_priv;
	u8 tmp;
	int r;

#ifndef PWM_MODE
	return 0;
#endif

	serial_priv = usb_get_serial_data(serial);

	if (serial_priv->fw_ver <= 0xaa88) {
		dev_info(&serial->dev->dev,
			 "%s: the version(%lx) not support PWM", __func__,
			 serial_priv->fw_ver);

		return 0;
	}

	f75115_pwm_reload(serial);

	/* change pwm from p2 to p4 */
	tmp = 0x01;
	r = f75115_set_normal_register_size(serial->dev, 0xe002, &tmp, 1);
	if (r) {
		pr_info("%s: write 0xe002 failed: %d\n", __func__, r);
		return r;
	}

	serial_priv->f75115_pwm_chip.dev =
		&serial->interface->dev; //&serial->dev->dev;
	serial_priv->f75115_pwm_chip.ops = &pwm_f75115_ops;
	serial_priv->f75115_pwm_chip.npwm = F75115_MAX_PWN_CNT;
	serial_priv->f75115_pwm_chip.base = -1;

	r = device_create_file(&serial->interface->dev, &dev_attr_pwm_freq);
	if (r)
		return -EPERM;

	r = device_create_file(&serial->interface->dev, &dev_attr_pwm_div);
	if (r) {
		device_remove_file(&serial->interface->dev,
				   &dev_attr_pwm_freq);
		return -EPERM;
	}

	//pr_info("%s: dev_attr_pwm_freq created", __func__);

	r = pwmchip_add(&serial_priv->f75115_pwm_chip);
	if (r) {
		dev_err(&serial->dev->dev, "failed to add PWM chip: %d\n", r);
		return r;
	}

	return 0;
}

static void f75115_pwm_deinit(struct usb_serial *serial)
{
	struct f75115_serial_private *serial_priv;

#ifndef PWM_MODE
	return;
#endif

	serial_priv = usb_get_serial_data(serial);

	if (serial_priv->fw_ver <= 0xaa88)
		return;

	pwmchip_remove(&serial_priv->f75115_pwm_chip);
	//device_remove_file(&serial->dev->dev, &dev_attr_pwm_freq);
	device_remove_file(&serial->interface->dev, &dev_attr_pwm_freq);
	device_remove_file(&serial->interface->dev, &dev_attr_pwm_div);

	pr_info("%s: dev_attr_pwm_freq removed", __func__);
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 12, 0)
static int f75115_calc_num_ports(struct usb_serial *serial,
				 struct usb_serial_endpoints *epds)
#else
static int f75115_calc_num_ports(struct usb_serial *serial)
#endif
{
	uintptr_t setting_idx;
	int i;
	u8 num_port = 0;
	int status;
	unsigned char setting[F75115_CUSTOM_DATA_SIZE + 1];

	/* check had custom setting */
	status = f75115_find_config_idx(serial, &setting_idx);
	if (status) {
		dev_err(&serial->dev->dev,
			"%s: f75115_find_config_idx read failed!!\n",
			__func__);
		return 0;
	}

	/* Save the configuration area idx as private data for attach() */
	usb_set_serial_data(serial, (void *)setting_idx);

	/*
	 * if had custom setting, override it.
	 * 1st byte is a indicator, 0xff is empty, F75115_CUSTOM_VALID_TOKEN
	 * is had data, then skip with 1st data
	 */
	if (setting_idx != F75115_CUSTOM_NO_CUSTOM_DATA) {
		status = f75115_read_data(
			serial,
			F75115_CUSTOM_ADDRESS_START +
				F75115_CUSTOM_DATA_SIZE * setting_idx + 1,
			sizeof(setting), setting);
		if (status) {
			dev_err(&serial->dev->dev,
				"%s: get custom data failed!!\n", __func__);
			return 0;
		}

		dev_info(&serial->dev->dev,
			 "%s: read configure from block:%d\n", __func__,
			 (int)setting_idx);
	} else {
#if 0	
		reset = true;
#endif
		dev_info(&serial->dev->dev, "%s: read configure default\n",
			 __func__);
	}

#if 0	
	if (reset) {
		pr_info("reset\n");
		/* read default board setting */
		status = f75115_read_data(serial, F75115_DEF_CONF_ADDRESS_START,
					  F75115_CUSTOM_DATA_SIZE, setting);
		if (status) {
			dev_err(&serial->dev->dev,
					"%s: f75115_read_data read failed!!\n",
					__func__);
			return 0;
		}
	}
#endif

	/* new style, find all possible ports */
	for (i = 0; i < F75115_NUM_PORT; ++i) {
		if (setting[i] & F75115_PORT_UNAVAILABLE)
			continue;

		++num_port;
	}

	if (num_port)
		return num_port;

	dev_err(&serial->dev->dev, "Read Failed!!, default 4 ports\n");
	return 4; /* nothing found, oldest version IC */
}

static void f75115_set_termios(struct tty_struct *tty,
			       struct usb_serial_port *port,
			       struct ktermios *old_termios)
{
	struct usb_device *dev = port->serial->dev;
	struct f75115_port_private *port_priv;
	u32 baud, old_baud = 0;
	u16 new_lcr = 0;
	int status;

	port_priv = usb_get_serial_port_data(port);

	if (C_BAUD(tty) == B0)
		f75115_update_mctrl(port, 0, TIOCM_DTR | TIOCM_RTS);
	else if (old_termios && (old_termios->c_cflag & CBAUD) == B0)
		f75115_update_mctrl(port, TIOCM_DTR | TIOCM_RTS, 0);

	if (C_PARENB(tty)) {
		new_lcr |= UART_LCR_PARITY;

		if (!C_PARODD(tty))
			new_lcr |= UART_LCR_EPAR;

		if (C_CMSPAR(tty))
			new_lcr |= UART_LCR_SPAR;
	}

	if (C_CSTOPB(tty))
		new_lcr |= UART_LCR_STOP;

	switch (C_CSIZE(tty)) {
	case CS5:
		new_lcr |= UART_LCR_WLEN5;
		break;
	case CS6:
		new_lcr |= UART_LCR_WLEN6;
		break;
	case CS7:
		new_lcr |= UART_LCR_WLEN7;
		break;
	default:
	case CS8:
		new_lcr |= UART_LCR_WLEN8;
		break;
	}

	if (old_termios)
		old_baud = old_termios->c_ospeed;

	baud = tty_get_baud_rate(tty);
	if (baud) {
		tty_encode_baud_rate(tty, baud, baud);
		port_priv->current_baud_rate = baud;
	}

	port_priv->shadow_lcr = new_lcr;

	if (old_baud == baud)
		baud = 0;

	status = f75115_set_port_config(dev, port_priv->phy, port, baud,
					new_lcr);
	if (status < 0)
		dev_err(&port->dev, "%s - f75115_set_port_config failed: %i\n",
			__func__, status);

	/* Re-Enable writer for to check H/W flow Control */
	status = f75115_submit_writer(port, GFP_KERNEL);
	if (status)
		dev_err(&port->dev, "%s: submit failed\n", __func__);
}

static int f75115_init_uart(struct usb_serial_port *port)
{
	struct f75115_port_private *port_priv;
	struct f75115_serial_private *serial_priv;
	int status;

	port_priv = usb_get_serial_port_data(port);
	serial_priv = usb_get_serial_data(port->serial);

	status = f75115_switch_gpio_mode(port, F75115_PIN_SET_DEFAULT);
	if (status) {
		dev_err(&port->dev,
			"%s: switch gpio mode failed!! status:%d\n", __func__,
			status);
		return status;
	}

	status = f75115_setregister(
		port->serial->dev, port_priv->phy, INTERRUPT_ENABLE_REGISTER,
		UART_IER_MSI | UART_IER_THRI | UART_IER_RDI);

	return status;
}

static int f75115_open(struct tty_struct *tty, struct usb_serial_port *port)
{
	struct f75115_serial_private *serial_priv =
		usb_get_serial_data(port->serial);
	struct f75115_port_private *port_priv = usb_get_serial_port_data(port);
	int phy = port_priv->phy;
	int status;

	status = f75115_init_uart(port);
	if (status)
		return status;

	if (tty)
		f75115_set_termios(tty, port, NULL);

#if 0
	status = f75115_pwm_div_set(port->serial, 1023);
	if (status) {
		pr_info("%s: f75115_pwm_div_set failed: %d", __func__, status);
		return status;
	}

	status = f75115_pwm_clock_set(port->serial, 24000000);
	if (status) {
		pr_info("%s: f75115_pwm_clock_set failed: %d", __func__, status);
		return status;
	}

	//status = f75115_pwm_en_param_set(port->serial, 1, 1, 400000);
	status = f75115_pwm_en_param_set(port->serial, 1, 1, 20000);
	if (status) {
		pr_info("%s: f75115_pwm_en_param_set failed: %d", __func__, status);
		return status;
	}
#endif

#if 0
	// pwm1 test pattern
	int r, i;
	u8 buf[32];

	buf[0] = 0x01;
	r = f75115_set_normal_register_size(port->serial->dev, 0xe002, buf, 1);
	if (r) {
		pr_info("%s: write 0xe002 failed: %d\n", __func__, r);
		return r;
	}	

	buf[0] = 0x0e;//0x02;
	buf[1] = 0x00;
	buf[2] = 0x00;
	buf[3] = 0x00;
	buf[4] = 0x02;
	buf[5] = 0x00;
	buf[6] = 0x08;
	buf[7] = 0x00;
	buf[8] = 0x0c;
	buf[9] = 0x00;
	buf[10] = 0x18;
	
	r = f75115_set_normal_register_size(port->serial->dev, 0xe001, buf, 11);
	if (r) {
		pr_info("%s: write 0xe002 failed: %d\n", __func__, r);
		return r;
	}	



	r = f75115_get_normal_register_size(port->serial->dev, 0xe001, buf, 11);
	if (r) {
		pr_info("%s: read 0xe001 failed: %d\n", __func__, r);
		return r;
	}

	pr_info("%s: read 0xe001 ok\n", __func__);
	
	for (i = 0; i < 11; ++i)
		pr_info("%s: i: %d, data: %x\n", __func__, i, buf[i]);

	r = f75115_get_normal_register_size(port->serial->dev, 0xe002, buf, 1);
	if (r) {
		pr_info("%s: read 0xe002 failed: %d\n", __func__, r);
		return r;
	}	

	pr_info("%s: read 0xe002 ok\n", __func__);

	for (i = 0; i < 1; ++i)
		pr_info("%s: i: %d, data: %x\r\n", __func__, i, buf[i]);

#endif

	atomic_inc(&serial_priv->port_active[phy]);
	return 0;
}

static void f75115_close(struct usb_serial_port *port)
{
	int i;
	unsigned long flags;
	struct f75115_serial_private *serial_priv =
		usb_get_serial_data(port->serial);
	struct f75115_port_private *port_priv = usb_get_serial_port_data(port);
	int phy = port_priv->phy;

	atomic_dec(&serial_priv->port_active[phy]);

#ifdef WRITER_WQ
	cancel_work_sync(&port_priv->writer_work);
	flush_scheduled_work();
#endif

	/* referenced from usb_serial_generic_close() */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 35)
	for (i = 0; i < ARRAY_SIZE(port->write_urbs); ++i)
		usb_kill_urb(port->write_urbs[i]);
#else
	usb_kill_urb(port->write_urb);
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 33)
	spin_lock_irqsave(&port->lock, flags);
	kfifo_reset_out(&port->write_fifo);
	spin_unlock_irqrestore(&port->lock, flags);
#endif

	dev_dbg(&port->dev, "%s\n", __func__);

#ifdef DEBUG_TX_EMPTY
	/* check tx empty */
	i = 50;

	while (--i) {
		if (!serial_priv->is_phy_port_not_empty[phy])
			break;

		if (schedule_timeout_killable(msecs_to_jiffies(10))) {
			dev_info(&port->dev, "%s: breaked !!\n", __func__);
			break;
		}
	}

	if (i == 0) {
		dev_warn(&port->dev, "%s: force clear tx_empty\n", __func__);
		serial_priv->is_phy_port_not_empty[phy] = false;
	}
#endif
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 3, 0)
static void f75115_remove_urbs(struct usb_serial *serial)
{
	int i, j;
	struct usb_serial_port *port = NULL;

	for (i = 1; i < serial->num_ports; ++i) {
		port = serial->port[i];

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 35)
		for (j = 0; j < ARRAY_SIZE(port->write_urbs); ++j) {
			usb_kill_urb(port->write_urbs[j]);
			usb_free_urb(port->write_urbs[j]);
			kfree(port->bulk_out_buffers[j]);
			clear_bit(j, &port->write_urbs_free);

			port->write_urbs[j] = NULL;
			port->bulk_out_buffers[j] = NULL;
		}
#else
		usb_kill_urb(port->write_urb);
		usb_free_urb(port->write_urb);
		kfree(port->bulk_out_buffer);

		if (!IS_ERR(port->write_fifo) && port->write_fifo) {
			kfifo_free(port->write_fifo);
			port->write_fifo = NULL;
		}
#endif
		port->write_urb = NULL;
		port->bulk_out_buffer = NULL;
		port->bulk_out_size = 0;
		port->bulk_out_endpointAddress = 0;
	}
}
#endif

static void f75115_disconnect(struct usb_serial *serial)
{
	struct f75115_serial_private *serial_priv =
		usb_get_serial_data(serial);

	if (en_i2c)
		f75115_i2c_deinit(serial, serial_priv->f75115_gpio_chip.base);

	if (en_spi)
		f75115_spi_deinit(serial);

	f75115_pwm_deinit(serial);

	f75115_release_gpio(serial);

#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 3, 0)
	f75115_remove_urbs(serial);
#endif
}

static void f75115_release(struct usb_serial *serial)
{
	struct f75115_serial_private *serial_priv =
		usb_get_serial_data(serial);

	kfree(serial_priv);

	device_set_wakeup_enable(&serial->dev->dev, false);
}

static int f75115_get_serial_info(struct usb_serial_port *port,
				  struct serial_struct __user *retinfo)
{
	struct serial_struct tmp;
	struct f75115_port_private *port_priv;

	port_priv = usb_get_serial_port_data(port);
	if (!port_priv)
		return -EFAULT;

	if (!retinfo)
		return -EFAULT;

	memset(&tmp, 0, sizeof(tmp));

	tmp.type = PORT_16550A;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 10, 0)
	tmp.port = port->port_number;
	tmp.line = port->minor;
#else
	tmp.port = port->number - port->serial->minor;
	tmp.line = port->number;
#endif

	tmp.baud_base = port_priv->current_baud_base;

	if (copy_to_user(retinfo, &tmp, sizeof(*retinfo)))
		return -EFAULT;

	return 0;
}

#ifdef MULTIDROP_ENABLE
static int f75115_mask_setregister(struct usb_device *dev, u8 uart, u16 reg,
				   u8 mask, u8 data)
{
	int status;
	u8 tmp;

	status = f75115_getregister(dev, uart, reg, &tmp);
	if (status)
		return status;

	tmp &= ~mask;
	tmp |= (mask & data);

	status = f75115_setregister(dev, uart, reg, tmp);
	if (status)
		return status;

	return 0;
}
#endif

#ifdef MULTIDROP_ENABLE
static int f75115_set_9Bits(struct usb_serial_port *port, int __user *arg)
{
	struct usb_device *dev = port->serial->dev;
	struct f75115_port_private *port_priv = usb_get_serial_port_data(port);
	int status;
	int port_num = port_priv->phy;
	int mode = 0;

	if (copy_from_user(&mode, (int *)arg, sizeof(int)))
		return -EFAULT;

	if (mode & (1L << 17)) {
		port_priv->mode_9bit = true;
		port_priv->addr_9bit = (mode >> 0) & 0xFF;
		port_priv->bitmask_9bit = (mode >> 8) & 0xFF;
	} else {
		port_priv->mode_9bit = port_priv->bitmask_9bit =
			port_priv->addr_9bit = 0;
	}

	/* bit4 9bit mode */
	status = f75115_mask_setregister(dev, port_num, CONFIG1_REGISTER,
					 (1 << 4), port_priv->mode_9bit << 4);
	if (status) {
		dev_err(&port->dev, "%s: failed, %d\n", __func__, __LINE__);
		return status;
	}

	/* bit6 auto addr */
	status = f75115_mask_setregister(dev, port_num, CLK_SEL_REGISTER,
					 (1 << 6), port_priv->mode_9bit << 6);
	if (status) {
		dev_err(&port->dev, "%s: failed, %d\n", __func__, __LINE__);
		return status;
	}

	/* saddress */
	status = f75115_setregister(dev, port_num, SADDRESS_REGISTER,
				    port_priv->addr_9bit);
	if (status) {
		dev_err(&port->dev, "%s: failed, %d\n", __func__, __LINE__);
		return status;
	}

	/* saden */
	status = f75115_setregister(dev, port_num, SADEN_REGISTER,
				    port_priv->bitmask_9bit);
	if (status) {
		dev_err(&port->dev, "%s: failed, %d\n", __func__, __LINE__);
		return status;
	}

	/* bit4 sm2 */
	status = f75115_mask_setregister(dev, port_num,
					 INTERRUPT_ENABLE_REGISTER, (1 << 4),
					 port_priv->mode_9bit << 4);
	if (status) {
		dev_err(&port->dev, "%s: failed, %d\n", __func__, __LINE__);
		return status;
	}

	return 0;
}

static int f75115_get_9Bits(struct usb_serial_port *port, int __user *arg)
{
	struct f75115_port_private *port_priv = usb_get_serial_port_data(port);

	int data = 0;

	if (port_priv->mode_9bit != false)
		data = (port_priv->bitmask_9bit << 8) | (port_priv->addr_9bit);

	if (copy_to_user((int __user *)arg, &data, sizeof(int)))
		return -EFAULT;

	return 0;
}

static int f75115_get_sm2(struct usb_serial_port *port, int __user *arg)
{
	struct usb_device *dev = port->serial->dev;
	struct f75115_port_private *port_priv = usb_get_serial_port_data(port);
	unsigned int data = 0;
	u8 tmp = 0;
	int status;
	int port_num = port_priv->phy;

	status = f75115_getregister(dev, port_num, INTERRUPT_ENABLE_REGISTER,
				    &tmp);
	if (status) {
		dev_err(&port->dev, "%s: failed, %d\n", __func__, __LINE__);
		return status;
	}

	data = (tmp & 0x10) ? 1 : 0;

	if (copy_to_user((int __user *)arg, &data, sizeof(int)))
		return -EFAULT;

	return 0;
}
#endif

static int f75115_set_port_mode(struct usb_serial_port *port,
				enum uart_mode eMode)
{
	int status;
	u8 tmp;
	struct f75115_port_private *port_priv = usb_get_serial_port_data(port);

	if (eMode > uart_mode_invalid)
		return -EINVAL;

	if (eMode != uart_mode_invalid) {
		status = f75115_getregister(port->serial->dev, port_priv->phy,
					    CLK_SEL_REGISTER, &tmp);
		if (status)
			return status;

		tmp &= ~(F75115_RS485_MODE | F75115_RS485_INVERT);

#if 0
		switch (port_priv->port_pin_data.force_uart_mode) {
		case uart_mode_rs232:
		case uart_mode_shutdown:
		case uart_mode_rs232_coexist:
			break;

		case uart_mode_rs485:
			tmp |= (F75115_RS485_MODE | F75115_RS485_INVERT);
			dev_dbg(&port->dev, "%s: uart_mode_rs485 URB:%x\n",
					__func__, tmp);
			break;

		default:
			tmp |= F75115_RS485_MODE;
			dev_dbg(&port->dev, "%s others URB:%x\n",
					__func__, tmp);
			break;

		}
#endif

		status = f75115_setregister(port->serial->dev, port_priv->phy,
					    CLK_SEL_REGISTER, tmp);
		if (status)
			return status;
	}

	return 0;
}

#ifdef FLASH_RW_DRIVER
static int f75115_get_configure_data(struct usb_serial_port *port,
				     struct internal_data __user *arg)
{
	struct usb_serial *serial = port->serial;
	struct internal_data data;
	int nRet = 0;
	unsigned int max_block = F75115_MAX_DATA_BLOCK;

	memset(&data, 0, sizeof(data));

	if (copy_from_user(&data, (struct internal_data __user *)arg,
			   sizeof(data)))
		return -EFAULT;

	data.size = min(data.size, max_block);

	nRet = f75115_read_data(serial, data.address, data.size, data.buf);
	if (nRet)
		return nRet;

	if (copy_to_user((struct internal_data __user *)arg, &data,
			 sizeof(data)))
		return -EFAULT;

	return 0;
}

static int f75115_set_configure_data(struct usb_serial_port *port,
				     struct internal_data __user *arg)
{
	struct usb_serial *serial = port->serial;
	struct internal_data data;
	int nRet = 0;
	unsigned int max_block = F75115_MAX_DATA_BLOCK;

	memset(&data, 0, sizeof(data));

	if (copy_from_user(&data, (struct internal_data __user *)arg,
			   sizeof(data)))
		return -EFAULT;

	data.size = min(data.size, max_block);

	nRet = f75115_write_data(serial, data.address, data.size, data.buf);
	if (nRet)
		return nRet;

	if (copy_to_user((struct internal_data __user *)arg, &data,
			 sizeof(data)))
		return -EFAULT;

	return 0;
}

static int f75115_erase_configure_data(struct usb_serial_port *port,
				       struct internal_data __user *arg)
{
	struct usb_serial *serial = port->serial;
	struct internal_data data;
	int nRet = 0;

	memset(&data, 0, sizeof(data));

	if (copy_from_user(&data, (struct internal_data __user *)arg,
			   sizeof(data)))
		return -EFAULT;

	nRet = f75115_erase_sector(serial, data.address);
	if (nRet)
		return nRet;

	return 0;
}
#endif

static int f75115_ioctl_set_gpio(struct usb_serial_port *port,
				 struct gpio_access __user *arg)
{
	struct usb_serial *serial;
	struct f75115_serial_private *serial_priv;
	struct gpio_access data;
	int status;

	serial = port->serial;
	serial_priv = usb_get_serial_data(serial);

	memset(&data, 0, sizeof(data));

	if (copy_from_user(&data, (struct gpio_access __user *)arg,
			   sizeof(data)))
		return -EFAULT;

	if (data.set != 0 && data.set != 3 && data.set != 5)
		return -EINVAL;

	mutex_lock(&serial_priv->change_mode_mutex);

#ifdef GPIO_OUTPUT_FAST_MODE
	serial_priv->gpio_data[data.set] = data.data;
#endif

	if (data.set == 5) {
		status = f75115_set_normal_register(serial->dev, 0x1630,
						    data.data);
	} else {
		status = f75115_set_normal_register(
			serial->dev, pin_data_table[data.set], data.data);
	}

	if (status) {
		dev_err(&serial->interface->dev,
			"%s: ioctl write data failed\n", __func__);
	} else {
		serial_priv->gpio_data[data.set] = data.data;
	}

	mutex_unlock(&serial_priv->change_mode_mutex);

	return status;
}

static int f75115_ioctl_get_gpio(struct usb_serial_port *port,
				 struct gpio_access __user *arg)
{
	struct usb_serial *serial;
	struct f75115_serial_private *serial_priv;
	struct gpio_access data;
	int status;

	serial = port->serial;
	serial_priv = usb_get_serial_data(serial);

	memset(&data, 0, sizeof(data));

	if (copy_from_user(&data, (struct gpio_access __user *)arg,
			   sizeof(data)))
		return -EFAULT;

	if (data.set != 0 && data.set != 3 && data.set != 5)
		return -EINVAL;

	mutex_lock(&serial_priv->change_mode_mutex);

	if (data.set == 5) {
		status = f75115_get_normal_register(serial->dev, 0x1650,
						    &data.data);
	} else {
		status = f75115_get_normal_register(
			serial->dev, pin_data_table[data.set], &data.data);
		if (!status)
			serial_priv->gpio_data[data.set] = data.data;
	}

	if (status) {
		dev_err(&serial->interface->dev,
			"%s: ioctl write data failed\n", __func__);
		goto end;
	}

#ifdef GPIO_OUTPUT_FAST_MODE
	serial_priv->gpio_data[data.set] = data.data;
#endif

	if (copy_to_user((struct gpio_access __user *)arg, &data,
			 sizeof(data)))
		status = -EFAULT;
end:
	mutex_unlock(&serial_priv->change_mode_mutex);

	return status;
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 0, 0)
static int f75115_ioctl(struct tty_struct *tty, unsigned int cmd,
			unsigned long arg)
#else
static int f75115_ioctl(struct tty_struct *tty, struct file *file,
			unsigned int cmd, unsigned long arg)
#endif
{
	struct usb_serial_port *port = tty->driver_data;

	switch (cmd) {
	case TIOCGSERIAL:
		return f75115_get_serial_info(
			port, (struct serial_struct __user *)arg);
#ifdef MULTIDROP_ENABLE
	case FINTEK_SET_MULTI_DROP_MODE:
		return f75115_set_9Bits(port, (int __user *)arg);
	case FINTEK_GET_MULTI_DROP_MODE:
		return f75115_get_9Bits(port, (int __user *)arg);
	case FINTEK_GET_SM2_STATE:
		return f75115_get_sm2(port, (int __user *)arg);
#endif
#ifdef FLASH_RW_DRIVER
	case FINTEK_ERASE_DATA_PAGE:
		return f75115_erase_configure_data(
			port, (struct internal_data __user *)arg);

	case FINTEK_GET_DATA:
		return f75115_get_configure_data(
			port, (struct internal_data __user *)arg);

	case FINTEK_SET_DATA:
		return f75115_set_configure_data(
			port, (struct internal_data __user *)arg);
#endif
	case FINTEK_GET_GPIO:
		return f75115_ioctl_get_gpio(port,
					     (struct gpio_access __user *)arg);
	case FINTEK_SET_GPIO:
		return f75115_ioctl_set_gpio(port,
					     (struct gpio_access __user *)arg);
	default:
		break;
	}

	return -ENOIOCTLCMD;
}

#ifdef WRITER_WQ
static void f75115_writer_worker(struct work_struct *work)
{
	struct f75115_port_private *port_priv =
		container_of(work, struct f75115_port_private, writer_work);
	struct usb_serial_port *port = port_priv->port;
	int status;

	status = f75115_submit_writer(port, GFP_KERNEL);
	if (status) {
		dev_err(&port->dev, "%s: submit failed\n", __func__);
	}
}
#endif

#ifdef LSR_ISSUE
static void f75115_lsr_worker(struct work_struct *work)
{
	struct f75115_port_private *port_priv =
		container_of(work, struct f75115_port_private, lsr_work);
	struct usb_serial_port *port = port_priv->port;
	int status;
	u8 tmp;

	status = f75115_getregister(port->serial->dev, port_priv->phy,
				    LINE_STATUS_REGISTER, &tmp);
	if (status) {
		dev_err(&port->dev, "%s: read failed: %x\n", __func__, status);
	}
}
#endif

static void f75115_compare_msr(struct usb_serial_port *port, u8 msr,
			       bool is_port_open)
{
	u8 old_msr;
	struct tty_struct *tty = NULL;
	struct f75115_port_private *port_priv = usb_get_serial_port_data(port);
	unsigned long flags;
#ifndef WRITER_WQ
	int status;
#endif

	if (!(msr & UART_MSR_ANY_DELTA))
		return;

	spin_lock_irqsave(&port_priv->msr_lock, flags);
	old_msr = port_priv->shadow_msr;
	port_priv->shadow_msr = msr;
	spin_unlock_irqrestore(&port_priv->msr_lock, flags);

	if ((msr & (UART_MSR_CTS | UART_MSR_DCTS)) ==
	    (UART_MSR_CTS | UART_MSR_DCTS)) {
		/* CTS changed, wakeup writer to re-check flow control */
		if (is_port_open) {
#ifdef WRITER_WQ
			schedule_work(&port_priv->writer_work);
#else
			status = f75115_submit_writer(port, GFP_ATOMIC);
			if (status) {
				dev_err(&port->dev, "%s: submit failed\n",
					__func__);
			}
#endif
		}
		dev_dbg(&port->dev, "%s: CTS Flag changed, value: %x\n",
			__func__, !!(msr & UART_MSR_CTS));
	}

	dev_dbg(&port->dev, "%s: MSR from %02x to %02x\n", __func__, old_msr,
		msr);

	if (!is_port_open)
		return;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 10, 0)
	/* update input line counters */
	if (msr & UART_MSR_DCTS)
		port->icount.cts++;
	if (msr & UART_MSR_DDSR)
		port->icount.dsr++;
	if (msr & UART_MSR_DDCD)
		port->icount.dcd++;
	if (msr & UART_MSR_TERI)
		port->icount.rng++;
#endif
	wake_up_interruptible(&port->port.delta_msr_wait);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 38)
	if (!(msr & UART_MSR_DDCD))
		return;

	dev_dbg(&port->dev, "%s: DCD Changed: port %d from %x to %x.\n",
		__func__, port_priv->phy, old_msr, msr);

	tty = tty_port_tty_get(&port->port);
	if (!tty)
		return;

	usb_serial_handle_dcd_change(port, tty, msr & UART_MSR_DCD);
	tty_kref_put(tty);
#endif
}

static void f75115_process_per_serial_block(struct usb_serial_port *port,
					    unsigned char *data)
{
	u8 lsr, lsr_total = 0;
	char tty_flag;
#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 9, 0)
	struct tty_struct *tty;
#endif
	struct usb_serial *serial = port->serial;
	struct f75115_serial_private *serial_priv =
		usb_get_serial_data(serial);
#ifdef LSR_ISSUE
	struct f75115_port_private *port_priv = usb_get_serial_port_data(port);
#endif
#ifdef WRITER_WQ
	struct f75115_port_private *port_priv = usb_get_serial_port_data(port);
#else
	int status;
#endif
	int phy_port_num = data[0];
	int read_size = 0;
	int i;
	unsigned long flags;
	bool available =
		!!atomic_read(&serial_priv->port_active[phy_port_num]);

	/*
	 * The block layout is 128 Bytes
	 * index 0: port phy idx (e.g., 0,1,2,3),
	 * index 1: It's could be
	 *			F75115_TOKEN_RECEIVE
	 *			F75115_TOKEN_TX_EMPTY
	 *			F75115_TOKEN_MSR_CHANGE
	 * index 2: serial in size (data+lsr, must be even)
	 *			meaningful for F75115_TOKEN_RECEIVE only
	 * index 3: current MSR with device read
	 * index 4~127: serial in data block (data+lsr, must be even)
	 */
	switch (data[1]) {
	case F75115_TOKEN_TX_EMPTY:
		/*
		 * We should record TX_EMPTY flag even the port is not opened
		 */
		spin_lock_irqsave(&serial_priv->tx_empty_lock, flags);
		serial_priv->is_phy_port_not_empty[phy_port_num] = false;
		spin_unlock_irqrestore(&serial_priv->tx_empty_lock, flags);
		usb_serial_port_softint(port);
		dev_dbg(&port->dev, "%s: F75115_TOKEN_TX_EMPTY\n", __func__);
		break;

	case F75115_TOKEN_MSR_CHANGE:
		/*
		 * We'll save MSR value when device reported even when port
		 * is not opened. If the port is not opened, the MSR will only
		 * recorded without any future process.
		 */
		f75115_compare_msr(port, data[3], available);
		dev_dbg(&port->dev, "%s: F75115_TOKEN_MSR_CHANGE\n", __func__);
		break;

	case F75115_TOKEN_RECEIVE:
		read_size = data[2];
		dev_dbg(&port->dev, "%s: F75115_TOKEN_RECEIVE read_size:%d\n",
			__func__, read_size);
		break;

	default:
		dev_warn(&port->dev, "%s: unknown token:%02x\n", __func__,
			 data[1]);
		return;
	}

	/* if the port not had open, dont do future process */
	if (!available)
		return;

	/* Wakeup writer workqueue only when port is opened */
	if (data[1] == F75115_TOKEN_TX_EMPTY) {
#ifdef WRITER_WQ
		schedule_work(&port_priv->writer_work);
#else
		status = f75115_submit_writer(port, GFP_ATOMIC);
		if (status)
			dev_err(&port->dev, "%s: submit failed\n", __func__);
#endif
	}

	if (data[1] != F75115_TOKEN_RECEIVE)
		return;

#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 9, 0)
	tty = tty_port_tty_get(&port->port);
	if (!tty)
		return;
#endif

	for (i = 4; i < 4 + read_size; i += 2) {
		tty_flag = TTY_NORMAL;
		lsr = data[i + 1];

		lsr_total |= lsr;

		if (lsr & UART_LSR_BRK_ERROR_BITS) {
			//dev_warn(&port->dev, "lsr : %x\n", lsr);

			if (lsr & UART_LSR_BI) {
				tty_flag = TTY_BREAK;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 10, 0)
				port->icount.brk++;
#endif
				usb_serial_handle_break(port);
			} else if (lsr & UART_LSR_PE) {
				tty_flag = TTY_PARITY;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 10, 0)
				port->icount.parity++;
#endif
			} else if (lsr & UART_LSR_FE) {
				tty_flag = TTY_FRAME;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 10, 0)
				port->icount.frame++;
#endif
			}

			if (lsr & UART_LSR_OE) {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 10, 0)
				port->icount.overrun++;
#endif
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 9, 0)
				tty_insert_flip_char(&port->port, 0,
						     TTY_OVERRUN);
#else
				tty_insert_flip_char(tty, 0, TTY_OVERRUN);
#endif
			}
		}

		if (
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 34)
			port->port.console &&
#endif
			port->sysrq) {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 36)
			if (usb_serial_handle_sysrq_char(port, data[i]))
#else
			if (usb_serial_handle_sysrq_char(tty, port, data[i]))
#endif
				continue;
		}
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 9, 0)
		tty_insert_flip_char(&port->port, data[i], tty_flag);
#else
		tty_insert_flip_char(tty, data[i], tty_flag);
#endif
	}

	if (lsr_total & UART_LSR_BRK_ERROR_BITS) {
		//if (phy_port_num != 3)
		dev_dbg(&port->dev, "lsr : %x\n", lsr_total);
#ifdef LSR_ISSUE
		schedule_work(&port_priv->lsr_work);
#endif
	}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 9, 0)
	tty_flip_buffer_push(&port->port);
#else
	tty_flip_buffer_push(tty);
	tty_kref_put(tty);
#endif
}

static void f75115_process_read_urb(struct urb *urb)
{
	int i;
	int phy_port_num;
	int tty_port_num;
	unsigned char *ch;
	struct usb_serial *serial;
	struct f75115_serial_private *serial_priv = NULL;
	struct usb_serial_port *port = NULL;
	struct f75115_port_private *port_priv = NULL;

	if (!urb->actual_length)
		return;

	port = urb->context;
	serial = port->serial;
	ch = urb->transfer_buffer;
	serial_priv = usb_get_serial_data(serial);

	for (i = 0; i < urb->actual_length; i += F75115_RECEIVE_BLOCK_SIZE) {
		phy_port_num = ch[i];
		if (phy_port_num >= F75115_NUM_PORT) {
			dev_err(&serial->dev->dev,
				"phy_port_num >= F75115_NUM_PORT: %d\n",
				phy_port_num);
			continue;
		}

		if (serial_priv->default_conf_data[phy_port_num] &
		    F75115_PORT_UNAVAILABLE) {
			dev_dbg(&serial->dev->dev,
				"phy_port_num: %d, skipped\n", phy_port_num);
			continue;
		}

		tty_port_num = f75115_phy_to_logic_port(serial, phy_port_num);
		port = serial->port[tty_port_num];

		/*
		 * The device will send back all information when we submitted
		 * a read URB (MSR/DATA/TX_EMPTY). But it maybe get callback
		 * before port_probe() or after port_remove().
		 *
		 * So we'll verify the pointer. If the pointer is NULL, it's
		 * mean the port not init complete and the block will skip.
		 */
		port_priv = usb_get_serial_port_data(port);
		if (!port_priv) {
			dev_dbg(&serial->dev->dev, "%s: phy: %d not ready!\n",
				__func__, phy_port_num);
			continue;
		}

		f75115_process_per_serial_block(port, &ch[i]);
	}
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 35)
	i = usb_submit_urb(urb, GFP_ATOMIC);
	if (i)
		dev_err(&port->dev, "%s: resubmitting read URB, failed %d\n",
			__func__, i);
#endif
}

static void f75115_write_usb_callback(struct urb *urb)
{
	struct usb_serial_port *port = urb->context;
	int status = urb->status;

	if (status) {
		dev_warn(&port->dev, "%s - non-zero URB status: %d\n",
			 __func__, status);
	} else {
		usb_serial_port_softint(port);
	}
}

static int f75115_setup_urbs(struct usb_serial *serial)
{
	int i = 0;
	u8 port0_out_address;
	int j;
	int buffer_size;
	struct usb_serial_port *port = NULL;

	/*
	 * In our system architecture, we had 4 or 2 serial ports,
	 * but only get 1 set of bulk in/out endpoints.
	 *
	 * The usb-serial subsystem will generate port 0 data,
	 * but port 1/2/3 will not. It's will generate write URB and buffer
	 * by following code
	 */
	for (i = 1; i < serial->num_ports; ++i) {
		port0_out_address = serial->port[0]->bulk_out_endpointAddress;
		buffer_size = serial->port[0]->bulk_out_size;
		port = serial->port[i];

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 35)
		if (kfifo_alloc(&port->write_fifo, PAGE_SIZE, GFP_KERNEL))
			goto failed;
#else
		port->write_fifo =
			kfifo_alloc(PAGE_SIZE, GFP_KERNEL, &port->lock);
		if (IS_ERR(port->write_fifo))
			goto failed;
#endif

		port->bulk_out_size = buffer_size;
		port->bulk_out_endpointAddress = port0_out_address;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 35)
		for (j = 0; j < ARRAY_SIZE(port->write_urbs); ++j) {
			set_bit(j, &port->write_urbs_free);

			port->write_urbs[j] = usb_alloc_urb(0, GFP_KERNEL);
			if (!port->write_urbs[j])
				goto failed;

			port->bulk_out_buffers[j] =
				kmalloc(buffer_size, GFP_KERNEL);
			if (!port->bulk_out_buffers[j])
				goto failed;

			usb_fill_bulk_urb(port->write_urbs[j], serial->dev,
					  usb_sndbulkpipe(serial->dev,
							  port0_out_address),
					  port->bulk_out_buffers[j],
					  buffer_size,
					  serial->type->write_bulk_callback,
					  port);
		}

		port->write_urb = port->write_urbs[0];
		port->bulk_out_buffer = port->bulk_out_buffers[0];
#else
		port->write_urb = usb_alloc_urb(0, GFP_KERNEL);
		if (!port->write_urb)
			goto failed;

		port->bulk_out_buffer = kmalloc(buffer_size, GFP_KERNEL);
		if (!port->bulk_out_buffer)
			goto failed;

		usb_fill_bulk_urb(port->write_urb, serial->dev,
				  usb_sndbulkpipe(serial->dev,
						  port0_out_address),
				  port->bulk_out_buffer, buffer_size,
				  serial->type->write_bulk_callback, port);
#endif
	}
	return 0;

failed:
	return -ENOMEM;
}

static int f75115_submit_read_urb(struct usb_serial *serial, gfp_t mem_flags)
{
	int status;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 3, 0)
	status = usb_serial_generic_submit_read_urbs(serial->port[0],
						     mem_flags);
#elif LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 35)
	status =
		usb_serial_generic_submit_read_urb(serial->port[0], mem_flags);
#else
	/* Continue reading from device */
	usb_fill_bulk_urb(
		serial->port[0]->read_urb, serial->dev,
		usb_rcvbulkpipe(serial->dev,
				serial->port[0]->bulk_in_endpointAddress),
		serial->port[0]->read_urb->transfer_buffer,
		serial->port[0]->read_urb->transfer_buffer_length,
		f75115_process_read_urb, serial->port[0]);
	status = usb_submit_urb(serial->port[0]->read_urb, mem_flags);
	if (status)
		dev_err(&serial->dev->dev,
			"%s: submitting read URB, fail %d\n", __func__,
			status);
#endif
	if (status) {
		dev_err(&serial->dev->dev,
			"%s: submit read URB failed!! status:%d!!\n", __func__,
			status);
		return status;
	}

	return 0;
}

#if 0
static int f75115_detect_port_count(struct usb_serial *serial)
{
	int status;
	u8 buf[4];

	status = f75115_read_data(serial, 0x3022, 1, buf);
	if (status)
		return status;

	return buf[0] - '0';
}

static int f75115_fixup_aa66(struct usb_serial *serial)
{
	int status;
	int addr;
	int port;
	u8 buf[16];
#if 0
	int i;
	bool flag = false;
	u8 config_port2[] = {0xf0, 0x00, 0x80, 0x80, 0x00, 0x07, 0x80, 0x80,
				0x07, 0x01, 0x01, 0x01, 0x01};
	u8 config_port4[] = {0xf0, 0x00, 0x00, 0x00, 0x00, 0x07, 0x07, 0x07,
				0x07, 0x01, 0x01, 0x01, 0x01};
	u8 *config;
#endif

	port = f75115_detect_port_count(serial);
	dev_info(&serial->dev->dev, "%s: %d port detected\n", __func__, port);

	switch (port) {
	case 2:
		addr = 0x0B5B;
		break;
	case 1:
	case 4:
		addr = 0x0C63;
		break;
	default:
		return -EINVAL;
	}

	status = f75115_read_data(serial, addr, 1, buf);
	if (status)
		return status;
	
	if (buf[0] != 0x00) {
		pr_info("%x\n", buf[0]);
		buf[0] = 0;
		dev_dbg(&serial->dev->dev, "%s: patch 1 Patching: %x\n", __func__,
				addr);

		status = f75115_write_data(serial, addr, 1, buf);
		if (status)
			return status;
	} else {
		dev_dbg(&serial->dev->dev, "%s: patch 1 Patched\n", __func__);
	}

	switch (port) {
	case 2:
		addr = 0x0AA4;
		break;
	case 1:
	case 4:
		addr = 0x0D26;
		break;
	default:
		return -EINVAL;
	}

	status = f75115_read_data(serial, addr, 1, buf);
	if (status)
		return status;
	
	if (buf[0] != 0x00) {
		dev_dbg(&serial->dev->dev, "%s: patch 2 Patching: %x %x\n", __func__,
				addr, buf[0]);

		buf[0] = 0;
		status = f75115_write_data(serial, addr, 1, buf);
		if (status)
			return status;
	} else {
		dev_dbg(&serial->dev->dev, "%s: patch 2 Patched\n", __func__);
	}

#if 0
	status = f75115_read_data(serial, F75115_CUSTOM_ADDRESS_START,
					sizeof(buf), buf);
	if (status)
		return status;

	for (i = 5; i < 9; ++i) {
		if (buf[i] != 7 && buf[i] != 0x80) {
			dev_info(&serial->dev->dev, "%s: %d need reset\n",
					__func__, i);
			flag = true;
			break;
		}
	}

	if (flag) {
		switch (port) {
		case 2: config = config_port2; break;
		case 4: config = config_port4; break;
		default:
			return -EINVAL;
		}

		status = f75115_erase_sector(serial,
						F75115_CUSTOM_ADDRESS_START);
		if (status)
			return status;

		status = f75115_write_data(serial, F75115_CUSTOM_ADDRESS_START,
						13, config);
		if (status)
			return status;
	}
#endif

	return 0;
}

static int f75115_fixup(struct usb_serial *serial)
{
	int status;
	int ver = 0;
	int i;
	u8 buf[4];

	/* Get H/W version */
	status = f75115_read_data(serial, 0x1ffc, 4, buf);
	if (status)
		return status;

	for (i = 0; i < 4; ++i)
		ver |= buf[i] << (3 - i) * 4;

	dev_info(&serial->dev->dev, "%s: H/W version: %x\n", __func__, ver);

	switch (ver) {
	case 0xaa66:
		return f75115_fixup_aa66(serial);
	default:
		dev_err(&serial->dev->dev, "%s: ver: %x dont need patch\n",
			__func__, ver);
	}

	return 0;
}
#endif

static int f75115_read_fw_ver(struct usb_serial *serial)
{
	struct f75115_serial_private *serial_priv;
	int i, r, len;
	u8 buf[5] = { 0 };

	serial_priv = usb_get_serial_data(serial);
	len = strlen(serial->dev->product);

	for (i = 0; i < 4; ++i)
		buf[i] = serial->dev->product[len - 4 + i];

	r = kstrtol(buf, 16, &serial_priv->fw_ver);
	//pr_info("%s: %lx\n", __func__, serial_priv->fw_ver);

	//dev_info(&serial->dev->dev, "%s: H/W version: %x\n", __func__, ver);

	return 0;
}

static void f75115_gpio_init_status(struct usb_serial *serial)
{
        struct f75115_serial_private *serial_priv =
                usb_get_serial_data(serial);
	int base = serial_priv->f75115_gpio_chip.base;

        dev_info(&serial->dev->dev, "%s: base=%d\n", __func__, base);
	gpio_request(base + F75115_GPIO52,"U2_E12_1_EN");
	gpio_request(base + F75115_GPIO16,"U2_E12_2_EN");
	gpio_request(base + F75115_GPIO30,"U2_E34_1_EN");
	gpio_request(base + F75115_GPIO32,"U2_E34_2_EN");
	gpio_request(base + F75115_GPIO34,"CC_SW_EN");
	gpio_request(base + F75115_GPIO36,"DIS_SW_EN");
	gpio_request(base + F75115_GPIO20,"USB_5V_EN");
	gpio_request(base + F75115_GPIO21,"S_U2H_RESET_N");
	gpio_request(base + F75115_GPIO24,"EX_DO1");
	gpio_request(base + F75115_GPIO07,"SSR_EN");
	gpio_request(base + F75115_GPIO03,"AMP_SDZ_N");
	gpio_request(base + F75115_GPIO01,"EX_DO2");
        gpio_request(base + F75115_GPIO10,"U2H_RESET_N");
	
	mdelay(150);

	gpio_direction_output(base + F75115_GPIO52, 1);
	gpio_direction_output(base + F75115_GPIO16, 1);
	gpio_direction_output(base + F75115_GPIO30, 1);
	gpio_direction_output(base + F75115_GPIO32, 1);
	gpio_direction_output(base + F75115_GPIO34, 1);
	gpio_direction_output(base + F75115_GPIO36, 1);
	gpio_direction_output(base + F75115_GPIO20, 1);
	gpio_direction_output(base + F75115_GPIO21, 1);
	gpio_direction_output(base + F75115_GPIO24, 1);
	gpio_direction_output(base + F75115_GPIO07, 1);
	gpio_direction_output(base + F75115_GPIO03, 1);
	gpio_direction_output(base + F75115_GPIO01, 1);
        gpio_direction_output(base + F75115_GPIO10, 1);

	gpio_export(base + F75115_GPIO52, 1);
	gpio_export(base + F75115_GPIO16, 1);
	gpio_export(base + F75115_GPIO30, 1);
	gpio_export(base + F75115_GPIO32, 1);
	gpio_export(base + F75115_GPIO34, 1);
	gpio_export(base + F75115_GPIO36, 1);
	gpio_export(base + F75115_GPIO20, 1);
	gpio_export(base + F75115_GPIO21, 1);
	gpio_export(base + F75115_GPIO24, 1);
	gpio_export(base + F75115_GPIO07, 1);
        gpio_export(base + F75115_GPIO03, 1);
	gpio_export(base + F75115_GPIO01, 1);
        gpio_export(base + F75115_GPIO10, 1);

}

static int f75115_attach(struct usb_serial *serial)
{
	struct f75115_serial_private *serial_priv = NULL;
	int status;
	int offset;
	int i;

	status = f75115_pin_init(serial);
	if (status)
		return status;

#if 0
	status = f75115_fixup(serial);
	if (status)
		return status;
#endif
	serial_priv = kzalloc(sizeof(*serial_priv), GFP_KERNEL);
	if (!serial_priv)
		return -ENOMEM;

	serial_priv->serial = serial;
	usb_set_serial_data(serial, serial_priv);

	f75115_read_fw_ver(serial);

	for (i = 0; i < F75115_NUM_PORT; ++i)
		atomic_set(&serial_priv->port_active[i], 0);

	spin_lock_init(&serial_priv->tx_empty_lock);
	mutex_init(&serial_priv->change_mode_mutex);
	mutex_init(&serial_priv->pwm_mutex);

	status = f75115_prepare_gpio(serial);
	if (status) {
		dev_err(&serial->dev->dev,
			"%s: f75115_prepare_gpio failed!! status:%d!!\n",
			__func__, status);

		goto failed;
	}

	offset = F75115_CUSTOM_ADDRESS_START + 1;
	/* only read 8 bytes for mode & GPIO */
	status = f75115_read_data(serial, offset,
				  sizeof(serial_priv->default_conf_data),
				  serial_priv->default_conf_data);

	status = f75115_setup_urbs(serial);
	if (status)
		goto failed;

	pr_info(KBUILD_MODNAME ": " DRIVER_DESC ": " VERSION "\n");

	/*
	 * We'll register port 0 bulkin only once, It'll take all port received
	 * data, MSR register change and TX_EMPTY information.
	 */
	status = f75115_submit_read_urb(serial, GFP_KERNEL);
	if (status) {
		dev_err(&serial->dev->dev,
			"%s: submit read URB failed!! status:%d!!\n", __func__,
			status);
		goto failed;
	}

	if (en_i2c) {
		status = f75115_i2c_init(serial,
					 serial_priv->f75115_gpio_chip.base);
		if (status) {
			f75115_release_gpio(serial);
			return -EINVAL;
		}
	}

	if (en_spi) {
		status = f75115_spi_init(serial,
					 serial_priv->f75115_gpio_chip.base);
		if (status) {
			f75115_release_gpio(serial);
			return -EINVAL;
		}
	}

	status = f75115_pwm_init(serial);
	if (status) {
		f75115_release_gpio(serial);
		return -EINVAL;
	}

	f75115_gpio_init_status(serial);

	device_set_wakeup_enable(&serial->dev->dev, true);
	return 0;

failed:
	kfree(serial_priv);
	return status;
}

#ifdef SAVE_CONTROL
static ssize_t no_save_show(struct device *dev, struct device_attribute *attr,
			    char *buf)
{
	struct usb_serial_port *port = to_usb_serial_port(dev);
	struct f75115_port_private *port_priv = usb_get_serial_port_data(port);

	return sprintf(buf, "%d\n", !!(port_priv->port_flag & PORT_NOSAVE));
}

static ssize_t no_save_store(struct device *dev, struct device_attribute *attr,
			     const char *buf, size_t count)
{
	struct usb_serial_port *port = to_usb_serial_port(dev);
	struct f75115_port_private *port_priv = usb_get_serial_port_data(port);

	if (!count)
		return -EINVAL;

	if (buf[0] != '0')
		port_priv->port_flag |= PORT_NOSAVE;
	else
		port_priv->port_flag &= ~PORT_NOSAVE;

	return count;
}
#endif

#ifdef DEBUG_INTERNAL_SYSFS
static ssize_t internal_show(struct device *dev, struct device_attribute *attr,
			     char *buf)
{
	struct usb_serial_port *port = to_usb_serial_port(dev);
	struct usb_serial *serial = port->serial;
	struct f75115_serial_private *serial_priv =
		usb_get_serial_data(serial);
	struct f75115_port_private *port_priv = usb_get_serial_port_data(port);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 33)
	int fifo_size = kfifo_len(&port->write_fifo);
#else
	int fifo_size = kfifo_len(port->write_fifo);
#endif

	return sprintf(buf, "is using(txempty):%d\nfifo size:%d\n",
		       serial_priv->is_phy_port_not_empty[port_priv->phy],
		       fifo_size);
}

static ssize_t internal_store(struct device *dev,
			      struct device_attribute *attr, const char *buf,
			      size_t count)
{
	struct usb_serial_port *port = to_usb_serial_port(dev);
	struct usb_serial *serial = port->serial;
	struct f75115_serial_private *serial_priv =
		usb_get_serial_data(serial);
	struct f75115_port_private *port_priv = usb_get_serial_port_data(port);

	if (!count)
		return -EINVAL;

	if (buf[0] == '0')
		serial_priv->is_phy_port_not_empty[port_priv->phy] = false;
	else
		serial_priv->is_phy_port_not_empty[port_priv->phy] = true;

	return count;
}

static ssize_t reg_dump_show(struct device *dev, struct device_attribute *attr,
			     char *buf)
{
	struct usb_serial_port *port = to_usb_serial_port(dev);
	struct usb_serial *serial = port->serial;
	struct f75115_port_private *port_priv = usb_get_serial_port_data(port);
	u8 tmp;
	int i, status, len = 0;

	len += sprintf(&buf[len], "Port Phy: %d\n", port_priv->phy);
	for (i = 0; i < 12; ++i) {
		status = f75115_getregister(serial->dev, port_priv->phy,
					    SERIAL_BASE_ADDRESS + i, &tmp);
		if (status) {
			dev_err(&port->dev, "%s: failed, %d\n", __func__,
				__LINE__);
			return status;
		}
		len += sprintf(&buf[len], "Reg: %04x, value:%02x\n",
			       SERIAL_BASE_ADDRESS + i, tmp);
	}

	return len;
}

static ssize_t reg_dump_store(struct device *dev,
			      struct device_attribute *attr, const char *buf,
			      size_t count)
{
	struct usb_serial_port *port = to_usb_serial_port(dev);
	struct usb_serial *serial = port->serial;
	//struct f75115_port_private *port_priv = usb_get_serial_port_data(port);
	int status;

	status = f75115_set_normal_register(serial->dev, 0x2007, 0x01);

	return count;
}
#endif

#ifndef CONFIG_GPIO_SYSFS
static ssize_t gpio_show(struct device *dev, struct device_attribute *attr,
			 char *buf)
{
	struct usb_serial_port *port = to_usb_serial_port(dev);
	struct f75115_port_private *port_priv = usb_get_serial_port_data(port);

	return sprintf(buf, "gpio mode: %d\n",
		       port_priv->port_pin_data.gpio_mode);
}

static ssize_t gpio_store(struct device *dev, struct device_attribute *attr,
			  const char *buf, size_t count)
{
	struct usb_serial_port *port = to_usb_serial_port(dev);
	struct usb_serial *serial = port->serial;
	struct f75115_port_private *port_priv = usb_get_serial_port_data(port);
	struct f75115_serial_private *serial_priv =
		usb_get_serial_data(serial);
	int status;
	u8 tmp;

	if (count <= 0)
		return -EINVAL;

	tmp = buf[0] - '0';

	mutex_lock(&serial_priv->change_mode_mutex);

	status = f75115_switch_gpio_mode(port, tmp);
	if (!status) {
		port_priv->port_pin_data.gpio_mode = tmp;
	}

	mutex_unlock(&serial_priv->change_mode_mutex);

	return status ? -EINVAL : count;
}
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 11, 0)
#ifdef SAVE_CONTROL
static DEVICE_ATTR_RW(no_save);
#endif

#ifdef DEBUG_INTERNAL_SYSFS
static DEVICE_ATTR_RW(internal);
static DEVICE_ATTR_RW(reg_dump);
#endif

#ifndef CONFIG_GPIO_SYSFS
static DEVICE_ATTR_RW(gpio);
#endif

#else
#ifdef SAVE_CONTROL
static DEVICE_ATTR(no_save, S_IRUGO | S_IWUSR, no_save_show, no_save_store);
#endif

#ifdef DEBUG_INTERNAL_SYSFS
static DEVICE_ATTR(internal, S_IRUGO | S_IWUSR, internal_show, internal_store);
static DEVICE_ATTR(reg_dump, S_IRUGO | S_IWUSR, reg_dump_show, reg_dump_store);
#endif

#ifndef CONFIG_GPIO_SYSFS
static DEVICE_ATTR(gpio, S_IRUGO | S_IWUSR, gpio_show, gpio_store);
#endif

#endif

static int f75115_init_msr(struct usb_serial_port *port)
{
	int status;
	struct f75115_port_private *port_priv = usb_get_serial_port_data(port);
	struct usb_serial *serial = port->serial;
	unsigned long flags;
	int phy = port_priv->phy;
	u8 msr;

	/* Get MSR initial value*/
	status = f75115_getregister(serial->dev, phy, MODEM_STATUS_REGISTER,
				    &msr);
	if (status)
		return status;

	spin_lock_irqsave(&port_priv->msr_lock, flags);
	port_priv->shadow_msr = msr;
	spin_unlock_irqrestore(&port_priv->msr_lock, flags);
	return 0;
}

static int f75115_port_probe(struct usb_serial_port *port)
{
	struct usb_serial *serial = port->serial;
	struct f75115_serial_private *serial_priv =
		usb_get_serial_data(serial);
	struct f75115_port_private *port_priv = NULL;
	int status, i, count = 0;
	int port_index = f75115_port_index(port);

#ifdef SAVE_CONTROL
	status = device_create_file(&port->dev, &dev_attr_no_save);
	if (status)
		return -EPERM;
#endif

#ifdef DEBUG_INTERNAL_SYSFS
	status = device_create_file(&port->dev, &dev_attr_internal);
	if (status)
		return -EPERM;

	status = device_create_file(&port->dev, &dev_attr_reg_dump);
	if (status)
		return -EPERM;
#endif

#ifndef CONFIG_GPIO_SYSFS
	status = device_create_file(&port->dev, &dev_attr_gpio);
	if (status)
		return -EPERM;
#endif

	port_priv = kzalloc(sizeof(*port_priv), GFP_KERNEL);
	if (!port_priv)
		return -ENOMEM;

	spin_lock_init(&port_priv->msr_lock);
	mutex_init(&port_priv->msr_mutex);
#ifdef WRITER_WQ
	INIT_WORK(&port_priv->writer_work, f75115_writer_worker);
	port_priv->port = port;
#endif

#ifdef LSR_ISSUE
	INIT_WORK(&port_priv->lsr_work, f75115_lsr_worker);
	port_priv->port = port;
#endif

	/* assign logic-to-phy mapping */
	port_priv->phy = F75115_UNUSED_PORT;

	for (i = 0; i < F75115_NUM_PORT; ++i) {
		if (serial_priv->default_conf_data[i] &
		    F75115_PORT_UNAVAILABLE)
			continue;

		if (port_index == count) {
			port_priv->phy = i;
			break;
		}

		++count;
	}

	if (port_priv->phy == F75115_UNUSED_PORT) {
		status = -ENODEV;
		goto port_fail;
	}

	usb_set_serial_port_data(port, port_priv);
	dev_info(&port->dev, "%s: mapping to phy: %d\n", __func__,
		 port_priv->phy);

	/*
	 * We'll read MSR reg only with port_porbe() for initial once, then
	 * the MSR will received from read URB with token
	 * F75115_TOKEN_MSR_CHANGE when MSR is changed.
	 */
	status = f75115_init_msr(port);
	if (status)
		goto port_fail;

	status = f75115_set_port_mode(port, uart_mode_rs232);
	if (status < 0) {
		dev_err(&port->dev, "%s: initial setup failed phy: (%i)\n",
			__func__, port_priv->phy);
		goto port_fail;
	}

	return 0;
port_fail:
	dev_err(&port->dev, "%s: failed, %d\n", __func__, status);
	kfree(port_priv);
	return status;
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 12, 0)
static void f75115_port_remove(struct usb_serial_port *port)
{
	int i;
	struct f75115_port_private *port_priv;
	struct usb_serial *serial = port->serial;
	struct usb_serial_port *port0 = serial->port[0];

#ifdef SAVE_CONTROL
	device_remove_file(&port->dev, &dev_attr_no_save);
#endif

#ifdef DEBUG_INTERNAL_SYSFS
	device_remove_file(&port->dev, &dev_attr_internal);
	device_remove_file(&port->dev, &dev_attr_reg_dump);
#endif

#ifndef CONFIG_GPIO_SYSFS
	device_remove_file(&port->dev, &dev_attr_gpio);
#endif

	/*
	 * Cancel "port0" read URB to avoid reference to a freed pointer on
	 * f75115_process_read_urb()
	 */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 3, 0)
	for (i = 0; i < ARRAY_SIZE(port0->read_urbs); ++i)
		usb_kill_urb(port0->read_urbs[i]);
#else
	usb_kill_urb(port0->read_urb);
#endif

	port_priv = usb_get_serial_port_data(port);
	kfree(port_priv);
}
#else
static int f75115_port_remove(struct usb_serial_port *port)
{
	int i;
	struct f75115_port_private *port_priv;
	struct usb_serial *serial = port->serial;
	struct usb_serial_port *port0 = serial->port[0];

#ifdef SAVE_CONTROL
	device_remove_file(&port->dev, &dev_attr_no_save);
#endif

#ifdef DEBUG_INTERNAL_SYSFS
	device_remove_file(&port->dev, &dev_attr_internal);
	device_remove_file(&port->dev, &dev_attr_reg_dump);
#endif

#ifndef CONFIG_GPIO_SYSFS
	device_remove_file(&port->dev, &dev_attr_gpio);
#endif

	/*
	 * Cancel "port0" read URB to avoid reference to a freed pointer on
	 * f75115_process_read_urb()
	 */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 3, 0)
	for (i = 0; i < ARRAY_SIZE(port0->read_urbs); ++i)
		usb_kill_urb(port0->read_urbs[i]);
#else
	usb_kill_urb(port0->read_urb);
#endif

	port_priv = usb_get_serial_port_data(port);
	kfree(port_priv);

	return 0;
}
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 0, 0)
static int f75115_tiocmget(struct tty_struct *tty)
#else
static int f75115_tiocmget(struct tty_struct *tty, struct file *file)
#endif
{
	struct usb_serial_port *port = tty->driver_data;
	struct f75115_port_private *port_priv = usb_get_serial_port_data(port);
	struct usb_serial *serial = port->serial;
	int r;
	u8 msr, mcr;

#if 0
	unsigned long flags;

	/*
	 * We'll avoid to direct read MSR register. The IC will read the MSR
	 * changed and report it f75115_process_per_serial_block() by
	 * F75115_TOKEN_MSR_CHANGE.
	 *
	 * When this device in heavy loading (e.g., BurnInTest Loopback Test)
	 * The report of MSR register will delay received a bit. It's due to
	 * MSR interrupt is lowest priority in 16550A. So we decide to sleep
	 * a little time to pass the test.
	 */
	if (schedule_timeout_killable(
			msecs_to_jiffies(F75115_DELAY_READ_MSR))) {
		dev_info(&port->dev, "%s: breaked !!\n", __func__);
		return -EINTR;
	}

	mutex_lock(&port_priv->msr_mutex);
	spin_lock_irqsave(&port_priv->msr_lock, flags);

	msr = port_priv->shadow_msr;
	mcr = port_priv->shadow_mcr;

	spin_unlock_irqrestore(&port_priv->msr_lock, flags);
	mutex_unlock(&port_priv->msr_mutex);
#else
	mutex_lock(&port_priv->msr_mutex);

	mcr = port_priv->shadow_mcr;

	r = f75115_getregister(serial->dev, port_priv->phy,
			       MODEM_STATUS_REGISTER, &msr);
	if (r) {
		mutex_unlock(&port_priv->msr_mutex);
		return r;
	}

	f75115_compare_msr(port, msr, 1);

	mutex_unlock(&port_priv->msr_mutex);
#endif

	r = (mcr & UART_MCR_DTR ? TIOCM_DTR : 0) |
	    (mcr & UART_MCR_RTS ? TIOCM_RTS : 0) |
	    (msr & UART_MSR_CTS ? TIOCM_CTS : 0) |
	    (msr & UART_MSR_DCD ? TIOCM_CAR : 0) |
	    (msr & UART_MSR_RI ? TIOCM_RI : 0) |
	    (msr & UART_MSR_DSR ? TIOCM_DSR : 0);

	return r;
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 0, 0)
static int f75115_tiocmset(struct tty_struct *tty, unsigned int set,
			   unsigned int clear)
#else
static int f75115_tiocmset(struct tty_struct *tty, struct file *file,
			   unsigned int set, unsigned int clear)
#endif
{
	struct usb_serial_port *port = tty->driver_data;

	return f75115_update_mctrl(port, set, clear);
}

static void f75115_dtr_rts(struct usb_serial_port *port, int on)
{
	if (on)
		f75115_update_mctrl(port, TIOCM_DTR | TIOCM_RTS, 0);
	else
		f75115_update_mctrl(port, 0, TIOCM_DTR | TIOCM_RTS);

	//dev_info(&port->dev, "%s: %d\n", __func__, on);
}

static int f75115_write(struct tty_struct *tty, struct usb_serial_port *port,
			const unsigned char *buf, int count)
{
#ifdef WRITER_WQ
	struct f75115_port_private *port_priv = usb_get_serial_port_data(port);
	int bytes_out;
#else
	int bytes_out, status;
#endif

	if (!count)
		return 0;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 33)
	bytes_out =
		kfifo_in_locked(&port->write_fifo, buf, count, &port->lock);
#else
	bytes_out = kfifo_put(port->write_fifo, buf, count);
#endif
#ifdef WRITER_WQ
	schedule_work(&port_priv->writer_work);
#else
	status = f75115_submit_writer(port, GFP_ATOMIC);
	if (status) {
		dev_err(&port->dev, "%s: submit failed\n", __func__);
		return status;
	}

#endif
	return bytes_out;
}

static int f75115_resume(struct usb_serial *serial)
{
	struct usb_serial_port *port;
#ifdef WRITER_WQ
	int status;
	struct f75115_port_private *port_priv;
#else
	int status, error = 0;
#endif
	int i;

	status = f75115_submit_read_urb(serial, GFP_NOIO);
	if (status) {
		dev_err(&serial->dev->dev,
			"%s: submit read URB failed!! status:%d!!\n", __func__,
			status);
		return status;
	}

	for (i = 0; i < serial->num_ports; i++) {
		port = serial->port[i];
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 7, 0)
		if (!tty_port_initialized(&port->port))
#else
		if (!test_bit(ASYNCB_INITIALIZED, &port->port.flags))
#endif
			continue;

		status = f75115_init_uart(port);
		if (status) {
			dev_err(&port->dev, "%s: submit failed\n", __func__);
			++error;
		}

#ifdef WRITER_WQ
		port_priv = usb_get_serial_port_data(port);
		schedule_work(&port_priv->writer_work);
#else
		status = f75115_submit_writer(port, GFP_NOIO);
		if (status) {
			dev_err(&port->dev, "%s: submit failed\n", __func__);
			++error;
		}
#endif
	}

#ifdef WRITER_WQ
	return 0;
#else
	return error ? -EIO : 0;
#endif
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 2, 0)
static inline int usb_endpoint_maxp(const struct usb_endpoint_descriptor *epd)
{
	return __le16_to_cpu(epd->wMaxPacketSize);
}
#endif

static int f75115_probe(struct usb_serial *serial,
			const struct usb_device_id *id)
{
	struct usb_endpoint_descriptor *endpoint;
	struct usb_host_interface *iface_desc;
	struct device *dev;
	int num_bulk_in = 0;
	int num_bulk_out = 0;
	int size_bulk_in = 0;
	int size_bulk_out = 0;
	int i;

	dev = &serial->interface->dev;
	iface_desc = serial->interface->cur_altsetting;

	for (i = 0; i < iface_desc->desc.bNumEndpoints; ++i) {
		endpoint = &iface_desc->endpoint[i].desc;

		if (usb_endpoint_is_bulk_in(endpoint)) {
			++num_bulk_in;
			size_bulk_in = usb_endpoint_maxp(endpoint);
		}

		if (usb_endpoint_is_bulk_out(endpoint)) {
			++num_bulk_out;
			size_bulk_out = usb_endpoint_maxp(endpoint);
		}
	}

	if (num_bulk_in != 1 || num_bulk_out != 1) {
		dev_err(dev, "expected endpoints not found\n");
		return -ENODEV;
	}

	if (size_bulk_out != F75115_WRITE_BUFFER_SIZE ||
	    size_bulk_in != F75115_MAX_RECEIVE_BLOCK_SIZE) {
		dev_err(dev, "unsupported endpoint max packet size\n");
		return -ENODEV;
	}

	pr_info("%s: f75115 driver ver: %s\n", __func__, DRIVER_VER);

	return 0;
}

static bool f75115_tx_empty(struct usb_serial_port *port)
{
	struct f75115_port_private *port_priv = usb_get_serial_port_data(port);
	struct f75115_serial_private *serial_priv =
		usb_get_serial_data(port->serial);
	unsigned long flags;
	bool status;

	spin_lock_irqsave(&serial_priv->tx_empty_lock, flags);
	status = serial_priv->is_phy_port_not_empty[port_priv->phy];
	spin_unlock_irqrestore(&serial_priv->tx_empty_lock, flags);

	return !status;
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 5, 0)
static struct usb_driver f75115_driver = {
	.name = "F75115",
	.probe = usb_serial_probe,
	.disconnect = usb_serial_disconnect,
	.id_table = id_table,
	.suspend = usb_serial_suspend,
	.resume = usb_serial_resume,
	.no_dynamic_id = 1,
	.supports_autosuspend = 1,
};
#endif

static struct usb_serial_driver f75115_device = {
	.driver = {
		   .owner = THIS_MODULE,
		   .name = IC_NAME,
		   },
	.description = DRIVER_DESC,
	.id_table = id_table,
#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 4, 0)
	.usb_driver = &f75115_driver,
#endif
	.probe = f75115_probe,
	.open = f75115_open,
	.close = f75115_close,
	.write = f75115_write,
	.calc_num_ports = f75115_calc_num_ports,
	.attach = f75115_attach,
	.disconnect = f75115_disconnect,

	.release = f75115_release,
	.port_probe = f75115_port_probe,
	.port_remove = f75115_port_remove,
	.dtr_rts = f75115_dtr_rts,
	.tx_empty = f75115_tx_empty,
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 35)
	.process_read_urb = f75115_process_read_urb,
#endif
	.ioctl = f75115_ioctl,
	.tiocmget = f75115_tiocmget,
	.tiocmset = f75115_tiocmset,
	.write_bulk_callback = f75115_write_usb_callback,
	.set_termios = f75115_set_termios,
	.resume = f75115_resume,
};

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 4, 0)
static struct usb_serial_driver *const serial_drivers[] = { &f75115_device,
							    NULL };

static int __init f75115_init(void)
{
	int r;

	r = usb_serial_register_drivers(serial_drivers, KBUILD_MODNAME,
					id_table);
	if (r)
		return r;

#ifdef I2C_HWMODE
	r = platform_driver_register(&f75115_i2c_driver);
	if (r) {
		usb_serial_deregister_drivers(serial_drivers);
		return r;
	}
#endif

	return 0;
}

static void __exit f75115_exit(void)
{
#ifdef I2C_HWMODE
	platform_driver_unregister(&f75115_i2c_driver);
#endif
	usb_serial_deregister_drivers(serial_drivers);
}

#else
static int __init f75115_init(void)
{
	int retval;

	retval = usb_serial_register(&f75115_device);
	if (retval)
		goto failed_usb_serial_register;

	retval = usb_register(&f75115_driver);
	if (retval)
		goto failed_usb_register;

#ifdef I2C_HWMODE
	retval = platform_driver_register(&f75115_i2c_driver);
	if (retval)
		goto failed_platform_register;
#endif

	return 0;

#ifdef I2C_HWMODE
failed_platform_register:
	usb_deregister(&f75115_driver);
#endif

failed_usb_register:
	usb_serial_deregister(&f75115_device);

failed_usb_serial_register:
	return retval;
}

static void __exit f75115_exit(void)
{
#ifdef I2C_HWMODE
	platform_driver_unregister(&f75115_i2c_driver);
#endif
	usb_deregister(&f75115_driver);
	usb_serial_deregister(&f75115_device);
}

#endif

module_init(f75115_init);
module_exit(f75115_exit);

MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_AUTHOR("Peter Hong <Peter_Hong@fintek.com.tw>");
MODULE_AUTHOR("Tom Tsai <Tom_Tsai@fintek.com.tw>");
MODULE_LICENSE("GPL");
