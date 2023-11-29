#ifndef __F81534_40_PIN_H__
#define __F81534_40_PIN_H__

struct spi_gpio_platform_data_extra {
	struct spi_gpio_platform_data gpio_data;
	struct spi_device *device[2];
	unsigned int cs[0];
};

#endif