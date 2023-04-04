#include <stdio.h>
#include "rffc507x.h"
#include "../io_utils/io_utils.h"
#include "../tr01_spi.h"

#define SPI_DEVICE "/dev/spidev1.0"

static spi_t spi;

rffc507x_st dev =
{
    .reset_chip = "gpiochip0",
    .reset_pin = 8,
    .reset_consumer = "rffc507x_reset",
	.ref_freq_hz = 32e6,
};

int main ()
{
    spi_init(&spi, SPI_DEVICE, 0, 0, 5000000);

    rffc507x_init(&dev, &spi);

    printf("RFFC507X Registers:\n");
	for (int i = 0; i < RFFC507X_NUM_REGS; i ++)
	{
		uint16_t reg_val = rffc507x_reg_read(&dev, i);
		printf("REG #%d => %04X\n", i, reg_val);
	}

    rffc507x_device_id_st dev_id;
	rffc507x_device_status_st stat;
	rffc507x_readback_status(&dev, &dev_id, &stat);
	rffc507x_print_dev_id(&dev_id);
	rffc507x_print_stat(&stat);

	rffc507x_set_frequency(&dev, 85e6);

	for (int i = 0; i<5; i++)
	{
		io_utils_usleep(10000);
		rffc507x_readback_status(&dev, NULL, &stat);
		rffc507x_print_stat(&stat);
	}



	rffc507x_set_frequency(&dev, 314159265);

	for (int i = 0; i<5; i++)
	{
		io_utils_usleep(10000);
		rffc507x_readback_status(&dev, NULL, &stat);
		rffc507x_print_stat(&stat);
	}


	rffc507x_set_frequency(&dev, 915e6);

	for (int i = 0; i<5; i++)
	{
		io_utils_usleep(10000);
		rffc507x_readback_status(&dev, NULL, &stat);
		rffc507x_print_stat(&stat);
	}

	rffc507x_set_frequency(&dev, 1200e6);

	for (int i = 0; i<5; i++)
	{
		io_utils_usleep(10000);
		rffc507x_readback_status(&dev, NULL, &stat);
		rffc507x_print_stat(&stat);
	}


	rffc507x_set_frequency(&dev, 4600e6);

	for (int i = 0; i<5; i++)
	{
		io_utils_usleep(10000);
		rffc507x_readback_status(&dev, NULL, &stat);
		rffc507x_print_stat(&stat);
	}


	rffc507x_set_frequency(&dev, 5600e6);

	for (int i = 0; i<5; i++)
	{
		io_utils_usleep(10000);
		rffc507x_readback_status(&dev, NULL, &stat);
		rffc507x_print_stat(&stat);
	}

	rffc507x_release(&dev);

    spi_free(&spi);

    return 0;
}