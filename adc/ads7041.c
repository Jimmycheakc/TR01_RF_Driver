/**
* @file ads7041.c
* @brief This implementation file contains the code for access the ADC from TI ADS7041IRUGR. 
*
* @author Jimmy Chea
* @date 30/03/2023
*/

#include <stdio.h>
#include <stdint.h>
#include "ads7041.h"

static uint16_t ads7041_read_register(ads7041_t *self, uint8_t reg_addr)
{
    int ret;
    uint8_t data_buf[2];
    uint16_t ret_buf = 0;

    ret = i2c_write_to_buffer(self->fd, I2C_TO_SPI_SLAVE_WRITE_ADDRESS, self->slave_select, NULL, 0);
    if (ret < 0)
    {
        printf("Failed to write to buffer.\n");
        return -1;
    }

    // Temp: Need to update the i2c_read_from_buffer()
    /*
    ret = i2c_read_from_buffer(self->fd, I2C_TO_SPI_SLAVE_READ_ADDRESS, data_buf, 2);
    if (ret < 0)
    {
        printf("Failed to read from buffer.\n");
        return -1;
    }

    ret_buf |= ((uint16_t)data_buf[0] << 4);
    ret_buf |= data_buf[1] >> 4;
    */

    return ret_buf;
}

int ads7041_init(ads7041_t *self, const char *device_name, uint8_t addr, spi_slave_select slave_select)
{
    int fd = i2c_open(device_name);
    if (fd < 0)
    {
        printf("unable to open i2c-dev\n");
        return -1;
    }

    if (slave_select < ads7041_1_ss || slave_select > ads7041_2_ss)
    {
        printf("Slave select must be within its range.\n");
        return -1;
    }

    self->fd = fd;
    self->dev_name = device_name;
    self->slave_addr = addr;
    self->slave_select = slave_select;
}

void ads7041_close(ads7041_t *self)
{
    i2c_close(self->fd);
    self->dev_name = NULL;
    self->slave_addr = 0;
    self->slave_select = unknow_ss;
}

double ads7041_get_adc_millivolt(ads7041_t *self, uint8_t reg_addr)
{
    double adc_mV;
    uint16_t adc_raw;

    adc_raw = ads7041_read_register(self, reg_addr);
    adc_mV = (adc_raw * 3300) / 1024.0;

    return adc_mV;
}