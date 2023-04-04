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
#include "../i2c/i2c.h"

static uint16_t ads7041_read_register(ads7041_t *self, uint8_t reg_addr)
{
    uint8_t data_buf[2];
    uint16_t ret_buf = 0;

    int ret = i2c_read_multipe_bytes(self->fd, self->slave_addr, reg_addr, data_buf, 2);
    if (ret < 0)
    {
        return -1;
    }

    ret_buf |= ((uint16_t)data_buf[0] << 4);
    ret_buf |= data_buf[1] >> 4;

    return ret_buf;
}

void ads7041_init(ads7041_t *self, const char *device_name, uint8_t addr)
{
    self->fd = i2c_open(device_name);
    self->dev_name = device_name;
    self->slave_addr = addr;
}

void ads7041_close(ads7041_t *self)
{
    i2c_close(self->fd);
}

double ads7041_get_adc_millivolt(ads7041_t *self, uint8_t reg_addr)
{
    double adc_mV;
    uint16_t adc_raw;

    adc_raw = ads7041_read_register(self, reg_addr);
    adc_mV = (adc_raw * 3300) / 1024.0;

    return adc_mV;
}