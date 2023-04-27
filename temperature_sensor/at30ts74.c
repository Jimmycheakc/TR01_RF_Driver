/**
* @file at30ts74.c
* @brief This implementation file contains the code for access the interface of the Microchip at30ts74 temperature sensor. 
*
* @author Jimmy Chea
* @date 30/03/2023
*/

#include <stdio.h>
#include <stdint.h>
#include "at30ts74.h"
#include "../i2c/i2c.h"

static void at30ts74_write_register(at30ts74_t *self, uint8_t reg_addr, uint16_t value)
{
    uint8_t data_buf[2];

    data_buf[0] = (value >> 8) & 0xFF;
    data_buf[1] = value & 0xFF;

    i2c_write_multiple_bytes(self->fd, self->slave_addr, reg_addr, data_buf, 2);
}

static uint16_t at30ts74_read_register(at30ts74_t *self, uint8_t reg_addr)
{
    uint8_t data_buf[2];
    uint16_t ret_buf = 0;

    int ret = i2c_read_multipe_bytes(self->fd, self->slave_addr, reg_addr, data_buf, 2);
    if (ret < 0)
    {
        return -1;
    }

    ret_buf |= ((uint16_t)data_buf[0] << 8);
    ret_buf |= data_buf[1];

    return ret_buf;
}

void at30ts74_set_oneshot(at30ts74_t *self, atOneShot_t oneshot)
{
    at30ts74_write_register(self, AT30TS74_REG_POINTER_CONFIG, oneshot);
}

uint16_t at30ts74_get_oneshot(at30ts74_t *self)
{
    uint16_t val = at30ts74_read_register(self, AT30TS74_REG_POINTER_CONFIG);

    return (val && AT30TS74_REG_CONFIG_OS_MASK);
}

void at30ts74_set_resolution(at30ts74_t *self, atResolution_t resolution)
{
    at30ts74_write_register(self, AT30TS74_REG_POINTER_CONFIG, resolution);
}

uint16_t at30ts74_get_resolution(at30ts74_t *self)
{
    uint16_t val = at30ts74_read_register(self, AT30TS74_REG_POINTER_CONFIG);

    return (val && AT30TS74_REG_CONFIG_RES_MASK);
}

void at30ts74_set_fault_queue(at30ts74_t *self, atFaultQueue_t faultQueue)
{
    at30ts74_write_register(self, AT30TS74_REG_POINTER_CONFIG, faultQueue);
}

uint16_t at30ts74_get_fault_queue(at30ts74_t *self)
{
    uint16_t val = at30ts74_read_register(self, AT30TS74_REG_POINTER_CONFIG);

    return (val && AT30TS74_REG_CONFIG_FAULTQUE_MASK);
}

void at30ts74_set_alert_polarity(at30ts74_t *self, atAlertPolarity_t alertPolarity)
{
    at30ts74_write_register(self, AT30TS74_REG_POINTER_CONFIG, alertPolarity);
}

uint16_t at30ts74_get_alert_polarity(at30ts74_t *self)
{
    uint16_t val = at30ts74_read_register(self, AT30TS74_REG_POINTER_CONFIG);

    return (val && AT30TS74_REG_CONFIG_ALERPOL_MASK);
}

void at30ts74_set_mode(at30ts74_t *self, atMode_t mode)
{
    at30ts74_write_register(self, AT30TS74_REG_POINTER_CONFIG, mode);
}

uint16_t at30ts74_get_mode(at30ts74_t *self)
{
    uint16_t val = at30ts74_read_register(self, AT30TS74_REG_POINTER_CONFIG);

    return (val && AT30TS74_REG_CONFIG_CMPINT_MASK);
}

void at30ts74_set_shutdown(at30ts74_t *self, atShutDown_t shudown)
{
    at30ts74_write_register(self, AT30TS74_REG_POINTER_CONFIG, shudown);
}

uint16_t at30ts74_get_shoutdown(at30ts74_t *self)
{
    uint16_t val = at30ts74_read_register(self, AT30TS74_REG_POINTER_CONFIG);

    return (val && AT30TS74_REG_CONFIG_SD_MASK);
}

int at30ts74_init(at30ts74_t *self, const char *device_name, uint8_t addr)
{
    int fd = i2c_open(device_name);
    if (fd < 0)
    {
        printf("unable to open i2c-dev\n");
        return -1;
    }
    self->fd = fd;
    self->dev_name = device_name;
    self->slave_addr = addr;
}

void at30ts74_close(at30ts74_t *self)
{
    i2c_close(self->fd);
    self->dev_name = NULL;
    self->slave_addr = 0;
}

int16_t at30ts74_get_raw_temperature(at30ts74_t *self)
{
    uint16_t temperature = at30ts74_read_register(self, AT30TS74_REG_POINTER_TEMP);
    return (int16_t)temperature;
}

double at30ts74_get_temperature_celsius(at30ts74_t *self)
{
    int16_t raw_temp;
    int shift_bit;
    double res_per_bit;
    double ret_temp;

    switch(at30ts74_get_resolution(self))
    {
        case AT30TS74_REG_CONFIG_RES_9BIT:
            shift_bit = 7;
            res_per_bit = 0.5;
            usleep(25000);
            break;
        case AT30TS74_REG_CONFIG_RES_10BIT:
            shift_bit = 6;
            res_per_bit = 0.25;
            usleep(50000);
            break;
        case AT30TS74_REG_CONFIG_RES_11BIT:
            shift_bit = 5;
            res_per_bit = 0.125;
            usleep(100000);
            break;
        case AT30TS74_REG_CONFIG_RES_12BIT:
            shift_bit = 4;
            res_per_bit = 0.0625;
            usleep(200000);
            break;
        default:
            shift_bit = 7;
            res_per_bit = 0.5;
            usleep(25000);
            break;
    }

    raw_temp = at30ts74_get_raw_temperature(self);

    ret_temp = (raw_temp >> shift_bit) * res_per_bit;

    return ret_temp;
}