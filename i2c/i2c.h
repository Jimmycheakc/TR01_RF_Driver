/**
* @file i2c.h
* @brief This header file provides the i2c interfaces.
*
* @author Jimmy Chea
* @date 30/03/2023
*/

#ifndef __I2C__H__
#define __I2C__H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include <stdint.h>
#include <unistd.h>

#define I2C_TO_SPI_SLAVE_ADDRESS 0x28
#define I2C_TO_SPI_SLAVE_READ_ADDRESS 0xA8
#define I2C_TO_SPI_SLAVE_WRITE_ADDRESS 0x28

typedef enum
{
    unknow_ss = 0,
    at86rf215_1_ss = 1,
    at86rf215_2_ss = 2,
    at86rf215_3_ss = 3,
    at86rf215_4_ss = 4,

    rffc507x_1_ss = 5,
    rffc507x_2_ss = 6,

    ads7041_1_ss = 7,
    ads7041_2_ss = 8,
} spi_slave_select;

int i2c_open(const char *device);

void i2c_close(int fd);

uint8_t i2c_read(int fd, uint8_t addr, uint8_t reg_addr);

uint8_t i2c_read_multipe_bytes(int fd, uint8_t addr, uint8_t reg_addr, uint8_t *buf, uint8_t len);

int i2c_write(int fd, uint8_t addr, uint8_t reg_addr, uint8_t new_reg_value);

int i2c_write_multiple_bytes(int fd, uint8_t addr, uint8_t reg_addr, uint8_t *buf, uint8_t len);

int i2c_write_to_buffer(int fd, uint8_t addr, uint8_t reg_addr, uint8_t *buf, uint8_t len);

uint8_t i2c_read_from_buffer(int fd, uint8_t addr, uint8_t *buf, uint8_t len);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __I2C_H__ */