/*
 * Simple Linux wrapper for access to /dev/spidev
 * File: tr01_spi_test.c
 */

#include <stdint.h>
#include <stdio.h>
#include "tr01_spi.h"

#define SPI_DEVICE "/dev/spidev1.0"

int main()
{
    spi_t spi;
    uint16_t reg_addr = 0x0000;
    char tx_buf[3] = {0x00, 0x00, 0x00};
    char tx_buf1[3] = {0x80, 0x00, 0x00};
    char rx_buf[3] = {0x00, 0x00, 0x00};
    int i;

    int retv = spi_init(&spi, SPI_DEVICE, 0, 0, 2500000);

    printf(">>> spi_init() return %d\n", retv);

/*
    tx_buf1[0] |= (char)(reg_addr >> 8);
    tx_buf1[1] |= (char)reg_addr;
    retv = spi_write(&spi, tx_buf1, 3);
    printf(">>> spi_write(1024) return %d\n", retv);
*/

    tx_buf[0] |= (char)(reg_addr >> 8);
    tx_buf[1] |= (char)reg_addr;
    retv = spi_exchange(&spi, rx_buf, tx_buf, 6);
    printf(">>> spi_exchange() return %d\n", retv);
    printf("rx_buf[0] = %d\n", rx_buf[0]);
    printf("rx_buf[1] = %d\n", rx_buf[1]);
    printf("rx_buf[2] = %d\n", rx_buf[2]);
    printf("rx_buf[3] = %d\n", rx_buf[3]);

    spi_free(&spi);

    return 0;
}