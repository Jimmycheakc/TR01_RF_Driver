/*
 * Simple Linux wrapper for access to /dev/spidev
 * File: tr01_spi_test.c
 */

#include <stdint.h>
#include <stdio.h>
#include "spi.h"

#define SPI_DEVICE "/dev/spidev1.0"
#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))

int main()
{
    spi_t spi;
    uint16_t reg_addr = 0x0000;
    uint8_t tx_buf1[] = {0x80, 0x00, 0x00, 0xFF};
    uint16_t tx_buf2[] = {0x0101, 0x0101, 0x0101};
    uint32_t tx_buf3[] = {
                            0x00000001, 0x00000001, 0x00000001, 0x00000001,
                            0x00000002, 0x00000002, 0x00000002, 0x00000002,
                            0x00000003, 0x00000003, 0x00000003, 0x00000003,
                            0x00000004, 0x00000004, 0x00000004, 0x00000004,
                            0x00000005, 0x00000005, 0x00000005, 0x00000005,
                            0x00000006, 0x00000006, 0x00000006, 0x00000006,
                            0x00000007, 0x00000007, 0x00000007, 0x00000007,
                            0x00000008, 0x00000008, 0x00000008, 0x00000008,
                            0x00000009, 0x00000009, 0x00000009, 0x00000009,
                            0x00000010, 0x00000010, 0x00000010, 0x00000010,
                            0x00000011, 0x00000011, 0x00000011, 0x00000011,
                            0x00000012, 0x00000012, 0x00000012, 0x00000012,
                            0x00000013, 0x00000013, 0x00000013, 0x00000013,
                            0x00000014, 0x00000014, 0x00000014, 0x00000014,
                            0x00000015, 0x00000015, 0x00000015, 0x00000015,
                            0x00000016, 0x00000016, 0x00000016, 0x00000016,

                         };
    uint32_t rx_buf[ARRAY_SIZE(tx_buf3)];

    for (size_t i = 0; i < ARRAY_SIZE(tx_buf3); i++)
    {
        rx_buf[i] = 0x00000000;
    }

    int retv = spi_init(&spi, SPI_DEVICE, 0, 8, 2000000);

    printf(">>> spi_init() return %d\n", retv);

    retv = spi_write(&spi, tx_buf1, sizeof(tx_buf1));
    printf(">>> spi_write(1024) return %d\n", retv);
    //retv = spi_exchange32(&spi, rx_buf, tx_buf3, sizeof(tx_buf3));
    //printf(">>> spi_write(1024) return %d\n", retv);

    spi_free(&spi);

    return 0;
}