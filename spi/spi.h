/**
* @file tr01_spi.h
* @brief This header file provides the spi interfaces.
*
* @author Jimmy Chea
* @date 30/03/2023
*/

#ifndef __SPI_H__
#define __SPI_H__

#include <stdint.h>
#include <stdio.h>
#include <linux/spi/spidev.h>

// Define Common error codes (return values)
#define SPI_ERR_NONE        0  // no error, success
#define SPI_ERR_OPEN       -1  // failed to open the bus
#define SPI_ERR_SET_MODE   -2  // can't set bus mode
#define SPI_ERR_GET_MODE   -3  // can't get bus mode
#define SPI_ERR_GET_LSB    -4  // can't get 'LSB first'
#define SPI_ERR_SET_BITS   -5  // can't set bits per word
#define SPI_ERR_GET_BITS   -6  // can't get bits per word
#define SPI_ERR_SET_SPEED  -7  // can't set max speed [Hz]
#define SPI_ERR_GET_SPEED  -8  // can't get max speed [Hz]
#define SPI_ERR_READ       -9  // can't read
#define SPI_ERR_WRITE      -10 // can't write
#define SPI_ERR_EXCHANGE   -11 // can't read/write

#define SPI_DEBUG 1

#ifdef SPI_DEBUG
#   include <stdio.h>
#       define SPI_DBG(fmt, arg...) fprintf(stderr, "SPI: " fmt "\n", ## arg)
#else
#   define SPI_DBG(fmt, ...) // debug output off
#endif

#ifdef __cplusplus
extern "C"
{
#endif

// Structure of spi_t
typedef struct spi_ {
    int         fd;     // file descriptor: fd = open(filename, O_RDWR);
    uint32_t    speed;  // speed [Hz]
    uint32_t    mode;   // SPI mode
    uint8_t     lsb;    // LSB first
    uint8_t     bits;   // bits per word
    struct  spi_ioc_transfer xfer;
} spi_t;

int spi_init(spi_t *self,
             const char *device,    // filename like "/dev/spidev0.0"
             uint32_t mode,              // SPI_* (look "linux/spi/spidev.h")
             uint8_t bits,              // bits per word (usually 8)
             uint32_t speed);            // max speed [Hz]

void spi_free(spi_t *self);

int spi_read(spi_t *self, uint8_t *rx_buf, size_t len);

int spi_write(spi_t *self, const uint8_t *tx_buf, size_t len);

int spi_write16(spi_t *self, const uint16_t *tx_buf, size_t len);

int spi_write32(spi_t *self, const uint32_t *tx_buf, size_t len);

int spi_exchange(spi_t * self, uint8_t *rx_buf, const uint8_t *tx_buf, size_t len);

int spi_exchange32(spi_t *self, uint32_t *rx_buf, const uint32_t *tx_buf, size_t len);

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // __SPI_H__