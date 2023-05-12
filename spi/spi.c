/**
* @file spi.c
* @brief This implementation file contains the code for spi interface.
*
* @author Jimmy Chea
* @date 30/03/2023
*/

#include "unistd.h"
#include <string.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include "spi.h"

/**
 *  A detailed description of the spi_init function
 *
 *  This function is to setup spi device with mode, bits and speed
 *
 *  @param \*self - spi device in struct of spi_t
 *  @param \*device - input the spi device name, eg, "/dev/spidev0.0"
 *  @param mode - 0 default mode, others can refer linux/spi/spidev.h
 *  @param bits - bits per word (usually 8)
 *  @param speed - max speed [Hz]
 *  @return - SPI_ERR_NONE(0) on success, else return other error refer tr01_spi.h
 */
int spi_init(spi_t *self,
             const char *device,    // filename like "/dev/spidev0.0"
             uint32_t mode,              // SPI_* (look "linux/spi/spidev.h")
             uint8_t bits,              // bits per word (usually 8)
             uint32_t speed)             // max speed [Hz]
{
    memset(&(self->xfer), 0, sizeof(self->xfer));

    // Open SPIdev
    self->fd = open(device, O_RDWR);
    if (self->fd < 0)
    {
        SPI_DBG("Error in spi_init(): failed to open the bus.");
        return SPI_ERR_OPEN;
    }

    // Set mode
    if (mode)
    {
        self->mode = (uint32_t) mode;
        if (ioctl(self->fd, SPI_IOC_WR_MODE32, &self->mode) < 0)
        {
            SPI_DBG("Error is spi_init(): can't set bus mode.");
            return SPI_ERR_SET_MODE;
        }
    }

    // Get mode
    if (ioctl(self->fd, SPI_IOC_RD_MODE32, &self->mode) < 0)
    {
        SPI_DBG("Error in spi_init(): can't get bus mode.");
        return SPI_ERR_GET_MODE;
    }

    // Get "LSB first"
    if (ioctl(self->fd, SPI_IOC_RD_LSB_FIRST, &self->lsb) < 0)
    {
        SPI_DBG("Error in spi_init(): can't get 'LSB first'.");
        return SPI_ERR_GET_LSB;
    }

    // Set bits per word
    if (bits)
    {
        self->bits = (uint8_t) bits;
        if (ioctl(self->fd, SPI_IOC_WR_BITS_PER_WORD, &self->bits) < 0)
        {
            SPI_DBG("Error in spi_init(): can't set bits per word.");
            return SPI_ERR_SET_BITS;
        }
    }

    // Get bits per word
    if (ioctl(self->fd, SPI_IOC_RD_BITS_PER_WORD, &self->bits) < 0)
    {
        SPI_DBG("Error in spi_init(): can't get bits per word.");
        return SPI_ERR_GET_BITS;
    }

    // Set max speed [Hz]
    if (speed)
    {
        self->speed = (uint32_t) speed;
        if (ioctl(self->fd, SPI_IOC_WR_MAX_SPEED_HZ, &self->speed) < 0)
        {
            SPI_DBG("Error in spi_init(): can't set max speed [Hz].");
            return SPI_ERR_SET_SPEED;
        }
    }

    // Get max speed [Hz]
    if (ioctl(self->fd, SPI_IOC_RD_MAX_SPEED_HZ, &self->speed) < 0)
    {
        SPI_DBG("Error in spi_init(): can't get max speed [Hz].");
        return SPI_ERR_GET_SPEED;
    }

    SPI_DBG("Open device='%s' mode=0x%x bits=%u lsb=%d max_speed=%u [Hz]",
            device, self->mode, self->bits, self->lsb,
            self->speed);

    return SPI_ERR_NONE;
}

/**
 *  A detailed description of the spi_free function
 *
 *  This function is to close the spi device
 *
 *  @param \*self - spi device in struct of spi_t
 *  @return - none
 */
void spi_free(spi_t *self)
{
    close(self->fd);
}

/**
 *  A detailed description of the spi_read function
 *
 *  This function is to read the buffer from spi device via /dev/spidev* and return result
 *
 *  @param \*self - spi device in struct of spi_t
 *  @param \*rx_buf - receive buffer
 *  @param len - size of receive buffer
 *  @return - number of read bytes on success, else return SPI_ERR_READ(-9) on error
 */
int spi_read(spi_t *self, uint8_t *rx_buf, size_t len)
{
    int retv;

    self->xfer.tx_buf = (unsigned long) 0;
    self->xfer.rx_buf = (unsigned long) rx_buf;
    self->xfer.len = len;
    self->xfer.delay_usecs = 0;
    self->xfer.speed_hz = self->speed;
    self->xfer.bits_per_word = self->bits;

    retv = ioctl(self->fd, SPI_IOC_MESSAGE(1), &self->xfer);
    if (retv < 0)
    {
        SPI_DBG("Error in spi_read(): ioctl(SPI_IOC_MESSAGE(1)) return %d", retv);
        return SPI_ERR_READ;
    }

    return retv;
}

/**
 *  A detailed description of the spi_write function
 *
 *  This function is to write the buffer to spi device via /dev/spidev* and return result
 *
 *  @param \*self - spi device in struct of spi_t
 *  @param \*tx_buf - transmit buffer
 *  @param len - size of transmit buffer
 *  @return - number of write bytes on success, else return SPI_ERR_WRITE(-10) on error
 */
int spi_write(spi_t *self, const uint8_t *tx_buf, size_t len)
{
    int retv;

    self->xfer.tx_buf = (unsigned long) tx_buf;
    self->xfer.rx_buf = (unsigned long) 0;
    self->xfer.len = len;
    self->xfer.delay_usecs = 0;
    self->xfer.speed_hz = self->speed;
    self->xfer.bits_per_word = self->bits;

    retv = ioctl(self->fd, SPI_IOC_MESSAGE(1), &(self->xfer));
    if (retv < 1)
    {
        SPI_DBG("Error in spi_write(): ioctl(SPI_IOC_MESSAGE(1)) return %d", retv);
        return SPI_ERR_WRITE;
    }

    return retv;
}

/**
 *  A detailed description of the spi_exchange function
 *
 *  This function is to perform write and read from spi device and return result
 *
 *  @param \*self - spi device in struct of spi_t
 *  @param \*rx_buf - receive buffer
 *  @param \*tx_buf - transmit buffer
 *  @param len - size of the buffer
 *  @return - number of write bytes on success, else return SPI_ERR_EXCHANGE(-11) on error
 */
int spi_exchange(spi_t *self, uint8_t *rx_buf, const uint8_t *tx_buf, size_t len)
{
    int retv;

    self->xfer.tx_buf = (unsigned long) tx_buf;
    self->xfer.rx_buf = (unsigned long) rx_buf;
    self->xfer.len = len;
    self->xfer.delay_usecs = 0;
    self->xfer.speed_hz = self->speed;
    self->xfer.bits_per_word = self->bits;

    retv = ioctl(self->fd, SPI_IOC_MESSAGE(1), &self->xfer);
    if (retv < 0)
    {
        SPI_DBG("Error in spi_exchange(): ioctl(SPI_IOC_MESSAGE(1)) return %d", retv);
        return SPI_ERR_EXCHANGE;
    }

    return retv;
}

int spi_write16(spi_t *self, const uint16_t *tx_buf, size_t len)
{
    int retv;

    self->xfer.tx_buf = (unsigned long) tx_buf;
    self->xfer.rx_buf = (unsigned long) 0;
    self->xfer.len = len;
    self->xfer.delay_usecs = 0;
    self->xfer.speed_hz = self->speed;
    self->xfer.bits_per_word = self->bits;

    retv = ioctl(self->fd, SPI_IOC_MESSAGE(1), &(self->xfer));
    if (retv < 1)
    {
        SPI_DBG("Error in spi_write(): ioctl(SPI_IOC_MESSAGE(1)) return %d", retv);
        return SPI_ERR_WRITE;
    }

    return retv;
}

int spi_write32(spi_t *self, const uint32_t *tx_buf, size_t len)
{
    int retv;

    self->xfer.tx_buf = (unsigned long) tx_buf;
    self->xfer.rx_buf = (unsigned long) 0;
    self->xfer.len = len;
    self->xfer.delay_usecs = 0;
    self->xfer.speed_hz = self->speed;
    self->xfer.bits_per_word = self->bits;

    retv = ioctl(self->fd, SPI_IOC_MESSAGE(1), &(self->xfer));
    if (retv < 1)
    {
        SPI_DBG("Error in spi_write(): ioctl(SPI_IOC_MESSAGE(1)) return %d", retv);
        return SPI_ERR_WRITE;
    }

    return retv;
}

int spi_exchange32(spi_t *self, uint32_t *rx_buf, const uint32_t *tx_buf, size_t len)
{
    int retv;

    self->xfer.tx_buf = (unsigned long) tx_buf;
    self->xfer.rx_buf = (unsigned long) rx_buf;
    self->xfer.len = len;
    self->xfer.delay_usecs = 0;
    self->xfer.speed_hz = self->speed;
    self->xfer.bits_per_word = self->bits;

    retv = ioctl(self->fd, SPI_IOC_MESSAGE(1), &self->xfer);
    if (retv < 0)
    {
        SPI_DBG("Error in spi_exchange(): ioctl(SPI_IOC_MESSAGE(1)) return %d", retv);
        return SPI_ERR_EXCHANGE;
    }

    return retv;
}
