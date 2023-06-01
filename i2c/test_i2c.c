#include <stdio.h>
#include "i2c.h"

int main ()
{
    int fd;
    int ret;
    int val;

    fd = i2c_open("/dev/i2c-2");

    ret = i2c_write(fd, 0x28, 0x00, 0x01);
    if (ret < 0)
    {
        printf("Unable to write the register\n");
    }

    val = i2c_read(fd, 0x28, 0x00);
    if (ret < 0)
    {
        printf("Unable to read the register\n");
    }
    printf("The value is %d\n", val);

    uint8_t chunk_tx[3] = {0};

    chunk_tx[0] = ((0x0100 >> 8) & 0x3F) | 0x80;
    chunk_tx[1] = 0x0100 & 0xFF;
    chunk_tx[2] = 0x0F;

    ret = i2c_write_to_buffer(fd, I2C_TO_SPI_SLAVE_WRITE_ADDRESS, 0x01, chunk_tx, 3);
    if (ret < 0)
    {
        printf("Unable to write the register\n");
    }
    i2c_close(fd);

    return 0;
}