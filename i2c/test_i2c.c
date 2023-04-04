#include <stdio.h>
#include "i2c.h"

int main ()
{
    int fd;
    int ret;
    int val;

    fd = i2c_open("/dev/i2c-4");

    ret = i2c_write(fd, 0x48, 0x00, 0x01);
    if (ret < 0)
    {
        printf("Unable to write the register\n");
    }

    val = i2c_read(fd, 0x48, 0x00);
    if (ret < 0)
    {
        printf("Unable to read the register\n");
    }
    printf("The value is %d\n", val);

    i2c_close(fd);

    return 0;
}