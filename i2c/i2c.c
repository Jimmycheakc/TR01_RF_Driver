/**
* @file i2c.c
* @brief This implementation file contains the code for i2c interface.
*
* @author Jimmy Chea
* @date 30/03/2023
*/

#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <string.h>

/**
 *  A detailed description of the i2c_open function
 *
 *  This function is to open the i2c device under /dev/i2c-* and return file descriptor
 *
 *  @param \*device - input the device name "/dev/i2c-*"
 *  @return - file descriptor on success, else return -1 on error
 */
int i2c_open(const char *device)
{
    int fd = open(device, O_RDWR);
    if (fd < 0)
    {
        printf("ERROR: open(%d) failed.\n", fd);
    }

    return fd;
}

/**
 *  A detailed description of the i2c_close function
 *
 *  This function is to close the i2c device based on the file descriptor
 *
 *  @param fd - file descriptor
 *  @return - none
 */
void i2c_close(int fd)
{
    close(fd);
}

/**
 *  A detailed description of the i2c_read function
 *
 *  This function is to read the register value from i2c slave device
 *
 *  @param fd - file descriptor
 *  @param addr - i2c slave address
 *  @param reg_addr - i2c slave register address
 *  @return - register value on success, else return -1 on error
 */
uint8_t i2c_read(int fd, uint8_t addr, uint8_t reg_addr)
{
    struct i2c_msg msg[2];
    struct i2c_rdwr_ioctl_data data;
    uint8_t reg_value;
    int ret;

    msg[0].addr = (__u16)addr;
    msg[0].flags = 0;
    msg[0].len = 1;
    msg[0].buf = &reg_addr;

    msg[1].addr = (__u16)addr;
    msg[1].flags = I2C_M_RD;
    msg[1].len = 1;
    msg[1].buf = &reg_value;

    data.msgs = msg;
    data.nmsgs = 2;

    ret = ioctl(fd, I2C_RDWR, &data);
    if (ret < 0)
    {
        printf("ERROR: unable to read data\n");
        return ret;
    }

    return reg_value;
}

/**
 *  A detailed description of the i2c_read function
 *
 *  This function is to read the register value from i2c slave device
 *
 *  @param fd - file descriptor
 *  @param addr - i2c slave address
 *  @param reg_addr - i2c slave register address
 *  @param \*buf - buffer to read
 *  @param len - number of bytes to read
 *  @return - number of read bytes on success, else return -1 on error
 */
uint8_t i2c_read_multipe_bytes(int fd, uint8_t addr, uint8_t reg_addr, uint8_t *buf, uint8_t len)
{
    struct i2c_msg msg[2];
    struct i2c_rdwr_ioctl_data data;
    uint8_t reg_value;
    int ret;

    msg[0].addr = (__u16)addr;
    msg[0].flags = 0;
    msg[0].len = 1;
    msg[0].buf = &reg_addr;

    msg[1].addr = (__u16)addr;
    msg[1].flags = I2C_M_RD;
    msg[1].len = len;
    msg[1].buf = buf;

    data.msgs = msg;
    data.nmsgs = 2;

    ret = ioctl(fd, I2C_RDWR, &data);
    if (ret < 0)
    {
        printf("ERROR: unable to read data\n");
        return ret;
    }

    return len;
}

/**
 *  A detailed description of the i2c_write function
 *
 *  This function is to write the register value to i2c slave device's register
 *
 *  @param fd - file descriptor
 *  @param addr - i2c slave address
 *  @param reg_addr - i2c slave register address
 *  @param new_reg_value - register value
 *  @return - integer value on success, else return -1 on error
 */
int i2c_write(int fd, uint8_t addr, uint8_t reg_addr, uint8_t new_reg_value)
{
    struct i2c_msg msg;
    struct i2c_rdwr_ioctl_data data;
    uint8_t buf[2];
    int ret;

    buf[0] = reg_addr;
    buf[1] = new_reg_value;

    msg.addr = (__u16)addr;
    msg.flags = 0;
    msg.len = 2;
    msg.buf = buf;

    data.msgs = &msg;
    data.nmsgs = 1;

    ret = ioctl(fd, I2C_RDWR, &data);
    if (ret < 0)
    {
        printf("Error: unable to write data\n");
    }

    return ret;
}

/**
 *  A detailed description of the i2c_write function
 *
 *  This function is to write the register value to i2c slave device's register
 *
 *  @param fd - file descriptor
 *  @param addr - i2c slave address
 *  @param reg_addr - i2c slave register address
 *  @param new_reg_value - register value
 *  @param \*buf - data buffer to write
 *  @param len - number of bytes to write
 *  @return - number of bytes written on success, else return -1 on error
 */
int i2c_write_multiple_bytes(int fd, uint8_t addr, uint8_t reg_addr, uint8_t *buf, uint8_t len)
{
    struct i2c_msg msg;
    struct i2c_rdwr_ioctl_data data;
    uint8_t data_buf[2];
    int ret;

    data_buf[0] = reg_addr;
    memcpy(&data_buf[1], buf, len);

    msg.addr = (__u16)addr;
    msg.flags = 0;
    msg.len = len + 1;
    msg.buf = data_buf;

    data.msgs = &msg;
    data.nmsgs = 1;

    ret = ioctl(fd, I2C_RDWR, &data);
    if (ret < 0)
    {
        printf("Error: unable to write data\n");
    }

    return ret;
}
