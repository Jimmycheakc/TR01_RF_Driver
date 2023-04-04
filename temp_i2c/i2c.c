#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/select.h>
#include <sys/time.h>
#include <errno.h>
#include <string.h>
/* I2C Device*/
#define I2C0_DEV_NAME                      "/dev/i2c-0"
#define I2C1_DEV_NAME                      "/dev/i2c-1"
#define I2C2_DEV_NAME                      "/dev/i2c-2"
#define I2C3_DEV_NAME                      "/dev/i2c-3"
#define I2C_RETRIES                        0x0701
#define I2C_TIMEOUT                        0x0702
#define I2C_SLAVE                          0x0703
#define I2C_RDWR                           0x0707
#define I2C_BUS_MODE                       0x0780
#define I2C_M_RD                           0x01
#define PCA6416_SLAVE_ADDR                 0x20
#define PCA6416_GPIO_CFG_REG               0x07
#define PCA6416_GPIO_PORARITY_REG          0x05
#define PCA6416_GPIO_OUT_REG               0x03
#define PCA6416_GPIO_IN_REG                0x01
//GPIO MASK
#define GPIO3_MASK                         0x10
#define GPIO4_MASK                         0x20
#define GPIO5_MASK                         0x40
#define GPIO6_MASK                         0x80
#define GPIO7_MASK                         0x08
struct i2c_msg {
    unsigned short addr;     /* slave address */
    unsigned short flags;
    unsigned short len;
    unsigned char *buf;     /* message data pointer */
};
struct i2c_rdwr_ioctl_data {
    struct i2c_msg *msgs;   /* i2c_msg[] pointer */
    int nmsgs;              /* i2c_msg Nums */
};
int fd = 0;
/*
 * i2c_write, for configure PCA6416 register.
 */
int i2c_write(unsigned char slave, unsigned char reg, unsigned char value)
{
    int ret;
    struct i2c_rdwr_ioctl_data ssm_msg = {0};
    unsigned char buf[2] = {0};
    ssm_msg.nmsgs = 1;
    ssm_msg.msgs = (struct i2c_msg *)malloc(ssm_msg.nmsgs * sizeof(struct i2c_msg));
    if (ssm_msg.msgs == NULL) {
        printf("Memory alloc error!\n");
        return -1;
    }
    buf[0] = reg;
    buf[1] = value;
    (ssm_msg.msgs[0]).flags = 0;
    (ssm_msg.msgs[0]).addr = (unsigned short)slave;
    (ssm_msg.msgs[0]).buf = buf;
    (ssm_msg.msgs[0]).len = 2;
    ret = ioctl(fd, I2C_RDWR, &ssm_msg);
    if (ret < 0) {
        printf("write error, ret=%#x, errorno=%#x, %s!\n", ret, errno, strerror(errno));
        free(ssm_msg.msgs);
        ssm_msg.msgs = NULL;
        return -1;
    }
free(ssm_msg.msgs);
    ssm_msg.msgs = NULL;
    return 0;
}
/*
 * i2c_read, for reading PCA6416 register.
 */
int i2c_read(unsigned char slave, unsigned char reg, unsigned char *buf)
{
    int ret;
    struct i2c_rdwr_ioctl_data ssm_msg = {0};
    unsigned char regs[2] = {0};
    regs[0] = reg;
    regs[1] = reg;
    ssm_msg.nmsgs = 2;
    ssm_msg.msgs = (struct i2c_msg *)malloc(ssm_msg.nmsgs * sizeof(struct i2c_msg));
    if (ssm_msg.msgs == NULL) {
        printf("Memory alloc error!\n");
        return -1;
    }
    (ssm_msg.msgs[0]).flags = 0;
    (ssm_msg.msgs[0]).addr = slave;
    (ssm_msg.msgs[0]).buf = regs;
    (ssm_msg.msgs[0]).len = 1;
    (ssm_msg.msgs[1]).flags = I2C_M_RD;
    (ssm_msg.msgs[1]).addr = slave;
    (ssm_msg.msgs[1]).buf = buf;
    (ssm_msg.msgs[1]).len = 1;
    ret = ioctl(fd, I2C_RDWR, &ssm_msg);
    if (ret < 0) {
        printf("read data error,ret=%#x !\n", ret);
        free(ssm_msg.msgs);
        ssm_msg.msgs = NULL;
        return -1;
    }
    free(ssm_msg.msgs);
    ssm_msg.msgs = NULL;
    return 0;
}
/*
 * i2c_init, for access i2c device.
 */
int i2c_init(char *i2cdev_name)
{
    // open i2c-1 device
    fd = open(i2cdev_name, O_RDWR);
    if (fd < 0) {
        printf("Can't open %s!\n", i2cdev_name);
        return -1;
    }
    // set i2c-1 retries time
    if (ioctl(fd, I2C_RETRIES, 1) < 0) {
        close(fd);
        fd = 0;
        printf("set i2c retry fail!\n");
        return -1;
    }
    // set i2c-1 timeout time, 10ms as unit
    if (ioctl(fd, I2C_TIMEOUT, 1) < 0) {
        close(fd);
        fd = 0;
        printf("set i2c timeout fail!\n");
        return -1;
    }
    return 0;
}
int main(int argc, char *argv[])
{
    char *dev_name = I2C2_DEV_NAME;
    unsigned char slave;
    unsigned char reg;
    unsigned char data;
    int ret;
    if (i2c_init(dev_name)) {
        printf("i2c init fail!\n");
        return -1;
    }
    usleep(100);
    // set GPIO3 as output
    slave = PCA6416_SLAVE_ADDR;
    reg   = PCA6416_GPIO_CFG_REG;
    data  = 0;
    ret = i2c_read(slave, reg, &data);
    if (ret != 0) {
        close(fd);
        fd = 0;
        printf("read %s %#x %#x to %#x fail!\n", dev_name, slave, data, reg);
        return -1;
    }
    printf("read %#x \n", data);
    
    slave = PCA6416_SLAVE_ADDR;
    reg   = PCA6416_GPIO_CFG_REG;
    data  &= ~0x01;
    ret = i2c_write(slave, reg, data);
    if (ret != 0) {
        close(fd);
        fd = 0;
        printf("write %s %#x %#x to %#x fail!\n", dev_name, slave, data, reg);
        return -1;
    }

    // Set GPIO3 output high level
    slave = PCA6416_SLAVE_ADDR;
    reg   = PCA6416_GPIO_OUT_REG;
    data  = 0;
    ret = i2c_read(slave, reg, &data);
    if (ret != 0) {
        close(fd);
        fd = 0;
        printf("read %s %#x %#x to %#x fail!\n", dev_name, slave, data, reg);
        return -1;
    }
    printf("read %#x \n", data);
    slave = PCA6416_SLAVE_ADDR;
    reg   = PCA6416_GPIO_OUT_REG;
    data  |= 0x01;
    ret = i2c_write(slave, reg, data);
    if (ret != 0) {
        close(fd);
        fd = 0;
        printf("write %s %#x %#x to %#x fail!\n", dev_name, slave, data, reg);
        return -1;
    }
    close(fd);
    fd = 0;
    return 0;
}