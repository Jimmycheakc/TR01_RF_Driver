/**
* @file peripheral_gpio.c
* @brief This implementation file contains the code for peripheral gpio interface.
*
* @author Jimmy Chea
* @date 30/03/2023
*/

#include "../i2c/i2c.h"
#include "peripheral_gpio.h"

/**
 *  A detailed description of the peri_gpio_init function
 *
 *  This function is to initialize the peripheral gpio on RF card
 *
 *  @param \*self - pointer to the structure of peri_gpio_t
 *  @param \*device_name - input the device name "/dev/i2c-*"
 *  @param addr - i2c-gpio converter slave address
 *  @return - none
 */
void peri_gpio_init(peri_gpio_t *self, const char *device_name, uint8_t addr)
{
    self->fd = i2c_open(device_name);
    self->dev_name = device_name;
    self->slave_addr = addr;
}

/**
 *  A detailed description of the peri_gpio_close function
 *
 *  This function is to close the peripheral gpio.
 *
 *  @param \*self - pointer to the structure of peri_gpio_t
 *  @return - none
 */
void peri_gpio_close(peri_gpio_t *self)
{
    i2c_close(self->fd);
    self->dev_name = NULL;
    self->slave_addr = 0;
}

/**
 *  A detailed description of the peri_gpio_lna_enable function
 *
 *  This function is to enable/disable gpio lna.
 *
 *  @param \*self - pointer to the structure of peri_gpio_t
 *  @param value - 0(disable) or 1(enable)
 *  @return - integer value on success, else return -1 on error
 */
int peri_gpio_lna_enable(peri_gpio_t *self, uint8_t value)
{
    uint16_t data = value << LNA_EN_BIT_MASK;
    uint8_t msb = (data >> 8) & 0xFF;
    uint8_t lsb = data & 0xFF;

    return i2c_write(self->fd, self->slave_addr, msb, lsb);
}

/**
 *  A detailed description of the peri_gpio_pa_enable function
 *
 *  This function is to enable/disable gpio pa.
 *
 *  @param \*self - pointer to the structure of peri_gpio_t
 *  @param value - 0(disable) or 1(enable)
 *  @return - integer value on success, else return -1 on error
 */
int peri_gpio_pa_enable(peri_gpio_t *self, uint8_t value)
{
    uint16_t data = value << PA_EN_BIT_MASK;
    uint8_t msb = (data >> 8) & 0xFF;
    uint8_t lsb = data & 0xFF;

    return i2c_write(self->fd, self->slave_addr, msb, lsb);
}

/**
 *  A detailed description of the peri_gpio_cntl1 function
 *
 *  This function is to enable/disable gpio cntl1.
 *
 *  @param \*self - pointer to the structure of peri_gpio_t
 *  @param value - 0(disable) or 1(enable)
 *  @return - integer value on success, else return -1 on error
 */
int peri_gpio_cntl1(peri_gpio_t *self, uint8_t value)
{
    uint16_t data = value << CNTL1_BIT_MASK;
    uint8_t msb = (data >> 8) & 0xFF;
    uint8_t lsb = data & 0xFF;

    return i2c_write(self->fd, self->slave_addr, msb, lsb);
}

/**
 *  A detailed description of the peri_gpio_cntl2 function
 *
 *  This function is to enable/disable gpio cntl2.
 *
 *  @param \*self - pointer to the structure of peri_gpio_t
 *  @param value - 0(disable) or 1(enable)
 *  @return - integer value on success, else return -1 on error
 */
int peri_gpio_cntl2(peri_gpio_t *self, uint8_t value)
{
    uint16_t data = value << CNTL2_BIT_MASK;
    uint8_t msb = (data >> 8) & 0xFF;
    uint8_t lsb = data & 0xFF;

    return i2c_write(self->fd, self->slave_addr, msb, lsb);
}

/**
 *  A detailed description of the peri_gpio_cntl3 function
 *
 *  This function is to enable/disable gpio cntl3.
 *
 *  @param \*self - pointer to the structure of peri_gpio_t
 *  @param value - 0(disable) or 1(enable)
 *  @return - integer value on success, else return -1 on error
 */
int peri_gpio_cntl3(peri_gpio_t *self, uint8_t value)
{
    uint16_t data = value << CNTL3_BIT_MASK;
    uint8_t msb = (data >> 8) & 0xFF;
    uint8_t lsb = data & 0xFF;

    return i2c_write(self->fd, self->slave_addr, msb, lsb);
}

/**
 *  A detailed description of the peri_gpio_cntl4 function
 *
 *  This function is to enable/disable gpio cntl4.
 *
 *  @param \*self - pointer to the structure of peri_gpio_t
 *  @param value - 0(disable) or 1(enable)
 *  @return - integer value on success, else return -1 on error
 */
int peri_gpio_cntl4(peri_gpio_t *self, uint8_t value)
{
    uint16_t data = value << CNTL4_BIT_MASK;
    uint8_t msb = (data >> 8) & 0xFF;
    uint8_t lsb = data & 0xFF;

    return i2c_write(self->fd, self->slave_addr, msb, lsb);
}

/**
 *  A detailed description of the peri_gpio_cntl5 function
 *
 *  This function is to enable/disable gpio cntl5.
 *
 *  @param \*self - pointer to the structure of peri_gpio_t
 *  @param value - 0(disable) or 1(enable)
 *  @return - integer value on success, else return -1 on error
 */
int peri_gpio_cntl5(peri_gpio_t *self, uint8_t value)
{
    uint16_t data = value << CNTL5_BIT_MASK;
    uint8_t msb = (data >> 8) & 0xFF;
    uint8_t lsb = data & 0xFF;

    return i2c_write(self->fd, self->slave_addr, msb, lsb);
}

/**
 *  A detailed description of the peri_gpio_cntl6 function
 *
 *  This function is to enable/disable gpio cntl6.
 *
 *  @param \*self - pointer to the structure of peri_gpio_t
 *  @param value - 0(disable) or 1(enable)
 *  @return - integer value on success, else return -1 on error
 */
int peri_gpio_cntl6(peri_gpio_t *self, uint8_t value)
{
    uint16_t data = value << CNTL6_BIT_MASK;
    uint8_t msb = (data >> 8) & 0xFF;
    uint8_t lsb = data & 0xFF;

    return i2c_write(self->fd, self->slave_addr, msb, lsb);
}

/**
 *  A detailed description of the peri_gpio_mixer1_mode function
 *
 *  This function is to enable/disable gpio for mixer 1 mode.
 *
 *  @param \*self - pointer to the structure of peri_gpio_t
 *  @param value - 0(disable) or 1(enable)
 *  @return - integer value on success, else return -1 on error
 */
int peri_gpio_mixer1_mode(peri_gpio_t *self, uint8_t value)
{
    uint16_t data = value << MIXER1_MODE_BIT_MASK;
    uint8_t msb = (data >> 8) & 0xFF;
    uint8_t lsb = data & 0xFF;

    return i2c_write(self->fd, self->slave_addr, msb, lsb);
}

/**
 *  A detailed description of the peri_gpio_mixer2_mode function
 *
 *  This function is to enable/disable gpio for mixer 2 mode.
 *
 *  @param \*self - pointer to the structure of peri_gpio_t
 *  @param value - 0(disable) or 1(enable)
 *  @return - integer value on success, else return -1 on error
 */
int peri_gpio_mixer2_mode(peri_gpio_t *self, uint8_t value)
{
    uint16_t data = value << MIXER2_MODE_BIT_MASK;
    uint8_t msb = (data >> 8) & 0xFF;
    uint8_t lsb = data & 0xFF;

    return i2c_write(self->fd, self->slave_addr, msb, lsb);
}

/**
 *  A detailed description of the peri_gpio_switch_v1_1 function
 *
 *  This function is to enable/disable gpio for switch v1_1 enable.
 *
 *  @param \*self - pointer to the structure of peri_gpio_t
 *  @param value - 0(disable) or 1(enable)
 *  @return - integer value on success, else return -1 on error
 */
int peri_gpio_switch_v1_1(peri_gpio_t *self, uint8_t value)
{
    uint16_t data = value << SWITCH_V1_1_BIT_MASK;
    uint8_t msb = (data >> 8) & 0xFF;
    uint8_t lsb = data & 0xFF;

    return i2c_write(self->fd, self->slave_addr, msb, lsb);
}

/**
 *  A detailed description of the peri_gpio_switch_v2_1 function
 *
 *  This function is to enable/disable gpio for switch v2_1 enable.
 *
 *  @param \*self - pointer to the structure of peri_gpio_t
 *  @param value - 0(disable) or 1(enable)
 *  @return - integer value on success, else return -1 on error
 */
int peri_gpio_switch_v2_1(peri_gpio_t *self, uint8_t value)
{
    uint16_t data = value << SWITCH_V2_1_BIT_MASK;
    uint8_t msb = (data >> 8) & 0xFF;
    uint8_t lsb = data & 0xFF;

    return i2c_write(self->fd, self->slave_addr, msb, lsb);
}

/**
 *  A detailed description of the peri_gpio_switch_v1_2 function
 *
 *  This function is to enable/disable gpio for switch v1_2 enable.
 *
 *  @param \*self - pointer to the structure of peri_gpio_t
 *  @param value - 0(disable) or 1(enable)
 *  @return - integer value on success, else return -1 on error
 */
int peri_gpio_switch_v1_2(peri_gpio_t *self, uint8_t value)
{
    uint16_t data = value << SWITCH_V1_2_BIT_MASK;
    uint8_t msb = (data >> 8) & 0xFF;
    uint8_t lsb = data & 0xFF;

    return i2c_write(self->fd, self->slave_addr, msb, lsb);
}

/**
 *  A detailed description of the peri_gpio_switch_v2_2 function
 *
 *  This function is to enable/disable gpio for switch v2_2 enable.
 *
 *  @param \*self - pointer to the structure of peri_gpio_t
 *  @param value - 0(disable) or 1(enable)
 *  @return - integer value on success, else return -1 on error
 */
int peri_gpio_switch_v2_2(peri_gpio_t *self, uint8_t value)
{
    uint16_t data = value << SWITCH_V2_2_BIT_MASK;
    uint8_t msb = (data >> 8) & 0xFF;
    uint8_t lsb = data & 0xFF;

    return i2c_write(self->fd, self->slave_addr, msb, lsb);
}
