/**
* @file peripheral_gpio.c
* @brief This implementation file contains the code for peripheral gpio interface.
*
* @author Jimmy Chea
* @date 30/03/2023
*/

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
    return i2c_write(self->fd, self->slave_addr, REG_LNA_EN, value);
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
    return i2c_write(self->fd, self->slave_addr, REG_PA_EN, value);
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
    return i2c_write(self->fd, self->slave_addr, REG_CNTL1, value);
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
    return i2c_write(self->fd, self->slave_addr, REG_CNTL2, value);
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
    return i2c_write(self->fd, self->slave_addr, REG_CNTL3, value);
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
    return i2c_write(self->fd, self->slave_addr, REG_CNTL4, value);
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
    return i2c_write(self->fd, self->slave_addr, REG_CNTL5, value);
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
    return i2c_write(self->fd, self->slave_addr, REG_CNTL6, value);
}

/**
 *  A detailed description of the peri_gpio_mixer1_reset function
 *
 *  This function is to enable/disable gpio for mixer 1 reset.
 *
 *  @param \*self - pointer to the structure of peri_gpio_t
 *  @param value - 0(disable) or 1(enable)
 *  @return - integer value on success, else return -1 on error
 */
int peri_gpio_mixer1_reset(peri_gpio_t *self, uint8_t value)
{
    return i2c_write(self->fd, self->slave_addr, REG_MIXER1_RESET, value);
}

/**
 *  A detailed description of the peri_gpio_mixer1_enable function
 *
 *  This function is to enable/disable gpio for mixer 1 enable.
 *
 *  @param \*self - pointer to the structure of peri_gpio_t
 *  @param value - 0(disable) or 1(enable)
 *  @return - integer value on success, else return -1 on error
 */
int peri_gpio_mixer1_enable(peri_gpio_t *self, uint8_t value)
{
    return i2c_write(self->fd, self->slave_addr, REG_MIXER1_EN, value);
}

/**
 *  A detailed description of the peri_gpio_mixer1_enable_bl function
 *
 *  This function is to enable/disable gpio for mixer 1 enable bl.
 *
 *  @param \*self - pointer to the structure of peri_gpio_t
 *  @param value - 0(disable) or 1(enable)
 *  @return - integer value on success, else return -1 on error
 */
int peri_gpio_mixer1_enable_bl(peri_gpio_t *self, uint8_t value)
{
    return i2c_write(self->fd, self->slave_addr, REG_MIXER1_ENBL, value);
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
    return i2c_write(self->fd, self->slave_addr, REG_MIXER1_MODE, value);
}

/**
 *  A detailed description of the peri_gpio_mixer2_reset function
 *
 *  This function is to enable/disable gpio for mixer 2 reset.
 *
 *  @param \*self - pointer to the structure of peri_gpio_t
 *  @param value - 0(disable) or 1(enable)
 *  @return - integer value on success, else return -1 on error
 */
int peri_gpio_mixer2_reset(peri_gpio_t *self, uint8_t value)
{
    return i2c_write(self->fd, self->slave_addr, REG_MIXER2_RESET, value);
}

/**
 *  A detailed description of the peri_gpio_mixer2_enable function
 *
 *  This function is to enable/disable gpio for mixer 2 enable.
 *
 *  @param \*self - pointer to the structure of peri_gpio_t
 *  @param value - 0(disable) or 1(enable)
 *  @return - integer value on success, else return -1 on error
 */
int peri_gpio_mixer2_enable(peri_gpio_t *self, uint8_t value)
{
    return i2c_write(self->fd, self->slave_addr, REG_MIXER2_EN, value);
}

/**
 *  A detailed description of the peri_gpio_mixer2_enable_bl function
 *
 *  This function is to enable/disable gpio for mixer 2 enable bl.
 *
 *  @param \*self - pointer to the structure of peri_gpio_t
 *  @param value - 0(disable) or 1(enable)
 *  @return - integer value on success, else return -1 on error
 */
int peri_gpio_mixer2_enable_bl(peri_gpio_t *self, uint8_t value)
{
    return i2c_write(self->fd, self->slave_addr, REG_MIXER2_ENBL, value);
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
    return i2c_write(self->fd, self->slave_addr, REG_MIXER2_MODE, value);
}

/**
 *  A detailed description of the peri_gpio_adc1_cs function
 *
 *  This function is to enable/disable gpio for adc 1 chip select.
 *
 *  @param \*self - pointer to the structure of peri_gpio_t
 *  @param value - 0(disable) or 1(enable)
 *  @return - integer value on success, else return -1 on error
 */
int peri_gpio_adc1_cs(peri_gpio_t *self, uint8_t value)
{
    return i2c_write(self->fd, self->slave_addr, REG_ADC1_CS, value);
}

/**
 *  A detailed description of the peri_gpio_adc2_cs function
 *
 *  This function is to enable/disable gpio for adc 2 chip select.
 *
 *  @param \*self - pointer to the structure of peri_gpio_t
 *  @param value - 0(disable) or 1(enable)
 *  @return - integer value on success, else return -1 on error
 */
int peri_gpio_adc2_cs(peri_gpio_t *self, uint8_t value)
{
    return i2c_write(self->fd, self->slave_addr, REG_ADC2_CS, value);
}

/**
 *  A detailed description of the peri_gpio_rfic1_seln function
 *
 *  This function is to enable/disable gpio for rfic 1 select enable.
 *
 *  @param \*self - pointer to the structure of peri_gpio_t
 *  @param value - 0(disable) or 1(enable)
 *  @return - integer value on success, else return -1 on error
 */
int peri_gpio_rfic1_seln(peri_gpio_t *self, uint8_t value)
{
    return i2c_write(self->fd, self->slave_addr, REG_RFIC1_SELN, value);
}

/**
 *  A detailed description of the peri_gpio_rfic2_seln function
 *
 *  This function is to enable/disable gpio for rfic 2 select enable.
 *
 *  @param \*self - pointer to the structure of peri_gpio_t
 *  @param value - 0(disable) or 1(enable)
 *  @return - integer value on success, else return -1 on error
 */
int peri_gpio_rfic2_seln(peri_gpio_t *self, uint8_t value)
{
    return i2c_write(self->fd, self->slave_addr, REG_RFIC2_SELN, value);
}

/**
 *  A detailed description of the peri_gpio_rfic3_seln function
 *
 *  This function is to enable/disable gpio for rfic 3 select enable.
 *
 *  @param \*self - pointer to the structure of peri_gpio_t
 *  @param value - 0(disable) or 1(enable)
 *  @return - integer value on success, else return -1 on error
 */
int peri_gpio_rfic3_seln(peri_gpio_t *self, uint8_t value)
{
    return i2c_write(self->fd, self->slave_addr, REG_RFIC3_SELN, value);
}

/**
 *  A detailed description of the peri_gpio_rfic4_seln function
 *
 *  This function is to enable/disable gpio for rfic 4 select enable.
 *
 *  @param \*self - pointer to the structure of peri_gpio_t
 *  @param value - 0(disable) or 1(enable)
 *  @return - integer value on success, else return -1 on error
 */
int peri_gpio_rfic4_seln(peri_gpio_t *self, uint8_t value)
{
    return i2c_write(self->fd, self->slave_addr, REG_RFIC4_SELN, value);
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
    return i2c_write(self->fd, self->slave_addr, REG_SWITCH_V1_1, value);
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
    return i2c_write(self->fd, self->slave_addr, REG_SWITCH_V2_1, value);
}
