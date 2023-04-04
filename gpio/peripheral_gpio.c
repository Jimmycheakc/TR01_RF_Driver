/**
* @file peripheral_gpio.c
* @brief This implementation file contains the code for peripheral gpio interface.
*
* @author Jimmy Chea
* @date 30/03/2023
*/

/**
 *  A detailed description of the gpio_lna_enable function
 *
 *  This function is to enable/disable gpio lna.
 *
 *  @param fd - file descriptor
 *  @param value - 0(disable) or 1(enable)
 *  @return - integer value on success, else return -1 on error
 */
int gpio_lna_enable(int fd, uint8_t value)
{
    return i2c_write(fd, I2C_GPIO_ADDRESS, REG_LNA_EN, value);
}

/**
 *  A detailed description of the gpio_pa_enable function
 *
 *  This function is to enable/disable gpio pa.
 *
 *  @param fd - file descriptor
 *  @param value - 0(disable) or 1(enable)
 *  @return - integer value on success, else return -1 on error
 */
int gpio_pa_enable(int fd, uint8_t value)
{
    return i2c_write(fd, I2C_GPIO_ADDRESS, REG_PA_EN, value);
}

/**
 *  A detailed description of the gpio_cntl1 function
 *
 *  This function is to enable/disable gpio cntl1.
 *
 *  @param fd - file descriptor
 *  @param value - 0(disable) or 1(enable)
 *  @return - integer value on success, else return -1 on error
 */
int gpio_cntl1(int fd, uint8_t value)
{
    return i2c_write(fd, I2C_GPIO_ADDRESS, REG_CNTL1, value);
}

/**
 *  A detailed description of the gpio_cntl2 function
 *
 *  This function is to enable/disable gpio cntl2.
 *
 *  @param fd - file descriptor
 *  @param value - 0(disable) or 1(enable)
 *  @return - integer value on success, else return -1 on error
 */
int gpio_cntl2(int fd, uint8_t value)
{
    return i2c_write(fd, I2C_GPIO_ADDRESS, REG_CNTL2, value);
}

/**
 *  A detailed description of the gpio_cntl3 function
 *
 *  This function is to enable/disable gpio cntl3.
 *
 *  @param fd - file descriptor
 *  @param value - 0(disable) or 1(enable)
 *  @return - integer value on success, else return -1 on error
 */
int gpio_cntl3(int fd, uint8_t value)
{
    return i2c_write(fd, I2C_GPIO_ADDRESS, REG_CNTL3, value);
}

/**
 *  A detailed description of the gpio_cntl4 function
 *
 *  This function is to enable/disable gpio cntl4.
 *
 *  @param fd - file descriptor
 *  @param value - 0(disable) or 1(enable)
 *  @return - integer value on success, else return -1 on error
 */
int gpio_cntl4(int fd, uint8_t value)
{
    return i2c_write(fd, I2C_GPIO_ADDRESS, REG_CNTL4, value);
}

/**
 *  A detailed description of the gpio_cntl5 function
 *
 *  This function is to enable/disable gpio cntl5.
 *
 *  @param fd - file descriptor
 *  @param value - 0(disable) or 1(enable)
 *  @return - integer value on success, else return -1 on error
 */
int gpio_cntl5(int fd, uint8_t value)
{
    return i2c_write(fd, I2C_GPIO_ADDRESS, REG_CNTL5, value);
}

/**
 *  A detailed description of the gpio_cntl6 function
 *
 *  This function is to enable/disable gpio cntl6.
 *
 *  @param fd - file descriptor
 *  @param value - 0(disable) or 1(enable)
 *  @return - integer value on success, else return -1 on error
 */
int gpio_cntl6(int fd, uint8_t value)
{
    return i2c_write(fd, I2C_GPIO_ADDRESS, REG_CNTL6, value);
}

/**
 *  A detailed description of the gpio_mixer1_reset function
 *
 *  This function is to enable/disable gpio for mixer 1 reset.
 *
 *  @param fd - file descriptor
 *  @param value - 0(disable) or 1(enable)
 *  @return - integer value on success, else return -1 on error
 */
int gpio_mixer1_reset(int fd, uint8_t value)
{
    return i2c_write(fd, I2C_GPIO_ADDRESS, REG_MIXER1_RESET, value);
}

/**
 *  A detailed description of the gpio_mixer1_enable function
 *
 *  This function is to enable/disable gpio for mixer 1 enable.
 *
 *  @param fd - file descriptor
 *  @param value - 0(disable) or 1(enable)
 *  @return - integer value on success, else return -1 on error
 */
int gpio_mixer1_enable(int fd, uint8_t value)
{
    return i2c_write(fd, I2C_GPIO_ADDRESS, REG_MIXER1_EN, value);
}

/**
 *  A detailed description of the gpio_mixer1_enable_bl function
 *
 *  This function is to enable/disable gpio for mixer 1 enable bl.
 *
 *  @param fd - file descriptor
 *  @param value - 0(disable) or 1(enable)
 *  @return - integer value on success, else return -1 on error
 */
int gpio_mixer1_enable_bl(int fd, uint8_t value)
{
    return i2c_write(fd, I2C_GPIO_ADDRESS, REG_MIXER1_ENBL, value);
}

/**
 *  A detailed description of the gpio_mixer1_mode function
 *
 *  This function is to enable/disable gpio for mixer 1 mode.
 *
 *  @param fd - file descriptor
 *  @param value - 0(disable) or 1(enable)
 *  @return - integer value on success, else return -1 on error
 */
int gpio_mixer1_mode(int fd, uint8_t value)
{
    return i2c_write(fd, I2C_GPIO_ADDRESS, REG_MIXER1_MODE, value);
}

/**
 *  A detailed description of the gpio_mixer2_reset function
 *
 *  This function is to enable/disable gpio for mixer 2 reset.
 *
 *  @param fd - file descriptor
 *  @param value - 0(disable) or 1(enable)
 *  @return - integer value on success, else return -1 on error
 */
int gpio_mixer2_reset(int fd, uint8_t value)
{
    return i2c_write(fd, I2C_GPIO_ADDRESS, REG_MIXER2_RESET, value);
}

/**
 *  A detailed description of the gpio_mixer2_enable function
 *
 *  This function is to enable/disable gpio for mixer 2 enable.
 *
 *  @param fd - file descriptor
 *  @param value - 0(disable) or 1(enable)
 *  @return - integer value on success, else return -1 on error
 */
int gpio_mixer2_enable(int fd, uint8_t value)
{
    return i2c_write(fd, I2C_GPIO_ADDRESS, REG_MIXER2_EN, value);
}

/**
 *  A detailed description of the gpio_mixer2_enable_bl function
 *
 *  This function is to enable/disable gpio for mixer 2 enable bl.
 *
 *  @param fd - file descriptor
 *  @param value - 0(disable) or 1(enable)
 *  @return - integer value on success, else return -1 on error
 */
int gpio_mixer2_enable_bl(int fd, uint8_t value)
{
    return i2c_write(fd, I2C_GPIO_ADDRESS, REG_MIXER2_ENBL, value);
}

/**
 *  A detailed description of the gpio_mixer2_mode function
 *
 *  This function is to enable/disable gpio for mixer 2 mode.
 *
 *  @param fd - file descriptor
 *  @param value - 0(disable) or 1(enable)
 *  @return - integer value on success, else return -1 on error
 */
int gpio_mixer2_mode(int fd, uint8_t value)
{
    return i2c_write(fd, I2C_GPIO_ADDRESS, REG_MIXER2_MODE, value);
}

/**
 *  A detailed description of the gpio_adc1_cs function
 *
 *  This function is to enable/disable gpio for adc 1 chip select.
 *
 *  @param fd - file descriptor
 *  @param value - 0(disable) or 1(enable)
 *  @return - integer value on success, else return -1 on error
 */
int gpio_adc1_cs(int fd, uint8_t value)
{
    return i2c_write(fd, I2C_GPIO_ADDRESS, REG_ADC1_CS, value);
}

/**
 *  A detailed description of the gpio_adc2_cs function
 *
 *  This function is to enable/disable gpio for adc 2 chip select.
 *
 *  @param fd - file descriptor
 *  @param value - 0(disable) or 1(enable)
 *  @return - integer value on success, else return -1 on error
 */
int gpio_adc2_cs(int fd, uint8_t value)
{
    return i2c_write(fd, I2C_GPIO_ADDRESS, REG_ADC2_CS, value);
}

/**
 *  A detailed description of the gpio_rfic1_seln function
 *
 *  This function is to enable/disable gpio for rfic 1 select enable.
 *
 *  @param fd - file descriptor
 *  @param value - 0(disable) or 1(enable)
 *  @return - integer value on success, else return -1 on error
 */
int gpio_rfic1_seln(int fd, uint8_t value)
{
    return i2c_write(fd, I2C_GPIO_ADDRESS, REG_RFIC1_SELN, value);
}

/**
 *  A detailed description of the gpio_rfic2_seln function
 *
 *  This function is to enable/disable gpio for rfic 2 select enable.
 *
 *  @param fd - file descriptor
 *  @param value - 0(disable) or 1(enable)
 *  @return - integer value on success, else return -1 on error
 */
int gpio_rfic2_seln(int fd, uint8_t value)
{
    return i2c_write(fd, I2C_GPIO_ADDRESS, REG_RFIC2_SELN, value);
}

/**
 *  A detailed description of the gpio_rfic3_seln function
 *
 *  This function is to enable/disable gpio for rfic 3 select enable.
 *
 *  @param fd - file descriptor
 *  @param value - 0(disable) or 1(enable)
 *  @return - integer value on success, else return -1 on error
 */
int gpio_rfic3_seln(int fd, uint8_t value)
{
    return i2c_write(fd, I2C_GPIO_ADDRESS, REG_RFIC3_SELN, value);
}

/**
 *  A detailed description of the gpio_rfic4_seln function
 *
 *  This function is to enable/disable gpio for rfic 4 select enable.
 *
 *  @param fd - file descriptor
 *  @param value - 0(disable) or 1(enable)
 *  @return - integer value on success, else return -1 on error
 */
int gpio_rfic4_seln(int fd, uint8_t value)
{
    return i2c_write(fd, I2C_GPIO_ADDRESS, REG_RFIC4_SELN, value);
}

/**
 *  A detailed description of the gpio_switch_v1_1 function
 *
 *  This function is to enable/disable gpio for switch v1_1 enable.
 *
 *  @param fd - file descriptor
 *  @param value - 0(disable) or 1(enable)
 *  @return - integer value on success, else return -1 on error
 */
int gpio_switch_v1_1(int fd, uint8_t value)
{
    return i2c_write(fd, I2C_GPIO_ADDRESS, REG_SWITCH_V1_1, value);
}

/**
 *  A detailed description of the gpio_switch_v2_1 function
 *
 *  This function is to enable/disable gpio for switch v2_1 enable.
 *
 *  @param fd - file descriptor
 *  @param value - 0(disable) or 1(enable)
 *  @return - integer value on success, else return -1 on error
 */
int gpio_switch_v2_1(int fd, uint8_t value)
{
    return i2c_write(fd, I2C_GPIO_ADDRESS, REG_SWITCH_V2_1, value);
}
