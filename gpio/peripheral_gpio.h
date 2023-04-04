/**
* @file peripheral_gpio.h
* @brief This header file provides the peripheral gpio interfaces.
*
* @author Jimmy Chea
* @date 30/03/2023
*/

#ifndef __PERIPHERAL_GPIO_H__
#define __PERIPHERAL_GPIO_H__

#ifdef __cplusplus
extern "C" {
#endif


// Define the I2C-to-GPIO converter address
#define I2C_GPIO_ADDRESS 0x40

#define REG_LNA_EN          0x00

#define REG_PA_EN           0x00

#define REG_CNTL1           0x00
#define REG_CNTL2           0x00
#define REG_CNTL3           0x00
#define REG_CNTL4           0x00
#define REG_CNTL5           0x00
#define REG_CNTL6           0x00

#define REG_MIXER1_RESET    0x00
#define REG_MIXER1_EN       0x00
#define REG_MIXER1_ENBL     0x00
#define REG_MIXER1_MODE     0x00

#define REG_MIXER2_RESET    0x00
#define REG_MIXER2_EN       0x00
#define REG_MIXER2_ENBL     0x00
#define REG_MIXER2_MODE     0x00

#define REG_ADC1_CS         0x00
#define REG_ADC2_CS         0x00

#define REG_RFIC1_SELN      0x00
#define REG_RFIC2_SELN      0x00
#define REG_RFIC3_SELN      0x00
#define REG_RFIC4_SELN      0x00

#define REG_SWITCH_V1_1     0x00
#define REG_SWITCH_V2_1     0x00


int gpio_lna_enable(int fd, uint8_t value);

int gpio_pa_enable(int fd, uint8_t value);

int gpio_cntl1(int fd, uint8_t value);

int gpio_cntl2(int fd, uint8_t value);

int gpio_cntl3(int fd, uint8_t value);

int gpio_cntl4(int fd, uint8_t value);

int gpio_cntl5(int fd, uint8_t value);

int gpio_cntl6(int fd, uint8_t value);

int gpio_mixer1_reset(int fd, uint8_t value);

int gpio_mixer1_enable(int fd, uint8_t value);

int gpio_mixer1_enable_bl(int fd, uint8_t value);

int gpio_mixer1_mode(int fd, uint8_t value);

int gpio_mixer2_reset(int fd, uint8_t value);

int gpio_mixer2_enable(int fd, uint8_t value);

int gpio_mixer2_enable_bl(int fd, uint8_t value);

int gpio_mixer2_mode(int fd, uint8_t value);

int gpio_adc1_cs(int fd, uint8_t value);

int gpio_adc2_cs(int fd, uint8_t value);

int gpio_rfic1_seln(int fd, uint8_t value);

int gpio_rfic2_seln(int fd, uint8_t value);

int gpio_rfic3_seln(int fd, uint8_t value);

int gpio_rfic4_seln(int fd, uint8_t value);

int gpio_switch_v1_1(int fd, uint8_t value);

int gpio_switch_v2_1(int fd, uint8_t value);


#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __PERIPHERAL_GPIO_H__ */