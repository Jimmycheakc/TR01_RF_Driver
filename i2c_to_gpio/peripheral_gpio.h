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

#include "stdio.h"
#include "stdint.h"

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


typedef struct {
    int fd;
    const char *dev_name;
    uint8_t slave_addr;
} peri_gpio_t;

void peri_gpio_init(peri_gpio_t *self, const char *device_name, uint8_t addr);

void peri_gpio_close(peri_gpio_t *self);

int peri_gpio_lna_enable(peri_gpio_t *self, uint8_t value);

int peri_gpio_pa_enable(peri_gpio_t *self, uint8_t value);

int peri_gpio_cntl1(peri_gpio_t *self, uint8_t value);

int peri_gpio_cntl2(peri_gpio_t *self, uint8_t value);

int peri_gpio_cntl3(peri_gpio_t *self, uint8_t value);

int peri_gpio_cntl4(peri_gpio_t *self, uint8_t value);

int peri_gpio_cntl5(peri_gpio_t *self, uint8_t value);

int peri_gpio_cntl6(peri_gpio_t *self, uint8_t value);

int peri_gpio_mixer1_reset(peri_gpio_t *self, uint8_t value);

int peri_gpio_mixer1_enable(peri_gpio_t *self, uint8_t value);

int peri_gpio_mixer1_enable_bl(peri_gpio_t *self, uint8_t value);

int peri_gpio_mixer1_mode(peri_gpio_t *self, uint8_t value);

int peri_gpio_mixer2_reset(peri_gpio_t *self, uint8_t value);

int peri_gpio_mixer2_enable(peri_gpio_t *self, uint8_t value);

int peri_gpio_mixer2_enable_bl(peri_gpio_t *self, uint8_t value);

int peri_gpio_mixer2_mode(peri_gpio_t *self, uint8_t value);

int peri_gpio_adc1_cs(peri_gpio_t *self, uint8_t value);

int peri_gpio_adc2_cs(peri_gpio_t *self, uint8_t value);

int peri_gpio_rfic1_seln(peri_gpio_t *self, uint8_t value);

int peri_gpio_rfic2_seln(peri_gpio_t *self, uint8_t value);

int peri_gpio_rfic3_seln(peri_gpio_t *self, uint8_t value);

int peri_gpio_rfic4_seln(peri_gpio_t *self, uint8_t value);

int peri_gpio_switch_v1_1(peri_gpio_t *self, uint8_t value);

int peri_gpio_switch_v2_1(peri_gpio_t *self, uint8_t value);


#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __PERIPHERAL_GPIO_H__ */