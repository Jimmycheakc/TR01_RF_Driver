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

#define I2C_GPIO_ADDRESS         0x2E

#define LNA_EN_BIT_MASK          0x00

#define PA_EN_BIT_MASK           0x01

#define CNTL1_BIT_MASK           0x02
#define CNTL2_BIT_MASK           0x03
#define CNTL3_BIT_MASK           0x04
#define CNTL4_BIT_MASK           0x05
#define CNTL5_BIT_MASK           0x06
#define CNTL6_BIT_MASK           0x07

#define MIXER1_MODE_BIT_MASK     0x08

#define MIXER2_MODE_BIT_MASK     0x09

#define SWITCH_V1_1_BIT_MASK     0x0A
#define SWITCH_V2_1_BIT_MASK     0x0B

#define SWITCH_V1_2_BIT_MASK     0x0C
#define SWITCH_V2_2_BIT_MASK     0x0D


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

int peri_gpio_mixer1_mode(peri_gpio_t *self, uint8_t value);

int peri_gpio_mixer2_mode(peri_gpio_t *self, uint8_t value);

int peri_gpio_switch_v1_1(peri_gpio_t *self, uint8_t value);

int peri_gpio_switch_v2_1(peri_gpio_t *self, uint8_t value);

int peri_gpio_switch_v1_2(peri_gpio_t *self, uint8_t value);

int peri_gpio_switch_v2_2(peri_gpio_t *self, uint8_t value);


#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __PERIPHERAL_GPIO_H__ */