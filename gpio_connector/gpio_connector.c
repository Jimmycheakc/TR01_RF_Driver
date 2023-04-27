/**
* @file gpio_connector.c
* @brief This implementation file contains the code for gpio connector interface.
*
* @author Jimmy Chea
* @date 30/03/2023
*/

#include <stdio.h>
#include <stdint.h>
#include "../io_utils/io_utils.h"
#include "gpiod.h"

#define DEVICE_CHIP_NAME    "gpiochip5"
#define OFFSET_RF_1V2_EN    8
#define OFFSET_RST_RF_B     9
#define OFFSET_RBE          10

int gpio_rst_rf_b_enabled(void)
{
    return io_utils_write_gpio(DEVICE_CHIP_NAME, OFFSET_RST_RF_B, 1, false, "gpio_rst_rf_b", GPIOD_CTXLESS_FLAG_BIAS_DISABLE);
}

int gpio_rst_rf_b_disabled(void)
{
    return io_utils_write_gpio(DEVICE_CHIP_NAME, OFFSET_RST_RF_B, 0, false, "gpio_rst_rf_b", GPIOD_CTXLESS_FLAG_BIAS_DISABLE);
}

int gpio_rbe_enabled(void)
{
    return io_utils_write_gpio(DEVICE_CHIP_NAME, OFFSET_RBE, 1, false, "gpio_rbe", GPIOD_CTXLESS_FLAG_BIAS_DISABLE);
}

int gpio_rbe_disabled(void)
{
    return io_utils_write_gpio(DEVICE_CHIP_NAME, OFFSET_RBE, 0, false, "gpio_rbe", GPIOD_CTXLESS_FLAG_BIAS_DISABLE);
}

int gpio_rf_1v2_en_enabled(void)
{
    return io_utils_write_gpio(DEVICE_CHIP_NAME, OFFSET_RF_1V2_EN, 1, false, "gpio_rf_1v2_en", GPIOD_CTXLESS_FLAG_BIAS_DISABLE);
}

int gpio_rf_1v2_en_disabled(void)
{
    return io_utils_write_gpio(DEVICE_CHIP_NAME, OFFSET_RF_1V2_EN, 0, false, "gpio_rf_1v2_en", GPIOD_CTXLESS_FLAG_BIAS_DISABLE);
}

