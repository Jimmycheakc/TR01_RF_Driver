/**
* @file gpio_connector.h
* @brief This header file provides the gpio connector interfaces.
*
* @author Jimmy Chea
* @date 30/03/2023
*/

#ifndef __GPIO_CONNECTOR_H__
#define __GPIO_CONNECTOR_H__

#ifdef __cplusplus
extern "C" {
#endif

int gpio_rst_rf_b_enabled(void);
int gpio_rst_rf_b_disabled(void);
int gpio_rbe_enabled(void);
int gpio_rbe_disabled(void);
int gpio_rf_1v2_en_enabled(void);
int gpio_rf_1v2_en_disabled(void);

#ifdef __cplusplus
}
#endif  /* __cplusplus */

#endif  /* __GPIO_CONNECTOR_H__ */