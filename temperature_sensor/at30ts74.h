/**
* @file at30ts74.h
* @brief This header file provides the interface to access the temperature sensor from Microchip at30ts74.
*
* @author Jimmy Chea
* @date 30/03/2023
*/

#ifndef __AT30TS74_H__
#define __AT30TS74_H__

#ifdef __cplusplus
extern "C" {
#endif
                                                       //         A2  A1  A0
#define AT30TS74_DEFAULT_ADDRESS            (0x48)     // 1001     0   0   0
#define AT30TS74_ADDRESS_A2A1A0_010         (0x4A)     // 1001     0   1   0

#define AT30TS74_CONVERSIONDELAY            (100)      // in milli seconds

#define AT30TS74_REG_POINTER_MASK           (0x03)
#define AT30TS74_REG_POINTER_TEMP           (0x00)
#define AT30TS74_REG_POINTER_CONFIG         (0x01)
#define AT30TS74_REG_POINTER_TLOW           (0x02)
#define AT30TS74_REG_POINTER_THIGH          (0x03)

#define AT30TS74_REG_CONFIG_OS_MASK         (0x8000)    // One-Shot Mode
#define AT30TS74_REG_CONFIG_OS_ENABLED      (0x0000)    // Normal Operation (Default)
#define AT30TS74_REG_CONFIG_OS_DISABLED     (0x8000)    // Perform One-Shot Measurement

#define AT30TS74_REG_CONFIG_RES_MASK        (0x6000)    // Conversion Resolution
#define AT30TS74_REG_CONFIG_RES_9BIT        (0x0000)    // 9 bits (Default)
#define AT30TS74_REG_CONFIG_RES_10BIT       (0x2000)    // 10 bits
#define AT30TS74_REG_CONFIG_RES_11BIT       (0x4000)    // 11 bits
#define AT30TS74_REG_CONFIG_RES_12BIT       (0x6000)    // 12 bits

#define AT30TS74_REG_CONFIG_FAULTQUE_MASK   (0x1800)    // Fault Tolerance Queue
#define AT30TS74_REG_CONFIG_FAULTQUE_1      (0x0000)    // Alarm after 1 Fault (Default)
#define AT30TS74_REG_CONFIG_FAULTQUE_2      (0x0800)    // Alarm after 2 Consecutive Faults
#define AT30TS74_REG_CONFIG_FAULTQUE_4      (0x1000)    // Alarm after 4 Consecutive Faults
#define AT30TS74_REG_CONFIG_FAULTQUE_6      (0x1800)    // Alarm after 6 Consecutive Faults

#define AT30TS74_REG_CONFIG_ALERPOL_MASK    (0x0400)    // ALERT Pin Polarity
#define AT30TS74_REG_CONFIG_ALERPOL_HIGH    (0x0000)    // ALERT Pin is Active Low (Default)
#define AT30TS74_REG_CONFIG_ALERPOL_LOW     (0x0400)    // ALERT Pin is Active High

#define AT30TS74_REG_CONFIG_CMPINT_MASK     (0x0200)    // Alarm Thermostat Mode
#define AT30TS74_REG_CONFIG_CMPINT_INTR     (0x0000)    // Comparator Mode (Default)
#define AT30TS74_REG_CONFIG_CMPINT_COMP     (0x0200)    // Interrupt Mode

#define AT30TS74_REG_CONFIG_SD_MASK         (0x0100)    // Shutdown Mode
#define AT30TS74_REG_CONFIG_SD_ENABLE       (0x0000)    // Temperature Sensor performing Active Measurements
#define AT30TS74_REG_CONFIG_SD_DISABLE      (0x0100)    // Temperature Sensor Disabled and Device is ShutDown Mode

typedef enum
{
    ONESHOT_ENABLED     = AT30TS74_REG_CONFIG_OS_ENABLED,
    ONESHOT_DISABLED    = AT30TS74_REG_CONFIG_OS_DISABLED
} atOneShot_t;

typedef enum
{
    RESOLUTION_9_BITS   = AT30TS74_REG_CONFIG_RES_9BIT,
    RESOLUTION_10_BITS  = AT30TS74_REG_CONFIG_RES_10BIT,
    RESOLUTION_11_BITS  = AT30TS74_REG_CONFIG_RES_11BIT,
    RESOLUTION_12_BITS  = AT30TS74_REG_CONFIG_RES_12BIT
} atResolution_t;

typedef enum
{
    FAULT_QUEUE_1       = AT30TS74_REG_CONFIG_FAULTQUE_1,
    FAULT_QUEUE_2       = AT30TS74_REG_CONFIG_FAULTQUE_2,
    FAULT_QUEUE_4       = AT30TS74_REG_CONFIG_FAULTQUE_4,
    FAULT_QUEUE_6       = AT30TS74_REG_CONFIG_FAULTQUE_6
} atFaultQueue_t;

typedef enum
{
    ALERT_POLARITY_LOW  = AT30TS74_REG_CONFIG_ALERPOL_LOW,
    ALERT_POLARITY_HIGH = AT30TS74_REG_CONFIG_ALERPOL_HIGH
} atAlertPolarity_t;

typedef enum
{
    COMP_MODE           = AT30TS74_REG_CONFIG_CMPINT_COMP,
    INTR_MODE           = AT30TS74_REG_CONFIG_CMPINT_INTR
} atMode_t;

typedef enum
{
    SHUT_DOWN_ENABLE    = AT30TS74_REG_CONFIG_SD_ENABLE,
    SHUT_DOWN_DISABLE   = AT30TS74_REG_CONFIG_SD_DISABLE
} atShutDown_t;


typedef struct {
    int fd;
    const char *dev_name;
    uint8_t slave_addr;
} at30ts74_t;

int at30ts74_init(at30ts74_t *self, const char *device_name, uint8_t addr);
void at30ts74_close(at30ts74_t *self);
void at30ts74_set_oneshot(at30ts74_t *self, atOneShot_t oneshot);
uint16_t at30ts74_get_oneshot(at30ts74_t *self);
void at30ts74_set_resolution(at30ts74_t *self, atResolution_t resolution);
uint16_t at30ts74_get_resolution(at30ts74_t *self);
void at30ts74_set_fault_queue(at30ts74_t *self, atFaultQueue_t faultQueue);
uint16_t at30ts74_get_fault_queue(at30ts74_t *self);
void at30ts74_set_alert_polarity(at30ts74_t *self, atAlertPolarity_t alertPolarity);
uint16_t at30ts74_get_alert_polarity(at30ts74_t *self);
void at30ts74_set_mode(at30ts74_t *self, atMode_t mode);
uint16_t at30ts74_get_mode(at30ts74_t *self);
void at30ts74_set_shutdown(at30ts74_t *self, atShutDown_t shudown);
uint16_t at30ts74_get_shoutdown(at30ts74_t *self);
int16_t at30ts74_get_raw_temperature(at30ts74_t *self);
double at30ts74_get_temperature_celsius(at30ts74_t *self);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __AT30TS74_H__ */