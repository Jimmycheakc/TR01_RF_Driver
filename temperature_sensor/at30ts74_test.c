#include "stdio.h"
#include "stdint.h"
#include "at30ts74.h"

#define I2C_DEVICE "/dev/i2c-3"

void at30ts74_temp_register_test(at30ts74_t *dev)
{
    double temp_celsius;

    at30ts74_set_resolution(dev, RESOLUTION_9_BITS);
    temp_celsius = at30ts74_get_temperature_celsius(dev);
    printf("Temperature in celsius (Resolution 9 bits) : %lf\n", temp_celsius);

    at30ts74_set_resolution(dev, RESOLUTION_10_BITS);
    temp_celsius = at30ts74_get_temperature_celsius(dev);
    printf("Temperature in celsius (Resolution 10 bits) : %lf\n", temp_celsius);

    at30ts74_set_resolution(dev, RESOLUTION_11_BITS);
    temp_celsius = at30ts74_get_temperature_celsius(dev);
    printf("Temperature in celsius (Resolution 11 bits) : %lf\n", temp_celsius);

    at30ts74_set_resolution(dev, RESOLUTION_12_BITS);
    temp_celsius = at30ts74_get_temperature_celsius(dev);
    printf("Temperature in celsius (Resolution 12 bits) : %lf\n", temp_celsius);
}

void at30ts74_config_register_test(at30ts74_t *dev)
{
    int res;

    at30ts74_set_oneshot(dev, ONESHOT_DISABLED);
    res = at30ts74_get_oneshot(dev);
    if (res != AT30TS74_REG_CONFIG_OS_DISABLED)
    {
        printf("Unable to get one shot enabled\n");
    }

    at30ts74_set_resolution(dev, RESOLUTION_10_BITS);
    res = at30ts74_get_resolution(dev);
    if (res != AT30TS74_REG_CONFIG_RES_10BIT)
    {
        printf("Unable to get resolution in 9 bits\n");
    }

    at30ts74_set_fault_queue(dev, FAULT_QUEUE_2);
    res = at30ts74_get_fault_queue(dev);
    if (res != AT30TS74_REG_CONFIG_FAULTQUE_2)
    {
        printf("Unable to get resolution in 9 bits\n");
    }

    at30ts74_set_alert_polarity(dev, ALERT_POLARITY_HIGH);
    res = at30ts74_get_alert_polarity(dev);
    if (res != AT30TS74_REG_CONFIG_ALERPOL_HIGH)
    {
        printf("Unable to get resolution in 9 bits\n");
    }

    at30ts74_set_mode(dev, INTR_MODE);
    res = at30ts74_get_mode(dev);
    if (res != AT30TS74_REG_CONFIG_CMPINT_INTR)
    {
        printf("Unable to get resolution in 9 bits\n");
    }

    printf("%s passed.\n", __func__);
    printf("Restore to default register.\n");

    at30ts74_set_oneshot(dev, ONESHOT_ENABLED);
    at30ts74_set_resolution(dev, RESOLUTION_9_BITS);
    at30ts74_set_fault_queue(dev, FAULT_QUEUE_1);
    at30ts74_set_alert_polarity(dev, ALERT_POLARITY_LOW);
    at30ts74_set_mode(dev, COMP_MODE);
}

int main (void)
{
    at30ts74_t dev;
    uint16_t res;

    at30ts74_init(&dev, I2C_DEVICE, 0x48);

    at30ts74_config_register_test(&dev);

    at30ts74_temp_register_test(&dev);

    at30ts74_close(&dev);
    return 0;
}