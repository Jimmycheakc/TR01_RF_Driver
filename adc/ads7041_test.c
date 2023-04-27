#include <stdio.h>
#include <stdint.h>
#include "ads7041.h"

#define I2C_DEVICE "/dev/i2c-3"

int main (void)
{
    ads7041_t dev;
    double adc;

    ads7041_init(&dev, I2C_DEVICE, 0x22, ads7041_1_ss); // Temp: slave address need to update

    adc = ads7041_get_adc_millivolt(&dev, 0x22);  // Temp: register address need to update

    printf("ADC value in milli Volts : %lf\n", adc);

    ads7041_close(&dev);

    return 0;
}
