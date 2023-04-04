/**
* @file ads7041.h
* @brief This header file provides the interface to access the ADC from TI ADS7041IRUGR.
*
* @author Jimmy Chea
* @date 30/03/2023
*/

#ifndef __ADS7041_H__
#define __ADS7041_H__

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    int fd;
    const char *dev_name;
    uint8_t slave_addr;
} ads7041_t;

void ads7041_init(ads7041_t *self, const char *device_name, uint8_t addr);
void ads7041_close(ads7041_t *self);
double ads7041_get_adc_millivolt(ads7041_t *self, uint8_t reg_addr);

#ifdef __cplusplus
}
#endif  /* __cplusplus */

#endif /* __ADS7041_H__ */
