/**
* @file io_utils.h
* @brief This header file provides the gpio interface.
*
* @author Jimmy Chea
* @date 30/03/2023
*/

#ifndef __IO_UTILS_H__
#define __IO_UTILS_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdlib.h>
#include "gpiod.h"


typedef struct {
   char device[64];
   int event_type;
   unsigned int pin;
   bool active_low;
   char consumer[64];
   gpiod_ctxless_event_handle_cb event_cb;
   void *data;
   int flags;
} gpio_interrupt_t;

int io_utils_write_gpio(const char *device, unsigned int pin, int value, bool active_low, const char *consumer, int flags);
int io_utils_write_gpio_with_wait(const char *device, unsigned int pin, int value, bool active_low, const char *consumer, int flags,  int nopcnt);
int io_utils_read_gpio(const char *device, unsigned int pin, bool active_low, const char *consumer, int flags);
void io_utils_usleep(int usec);
int io_utils_setup_interrupt(const char *device, int event_type,
                             unsigned int pin, bool active_low,
                             const char *consumer,
                             gpiod_ctxless_event_handle_cb event_cb,
                             void *data, int flags);
int io_utils_setup_interrupt_bb(const char *device, int event_type,
                              unsigned int pin, bool active_low,
                              const char *consumer,
                              gpiod_ctxless_event_handle_cb event_cb,
                              void *data, int flags);
int io_utils_setup_interrupt_bb2(const char *device, int event_type,
                              unsigned int pin, bool active_low,
                              const char *consumer,
                              gpiod_ctxless_event_handle_cb event_cb,
                              void *data, int flags);
void io_utils_destroy_interrupt(pthread_t pthread, gpio_interrupt_t *irq_data);

#ifdef __cplusplus
}
#endif

#endif // __IO_UTILS_H__