/**
* @file io_utils.c
* @brief This implementation file contains the code for gpio interface.
*
* @author Jimmy Chea
* @date 30/03/2023
*/

#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include "io_utils.h"
#include "../at86rf215/at86rf215_common.h"


typedef struct {
   const char device[64];
   int event_type;
   unsigned int pin;
   bool active_low;
   const char consumer[64];
   gpiod_ctxless_event_handle_cb event_cb;
   void *data;
   int flags;
} gpio_interrupt_t;


#define CHIP_NAME    "gpiochip5"
#define IO_UTILS_SHORT_WAIT(N)   {for (int i=0; i<(N); i++) { asm volatile("nop"); }}

/**
 *  A detailed description of the io_utils_write_gpio function
 *
 *  This function is to write the gpio pin with value, pin status, pin consumer and flag.
 *
 *  @param \*device - input the gpio chip name "gpiochip*"
 *  @param pin - pin offset
 *  @param value - low(0) or high(1)
 *  @param active_low - true(active low) or false(default)
 *  @param \*consumer - provide the consumer name for this pin, eg, "interrupt"
 *  @param flags - refer to enum of GPIO flags on gpiod.h
 *  @return - 0 on success, else return -1 on error
 */
int io_utils_write_gpio(const char *device, unsigned int pin, int value, bool active_low, const char *consumer, int flags)
{
   return gpiod_ctxless_set_value_ext(device, pin, value, active_low, consumer, NULL, NULL, flags);
}

/**
 *  A detailed description of the io_utils_write_gpio_with_wait function
 *
 *  This function is to write the gpio pin with value, pin status, pin consumer and flag.
 *  After that wait for the requested time
 *
 *  @param \*device - input the gpio chip name "gpiochip*"
 *  @param pin - pin offset
 *  @param value - low(0) or high(1)
 *  @param active_low - true(active low) or false(default)
 *  @param \*consumer - provide the consumer name for this pin, eg, "interrupt"
 *  @param flags - refer to enum of GPIO flags on gpiod.h
 *  @param nopcnt - the amount of nop count
 *  @return - 0 on success, else return -1 on error
 */
int io_utils_write_gpio_with_wait(const char *device, unsigned int pin, int value, bool active_low, const char *consumer, int flags,  int nopcnt)
{
   int ret;

   ret = io_utils_write_gpio(device, pin, value, active_low, consumer, flags);
   for (volatile int i = 0; i < nopcnt; i++)
   {
      __asm("nop");
   }

   return ret;
}

/**
 *  A detailed description of the io_utils_read_gpio function
 *
 *  This function is to read the gpio pin with value, pin status, pin consumer and flag.
 *  After that wait for the requested time
 *
 *  @param \*device - input the gpio chip name "gpiochip*"
 *  @param pin - pin offset
 *  @param value - low(0) or high(1)
 *  @param active_low - true(active low) or false(default)
 *  @param \*consumer - provide the consumer name for this pin, eg, "interrupt"
 *  @param flags - refer to enum of GPIO flags on gpiod.h
 *  @return - 0 or 1 (GPIO value) on success, else return -1 on error
 */
int io_utils_read_gpio(const char *device, unsigned int pin, bool active_low, const char *consumer, int flags)
{
   return gpiod_ctxless_get_value_ext(device, pin, active_low, consumer, flags);
}

/**
 *  A detailed description of the io_utils_usleep function
 *
 *  This function is to perform sleep in micro seconds.
 *
 *  @param usec - micro seconds
 *  @return - none
 */
void io_utils_usleep(int usec)
{
   struct timespec req = {.tv_sec = 0, .tv_nsec = usec * 1000L};
   nanosleep(&req, (struct timespec *)NULL);
}

/**
 *  A detailed description of the *gpio_interrupt_thread function
 *
 *  This function is to perform the interrupt in the form of thread.
 *  The interrupt will use for monitoring the gpio interrupt and perform a callback function.
 *
 *  @param \*data - gpio interrupt data in struct gpio_interrupt_t
 *  @return - none
 */
void *gpio_interrupt_thread(void *data)
{
   int ret;
   gpio_interrupt_t *gpio_interrupt = (gpio_interrupt_t*)data;

   ret = gpiod_ctxless_event_monitor_ext(gpio_interrupt->device,
                                       gpio_interrupt->event_type,
                                       gpio_interrupt->pin,
                                       gpio_interrupt->active_low,
                                       gpio_interrupt->consumer,
                                       NULL,
                                       NULL,
                                       gpio_interrupt->event_cb,
                                       gpio_interrupt->data,
                                       gpio_interrupt->flags);
   if (ret < 0)
   {
      printf("Failed to register interrupt event callback function.\n");
   }

   free(gpio_interrupt);

   return NULL;
}

/**
 *  A detailed description of the io_utils_setup_interrupt function
 *
 *  This function is to setup the interrupt and create a thread to handle interrupt.
 *  The interrupt will use for monitoring the gpio interrupt and perform a callback function.
 *
 *  @param pthread - thread id 
 *  @param \*device - input the gpio chip name "gpiochip*"
 *  @param event_type - rising, falling or both edge event, can refer to event types in gpiod.h
 *  @param pin - pin offset
 *  @param active_low - true(active low) or false(default)
 *  @param \*consumer - provide the consumer name for this pin, eg, "interrupt"
 *  @param event_cb - event callback function
 *  @param \*data - event callback function data
 *  @param flags - refer to enum of GPIO flags on gpiod.h
 *  @return - 0 on success, else return error number on error
 */
int io_utils_setup_interrupt(pthread_t pthread, const char *device, int event_type,
                             unsigned int pin, bool active_low,
                             const char *consumer,
                             gpiod_ctxless_event_handle_cb event_cb,
                             void *data, int flags)
{
   int ret;
   gpio_interrupt_t *irq_data = malloc(sizeof(gpio_interrupt_t));
   if (irq_data == NULL)
   {
      printf("Failed to allocate memory for irq data \n");
      return -1;
   }

   strcpy(irq_data->device, device);
   irq_data->event_type = event_type;
   irq_data->pin = pin;
   irq_data->active_low = active_low;
   strcpy(irq_data->consumer, consumer);
   irq_data->event_cb = event_cb;
   irq_data->data = data;
   irq_data->flags = flags;

   ret = pthread_create(&pthread, NULL, gpio_interrupt_thread, (void *)irq_data);
   if (ret < 0)
   {
      printf("Failed to create a thread.\n");
      free(irq_data);
   }

   return ret;
}

/**
 *  A detailed description of the io_utils_destroy_interrupt function
 *
 *  This function is to destroy the pthread created based on the thread id
 *
 *  @param pthread - thread id
 *  @return - none
 */
void io_utils_destroy_interrupt(pthread_t pthread)
{
   pthread_join(pthread, NULL);
   pthread_cancel(pthread);
}
