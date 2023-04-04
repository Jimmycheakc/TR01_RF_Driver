#include <stdio.h>
#include "at86rf215_common.h"
#include <pthread.h>


void event_node_init(event_st* ev)
{
    pthread_cond_init(&ev->ready_cond, NULL);
    pthread_mutex_init(&ev->ready_mutex, NULL);
}

void event_node_close(event_st* ev)
{
    pthread_cond_destroy(&ev->ready_cond);
    pthread_mutex_destroy(&ev->ready_mutex);
}

void event_node_wait_ready(event_st* ev)
{
    pthread_mutex_lock(&ev->ready_mutex);
    while (!ev->ready)
    {
        pthread_cond_wait(&ev->ready_cond, &ev->ready_mutex);
    }
    ev->ready = 0;
    pthread_mutex_unlock(&ev->ready_mutex);
}

void event_node_signal_ready(event_st* ev, int ready)
{
    pthread_mutex_lock(&ev->ready_mutex);
    ev->ready = ready;
    pthread_cond_signal(&ev->ready_cond);
    pthread_mutex_unlock(&ev->ready_mutex);
}

//===================================================================
static void at86rf215_radio_event_handler (at86rf215_st* dev,
                                at86rf215_rf_channel_en ch,
                                at86rf215_radio_irq_st *events)
{
    char channel_st[3];
    if (ch == at86rf215_rf_channel_900mhz) strcpy(channel_st, "09");
    else strcpy(channel_st, "24");

    if (events->wake_up_por)
    {
        printf("INT @ RADIO%s: Woke up\n", channel_st);
    }

    if (events->trx_ready)
    {
        printf("INT @ RADIO%s: Transceiver ready\n", channel_st);
        if (ch == at86rf215_rf_channel_900mhz) event_node_signal_ready(&dev->events.lo_trx_ready_event, 1);
        else if (ch == at86rf215_rf_channel_2400mhz) event_node_signal_ready(&dev->events.hi_trx_ready_event, 1);
    }

    if (events->energy_detection_complete)
    {
        printf("INT @ RADIO%s: Energy detection complete\n", channel_st);
        if (ch == at86rf215_rf_channel_900mhz) event_node_signal_ready(&dev->events.lo_energy_measure_event, 1);
        else if (ch == at86rf215_rf_channel_2400mhz) event_node_signal_ready(&dev->events.hi_energy_measure_event, 1);
    }

    if (events->battery_low)
    {
        printf("INT @ RADIO%s: Battery low\n", channel_st);
    }

    if (events->trx_error)
    {
        printf("INT @ RADIO%s: Transceiver error\n", channel_st);
    }

    if (events->IQ_if_sync_fail)
    {
        printf("INT @ RADIO%s: I/Q interface sync failed\n", channel_st);
    }
}

//===================================================================
static void at86rf215_baseband_event_handler (at86rf215_st* dev,
                                at86rf215_rf_channel_en ch,
                                at86rf215_baseband_irq_st *events)
{
    char channel_st[3];
    if (ch == at86rf215_rf_channel_900mhz) strcpy(channel_st, "09");
    else strcpy(channel_st, "24");

    if (events->frame_rx_started)
    {
        printf("INT @ BB%s: Frame reception started\n", channel_st);
    }

    if (events->frame_rx_complete)
    {
        printf("INT @ BB%s: Frame reception complete\n", channel_st);
    }

    if (events->frame_rx_address_match)
    {
        printf("INT @ BB%s: Frame address matched\n", channel_st);
    }

    if (events->frame_rx_match_extended)
    {
        printf("INT @ BB%s: Frame extended address matched\n", channel_st);
    }

    if (events->frame_tx_complete)
    {
        printf("INT @ BB%s: Frame transmission complete\n", channel_st);
    }

    if (events->agc_hold)
    {
        printf("INT @ BB%s: AGC hold\n", channel_st);
    }

    if (events->agc_release)
    {
        printf("INT @ BB%s: AGC released\n", channel_st);
    }

    if (events->frame_buffer_level)
    {
        printf("INT @ BB%s: Frame buffer level\n", channel_st);
    }
}

//===================================================================
void at86rf215_interrupt_handler (int event, unsigned int line_offset, const struct timespec * time, void *data)
{
    int i;
    at86rf215_st* dev = (at86rf215_st*)data;
    at86rf215_irq_st irq = {0};

    //if (level == 0) return;
    // first read the irqs
    printf("%s\n", __func__);
    at86rf215_get_irqs(dev, &irq, 0);
    uint8_t *tmp = (uint8_t *)&irq;
	dev->num_interrupts ++;

    if (tmp[0] != 0) at86rf215_radio_event_handler (dev, at86rf215_rf_channel_900mhz, &irq.radio09);
    if (tmp[1] != 0) at86rf215_radio_event_handler (dev, at86rf215_rf_channel_2400mhz, &irq.radio24);
    if (tmp[2] != 0) at86rf215_baseband_event_handler (dev, at86rf215_rf_channel_900mhz, &irq.bb0);
    if (tmp[3] != 0) at86rf215_baseband_event_handler (dev, at86rf215_rf_channel_2400mhz, &irq.bb1);
}
