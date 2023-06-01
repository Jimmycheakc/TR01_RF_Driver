#include <stdio.h>
#include "at86rf215_bb.h"
#include "at86rf215_bb_regs.h"
#include "at86rf215_bb_usb.h"
#include <pthread.h>

#define BIT(n) (1UL << (n))

void event_node_init(at86rf215_event_st* ev)
{
    pthread_cond_init(&ev->ready_cond, NULL);
    pthread_mutex_init(&ev->ready_mutex, NULL);
}

void event_node_close(at86rf215_event_st* ev)
{
    pthread_cond_destroy(&ev->ready_cond);
    pthread_mutex_destroy(&ev->ready_mutex);
}

void event_node_wait_ready(at86rf215_event_st* ev)
{
    pthread_mutex_lock(&ev->ready_mutex);
    while (!ev->ready)
    {
        pthread_cond_wait(&ev->ready_cond, &ev->ready_mutex);
    }
    ev->ready = 0;
    pthread_mutex_unlock(&ev->ready_mutex);
}

void event_node_signal_ready(at86rf215_event_st* ev, int ready)
{
    pthread_mutex_lock(&ev->ready_mutex);
    ev->ready = ready;
    pthread_cond_signal(&ev->ready_cond);
    pthread_mutex_unlock(&ev->ready_mutex);
}

static void handle_bb_irq(struct at86rf215 *h, at86rf215_radio_t radio, uint8_t bbcn_irqs)
{
	struct at86rf215_radio *r = &h->priv.radios[radio];

	char channel_st[3];
    if (radio == AT86RF215_RF09) strcpy(channel_st, "09");
    else strcpy(channel_st, "24");

    if (bbcn_irqs & BIT(0))
    {
        printf("INT @ BB%s: Frame reception started\n", channel_st);
    }

    if (bbcn_irqs & BIT(1))
    {
        printf("INT @ BB%s: Frame reception complete\n", channel_st);
    }

    if (bbcn_irqs & BIT(2))
    {
        printf("INT @ BB%s: Frame address matched\n", channel_st);
    }

    if (bbcn_irqs & BIT(3))
    {
        printf("INT @ BB%s: Frame extended address matched\n", channel_st);
    }

    if (bbcn_irqs & BIT(4))
    {
        printf("INT @ BB%s: Frame transmission complete\n", channel_st);
    }

    if (bbcn_irqs & BIT(5))
    {
        printf("INT @ BB%s: AGC hold\n", channel_st);
    }

    if (bbcn_irqs & BIT(6))
    {
        printf("INT @ BB%s: AGC released\n", channel_st);
    }

    if (bbcn_irqs & BIT(7))
    {
        printf("INT @ BB%s: Frame buffer level\n", channel_st);
    }
}

static void handle_rf_irq(struct at86rf215 *h, at86rf215_radio_t radio, uint8_t rfn_irqs)
{
	struct at86rf215_radio *r = &h->priv.radios[radio];
	
    char channel_st[3];
    if (radio == AT86RF215_RF09) strcpy(channel_st, "09");
    else strcpy(channel_st, "24");

    if (rfn_irqs & BIT(0))
    {
        printf("INT @ RADIO%s: Woke up\n", channel_st);
    }

    if (rfn_irqs & BIT(1))
    {
        printf("INT @ RADIO%s: Transceiver ready\n", channel_st);
        event_node_signal_ready(&r->trxready, 1);
    }

    if (rfn_irqs & BIT(2))
    {
        printf("INT @ RADIO%s: Energy detection complete\n", channel_st);
    }

    if (rfn_irqs & BIT(3))
    {
        printf("INT @ RADIO%s: Battery low\n", channel_st);
    }

    if (rfn_irqs & BIT(4))
    {
        printf("INT @ RADIO%s: Transceiver error\n", channel_st);
    }

    if (rfn_irqs & BIT(5))
    {
        printf("INT @ RADIO%s: I/Q interface sync failed\n", channel_st);
    }
}

static void handle_usb_bb_irq(struct at86rf215 *h, at86rf215_radio_t radio, uint8_t bbcn_irqs)
{
	struct at86rf215_radio *r = &h->priv.radios[radio];

	char channel_st[3];
    if (radio == AT86RF215_RF09) strcpy(channel_st, "09");
    else strcpy(channel_st, "24");

    if (bbcn_irqs & BIT(0))
    {
        printf("INT @ BB%s: Frame reception started\n", channel_st);
    }

    if (bbcn_irqs & BIT(1))
    {
        printf("INT @ BB%s: Frame reception complete\n", channel_st);
    }

    if (bbcn_irqs & BIT(2))
    {
        printf("INT @ BB%s: Frame address matched\n", channel_st);
    }

    if (bbcn_irqs & BIT(3))
    {
        printf("INT @ BB%s: Frame extended address matched\n", channel_st);
    }

    if (bbcn_irqs & BIT(4))
    {
        printf("INT @ BB%s: Frame transmission complete\n", channel_st);
    }

    if (bbcn_irqs & BIT(5))
    {
        printf("INT @ BB%s: AGC hold\n", channel_st);
    }

    if (bbcn_irqs & BIT(6))
    {
        printf("INT @ BB%s: AGC released\n", channel_st);
    }

    if (bbcn_irqs & BIT(7))
    {
        printf("INT @ BB%s: Frame buffer level\n", channel_st);
    }
}

static void handle_usb_rf_irq(struct at86rf215 *h, at86rf215_radio_t radio, uint8_t rfn_irqs)
{
	struct at86rf215_radio *r = &h->priv.radios[radio];
	
    char channel_st[3];
    if (radio == AT86RF215_RF09) strcpy(channel_st, "09");
    else strcpy(channel_st, "24");

    if (rfn_irqs & BIT(0))
    {
        printf("INT @ RADIO%s: Woke up\n", channel_st);
    }

    if (rfn_irqs & BIT(1))
    {
        printf("INT @ RADIO%s: Transceiver ready\n", channel_st);
        event_node_signal_ready(&r->trxready, 1);
    }

    if (rfn_irqs & BIT(2))
    {
        printf("INT @ RADIO%s: Energy detection complete\n", channel_st);
    }

    if (rfn_irqs & BIT(3))
    {
        printf("INT @ RADIO%s: Battery low\n", channel_st);
    }

    if (rfn_irqs & BIT(4))
    {
        printf("INT @ RADIO%s: Transceiver error\n", channel_st);
    }

    if (rfn_irqs & BIT(5))
    {
        printf("INT @ RADIO%s: I/Q interface sync failed\n", channel_st);
    }
}

/**
 * The IRQ handler of the AT86RF215. All IRQ sources are automatically
 * acknowledged
 * @note If custom IRQ handling is needed, please re-implement the
 * at86rf215_irq_user_callback() which is called internally by this handler.
 * @param h the device handler
 * @return 0 on success or negative error code
 */
int at86rf215_irq_callback(int event, unsigned int line_offset, const struct timespec * time, void *data)
{
	printf("%s\n", __func__);
	struct at86rf215 *h = (struct at86rf215*)data;

    // TODO : Move it into at86rf215_bb.c as ready() static function
	int ret = ready(h);
	if (ret) {
		return ret;
	}

	/*
	 * Read and acknowledge all IRQ sources
	 * NOTE: Block mode did not acknowledged the triggered IRQs, even if
	 * the manual says that it should
	 */
	uint8_t irqs[4] = {0x0, 0x0, 0x0, 0x0};
	at86rf215_reg_read_8(h, &irqs[0], REG_RF09_IRQS);
	at86rf215_reg_read_8(h, &irqs[1], REG_RF24_IRQS);
	at86rf215_reg_read_8(h, &irqs[2], REG_BBC0_IRQS);
	at86rf215_reg_read_8(h, &irqs[3], REG_BBC1_IRQS);

	handle_rf_irq(h, AT86RF215_RF09, irqs[0]);
	handle_rf_irq(h, AT86RF215_RF24, irqs[1]);
	handle_bb_irq(h, AT86RF215_RF09, irqs[2]);
	handle_bb_irq(h, AT86RF215_RF24, irqs[3]);


	return at86rf215_irq_user_callback(h, irqs[0], irqs[1],
	                                   irqs[2], irqs[3]);
}

int at86rf215_usb_irq_callback(int event, unsigned int line_offset, const struct timespec * time, void *data)
{
	printf("%s\n", __func__);
	struct at86rf215 *h = (struct at86rf215*)data;

    // TODO : Move it into at86rf215_bb.c as ready() static function
	int ret = ready(h);
	if (ret) {
		return ret;
	}

	/*
	 * Read and acknowledge all IRQ sources
	 * NOTE: Block mode did not acknowledged the triggered IRQs, even if
	 * the manual says that it should
	 */
	uint8_t irqs[4] = {0x0, 0x0, 0x0, 0x0};
	at86rf215_usb_reg_read_8(&irqs[0], REG_RF09_IRQS);
	at86rf215_usb_reg_read_8(&irqs[1], REG_RF24_IRQS);
	at86rf215_usb_reg_read_8(&irqs[2], REG_BBC0_IRQS);
	at86rf215_usb_reg_read_8(&irqs[3], REG_BBC1_IRQS);

	handle_usb_rf_irq(h, AT86RF215_RF09, irqs[0]);
	handle_usb_rf_irq(h, AT86RF215_RF24, irqs[1]);
	handle_usb_bb_irq(h, AT86RF215_RF09, irqs[2]);
	handle_usb_bb_irq(h, AT86RF215_RF24, irqs[3]);


	return at86rf215_irq_user_callback(h, irqs[0], irqs[1],
	                                   irqs[2], irqs[3]);
}

/**
 * Custom IRQ handling. Called internally by the at86rf215_irq_callback()
 * @param h the device handler
 * @param rf09_irqs RF09_IRQS register
 * @param rf24_irqs RF24_IRQS register
 * @param bbc0_irqs BBC0_IRQS register
 * @param bbc1_irqs BBC1_IRQS register
 * @return 0 on success or negative error code
 */
__attribute__((weak)) int
at86rf215_irq_user_callback(const struct at86rf215 *h, uint8_t rf09_irqs,
                            uint8_t rf24_irqs, uint8_t bbc0_irqs, uint8_t bbc1_irqs)
{
	return AT86RF215_OK;
}