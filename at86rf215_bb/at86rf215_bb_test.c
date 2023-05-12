#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include "gpiod.h"
#include <pthread.h>
#include <string.h>
#include "at86rf215_bb.h"
#include "at86rf215_bb_regs.h"
#include "../io_utils/io_utils.h"
#include "../spi/spi.h"

struct gpiod_line;

#define GPIO_CHIP_NAME "gpiochip5"
#define GPIO_PIN_NUM    8

int
at86rf215_irq_callback_temp(int event, unsigned int line_offset, const struct timespec * time, void *data)
{
    printf("%s\n", __func__);
    return 0;
}

int transmit_frame()
{
    int ret;
    at86rf215_rf_state_t radio_state;

    struct at86rf215 rfic_dev = {
        .clko_os = AT86RF215_RF_CLKO_26_MHZ,
        .clk_drv = AT86RF215_RF_DRVCLKO4,
        .rf_femode_09 = AT86RF215_RF_FEMODE0,
        .rf_femode_24= AT86RF215_RF_FEMODE0,
        .xo_fs = 1,
        .xo_trim = 0,
        .irqmm = 1,
        .irqp = 0,
        .pad_drv = AT86RF215_RF_DRV4,
    };

    ret = at86rf215_init(&rfic_dev);
    if (ret)
    {
        printf("Failed to init at86rf215 RFIC.\n");
        return -1;
    }

    // Setup GPIO interrupt
    ret = io_utils_setup_interrupt_bb(GPIO_CHIP_NAME, GPIOD_CTXLESS_EVENT_RISING_EDGE,
                                    GPIO_PIN_NUM, false, 
                                    "gpio_handler", 
                                    at86rf215_irq_callback,
                                    &rfic_dev,
                                    GPIOD_CTXLESS_FLAG_BIAS_PULL_UP
                                    );
    if (ret)
    {
        printf("io_utils_setup_interrupt() error.\n");
        return -1;
    }

    at86rf215_set_mode(&rfic_dev, AT86RF215_RF_MODE_BBRF);

    // Setup OFDM
    struct at86rf215_radio_conf radio_conf = {
        .cm = AT86RF215_CM_IEEE,
	    .cs = 1200000,
	    .base_freq = 903200000,
	    .lbw = AT86RF215_PLL_LBW_DEFAULT,
    };
    ret = at86rf215_radio_conf(&rfic_dev, AT86RF215_RF09, &radio_conf);
    if (ret)
    {
        printf("at86rf215_radio_conf() error.\n");
        return -1;
    }

    ret = at86rf215_set_radio_irq_mask(&rfic_dev, AT86RF215_RF09, 0x1F);
    if (ret)
    {
        printf("at86rf215_set_radio_irq_mask() error.\n");
        return -1;
    }
    ret = at86rf215_set_bbc_irq_mask(&rfic_dev, AT86RF215_RF09, 0x1F);
    if (ret)
    {
        printf("at86rf215_set_bbc_irq_mask() error.\n");
        return -1;
    }

    uint8_t mask;
    at86rf215_get_irq_mask(&rfic_dev, &mask, AT86RF215_RF09);
    printf("Mask = %x\n", mask);

    at86rf215_get_state(&rfic_dev, &radio_state, AT86RF215_RF09);
    printf("radio state = %d\n", radio_state);

    sleep(2);
    at86rf215_set_cmd(&rfic_dev, AT86RF215_STATE_RF_TRXOFF, AT86RF215_RF09);
    at86rf215_get_state(&rfic_dev, &radio_state, AT86RF215_RF09);
    printf("radio state = %d\n", radio_state);

    ret = at86rf215_set_channel(&rfic_dev, AT86RF215_RF09, 0x0003);
    if (ret)
    {
        printf("at86rf215_set_channel() error.\n");
        return -1;
    }
    ret = at86rf215_set_txcutc(&rfic_dev, AT86RF215_RF09, AT86RF215_RF_PARAMP4U, AT86RF215_RF_FLC1000KHZ);
    if (ret)
    {
        printf("at86rf215_set_txcutc() error.\n");
        return -1;
    }
    ret = at86rf215_set_txdfe(&rfic_dev, AT86RF215_RF09, 4, 0, AT86RF215_SR_1333KHZ);
    if (ret)
    {
        printf("at86rf215_set_txdfe() error.\n");
        return -1;
    }
    ret = at86rf215_set_rxdfe(&rfic_dev, AT86RF215_RF09, 4, AT86RF215_SR_1333KHZ);
    if (ret)
    {
        printf("at86rf215_set_rxdfe() error.\n");
        return -1;
    }
    ret = at86rf215_set_bw(&rfic_dev, AT86RF215_RF09, 0, 0, AT86RF215_RF_BW1250KHZ_IF2000KHZ);
    if (ret)
    {
        printf("at86rf215_set_bw() error.\n");
        return -1;
    }
    ret = at86rf215_set_pac(&rfic_dev, AT86RF215_RF09, AT86RF215_PACUR_NO_RED, 0x1C);
    if (ret)
    {
        printf("at86rf215_set_pac() error.\n");
        return -1;
    }
    ret = at86rf215_set_edd(&rfic_dev, AT86RF215_RF09, 0x1E, AT86RF215_EDD_DTB_32US);
    if (ret)
    {
        printf("at86rf215_set_edd() error.\n");
        return -1;
    }

    struct at86rf215_bb_conf bb_ofdm_conf = {
        .ctx = 0,
        .fcsfe = 1,
        .txafcs = 1,
        .fcst = AT86RF215_FCS_32,
        .pt = AT86RF215_BB_MROFDM,
        .ofdm = {
            .mcs = AT86RF215_QPSK_1_2,
            .ssrx = AT86RF215_SSRX_0,
            .sstx = AT86RF215_SSTX_0,
            .lfo = 0,
            .poi = 0,
            .opt = AT86RF215_OPT_1,
            .pdt = 3,
            .rxo = 1,
        }
    };
    ret = at86rf215_bb_conf(&rfic_dev, AT86RF215_RF09, &bb_ofdm_conf);
    if (ret)
    {
        printf("at86rf215_bb_conf() error.\n");
        return -1;
    }
    ret = at86rf215_bb_enable(&rfic_dev, AT86RF215_RF09, 1);
    if (ret)
    {
        printf("at86rf215_bb_enable() error.\n");
        return -1;
    }

    at86rf215_set_cmd(&rfic_dev, AT86RF215_CMD_RF_TXPREP, AT86RF215_RF09);

    at86rf215_get_state(&rfic_dev, &radio_state, AT86RF215_RF09);
    printf("radio state = %d\n", radio_state);

    printf("Sending TX frame.\n");
    uint8_t tx_data[100];
    memset(&tx_data, 0xFF, sizeof(tx_data));
    at86rf215_tx_frame(&rfic_dev, AT86RF215_RF09, tx_data, sizeof(tx_data));

    at86rf215_get_state(&rfic_dev, &radio_state, AT86RF215_RF09);

    printf("TX done. state = %d\n", radio_state);

    printf("Waiting Interrupt or Press enter to stop.\n");
    getchar();

    io_utils_destroy_interrupt(rfic_dev.irq_tid, rfic_dev.irq_data);

    return 0;
}

int main (void)
{
    int ret;
    at86rf215_rf_state_t radio_state;

    struct at86rf215 rfic_dev = {
        .clko_os = AT86RF215_RF_CLKO_26_MHZ,
        .clk_drv = AT86RF215_RF_DRVCLKO4,
        .rf_femode_09 = AT86RF215_RF_FEMODE0,
        .rf_femode_24= AT86RF215_RF_FEMODE0,
        .xo_fs = 1,
        .xo_trim = 0,
        .irqmm = 1,
        .irqp = 0,
        .pad_drv = AT86RF215_RF_DRV4,
    };

    ret = at86rf215_init(&rfic_dev);
    if (ret)
    {
        printf("Failed to init at86rf215 RFIC.\n");
        return -1;
    }

    // Setup GPIO interrupt
    ret = io_utils_setup_interrupt_bb(GPIO_CHIP_NAME, GPIOD_CTXLESS_EVENT_RISING_EDGE,
                                    GPIO_PIN_NUM, false, 
                                    "gpio_handler", 
                                    at86rf215_irq_callback,
                                    &rfic_dev,
                                    GPIOD_CTXLESS_FLAG_BIAS_PULL_UP
                                    );
    if (ret)
    {
        printf("io_utils_setup_interrupt() error.\n");
        return -1;
    }

    at86rf215_set_mode(&rfic_dev, AT86RF215_RF_MODE_BBRF);

    // Setup OFDM
    struct at86rf215_radio_conf radio_conf = {
        .cm = AT86RF215_CM_IEEE,
	    .cs = 1200000,
	    .base_freq = 903200000,
	    .lbw = AT86RF215_PLL_LBW_DEFAULT,
    };
    ret = at86rf215_radio_conf(&rfic_dev, AT86RF215_RF09, &radio_conf);
    if (ret)
    {
        printf("at86rf215_radio_conf() error.\n");
        return -1;
    }

    ret = at86rf215_set_radio_irq_mask(&rfic_dev, AT86RF215_RF09, 0x1F);
    if (ret)
    {
        printf("at86rf215_set_radio_irq_mask() error.\n");
        return -1;
    }
    ret = at86rf215_set_bbc_irq_mask(&rfic_dev, AT86RF215_RF09, 0x1F);
    if (ret)
    {
        printf("at86rf215_set_bbc_irq_mask() error.\n");
        return -1;
    }

    uint8_t mask;
    at86rf215_get_irq_mask(&rfic_dev, &mask, AT86RF215_RF09);
    printf("Mask = %x\n", mask);

    at86rf215_get_state(&rfic_dev, &radio_state, AT86RF215_RF09);
    printf("radio state = %d\n", radio_state);

    sleep(2);
    at86rf215_set_cmd(&rfic_dev, AT86RF215_STATE_RF_TRXOFF, AT86RF215_RF09);
    at86rf215_get_state(&rfic_dev, &radio_state, AT86RF215_RF09);
    printf("radio state = %d\n", radio_state);

    at86rf215_set_cmd(&rfic_dev, AT86RF215_CMD_RF_TXPREP, AT86RF215_RF09);

    ret = at86rf215_set_channel(&rfic_dev, AT86RF215_RF09, 0x0003);
    if (ret)
    {
        printf("at86rf215_set_channel() error.\n");
        return -1;
    }
    ret = at86rf215_set_txcutc(&rfic_dev, AT86RF215_RF09, AT86RF215_RF_PARAMP4U, AT86RF215_RF_FLC1000KHZ);
    if (ret)
    {
        printf("at86rf215_set_txcutc() error.\n");
        return -1;
    }
    ret = at86rf215_set_txdfe(&rfic_dev, AT86RF215_RF09, 4, 0, AT86RF215_SR_1333KHZ);
    if (ret)
    {
        printf("at86rf215_set_txdfe() error.\n");
        return -1;
    }
    ret = at86rf215_set_rxdfe(&rfic_dev, AT86RF215_RF09, 4, AT86RF215_SR_1333KHZ);
    if (ret)
    {
        printf("at86rf215_set_rxdfe() error.\n");
        return -1;
    }
    ret = at86rf215_set_bw(&rfic_dev, AT86RF215_RF09, 0, 0, AT86RF215_RF_BW1250KHZ_IF2000KHZ);
    if (ret)
    {
        printf("at86rf215_set_bw() error.\n");
        return -1;
    }
    ret = at86rf215_set_pac(&rfic_dev, AT86RF215_RF09, AT86RF215_PACUR_NO_RED, 0x1C);
    if (ret)
    {
        printf("at86rf215_set_pac() error.\n");
        return -1;
    }
    ret = at86rf215_set_edd(&rfic_dev, AT86RF215_RF09, 0x1E, AT86RF215_EDD_DTB_32US);
    if (ret)
    {
        printf("at86rf215_set_edd() error.\n");
        return -1;
    }

    struct at86rf215_bb_conf bb_ofdm_conf = {
        .ctx = 1,
        .fcsfe = 1,
        .txafcs = 1,
        .fcst = AT86RF215_FCS_32,
        .pt = AT86RF215_BB_MROFDM,
        .ofdm = {
            .mcs = AT86RF215_QPSK_1_2,
            .ssrx = AT86RF215_SSRX_0,
            .sstx = AT86RF215_SSTX_0,
            .lfo = 0,
            .poi = 0,
            .opt = AT86RF215_OPT_1,
            .pdt = 3,
            .rxo = 1,
        }
    };
    ret = at86rf215_bb_conf(&rfic_dev, AT86RF215_RF09, &bb_ofdm_conf);
    if (ret)
    {
        printf("at86rf215_bb_conf() error.\n");
        return -1;
    }
    ret = at86rf215_bb_enable(&rfic_dev, AT86RF215_RF09, 1);
    if (ret)
    {
        printf("at86rf215_bb_enable() error.\n");
        return -1;
    }

    at86rf215_get_state(&rfic_dev, &radio_state, AT86RF215_RF09);
    printf("radio state = %d\n", radio_state);
    
    uint8_t value;
    at86rf215_reg_read_8(&value, REG_BBC0_PC);
    printf("The REG_BBC0_PC = 0x%x\n", value);

    printf("Sending TX frame.\n");
    uint8_t tx_data[100];
    memset(&tx_data, 0xFF, sizeof(tx_data));
    at86rf215_tx_frame(&rfic_dev, AT86RF215_RF09, tx_data, sizeof(tx_data));

    at86rf215_get_state(&rfic_dev, &radio_state, AT86RF215_RF09);

    printf("TX done. state = %d\n", radio_state);

    printf("Waiting Interrupt or Press enter to stop.\n");
    getchar();

    value = 0;
    at86rf215_reg_read_8(&value, REG_BBC0_PC);
    printf("The REG_BBC0_PC = 0x%x\n", value);

    at86rf215_get_state(&rfic_dev, &radio_state, AT86RF215_RF09);
    printf("radio state = %d\n", radio_state);

    at86rf215_set_cmd(&rfic_dev, AT86RF215_STATE_RF_TRXOFF, AT86RF215_RF09);

    at86rf215_get_state(&rfic_dev, &radio_state, AT86RF215_RF09);
    printf("radio state = %d\n", radio_state);

    io_utils_destroy_interrupt(rfic_dev.irq_tid, rfic_dev.irq_data);

    return 0;
}