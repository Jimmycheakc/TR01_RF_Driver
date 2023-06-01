#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <ctype.h>
#include "gpiod.h"
#include <pthread.h>
#include <string.h>
#include "at86rf215_bb.h"
#include "at86rf215_bb_regs.h"
#include "../io_utils/io_utils.h"
#include "../spi/spi.h"
#include "../Json/cJSON.h"
#include "ftd2xx.h"
#include "libft4222.h"
#include "at86rf215_bb_usb.h"

#define SPI_DEVICE "/dev/spidev1.0"
#define GPIO_CHIP_NAME "gpiochip5"
#define GPIO_PIN_NUM    8
#define GPIO_PIN2_NUM    11

#ifdef DEBUG
    #define DEBUG_PRINT(fmt, ...) printf(fmt, __VA_ARGS__)
#else
    #define DEBUG_PRINT(fmt, ...)
#endif


int continuous_transmit_frame()
{
    int ret;
    at86rf215_rf_state_t radio_state;

    spi_t spi;
    ret = spi_init(&spi, SPI_DEVICE, 0, 8, 20000000);
    if (ret)
    {
        return -1;
    }

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

    ret = at86rf215_init(&rfic_dev, &spi);
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
    at86rf215_reg_read_8(&rfic_dev, &value, REG_BBC0_PC);
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
    at86rf215_reg_read_8(&rfic_dev, &value, REG_BBC0_PC);
    printf("The REG_BBC0_PC = 0x%x\n", value);

    at86rf215_get_state(&rfic_dev, &radio_state, AT86RF215_RF09);
    printf("radio state = %d\n", radio_state);

    at86rf215_set_cmd(&rfic_dev, AT86RF215_STATE_RF_TRXOFF, AT86RF215_RF09);

    at86rf215_get_state(&rfic_dev, &radio_state, AT86RF215_RF09);
    printf("radio state = %d\n", radio_state);

    io_utils_destroy_interrupt(rfic_dev.irq_tid, rfic_dev.irq_data);

    return 0;
}

char rf_config_string [][30] = 
{
    "RF_BBRF_MODE",
    "RF_CLKO.OS",
    "RF_CLKO.DRV",
    "RF09_PADFE.PADFE",
    "RF24_PADFE.PADFE",
    "RF_XOC.FS",
    "RF_XOC.TRIM",
    "RF_CFG.IRQMM",
    "RF_CFG.IRQP",
    "RF_CFG.DRV",
    "RF_IQIFC1.CHPM",
    "RF09_CNM.CM",
    "RF09_CS.CS",
    "RF09_BASE_FREQUENCY",
    "RF09_PLL.LBW",
    "RF09_IRQM",
    "BBC0_IRQM",
    "RF09_CNL",
    "RF09_TXCUTC.PARAMP",
    "RF09_TXCUTC.LPFCUT",
    "RF09_TXDFE.RCUT",
    "RF09_TXDFE.DM",
    "RF09_TXDFE.SR",
    "RF09_RXDFE.RCUT",
    "RF09_RXDFE.SR",
    "RF09_RXBWC.IFI",
    "RF09_RXBWC.IFS",
    "RF09_RXBWC.BW",
    "RF09_PAC.PACUR",
    "RF09_PAC.TXPWR",
    "RF09_EDD.DF",
    "RF09_EDD.DTB",
    "BBC0_PC.CTX",
    "BBC0_PC.FCSFE",
    "BBC0_PC.TXAFCS",
    "BBC0_PC.FCST",
    "BBC0_PC.BBEN",
    "BBC0_PC.PT",
    "BBC0_OFDMPHRTX.MCS",
    "BBC0_OFDMC.SSRX",
    "BBC0_OFDMC.SSTX",
    "BBC0_OFDMC.LFO",
    "BBC0_OFDMC.POI",
    "BBC0_OFDMC.OPT",
    "BBC0_OFDMSW.PDT",
    "BBC0_OFDMSW.RXO"
};

typedef struct {
    at86rf215_radio_t                 radio_mode;
    at86rf215_rf_clko_os_t            clko_os;
	at86rf215_rf_clko_drv_t           clko_drv;
	at86rf215_external_rffe_control_t rf_femode_09;
	at86rf215_external_rffe_control_t rf_femode_24;
	uint8_t                           xoc_fs   : 1;
	uint8_t                           xoc_trim : 4;
	uint8_t                           cfg_irqmm   : 1;
	uint8_t                           cfg_irqp    : 1;
	at86rf215_drv_t                   cfg_pad_drv;
    at86rf215_chpm_t                  chpm_mode;
    at86rf215_cm_t                    cnm_cm;
	uint32_t                          cs_cs;
	uint32_t                          base_freq;
	at86rf215_pll_lbw_t               pll_lbw;
    uint8_t                           rf_irqm;
    uint8_t                           bbc_irqm;
    uint16_t                          cnl_channel;
    at86rf215_paramp_t                txcutc_paramp;
    at86rf215_lpfcut_t                txcutc_lpf;
    uint8_t                           txdfe_rcut;
    uint8_t                           txdfe_dm;
    at86rf215_sr_t                    txdfe_sr;
    uint8_t                           rxdfe_rcut;
    at86rf215_sr_t                    rxdfe_sr;
    uint8_t                           rxbwc_ifi;
    uint8_t                           rxbwc_ifs;
    at86rf215_rx_bw_t                 rxbwc_bw;
    at86rf215_pacur_t                 pac_pacur;
    uint8_t                           pac_power;
    uint8_t                           edd_df;
    at86rf215_edd_dtb_t               edd_dtb;
    uint8_t                           pc_ctx    : 1;
	uint8_t                           pc_fcsfe  : 1;
	uint8_t                           pc_txafcs : 1;
	at86rf215_fcs_t                   pc_fcst;
    uint8_t                           pc_bben;
	at86rf215_phy_t                   pc_pt;
    at86rf215_ofdm_tx_mcs_t	          ofdmphrtx_mcs;
	at86rf215_ofdm_ssrx_t 	          ofdmc_ssrx;
	at86rf215_ofdm_sstx_t 	          ofdmc_sstx;
	uint8_t 				          ofdmc_lfo	: 1;
	uint8_t					          ofdmc_poi	: 1;
	at86rf215_ofdm_opt_t  	          ofdmc_opt;
	uint8_t					          ofdmsw_pdt : 3;
	uint8_t					          ofdmsw_rxo : 1;
} rf09_t;

static void parse_rf_config_json(rf09_t *data)
{
    int total_row = sizeof(rf_config_string) / sizeof(rf_config_string[0]);
    int temp_array[total_row];

    FILE* file = fopen("rf_config.json", "r");
    if (file == NULL)
    {
        printf("Failed to open data.json\n");
        return;
    }

    fseek(file, 0, SEEK_END);
    long file_size = ftell(file);
    fseek(file, 0, SEEK_SET);
    char* jsondata = (char*)malloc(file_size + 1);
    fread(jsondata, sizeof(char), file_size, file);
    jsondata[file_size] = '\0';
    fclose(file);

    cJSON* root = cJSON_Parse(jsondata);
    if (root == NULL)
    {
        printf("Failed to parse the JSON data.\n");
        free(jsondata);
        return;
    }

    for (int i=0; i<total_row; i++)
    {
        cJSON* value = cJSON_GetObjectItemCaseSensitive(root, rf_config_string[i]);
        if (cJSON_IsNumber(value))
        {
            temp_array[i] = value->valueint;
        }
    }

    int size = 0;
    data->radio_mode = (at86rf215_radio_t)temp_array[size++];
    DEBUG_PRINT("radio_mode = %d\n", data->radio_mode);

    data->clko_os = (at86rf215_rf_clko_os_t)temp_array[size++];
	DEBUG_PRINT("clko_os = %d\n", data->clko_os);
    
    data->clko_drv = (at86rf215_rf_clko_drv_t)temp_array[size++];
	DEBUG_PRINT("clko_drv = %d\n", data->clko_drv);
    
    data->rf_femode_09 = (at86rf215_external_rffe_control_t)temp_array[size++];
	DEBUG_PRINT("rf_femode_09 = %d\n", data->rf_femode_09);
    
    data->rf_femode_24 = (at86rf215_external_rffe_control_t)temp_array[size++];
	DEBUG_PRINT("rf_femode_24 = %d\n", data->rf_femode_24);
    
    data->xoc_fs = (uint8_t)temp_array[size++];
	DEBUG_PRINT("xoc_fs = %d\n", data->xoc_fs);
    
    data->xoc_trim = (uint8_t)temp_array[size++];
	DEBUG_PRINT("xoc_trim = %d\n", data->xoc_trim);
    
    data->cfg_irqmm = (uint8_t)temp_array[size++];
	DEBUG_PRINT("cfg_irqmm = %d\n", data->cfg_irqmm);
    
    data->cfg_irqp = (uint8_t)temp_array[size++];
	DEBUG_PRINT("cfg_irqp = %d\n", data->cfg_irqp);
    
    data->cfg_pad_drv = (at86rf215_drv_t)temp_array[size++];
    DEBUG_PRINT("cfg_pad_drv = %d\n", data->cfg_pad_drv);
    
    data->chpm_mode = (at86rf215_chpm_t)temp_array[size++];
    DEBUG_PRINT("chpm_mode = %d\n", data->chpm_mode);

    data->cnm_cm = (at86rf215_cm_t)temp_array[size++];
	DEBUG_PRINT("cnm_cm = %d\n", data->cnm_cm);

    data->cs_cs = (uint32_t)temp_array[size++];
	DEBUG_PRINT("cs_cs = %d\n", data->cs_cs);
    
    data->base_freq = (uint32_t)temp_array[size++];
	DEBUG_PRINT("base_freq = %d\n", data->base_freq);
    
    data->pll_lbw = (at86rf215_pll_lbw_t)temp_array[size++];
    DEBUG_PRINT("pll_lbw = %d\n", data->pll_lbw);
    
    data->rf_irqm = (uint8_t)temp_array[size++];
    DEBUG_PRINT("rf_irqm = %d\n", data->rf_irqm);
    
    data->bbc_irqm = (uint8_t)temp_array[size++];
    DEBUG_PRINT("bbc_irqm = %d\n", data->bbc_irqm);
    
    data->cnl_channel = (uint16_t)temp_array[size++];
    DEBUG_PRINT("cnl_channel = %d\n", data->cnl_channel);
    
    data->txcutc_paramp = (at86rf215_paramp_t)temp_array[size++];
    DEBUG_PRINT("txcutc_paramp = %d\n", data->txcutc_paramp);
    
    data->txcutc_lpf = (at86rf215_lpfcut_t)temp_array[size++];
    DEBUG_PRINT("txcutc_lpf = %d\n", data->txcutc_lpf);
    
    data->txdfe_rcut = (uint8_t)temp_array[size++];
    DEBUG_PRINT("txdfe_rcut = %d\n", data->txdfe_rcut);
    
    data->txdfe_dm = (uint8_t)temp_array[size++];
    DEBUG_PRINT("txdfe_dm = %d\n", data->txdfe_dm);
    
    data->txdfe_sr = (at86rf215_sr_t)temp_array[size++];
    DEBUG_PRINT("txdfe_sr = %d\n", data->txdfe_sr);
    
    data->rxdfe_rcut = (uint8_t)temp_array[size++];
    DEBUG_PRINT("rxdfe_rcut = %d\n", data->rxdfe_rcut);
    
    data->rxdfe_sr = (at86rf215_sr_t)temp_array[size++];
    DEBUG_PRINT("rxdfe_sr = %d\n", data->rxdfe_sr);
    
    data->rxbwc_ifi = (uint8_t)temp_array[size++];
    DEBUG_PRINT("rxbwc_ifi = %d\n", data->rxbwc_ifi);
    
    data->rxbwc_ifs = (uint8_t)temp_array[size++];
    DEBUG_PRINT("rxbwc_ifs = %d\n", data->rxbwc_ifs);
    
    data->rxbwc_bw = (at86rf215_rx_bw_t)temp_array[size++];
    DEBUG_PRINT("rxbwc_bw = %d\n", data->rxbwc_bw);
    
    data->pac_pacur = (at86rf215_pacur_t)temp_array[size++];
    DEBUG_PRINT("pac_pacur = %d\n", data->pac_pacur);
    
    data->pac_power = (uint8_t)temp_array[size++];
    DEBUG_PRINT("pac_power = %d\n", data->pac_power);
    
    data->edd_df = (uint8_t)temp_array[size++];
    DEBUG_PRINT("edd_df = %d\n", data->edd_df);
    
    data->edd_dtb = (at86rf215_edd_dtb_t)temp_array[size++];
    DEBUG_PRINT("edd_dtb = %d\n", data->edd_dtb);
    
    data->pc_ctx = (uint8_t)temp_array[size++];
	DEBUG_PRINT("pc_ctx = %d\n", data->pc_ctx);
    
    data->pc_fcsfe = (uint8_t)temp_array[size++];
	DEBUG_PRINT("pc_fcsfe = %d\n", data->pc_fcsfe);
    
    data->pc_txafcs = (uint8_t)temp_array[size++];
	DEBUG_PRINT("pc_txafcs = %d\n", data->pc_txafcs);
    
    data->pc_fcst = (at86rf215_fcs_t)temp_array[size++];
    DEBUG_PRINT("pc_fcst = %d\n", data->pc_fcst);
    
    data->pc_bben = (uint8_t)temp_array[size++];
	DEBUG_PRINT("pc_bben = %d\n", data->pc_bben);
    
    data->pc_pt = (at86rf215_phy_t)temp_array[size++];
    DEBUG_PRINT("pc_pt = %d\n", data->pc_pt);
    
    data->ofdmphrtx_mcs = (at86rf215_ofdm_tx_mcs_t)temp_array[size++];
	DEBUG_PRINT("ofdmphrtx_mcs = %d\n", data->ofdmphrtx_mcs);
    
    data->ofdmc_ssrx = (at86rf215_ofdm_ssrx_t)temp_array[size++];
	DEBUG_PRINT("ofdmc_ssrx = %d\n", data->ofdmc_ssrx);
    
    data->ofdmc_sstx = (at86rf215_ofdm_sstx_t)temp_array[size++];
	DEBUG_PRINT("ofdmc_sstx = %d\n", data->ofdmc_sstx);
    
    data->ofdmc_lfo = (uint8_t)temp_array[size++];
	DEBUG_PRINT("ofdmc_lfo = %d\n", data->ofdmc_lfo);
    
    data->ofdmc_poi	= (uint8_t)temp_array[size++];
	DEBUG_PRINT("ofdmc_poi = %d\n", data->ofdmc_poi);
    
    data->ofdmc_opt = (at86rf215_ofdm_opt_t)temp_array[size++];
	DEBUG_PRINT("ofdmc_opt = %d\n", data->ofdmc_opt);
    
    data->ofdmsw_pdt = (uint8_t)temp_array[size++];
	DEBUG_PRINT("ofdmsw_pdt = %d\n", data->ofdmsw_pdt);
    
    data->ofdmsw_rxo = (uint8_t)temp_array[size++];
    DEBUG_PRINT("ofdmsw_rxo = %d\n", data->ofdmsw_rxo);

    cJSON_Delete(root);
    free(jsondata);
}

static int config_rfic_mode_bbrf_via_spi(struct at86rf215* rfic_dev, rf09_t *data)
{
    int ret;
    at86rf215_rf_state_t radio_state;
    spi_t spi;

    ret = spi_init(&spi, SPI_DEVICE, 0, 0, 2500000);
    if (ret)
    {
        return -1;
    }

    rfic_dev->clko_os = data->clko_os;
    rfic_dev->clk_drv = data->clko_drv;
    rfic_dev->rf_femode_09 = data->rf_femode_09;
    rfic_dev->rf_femode_24= data->rf_femode_24;
    rfic_dev->xo_fs = data->xoc_fs;
    rfic_dev->xo_trim = data->xoc_trim;
    rfic_dev->irqmm = data->cfg_irqmm;
    rfic_dev->irqp = data->cfg_irqp;
    rfic_dev->pad_drv = data->cfg_pad_drv;

    ret = at86rf215_init(rfic_dev, &spi);
    if (ret)
    {
        printf("Failed to init at86rf215 RFIC.\n");
        return -1;
    }

    at86rf215_set_cmd(rfic_dev, AT86RF215_STATE_RF_TRXOFF, AT86RF215_RF09);
    at86rf215_get_state(rfic_dev, &radio_state, AT86RF215_RF09);
    printf("radio state = %d\n", radio_state);

    at86rf215_set_mode(rfic_dev, data->chpm_mode);

    // Setup OFDM
    struct at86rf215_radio_conf radio_conf = {
        .cm = data->cnm_cm,
	    .cs = data->cs_cs,
	    .base_freq = data->base_freq,
	    .lbw = data->pll_lbw,
    };
    ret = at86rf215_radio_conf(rfic_dev, data->radio_mode, &radio_conf);
    if (ret)
    {
        printf("at86rf215_radio_conf() error.\n");
        return -1;
    }
    ret = at86rf215_set_radio_irq_mask(rfic_dev, data->radio_mode, data->rf_irqm);
    if (ret)
    {
        printf("at86rf215_set_radio_irq_mask() error.\n");
        return -1;
    }
    ret = at86rf215_set_bbc_irq_mask(rfic_dev, data->radio_mode, data->bbc_irqm);
    if (ret)
    {
        printf("at86rf215_set_bbc_irq_mask() error.\n");
        return -1;
    }
    ret = at86rf215_set_channel(rfic_dev, data->radio_mode, data->cnl_channel);
    if (ret)
    {
        printf("at86rf215_set_channel() error.\n");
        return -1;
    }
    ret = at86rf215_set_txcutc(rfic_dev, data->radio_mode, data->txcutc_paramp, data->txcutc_lpf);
    if (ret)
    {
        printf("at86rf215_set_txcutc() error.\n");
        return -1;
    }
    ret = at86rf215_set_txdfe(rfic_dev, data->radio_mode, data->txdfe_rcut, data->txdfe_dm, data->txdfe_sr);
    if (ret)
    {
        printf("at86rf215_set_txdfe() error.\n");
        return -1;
    }
    ret = at86rf215_set_rxdfe(rfic_dev, data->radio_mode, data->rxdfe_rcut, data->rxdfe_sr);
    if (ret)
    {
        printf("at86rf215_set_rxdfe() error.\n");
        return -1;
    }
    ret = at86rf215_set_bw(rfic_dev, data->radio_mode, data->rxbwc_ifi, data->rxbwc_ifs, data->rxbwc_bw);
    if (ret)
    {
        printf("at86rf215_set_bw() error.\n");
        return -1;
    }
    ret = at86rf215_set_pac(rfic_dev, data->radio_mode, data->pac_pacur, data->pac_power);
    if (ret)
    {
        printf("at86rf215_set_pac() error.\n");
        return -1;
    }
    ret = at86rf215_set_edd(rfic_dev, data->radio_mode, data->edd_df, data->edd_dtb);
    if (ret)
    {
        printf("at86rf215_set_edd() error.\n");
        return -1;
    }

    struct at86rf215_bb_conf bb_ofdm_conf = {
        .ctx = data->pc_ctx,
        .fcsfe = data->pc_fcsfe,
        .txafcs = data->pc_txafcs,
        .fcst = data->pc_fcst,
        .pt = data->pc_pt,
        .ofdm = {
            .mcs = data->ofdmphrtx_mcs,
            .ssrx = data->ofdmc_ssrx,
            .sstx = data->ofdmc_sstx,
            .lfo = data->ofdmc_lfo,
            .poi = data->ofdmc_poi,
            .opt = data->ofdmc_opt,
            .pdt = data->ofdmsw_pdt,
            .rxo = data->ofdmsw_rxo,
        }
    };
    ret = at86rf215_bb_conf(rfic_dev, data->radio_mode, &bb_ofdm_conf);
    if (ret)
    {
        printf("at86rf215_bb_conf() error.\n");
        return -1;
    }
    /*
    ret = at86rf215_bb_enable(rfic_dev, data->radio_mode, data->pc_bben);
    if (ret)
    {
        printf("at86rf215_bb_enable() error.\n");
        return -1;
    }
    */

    at86rf215_reg_write_8(rfic_dev, 0xAC, REG_BBC0_AMEDT);
    at86rf215_reg_write_8(rfic_dev, 0x02, REG_BBC0_AMCS);
    at86rf215_bb_enable(rfic_dev, data->radio_mode, 0);
}

static int transmit_frame_via_spi(struct at86rf215* rfic_dev, const rf09_t *data)
{
    int ret;
    at86rf215_rf_state_t radio_state;

   
    at86rf215_set_cmd(rfic_dev, AT86RF215_CMD_RF_TXPREP, data->radio_mode);
    usleep(10000);
    at86rf215_get_state(rfic_dev, &radio_state, data->radio_mode);
    printf("radio state = %d\n", radio_state);

    at86rf215_set_cmd(rfic_dev, AT86RF215_CMD_RF_RX, data->radio_mode);
    usleep(10000);
    at86rf215_get_state(rfic_dev, &radio_state, data->radio_mode);
    printf("radio state = %d\n", radio_state);

    at86rf215_reg_write_8(rfic_dev, 0x01, REG_RF09_EDC);

    printf("Sending TX frame.\n");
    uint8_t tx_data[10];
    memset(tx_data, 0x33, sizeof(tx_data));
    at86rf215_tx_frame_auto_mode(rfic_dev, data->radio_mode, &tx_data[0], 10);

    usleep(100000);
    uint8_t val;
    at86rf215_reg_read_8(rfic_dev, &val, REG_RF09_EDV);
    printf("Energy Value is 0x%x\n", val);

    at86rf215_get_state(rfic_dev, &radio_state, data->radio_mode);

    printf("TX done. state = %d\n", radio_state);

    return 0;
}

static int receive_frame_via_usb(struct at86rf215* rfic_dev, const rf09_t *data)
{
    at86rf215_rf_state_t radio_state;
    //at86rf215_usb_set_cmd(rfic_dev, AT86RF215_CMD_RF_TRXOFF, data->radio_mode);
    //at86rf215_usb_get_state(rfic_dev, &radio_state, data->radio_mode);
    //printf("radio state = %d\n", radio_state);

    at86rf215_usb_reg_write_8(0x04, REG_BBC0_AMAACKTH);
    at86rf215_usb_reg_write_8(0x38, REG_BBC0_AMAACKTL);

    at86rf215_usb_set_cmd(rfic_dev, AT86RF215_CMD_RF_RX, data->radio_mode);
    at86rf215_usb_set_cmd(rfic_dev, AT86RF215_CMD_RF_RX, data->radio_mode);
    at86rf215_usb_get_state(rfic_dev, &radio_state, data->radio_mode);
    printf("radio state = %d\n", radio_state);

    return 0;
}

static int config_rfic_mode_bbrf_via_usb(struct at86rf215* rfic_dev, rf09_t *data)
{
    int ret;
    at86rf215_rf_state_t radio_state;

    rfic_dev->clko_os = data->clko_os;
    rfic_dev->clk_drv = data->clko_drv;
    rfic_dev->rf_femode_09 = data->rf_femode_09;
    rfic_dev->rf_femode_24= data->rf_femode_24;
    rfic_dev->xo_fs = data->xoc_fs;
    rfic_dev->xo_trim = data->xoc_trim;
    rfic_dev->irqmm = data->cfg_irqmm;
    rfic_dev->irqp = data->cfg_irqp;
    rfic_dev->pad_drv = data->cfg_pad_drv;

    ret = at86rf215_init_usb(rfic_dev);
    if (ret)
    {
        printf("Failed to init usb at86rf215 RFIC.\n");
        return -1;
    }

    at86rf215_usb_set_cmd(rfic_dev, AT86RF215_STATE_RF_TRXOFF, AT86RF215_RF09);
    at86rf215_usb_get_state(rfic_dev, &radio_state, AT86RF215_RF09);
    printf("radio state = %d\n", radio_state);

    at86rf215_usb_set_mode(rfic_dev, data->chpm_mode);

    // Setup OFDM
    struct at86rf215_radio_conf radio_conf = {
        .cm = data->cnm_cm,
	    .cs = data->cs_cs,
	    .base_freq = data->base_freq,
	    .lbw = data->pll_lbw,
    };
    ret = at86rf215_usb_radio_conf(rfic_dev, data->radio_mode, &radio_conf);
    if (ret)
    {
        printf("at86rf215_radio_conf() error.\n");
        return -1;
    }
    ret = at86rf215_usb_set_radio_irq_mask(rfic_dev, data->radio_mode, data->rf_irqm);
    if (ret)
    {
        printf("at86rf215_set_radio_irq_mask() error.\n");
        return -1;
    }
    ret = at86rf215_usb_set_bbc_irq_mask(rfic_dev, data->radio_mode, data->bbc_irqm);
    if (ret)
    {
        printf("at86rf215_set_bbc_irq_mask() error.\n");
        return -1;
    }
    ret = at86rf215_usb_set_channel(rfic_dev, data->radio_mode, data->cnl_channel);
    if (ret)
    {
        printf("at86rf215_set_channel() error.\n");
        return -1;
    }
    ret = at86rf215_usb_set_txcutc(rfic_dev, data->radio_mode, data->txcutc_paramp, data->txcutc_lpf);
    if (ret)
    {
        printf("at86rf215_set_txcutc() error.\n");
        return -1;
    }
    ret = at86rf215_usb_set_txdfe(rfic_dev, data->radio_mode, data->txdfe_rcut, data->txdfe_dm, data->txdfe_sr);
    if (ret)
    {
        printf("at86rf215_set_txdfe() error.\n");
        return -1;
    }
    ret = at86rf215_usb_set_rxdfe(rfic_dev, data->radio_mode, data->rxdfe_rcut, data->rxdfe_sr);
    if (ret)
    {
        printf("at86rf215_set_rxdfe() error.\n");
        return -1;
    }
    ret = at86rf215_usb_set_bw(rfic_dev, data->radio_mode, data->rxbwc_ifi, data->rxbwc_ifs, data->rxbwc_bw);
    if (ret)
    {
        printf("at86rf215_set_bw() error.\n");
        return -1;
    }
    ret = at86rf215_usb_set_pac(rfic_dev, data->radio_mode, data->pac_pacur, data->pac_power);
    if (ret)
    {
        printf("at86rf215_set_pac() error.\n");
        return -1;
    }
    ret = at86rf215_usb_set_edd(rfic_dev, data->radio_mode, data->edd_df, data->edd_dtb);
    if (ret)
    {
        printf("at86rf215_set_edd() error.\n");
        return -1;
    }

    struct at86rf215_bb_conf bb_ofdm_conf = {
        .ctx = data->pc_ctx,
        .fcsfe = data->pc_fcsfe,
        .txafcs = data->pc_txafcs,
        .fcst = data->pc_fcst,
        .pt = data->pc_pt,
        .ofdm = {
            .mcs = data->ofdmphrtx_mcs,
            .ssrx = data->ofdmc_ssrx,
            .sstx = data->ofdmc_sstx,
            .lfo = data->ofdmc_lfo,
            .poi = data->ofdmc_poi,
            .opt = data->ofdmc_opt,
            .pdt = data->ofdmsw_pdt,
            .rxo = data->ofdmsw_rxo,
        }
    };
    ret = at86rf215_usb_bb_conf(rfic_dev, data->radio_mode, &bb_ofdm_conf);
    if (ret)
    {
        printf("at86rf215_bb_conf() error.\n");
        return -1;
    }
    ret = at86rf215_usb_bb_enable(rfic_dev, data->radio_mode, data->pc_bben);
    if (ret)
    {
        printf("at86rf215_bb_enable() error.\n");
        return -1;
    }
}


int main (void)
{
    int ret = 0;
    rf09_t* data;
    struct at86rf215 rfic_dev1, rfic_dev2;
    memset(&rfic_dev1, 0, sizeof(struct at86rf215));
    memset(&rfic_dev2, 0, sizeof(struct at86rf215));
    data = (rf09_t *)malloc(sizeof(rf09_t));

    parse_rf_config_json(data);

    // Receiver
    // Setup GPIO interrupt
    ret = io_utils_setup_interrupt_bb2(GPIO_CHIP_NAME, GPIOD_CTXLESS_EVENT_RISING_EDGE,
                                    GPIO_PIN2_NUM, false, 
                                    "gpio_handler2", 
                                    at86rf215_usb_irq_callback,
                                    &rfic_dev2,
                                    GPIOD_CTXLESS_FLAG_BIAS_PULL_UP
                                    );
    if (ret)
    {
        printf("io_utils_setup_interrupt() error.\n");
        return -1;
    }
    config_rfic_mode_bbrf_via_usb(&rfic_dev2, data);
    receive_frame_via_usb(&rfic_dev2, data);

    usleep(1000000);

    // Transmitter
    // Setup GPIO interrupt
    ret = io_utils_setup_interrupt_bb(GPIO_CHIP_NAME, GPIOD_CTXLESS_EVENT_RISING_EDGE,
                                    GPIO_PIN_NUM, false, 
                                    "gpio_handler", 
                                    at86rf215_irq_callback,
                                    &rfic_dev1,
                                    GPIOD_CTXLESS_FLAG_BIAS_PULL_UP
                                    );
    if (ret)
    {
        printf("io_utils_setup_interrupt() error.\n");
        return -1;
    }
    config_rfic_mode_bbrf_via_spi(&rfic_dev1, data);
    transmit_frame_via_spi(&rfic_dev1, data);

    printf("Waiting Interrupt or Press enter to stop.\n");
    getchar();

    uint8_t* recvData;
    at86rf215_usb_rx_frame(&rfic_dev2, data->radio_mode ,recvData);

    io_utils_destroy_interrupt(rfic_dev1.irq_tid, rfic_dev1.irq_data);
    io_utils_destroy_interrupt(rfic_dev2.irq_tid,  rfic_dev2.irq_data);
    free(data);
    close_usb();

    return 0;
}