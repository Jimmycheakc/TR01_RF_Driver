/**
* @file test_app_tr01.c
* @brief This implementation file contains the code for testing tr01.
*
* @author Jimmy Chea
* @date 30/03/2023
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "at86rf215/at86rf215.h"
#include "at86rf215/at86rf215_radio.h"
#include "i2c/i2c.h"
#include "io_utils/io_utils.h"
#include "gpio_connector/gpio_connector.h"
#include "ftd2xx.h"
#include "libft4222.h"
#include "i2c_to_gpio/peripheral_gpio.h"


void rfic_supply_voltage_2_5v_test(void);
void fpga_supply_voltage_1_8v_test(void);
void fpga_supply_voltage_1_2v_test(void);
void fpga_supply_voltage_3_3v_test(void);
void mixer_supply_voltage_3_3v_test(void);
void pa_supply_voltage_8_0v_test(void);
void lna_supply_voltage_8_0v_test(void);

void rfic_current_idle_state_test(void);
void rfic_current_tx_state_test(void);
void rfic_current_rx_state_test(void);
void mixer_current_test(void);

void rfic_tx_power_test(void);

void rfic_rx_sensitivity_test(void);

void freq_on_udc_while_tx_state_test(void);
void freq_on_udc_while_rx_state_test(void);

void adc_while_tx_state_test(void);

void temp_while_tx_state_test(void);

void i2c_test(void);

void spi_test(void);

void usb_test(void);

void rfic_interrupt_test(void);


typedef struct {
    char *name;
    void (*test)(void);
} FunctionEntry;

FunctionEntry functionList[] = {
    // Voltage Measurement Test
    {"rfic_supply_voltage_2_5v",     &rfic_supply_voltage_2_5v_test},
    {"fpga_supply_voltage_1_8v",     &fpga_supply_voltage_1_8v_test},
    {"fpga_supply_voltage_1_2v",     &fpga_supply_voltage_1_2v_test},
    {"fpga_supply_voltage_3_3v",     &fpga_supply_voltage_3_3v_test},
    {"mixer_supply_voltage_3_3v",    &mixer_supply_voltage_3_3v_test},
    {"pa_supply_voltage_8_0v",       &pa_supply_voltage_8_0v_test},
    {"lna_supply_voltage_8_0v",      &lna_supply_voltage_8_0v_test},

    // Current Measurement Test
    {"rfic_current_idle_state",      &rfic_current_idle_state_test},
    {"rfic_current_tx_state",        &rfic_current_tx_state_test},
    {"rfic_current_rx_state",        &rfic_current_rx_state_test},
    {"mixer_current",                &mixer_current_test},

    // RFIC Tx Power Measurement Test
    {"rfic_tx_power",                &rfic_tx_power_test},

    // RFIC Rx Sensitivity Measurement Test
    {"rfic_rx_sensitivity",          &rfic_rx_sensitivity_test},

    // UDC Path Measurement Test
    {"freq_on_udc_while_tx_state",   &freq_on_udc_while_tx_state_test},
    {"freq_on_udc_while_rx_state",   &freq_on_udc_while_rx_state_test},

    // ADC Measurement Test
    {"adc_while_tx_state",           &adc_while_tx_state_test},

    // Temperature Measurement Test
    {"temp_while_tx_state",          &temp_while_tx_state_test},

    // I2C Interface Test
    {"i2c",                          &i2c_test},

    // SPI Interface Test
    {"spi",                          &spi_test},

    // USB Interface Test
    {"usb",                          &usb_test},

    // RFIC Interrupt
    {"rfic_interrupt",               &rfic_interrupt_test},
};

static int mixer_3v3_rfic_2v5_fpga_1v8_power_enabled(void)
{
    int ret;

    printf("Enabled the mixer_3v3, rfic_2v5, fpga_1v8 supply voltage.\n");
    ret = gpio_rbe_enabled();
    if (ret < 0)
    {
        printf("RBE gpio enabled : Failed.\n");
    }
    else
    {
        printf("RBE gpio enabled : Passed.\n");
    }

    return ret;
}

static int mixer_3v3_rfic_2v5_fpga_1v8_power_disabled(void)
{
    int ret;

    printf("Disabled the mixer_3v3, rfic_2v5, fpga_1v8 supply voltage.\n");
    ret = gpio_rbe_disabled();
    if (ret < 0)
    {
        printf("RBE gpio disabled : Failed.\n");
    }
    else
    {
        printf("RBE gpio disabled : Passed.\n");
    }

    return ret;
}

static int fpga_1v2_3v3_power_enabled(void)
{
    int ret;

    printf("Enabled the fpga_1v2 and fpga_3v3 supply voltage.\n");
    ret = gpio_rf_1v2_en_enabled();
    if (ret < 0)
    {
        printf("RF_1V2_EN gpio enabled : Failed.\n");
    }
    else
    {
        printf("RF_1V2_EN gpio enabled : Passed.\n");
    }

    return ret;
}

static int fpga_1v2_3v3_power_disabled(void)
{
    int ret;

    printf("Disabled the fpga_1v2 and fpga_3v3 supply voltage.\n");
    ret = gpio_rf_1v2_en_disabled();
    if (ret < 0)
    {
        printf("RF_1V2_EN gpio disabled : Failed.\n");
    }
    else
    {
        printf("RF_1V2_EN gpio disabled : Passed.\n");
    }

    return ret;
}

static void print_func_start_format(const char* func_name)
{
    printf("\n");
    printf("<<< Start >>> : %s\n", func_name);
    printf("\n");
}

static void print_func_end_format(const char* func_name, int ret)
{
    printf("\n");
    printf("<<< End >>> : Completed testing with %s\n", func_name);
    printf("\n");

    if (ret < 0)
    {
        printf(">>> Test Result : < Failed >\n");
    }
    else
    {
        printf(">>> Test Result : < Passed >\n");
    }

    printf("\n");
}

void rfic_supply_voltage_2_5v_test(void)
{
    int ret = 0;

    print_func_start_format(__func__);

    ret = mixer_3v3_rfic_2v5_fpga_1v8_power_enabled();
    if (ret < 0)
    {
        goto exit;
    }

    printf("\n");
    printf("Measure your rfic supply voltage 2.5v.\n");
    printf("Once completed, press ENTER key to turn off the rfic supply voltage 2.5v.\n");

    getchar();
    ret = mixer_3v3_rfic_2v5_fpga_1v8_power_disabled();

exit:
    print_func_end_format(__func__, ret);
}

void fpga_supply_voltage_1_8v_test(void)
{
    int ret = 0;

    print_func_start_format(__func__);

    ret = mixer_3v3_rfic_2v5_fpga_1v8_power_enabled();
    if (ret < 0)
    {
        goto exit;
    }

    printf("\n");
    printf("Measure your fpga supply voltage 1.8v.\n");
    printf("Once completed, press ENTER key to turn off the fpga supply voltage 1.8v.\n");

    getchar();
    ret = mixer_3v3_rfic_2v5_fpga_1v8_power_disabled();

exit:

    print_func_end_format(__func__, ret);
}

void fpga_supply_voltage_1_2v_test(void)
{
    int ret = 0;

    print_func_start_format(__func__);

    ret = fpga_1v2_3v3_power_enabled();
    if (ret < 0)
    {
        goto exit;
    }

    printf("\n");
    printf("Measure your fpga supply voltage 1.2v.\n");
    printf("Once completed, press ENTER key to turn off the fpga supply voltage 1.2v.\n");

    getchar();
    ret = fpga_1v2_3v3_power_disabled();

exit:

    print_func_end_format(__func__, ret);
}

void fpga_supply_voltage_3_3v_test(void)
{
    int ret = 0;

    print_func_start_format(__func__);

    ret = fpga_1v2_3v3_power_enabled();
    if (ret < 0)
    {
        goto exit;
    }

    printf("\n");
    printf("Measure your fpga supply voltage 3.3v.\n");
    printf("Once completed, press ENTER key to turn off the fpga supply voltage 3.3v.\n");

    getchar();
    ret = fpga_1v2_3v3_power_disabled();

exit:

    print_func_end_format(__func__, ret);
}

void mixer_supply_voltage_3_3v_test(void)
{
    int ret = 0;

    print_func_start_format(__func__);

    ret = mixer_3v3_rfic_2v5_fpga_1v8_power_enabled();
    if (ret < 0)
    {
        goto exit;
    }

    printf("\n");
    printf("Measure your mixer supply voltage 3.3v.\n");
    printf("Once completed, press ENTER key to turn off the mixer supply voltage 3.3v.\n");

    getchar();
    ret = mixer_3v3_rfic_2v5_fpga_1v8_power_disabled();

exit:

    print_func_end_format(__func__, ret);
}

// Temp: Need to revisit
void pa_supply_voltage_8_0v_test(void)
{
    int ret = 0;

    print_func_start_format(__func__);

    peri_gpio_t dev;

    peri_gpio_init(&dev, "dev/i2c-3", I2C_GPIO_ADDRESS);

    ret = peri_gpio_pa_enable(&dev, 1);
    if (ret < 0)
    {
        printf("Peripheral gpio pa enabled : Failed.\n");
        goto exit;
    }
    else
    {
        printf("Peripheral gpio pa enabled : Passed.\n");
    }

    printf("\n");
    printf("Measure your pa supply voltage 8v.\n");
    printf("Once completed, press ENTER key to turn off the peripheral gpio pa.\n");

    getchar();
    printf("Disabled the peripheral gpio pa.\n");
    ret = peri_gpio_pa_enable(&dev, 0);
    if (ret < 0)
    {
        printf("Peripheral gpio pa disabled : Failed.\n");
    }
    else
    {
        printf("Peripheral gpio pa disabled : Passed.\n");
    }

exit:

    peri_gpio_close(&dev);

    print_func_end_format(__func__, ret);
}

// Temp: Need to revisit
void lna_supply_voltage_8_0v_test(void)
{
    int ret = 0;

    print_func_start_format(__func__);

    peri_gpio_t dev;

    peri_gpio_init(&dev, "dev/i2c-3", I2C_GPIO_ADDRESS);

    ret = peri_gpio_lna_enable(&dev, 1);
    if (ret < 0)
    {
        printf("Peripheral gpio lna enabled : Failed.\n");
        goto exit;
    }
    else
    {
        printf("Peripheral gpio lna enabled : Passed.\n");
    }

    printf("\n");
    printf("Measure your lna supply voltage 8v.\n");
    printf("Once completed, press ENTER key to turn off the peripheral gpio lna.\n");

    getchar();
    printf("Disabled the peripheral gpio lna.\n");
    ret = peri_gpio_lna_enable(&dev, 0);
    if (ret < 0)
    {
        printf("Peripheral gpio lna disabled : Failed.\n");
    }
    else
    {
        printf("Peripheral gpio lna disabled : Passed.\n");
    }

exit:

    peri_gpio_close(&dev);

    print_func_end_format(__func__, ret);
}

static int check_rfic_idle_state()
{
    int ret = 0;
    at86rf215_st rfic1, rfic2, rfic3, rfic4;
    at86rf215_radio_state_cmd_en rfic1_900mhz_state, rfic1_2400mhz_state;
    at86rf215_radio_state_cmd_en rfic2_900mhz_state, rfic2_2400mhz_state;
    at86rf215_radio_state_cmd_en rfic3_900mhz_state, rfic3_2400mhz_state;
    at86rf215_radio_state_cmd_en rfic4_900mhz_state, rfic4_2400mhz_state;

    //Temp: To do init rfic1
    //Temp: To do init rfic2
    //Temp: To do init rfic3
    //Temp: To do init rfic4
    at86rf215_init(&rfic1, "/dev/i2c-3", I2C_TO_SPI_SLAVE_ADDRESS, at86rf215_1_ss, "gpiochip5", 11, "gpio_interrupt");
    at86rf215_init(&rfic2, "/dev/i2c-3", I2C_TO_SPI_SLAVE_ADDRESS, at86rf215_2_ss, "gpiochip5", 12, "gpio_interrupt");
    at86rf215_init(&rfic3, "/dev/i2c-3", I2C_TO_SPI_SLAVE_ADDRESS, at86rf215_3_ss, "gpiochip5", 13, "gpio_interrupt");
    at86rf215_init(&rfic4, "/dev/i2c-3", I2C_TO_SPI_SLAVE_ADDRESS, at86rf215_4_ss, "gpiochip5", 14, "gpio_interrupt");

    //Temp: To do wait 5s
    io_utils_usleep(5000000);

    //Temp: To do at86rf215_radio_get_state() 2channels for rfic1, if IDLE, print IDLE, else print its state
    //Temp: To do at86rf215_radio_get_state() 2channels for rfic2, if IDLE, print IDLE, else print its state
    //Temp: To do at86rf215_radio_get_state() 2channels for rfic3, if IDLE, print IDLE, else print its state
    //Temp: To do at86rf215_radio_get_state() 2channels for rfic4, if IDLE, print IDLE, else print its state
    at86rf215_radio_set_state(&rfic1, at86rf215_rf_channel_900mhz, at86rf215_radio_state_cmd_trx_off);
    at86rf215_radio_set_state(&rfic1, at86rf215_rf_channel_2400mhz, at86rf215_radio_state_cmd_trx_off);
    at86rf215_radio_set_state(&rfic2, at86rf215_rf_channel_900mhz, at86rf215_radio_state_cmd_trx_off);
    at86rf215_radio_set_state(&rfic2, at86rf215_rf_channel_2400mhz, at86rf215_radio_state_cmd_trx_off);
    at86rf215_radio_set_state(&rfic3, at86rf215_rf_channel_900mhz, at86rf215_radio_state_cmd_trx_off);
    at86rf215_radio_set_state(&rfic3, at86rf215_rf_channel_2400mhz, at86rf215_radio_state_cmd_trx_off);
    at86rf215_radio_set_state(&rfic4, at86rf215_rf_channel_900mhz, at86rf215_radio_state_cmd_trx_off);
    at86rf215_radio_set_state(&rfic4, at86rf215_rf_channel_2400mhz, at86rf215_radio_state_cmd_trx_off);

    io_utils_usleep(1000000);

    rfic1_900mhz_state = at86rf215_radio_get_state(&rfic1, at86rf215_rf_channel_900mhz);
    rfic1_2400mhz_state = at86rf215_radio_get_state(&rfic1, at86rf215_rf_channel_2400mhz);
    rfic2_900mhz_state = at86rf215_radio_get_state(&rfic1, at86rf215_rf_channel_900mhz);
    rfic2_2400mhz_state = at86rf215_radio_get_state(&rfic1, at86rf215_rf_channel_2400mhz);
    rfic3_900mhz_state = at86rf215_radio_get_state(&rfic1, at86rf215_rf_channel_900mhz);
    rfic3_2400mhz_state = at86rf215_radio_get_state(&rfic1, at86rf215_rf_channel_2400mhz);
    rfic4_900mhz_state = at86rf215_radio_get_state(&rfic1, at86rf215_rf_channel_900mhz);
    rfic4_2400mhz_state = at86rf215_radio_get_state(&rfic1, at86rf215_rf_channel_2400mhz);

    //Temp: To do compare all rfic states, if not IDLE state, then set ret = -1

    if (rfic1_900mhz_state != at86rf215_radio_state_cmd_trx_off &&
        rfic1_2400mhz_state != at86rf215_radio_state_cmd_trx_off &&
        rfic2_900mhz_state != at86rf215_radio_state_cmd_trx_off &&
        rfic2_2400mhz_state != at86rf215_radio_state_cmd_trx_off &&
        rfic3_900mhz_state != at86rf215_radio_state_cmd_trx_off &&
        rfic3_2400mhz_state != at86rf215_radio_state_cmd_trx_off &&
        rfic4_900mhz_state != at86rf215_radio_state_cmd_trx_off &&
        rfic4_2400mhz_state != at86rf215_radio_state_cmd_trx_off
        )
    {
        ret = -1;
    }

    //Temp: To do close rfic1
    //Temp: To do close rfic2
    //Temp: To do close rfic3
    //Temp: To do close rfic4
    at86rf215_close(&rfic1);
    at86rf215_close(&rfic2);
    at86rf215_close(&rfic3);
    at86rf215_close(&rfic4);

    return ret;
}

void rfic_current_idle_state_test(void)
{
    int ret = 0;

    print_func_start_format(__func__);

    ret = mixer_3v3_rfic_2v5_fpga_1v8_power_enabled();
    if (ret < 0)
    {
        goto exit;
    }

    printf("\n");
    printf("Waiting the rfic swith to IDLE state.\n");
    if (!check_rfic_idle_state())
    {
        return;
    }
    
    printf("Measure your 4 rfic current in IDLE state individually.\n");
    printf("Once completed, press ENTER key to turn off the rfic supply voltage 2.5v.\n");

    getchar();
    ret = mixer_3v3_rfic_2v5_fpga_1v8_power_disabled();

exit:

    print_func_end_format(__func__, ret);
}

static int check_rfic_tx_state()
{
    int ret = 0;
    at86rf215_st rfic1, rfic2, rfic3, rfic4;
    at86rf215_radio_state_cmd_en rfic1_900mhz_state, rfic1_2400mhz_state;
    at86rf215_radio_state_cmd_en rfic2_900mhz_state, rfic2_2400mhz_state;
    at86rf215_radio_state_cmd_en rfic3_900mhz_state, rfic3_2400mhz_state;
    at86rf215_radio_state_cmd_en rfic4_900mhz_state, rfic4_2400mhz_state;

    //Temp: To do init rfic1
    //Temp: To do init rfic2
    //Temp: To do init rfic3
    //Temp: To do init rfic4
    at86rf215_init(&rfic1, "/dev/i2c-3", I2C_TO_SPI_SLAVE_ADDRESS, at86rf215_1_ss, "gpiochip5", 11, "gpio_interrupt");
    at86rf215_init(&rfic2, "/dev/i2c-3", I2C_TO_SPI_SLAVE_ADDRESS, at86rf215_2_ss, "gpiochip5", 12, "gpio_interrupt");
    at86rf215_init(&rfic3, "/dev/i2c-3", I2C_TO_SPI_SLAVE_ADDRESS, at86rf215_3_ss, "gpiochip5", 13, "gpio_interrupt");
    at86rf215_init(&rfic4, "/dev/i2c-3", I2C_TO_SPI_SLAVE_ADDRESS, at86rf215_4_ss, "gpiochip5", 14, "gpio_interrupt");

    //Temp: To do wait 5s
    io_utils_usleep(5000000);

    //Temp: To do at86rf215_radio_set_state() 2channels to TX for rfic1, if TX, print TX, else print its state
    //Temp: To do at86rf215_radio_set_state() 2channels to TX for rfic2, if TX, print TX, else print its state
    //Temp: To do at86rf215_radio_set_state() 2channels to TX for rfic3, if TX, print TX, else print its state
    //Temp: To do at86rf215_radio_set_state() 2channels to TX for rfic4, if TX, print TX, else print its state
    at86rf215_radio_set_state(&rfic1, at86rf215_rf_channel_900mhz, at86rf215_radio_state_cmd_trx_off);
    at86rf215_radio_set_state(&rfic1, at86rf215_rf_channel_2400mhz, at86rf215_radio_state_cmd_trx_off);
    at86rf215_radio_set_state(&rfic2, at86rf215_rf_channel_900mhz, at86rf215_radio_state_cmd_trx_off);
    at86rf215_radio_set_state(&rfic2, at86rf215_rf_channel_2400mhz, at86rf215_radio_state_cmd_trx_off);
    at86rf215_radio_set_state(&rfic3, at86rf215_rf_channel_900mhz, at86rf215_radio_state_cmd_trx_off);
    at86rf215_radio_set_state(&rfic3, at86rf215_rf_channel_2400mhz, at86rf215_radio_state_cmd_trx_off);
    at86rf215_radio_set_state(&rfic4, at86rf215_rf_channel_900mhz, at86rf215_radio_state_cmd_trx_off);
    at86rf215_radio_set_state(&rfic4, at86rf215_rf_channel_2400mhz, at86rf215_radio_state_cmd_trx_off);

    io_utils_usleep(1000000);

    at86rf215_radio_set_state(&rfic1, at86rf215_rf_channel_900mhz, at86rf215_radio_state_cmd_tx_prep);
    at86rf215_radio_set_state(&rfic1, at86rf215_rf_channel_2400mhz, at86rf215_radio_state_cmd_tx_prep);
    at86rf215_radio_set_state(&rfic2, at86rf215_rf_channel_900mhz, at86rf215_radio_state_cmd_tx_prep);
    at86rf215_radio_set_state(&rfic2, at86rf215_rf_channel_2400mhz, at86rf215_radio_state_cmd_tx_prep);
    at86rf215_radio_set_state(&rfic3, at86rf215_rf_channel_900mhz, at86rf215_radio_state_cmd_tx_prep);
    at86rf215_radio_set_state(&rfic3, at86rf215_rf_channel_2400mhz, at86rf215_radio_state_cmd_tx_prep);
    at86rf215_radio_set_state(&rfic4, at86rf215_rf_channel_900mhz, at86rf215_radio_state_cmd_tx_prep);
    at86rf215_radio_set_state(&rfic4, at86rf215_rf_channel_2400mhz, at86rf215_radio_state_cmd_tx_prep);

    io_utils_usleep(1000000);

    at86rf215_radio_set_state(&rfic1, at86rf215_rf_channel_900mhz, at86rf215_radio_state_cmd_tx);
    at86rf215_radio_set_state(&rfic1, at86rf215_rf_channel_2400mhz, at86rf215_radio_state_cmd_tx);
    at86rf215_radio_set_state(&rfic2, at86rf215_rf_channel_900mhz, at86rf215_radio_state_cmd_tx);
    at86rf215_radio_set_state(&rfic2, at86rf215_rf_channel_2400mhz, at86rf215_radio_state_cmd_tx);
    at86rf215_radio_set_state(&rfic3, at86rf215_rf_channel_900mhz, at86rf215_radio_state_cmd_tx);
    at86rf215_radio_set_state(&rfic3, at86rf215_rf_channel_2400mhz, at86rf215_radio_state_cmd_tx);
    at86rf215_radio_set_state(&rfic4, at86rf215_rf_channel_900mhz, at86rf215_radio_state_cmd_tx);
    at86rf215_radio_set_state(&rfic4, at86rf215_rf_channel_2400mhz, at86rf215_radio_state_cmd_tx);

    io_utils_usleep(1000000);

    //Temp: To do at86rf215_radio_get_state() 2channels for rfic1, if TX, print TX, else print its state
    //Temp: To do at86rf215_radio_get_state() 2channels for rfic2, if TX, print TX, else print its state
    //Temp: To do at86rf215_radio_get_state() 2channels for rfic3, if TX, print TX, else print its state
    //Temp: To do at86rf215_radio_get_state() 2channels for rfic4, if TX, print TX, else print its state
    rfic1_900mhz_state = at86rf215_radio_get_state(&rfic1, at86rf215_rf_channel_900mhz);
    rfic1_2400mhz_state = at86rf215_radio_get_state(&rfic1, at86rf215_rf_channel_2400mhz);
    rfic2_900mhz_state = at86rf215_radio_get_state(&rfic1, at86rf215_rf_channel_900mhz);
    rfic2_2400mhz_state = at86rf215_radio_get_state(&rfic1, at86rf215_rf_channel_2400mhz);
    rfic3_900mhz_state = at86rf215_radio_get_state(&rfic1, at86rf215_rf_channel_900mhz);
    rfic3_2400mhz_state = at86rf215_radio_get_state(&rfic1, at86rf215_rf_channel_2400mhz);
    rfic4_900mhz_state = at86rf215_radio_get_state(&rfic1, at86rf215_rf_channel_900mhz);
    rfic4_2400mhz_state = at86rf215_radio_get_state(&rfic1, at86rf215_rf_channel_2400mhz);

    //Temp: To do compare all rfic states, if not TX state, then set ret = -1
    if (rfic1_900mhz_state != at86rf215_radio_state_cmd_tx &&
        rfic1_2400mhz_state != at86rf215_radio_state_cmd_tx &&
        rfic2_900mhz_state != at86rf215_radio_state_cmd_tx &&
        rfic2_2400mhz_state != at86rf215_radio_state_cmd_tx &&
        rfic3_900mhz_state != at86rf215_radio_state_cmd_tx &&
        rfic3_2400mhz_state != at86rf215_radio_state_cmd_tx &&
        rfic4_900mhz_state != at86rf215_radio_state_cmd_tx &&
        rfic4_2400mhz_state != at86rf215_radio_state_cmd_tx
        )
    {
        ret = -1;
    }

    //Temp: To do close rfic1
    //Temp: To do close rfic2
    //Temp: To do close rfic3
    //Temp: To do close rfic4
    at86rf215_close(&rfic1);
    at86rf215_close(&rfic2);
    at86rf215_close(&rfic3);
    at86rf215_close(&rfic4);

    return ret;
}

void rfic_current_tx_state_test(void)
{
    int ret = 0;

    print_func_start_format(__func__);

    ret = mixer_3v3_rfic_2v5_fpga_1v8_power_enabled();
    if (ret < 0)
    {
        goto exit;
    }

    printf("\n");
    printf("Waiting the rfic swith to TX state.\n");
    if (!check_rfic_tx_state())
    {
        return;
    }
    
    printf("Measure your 4 rfic current in TX state individually.\n");
    printf("Once completed, press ENTER key to turn off the rfic supply voltage 2.5v.\n");

    getchar();
    ret = mixer_3v3_rfic_2v5_fpga_1v8_power_disabled();

exit:

    print_func_end_format(__func__, ret);
}

static int check_rfic_rx_state()
{
    int ret = 0;
    at86rf215_st rfic1, rfic2, rfic3, rfic4;
    at86rf215_radio_state_cmd_en rfic1_900mhz_state, rfic1_2400mhz_state;
    at86rf215_radio_state_cmd_en rfic2_900mhz_state, rfic2_2400mhz_state;
    at86rf215_radio_state_cmd_en rfic3_900mhz_state, rfic3_2400mhz_state;
    at86rf215_radio_state_cmd_en rfic4_900mhz_state, rfic4_2400mhz_state;

    //Temp: To do init rfic1
    //Temp: To do init rfic2
    //Temp: To do init rfic3
    //Temp: To do init rfic4
    at86rf215_init(&rfic1, "/dev/i2c-3", I2C_TO_SPI_SLAVE_ADDRESS, at86rf215_1_ss, "gpiochip5", 11, "gpio_interrupt");
    at86rf215_init(&rfic2, "/dev/i2c-3", I2C_TO_SPI_SLAVE_ADDRESS, at86rf215_2_ss, "gpiochip5", 12, "gpio_interrupt");
    at86rf215_init(&rfic3, "/dev/i2c-3", I2C_TO_SPI_SLAVE_ADDRESS, at86rf215_3_ss, "gpiochip5", 13, "gpio_interrupt");
    at86rf215_init(&rfic4, "/dev/i2c-3", I2C_TO_SPI_SLAVE_ADDRESS, at86rf215_4_ss, "gpiochip5", 14, "gpio_interrupt");

    //Temp: To do wait 5s
    io_utils_usleep(5000000);

    //Temp: To do at86rf215_radio_set_state() 2channels to RX for rfic1, if RX, print RX, else print its state
    //Temp: To do at86rf215_radio_set_state() 2channels to RX for rfic2, if RX, print RX, else print its state
    //Temp: To do at86rf215_radio_set_state() 2channels to RX for rfic3, if RX, print RX, else print its state
    //Temp: To do at86rf215_radio_set_state() 2channels to RX for rfic4, if RX, print RX, else print its state
    at86rf215_radio_set_state(&rfic1, at86rf215_rf_channel_900mhz, at86rf215_radio_state_cmd_trx_off);
    at86rf215_radio_set_state(&rfic1, at86rf215_rf_channel_2400mhz, at86rf215_radio_state_cmd_trx_off);
    at86rf215_radio_set_state(&rfic2, at86rf215_rf_channel_900mhz, at86rf215_radio_state_cmd_trx_off);
    at86rf215_radio_set_state(&rfic2, at86rf215_rf_channel_2400mhz, at86rf215_radio_state_cmd_trx_off);
    at86rf215_radio_set_state(&rfic3, at86rf215_rf_channel_900mhz, at86rf215_radio_state_cmd_trx_off);
    at86rf215_radio_set_state(&rfic3, at86rf215_rf_channel_2400mhz, at86rf215_radio_state_cmd_trx_off);
    at86rf215_radio_set_state(&rfic4, at86rf215_rf_channel_900mhz, at86rf215_radio_state_cmd_trx_off);
    at86rf215_radio_set_state(&rfic4, at86rf215_rf_channel_2400mhz, at86rf215_radio_state_cmd_trx_off);

    io_utils_usleep(1000000);

    at86rf215_radio_set_state(&rfic1, at86rf215_rf_channel_900mhz, at86rf215_radio_state_cmd_tx_prep);
    at86rf215_radio_set_state(&rfic1, at86rf215_rf_channel_2400mhz, at86rf215_radio_state_cmd_tx_prep);
    at86rf215_radio_set_state(&rfic2, at86rf215_rf_channel_900mhz, at86rf215_radio_state_cmd_tx_prep);
    at86rf215_radio_set_state(&rfic2, at86rf215_rf_channel_2400mhz, at86rf215_radio_state_cmd_tx_prep);
    at86rf215_radio_set_state(&rfic3, at86rf215_rf_channel_900mhz, at86rf215_radio_state_cmd_tx_prep);
    at86rf215_radio_set_state(&rfic3, at86rf215_rf_channel_2400mhz, at86rf215_radio_state_cmd_tx_prep);
    at86rf215_radio_set_state(&rfic4, at86rf215_rf_channel_900mhz, at86rf215_radio_state_cmd_tx_prep);
    at86rf215_radio_set_state(&rfic4, at86rf215_rf_channel_2400mhz, at86rf215_radio_state_cmd_tx_prep);

    io_utils_usleep(1000000);

    at86rf215_radio_set_state(&rfic1, at86rf215_rf_channel_900mhz, at86rf215_radio_state_cmd_rx);
    at86rf215_radio_set_state(&rfic1, at86rf215_rf_channel_2400mhz, at86rf215_radio_state_cmd_rx);
    at86rf215_radio_set_state(&rfic2, at86rf215_rf_channel_900mhz, at86rf215_radio_state_cmd_rx);
    at86rf215_radio_set_state(&rfic2, at86rf215_rf_channel_2400mhz, at86rf215_radio_state_cmd_rx);
    at86rf215_radio_set_state(&rfic3, at86rf215_rf_channel_900mhz, at86rf215_radio_state_cmd_rx);
    at86rf215_radio_set_state(&rfic3, at86rf215_rf_channel_2400mhz, at86rf215_radio_state_cmd_rx);
    at86rf215_radio_set_state(&rfic4, at86rf215_rf_channel_900mhz, at86rf215_radio_state_cmd_rx);
    at86rf215_radio_set_state(&rfic4, at86rf215_rf_channel_2400mhz, at86rf215_radio_state_cmd_rx);

    io_utils_usleep(1000000);

    //Temp: To do at86rf215_radio_get_state() 2channels for rfic1, if RX, print RX, else print its state
    //Temp: To do at86rf215_radio_get_state() 2channels for rfic2, if RX, print RX, else print its state
    //Temp: To do at86rf215_radio_get_state() 2channels for rfic3, if RX, print RX, else print its state
    //Temp: To do at86rf215_radio_get_state() 2channels for rfic4, if RX, print RX, else print its state
    rfic1_900mhz_state = at86rf215_radio_get_state(&rfic1, at86rf215_rf_channel_900mhz);
    rfic1_2400mhz_state = at86rf215_radio_get_state(&rfic1, at86rf215_rf_channel_2400mhz);
    rfic2_900mhz_state = at86rf215_radio_get_state(&rfic1, at86rf215_rf_channel_900mhz);
    rfic2_2400mhz_state = at86rf215_radio_get_state(&rfic1, at86rf215_rf_channel_2400mhz);
    rfic3_900mhz_state = at86rf215_radio_get_state(&rfic1, at86rf215_rf_channel_900mhz);
    rfic3_2400mhz_state = at86rf215_radio_get_state(&rfic1, at86rf215_rf_channel_2400mhz);
    rfic4_900mhz_state = at86rf215_radio_get_state(&rfic1, at86rf215_rf_channel_900mhz);
    rfic4_2400mhz_state = at86rf215_radio_get_state(&rfic1, at86rf215_rf_channel_2400mhz);

    //Temp: To do compare all rfic states, if not RX state, then set ret = -1

    //Temp: To do close rfic1
    //Temp: To do close rfic2
    //Temp: To do close rfic3
    //Temp: To do close rfic4
    if (rfic1_900mhz_state != at86rf215_radio_state_cmd_rx &&
        rfic1_2400mhz_state != at86rf215_radio_state_cmd_rx &&
        rfic2_900mhz_state != at86rf215_radio_state_cmd_rx &&
        rfic2_2400mhz_state != at86rf215_radio_state_cmd_rx &&
        rfic3_900mhz_state != at86rf215_radio_state_cmd_rx &&
        rfic3_2400mhz_state != at86rf215_radio_state_cmd_rx &&
        rfic4_900mhz_state != at86rf215_radio_state_cmd_rx &&
        rfic4_2400mhz_state != at86rf215_radio_state_cmd_rx
        )
    {
        ret = -1;
    }

    return ret;
}

void rfic_current_rx_state_test(void)
{
    int ret = 0;

    print_func_start_format(__func__);

    ret = mixer_3v3_rfic_2v5_fpga_1v8_power_enabled();
    if (ret < 0)
    {
        goto exit;
    }

    printf("\n");
    printf("Waiting the rfic swith to TX state.\n");
    if (!check_rfic_rx_state())
    {
        return;
    }
    
    printf("Measure your 4 rfic current in TX state individually.\n");
    printf("Once completed, press ENTER key to turn off the rfic supply voltage 2.5v.\n");

    getchar();
    ret = mixer_3v3_rfic_2v5_fpga_1v8_power_disabled();

exit:

    print_func_end_format(__func__, ret);
}

void mixer_current_test(void)
{
    int ret = 0;

    print_func_start_format(__func__);

    ret = mixer_3v3_rfic_2v5_fpga_1v8_power_enabled();
    if (ret < 0)
    {
        goto exit;
    }

    printf("\n");
    printf("Measure your mixer current.\n");
    printf("Once completed, press ENTER key to turn off the mixer supply voltage 3.3v.\n");

    getchar();
    ret = mixer_3v3_rfic_2v5_fpga_1v8_power_disabled();

exit:

    print_func_end_format(__func__, ret);
}

void rfic_tx_power_test(void)
{
    int ret = 0;

    print_func_start_format(__func__);

    ret = mixer_3v3_rfic_2v5_fpga_1v8_power_enabled();
    if (ret < 0)
    {
        goto exit;
    }

    printf("\n");
    printf("Waiting the rfic swith to TX state.\n");
    if (!check_rfic_tx_state())
    {
        return;
    }
    
    //Temp: To do setup the TX power for 4 RFIC
    
    printf("Measure your 4 rfic TX power in TX state individually.\n");
    printf("Once completed, press ENTER key to turn off the rfic supply voltage 2.5v.\n");

    getchar();
    ret = mixer_3v3_rfic_2v5_fpga_1v8_power_disabled();

exit:

    print_func_end_format(__func__, ret);
}

//Temp: Need to revisit
void rfic_rx_sensitivity_test(void)
{
    int ret = 0;

    print_func_start_format(__func__);

    ret = mixer_3v3_rfic_2v5_fpga_1v8_power_enabled();
    if (ret < 0)
    {
        goto exit;
    }

    printf("\n");
    printf("Waiting the rfic swith to RX state.\n");
    if (!check_rfic_rx_state())
    {
        return;
    }
    
    printf("Measure your 4 rfic RX sensitivity individually.\n");

    //Temp: To do ???

    printf("Once completed, press ENTER key to turn off the rfic supply voltage 2.5v.\n");

    getchar();
    ret = mixer_3v3_rfic_2v5_fpga_1v8_power_disabled();

exit:

    print_func_end_format(__func__, ret);
}

void freq_on_udc_while_tx_state_test(void)
{
    int ret = 0;

    print_func_start_format(__func__);

    ret = mixer_3v3_rfic_2v5_fpga_1v8_power_enabled();
    if (ret < 0)
    {
        goto exit;
    }

    printf("\n");
    printf("Waiting the rfic swith to TX state.\n");
    if (!check_rfic_tx_state())
    {
        return;
    }
    
    printf("Measure your frequency on UDC while in TX state.\n");
    
    //Temp: To do ????

    printf("Once completed, press ENTER key to turn off the rfic supply voltage 2.5v.\n");

    getchar();
    ret = mixer_3v3_rfic_2v5_fpga_1v8_power_disabled();

exit:

    print_func_end_format(__func__, ret);
}

void freq_on_udc_while_rx_state_test(void)
{
    int ret = 0;

    print_func_start_format(__func__);

    ret = mixer_3v3_rfic_2v5_fpga_1v8_power_enabled();
    if (ret < 0)
    {
        goto exit;
    }

    printf("\n");
    printf("Waiting the rfic swith to RX state.\n");
    if (!check_rfic_rx_state())
    {
        return;
    }
    
    printf("Measure your frequency on UDS while in RX state individually.\n");

    //Temp: To do ???

    printf("Once completed, press ENTER key to turn off the rfic supply voltage 2.5v.\n");

    getchar();
    ret = mixer_3v3_rfic_2v5_fpga_1v8_power_disabled();

exit:

    print_func_end_format(__func__, ret);
}

void adc_while_tx_state_test(void)
{
    int ret = 0;

    print_func_start_format(__func__);

    ret = mixer_3v3_rfic_2v5_fpga_1v8_power_enabled();
    if (ret < 0)
    {
        goto exit;
    }

    printf("\n");
    printf("Waiting the rfic swith to TX state.\n");
    if (!check_rfic_tx_state())
    {
        return;
    }
    
    printf("Measure your ADC while in TX state.\n");
    
    //Temp: To do ????

    printf("Once completed, press ENTER key to turn off the rfic supply voltage 2.5v.\n");

    getchar();
    ret = mixer_3v3_rfic_2v5_fpga_1v8_power_disabled();

exit:

    print_func_end_format(__func__, ret);
}

void temp_while_tx_state_test(void)
{
    int ret = 0;

    print_func_start_format(__func__);

    ret = mixer_3v3_rfic_2v5_fpga_1v8_power_enabled();
    if (ret < 0)
    {
        goto exit;
    }

    printf("\n");
    printf("Waiting the rfic swith to TX state.\n");
    if (!check_rfic_tx_state())
    {
        return;
    }
    
    printf("Measure your temperature while in TX state.\n");
    
    //Temp: To do ????

    printf("Once completed, press ENTER key to turn off the rfic supply voltage 2.5v.\n");

    getchar();
    ret = mixer_3v3_rfic_2v5_fpga_1v8_power_disabled();

exit:

    print_func_end_format(__func__, ret);
}

void i2c_test(void)
{
    int ret = 0;

    print_func_start_format(__func__);

    // I2C write and read RFFC Mixer
    // I2C to enable all GPIO

    print_func_end_format(__func__, ret);
}

void spi_test(void)
{
    int ret = 0;

    print_func_start_format(__func__);

    // SPI write and read RFIC

    print_func_end_format(__func__, ret);
}

static void showVersion(DWORD locationId)
{
    FT_STATUS            ftStatus;
    FT_HANDLE            ftHandle = (FT_HANDLE)NULL;
    FT4222_STATUS        ft4222Status;
    FT4222_Version       ft4222Version;

    ftStatus = FT_OpenEx((PVOID)(uintptr_t)locationId, 
                         FT_OPEN_BY_LOCATION, 
                         &ftHandle);
    if (ftStatus != FT_OK)
    {
        printf("FT_OpenEx failed (error %d)\n", 
               (int)ftStatus);
        return;
    }

    // Get version of library and chip.    
    ft4222Status = FT4222_GetVersion(ftHandle,
                                     &ft4222Version);
    if (FT4222_OK != ft4222Status)
    {
        printf("FT4222_GetVersion failed (error %d)\n",
               (int)ft4222Status);
    }
    else
    {
        printf("  Chip version: %08X, LibFT4222 version: %08X\n",
               (unsigned int)ft4222Version.chipVersion,
               (unsigned int)ft4222Version.dllVersion);
    }

    (void)FT_Close(ftHandle);
}

void usb_test(void)
{
    print_func_start_format(__func__);

    int ret = 0;
    FT_STATUS                 ftStatus;
    FT_DEVICE_LIST_INFO_NODE *devInfo = NULL;
    DWORD                     numDevs = 0;
    int                       i;
    int                       retCode = 0;
    int                       found4222 = 0;
    
    ftStatus = FT_CreateDeviceInfoList(&numDevs);
    if (ftStatus != FT_OK) 
    {
        printf("FT_CreateDeviceInfoList failed (error code %d)\n", 
               (int)ftStatus);
        retCode = -10;
        goto exit;
    }
    
    if (numDevs == 0)
    {
        printf("No devices connected.\n");
        retCode = -20;
        goto exit;
    }

    /* Allocate storage */
    devInfo = calloc((size_t)numDevs,
                     sizeof(FT_DEVICE_LIST_INFO_NODE));
    if (devInfo == NULL)
    {
        printf("Allocation failure.\n");
        retCode = -30;
        goto exit;
    }
    
    /* Populate the list of info nodes */
    ftStatus = FT_GetDeviceInfoList(devInfo, &numDevs);
    if (ftStatus != FT_OK)
    {
        printf("FT_GetDeviceInfoList failed (error code %d)\n",
               (int)ftStatus);
        retCode = -40;
        goto exit;
    }

    for (i = 0; i < (int)numDevs; i++) 
    {
        if (devInfo[i].Type == FT_DEVICE_4222H_0  ||
            devInfo[i].Type == FT_DEVICE_4222H_1_2)
        {
            // In mode 0, the FT4222H presents two interfaces: A and B.
            // In modes 1 and 2, it presents four interfaces: A, B, C and D.

            size_t descLen = strlen(devInfo[i].Description);
            
            if ('A' == devInfo[i].Description[descLen - 1])
            {
                // Interface A may be configured as an I2C master.
                printf("\nDevice %d: '%s'\n",
                       i,
                       devInfo[i].Description);
                showVersion(devInfo[i].LocId);
            }
            else
            {
                // Interface B, C or D.
                // No need to repeat version info of same chip.
            }
            
            found4222++;
        }
         
        if (devInfo[i].Type == FT_DEVICE_4222H_3)
        {
            // In mode 3, the FT4222H presents a single interface.  
            printf("\nDevice %d: '%s'\n",
                   i,
                   devInfo[i].Description);
            showVersion(devInfo[i].LocId);

            found4222++;
        }
    }

    if (found4222 == 0)
    {
        printf("No FT4222H detected.\n");
        ret = -1;
    }

exit:
    free(devInfo);

    print_func_end_format(__func__, ret);
}

void rfic_interrupt_test(void)
{
    int ret = 0;

    print_func_start_format(__func__);

    print_func_end_format(__func__, ret);
}

static void print_help(void)
{
    printf("\n\n");
    printf("USAGE: ./test_app_tr01 [argument]\n");
    printf("Example: ./test_app_tr01 10\n");
    printf("\n");
    printf("Argument :\n");
    printf("\n");

    for (int i=0; i < sizeof(functionList) / sizeof(functionList[0]); i++)
    {
        printf("%d\t - %s\n", (i+1), functionList[i].name);
    }
    printf("\n");
}

int main (int argc, char* argv[])
{
    if (argc > 2)
    {
        printf("Invalid Argument. Please type --help to look for command usage.\n");
        return -1;
    }

    if (!(strcmp(argv[1], "--help")))
    {
        print_help();
        return 0;
    }

    int func_num = atoi(argv[1]);
    if (func_num > 0 && func_num < 22)
    {
        functionList[func_num - 1].test();
        return 0;
    }

    printf("Invalid test name. Please type --help to look for available test case.\n");
    return 0;
}