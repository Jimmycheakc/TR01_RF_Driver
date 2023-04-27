#include <stdio.h>
#include "at86rf215.h"
#include "at86rf215_radio.h"
#include "../io_utils/io_utils.h"
#include "../i2c/i2c.h"


#define IMX_SPI_DEV 1
#define IMX_MODEM_SPI_CHANNEL 1
#define IMX_MODEM_SS 13
#define IMX_MODEM_RESET 23
#define IMX_MODEM_IRQ 8


#define I2C_DEVICE "/dev/i2c-3"

uint16_t unknown_regs[] = { 0x0015, 
                            0x0115, 0x0117, 0x0118, 0x0119, 0x011A, 0x011B, 0x011C, 0x011D, 0x011E, 0x011F, 0x0120, 0x0123, 0x0124, 0x0129,
                            0x0215, 0x0217, 0x0218, 0x0219, 0x021A, 0x021B, 0x021C, 0x021D, 0x021E, 0x021F, 0x0220, 0x0223, 0x0224, 0x0229};

uint8_t defaults[] =    {   0x0E,
                            0x40, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x18, 0x0f, 0x0f, 0x08,
                            0x40, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x18, 0x0f, 0x0f, 0x08};

// -----------------------------------------------------------------------------------------
// This test checks the version number and part numbers of the device
int test_at86rf215_read_all_regs(at86rf215_st* dev)
{
    for (uint16_t reg = 0; reg < 0x300; reg ++)
    {
        uint8_t val = at86rf215_read_byte(dev, reg);
        printf("REG #0x%04X  :  0x%02X\n", reg, val);
    }
    return 0;
}

int test_at86rf215_read_all_regs_check(at86rf215_st* dev)
{
    int num_regs = sizeof(unknown_regs) / sizeof(uint16_t);

    for (int i = 0; i < num_regs; i ++)
    {
        uint8_t val = at86rf215_read_byte(dev, unknown_regs[i]);
        if (val != defaults[i]) printf("REG #0x%04X  :  0x%02X    (instead of 0x%02X)\n", unknown_regs[i], val, defaults[i]);
    }
    return 0;
}

// -----------------------------------------------------------------------------------------
// This test checks the version number and part numbers of the device
int test_at86rf215_read_chip_vn_pn(at86rf215_st* dev)
{
    int pass = 0;
    uint8_t pn, vn;
    char pn_st[15] = {0};
	at86rf215_get_versions(dev, &pn, &vn);

    if (pn==0x34 && vn > 0 && vn < 5) pass = 1;
    if (pn==0x34)
        sprintf(pn_st, "AT86RF215");
    else if (pn==0x35)
        sprintf(pn_st, "AT86RF215IQ");
    else if (pn==0x36)
        sprintf(pn_st, "AT86RF215M");
    else
        sprintf(pn_st, "UNKNOWN");

    printf("TEST:AT86RF215:VERSIONS:PN=0x%02X\n", pn);
    printf("TEST:AT86RF215:VERSIONS:VN=%d\n", vn);
    printf("TEST:AT86RF215:VERSIONS:PASS=%d\n", pass);
    printf("TEST:AT86RF215:VERSIONS:INFO=The component PN is %s (0x%02X), Version %d\n", pn_st, pn, vn);

    at86rf215_set_clock_output(dev, at86rf215_drive_current_8ma, at86rf215_clock_out_freq_32mhz);

    return pass;
}

// -----------------------------------------------------------------------------------------
// This test performs a frequency sweep on a certain channel:
//      at86rf215_rf_channel_900mhz / at86rf215_rf_channel_2400mhz
// and does it from "start_freq" to "start_freq + num_freq * step_freq"
// usec_gaps - specifies the micro-second gaps between freq steps or '-1' that
//             tell the function to put "getchars" (wait for enter key)
void test_at86rf215_sweep_frequencies(at86rf215_st* dev,
                                        at86rf215_rf_channel_en channel,
                                        int start_freq,
                                        int num_freq,
                                        int step_freq,
                                        int usec_gaps)
{
    char channel_name[10];

    if (channel == at86rf215_rf_channel_900mhz)
        sprintf(channel_name, "900MHz");
    else sprintf(channel_name, "2400MHz");

    io_utils_usleep(usec_gaps);
    int stop_freq = start_freq + step_freq*num_freq;
    int f = start_freq;
    while (f <= stop_freq)
    {
        printf("TEST:AT86RF215:FREQ_SWEEP:FREQ=%.2fMHz, CH=%s)\n", ((float)f) / 1e6, channel_name);
        at86rf215_setup_iq_radio_dac_value_override (dev,
                                                    channel,
                                                    f,
                                                    31);


        //printf("Press enter to switch\n");
        if (usec_gaps > 0) io_utils_usleep(usec_gaps);
        else
        {
            printf("Press enter to step...\n");
            getchar();
        }

        //test_at86rf215_read_all_regs_check(dev);
        f += step_freq;
    }

    // back to TX off state
    at86rf215_radio_set_state(dev, at86rf215_rf_channel_900mhz, at86rf215_radio_state_cmd_trx_off);
}

// -----------------------------------------------------------------------------------------
// Starting a reception window
// usec_timeout - set up a timeout value in micro-seconds or -1 to wait for "enter" key
int test_at86rf215_continues_iq_rx (at86rf215_st* dev, at86rf215_rf_channel_en radio,
                                        uint64_t freq_hz, int usec_timeout)
{
    at86rf215_iq_clock_data_skew_en skew = radio == at86rf215_rf_channel_900mhz?
                                            at86rf215_iq_clock_data_skew_4_906ns:
                                            at86rf215_iq_clock_data_skew_4_906ns;

    at86rf215_setup_iq_radio_receive (dev, radio, freq_hz, 0, skew);
    printf("Started I/Q RX session for Radio %d, Freq: %lu Hz, timeout: %d usec (0=infinity)\n",
        radio, freq_hz, usec_timeout);


    test_at86rf215_read_all_regs_check(dev);

    if (usec_timeout>0)
    {
        io_utils_usleep(usec_timeout);
    }
    else
    {
        printf("Press enter to stop...\n");
        getchar();
    }
    at86rf215_stop_iq_radio_receive (dev, radio);

    test_at86rf215_read_all_regs_check(dev);
    return 1;
}

// -----------------------------------------------------------------------------------------
// SMI communication tesing with loopback
// usec_timeout - set up a timeout value in micro-seconds or -1 to wait for "enter" key
int test_at86rf215_continues_iq_loopback (at86rf215_st* dev, at86rf215_rf_channel_en radio,
                                        uint32_t freq_hz, int usec_timeout)
{
    at86rf215_setup_iq_radio_receive (dev, radio, freq_hz, 0, at86rf215_iq_clock_data_skew_4_906ns);
    //at86rf215_radio_set_tx_dac_input_iq(dev, radio, 1, 0x7E, 1, 0x3F);

    printf("Started I/Q RX session for Radio %d, Freq: %d Hz, timeout: %d usec (0=infinity)\n",
        radio, freq_hz, usec_timeout);

    if (usec_timeout>0)
    {
        io_utils_usleep(usec_timeout);
    }
    else
    {
        printf("Press enter to stop...\n");
        getchar();
    }
    at86rf215_stop_iq_radio_receive (dev, radio);
    return 1;
}

// -----------------------------------------------------------------------------------------
// TEST SELECTION
// -----------------------------------------------------------------------------------------
#define TEST_VERSIONS       1
#define TEST_FREQ_SWEEP     0
#define TEST_IQ_RX_WIND     0
#define TEST_IQ_RX_WIND_RAD 0 
#define TEST_IQ_LB_WIND     0
#define TEST_READ_ALL_REGS  0

// -----------------------------------------------------------------------------------------
// MAIN
// -----------------------------------------------------------------------------------------
int main ()
{
    at86rf215_st dev = {0};
    at86rf215_iq_interface_config_st cfg = {0};
    at86rf215_irq_st irq = {0};

    // Init GPIOs on reset
	//io_utils_setup("gpiochip0");

	at86rf215_init(&dev, I2C_DEVICE, I2C_TO_SPI_SLAVE_ADDRESS, at86rf215_1_ss, "gpiochip5", IMX_MODEM_IRQ, "gpio_interrupt");

    // TEST: read the p/n and v/n from the IC
    #if TEST_VERSIONS
        test_at86rf215_read_chip_vn_pn(&dev);
    #endif // TEST_VERSIONS

    // TEST: Frequency sweep
    #if TEST_FREQ_SWEEP
        test_at86rf215_sweep_frequencies(&dev, at86rf215_rf_channel_900mhz, 900e6, 3, 100e6, 10000);
    #endif // TEST_FREQ_SWEEP

    #if TEST_IQ_RX_WIND
        #if TEST_IQ_RX_WIND_RAD==0
            test_at86rf215_continues_iq_rx (&dev, at86rf215_rf_channel_900mhz, 900e6, -1);
        #else
            test_at86rf215_continues_iq_rx (&dev, at86rf215_rf_channel_2400mhz, 2400e6, -1);
        #endif
    #endif // TEST_IQ_RX_WIND

    #if TEST_IQ_LB_WIND
        test_at86rf215_continues_iq_loopback (&dev, at86rf215_rf_channel_900mhz, 900e6, -1);
    #endif

    #if TEST_READ_ALL_REGS
        test_at86rf215_read_all_regs_check(&dev);
    #endif


    printf("Waiting Interrupt or Press enter to stop.\n");
    getchar();

	at86rf215_close(&dev);

    return 0;
}
