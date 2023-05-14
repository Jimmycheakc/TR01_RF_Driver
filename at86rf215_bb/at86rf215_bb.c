/*
 *  at86rf215-driver: OS-independent driver for the AT86RF215 transceiver
 *
 *  Copyright (C) 2020, Libre Space Foundation <http://libre.space>
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "at86rf215_bb.h"
#include "at86rf215_bb_regs.h"
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "../io_utils/io_utils.h"
#include "../spi/spi.h"

#define INIT_MAGIC_VAL 0x92c2f0e3

#ifndef max
#define max(a,b)             \
({                           \
    __typeof__ (a) _a = (a); \
    __typeof__ (b) _b = (b); \
    _a > _b ? _a : _b;       \
})
#endif

#ifndef min
#define min(a,b)             \
({                           \
    __typeof__ (a) _a = (a); \
    __typeof__ (b) _b = (b); \
    _a < _b ? _a : _b;       \
})
#endif

#define BIT(n) (1UL << (n))

#define SPI_DEVICE "/dev/spidev1.0"
static spi_t spidev;

#define NUM_CAL_STEPS 7

void swap(int *p,int *q) 
{
   int t;  
   t=*p; 
   *p=*q; 
   *q=t;
}

void sort(int a[],int n) 
{ 
   int i,j,temp;

   for(i = 0;i < n-1;i++) {
      for(j = 0;j < n-i-1;j++) {
         if(a[j] > a[j+1])
            swap(&a[j],&a[j+1]);
      }
   }
}

int median(int a[], int n)
{
    if (n==0) return 0;
    sort(a,n);
    return a[(n+1)/2-1];
}

/**
 * Checks if the device structure has been successfully initialized through the
 * at86rf215_init().
 * @param h the device handle
 * @return 0 on success or negative error code
 */
 // TODO: Make it back to static
int ready(const struct at86rf215 *h)
{
	if (!h) {
		return -AT86RF215_INVAL_PARAM;
	}
	if (h->priv.init != INIT_MAGIC_VAL) {
		return -AT86RF215_NO_INIT;
	}
	return AT86RF215_OK;
}

/**
 * Checks if the RF interface is supported based on the device family
 * @param h the device handle
 * @param radio the RF frontend
 * @return 0 on success or negative error code
 */
static int supports_rf(const struct at86rf215 *h,  at86rf215_radio_t radio)
{
	int ret = ready(h);
	if (ret) {
		return ret;
	}
	switch (radio) {
		case AT86RF215_RF09:
		case AT86RF215_RF24:
			break;
		default:
			return -AT86RF215_INVAL_PARAM;
	}
	if (radio == AT86RF215_RF24 && h->priv.family == AT86RF215M) {
		return -AT86RF215_NOT_SUPPORTED;
	}
	return AT86RF215_OK;
}

/**
 * Checks if the baseband mode is supported based on the device family
 * @param h the device handle
 * @param radio the RF frontend
 * @return 0 on success or negative error code
 */
static int supports_mode(const struct at86rf215 *h,  at86rf215_chpm_t mode)
{
	int ret = ready(h);
	if (ret) {
		return ret;
	}
	switch (mode) {
		case AT86RF215_RF_MODE_BBRF:
		case AT86RF215_RF_MODE_RF:
		case AT86RF215_RF_MODE_BBRF09:
		case AT86RF215_RF_MODE_BBRF24:
			break;
		default:
			return -AT86RF215_INVAL_PARAM;
	}
	if (h->priv.family == AT86RF215IQ && mode != AT86RF215_RF_MODE_RF) {
		return -AT86RF215_NOT_SUPPORTED;
	}
	if (h->priv.family == AT86RF215M && mode == AT86RF215_RF_MODE_BBRF24) {
		return -AT86RF215_NOT_SUPPORTED;
	}
	return AT86RF215_OK;
}

/**
 * Resets and initializes the AT86RF215 IC
 * @param h the device handle
 * @param drv clock output driving current
 * @param os clock output selection
 * @return 0 on success or negative error code
 */
int at86rf215_init(struct at86rf215 *h)
{
	if (!h) {
		return -AT86RF215_INVAL_PARAM;
	}

	// TODO: Move out the SPI device init, can pass int spidev and store into at86rf215 structure
	spi_init(&spidev, SPI_DEVICE, 0, 0, 2500000);

	/* Reset the state of the private struct members */
	memset(&h->priv, 0, sizeof(struct at86rf215_priv));

	uint8_t val = 0;
	int ret = at86rf215_reg_read_8(&val, REG_RF_PN);
	if (ret) {
		return ret;
	}
	switch (val) {
		case AT86RF215:
		case AT86RF215IQ:
		case AT86RF215M:
			h->priv.family = (at86rf215_family_t) val;
			break;
		default:
			return -AT86RF215_UNKNOWN_IC;
	}

	val = 0;
	switch (h->clk_drv) {
		case AT86RF215_RF_DRVCLKO2:
		case AT86RF215_RF_DRVCLKO4:
		case AT86RF215_RF_DRVCLKO6:
		case AT86RF215_RF_DRVCLKO8:
			val = h->clk_drv << 3;
			break;
		default:
			return -AT86RF215_INVAL_PARAM;
	}
	switch (h->clko_os) {
		case AT86RF215_RF_CLKO_OFF:
		case AT86RF215_RF_CLKO_26_MHZ:
		case AT86RF215_RF_CLKO_32_MHZ:
		case AT86RF215_RF_CLKO_16_MHZ:
		case AT86RF215_RF_CLKO_8_MHZ:
		case AT86RF215_RF_CLKO_4_MHZ:
		case AT86RF215_RF_CLKO_2_MHZ:
		case AT86RF215_RF_CLKO_1_MHZ:
			val |= h->clko_os;
			break;
	}
	ret = at86rf215_reg_write_8(val, REG_RF_CLKO);
	if (ret) {
		return ret;
	}

	/* Apply XO settings */
	ret = at86rf215_reg_write_8((h->xo_fs << 4) | h->xo_trim, REG_RF_XOC);
	if (ret) {
		return ret;
	}

	/* Set the RF_CFG */
	ret = at86rf215_reg_write_8((h->irqmm << 3) | (h->irqp << 2) | h->pad_drv, REG_RF_CFG);
	if (ret) {
		return ret;
	}

	/* Set RF09_PADFE and RF24_PADFE */
	val = 0;
	switch (h->rf_femode_09) {
		case AT86RF215_RF_FEMODE0:
		case AT86RF215_RF_FEMODE1:
		case AT86RF215_RF_FEMODE2:
		case AT86RF215_RF_FEMODE3:
			val = h->rf_femode_09 << 6;
			break;
		default:
			return -AT86RF215_INVAL_PARAM;
	}
	ret = at86rf215_reg_write_8(val, REG_RF09_PADFE);
	if (ret) {
		return ret;
	}

	val = 0;
	switch (h->rf_femode_24) {
		case AT86RF215_RF_FEMODE0:
		case AT86RF215_RF_FEMODE1:
		case AT86RF215_RF_FEMODE2:
		case AT86RF215_RF_FEMODE3:
			val = h->rf_femode_24 << 6;
			break;
		default:
			return -AT86RF215_INVAL_PARAM;
	}
	ret = at86rf215_reg_write_8(val, REG_RF24_PADFE);
	if (ret) {
		return ret;
	}

	/* Get the version of the IC */
	ret = at86rf215_reg_read_8(&val, REG_RF_VN);
	if (ret) {
		return ret;
	}
	h->priv.version = val;
	h->priv.chpm = AT86RF215_RF_MODE_BBRF;
	h->priv.init = INIT_MAGIC_VAL;

	/* Disable all IRQ sources */
	at86rf215_set_bbc_irq_mask(h, AT86RF215_RF09, 0x0);
	at86rf215_set_bbc_irq_mask(h, AT86RF215_RF24, 0x0);
	at86rf215_set_radio_irq_mask(h, AT86RF215_RF09, 0x0);
	at86rf215_set_radio_irq_mask(h, AT86RF215_RF24, 0x0);


	event_node_init(&h->priv.radios[AT86RF215_RF09].trxready);
	event_node_init(&h->priv.radios[AT86RF215_RF24].trxready);

	at86rf215_calibrate_device(h, AT86RF215_RF09, &h->cal.low_ch_i, &h->cal.low_ch_q);
    at86rf215_calibrate_device(h, AT86RF215_RF24, &h->cal.hi_ch_i, &h->cal.hi_ch_q);

	return AT86RF215_OK;
}

/**
 * Checks if the at86rf215_radio_init() has been succesfully called for the
 * corresponding RF frontend
 * @param h the device handle
 * @param radio the radio frontend
 * @return 0 on success or negative error code
 */
static int radio_ready(const struct at86rf215 *h,  at86rf215_radio_t radio)
{
	int ret = supports_rf(h, radio);
	if (ret) {
		return ret;
	}
	if (h->priv.radios[radio].init != INIT_MAGIC_VAL) {
		return -AT86RF215_NO_INIT;
	}
	return AT86RF215_OK;
}

/**
 * (Re-)Initializes a RF frontend
 * @param h the device handle
 * @param radio the RF frontend
 * @param conf configuration parameters
 * @return 0 on success or negative error code
 */
int at86rf215_radio_conf(struct at86rf215 *h, at86rf215_radio_t radio,
                    	const struct at86rf215_radio_conf *conf)
{
	if (!conf) {
		return -AT86RF215_INVAL_PARAM;
	}
	int ret = supports_rf(h, radio);
	if (ret) {
		return ret;
	}
	uint32_t spacing = conf->cs / 25000;
	spacing = min(0xFF, spacing);

	/* Set the channel configuration */
	if (radio == AT86RF215_RF09) {
		switch (conf->cm) {
			case AT86RF215_CM_IEEE: {
				ret = at86rf215_reg_write_8(spacing, REG_RF09_CS);
				if (ret) {
					return ret;
				}
				h->priv.radios[AT86RF215_RF09].cs_reg = spacing;
				h->priv.radios[AT86RF215_RF09].base_freq = conf->base_freq;
				ret = at86rf215_reg_write_8(spacing, REG_RF09_CS);
				if (ret) {
					return ret;
				}

				/* Apply base frequency */
				if (conf->base_freq < 389500000 || conf->base_freq > 1088000000) {
					return -AT86RF215_INVAL_PARAM;
				}
				uint16_t base = conf->base_freq / 25000;
				ret = at86rf215_reg_write_8(base & 0xFF, REG_RF09_CCF0L);
				if (ret) {
					return ret;
				}
				ret = at86rf215_reg_write_8(base >> 8, REG_RF09_CCF0H);
				if (ret) {
					return ret;
				}
			}
			case AT86RF215_CM_FINE_RES_04:
			case AT86RF215_CM_FINE_RES_09:
				break;
			default:
				return -AT86RF215_INVAL_PARAM;
		}
	} else {
		switch (conf->cm) {
			case AT86RF215_CM_IEEE: {
				ret = at86rf215_reg_write_8(spacing, REG_RF24_CS);
				if (ret) {
					return ret;
				}
				h->priv.radios[AT86RF215_RF24].cs_reg = spacing;
				h->priv.radios[AT86RF215_RF24].base_freq = conf->base_freq;

				/* Apply base frequency */
				if (conf->base_freq < 2400000000 || conf->base_freq > 2483500000) {
					return -AT86RF215_INVAL_PARAM;
				}
				/* At 2.4 GHz band the base frequency has a
				 * 1.5 GHz offset
				 */
				uint16_t base = (conf->base_freq - 1500000000) / 25000;
				ret = at86rf215_reg_write_8(base & 0xFF, REG_RF24_CCF0L);
				if (ret) {
					return ret;
				}
				ret = at86rf215_reg_write_8(base >> 8, REG_RF24_CCF0H);
				if (ret) {
					return ret;
				}
			}
			case AT86RF215_CM_FINE_RES_24:
				break;
			default:
				return -AT86RF215_INVAL_PARAM;
		}
	}

	h->priv.radios[radio].cm = conf->cm;
	h->priv.radios[radio].cs = conf->cs;

	/* PLL loop bandwidth is applicable for the sub-1GHz radio only*/
	if (radio == AT86RF215_RF09) {
		switch (conf->lbw) {
			case AT86RF215_PLL_LBW_DEFAULT:
			case AT86RF215_PLL_LBW_SMALLER:
			case AT86RF215_PLL_LBW_LARGER:
				break;
			default:
				return -AT86RF215_INVAL_PARAM;
		}
		ret = at86rf215_reg_write_8(conf->lbw, REG_RF09_PLL);
		if (ret) {
			return ret;
		}
	}
	h->priv.radios[radio].op_state = AT86RF215_OP_STATE_IDLE;
	h->priv.radios[radio].init = INIT_MAGIC_VAL;
	return AT86RF215_OK;
}

/**
 * Reads from the SPI peripheral
 * @param out the output buffer to hold MISO response from the SPI peripheral
 * @param in input buffer containing MOSI data
 * @param len the length of the SPI transaction (max(MOSI, MISO))
 * @return 0 on success or negative error code
 */
int at86rf215_spi_read(uint8_t *out, const uint8_t *in, size_t len)
{
	int ret;

	ret = spi_exchange(&spidev, out, in, len);
	if (ret >= 0)
	{
		ret = AT86RF215_OK;
	}
	else
	{
		ret = -AT86RF215_SPI_READ_FAILED;
	}
	return ret;
}

/**
 * Writes to the device using the SPI peripheral
 * @param in the input buffer
 * @param len the size of the input buffer
 * @return 0 on success or negative error code
 */
int at86rf215_spi_write(const uint8_t *in, size_t len)
{
	int ret;

	ret = spi_write(&spidev, in, len);
	if (ret >= 0)
	{
		ret = AT86RF215_OK;
	}
	else
	{
		ret = -AT86RF215_SPI_WRITE_FAILED;
	}
	return ret;
}

/**
 * Reads an 8-bit register
 * @note internally the function uses the at86rf215_spi_read() 
 * to accomplish the SPI transaction. Developers should
 * provide a proper implementation of those functions.
 *
 * @param out pointer to hold the read value
 * @param reg the register to read
 * @return 0 on success or negative error code
 */
int at86rf215_reg_read_8(uint8_t *out, uint16_t reg)
{
	if (!out) {
		return -AT86RF215_INVAL_PARAM;
	}
	int ret = 0;

	/* Construct properly the MOSI buffer */
	uint8_t mosi[3] = {(reg >> 8) & 0x3F, reg & 0xFF, 0x0};
	uint8_t miso[3] = {0x0, 0x0, 0x0};
	ret = at86rf215_spi_read(miso, mosi, 3);
	if (ret) {
		return ret;
	}
	*out = miso[2];
	return AT86RF215_OK;
}

/**
 * Reads an 32-bit register
 * @note internally the function uses the at86rf215_spi_read() 
 * to accomplish the SPI transaction. Developers should
 * provide a proper implementation of those functions.
 * @note the result is stored in a MS byte first order
 *
 * @param out pointer to hold the read value
 * @param reg the register to read
 * @return 0 on success or negative error code
 */
int at86rf215_reg_read_32(uint32_t *out, uint16_t reg)
{
	if (!out) {
		return -AT86RF215_INVAL_PARAM;
	}
	int ret = 0;

	/* Construct properly the MOSI buffer */
	uint8_t mosi[6] = {(reg >> 8) & 0x3F, reg & 0xFF, 0x0, 0x0, 0x0, 0x0};
	uint8_t miso[6] = {0x0, 0x0, 0x0, 0x0, 0x0, 0x0};
	ret = at86rf215_spi_read(miso, mosi, 6);
	if (ret) {
		return ret;
	}
	*out = (miso[2] << 24) | (miso[3] << 16) | (miso[4] << 8) | miso[5];
	return AT86RF215_OK;
}

/**
 * Writes an 8-bit register
 *
 * @note internally the function uses the at86rf215_spi_write() 
 * to accomplish the SPI transaction. Developers should
 * provide a proper implementation of those functions.
 *
 * @param in the value to write
 * @param reg the register to write
 * @return 0 on success or negative error code
 */
int at86rf215_reg_write_8(const uint8_t in, uint16_t reg)
{
	int ret = 0;

	/* Construct properly the MOSI buffer */
	uint8_t mosi[3] = {(reg >> 8) | 0x80, reg & 0xFF, in};
	ret = at86rf215_spi_write(mosi, 3);
	if (ret) {
		return ret;
	}
	return AT86RF215_OK;
}

/**
 * Writes an 16-bit register
 *
 * @note internally the function uses the at86rf215_spi_write() 
 * to accomplish the SPI transaction. Developers should
 * provide a proper implementation of those functions.
 * @note the value is written in a MS byte first order
 *
 * @param in the value to write
 * @param reg the register to write
 * @return 0 on success or negative error code
 */
int at86rf215_reg_write_16(const uint16_t in, uint16_t reg)
{
	int ret = 0;

	/* Construct properly the MOSI buffer */
	uint8_t mosi[4] = {(reg >> 8) | 0x80, reg & 0xFF, in >> 8, in & 0xFF};
	ret = at86rf215_spi_write(mosi, 4);
	if (ret) {
		return ret;
	}
	return AT86RF215_OK;
}

/**
 * Retrieve the RF state of the transceiver
 * @param h the device handle
 * @param state pointer to store the result
 * @param radio the radio front-end to query its state
 * @return 0 on success or negative error code
 */
int at86rf215_get_state(const struct at86rf215 *h, at86rf215_rf_state_t *state, at86rf215_radio_t radio)
{
	int ret = supports_rf(h, radio);
	if (ret) {
		return ret;
	}
	if (!state) {
		return -AT86RF215_INVAL_PARAM;
	}
	uint8_t val = 0;
	uint16_t reg = 0;
	switch (radio) {
		case AT86RF215_RF24:
			reg = REG_RF24_STATE;
			break;
		case AT86RF215_RF09:
			reg = REG_RF09_STATE;
			break;
		default:
			return -AT86RF215_INVAL_PARAM;
	}
	ret = at86rf215_reg_read_8(&val, reg);
	if (ret) {
		return ret;
	}
	/* Sanity check on the returned value */
	val &= 0x7;
	switch (val) {
		case AT86RF215_STATE_RF_TRXOFF:
		case AT86RF215_STATE_RF_TXPREP:
		case AT86RF215_STATE_RF_TX:
		case AT86RF215_STATE_RF_RX:
		case AT86RF215_STATE_RF_TRANSITION:
		case AT86RF215_STATE_RF_RESET:
			*state = (at86rf215_rf_state_t) val;
			break;
		default:
			return -AT86RF215_INVAL_VAL;
	}
	return AT86RF215_OK;
}


/**
 * Issue a command to a specified RF frontend of the IC
 * @param h the device handle
 * @param cmd the command
 * @param radio the radio front-end
 * @return 0 on success or negative error code
 */
int at86rf215_set_cmd(struct at86rf215 *h, at86rf215_rf_cmd_t cmd, at86rf215_radio_t radio)
{
	int ret = supports_rf(h, radio);
	if (ret) {
		return ret;
	}
	switch (cmd) {
		case AT86RF215_CMD_RF_NOP:
		case AT86RF215_CMD_RF_SLEEP:
		case AT86RF215_CMD_RF_TRXOFF:
		case AT86RF215_CMD_RF_TXPREP:
		case AT86RF215_CMD_RF_TX:
		case AT86RF215_CMD_RF_RX:
		case AT86RF215_CMD_RF_RESET:
			break;
		default:
			return -AT86RF215_INVAL_PARAM;
	}
	uint16_t reg = 0;
	switch (radio) {
		case AT86RF215_RF09:
			reg = REG_RF09_CMD;
			break;
		case AT86RF215_RF24:
			reg = REG_RF24_CMD;
			break;
		default:
			return -AT86RF215_INVAL_PARAM;
	}

	ret = at86rf215_reg_write_8(cmd, reg);

	/*Errata #6:    State Machine Command RFn_CMD=TRXOFF may not be succeeded
                    Description: If the current state is different from SLEEP, the execution of the command TRXOFF may fail.
                    Software workaround: Check state by reading register RFn_STATE Repeat the command RFn_CMD=TRXOFF
                    if the target state was not reached.
    */
    if (cmd == AT86RF215_CMD_RF_TRXOFF)
    {
        int retries = 5;
		at86rf215_rf_state_t radio_state;
		at86rf215_get_state(h, &radio_state, radio);
        while (radio_state != cmd && retries--)
        {
            io_utils_usleep(100);
            at86rf215_reg_write_8(cmd, reg);
        }
    }
    if (cmd == AT86RF215_CMD_RF_TXPREP)
    {
        //if (ch == at86rf215_rf_channel_900mhz) event_node_wait_ready(&dev->events.lo_trx_ready_event);
        //else if (ch == at86rf215_rf_channel_2400mhz) event_node_wait_ready(&dev->events.hi_trx_ready_event);

        io_utils_usleep(1000);
        if (h->override_cal)
        {
            int i = radio == AT86RF215_RF09 ? h->cal.low_ch_i : h->cal.hi_ch_i;
            int q = radio == AT86RF215_RF09 ? h->cal.low_ch_q : h->cal.hi_ch_q;
            at86rf215_radio_set_tx_iq_calibration(h, radio, i, q);
        }
    }
	return ret;
}

/**
 * Sets the chip mode,
 * @note this function ensures that the SKEWDRV part of the register
 * remains unaffected.
 * @param h the device handle
 * @param mode the chip mode
 * @return 0 on success or negative error code
 */
int at86rf215_set_mode(struct at86rf215 *h, at86rf215_chpm_t mode)
{
	int ret = supports_mode(h, mode);
	if (ret) {
		return ret;
	}

	uint8_t val = 0;
	ret = at86rf215_reg_read_8(&val, REG_RF_IQIFC1);
	if (ret) {
		return ret;
	}
	val = (val & 0x3) | (mode << 4);
	ret = at86rf215_reg_write_8(val, REG_RF_IQIFC1);
	if (ret) {
		return ret;
	}
	h->priv.chpm = mode;
	return AT86RF215_OK;
}

/**
 * Resets a specific transceiver
 * @param h the device handle
 * @param radio the transceiver to reset
 * @return 0 on success or negative error code
 */
int at86rf215_transceiver_reset(struct at86rf215 *h, at86rf215_radio_t radio)
{
	return at86rf215_set_cmd(h, AT86RF215_CMD_RF_RESET, radio);
}

int at86rf215_set_bbc_irq_mask(struct at86rf215 *h, at86rf215_radio_t radio, uint8_t mask)
{
	int ret = supports_rf(h, radio);
	if (ret) {
		return ret;
	}
	uint16_t reg = 0x0;
	switch (radio) {
		case AT86RF215_RF09:
			reg = REG_BBC0_IRQM;
			break;
		case AT86RF215_RF24:
			reg = REG_BBC1_IRQM;
			break;
		default:
			return -AT86RF215_NOT_SUPPORTED;
	}
	return at86rf215_reg_write_8(mask, reg);
}

/**
 * Set the mask for IRQs originating from the RF frontends. 1 activates the
 * corresponding IRQ, 0 deactivates it.
 * @param h the device handle
 * @param radio the RF frontend
 * @param mask the activation mask
 * @return 0 on success or negative error code
 */
int at86rf215_set_radio_irq_mask(struct at86rf215 *h, at86rf215_radio_t radio, uint8_t mask)
{
	int ret = supports_rf(h, radio);
	if (ret) {
		return ret;
	}
	uint16_t reg = 0x0;
	if (radio == AT86RF215_RF09) {
		reg = REG_RF09_IRQM;
	} else {
		reg = REG_RF24_IRQM;
	}
	return at86rf215_reg_write_8(mask, reg);
}

static bool is_lpf_valid(at86rf215_lpfcut_t lpf)
{
	switch (lpf) {
		case AT86RF215_RF_FLC80KHZ:
		case AT86RF215_RF_FLC100KHZ:
		case AT86RF215_RF_FLC125KHZ:
		case AT86RF215_RF_FLC160KHZ:
		case AT86RF215_RF_FLC200KHZ:
		case AT86RF215_RF_FLC250KHZ:
		case AT86RF215_RF_FLC315KHZ:
		case AT86RF215_RF_FLC400KHZ:
		case AT86RF215_RF_FLC500KHZ:
		case AT86RF215_RF_FLC624KHZ:
		case AT86RF215_RF_FLC800KHZ:
		case AT86RF215_RF_FLC1000KHZ:
			return 1;
	}
	return 0;
}

static bool is_paramp_valid(at86rf215_paramp_t paramp)
{
	switch (paramp) {
		case AT86RF215_RF_PARAMP4U:
		case AT86RF215_RF_PARAMP8U:
		case AT86RF215_RF_PARAMP16U:
		case AT86RF215_RF_PARAMP32U:
			return 1;
	}
	return 0;
}

/**
 * Sets the PA ramping timing and the LPF configuration for the TX.
 * @param h the device handle
 * @param radio the RF frontend
 * @param paramp PA ramping setting
 * @param lpf LPF setting
 * @return 0 on success or negative error code
 */
int at86rf215_set_txcutc(struct at86rf215 *h, at86rf215_radio_t radio, at86rf215_paramp_t paramp, at86rf215_lpfcut_t lpf)
{
	int ret = supports_rf(h, radio);
	if (ret) {
		return ret;
	}
	if (!is_lpf_valid(lpf) || !is_paramp_valid(paramp)) {
		return -AT86RF215_INVAL_PARAM;
	}
	uint16_t reg = 0x0;
	if (radio == AT86RF215_RF09) {
		reg = REG_RF09_TXCUTC;
	} else {
		reg = REG_RF24_TXCUTC;
	}
	return at86rf215_reg_write_8(paramp << 6 | lpf, reg);
}

/**
 * Configures the TX digital frontend parameters using the TXDFE register.
 * @param radio the RF frontend
 * @param rcut TX filter relative to the cut-off frequency
 * @param dm set 1 to enable the direct modulation, 0 otherwise
 * @param sr the sampling rate setting
 * @return 0 on success or negative error code
 */
static int set_txdfe(at86rf215_radio_t radio, uint8_t rcut, uint8_t dm, at86rf215_sr_t sr)
{
	uint16_t reg = 0x0;
	if (radio == AT86RF215_RF09) {
		reg = REG_RF09_TXDFE;
	} else {
		reg = REG_RF24_TXDFE;
	}
	switch (sr) {
		case AT86RF215_SR_4000KHZ:
		case AT86RF215_SR_2000KHZ:
		case AT86RF215_SR_1333KHZ:
		case AT86RF215_SR_1000KHZ:
		case AT86RF215_SR_800KHZ:
		case AT86RF215_SR_666KHZ:
		case AT86RF215_SR_500KHZ:
		case AT86RF215_SR_400KHZ:
			break;
		default:
			return -AT86RF215_INVAL_PARAM;
	}
	if (rcut > 0x4) {
		return -AT86RF215_INVAL_PARAM;
	}
	const uint8_t val = (rcut << 5) | ((dm & 0x1) << 4) | sr;
	return at86rf215_reg_write_8(val, reg);
}

/**
 * Configures the TX digital frontend parameters using the TXDFE register.
 * @param h the device handle
 * @param radio the RF frontend
 * @param rcut TX filter relative to the cut-off frequency
 * @param dm set 1 to enable the direct modulation, 0 otherwise
 * @param sr the sampling rate setting
 * @return 0 on success or negative error code
 */
int at86rf215_set_txdfe(struct at86rf215 *h, at86rf215_radio_t radio, uint8_t rcut, uint8_t dm, at86rf215_sr_t sr)
{
	int ret = supports_rf(h, radio);
	if (ret) {
		return ret;
	}

	return set_txdfe(radio, rcut, dm, sr);
}


/**
 * Configures the RX digital frontend parameters using the RXDFE register.
 * @param radio the RF frontend
 * @param rcut RX filter relative to the cut-off frequency
 * @param sr the sampling rate setting
 * @return 0 on success or negative error code
 */
static int set_rxdfe(at86rf215_radio_t radio, uint8_t rcut, at86rf215_sr_t sr)
{
	uint16_t reg = 0x0;
	if (radio == AT86RF215_RF09) {
		reg = REG_RF09_RXDFE;
	} else {
		reg = REG_RF24_RXDFE;
	}
	switch (sr) {
		case AT86RF215_SR_4000KHZ:
		case AT86RF215_SR_2000KHZ:
		case AT86RF215_SR_1333KHZ:
		case AT86RF215_SR_1000KHZ:
		case AT86RF215_SR_800KHZ:
		case AT86RF215_SR_666KHZ:
		case AT86RF215_SR_500KHZ:
		case AT86RF215_SR_400KHZ:
			break;
		default:
			return -AT86RF215_INVAL_PARAM;
	}
	if (rcut > 0x4) {
		return -AT86RF215_INVAL_PARAM;
	}
	const uint8_t val = (rcut << 5) | sr;
	return at86rf215_reg_write_8(val, reg);
}

/**
 * Configures the RX digital frontend parameters using the RXDFE register.
 * @param h the device handle
 * @param radio the RF frontend
 * @param rcut RX filter relative to the cut-off frequency
 * @param sr the sampling rate setting
 * @return 0 on success or negative error code
 */
int at86rf215_set_rxdfe(struct at86rf215 *h, at86rf215_radio_t radio, uint8_t rcut, at86rf215_sr_t sr)
{
	int ret = supports_rf(h, radio);
	if (ret) {
		return ret;
	}

	return set_rxdfe(radio, rcut, sr);
}

/**
 * Configures the RX digital frontend cuttoff frequency using the RXDFE register.
 * The rest of the contents of the RXDFE register remain unaffected and are
 * configured via the at86rf215_bb_conf() function.
 * @param h the device handle
 * @param radio the RF frontend
 * @param rcut RX filter relative to the cut-off frequency
 * @return 0 on success or negative error code
 */
int at86rf215_set_rx_rcut(struct at86rf215 *h, at86rf215_radio_t radio, uint8_t rcut)
{
	int ret = supports_rf(h, radio);
	if (ret) {
		return ret;
	}
	uint16_t reg = 0x0;
	if (radio == AT86RF215_RF09) {
		reg = REG_RF09_RXDFE;
	} else {
		reg = REG_RF24_RXDFE;
	}
	uint8_t val = 0;
	ret = at86rf215_reg_read_8(&val, reg);
	if (ret) {
		return ret;
	}
	val &= 0x1F;
	return at86rf215_reg_write_8(((rcut & 0x7) << 5) | val, reg);
}

/**
 * Controls the PA power
 * @param h the device handle
 * @param radio the RF frontend
 * @param pacur PA power control
 * @param power the output PA (0-31). 1-dB step resolution
 * @return 0 on success or negative error code
 */
int
at86rf215_set_pac(struct at86rf215 *h, at86rf215_radio_t radio, at86rf215_pacur_t pacur, uint8_t power)
{
	int ret = supports_rf(h, radio);
	if (ret) {
		return ret;
	}
	switch (pacur) {
		case AT86RF215_PACUR_22mA_RED:
		case AT86RF215_PACUR_18mA_RED:
		case AT86RF215_PACUR_11mA_RED:
		case AT86RF215_PACUR_NO_RED:
			break;
		default:
			return -AT86RF215_INVAL_PARAM;
	}
	if (power > 31) {
		power = 31;
	}

	uint16_t reg = 0x0;
	if (radio == AT86RF215_RF09) {
		reg = REG_RF09_PAC;
	} else {
		reg = REG_RF24_PAC;
	}
	return at86rf215_reg_write_8((pacur << 5) | power, reg);
}

/**
 * Set the channel of the RF frontend
 * @note the RF frontend should be in IEEE compliant channel mode. Otherwise the
 * -AT86RF215_INVAL_MODE error code is returned
 * @param h the device handle
 * @param radio the radio frontend
 * @param channel channel index
 * @return 0 on success or negative error code
 */
int at86rf215_set_channel(struct at86rf215 *h, at86rf215_radio_t radio, uint16_t channel)
{
	int ret = radio_ready(h, radio);
	if (ret) {
		return ret;
	}
	if (channel > 0x1F) {
		return -AT86RF215_INVAL_PARAM;
	}
	/* The radio should be in IEEE channel mode to set a channel */
	if (h->priv.radios[radio].cm != AT86RF215_CM_IEEE) {
		return -AT86RF215_INVAL_CONF;
	}
	if (radio == AT86RF215_RF09) {
		ret = at86rf215_reg_write_8(channel & 0xFF, REG_RF09_CNL);
		if (ret) {
			return ret;
		}
		ret = at86rf215_reg_write_8((h->priv.radios[radio].cm << 6) |
		                            ((channel >> 8) & 0x1), REG_RF09_CNM);
		if (ret) {
			return ret;
		}
	} else {
		ret = at86rf215_reg_write_8(channel & 0xFF, REG_RF24_CNL);
		if (ret) {
			return ret;
		}
		ret = at86rf215_reg_write_8((h->priv.radios[radio].cm << 6) |
		                            ((channel >> 8) & 0x1), REG_RF24_CNM);
		if (ret) {
			return ret;
		}
	}
	return AT86RF215_OK;
}

/**
 * Set the center frequency of the RF frontend.
 * @note the RF frontend should be in Fine Resolution mode. Otherwise the
 * -AT86RF215_INVAL_MODE error code is returned
 *
 * @param h the device handle
 * @param radio the RF frontend
 * @param freq the center frequency in Hz
 * @return 0 on success or negative error code
 */
int at86rf215_set_freq(struct at86rf215 *h, at86rf215_radio_t radio, uint32_t freq)
{
	int ret = radio_ready(h, radio);
	if (ret) {
		return ret;
	}
	/* The radio should be in fine freq mode to set the frequency */
	if (radio == AT86RF215_RF09) {
		uint32_t x = 0;
		if (h->priv.radios[AT86RF215_RF09].cm == AT86RF215_CM_FINE_RES_04) {
			if (freq < 389500000 || freq > 510000000) {
				return -AT86RF215_INVAL_PARAM;
			}
			x = ((freq - 377e6) * (1 << 16)) / 6.5e6;
		} else if (h->priv.radios[AT86RF215_RF09].cm == AT86RF215_CM_FINE_RES_09) {
			if (freq < 779000000 || freq > 1088000000) {
				return -AT86RF215_INVAL_PARAM;
			}
			x = ((freq - 754e6) * (1 << 16)) / 13e6;
		} else {
			return -AT86RF215_INVAL_CONF;
		}
		/* Apply the frequency setting */
		ret = at86rf215_reg_write_8((x >> 16) & 0xFF, REG_RF09_CCF0H);
		if (ret) {
			return ret;
		}
		ret = at86rf215_reg_write_8((x >> 8) & 0xFF, REG_RF09_CCF0L);
		if (ret) {
			return ret;
		}
		ret = at86rf215_reg_write_8(x & 0xFF, REG_RF09_CNL);
		if (ret) {
			return ret;
		}
		ret = at86rf215_reg_write_8(h->priv.radios[radio].cm << 6,
		                            REG_RF09_CNM);
		if (ret) {
			return ret;
		}
	} else {
		uint32_t x = 0;
		if (h->priv.radios[AT86RF215_RF24].cm == AT86RF215_CM_FINE_RES_24) {
			if (freq < 2400000000 || freq > 2486000000) {
				return -AT86RF215_INVAL_PARAM;
			}
			x = ((freq - 2366e6) * (1 << 16)) / 26e6;
		} else {
			return -AT86RF215_INVAL_CONF;
		}
		/* Apply the frequency setting */
		ret = at86rf215_reg_write_8((x >> 16) & 0xFF, REG_RF24_CCF0H);
		if (ret) {
			return ret;
		}
		ret = at86rf215_reg_write_8((x >> 8) & 0xFF, REG_RF24_CCF0L);
		if (ret) {
			return ret;
		}
		ret = at86rf215_reg_write_8(x & 0xFF, REG_RF24_CNL);
		if (ret) {
			return ret;
		}
		ret = at86rf215_reg_write_8(h->priv.radios[radio].cm << 6,
		                            REG_RF24_CNM);
		if (ret) {
			return ret;
		}
	}
	return AT86RF215_OK;
}

/**
 * Get the PLL lock status for the corresponding RF frontend
 * @param h the device handle
 * @param status pointer to store the lock status result
 * @param radio the RF frontend
 * @return 0 on success or negative error code
 */
int at86rf215_get_pll_ls(const struct at86rf215 *h, at86rf215_pll_ls_t *status, at86rf215_radio_t radio)
{
	int ret = radio_ready(h, radio);
	if (ret) {
		return ret;
	}
	uint16_t reg = 0;
	if (radio == AT86RF215_RF09) {
		reg = REG_RF09_PLL;
	} else {
		reg = REG_RF24_PLL;
	}
	uint8_t val = 0;
	ret = at86rf215_reg_read_8(&val, reg);
	if (ret) {
		return ret;
	}
	*status = (at86rf215_pll_ls_t)((val >> 1) & 0x1);
	return AT86RF215_OK;
}

/**
 * @brief Sets the RX bandwidth configuration
 *
 * @param h the device handle
 * @param radio the RF frontend
 * @param if_inv if 1, the IF will be inverted
 * @param if_shift if 1, the IF will be shifted at 2500
 * @param bw the RX bandwidth setting
 * @return 0 on success or negative error code
 */
int at86rf215_set_bw(struct at86rf215 *h, at86rf215_radio_t radio, uint8_t if_inv, uint8_t if_shift, at86rf215_rx_bw_t bw)
{
	int ret = radio_ready(h, radio);
	if (ret) {
		return ret;
	}

	switch (bw) {
		case AT86RF215_RF_BW160KHZ_IF250KHZ:
		case AT86RF215_RF_BW200KHZ_IF250KHZ:
		case AT86RF215_RF_BW250KHZ_IF250KHZ:
		case AT86RF215_RF_BW320KHZ_IF500KHZ:
		case AT86RF215_RF_BW400KHZ_IF500KHZ:
		case AT86RF215_RF_BW500KHZ_IF500KHZ:
		case AT86RF215_RF_BW630KHZ_IF1000KHZ:
		case AT86RF215_RF_BW800KHZ_IF1000KHZ:
		case AT86RF215_RF_BW1000KHZ_IF1000KHZ:
		case AT86RF215_RF_BW1250KHZ_IF2000KHZ:
		case AT86RF215_RF_BW1600KHZ_IF2000KHZ:
		case AT86RF215_RF_BW2000KHZ_IF2000KHZ:
			break;
		default:
			return -AT86RF215_INVAL_PARAM;
	}

	uint16_t reg = 0;
	if (radio == AT86RF215_RF09) {
		reg = REG_RF09_RXBWC;
	} else {
		reg = REG_RF24_RXBWC;
	}
	uint8_t val = ((if_inv & 0x1) << 5) | ((bw & 0x1) << 4) | bw;
	return at86rf215_reg_write_8(val, reg);
}

int at86rf215_set_edd(struct at86rf215 *h, at86rf215_radio_t radio, uint8_t df, at86rf215_edd_dtb_t dtb)
{
	int ret = radio_ready(h, radio);
	if (ret) {
		return ret;
	}

	uint16_t reg = 0;
	if (radio == AT86RF215_RF09) {
		reg = REG_RF09_EDD;
	} else {
		reg = REG_RF24_EDD;
	}
	uint8_t val = (df << 2) | dtb;
	return at86rf215_reg_write_8(val, reg);
}

/**
 * @brief Gets the RSSI of the RF frontend
 *
 * @param h the device handle
 * @param radio the RF frontend
 * @param rssi pointer to store the result
 * @return 0 on success, -AT86RF215_INVAL_VAL in case of invalid RSSI
 * or other appropriate negative error code
 */
int at86rf215_get_rssi(struct at86rf215 *h, at86rf215_radio_t radio, float *rssi)
{
	int ret = radio_ready(h, radio);
	if (ret) {
		return ret;
	}

	if (!rssi) {
		return -AT86RF215_INVAL_PARAM;
	}
	uint16_t reg = 0;
	if (radio == AT86RF215_RF09) {
		reg = REG_RF09_RSSI;
	} else {
		reg = REG_RF24_RSSI;
	}
	int8_t val = 0;
	ret = at86rf215_reg_read_8(&val, reg);
	if (ret) {
		return ret;
	}
	if (val == 127) {
		return -AT86RF215_INVAL_VAL;
	}
	*rssi = val;
	return AT86RF215_OK;
}


int at86rf215_get_edv(struct at86rf215 *h, at86rf215_radio_t radio, float *edv)
{
	int ret = radio_ready(h, radio);
	if (ret) {
		return ret;
	}

	if (!edv) {
		return -AT86RF215_INVAL_PARAM;
	}
	uint16_t reg = 0;
	if (radio == AT86RF215_RF09) {
		reg = REG_RF09_EDV;
	} else {
		reg = REG_RF24_EDV;
	}
	int8_t val = 0;
	ret = at86rf215_reg_read_8(&val, reg);
	if (ret) {
		return ret;
	}
	if (val == 127) {
		return -AT86RF215_INVAL_VAL;
	}
	*edv = val;
	return AT86RF215_OK;
}

int at86rf215_get_agc_gain(struct at86rf215 *h, at86rf215_radio_t radio, uint8_t *gain)
{
	int ret = radio_ready(h, radio);
	if (ret) {
		return ret;
	}

	if (!gain) {
		return -AT86RF215_INVAL_PARAM;
	}
	uint16_t reg = 0;
	if (radio == AT86RF215_RF09) {
		reg = REG_RF09_AGCS;
	} else {
		reg = REG_RF24_AGCS;
	}
	uint8_t val = 0;
	ret = at86rf215_reg_read_8(&val, reg);
	if (ret) {
		return ret;
	}
	*gain = val & 0x1F;
	return AT86RF215_OK;
}

static int bb_conf_mrfsk(struct at86rf215 *h, at86rf215_radio_t radio, const struct at86rf215_bb_conf *conf)
{
	/*
	 * In order to process efficiently the configuration registers for
	 * the two available baseband cores, we use the constant offset of the
	 * configuration address space of the two cores. For some RF related
	 * configuration registers (e.g. TXDFE, RXDFE) the offset between the
	 * two address spaces for the radio configuration is exactly the same.
	 */
	uint16_t offset;
	switch (radio) {
		case AT86RF215_RF09:
			offset = 0;
			break;
		case AT86RF215_RF24:
			offset = 256;
			break;
		default:
			return -AT86RF215_INVAL_PARAM;
	}

	uint8_t txdfe = 0x0;
	uint8_t rxdfe = 0x0;
	switch (conf->fsk.srate) {
		case AT86RF215_FSK_SRATE_50:
			if (h->priv.version == 1) {
				txdfe = AT86RF215_SR_400KHZ;
			} else {
				txdfe = AT86RF215_SR_500KHZ;
			}
			rxdfe = AT86RF215_SR_400KHZ;
			break;
		case AT86RF215_FSK_SRATE_100:
			if (h->priv.version == 1) {
				txdfe = AT86RF215_SR_800KHZ;
			} else {
				txdfe = AT86RF215_SR_1000KHZ;
			}
			rxdfe = AT86RF215_SR_800KHZ;
			break;
		case AT86RF215_FSK_SRATE_150:
			txdfe = AT86RF215_SR_2000KHZ;
			rxdfe = AT86RF215_SR_1000KHZ;
			break;
		case AT86RF215_FSK_SRATE_200:
			txdfe = AT86RF215_SR_2000KHZ;
			rxdfe = AT86RF215_SR_1000KHZ;
			break;
		case AT86RF215_FSK_SRATE_300:
			txdfe = AT86RF215_SR_4000KHZ;
			rxdfe = AT86RF215_SR_2000KHZ;
			break;
		case AT86RF215_FSK_SRATE_400:
			txdfe = AT86RF215_SR_4000KHZ;
			rxdfe = AT86RF215_SR_2000KHZ;
			break;
		default:
			return -AT86RF215_INVAL_PARAM;
	}

	uint8_t val;
	/* FSKC0 */
	switch (conf->fsk.mord) {
		case AT86RF215_2FSK:
			break;
		case AT86RF215_4FSK:
			/* Check for 4FSK restrictions (h >= 1, BT = 2) */
			if (conf->fsk.bt != 2 || conf->fsk.midx < AT86RF215_MIDX_3) {
				return -AT86RF215_INVAL_CONF;
			}
			if (conf->fsk.midx < AT86RF215_MIDX_3
			    && conf->fsk.midxs == AT86RF215_MIDXS_78) {
				return -AT86RF215_INVAL_CONF;
			}
		default:
			return -AT86RF215_INVAL_PARAM;
	}
	val = conf->fsk.mord;
	switch (conf->fsk.midx) {
		case AT86RF215_MIDX_0:
		case AT86RF215_MIDX_1:
		case AT86RF215_MIDX_2:
		case AT86RF215_MIDX_3:
		case AT86RF215_MIDX_4:
		case AT86RF215_MIDX_5:
		case AT86RF215_MIDX_6:
		case AT86RF215_MIDX_7:
			val |= conf->fsk.midx << 1;
			break;
		default:
			return -AT86RF215_INVAL_PARAM;
	}

	switch (conf->fsk.midxs) {
		case AT86RF215_MIDXS_78:
		case AT86RF215_MIDXS_88:
		case AT86RF215_MIDXS_98:
		case AT86RF215_MIDXS_108:
			val |= conf->fsk.midxs << 4;
			break;
		default:
			return -AT86RF215_INVAL_PARAM;
	}
	switch (conf->fsk.bt) {
		case AT86RF215_FSK_BT_05:
		case AT86RF215_FSK_BT_10:
		case AT86RF215_FSK_BT_15:
		case AT86RF215_FSK_BT_20:
			val |= conf->fsk.bt << 6;
			break;
		default:
			return -AT86RF215_INVAL_PARAM;
	}
	int ret = at86rf215_reg_write_8(val, REG_BBC0_FSKC0 + offset);
	if (ret) {
		return ret;
	}

	/* FSKC1 */
	val = conf->fsk.srate | (conf->fsk.fi << 5)
	      | ((conf->fsk.preamble_length >> 2) & 0xC0);
	ret = at86rf215_reg_write_8(val, REG_BBC0_FSKC1 + offset);
	if (ret) {
		return ret;
	}

	/* FSKC2 */
	val = conf->fsk.fecie;
	switch (conf->fsk.fecs) {
		case AT86RF215_FSK_FEC_NRNSC:
		case AT86RF215_FSK_FEC_RSC:
			val |= conf->fsk.fecs << 1;
			break;
		default:
			return -AT86RF215_INVAL_PARAM;
	}
	val |= (conf->fsk.pri << 2) | (conf->fsk.mse << 3)
	       | (conf->fsk.rxpto << 4);
	switch (conf->fsk.rxo) {
		case AT86RF215_FSK_RXO_6DB:
		case AT86RF215_FSK_RXO_12DB:
		case AT86RF215_FSK_RXO_18DB:
		case AT86RF215_FSK_RXO_DISABLED:
			val |= conf->fsk.rxo << 5;
			break;
		default:
			return -AT86RF215_INVAL_PARAM;
	}
	val |= conf->fsk.pdtm << 7;
	ret = at86rf215_reg_write_8(val, REG_BBC0_FSKC2 + offset);
	if (ret) {
		return ret;
	}

	/* FSKC3 */
	val = (conf->fsk.sfd_threshold << 4) | conf->fsk.preamble_threshold;
	ret = at86rf215_reg_write_8(val, REG_BBC0_FSKC3 + offset);
	if (ret) {
		return ret;
	}

	/* FSKC4 */
	switch (conf->fsk.csfd0) {
		case AT86RF215_SFD_UNCODED_IEEE:
		case AT86RF215_SFD_UNCODED_RAW:
		case AT86RF215_SFD_CODED_IEEE:
		case AT86RF215_SFD_CODED_RAW:
			val = conf->fsk.csfd0;
			break;
		default:
			return -AT86RF215_INVAL_PARAM;
	}
	switch (conf->fsk.csfd1) {
		case AT86RF215_SFD_UNCODED_IEEE:
		case AT86RF215_SFD_UNCODED_RAW:
		case AT86RF215_SFD_CODED_IEEE:
		case AT86RF215_SFD_CODED_RAW:
			val |= conf->fsk.csfd1 << 2;
			break;
		default:
			return -AT86RF215_INVAL_PARAM;
	}
	val |= (conf->fsk.rawrbit << 4) | (conf->fsk.sfd32 << 5)
	       | (conf->fsk.sfdq << 6);
	ret = at86rf215_reg_write_8(val, REG_BBC0_FSKC4 + offset);
	if (ret) {
		return ret;
	}

	/* FSKPLL */
	val = conf->fsk.preamble_length;
	ret = at86rf215_reg_write_8(val, REG_BBC0_FSKPLL + offset);
	if (ret) {
		return ret;
	}

	/* SFD configuration */
	ret = at86rf215_reg_write_8(conf->fsk.sfd0, REG_BBC0_FSKSFD0L + offset);
	if (ret) {
		return ret;
	}
	ret = at86rf215_reg_write_8(conf->fsk.sfd0 >> 8,
	                            REG_BBC0_FSKSFD0H + offset);
	if (ret) {
		return ret;
	}
	ret = at86rf215_reg_write_8(conf->fsk.sfd1, REG_BBC0_FSKSFD1L + offset);
	if (ret) {
		return ret;
	}
	ret = at86rf215_reg_write_8(conf->fsk.sfd1 >> 8,
	                            REG_BBC0_FSKSFD1H + offset);
	if (ret) {
		return ret;
	}

	/* FSKPHRTX */
	val = conf->fsk.rb1 | (conf->fsk.rb2 << 1) | (conf->fsk.dw << 2)
	      | (conf->fsk.sfd << 3);
	ret = at86rf215_reg_write_8(val, REG_BBC0_FSKPLL + offset);
	if (ret) {
		return ret;
	}

	/* FSKDM */
	ret = at86rf215_reg_write_8(conf->fsk.dm | (conf->fsk.preemphasis << 1),
	                            REG_BBC0_FSKDM + offset);
	if (ret) {
		return ret;
	}

	/* Both TXDFE and FSK DM should have the direct modulation option enabled*/
	txdfe |= conf->fsk.dm << 4;
	ret = at86rf215_reg_read_8(&val, REG_RF09_TXDFE + offset);
	if (ret) {
		return ret;
	}
	txdfe |= (val & 0xE0);
	ret = at86rf215_reg_write_8(txdfe, REG_RF09_TXDFE + offset);
	if (ret) {
		return ret;
	}

	/* PRemphasis filter setup */
	ret = at86rf215_reg_write_8(conf->fsk.preemphasis_taps,
	                            REG_BBC0_FSKPE0 + offset);
	if (ret) {
		return ret;
	}

	ret = at86rf215_reg_write_8(conf->fsk.preemphasis_taps >> 8,
	                            REG_BBC0_FSKPE1 + offset);
	if (ret) {
		return ret;
	}

	ret = at86rf215_reg_write_8(conf->fsk.preemphasis_taps >> 16,
	                            REG_BBC0_FSKPE2 + offset);
	if (ret) {
		return ret;
	}

	/* Apply RX sampling rate */
	ret = at86rf215_reg_read_8(&val, REG_RF09_RXDFE + offset);
	if (ret) {
		return ret;
	}
	rxdfe |= (val & 0xE0);
	ret = at86rf215_reg_write_8(rxdfe, REG_RF09_RXDFE + offset);
	if (ret) {
		return ret;
	}

	return AT86RF215_OK;
}

static int bb_conf_mrofdm(struct at86rf215 *h, at86rf215_radio_t radio, const struct at86rf215_bb_conf *conf)
{
	/*
	 * In order to process efficiently the configuration registers for
	 * the two available baseband cores, we use the constant offset of the
	 * configuration address space of the two cores. For some RF related
	 * configuration registers (e.g. TXDFE, RXDFE) the offset between the
	 * two address spaces for the radio configuration is exactly the same.
	 */
	uint16_t offset;
	switch (radio) {
		case AT86RF215_RF09:
			offset = 0;
			break;
		case AT86RF215_RF24:
			offset = 256;
			break;
		default:
			return -AT86RF215_INVAL_PARAM;
	}

	uint8_t val;

	switch (conf->ofdm.mcs) {
		case AT86RF215_BPSK_1_2_N_4_FREQ_REP:
		case AT86RF215_BPSK_1_2_N_2_FREQ_REP:
		case AT86RF215_QPSK_1_2_N_2_FREQ_REP:
		case AT86RF215_QPSK_1_2:
		case AT86RF215_QPSK_3_4:
		case AT86RF215_16_QAM_1_2:
		case AT86RF215_16_QAM_3_4:
			val = conf->ofdm.mcs;
			break;
		default:
			return -AT86RF215_INVAL_PARAM;
	}
	int ret = at86rf215_reg_write_8(val, REG_BBC0_OFDMPHRTX + offset);
	if (ret) {
		return ret;
	}

	switch (conf->ofdm.ssrx) {
		case AT86RF215_SSRX_0:
		case AT86RF215_SSRX_1:
		case AT86RF215_SSRX_2:
		case AT86RF215_SSRX_3:
			val = conf->ofdm.ssrx << 6;
			break;
		default:
			return -AT86RF215_INVAL_PARAM;
	}

	switch (conf->ofdm.sstx) {
		case AT86RF215_SSTX_0:
		case AT86RF215_SSTX_1:
		case AT86RF215_SSTX_2:
		case AT86RF215_SSTX_3:
			val |= ((conf->ofdm.sstx & 0x3) << 4);
			break;
		default:
			return -AT86RF215_INVAL_PARAM;
	}

	val |= (conf->ofdm.lfo << 3);

	val |= (conf->ofdm.poi << 2);

	switch (conf->ofdm.opt) {
		case AT86RF215_OPT_1:
		case AT86RF215_OPT_2:
		case AT86RF215_OPT_3:
		case AT86RF215_OPT_4:
			val |= conf->ofdm.opt;
			break;
		default:
			return -AT86RF215_INVAL_PARAM;
	}
	ret = at86rf215_reg_write_8(val, REG_BBC0_OFDMC + offset);
	if (ret) {
		return ret;
	}

	val = conf->ofdm.pdt << 5;
	val |= conf->ofdm.rxo << 4;
	ret = at86rf215_reg_write_8(val, REG_BBC0_OFDMSW + offset);
	if (ret) {
		return ret;
	}

	return AT86RF215_OK;
}

/**
 * @note The baseband core is explicitly disabled with the call of this function
 * (PC.BBEN = 0). Use the at86rf215_bb_enable() to enable it.
 * @param h the device handle
 * @param radio the RF frontend
 * @param conf the condiguration of the baseband core
 * @return 0 on success or negative error code
 */
int at86rf215_bb_conf(struct at86rf215 *h, at86rf215_radio_t radio, const struct at86rf215_bb_conf *conf)
{
	int ret = supports_rf(h, radio);
	if (ret) {
		return ret;
	}
	if (!conf) {
		return -AT86RF215_INVAL_PARAM;
	}
	uint8_t val = conf->pt | (conf->fcst << 3) | (conf->txafcs << 4)
	              | (conf->fcsfe << 6) | (conf->ctx << 7);

	uint16_t reg = 0;
	if (radio == AT86RF215_RF09) {
		reg = REG_BBC0_PC;
	} else {
		reg = REG_BBC1_PC;
	}
	ret = at86rf215_reg_write_8(val, reg);
	if (ret) {
		return ret;
	}

	switch (conf->pt) {
		case AT86RF215_BB_MRFSK:
			ret = bb_conf_mrfsk(h, radio, conf);
			break;
		case AT86RF215_BB_MROFDM:
			ret = bb_conf_mrofdm(h, radio, conf);
			break;
		case AT86RF215_BB_MROQPSK:
		case AT86RF215_BB_PHYOFF:
			return -AT86RF215_NOT_IMPL;
		default:
			return -AT86RF215_INVAL_PARAM;
	}
	if (ret) {
		return ret;
	}
	/* Copy internally the configuration */
	memcpy(&h->priv.bbc[radio], conf, sizeof(struct at86rf215_bb_conf));
	return AT86RF215_OK;
}

/**
 * Enables the baseband core of the corresponding radio frontend
 * @param h the device handle
 * @param radio the RF frontend
 * @param en 1 to enable, 0 to disable
 * @return 0 on success or negative error code
 */
int at86rf215_bb_enable(struct at86rf215 *h, at86rf215_radio_t radio, uint8_t en)
{
	int ret = supports_rf(h, radio);
	if (ret) {
		return ret;
	}
	const struct at86rf215_bb_conf *conf = &h->priv.bbc[radio];
	uint8_t val = conf->pt | ((en & 0x1) << 2) | (conf->fcst << 3)
	              | (conf->txafcs << 4) | (conf->fcsfe << 6)
	              | (conf->ctx << 7);
	uint16_t reg = 0;
	if (radio == AT86RF215_RF09) {
		reg = REG_BBC0_PC;
	} else {
		reg = REG_BBC1_PC;
	}
	ret = at86rf215_reg_write_8(val, reg);
	if (ret) {
		return ret;
	}
	return AT86RF215_OK;
}

/**
 * Writes PSDU data to the TX FIFO
 * @param h the device handle
 * @param radio the RF frontend
 * @param b buffer with the PSDU data
 * @param len the size of the PSDU
 * @return 0 on success or negative error code
 */
static int write_tx_fifo(const struct at86rf215 *h, at86rf215_radio_t radio, const uint8_t *b, size_t len)
{
	int ret = 0;
	if (radio == AT86RF215_RF09) {
		/* Declare the size of the PSDU */
		ret = at86rf215_reg_write_8(len & 0xFF, REG_BBC0_TXFLL);
		if (ret) {
			return ret;
		}
		ret = at86rf215_reg_write_8((len >> 8) & 0xFF, REG_BBC0_TXFLH);
		if (ret) {
			return ret;
		}

		/* Fill the FIFO */
		uint8_t mosi[2] = {(REG_BBC0_FBTXS >> 8) | 0x80,
		                   REG_BBC0_FBTXS & 0xFF
		                  };
		ret = at86rf215_spi_write(mosi, 2);
		if (ret) {
			return ret;
		}
		ret = at86rf215_spi_write(b, len);
		if (ret) {
			return ret;
		}
	} else {
		/* Declare the size of the PSDU */
		ret = at86rf215_reg_write_8(len & 0xFF, REG_BBC1_TXFLL);
		if (ret) {
			return ret;
		}
		ret = at86rf215_reg_write_8((len >> 8) & 0xFF, REG_BBC1_TXFLH);
		if (ret) {
			return ret;
		}

		/* Fill the FIFO */
		uint8_t mosi[2] = {(REG_BBC1_FBTXS >> 8) | 0x80,
		                   REG_BBC1_FBTXS & 0xFF
		                  };
		ret = at86rf215_spi_write(mosi, 2);
		if (ret) {
			return ret;
		}
		ret = at86rf215_spi_write(b, len);
		if (ret) {
			return ret;
		}
	}
	return AT86RF215_OK;
}

/**
 * Transmits a frame using the configured baseband core mode
 * @note the chip mode should ensure that the corresponding baseband core
 * is enabled
 * @param h the device handle
 * @param radio the RF fronted
 * @param psdu the data to send
 * @param len the number of bytes to send
 * @return 0 on success or negative error code
 */
int at86rf215_tx_frame(struct at86rf215 *h, at86rf215_radio_t radio, const uint8_t *psdu, size_t len)
{
	int ret = supports_rf(h, radio);
	if (ret) {
		return ret;
	}
	if (!psdu) {
		return -AT86RF215_INVAL_PARAM;
	}

	/*
	 * In order to transmit the frame, the corresponding baseband core should
	 * be enabled
	 */
	if (h->priv.chpm == AT86RF215_RF_MODE_RF) {
		return -AT86RF215_INVAL_CHPM;
	}
	if (radio == AT86RF215_RF09) {
		if (h->priv.chpm == AT86RF215_RF_MODE_BBRF09) {
			return -AT86RF215_INVAL_CHPM;
		}
	} else {
		if (h->priv.chpm == AT86RF215_RF_MODE_BBRF24) {
			return -AT86RF215_INVAL_CHPM;
		}
	}

	// TODO: May be can remove
	printf("op_state = %d\n", h->priv.radios[radio].op_state);
	while (h->priv.radios[radio].op_state != AT86RF215_OP_STATE_IDLE) {
	}

	h->priv.radios[radio].op_state = AT86RF215_OP_STATE_TX;
	ret = write_tx_fifo(h, radio, psdu, len);
	if (ret) {
		return ret;
	}

	// TODO: Remove after verify
	at86rf215_rf_state_t radio_state;
	at86rf215_get_state(h, &radio_state, radio);
	printf("radio state = %d\n", radio_state);

	ret = at86rf215_set_cmd(h, AT86RF215_CMD_RF_TXPREP, radio);
	if (ret) {
		return ret;
	}

	at86rf215_get_state(h, &radio_state, radio);
	printf("radio state = %d\n", radio_state);

	uint8_t irqs;
	/* Wait unitl the TXPREP is reached */
	//event_node_wait_ready(&h->priv.radios[radio].trxready);
	/*
	do {
		//at86rf215_get_state(h, &radio_state, radio);
		at86rf215_reg_read_8(&irqs, REG_RF09_IRQS);
		printf("IRQS = %x\n", irqs);
	} while(irqs & BIT(1) == 0);
	*/

	printf("Sending TX CMD.\n");
	/* Data are on the FIFO. Issue the TX cmd to send them */
	return at86rf215_set_cmd(h, AT86RF215_CMD_RF_TX, radio);
}

/**
 * Configures the IQ mode for a particular RF frontend.
 * @note Some settings are applied for the IQ mode of both sub-1 GHz and the
 * 2.4 GHz RF frontend.
 * @note This function does not change the chip mode in any way. Users should
 * call the at86rf215_set_mode() to chnage the operational mode of the IC
 * @param h the device handle
 * @param radio the RF frontend
 * @param conf IQ configration
 * @return 0 on success or negative error code
 */
int at86rf215_iq_conf(struct at86rf215 *h, at86rf215_radio_t radio, const struct at86rf215_iq_conf *conf)
{
	int ret = supports_rf(h, radio);
	if (ret) {
		return ret;
	}
	if (!conf) {
		return -AT86RF215_INVAL_PARAM;
	}

	switch (conf->drv) {
		case AT86RF215_LVDS_DRV1:
		case AT86RF215_LVDS_DRV2:
		case AT86RF215_LVDS_DRV3:
		case AT86RF215_LVDS_DRV4:
			break;
		default:
			return -AT86RF215_INVAL_PARAM;
	}

	switch (conf->cmv) {
		case AT86RF215_LVDS_CMV150:
		case AT86RF215_LVDS_CMV200:
		case AT86RF215_LVDS_CMV250:
		case AT86RF215_LVDS_CMV300:
			break;
		default:
			return -AT86RF215_INVAL_PARAM;
	}
	uint8_t val = conf->eec | (conf->cmv1v2 << 1) | (conf->cmv << 2)
	              | (conf->drv << 4) | (conf->extlb << 7);
	ret = at86rf215_reg_write_8(val, REG_RF_IQIFC0);
	if (ret) {
		return ret;
	}
	/* Set the RF_IQIFC1 but leave the CHPM unchanged  */
	ret = at86rf215_reg_read_8(&val, REG_RF_IQIFC1);
	if (ret) {
		return ret;
	}
	val &= BIT(4) | BIT(5) | BIT(6);
	val |= conf->skedrv;
	ret = at86rf215_reg_write_8(val, REG_RF_IQIFC1);
	if (ret) {
		return ret;
	}

	/* Apply the TX sampling rate settings */
	ret = set_txdfe(radio, conf->trcut, 0, conf->tsr);
	if (ret) {
		return ret;
	}

	/* Apply the RX sampling rate settings */
	ret = set_rxdfe(radio, conf->rrcut, conf->rsr);
	if (ret) {
		return ret;
	}
	return AT86RF215_OK;
}


int at86rf215_get_irq_mask(const struct at86rf215 *h, uint8_t *mask, at86rf215_radio_t radio)
{
	int ret = radio_ready(h, radio);
	if (ret) {
		return ret;
	}
	uint16_t reg = 0;
	if (radio == AT86RF215_RF09) {
		reg = REG_RF09_IRQM;
	} else {
		reg = REG_RF24_IRQM;
	}
	uint8_t val = 0;
	ret = at86rf215_reg_read_8(&val, reg);
	if (ret) {
		return ret;
	}
	*mask = val;
	return AT86RF215_OK;
}

int at86rf215_calibrate_device(struct at86rf215 *h, at86rf215_radio_t radio, int* i, int* q)
{
    int cal_i[NUM_CAL_STEPS] = {0};
    int cal_q[NUM_CAL_STEPS] = {0};
    bool override_flag = h->override_cal;
    h->override_cal = false;
    
    for (int i = 0; i < NUM_CAL_STEPS; i ++)
    {
		at86rf215_set_cmd(h, AT86RF215_CMD_RF_TRXOFF, radio);
		at86rf215_set_cmd(h, AT86RF215_CMD_RF_TXPREP, radio);

        io_utils_usleep(10000);

        at86rf215_radio_get_tx_iq_calibration(h, radio, &cal_i[i], &cal_q[i]);
        printf("Calibration of modem: I=%d, Q=%d\n", cal_i[i], cal_q[i]);
    }

    // medians
    int cal_i_med = median(cal_i, NUM_CAL_STEPS);
    int cal_q_med = median(cal_q, NUM_CAL_STEPS);
    printf("Calibration Results of the modem: I=%d, Q=%d\n", cal_i_med, cal_q_med);
    h->override_cal = override_flag;
    return 0;
}

void at86rf215_radio_set_tx_iq_calibration(const struct at86rf215 *h, at86rf215_radio_t radio, int cal_i, int cal_q)
{
	/*
	int ret = radio_ready(h, radio);
	if (ret) {
		return;
	}
	*/

	uint16_t reg_i = 0;
	uint16_t reg_q = 0;
	if (radio == AT86RF215_RF09) {
		reg_i = REG_RF09_TXCI;
		reg_q = REG_RF09_TXCQ;
	} else {
		reg_i = REG_RF24_TXCI;
		reg_q = REG_RF24_TXCQ;
	}
	uint8_t val_i = cal_i & 0x3F;
	uint8_t val_q = cal_q & 0x3F;

	at86rf215_reg_write_8(val_i, reg_i);
	at86rf215_reg_write_8(val_q, reg_q);

    // RFn_TXCI – Transmit calibration I path
    //  The register contains information about the TX LO leakage calibration value of the transmit I path. At the transition
    //  process from state TRXOFF to TX the calibration is started automatically

    // RFn_TXCQ – Transmit calibration Q path
    //  The register contains information about the TX LO leakage calibration value of the transmit Q path. At the transition
    //  process from state TRXOFF to TX the calibration is started automatically.
}

//==================================================================================
void at86rf215_radio_get_tx_iq_calibration(const struct at86rf215 *h, at86rf215_radio_t radio,
                                                int *cal_i, int *cal_q)
{
	/*
	int ret = radio_ready(h, radio);
	if (ret) {
		return;
	}
	*/

	uint16_t reg_i = 0;
	uint16_t reg_q = 0;
	if (radio == AT86RF215_RF09) {
		reg_i = REG_RF09_TXCI;
		reg_q = REG_RF09_TXCQ;
	} else {
		reg_i = REG_RF24_TXCI;
		reg_q = REG_RF24_TXCQ;
	}

	uint8_t val_i = 0;
	uint8_t val_q = 0;

	at86rf215_reg_read_8(&val_i, reg_i);
	at86rf215_reg_read_8(&val_q, reg_q);

	*cal_i = val_i & 0x3F;
    *cal_q = val_q & 0x3F;
}
