#include "at86rf215_bb_usb.h"
#include "at86rf215_bb_regs.h"
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdlib.h>
#include <unistd.h>
#include "../io_utils/io_utils.h"
#include "ftd2xx.h"
#include "libft4222.h"
#include "time.h"

#define INIT_MAGIC_VAL 0x92c2f0e3

#define SLAVE_SELECT(x) (1 << (x))

#define NUM_CAL_STEPS 7

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

static FT_HANDLE            ftHandle = (FT_HANDLE)NULL;
static FT_DEVICE_LIST_INFO_NODE *devInfo = NULL;

static int exercise4222(DWORD locationId)
{
    int                  success = 0;
    FT_STATUS            ftStatus;
    FT4222_STATUS        ft4222Status;
    FT4222_Version       ft4222Version;

    ftStatus = FT_OpenEx((PVOID)(uintptr_t)locationId, 
                         FT_OPEN_BY_LOCATION, 
                         &ftHandle);
    if (ftStatus != FT_OK)
    {
        printf("FT_OpenEx failed (error %d)\n", 
               (int)ftStatus);
        if (ftHandle != (FT_HANDLE)NULL)
        {
            (void)FT4222_UnInitialize(ftHandle);
            (void)FT_Close(ftHandle);
            return success;
        }
    }
    
    ft4222Status = FT4222_GetVersion(ftHandle,
                                     &ft4222Version);
    if (FT4222_OK != ft4222Status)
    {
        printf("FT4222_GetVersion failed (error %d)\n",
               (int)ft4222Status);
        if (ftHandle != (FT_HANDLE)NULL)
        {
            (void)FT4222_UnInitialize(ftHandle);
            (void)FT_Close(ftHandle);
            return success;
        }
    }
    
    printf("Chip version: %08X, LibFT4222 version: %08X\n",
           (unsigned int)ft4222Version.chipVersion,
           (unsigned int)ft4222Version.dllVersion);

    // Configure the FT4222 as an SPI Master.
    ft4222Status = FT4222_SPIMaster_Init(
                        ftHandle, 
                        SPI_IO_SINGLE, // 1 channel
                        CLK_DIV_32, // 60 MHz / 32 == 1.875 MHz
                        CLK_IDLE_LOW, // clock idles at logic 0
                        CLK_LEADING, // data captured on rising edge
                        SLAVE_SELECT(0)); // Use SS0O for slave-select
    if (FT4222_OK != ft4222Status)
    {
        printf("FT4222_SPIMaster_Init failed (error %d)\n",
               (int)ft4222Status);
        if (ftHandle != (FT_HANDLE)NULL)
        {
            (void)FT4222_UnInitialize(ftHandle);
            (void)FT_Close(ftHandle);
            return success;
        }
    }

    ft4222Status = FT4222_SPI_SetDrivingStrength(ftHandle,
                                                 DS_8MA,
                                                 DS_8MA,
                                                 DS_8MA);
    if (FT4222_OK != ft4222Status)
    {
        printf("FT4222_SPI_SetDrivingStrength failed (error %d)\n",
               (int)ft4222Status);
        if (ftHandle != (FT_HANDLE)NULL)
        {
            (void)FT4222_UnInitialize(ftHandle);
            (void)FT_Close(ftHandle);
            return success;
        }
    }

    success = 1;

    return success;
}

static int testFT4222(void)
{
    FT_STATUS                 ftStatus;
    DWORD                     numDevs = 0;
    int                       i;
    int                       retCode = 0;
    
    ftStatus = FT_CreateDeviceInfoList(&numDevs);
    if (ftStatus != FT_OK) 
    {
        printf("FT_CreateDeviceInfoList failed (error code %d)\n", 
               (int)ftStatus);
        retCode = -10;
        free(devInfo);
        return retCode;
    }
    
    if (numDevs == 0)
    {
        printf("No devices connected.\n");
        retCode = -20;
        free(devInfo);
        return retCode;
    }

    /* Allocate storage */
    devInfo = calloc((size_t)numDevs,
                     sizeof(FT_DEVICE_LIST_INFO_NODE));
    if (devInfo == NULL)
    {
        printf("Allocation failure.\n");
        retCode = -30;
        free(devInfo);
        return retCode;
    }
    
    /* Populate the list of info nodes */
    ftStatus = FT_GetDeviceInfoList(devInfo, &numDevs);
    if (ftStatus != FT_OK)
    {
        printf("FT_GetDeviceInfoList failed (error code %d)\n",
               (int)ftStatus);
        retCode = -40;
        free(devInfo);
        return retCode;
    }

    for (i = 0; i < (int)numDevs; i++) 
    {
        if (devInfo[i].Type == FT_DEVICE_4222H_3)
        {
            printf("\nDevice %d is FT4222H in mode 3 (single Master or Slave):\n",i);
            printf("  0x%08x  %s  %s\n", 
                   (unsigned int)devInfo[i].ID,
                   devInfo[i].SerialNumber,
                   devInfo[i].Description);
            (void)exercise4222(devInfo[i].LocId);
            break;
        }
    }

    return retCode;
}

int at86rf215_usb_read(uint8_t *out, uint8_t *in, size_t len)
{
	int ret;
    uint16 sizeTransferred;
    FT4222_STATUS ft4222Status;

    ft4222Status = FT4222_SPIMaster_SingleReadWrite(ftHandle, out, in, len, &sizeTransferred, true);
	if ((sizeTransferred == len) || (ft4222Status == FT4222_OK))
	{
		ret = AT86RF215_OK;
	}
	else
	{
		ret = -AT86RF215_SPI_READ_FAILED;
	}
	return ret;
}

int at86rf215_usb_write(uint8_t *in, size_t len)
{
	int ret;
    uint16 sizeTransferred;
    FT4222_STATUS ft4222Status;

    ft4222Status = FT4222_SPIMaster_SingleWrite(ftHandle, in, len, &sizeTransferred, true);
	if ((sizeTransferred == len) || (ft4222Status == FT4222_OK))
	{
		ret = AT86RF215_OK;
	}
	else
	{
		ret = -AT86RF215_SPI_WRITE_FAILED;
	}
	return ret;
}

int at86rf215_usb_reg_read_8(uint8_t *out, uint16_t reg)
{
	if (!out) {
		return -AT86RF215_INVAL_PARAM;
	}
	int ret = 0;

	/* Construct properly the MOSI buffer */
	uint8_t mosi[3] = {(reg >> 8) & 0x3F, reg & 0xFF, 0x0};
	uint8_t miso[3] = {0x0, 0x0, 0x0};
	ret = at86rf215_usb_read(miso, mosi, 3);
	if (ret) {
		return ret;
	}
	*out = miso[2];
	return AT86RF215_OK;
}

int at86rf215_usb_reg_write_8(const uint8_t in, uint16_t reg)
{
	int ret = 0;

	/* Construct properly the MOSI buffer */
	uint8_t mosi[3] = {(reg >> 8) | 0x80, reg & 0xFF, in};
	ret = at86rf215_usb_write(mosi, 3);
	if (ret) {
		return ret;
	}

	return AT86RF215_OK;
}

int at86rf215_usb_set_bbc_irq_mask(struct at86rf215 *h, at86rf215_radio_t radio, uint8_t mask)
{
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
	return at86rf215_usb_reg_write_8(mask, reg);
}

int at86rf215_usb_set_radio_irq_mask(struct at86rf215 *h, at86rf215_radio_t radio, uint8_t mask)
{
	uint16_t reg = 0x0;
	if (radio == AT86RF215_RF09) {
		reg = REG_RF09_IRQM;
	} else {
		reg = REG_RF24_IRQM;
	}
	return at86rf215_usb_reg_write_8(mask, reg);
}

int at86rf215_usb_get_state(const struct at86rf215 *h, at86rf215_rf_state_t *state, at86rf215_radio_t radio)
{
	int ret = 0;
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
	ret = at86rf215_usb_reg_read_8(&val, reg);
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

int at86rf215_usb_set_cmd(struct at86rf215 *h, at86rf215_rf_cmd_t cmd, at86rf215_radio_t radio)
{
    int ret;
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

	ret = at86rf215_usb_reg_write_8(cmd, reg);

	/*Errata #6:    State Machine Command RFn_CMD=TRXOFF may not be succeeded
                    Description: If the current state is different from SLEEP, the execution of the command TRXOFF may fail.
                    Software workaround: Check state by reading register RFn_STATE Repeat the command RFn_CMD=TRXOFF
                    if the target state was not reached.
    */
    if (cmd == AT86RF215_CMD_RF_TRXOFF)
    {
        int retries = 5;
		at86rf215_rf_state_t radio_state;
		at86rf215_usb_get_state(h, &radio_state, radio);
        while (radio_state != cmd && retries--)
        {
            io_utils_usleep(100);
            at86rf215_usb_reg_write_8(cmd, reg);
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
            at86rf215_usb_radio_set_tx_iq_calibration(h, radio, i, q);
        }
    }
	return ret;
}

int at86rf215_usb_calibrate_device(struct at86rf215 *h, at86rf215_radio_t radio, int* i, int* q)
{
    int cal_i[NUM_CAL_STEPS] = {0};
    int cal_q[NUM_CAL_STEPS] = {0};
    bool override_flag = h->override_cal;
    h->override_cal = false;
    
    for (int i = 0; i < NUM_CAL_STEPS; i ++)
    {
		at86rf215_usb_set_cmd(h, AT86RF215_CMD_RF_TRXOFF, radio);
		at86rf215_usb_set_cmd(h, AT86RF215_CMD_RF_TXPREP, radio);

        io_utils_usleep(10000);

        at86rf215_usb_radio_get_tx_iq_calibration(h, radio, &cal_i[i], &cal_q[i]);
        printf("Calibration of modem: I=%d, Q=%d\n", cal_i[i], cal_q[i]);
    }

    // medians
    int cal_i_med = median(cal_i, NUM_CAL_STEPS);
    int cal_q_med = median(cal_q, NUM_CAL_STEPS);
    printf("Calibration Results of the modem: I=%d, Q=%d\n", cal_i_med, cal_q_med);
    h->override_cal = override_flag;
    return 0;
}

void at86rf215_usb_radio_set_tx_iq_calibration(const struct at86rf215 *h, at86rf215_radio_t radio, int cal_i, int cal_q)
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

	at86rf215_usb_reg_write_8(val_i, reg_i);
	at86rf215_usb_reg_write_8(val_q, reg_q);

    // RFn_TXCI – Transmit calibration I path
    //  The register contains information about the TX LO leakage calibration value of the transmit I path. At the transition
    //  process from state TRXOFF to TX the calibration is started automatically

    // RFn_TXCQ – Transmit calibration Q path
    //  The register contains information about the TX LO leakage calibration value of the transmit Q path. At the transition
    //  process from state TRXOFF to TX the calibration is started automatically.
}

//==================================================================================
void at86rf215_usb_radio_get_tx_iq_calibration(const struct at86rf215 *h, at86rf215_radio_t radio,
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

	at86rf215_usb_reg_read_8(&val_i, reg_i);
	at86rf215_usb_reg_read_8(&val_q, reg_q);

	*cal_i = val_i & 0x3F;
    *cal_q = val_q & 0x3F;
}

int at86rf215_usb_set_mode(struct at86rf215 *h, at86rf215_chpm_t mode)
{
    int ret = 0;
	uint8_t val = 0;
	ret = at86rf215_usb_reg_read_8(&val, REG_RF_IQIFC1);
	if (ret) {
		return ret;
	}
	val = (val & 0x3) | (mode << 4);
	ret = at86rf215_usb_reg_write_8(val, REG_RF_IQIFC1);
	if (ret) {
		return ret;
	}
	h->priv.chpm = mode;
	return AT86RF215_OK;
}

int at86rf215_usb_radio_conf(struct at86rf215 *h, at86rf215_radio_t radio,
                    	const struct at86rf215_radio_conf *conf)
{
	if (!conf) {
		return -AT86RF215_INVAL_PARAM;
	}
	int ret = 0;
	uint32_t spacing = conf->cs / 25000;
	spacing = min(0xFF, spacing);

	/* Set the channel configuration */
	if (radio == AT86RF215_RF09) {
		switch (conf->cm) {
			case AT86RF215_CM_IEEE: {
				ret = at86rf215_usb_reg_write_8(spacing, REG_RF09_CS);
				if (ret) {
					return ret;
				}
				h->priv.radios[AT86RF215_RF09].cs_reg = spacing;
				h->priv.radios[AT86RF215_RF09].base_freq = conf->base_freq;
				ret = at86rf215_usb_reg_write_8(spacing, REG_RF09_CS);
				if (ret) {
					return ret;
				}

				/* Apply base frequency */
				if (conf->base_freq < 389500000 || conf->base_freq > 1088000000) {
					return -AT86RF215_INVAL_PARAM;
				}
				uint16_t base = conf->base_freq / 25000;
				ret = at86rf215_usb_reg_write_8(base & 0xFF, REG_RF09_CCF0L);
				if (ret) {
					return ret;
				}
				ret = at86rf215_usb_reg_write_8(base >> 8, REG_RF09_CCF0H);
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
				ret = at86rf215_usb_reg_write_8(spacing, REG_RF24_CS);
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
				ret = at86rf215_usb_reg_write_8(base & 0xFF, REG_RF24_CCF0L);
				if (ret) {
					return ret;
				}
				ret = at86rf215_usb_reg_write_8(base >> 8, REG_RF24_CCF0H);
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
		ret = at86rf215_usb_reg_write_8(conf->lbw, REG_RF09_PLL);
		if (ret) {
			return ret;
		}
	}
	h->priv.radios[radio].op_state = AT86RF215_OP_STATE_IDLE;
	h->priv.radios[radio].init = INIT_MAGIC_VAL;
	return AT86RF215_OK;
}

int at86rf215_usb_get_irq_mask(const struct at86rf215 *h, uint8_t *mask, at86rf215_radio_t radio)
{
	int ret = 0;
	uint16_t reg = 0;
	if (radio == AT86RF215_RF09) {
		reg = REG_RF09_IRQM;
	} else {
		reg = REG_RF24_IRQM;
	}
	uint8_t val = 0;
	ret = at86rf215_usb_reg_read_8(&val, reg);
	if (ret) {
		return ret;
	}
	*mask = val;
	return AT86RF215_OK;
}

int at86rf215_usb_set_channel(struct at86rf215 *h, at86rf215_radio_t radio, uint16_t channel)
{
	int ret = 0;
	if (channel > 0x1F) {
		return -AT86RF215_INVAL_PARAM;
	}
	/* The radio should be in IEEE channel mode to set a channel */
	if (h->priv.radios[radio].cm != AT86RF215_CM_IEEE) {
		return -AT86RF215_INVAL_CONF;
	}
	if (radio == AT86RF215_RF09) {
		ret = at86rf215_usb_reg_write_8(channel & 0xFF, REG_RF09_CNL);
		if (ret) {
			return ret;
		}
		ret = at86rf215_usb_reg_write_8((h->priv.radios[radio].cm << 6) |
		                            ((channel >> 8) & 0x1), REG_RF09_CNM);
		if (ret) {
			return ret;
		}
	} else {
		ret = at86rf215_usb_reg_write_8(channel & 0xFF, REG_RF24_CNL);
		if (ret) {
			return ret;
		}
		ret = at86rf215_usb_reg_write_8((h->priv.radios[radio].cm << 6) |
		                            ((channel >> 8) & 0x1), REG_RF24_CNM);
		if (ret) {
			return ret;
		}
	}
	return AT86RF215_OK;
}

static bool is_usb_lpf_valid(at86rf215_lpfcut_t lpf)
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

static bool is_usb_paramp_valid(at86rf215_paramp_t paramp)
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

int at86rf215_usb_set_txcutc(struct at86rf215 *h, at86rf215_radio_t radio, at86rf215_paramp_t paramp, at86rf215_lpfcut_t lpf)
{
	int ret = 0;
	if (!is_usb_lpf_valid(lpf) || !is_usb_paramp_valid(paramp)) {
		return -AT86RF215_INVAL_PARAM;
	}
	uint16_t reg = 0x0;
	if (radio == AT86RF215_RF09) {
		reg = REG_RF09_TXCUTC;
	} else {
		reg = REG_RF24_TXCUTC;
	}
	return at86rf215_usb_reg_write_8(paramp << 6 | lpf, reg);
}

static int set_usb_txdfe(at86rf215_radio_t radio, uint8_t rcut, uint8_t dm, at86rf215_sr_t sr)
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
	return at86rf215_usb_reg_write_8(val, reg);
}

int at86rf215_usb_set_txdfe(struct at86rf215 *h, at86rf215_radio_t radio, uint8_t rcut, uint8_t dm, at86rf215_sr_t sr)
{
	return set_usb_txdfe(radio, rcut, dm, sr);
}

static int set_usb_rxdfe(at86rf215_radio_t radio, uint8_t rcut, at86rf215_sr_t sr)
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
	return at86rf215_usb_reg_write_8(val, reg);
}

int at86rf215_usb_set_rxdfe(struct at86rf215 *h, at86rf215_radio_t radio, uint8_t rcut, at86rf215_sr_t sr)
{
	return set_usb_rxdfe(radio, rcut, sr);
}

int at86rf215_usb_set_bw(struct at86rf215 *h, at86rf215_radio_t radio, uint8_t if_inv, uint8_t if_shift, at86rf215_rx_bw_t bw)
{

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
	return at86rf215_usb_reg_write_8(val, reg);
}

int
at86rf215_usb_set_pac(struct at86rf215 *h, at86rf215_radio_t radio, at86rf215_pacur_t pacur, uint8_t power)
{
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
	return at86rf215_usb_reg_write_8((pacur << 5) | power, reg);
}

int at86rf215_usb_set_edd(struct at86rf215 *h, at86rf215_radio_t radio, uint8_t df, at86rf215_edd_dtb_t dtb)
{
	uint16_t reg = 0;
	if (radio == AT86RF215_RF09) {
		reg = REG_RF09_EDD;
	} else {
		reg = REG_RF24_EDD;
	}
	uint8_t val = (df << 2) | dtb;
	return at86rf215_usb_reg_write_8(val, reg);
}

static int bb_usb_conf_mrfsk(struct at86rf215 *h, at86rf215_radio_t radio, const struct at86rf215_bb_conf *conf)
{
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
	int ret = at86rf215_usb_reg_write_8(val, REG_BBC0_FSKC0 + offset);
	if (ret) {
		return ret;
	}

	/* FSKC1 */
	val = conf->fsk.srate | (conf->fsk.fi << 5)
	      | ((conf->fsk.preamble_length >> 2) & 0xC0);
	ret = at86rf215_usb_reg_write_8(val, REG_BBC0_FSKC1 + offset);
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
	ret = at86rf215_usb_reg_write_8(val, REG_BBC0_FSKC2 + offset);
	if (ret) {
		return ret;
	}

	/* FSKC3 */
	val = (conf->fsk.sfd_threshold << 4) | conf->fsk.preamble_threshold;
	ret = at86rf215_usb_reg_write_8(val, REG_BBC0_FSKC3 + offset);
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
	ret = at86rf215_usb_reg_write_8(val, REG_BBC0_FSKC4 + offset);
	if (ret) {
		return ret;
	}

	/* FSKPLL */
	val = conf->fsk.preamble_length;
	ret = at86rf215_usb_reg_write_8(val, REG_BBC0_FSKPLL + offset);
	if (ret) {
		return ret;
	}

	/* SFD configuration */
	ret = at86rf215_usb_reg_write_8(conf->fsk.sfd0, REG_BBC0_FSKSFD0L + offset);
	if (ret) {
		return ret;
	}
	ret = at86rf215_usb_reg_write_8(conf->fsk.sfd0 >> 8,
	                            REG_BBC0_FSKSFD0H + offset);
	if (ret) {
		return ret;
	}
	ret = at86rf215_usb_reg_write_8(conf->fsk.sfd1, REG_BBC0_FSKSFD1L + offset);
	if (ret) {
		return ret;
	}
	ret = at86rf215_usb_reg_write_8(conf->fsk.sfd1 >> 8,
	                            REG_BBC0_FSKSFD1H + offset);
	if (ret) {
		return ret;
	}

	/* FSKPHRTX */
	val = conf->fsk.rb1 | (conf->fsk.rb2 << 1) | (conf->fsk.dw << 2)
	      | (conf->fsk.sfd << 3);
	ret = at86rf215_usb_reg_write_8(val, REG_BBC0_FSKPLL + offset);
	if (ret) {
		return ret;
	}

	/* FSKDM */
	ret = at86rf215_usb_reg_write_8(conf->fsk.dm | (conf->fsk.preemphasis << 1),
	                            REG_BBC0_FSKDM + offset);
	if (ret) {
		return ret;
	}

	/* Both TXDFE and FSK DM should have the direct modulation option enabled*/
	txdfe |= conf->fsk.dm << 4;
	ret = at86rf215_usb_reg_read_8(&val, REG_RF09_TXDFE + offset);
	if (ret) {
		return ret;
	}
	txdfe |= (val & 0xE0);
	ret = at86rf215_usb_reg_write_8(txdfe, REG_RF09_TXDFE + offset);
	if (ret) {
		return ret;
	}

	/* PRemphasis filter setup */
	ret = at86rf215_usb_reg_write_8(conf->fsk.preemphasis_taps,
	                            REG_BBC0_FSKPE0 + offset);
	if (ret) {
		return ret;
	}

	ret = at86rf215_usb_reg_write_8(conf->fsk.preemphasis_taps >> 8,
	                            REG_BBC0_FSKPE1 + offset);
	if (ret) {
		return ret;
	}

	ret = at86rf215_usb_reg_write_8(conf->fsk.preemphasis_taps >> 16,
	                            REG_BBC0_FSKPE2 + offset);
	if (ret) {
		return ret;
	}

	/* Apply RX sampling rate */
	ret = at86rf215_usb_reg_read_8(&val, REG_RF09_RXDFE + offset);
	if (ret) {
		return ret;
	}
	rxdfe |= (val & 0xE0);
	ret = at86rf215_usb_reg_write_8(rxdfe, REG_RF09_RXDFE + offset);
	if (ret) {
		return ret;
	}

	return AT86RF215_OK;
}

static int bb_usb_conf_mrofdm(struct at86rf215 *h, at86rf215_radio_t radio, const struct at86rf215_bb_conf *conf)
{
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
	int ret = at86rf215_usb_reg_write_8(val, REG_BBC0_OFDMPHRTX + offset);
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
	ret = at86rf215_usb_reg_write_8(val, REG_BBC0_OFDMC + offset);
	if (ret) {
		return ret;
	}

	val = conf->ofdm.pdt << 5;
	val |= conf->ofdm.rxo << 4;
	ret = at86rf215_usb_reg_write_8(val, REG_BBC0_OFDMSW + offset);
	if (ret) {
		return ret;
	}

	return AT86RF215_OK;
}

int at86rf215_usb_bb_conf(struct at86rf215 *h, at86rf215_radio_t radio, const struct at86rf215_bb_conf *conf)
{
	int ret = 0;
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
	ret = at86rf215_usb_reg_write_8(val, reg);
	if (ret) {
		return ret;
	}

	switch (conf->pt) {
		case AT86RF215_BB_MRFSK:
			ret = bb_usb_conf_mrfsk(h, radio, conf);
			break;
		case AT86RF215_BB_MROFDM:
			ret = bb_usb_conf_mrofdm(h, radio, conf);
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

int at86rf215_usb_bb_enable(struct at86rf215 *h, at86rf215_radio_t radio, uint8_t en)
{
	int ret =0;
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
	ret = at86rf215_usb_reg_write_8(val, reg);
	if (ret) {
		return ret;
	}
	return AT86RF215_OK;
}

static int read_usb_tx_fifo(const struct at86rf215 *h, at86rf215_radio_t radio, uint8_t *b)
{
	int ret = 0;
    uint8_t lsb_len = 0;
    uint8_t msb_len = 0;
    uint16_t len = 0;

	if (radio == AT86RF215_RF09) {
		/* Declare the size of the PSDU */
		ret = at86rf215_usb_reg_read_8(&lsb_len, REG_BBC0_RXFLL);
		if (ret) {
			return ret;
		}
		ret = at86rf215_usb_reg_read_8(&msb_len, REG_BBC0_RXFLH);
		if (ret) {
			return ret;
		}
        len |= msb_len;
        len = (len << 8) | lsb_len;
		printf("length = %d\n", len);

		if (len > 0)
		{
			/* Fill the FIFO */
			uint8_t mosi[len+2];
			memset(mosi, 0, sizeof(mosi));
			mosi[0] = (REG_BBC0_FBRXS >> 8) & 0x3F;
			mosi[1] = REG_BBC0_FBRXS & 0xFF;
			uint8_t miso[len+2];
			memset(miso, 0, sizeof(miso));
			ret = at86rf215_usb_read(&miso[0], &mosi[0], len+2);
			if (ret) {
				return ret;
			}
			for (int i=0; i<len+2; i++)
			{
				printf("Received data, index = %d, data = %02X\n", i, miso[i]);
			}
		}
	} else {
		/* Declare the size of the PSDU */
		ret = at86rf215_usb_reg_read_8(&lsb_len, REG_BBC1_RXFLL);
		if (ret) {
			return ret;
		}
		ret = at86rf215_usb_reg_read_8(&msb_len, REG_BBC1_RXFLH);
		if (ret) {
			return ret;
		}
        len |= msb_len;
        len = (len << 8) | lsb_len;

		/* Fill the FIFO */
		uint8_t mosi[2] = {(REG_BBC1_FBRXS >> 8),
		                   REG_BBC1_FBRXS & 0xFF
		                  };
		ret = at86rf215_usb_read(b, &mosi[0], len);
		if (ret) {
			return ret;
		}
        for (int i=0; i<len; i++)
        {
            printf("Received data, index = %d, data = %d\n", i, *(b++));
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
int at86rf215_usb_rx_frame(struct at86rf215 *h, at86rf215_radio_t radio, uint8_t *psdu)
{
    int ret;
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

	h->priv.radios[radio].op_state = AT86RF215_OP_STATE_RX;
	ret = read_usb_tx_fifo(h, radio, psdu);
	if (ret) {
		return ret;
	}

	// TODO: Remove after verify
	at86rf215_rf_state_t radio_state;
	at86rf215_usb_get_state(h, &radio_state, radio);
	printf("radio state = %d\n", radio_state);

	printf("Receiving RX CMD.\n");
	/* Data are on the FIFO. Issue the TX cmd to send them */
	return AT86RF215_OK;
}

int at86rf215_init_usb(struct at86rf215 *h)
{
	if (!h) {
		return -AT86RF215_INVAL_PARAM;
	}

    testFT4222();

	/* Reset the state of the private struct members */
	memset(&h->priv, 0, sizeof(struct at86rf215_priv));
	
	at86rf215_usb_reg_write_8(AT86RF215_CMD_RF_RESET, REG_RF_RST);
	usleep(10000000);

	uint8_t val = 0;
	int ret = at86rf215_usb_reg_read_8(&val, REG_RF_PN);
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

	ret = at86rf215_usb_reg_write_8(val, REG_RF_CLKO);
	if (ret) {
		return ret;
	}

	/* Apply XO settings */
	ret = at86rf215_usb_reg_write_8((h->xo_fs << 4) | h->xo_trim, REG_RF_XOC);
	if (ret) {
		return ret;
	}

	/* Set the RF_CFG */
	ret = at86rf215_usb_reg_write_8((h->irqmm << 3) | (h->irqp << 2) | h->pad_drv, REG_RF_CFG);
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
	ret = at86rf215_usb_reg_write_8(val, REG_RF09_PADFE);
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
	ret = at86rf215_usb_reg_write_8(val, REG_RF24_PADFE);
	if (ret) {
		return ret;
	}

	/* Get the version of the IC */
	ret = at86rf215_usb_reg_read_8(&val, REG_RF_VN);
	if (ret) {
		return ret;
	}
	h->priv.version = val;
	h->priv.chpm = AT86RF215_RF_MODE_BBRF;
	h->priv.init = INIT_MAGIC_VAL;

	/* Disable all IRQ sources */
	at86rf215_usb_set_bbc_irq_mask(h, AT86RF215_RF09, 0x0);
	at86rf215_usb_set_bbc_irq_mask(h, AT86RF215_RF24, 0x0);
	at86rf215_usb_set_radio_irq_mask(h, AT86RF215_RF09, 0x0);
	at86rf215_usb_set_radio_irq_mask(h, AT86RF215_RF24, 0x0);


	event_node_init(&h->priv.radios[AT86RF215_RF09].trxready);
	event_node_init(&h->priv.radios[AT86RF215_RF24].trxready);

	at86rf215_usb_calibrate_device(h, AT86RF215_RF09, &h->cal.low_ch_i, &h->cal.low_ch_q);
    at86rf215_usb_calibrate_device(h, AT86RF215_RF24, &h->cal.hi_ch_i, &h->cal.hi_ch_q);

	return AT86RF215_OK;
}

void close_usb()
{
    (void)FT4222_UnInitialize(ftHandle);
    (void)FT_Close(ftHandle);
    free(devInfo);
}