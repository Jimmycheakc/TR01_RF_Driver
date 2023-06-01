#ifndef __AT86RF215_BB_USB_H__
#define __AT86RF215_BB_USB_H__

#include "at86rf215_bb.h"

#ifdef __cplusplus
extern "C" {
#endif


int at86rf215_usb_read(uint8_t *out, uint8_t *in, size_t len);

int at86rf215_usb_write(uint8_t *in, size_t len);

int at86rf215_usb_reg_read_8(uint8_t *out, uint16_t reg);

int at86rf215_usb_reg_write_8(const uint8_t in, uint16_t reg);

int at86rf215_usb_set_bbc_irq_mask(struct at86rf215 *h, at86rf215_radio_t radio, uint8_t mask);

int at86rf215_usb_set_radio_irq_mask(struct at86rf215 *h, at86rf215_radio_t radio, uint8_t mask);

int at86rf215_usb_get_state(const struct at86rf215 *h, at86rf215_rf_state_t *state, at86rf215_radio_t radio);

int at86rf215_usb_set_cmd(struct at86rf215 *h, at86rf215_rf_cmd_t cmd, at86rf215_radio_t radio);

int at86rf215_usb_calibrate_device(struct at86rf215 *h, at86rf215_radio_t radio, int* i, int* q);

void at86rf215_usb_radio_set_tx_iq_calibration(const struct at86rf215 *h, at86rf215_radio_t radio, int cal_i, int cal_q);

void at86rf215_usb_radio_get_tx_iq_calibration(const struct at86rf215 *h, at86rf215_radio_t radio,
                                                int *cal_i, int *cal_q);

int at86rf215_usb_set_mode(struct at86rf215 *h, at86rf215_chpm_t mode);

int at86rf215_usb_radio_conf(struct at86rf215 *h, at86rf215_radio_t radio,
                    	const struct at86rf215_radio_conf *conf);

int at86rf215_usb_get_irq_mask(const struct at86rf215 *h, uint8_t *mask, at86rf215_radio_t radio);

int at86rf215_usb_set_channel(struct at86rf215 *h, at86rf215_radio_t radio, uint16_t channel);

int at86rf215_usb_set_txcutc(struct at86rf215 *h, at86rf215_radio_t radio, at86rf215_paramp_t paramp, at86rf215_lpfcut_t lpf);

int at86rf215_usb_set_txdfe(struct at86rf215 *h, at86rf215_radio_t radio, uint8_t rcut, uint8_t dm, at86rf215_sr_t sr);

int at86rf215_usb_set_rxdfe(struct at86rf215 *h, at86rf215_radio_t radio, uint8_t rcut, at86rf215_sr_t sr);

int at86rf215_usb_set_bw(struct at86rf215 *h, at86rf215_radio_t radio, uint8_t if_inv, uint8_t if_shift, at86rf215_rx_bw_t bw);

int
at86rf215_usb_set_pac(struct at86rf215 *h, at86rf215_radio_t radio, at86rf215_pacur_t pacur, uint8_t power);

int at86rf215_usb_set_edd(struct at86rf215 *h, at86rf215_radio_t radio, uint8_t df, at86rf215_edd_dtb_t dtb);

int at86rf215_usb_bb_conf(struct at86rf215 *h, at86rf215_radio_t radio, const struct at86rf215_bb_conf *conf);

int at86rf215_usb_bb_enable(struct at86rf215 *h, at86rf215_radio_t radio, uint8_t en);

int at86rf215_usb_irq_callback(int event, unsigned int line_offset, const struct timespec * time, void *data);

int at86rf215_usb_tx_frame(struct at86rf215 *h, at86rf215_radio_t radio, const uint8_t *psdu, size_t len);

int at86rf215_usb_rx_frame(struct at86rf215 *h, at86rf215_radio_t radio, uint8_t *psdu);

int at86rf215_init_usb(struct at86rf215 *h);

void close_usb();

#ifdef __cplusplus
}
#endif

#endif /* __AT86RF215_BB_USB_H__ */
