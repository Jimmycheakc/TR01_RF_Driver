/*
 * This file was derived from code written by HackRF Team:
 * 	1. 2012 Michael Ossmann
 * 	2. 2014 Jared Boone <jared@sharebrained.com>
 * and was modified / rewritten by David Michaeli (cariboulabs.co@gmail.com) to
 * adapt if for the CaribouLite project running on Linux OS (RPI).
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; see the file COPYING.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street,
 * Boston, MA 02110-1301, USA.
 */

#ifndef __RFFC507X_H
#define __RFFC507X_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include <stdint.h>
#include "../io_utils/io_utils.h"
#include "../i2c/i2c.h"

#define I2C_SPI_RFFC_ADDRESS         0x2A

/* 31 registers, each containing 16 bits of data. */
#define RFFC507X_NUM_REGS 31
#define RFFC507X_REG_SET_CLEAN(d,r) (d)->rffc507x_regs_dirty &= ~(1UL<<r)
#define RFFC507X_REG_SET_DIRTY(d,r) (d)->rffc507x_regs_dirty |= (1UL<<r)


#pragma pack(1)
typedef struct  // readsel = 0000
{
    uint8_t device_rev  : 3;    // LSB
    uint16_t device_id  : 13;   // MSB
} rffc507x_device_id_st;

typedef struct  // readsel = 0001
{
    uint8_t dummy : 1;                  // LSB
    uint8_t coarse_tune_cal_fail : 1;
    uint8_t kv_cal_value : 6;
    uint8_t coarse_tune_cal_value : 7;
    uint8_t pll_lock :1;                // MSB
} rffc507x_device_status_st;
#pragma pack()

typedef enum
{
    rffc507x_1_ss = 0,
    rffc507x_2_ss = 1,

    rffc507x_unknow = 0xFF,
} rffc507x_ss_t;

typedef enum
{
    rffc507x_1_ss_read_req = 0x00,
    rffc507x_1_ss_read     = 0x01,
    rffc507x_1_ss_write    = 0x02,
    rffc507x_2_ss_read_req = 0x04,
    rffc507x_2_ss_read     = 0x05,
    rffc507x_2_ss_write    = 0x06,

    rffc507x_ss_unknown    = 0xFF,
} rffc507x_cmd_t;

typedef struct
{
    int fd;
    uint8_t slave_addr;
    rffc507x_ss_t slave_select;
    rffc507x_cmd_t slave_select_read_req_cmd;
    rffc507x_cmd_t slave_select_read_cmd;
    rffc507x_cmd_t slave_select_write_cmd;

    double ref_freq_hz;

    int initialized;
    uint16_t rffc507x_regs[RFFC507X_NUM_REGS];
    uint32_t rffc507x_regs_dirty;
} rffc507x_st;

// Initialize chip
int rffc507x_init(rffc507x_st* dev, const char *device_name, uint8_t addr, rffc507x_ss_t slave_select);
int rffc507x_release(rffc507x_st* dev);

// Read a register via SPI. Save a copy to memory and return
// value. Discard any uncommited changes and mark CLEAN
uint16_t rffc507x_reg_read(rffc507x_st* dev, uint8_t r);

// Write value to register via SPI and save a copy to memory. Mark
// CLEAN
void rffc507x_reg_write(rffc507x_st* dev, uint8_t r, uint16_t v);

// Write all dirty registers via SPI from memory. Mark all clean. Some
// operations require registers to be written in a certain order. Use
// provided routines for those operations
int rffc507x_regs_commit(rffc507x_st* dev);

// Set frequency (MHz)
double rffc507x_set_frequency(rffc507x_st* dev, double lo_hz);

void rffc507x_enable(rffc507x_st* dev);
void rffc507x_disable(rffc507x_st* dev);
void rffc507x_set_gpo(rffc507x_st* dev, uint8_t gpo);
void rffc507x_setup_pin_functions(rffc507x_st* dev);
void rffc507x_readback(rffc507x_st* dev, uint16_t *readback_buff, int buf_len);
void rffc507x_readback_status(rffc507x_st* dev, rffc507x_device_id_st* dev_id,
                                                rffc507x_device_status_st* stat);

void rffc507x_print_dev_id(rffc507x_device_id_st* dev_id);
void rffc507x_print_stat(rffc507x_device_status_st* stat);
void rffc507x_calibrate(rffc507x_st* dev);
void rffc507x_relock(rffc507x_st* dev);
void rffc507x_output_lo(rffc507x_st* dev, int state);
int rffc507x_setup_reference_freq(rffc507x_st* dev, double ref_freq_hz);

#ifdef __cplusplus
}
#endif

#endif // __RFFC507X_H
