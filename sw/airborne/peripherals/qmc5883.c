/*
 * Copyright (C) 2020 Paparazzi Team
 *
 * This file is part of paparazzi.
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

/**
 * @file peripherals/qmc5883.c
 *
 * Driver for QMC5883 magnetometer.
 * @todo DRDY/IRQ handling
 */

#include "peripherals/qmc5883.h"
#include "mcu_periph/sys_time.h"
#include "std.h"

/* QMC5883 default conf */

// When all correctly shifted outcome should be 0x1D
// only reason for lower ODR and OSR is for a few mA of power saving, not really relevant for UAS IMU to be honest
#ifndef QMC5883_DEFAULT_OSR 
#define QMC5883_DEFAULT_OSR  0x00 ///< 0b00000000 512 Over Sample Ratio
#endif
#ifndef QMC5883_DEFAULT_RNG
#define QMC5883_DEFAULT_RNG  0x10 ///< 0b00010000 8 Gauss Full Scale Range
#endif
#ifndef QMC5883_DEFAULT_ODR
#define QMC5883_DEFAULT_ODR  0x0C ///< 0b00001100 200Hz Output Data Rate, can be 10,50,100 or 200Hz 
#endif
#ifndef QMC5883_DEFAULT_MODE
#define QMC5883_DEFAULT_MODE 0x01 ///< 0b00000001 Continuous Measurement mode
#endif

/** QMC5883 startup delay
 *
 *  On startup, the qmc is making a first conversion in single mode.
 *  Trying to configure the mode register before the end of this conversion
 *  seems to void the configuration.
 *  Default conversion rate is 15 Hz (66ms) and worst case is O.75Hz (1.3s).
 *  Therefore we set the default delay to 1.5s afer boot time.
 */
#ifndef QMC5883_STARTUP_DELAY
#define QMC5883_STARTUP_DELAY 1.5
#endif

static void qmc5883_set_default_config(struct Qmc5883Config *c)
{
  c->osr  = QMC5883_DEFAULT_OSR;
  c->rng  = QMC5883_DEFAULT_RNG;
  c->odr  = QMC5883_DEFAULT_ODR;
  c->mode = QMC5883_DEFAULT_MODE;
}

/**
 * Initialize Qmc5883 struct and set default config options.
 * @param qmc   Qmc5883 struct
 * @param i2c_p I2C periperal to use
 * @param addr  I2C address of QMC5883
 */
void qmc5883_init(struct Qmc5883 *qmc, struct i2c_periph *i2c_p, uint8_t addr)
{
  qmc->initialized = false;
  qmc->adc_overflow_cnt = 0;
  qmc->type = QMC_TYPE_DB5883; //DA type not implemented since industry sample should not be in the wild on devices
  qmc->init_status = QMC_CONF_UNINIT;
  qmc->i2c_p = i2c_p;// set i2c_peripheral
  qmc->i2c_trans.slave_addr = addr;  // set i2c address
  qmc->i2c_trans.status = I2CTransDone;
  qmc5883_set_default_config(&(qmc->config)); //set default config options
}

static void qmc5883_i2c_tx_reg(struct Qmc5883 *qmc, uint8_t reg, uint8_t val)
{
  qmc->i2c_trans.type = I2CTransTx;
  qmc->i2c_trans.buf[0] = reg;
  qmc->i2c_trans.buf[1] = val;
  qmc->i2c_trans.len_r = 0;
  qmc->i2c_trans.len_w = 2;
  i2c_submit(qmc->i2c_p, &(qmc->i2c_trans));
}

/// Configuration function called once before normal use
static void qmc5883_send_config(struct Qmc5883 *qmc)
{
  switch (qmc->init_status) {
    case QMC_CONF_1:
      qmc5883_i2c_tx_reg(qmc, QMC5883_REG_CONTROL_1, 0x0); //No Soft reset, Disable Pointer roll-over, Enable interrupt PIN
      qmc->init_status++;
      break;
    case QMC_CONF_2:
      qmc5883_i2c_tx_reg(qmc, QMC5883_REG_CONTROL_2, (qmc->config.osr | qmc->config.rng | qmc->config.odr | qmc->config.mode));
      qmc->init_status++;
      break;
    case QMC_CONF_DONE:
      qmc->initialized = true;
      qmc->i2c_trans.status = I2CTransDone;
      break;
    default:
      break;
  }
}

// Configure
void qmc5883_start_configure(struct Qmc5883 *qmc)
{
  // wait before starting the configuration
  // doing to early may void the mode configuration
  if (qmc->init_status == QMC_CONF_UNINIT && get_sys_time_float() > QMC5883_STARTUP_DELAY) {
    qmc5883_i2c_tx_reg(qmc, QMC5883_REG_RESET_PERIOD, 0x01); //Set Reset period
    qmc->init_status++;
    if (qmc->i2c_trans.status == I2CTransSuccess || qmc->i2c_trans.status == I2CTransDone) {
      qmc5883_send_config(qmc);
    }
  }
}

// Normal reading
void qmc5883_read(struct Qmc5883 *qmc)
{
  if (qmc->initialized && qmc->i2c_trans.status == I2CTransDone) {
    qmc->i2c_trans.buf[0] = QMC5883_REG_DATXM;
    qmc->i2c_trans.type = I2CTransTxRx;
    qmc->i2c_trans.len_r = 6;
    qmc->i2c_trans.len_w = 1;
    i2c_submit(qmc->i2c_p, &(qmc->i2c_trans));
  }
}

#define Int16FromBuf(_buf,_idx) ((int16_t)((_buf[_idx]<<8) | _buf[_idx+1]))

void qmc5883_event(struct Qmc5883 *qmc)
{
  if (qmc->initialized) {
    if (qmc->i2c_trans.status == I2CTransFailed) {
      qmc->i2c_trans.status = I2CTransDone;
    } else if (qmc->i2c_trans.status == I2CTransSuccess) {
      if (qmc->type == QMC_TYPE_DB5883) {
        qmc->data.vect.x = Int16FromBuf(qmc->i2c_trans.buf, 0);
        qmc->data.vect.y = Int16FromBuf(qmc->i2c_trans.buf, 2);
        qmc->data.vect.z = Int16FromBuf(qmc->i2c_trans.buf, 4);
      }
      /* QMC_DA_5883 has xzy order of axes in returned data */
      else {
        qmc->data.vect.x = Int16FromBuf(qmc->i2c_trans.buf, 0);
        qmc->data.vect.y = Int16FromBuf(qmc->i2c_trans.buf, 4);
        qmc->data.vect.z = Int16FromBuf(qmc->i2c_trans.buf, 2);
      }
      /* only set available if measurements valid: -4096 if ADC under/overflow in sensor */
      if (qmc->data.vect.x != -4096 && qmc->data.vect.y != -4096 &&
          qmc->data.vect.z != -4096) {
        qmc->data_available = true;
      }
      else {
        qmc->adc_overflow_cnt++;
      }
      qmc->i2c_trans.status = I2CTransDone;
    }
  } else if (qmc->init_status != QMC_CONF_UNINIT) { // Configuring but not yet initialized
    if (qmc->i2c_trans.status == I2CTransSuccess || qmc->i2c_trans.status == I2CTransDone) {
      qmc->i2c_trans.status = I2CTransDone;
      qmc5883_send_config(qmc);
    }
    if (qmc->i2c_trans.status == I2CTransFailed) {
      qmc->init_status--;
      qmc->i2c_trans.status = I2CTransDone;
      qmc5883_send_config(qmc); // Retry config (TODO max retry)
    }
  }
}

