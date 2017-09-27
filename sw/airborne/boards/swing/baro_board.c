/*
 * Copyright (C) 2017 Paparazzi team
 *
 * This file is part of Paparazzi.
 *
 * Paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * Paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

/**
 * @file boards/swing/baro_board.c
 * Paparazzi implementation for Parrot Minidrones Baro Sensor :.
 */


#include "baro_board.h"
#include "subsystems/sensors/baro.h"
#include "subsystems/abi.h"
#include "led.h"

/** Option to use an extra median filter to smoothen barometric data
 */
#if USE_BARO_MEDIAN_FILTER
#include "filters/median_filter.h"
struct MedianFilterInt baro_median;
#endif


void baro_init(void)
{
  #if USE_BARO_MEDIAN_FILTER
    init_median_filter_i(&baro_median, MEDIAN_DEFAULT_SIZE);
  #endif

  #ifdef BARO_LED
    LED_OFF(BARO_LED);
  #endif
}

void baro_periodic(void) {}

/**
 * Apply temperature and sensor calibration to get pressure in Pa.
 * @param raw uncompensated raw pressure reading
 * @return compensated pressure in Pascal
 */
static inline int32_t baro_apply_calibration(int32_t raw)
{
  int32_t press = raw; //FIXME
  // Zero at sealevel
  return press;
}

/**
 * Apply temperature calibration.
 * @param tmp_raw uncompensated raw temperature reading
 * @return compensated temperature in 0.1 deg Celcius
 */
static inline int32_t baro_apply_calibration_temp(int32_t tmp_raw)
{
  return 0; //FIXME
}

void baro_event(void)
{
 /* if (minidrone.baro_available) {
    if (minidrone.baro_calibrated) {
      // first read temperature because pressure calibration depends on temperature
      float temp_deg = 0.1 * baro_apply_calibration_temp(navdata.measure.temperature_pressure);
      AbiSendMsgTEMPERATURE(BARO_BOARD_SENDER_ID, temp_deg);
      int32_t press_pascal = baro_apply_calibration(minidrone.measure.pressure);
#if USE_BARO_MEDIAN_FILTER
      press_pascal = update_median_filter_i(&baro_median, press_pascal);
#endif
      float pressure = (float)press_pascal;
      AbiSendMsgBARO_ABS(BARO_BOARD_SENDER_ID, pressure);
    }
    minidrone.baro_available = false;
  }
  */
#ifdef BARO_LED
    RunOnceEvery(10, LED_TOGGLE(BARO_LED));
#endif
}
