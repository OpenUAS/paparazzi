/*
 * Copyright (C) 2015 Freek van Tienen <freek.v.tienen@gmail.com>
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
 *
 */

/** @file subsystems/intermcu/intermcu_ap.h
 *  @brief Rotorcraft Inter-MCU on the autopilot
 */

#ifndef INTERMCU_AP_ROTORCRAFT_H
#define INTERMCU_AP_ROTORCRAFT_H

#include "generated/airframe.h"
#include "subsystems/intermcu.h"
#include "subsystems/electrical.h"

void intermcu_set_actuators(pprz_t *command_values, uint8_t ap_mode);
void RadioControlEvent(void (*frame_handler)(void));
void intermcu_send_spektrum_bind(void);
void intermcu_set_enabled(bool value);

/* We need radio defines for the Autopilot */
/* Default channel assignments */
#ifndef RADIO_THROTTLE
#define RADIO_THROTTLE   0
#endif
#ifndef RADIO_ROLL
#define RADIO_ROLL       1
#endif
#ifndef RADIO_PITCH
#define RADIO_PITCH      2
#endif
#ifndef RADIO_YAW
#define RADIO_YAW        3
#endif
#ifndef RADIO_GEAR
#define RADIO_GEAR       4
#endif
#ifndef RADIO_FLAP
#define RADIO_FLAP       5
#endif
#ifndef RADIO_AUX1
#define RADIO_AUX1       5
#endif
#ifndef RADIO_AUX2
#define RADIO_AUX2       6
#endif
#ifndef RADIO_AUX3
#define RADIO_AUX3       7
#endif

#ifndef RADIO_CONTROL_NB_CHANNEL
#define RADIO_CONTROL_NB_CHANNEL 8
#endif

#if RADIO_CONTROL_NB_CHANNEL > 8
#error "RADIO_CONTROL_NB_CHANNEL should not be higher than 8, else bandwith of could be IMCU saturated"
#endif

/* Structure for FBW status */
struct fbw_status_t {
  uint8_t rc_status;
  uint8_t frame_rate;
  uint8_t mode;
  struct Electrical electrical;
};

#endif /* INTERMCU_AP_ROTORCRAFT_H */
