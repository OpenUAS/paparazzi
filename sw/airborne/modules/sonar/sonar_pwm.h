/*
* Copyright (C) 2020 OpenuAS
*
* Thx Jean-François Erdelyi & Gautier Hattenberger
*
* This file is part of paparazzi
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
* along with paparazzi; see the file COPYING.  If not, see
* <http://www.gnu.org/licenses/>.
*/

/**
* @file "modules/sensors/sonar_pwm.h"
* @author OpenUAS
* @brief Range sensor input via PWM
*
* Driver for a PWM based sonar range sensor
* Reads sensor using PWM input and outputs sonar distance to object
*
* Sensor example: Maxbotix EZ1
*
*/

#ifndef SONAR_PWM_H
#define SONAR_PWM_H

#include "std.h"

struct SonarPwm {
  uint16_t raw;   ///< raw PWM value
  float offset;   ///< offset in M
  float scale;    ///< scale to convert raw to real distance
  float distance; ///< Distance measured in meters
};

extern struct SonarPwm sonar_pwm; // Range sensor

extern void sonar_pwm_init(void);
extern void sonar_pwm_read(void);

#endif /* SONAR_PWM_H */

