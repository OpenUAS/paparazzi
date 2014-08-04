/*
 * Copyright (C) 2014 Freek van Tienen <freek.v.tienen@gmail.com>
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

/** @file modules/loggers/file_logger.c
 *  @brief File logger for Linux based autopilots
 */

#include "file_logger.h"

#include <stdio.h> 
#include "subsystems/imu.h"
#include "firmwares/rotorcraft/stabilization.h"
#include "state.h"

/** Set the default File logger path to the USB drive */
#ifndef FILE_LOGGER_PATH
#define FILE_LOGGER_PATH "/data/video/usb/"
#endif

/** The file pointer */
static FILE* file_logger;

/** Start the file logger and open a new file */
void file_logger_start(void) {
  uint32_t counter = 0;
  char filename[512];

  // Check for available files
  sprintf(filename, "%s%05d.csv", FILE_LOGGER_PATH, counter);
  while((file_logger = fopen(filename, "r"))) {
    fclose(file_logger);

    counter++;
    sprintf(filename, "%s%05d.csv", FILE_LOGGER_PATH, counter);
  }

  file_logger = fopen(filename, "w");

  if(file_logger != NULL) {
    fprintf(
      file_logger, 
      "counter,gyro_unscaled_p,gyro_unscaled_q,gyro_unscaled_r,accel_unscaled_x,accel_unscaled_y,accel_unscaled_z,mag_unscaled_x,mag_unscaled_y,mag_unscaled_z,COMMAND_THRUST,COMMAND_ROLL,COMMAND_PITCH,COMMAND_YAW,qi,qx,qy,qz\n"
    );
  }
}

/** Stop the logger an nicely close the file */
void file_logger_stop(void) {
  fclose(file_logger);
}

/** Log the values to a csv file */
void file_logger_periodic(void)
{
  if (file_logger == NULL) {
    return;
  }
  static uint32_t counter;
//   struct Int32Quat* quat = stateGetNedToBodyQuat_i();

  fprintf(file_logger, "%d,%f,%f,%f,%f,%f,%d,%f,%f,%f,%f,%f,%d,%f,%f,%f,%f,%f,%d\n",
    counter,
    filtered_rate.r,
    filtered_rate_deriv.r,
    angular_accel_ref.r,
    indi_u.r,
    indi_du.r,
    imu.gyro_unscaled.r,
    filtered_rate.p,
    filtered_rate_deriv.p,
    angular_accel_ref.p,
    indi_u.p,
    indi_du.p,
    imu.gyro_unscaled.p,
    filtered_rate.q,
    filtered_rate_deriv.q,
    angular_accel_ref.q,
    indi_u.q,
    indi_du.q,
    imu.gyro_unscaled.q
  );
  counter++;
}
