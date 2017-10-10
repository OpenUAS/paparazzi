/*
 * Copyright (C) 2017 The Paparazzi Team
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
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

/**
 * @file boards/parrot_minidrone/board.c
 *
 * Parrot Minidrone board initialization functions.
 *
 */

/* NOTE: This define is used only for printing the baro type during compilation */
#ifndef BARO_BOARD
#define BARO_BOARD BARO_PARROT_MINIDRONE
#endif

#include <stdlib.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <pthread.h>
#include <linux/input.h>

#include "mcu.h"

#ifndef V4L2_BUF_FLAG_ERROR
#define V4L2_BUF_FLAG_ERROR	0x0040
#endif

// not used atm but thingy below #include <linux/videodev2.h>
#include "modules/computer_vision/lib/v4l/v4l2.h"
#include "peripherals/video_device.h"

#include "boards/parrot_minidrone.h"
#include "subsystems/electrical.h"
#include "subsystems/sensors/baro.h"
#include "subsystems/abi.h"

static struct {
	const char *name;
	unsigned int fourcc;
} pixel_formats[] = {
	{ "RGB332", V4L2_PIX_FMT_RGB332 },
	{ "RGB555", V4L2_PIX_FMT_RGB555 },
	{ "RGB565", V4L2_PIX_FMT_RGB565 },
	{ "RGB555X", V4L2_PIX_FMT_RGB555X },
	{ "RGB565X", V4L2_PIX_FMT_RGB565X },
	{ "BGR24", V4L2_PIX_FMT_BGR24 },
	{ "RGB24", V4L2_PIX_FMT_RGB24 },
	{ "BGR32", V4L2_PIX_FMT_BGR32 },
	{ "RGB32", V4L2_PIX_FMT_RGB32 },
	{ "Y8", V4L2_PIX_FMT_GREY },
	{ "Y10", V4L2_PIX_FMT_Y10 },
	{ "Y12", V4L2_PIX_FMT_Y12 },
	{ "Y16", V4L2_PIX_FMT_Y16 },
	{ "YUYV", V4L2_PIX_FMT_YUYV },
	{ "UYVY", V4L2_PIX_FMT_UYVY },
	{ "SBGGR8", V4L2_PIX_FMT_SBGGR8 },
	{ "SGBRG8", V4L2_PIX_FMT_SGBRG8 },
	{ "SGRBG8", V4L2_PIX_FMT_SGRBG8 },
	{ "SRGGB8", V4L2_PIX_FMT_SRGGB8 },
	{ "SGRBG10_DPCM8", V4L2_PIX_FMT_SGRBG10DPCM8 },
	{ "SBGGR10", V4L2_PIX_FMT_SBGGR10 },
	{ "SGBRG10", V4L2_PIX_FMT_SGBRG10 },
	{ "SGRBG10", V4L2_PIX_FMT_SGRBG10 },
	{ "SRGGB10", V4L2_PIX_FMT_SRGGB10 },
	{ "SBGGR12", V4L2_PIX_FMT_SBGGR12 },
	{ "SGBRG12", V4L2_PIX_FMT_SGBRG12 },
	{ "SGRBG12", V4L2_PIX_FMT_SGRBG12 },
	{ "SRGGB12", V4L2_PIX_FMT_SRGGB12 },
	{ "DV", V4L2_PIX_FMT_DV },
	{ "MJPEG", V4L2_PIX_FMT_MJPEG },
	{ "MPEG", V4L2_PIX_FMT_MPEG },
};

static const char *v4l2_format_name(unsigned int fourcc)
{
	static char name[5];
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(pixel_formats); ++i) {
		if (pixel_formats[i].fourcc == fourcc)
			return pixel_formats[i].name;
	}

	for (i = 0; i < 4; ++i) {
		name[i] = fourcc & 0xff;
		fourcc >>= 8;
	}

	name[4] = '\0';
	return name;
}

static unsigned int v4l2_format_code(const char *name)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(pixel_formats); ++i) {
		if (strcasecmp(pixel_formats[i].name, name) == 0)
			return pixel_formats[i].fourcc;
	}

	return 0;
}

//TODO use /dev/vertical_camera

//Bottom and front are Same for now
struct video_config_t front_camera = {
  .output_size = {
    .w = 320,
    .h = 240
  },
  .sensor_size = {
    .w = 640,
    .h = 480
  },
  .crop = {
    .x = 0,
    .y = 0,
    .w = 320,
    .h = 240
  },
  .dev_name = "/dev/video0",
  .subdev_name = NULL,
  .format = V4L2_PIX_FMT_YUYV,
  .buf_cnt = 2,
  .filters = 0,
  .cv_listener=NULL,
  .fps = 0
};

struct video_config_t bottom_camera = {
  .output_size = {
    .w = 320,
    .h = 240
  },
  .sensor_size = {
    .w = 640,
    .h = 480
  },
  .crop = {
    .x = 0,
    .y = 0,
    .w = 320,
    .h = 240
  },
  .dev_name = "/dev/video0",
  .subdev_name = NULL,
  .format = V4L2_PIX_FMT_YUYV,
  .buf_cnt = 2,
  .filters = 0,
  .cv_listener=NULL,
  .fps = 0
};


/**
 * Battery reading thread
 */
static void *bat_read(void *data __attribute__((unused)))
{
  FILE *fp;
  char path[16];

  while (TRUE) {
    /* Open the command for reading. */
    fp = popen("cat /sys/devices/platform/p6-spi.2/spi2.0/vbat", "r");
    if (fp == NULL) {
      printf("Failed to read battery\n" );
    }
    else {
      /* Read the output a line at a time - output it. */
      while (fgets(path, sizeof(path)-1, fp) != NULL) {
        int raw_bat = atoi(path);
        // convert to decivolt
        // from /bin/mcu_vbat.sh: MILLIVOLTS_VALUE=$(( ($RAW_VALUE * 4250) / 1023 ))
        electrical.vsupply = ((raw_bat * 4250) / 1023) / 100;
      }
      /* close */
      pclose(fp);
    }

    // Wait 100ms
    // reading is done at 10Hz like the electrical_periodic from rotorcraft main program
    usleep(100000);
  }

  return NULL;
}

/**
 * Check button thread
 */
static void *button_read(void *data __attribute__((unused)))
{
  struct input_event ev;
  ssize_t n;

  /* Open power button event sysfs file */
  int fd_button = open("/dev/input/pm_mcu_event", O_RDONLY);
  if (fd_button == -1) {
    printf("Unable to open mcu_event to read power button state\n");
    return NULL;
  }

  while (TRUE) {
    /* Check power button (read is blocking) */
    n = read(fd_button, &ev, sizeof(ev));
    if (n == sizeof(ev) && ev.type == EV_KEY && ev.code == KEY_POWER && ev.value > 0) {
      printf("Stopping Paparazzi from power button and rebooting\n");
      usleep(1000);
      int ret __attribute__((unused)) = system("reboot.sh");
      exit(0);
    }
  }

  return NULL;
}

/**
 * Baro reading thread
 */
static void *baro_read(void *data __attribute__((unused)))
{
  static int32_t baro_parrot_minidrone_raw;
  struct input_event ev;
  ssize_t n;

  /* Open Baro event sysfs file */
  int fd_baro = open("/dev/input/baro_event", O_RDONLY);
  if (fd_baro == -1) {
    printf("Unable to open baro_event to read baro state\n");
    return NULL;
  }

  while (TRUE)
  {
    /* Check new pressure (read is blocking?) */
    n = read(fd_baro, &ev, sizeof(ev));
	if (n == sizeof(ev) && ev.type == EV_ABS && ev.code == ABS_PRESSURE) {
	      baro_parrot_minidrone_raw = ev.value;
	      //printf("Read Baro RAW: %d\n", baro_parrot_minidrone_raw);
	      // From datasheet: raw_pressure / 4096 -> pressure in hPa
	      // send data in Pa
	      float pressure = 100.f * ((float)baro_parrot_minidrone_raw) / 4096.f;
	      //printf("Baro pressure: %f\n", pressure);
	      AbiSendMsgBARO_ABS(BARO_BOARD_SENDER_ID, pressure);
    }
  }

  return NULL;
}

/*
 Sonar reading thread
 This  maybe of help getting it to work...
 https://github.com/Parrot-Developers/mambo-opensource/tree/master/sources/linux-2.6.36/linux-2.6.36/drivers/parrot/ultra_sound
*/
static void *sonar_read(void *data __attribute__((unused)))
{
  static int32_t sonar_parrot_minidrone_raw;
  struct input_event ev;
  ssize_t n;

  /* Open sonar event sysfs file */
  int fd_sonar = open("/dev/ultra_snd", O_RDONLY);
  if (fd_sonar == -1) {
    printf("Unable to read sonar state\n");
    return NULL;
  }

  while (TRUE)
  {
    /* Check new sonar (read is blocking?) */
    n = read(fd_sonar, &ev, sizeof(ev));
    //printf("Read n: %d", n);
    //printf("Read type, code, value: %d,%d,%d\n", ev.type, ev.code, ev.value);
	if (n == sizeof(ev) && ev.type == 0 && ev.code == 0) {
	      sonar_parrot_minidrone_raw = ev.value;
	      printf("Read sonar RAW: %d\n", sonar_parrot_minidrone_raw);
	      // send data in cm?
	      float range = sonar_parrot_minidrone_raw/42; //fixme
	      printf("Sonar range: %f\n", range);
	     // AbiSendMsgBARO_ABS(SONAR_BOARD_SENDER_ID, distance);
    }
  }

  return NULL;
}

/**
 * Bottom camera reading thread
 */
//static void *bottom_camera_read(void *data __attribute__((unused)))
//{
//  //TODO: No bottom camera implemeted yet, feel free to help out and create it...
//  return NULL;
//}


void board_init(void)
{
  /*
   *  Stop original processes using pstop/ptart commands
   *  Don't kill to avoid automatic restart
   *
   */
  int ret __attribute__((unused));
  ret = system("pstop delosd");
  ret = system("pstop dragon-prog");
  usleep(50000); /* Give 50ms time to end on a busy system */

  /* Start battery reading thread */
  pthread_t bat_thread;
  if (pthread_create(&bat_thread, NULL, bat_read, NULL) != 0) {
    printf("[parrot_minidrone_board] Could not create battery reading thread!\n");
  }

  /* Start button reading thread */
  pthread_t button_thread;
  if (pthread_create(&button_thread, NULL, button_read, NULL) != 0) {
    printf("[parrot_minidrone_board] Could not create button reading thread!\n");
  }

  /* Start baro reading thread */
  pthread_t baro_thread;
  if (pthread_create(&baro_thread, NULL, baro_read, NULL) != 0) {
    printf("[parrot_minidrone_board] Could not create baro reading thread!\n");
  }

  /* Start sonar reading thread */
  pthread_t sonar_thread;
  //if (pthread_create(&sonar_thread, NULL, sonar_read, NULL) != 0) {
  //  printf("[parrot_minidrone_board] Could not create sonar reading thread!\n");
  //}

  /* Start bottom_camera reading thread */
 // pthread_t bottom_camera_thread;
 // if (pthread_create(&bottom_camera_thread, NULL, bottom_camera_read, NULL) != 0) {
//    printf("[parrot_minidrone_board] Could not create bottom camera reading thread!\n");
 // }
  //IDEA: check ram and storage size left and give warning if below a certin treshhole
}

void board_init2(void){/* Not used yet feel fee to inject you improved sourcecode here*/}

void baro_init(void) {}
void baro_periodic(void) {}
void baro_event(void) {}
