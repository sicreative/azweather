/* Copyright (c) SC Lee. All rights reserved.
   Licensed under the GNU GPLv3 License.

   Some of the code copied from Avnet
   @link https://github.com/CloudConnectKits/Azure_Sphere_SK_ADC_RTApp
*/


#pragma once

#include <stdbool.h>
#include "epoll_timerfd_utilities.h"

//// OLED
#include "oled.h"

//// BME680 sensor
#include "bme680_sensor.h"

//// LTC1695 motor driver
#include "ltc1695.h"

#define LSM6DSO_ID         0x6C   // register value
#define LSM6DSO_ADDRESS	   0x6A	  // I2C Address


#include <stdint.h>
#include <stdio.h>


/**
  *@brief  init of I2C bus
  *
  * @retval i : status
  */
int initI2c(void);

/**
  *@brief  close of I2C bus
  *
  * @retval i : status
  */
void closeI2c(void);

//IAQ class
int iaq_class;


//  The file descriptor for the I2C interface.
extern int i2cFd;