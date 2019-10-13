/* Copyright (c) SC Lee. All rights reserved.
   Licensed under the GNU GPLv3 License.
   Fan control  
*/

#pragma once
#include <stdint.h>
#include <stdio.h>

#define LTC1695_FAN_STORAGE_SIZE 3

#define LTC1695_ADDRESS 0x74

#define LTC1695_STATUS_NO_FAULT 0
#define LTC1695_STATUS_OVERCURRENT 0x80
#define LTC1695_STATUS_THERMAL_SHUTDOWN 0x40

#define LTC1695_ERR_VOLTAGE_OUTOFRANGE -1


#define LTC_1695_MIN_VOLTAGE 0
#define LTC_1695_MAX_VOLTAGE 4922
#define LTC_1695_MIN_FAN_VOLTAGE 1800


#define LTC_1695_MODE_AUTO 1
#define LTC_1695_MODE_MANU 0

//Fan mode Auto/Manul
uint8_t ltc1695_mode;
//Fan current value
uint16_t ltc1695_value;
//Fan manual preset value
uint16_t ltc1695_manual_value;

/**
  * @brief  init fan controller

  * @retval i : status
  */
uint8_t ltc1695_init(void);

/**
  * @brief  set fan voltage
  * @param[in]  mv: voltage of fan speed
  * @param[in]  start_boost:  full voltage for first 250ms
  * @retval i : status
  */
uint8_t ltc1695_set(uint16_t mV,_Bool start_boost);

/**
  * @brief  set fan mode Auto/manual
  * @param[in]:  mode
  * @retval i : status
  */
uint8_t ltc1695_setmode(_Bool mode);
/**
  * @brief  get fan status
  * @retval i : status
  */
uint8_t ltc1695_getstatus(void);

/**
  * @brief  received the PM and IAQ value for fan voltage control
  * @param[in]:  pm current max PM value
  * @param[in]:  pm current IAQ value
  * @retval i : status
  */
uint8_t ltc1695_auto(uint8_t pm, uint8_t iaq);

/**
  * @brief response button event
  * @param[in]:  true: press Up button false: press Down button
  * @retval i : status
  */
uint8_t ltc1695_button_press(int pressUp);

