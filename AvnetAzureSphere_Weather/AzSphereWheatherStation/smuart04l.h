
/* Copyright (c) SC Lee. All rights reserved.
    Licensed under the GNU GPLv3 License. */

   /* This header for Amphenol PM2.5 laser sensor.
   This is NOT offical driver by Amphenol and as use at your own risk. */

#ifndef SMUART04L_H 
#define SMUART04L_H 

#include <stdint.h>
#include <stdio.h>

#define SMUART04L_STATUS_UPDATE 0
#define SMUART04L_STATUS_WAITING_NEXT_BUFFER 1
#define SMUART04L_STATUS_NO_PM 3
#define SMUART04L_STATUS_ERR_CHECKSUM 4


/*
SM-UART-04L Error code based on Datasheet page 4
L_D13 Error code 0b0ABCDEFG
A = 1 Laser error
B = 1 Laser alarm
C = 1 High temperature alarm 
D = 1 Low temperature alarm 
E = 1 Fan error
F = 1 Fan speed compensation start 
G = 1 Fan speed alarm
*/

typedef struct smuart04l_error {
	_Bool laser_err;
	_Bool laser_alarm;
	_Bool high_temp;
	_Bool low_temp;
	_Bool fan;
	_Bool fan_speed_comp;
	_Bool fan_speed;


};




typedef struct smuart04l_pm_data{
	uint64_t id;
	//standard smoke for pm1,2.5 and 10
	uint16_t pm_1;
	uint16_t pm_2_5;
	uint16_t pm_10;

	//Enviroment for pm1,2.5,10

	uint16_t pm_1_e;
	uint16_t pm_2_5_e;
	uint16_t pm_10_e;

	uint8_t version;
	uint8_t numError;

	struct smuart04l_error error;


};

/**
  * @brief  UART buffer input
  * @param  receiveBuffer: UART buffer input
  * @param  str: uint8_t array output
  * @param  d: Number of zeros added
  * @retval i: receiver status 
  */
uint8_t smuart04l_update(uint8_t* receiveBuffer,size_t length);
/**
  * @brief  get PM value 
  * @retval i: struct of pm_data
  */
struct smuart04l_pm_data smuart04l_get(void);

/**
  * @brief  get PM1 value  ug/m3 (Standard Smoke, Calculated Value)
  * @retval i: pm1
  */
uint16_t smuart04l_getPM1(void);
/**
  * @brief  get PM2.5 value ug/m3 (Standard Smoke, Calculated Value)
  * @retval i: pm2.5
  */
uint16_t smuart04l_getPM2_5(void);
/**
  * @brief  get PM10 value ug/m3 (Standard Smoke, Calculated Value)
  * @retval i: pm10
  */
uint16_t smuart04l_getPM10(void);

/**
  * @brief  get PM1 value  ug/m3 (Environment, Calculated Value)
  * @retval i: pm1
  */
uint16_t smuart04l_getPM1_e(void);
/**
  * @brief  get PM2.5 value ug/m3 (Environment, Calculated Value)
  * @retval i: pm2.5
  */
uint16_t smuart04l_getPM2_5_e(void);
/**
  * @brief  get PM10 value ug/m3 (Environment, Calculated Value)
  * @retval i: pm10
  */
uint16_t smuart04l_getPM10_e(void);

/**
  * @brief  get sample id (sequence increment num from 0)
  * @retval i: pm10
  */
uint64_t smuart04l_getId(void);

/**
  * @brief  get num of error
  * @retval i: numoferror 
  */
uint8_t smuart04l_numoferror(void);

/**
  * @brief  geterrorcode
  * @retval i: error struct
  */
struct smuart04l_error smuart04l_getError(void);

/**
  * @brief  pm value to class 
  * @retval i: error struct
  */
uint8_t smuart04l_pm_to_class(uint16_t);


#endif