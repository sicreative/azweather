/* Copyright (c) SC Lee. All rights reserved.
   Licensed under the GNU GPLv3 License. */

   /// This header for BME680 relative
   /// Work together with Bosch's driver for AzSphere platform 
   /// This is NOT offical driver by Bosch and as use at your own risk.
/* Handling BME680 sensor  */
#pragma once


#include <stdint.h>
#include <stdio.h>
#include "bme680.h"

///Define of BSEC virutal sensor type 

#define BME680_VIR_SENSOR_IAQ 0
#define BME680_VIR_SENSOR_TEMP 1
#define BME680_VIR_SENSOR_PRESSURE 2
#define BME680_VIR_SENSOR_HUMIDITY 3
#define BME680_VIR_SENSOR_GAS 4
#define BME680_VIR_SENSOR_ECO2 5
#define BME680_VIR_SENSOR_EVOC 6
#define BME680_VIR_SENSOR_STATIC_IAQ 7
#define BME680_VIR_SENSOR_RAW_TEMP 8
#define BME680_VIR_SENSOR_RAW_HUMIDITY 9
///Define of BSEC accuracy 
#define BME680_ACCURACY_UNRELIABLE 0
#define BME680_ACCURACY_LOW 1
#define BME680_ACCURACY_MEDIUM 2
#define BME680_ACCURACY_HIGH 3
//// Max BSEC Virtual Sensor 
#define BME680_MAX_VS 10




/// define for quick access virtaul sensor data
#define bme680_temperature bme680_virtual_sensor_data[BME680_VIR_SENSOR_TEMP].value
#define bme680_humidity bme680_virtual_sensor_data[BME680_VIR_SENSOR_HUMIDITY].value
#define bme680_pressure bme680_virtual_sensor_data[BME680_VIR_SENSOR_PRESSURE].value
#define bme680_iaq (uint16_t)bme680_virtual_sensor_data[BME680_VIR_SENSOR_IAQ].value
#define bme680_siaq (uint16_t)bme680_virtual_sensor_data[BME680_VIR_SENSOR_STATIC_IAQ].value
#define bme680_eco2 (uint16_t)bme680_virtual_sensor_data[BME680_VIR_SENSOR_ECO2].value
#define bme680_evoc (uint16_t)bme680_virtual_sensor_data[BME680_VIR_SENSOR_EVOC].value

/// struct for BSEC virtual data
typedef struct bme680_virtual_sensor_reading {
	float value;
	uint8_t accuracy;
};
///Declare of virtual sensor 
struct bme680_virtual_sensor_reading bme680_virtual_sensor_data[BME680_MAX_VS];

/// sea level pressure for calcuate the altitube 
float sea;

/**
  *@brief  BME680 init
  * @retval i : status
  */
uint8_t bme680_sensor_init(void);

/**
  * @brief  BME680 force meassurement
  *
  * @retval i: status
  */
uint8_t bme680_sensor_force(void);

/**
  * @brief  BME680 get last meassurement data and sent to real core for BSEC
  *
  * @retval i: status
  */
uint8_t bme680_sensor_get(void);

/**
  * @brief Handler recevied data from real core BSEC result
  *
  * @retval i: status
  */
uint8_t bme680_sensor_bsec_feedback(uint8_t* data, uint16_t len);




/**
  * @brief Call for write BSEC status in permanent storage
  *
  * @retval i: status
  */
uint8_t bme680_sensor_bsec_status_write(uint8_t* data);

/**
  * @brief Call for read BSEC status in permanent storage
  *
  * @retval i: status
  */
uint8_t bme680_sensor_bsec_status_read(uint8_t* data);




