
/* Copyright (c) SC Lee. All rights reserved.
   Licensed under the GNU GPLv3 License.

*/

/* Some of the code in this file was copied from  microsoft ADC example
/* @link https://github.com/Azure/azure-sphere-samples/tree/master/Samples/ADC/ADC_HighLevelApp
/* Copyright (c) Microsoft Corporation. All rights reserved.
   Licensed under the MIT License. */

   

   


#pragma once
#include "applibs_versions.h"
#include "applibs/adc.h"
#include "mt3620_avnet_dev.h"


// Data of light sensor
float light_sensor;

/**
  *@brief  ADC init
  * 
  * @retval i : status
  */
uint8_t adc_init(void);

/**
  *@brief  request read of ADC sensor
  * 
  * @retval i : status
  */
uint8_t adc_request(void);


/**
  *@brief  close ADC 
  * 
  * @retval i : status
  */
void closeadc(void);