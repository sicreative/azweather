/* Copyright (c) SC Lee. All rights reserved.
   Licensed under the GNU GPLv3 License.

*/

/* Some of the code in this file was copied from  microsoft PWM highlevel example
/* @link https://github.com/Azure/azure-sphere-samples/tree/master/Samples/PWM/PWM_HighLevelApp
/* Copyright (c) Microsoft Corporation. All rights reserved.
   Licensed under the MIT License. */

#pragma once
#include "applibs_versions.h"
#include "applibs/adc.h"
#include "mt3620_avnet_dev.h"


//LED type
#define LED1RED 1
#define LED1BLUE 2
#define LED1GREEN 3
#define LEDAPP 4
#define LEDWIFI 5

#define LED1MIXCOLOR 6
#define LED1FLASH 7
#define LEDAPPFLASH 8
#define LEDWIFIFLASH 9

//define of constant for blinking pattern
#define LED_ON 0xFFFFFFFF
#define LED_OFF 0x0
#define LED_FLASH 0x0F0F0F0F
#define LED_FASTFLASH 0xAAAAAAAA

 /**
  *@brief  Init of LED Driver
  * @retval i : status
  */
uint8_t led_init(void);


/**
  * @brief  set led command 
  * @param[in]  type: LED command type
  * @param[in]  message: the command data
  * @param[in]  len:  length of message
  * @retval i : status
  */
uint8_t led_set(uint8_t type, const uint8_t* message, uint16_t len);

/**
 *@brief  Close of LED Driver
 * @retval i : status
 */
void closeled(void);




