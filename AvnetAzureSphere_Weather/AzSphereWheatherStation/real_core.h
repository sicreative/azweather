/* Copyright (c) SC Lee. All rights reserved.
   Licensed under the GNU GPLv3 License.

   For Inter-communication of Real Core (M4)

*/

#pragma once
#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <time.h>

#include "applibs_versions.h"
#include "build_options.h"
#include "epoll_timerfd_utilities.h"
#include <errno.h>
#include <signal.h>
#include <stdbool.h>

#include <applibs/log.h>

#include <sys/time.h>
#include <sys/socket.h>
#include <applibs/application.h>
#include <string.h> 

#include "bme680_sensor.h"
#include "adc.h"


//Type of Inter-Comm 
#define LOG	0
#define ADC_LIGHT 1
#define BME680_RECV 2
#define BME680_SEND 3
#define BME680_STATUS_RECV 4
#define BME680_STATUS_SEND 5





// BSEC real core componentId
static const char rtAppComponentId[] = "ec6232b9-26d5-a49f-9980-0bec91dd3385";


// LED real core componentId
static const char ledrtAppComponentId[] = "09ffcb31-c819-02d2-9a6a-6768937c19d9";

// file descriptor
static int sockFd;
static int timerFd;
static int ledsockFd;

//Event Handler
static void TimerEventHandler(EventData* eventData);
static void SocketEventHandler(EventData* eventData);
static void LedSocketEventHandler(EventData* eventData);


// Real-core status
uint8_t RTCore_status;
uint8_t LedRTCore_status;



/**
  * @brief  real core init
  * @retval i : status
  */
uint8_t real_core_init();
/**
  * @brief  send ADC read requirment
  * 
  * @retval i : status
  */
uint8_t send_request_ADC(void);
/**
  * @brief  send message to Real Core
  * @param[in]  type : Type
  * @param[in]  message : value
  * @param[in]  len : length
  * @retval i : status
  */
uint8_t send_message_to_rtcore(uint8_t type, const uint8_t* message, uint16_t len);

/**
  * @brief  set the LED (when using Real core softPWM)
  * @param[in]  type : Type
  * @param[in]  message : value
  * @param[in]  len : length
  * @retval i : status
  */
uint8_t set_led(uint8_t type, const uint8_t* message, uint16_t len);

/**
  * @brief  close real core

  * @retval i : status
  */
void closerealcore();