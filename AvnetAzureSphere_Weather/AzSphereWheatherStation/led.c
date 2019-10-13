/* Copyright (c) SC Lee. All rights reserved.
   Licensed under the GNU GPLv3 License.

*/

/* Some of the code in this file was copied from  microsoft PWM highlevel example
/* @link https://github.com/Azure/azure-sphere-samples/tree/master/Samples/PWM/PWM_HighLevelApp
/* Copyright (c) Microsoft Corporation. All rights reserved.
   Licensed under the MIT License. */


#include "led.h"
#include <stdbool.h>
#include <string.h>
#include <errno.h>
#include "epoll_timerfd_utilities.h"
#include <applibs/log.h>
#include <applibs/pwm.h>


#define PWMCONTROLLER_1 1
#define PWMCONTROLLER_2 2

#define LED_RED_CHANNEL 0
#define LED_GREEN_CHANNEL 1
#define LED_BLUE_CHANNEL 2

#define LED_APP_CHANNEL 0
#define LED_WIFI_CHANNEL 1
extern epollFd;
extern terminationRequired;


static int pwmFd_1 = -1;
static int pwmFd_2 = -1;
static int blinkingTimerFd = -1;


int led_red = 0;
int led_green = 0;
int led_blue = 0;
int led_app = 0;
int led_wifi = 0;

uint32_t led1_flash = LED_ON;
uint32_t ledapp_flash = LED_ON;
uint32_t ledwifi_flash = LED_ON;
uint8_t led_flash_count = 0;


static const int LED1REDMIXMAX = 255;
static const int LED1BLUEMIXMAX = 64;
static const int LED1GREENMIXMAX = 239;
const uint8_t LEDMAX = 75;




static const uint32_t fullCycleNs_2 = 0x1FFFF;
static const uint32_t fullCycleNs_1 = fullCycleNs_2;
static const uint32_t stepIncrementNs = 1000;
static uint32_t dutyCycleNs = 10000;

uint8_t led_apply_force(int controller, int channel, int value);

// Timer state variables 0.5s
static const struct timespec stepIntervalNs = { .tv_sec = 0, .tv_nsec = 400000000 };

static inline uint8_t patterntrigger(uint32_t* pattern) {
	return (*pattern >> led_flash_count & 0x1);
}


// The polarity is inverted because LEDs are driven low
static PwmState ledPwmState = { .period_nsec = fullCycleNs_2,
							   .polarity = PWM_Polarity_Inversed,
							   .dutyCycle_nsec = 0,
							   .enabled = true };

/// <summary>
///     Handle LED timer event: change LED brightness.
/// </summary>
static void blinkingTimerEventHandler(EventData* eventData)
{
	if (ConsumeTimerFdEvent(blinkingTimerFd) != 0) {
		terminationRequired = true;
		return;
	}

	


		led_apply_force(pwmFd_2, LED_RED_CHANNEL, patterntrigger(&led1_flash) * led_red);
		led_apply_force(pwmFd_2, LED_GREEN_CHANNEL, patterntrigger(&led1_flash) * led_green);
		led_apply_force(pwmFd_2, LED_BLUE_CHANNEL, patterntrigger(&led1_flash) * led_blue);
		led_apply_force(pwmFd_1, LED_WIFI_CHANNEL, patterntrigger(&ledwifi_flash) * led_wifi);
		led_apply_force(pwmFd_1, LED_APP_CHANNEL, patterntrigger(&ledapp_flash) * led_app);
		
		//le(ledPwmState);


	

	if (++led_flash_count >= 32)
		led_flash_count = 0;

	//int result = PWM_Apply(pwmFd_1, LED_APP_CHANNEL, &ledPwmState);
	//if (result != 0) {
	//	log_debug("pwm_apply failed: result = %d, errno: %s (%d)\n", result, strerror(errno),
	//		errno);
	//	terminationrequired = true;
	//}

	
}

static EventData blinkingTimerEventData = { .eventHandler = &blinkingTimerEventHandler };


uint8_t led_apply_force(int controller,int channel,int value) {



	int fullcycleNs = controller == pwmFd_1 ? fullCycleNs_1 : fullCycleNs_2;

	value *= fullcycleNs;
	value /= 255;
	value *= LEDMAX;
	value /= 255;



	ledPwmState.dutyCycle_nsec = value;
	ledPwmState.period_nsec = fullcycleNs;

	int result = PWM_Apply(controller, channel, &ledPwmState);
	if (result != 0) {
		Log_Debug("PWM_Apply failed: result = %d, errno: %s (%d)\n", result, strerror(errno),
			errno);
		terminationRequired = true;
	}


}

uint8_t led_apply(uint8_t id, int value) {
	

	

	switch (id) {
	case LED1RED:
		led_red = value;
		break;
	case LED1GREEN:
		led_green = value;
		break;
	case LED1BLUE:
		led_blue = value;
		break;
	case LEDWIFI:
		led_wifi = value;
		break;
	case LEDAPP:
		led_app = value;
		break;
	default:
		return -1;

	}
	
	return 0;
//	return led_apply_force(controller,channel,value);

	
}

uint8_t led_init() {
	

	pwmFd_1 = PWM_Open(PWMCONTROLLER_1);


	if (pwmFd_1 == -1) {
		Log_Debug(
			"Error opening LED_1 PWM: %s (%d). "
			,
			strerror(errno), errno);
		return -1;
	}

	pwmFd_2 = PWM_Open(PWMCONTROLLER_2);

	if (pwmFd_1 == -1) {
		Log_Debug(
			"Error opening LED_2 PWM: %s (%d). "
			,
			strerror(errno), errno);
		return -1;
	}


	led_apply(LED1RED, 0);
	led_apply(LED1BLUE, 0);
	led_apply(LED1GREEN, 0);
	led_apply(LEDWIFI, 0);
	led_apply(LEDAPP, 0);


	blinkingTimerFd =
		CreateTimerFdAndAddToEpoll(epollFd, &stepIntervalNs, &blinkingTimerEventData, EPOLLIN);
	if (blinkingTimerFd < 0) {
		return -1;
	}

}

uint8_t led_set(uint8_t type, const uint8_t* data, uint16_t len) {

	switch (type) {
	case LED1RED:
		led_apply(LED1RED,data[0]);
		break;
	case LED1BLUE:
		led_apply(LED1BLUE, data[0]);
		break;
	case LED1GREEN:
		led_apply(LED1GREEN, data[0]);
		break;
	case LEDWIFI:
		led_apply(LEDWIFI, data[0]);
		break;
	case LEDAPP:
		led_apply(LEDAPP, data[0]);
		break;
	case LED1FLASH:
		memcpy(&led1_flash, data, 4);
		break;
	case LEDAPPFLASH:
		memcpy(&ledapp_flash, data, 4);
		break;
	case LEDWIFIFLASH:
		memcpy(&ledwifi_flash, data, 4);
		break;
	case LED1MIXCOLOR:
		led_apply(LED1RED, (uint8_t)((float)data[0] / (float)255 * (float)LED1REDMIXMAX));
		led_apply(LED1BLUE, (uint8_t)((float)data[1] / (float)255 * (float)LED1BLUEMIXMAX));
		led_apply(LED1GREEN, (uint8_t)((float)data[2] / (float)255 * (float)LED1GREENMIXMAX));
		break;

	}


}


void closeled() {
	CloseFdAndPrintError(pwmFd_1, "PWM 1");
	CloseFdAndPrintError(pwmFd_2, "PWM 2");
	CloseFdAndPrintError(blinkingTimerFd, "blinkingTimer");

}