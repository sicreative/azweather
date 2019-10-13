
/* Copyright (c) SC Lee. All rights reserved.
   Licensed under the GNU GPLv3 License.

*/

/* Some of the code in this file was copied from  microsoft ADC example
/* @link https://github.com/Azure/azure-sphere-samples/tree/master/Samples/ADC/ADC_HighLevelApp
/* Copyright (c) Microsoft Corporation. All rights reserved.
   Licensed under the MIT License. */


#include "adc.h"

#include <errno.h>
#include <string.h>
#include <applibs/log.h>
#include "epoll_timerfd_utilities.h"

#define ADC_CONTROLLER 0
#define ADC_CHANNEL 0


static int adcControllerFd = -1;

// The size of a sample in bits
static int sampleBitCount = -1;

// The maximum voltage
static float sampleMaxVoltage = 2.5f;

uint8_t adc_init(){
	adcControllerFd = ADC_Open(ADC_CONTROLLER);
	if (adcControllerFd < 0) {
		Log_Debug("ADC_Open failed with error: %s (%d)\n", strerror(errno), errno);
		return -1;
	}

	sampleBitCount = ADC_GetSampleBitCount(adcControllerFd, ADC_CHANNEL);
	if (sampleBitCount == -1) {
		Log_Debug("ADC_GetSampleBitCount failed with error : %s (%d)\n", strerror(errno), errno);
		return -1;
	}
	if (sampleBitCount == 0) {
		Log_Debug("ADC_GetSampleBitCount returned sample size of 0 bits.\n");
		return -1;
	}

	int result = ADC_SetReferenceVoltage(adcControllerFd, ADC_CHANNEL,
		sampleMaxVoltage);
	if (result < 0) {
		Log_Debug("ADC_SetReferenceVoltage failed with error : %s (%d)\n", strerror(errno), errno);
		return -1;
	}





}





uint8_t adc_request(){
	uint32_t value;
	int result = ADC_Poll(adcControllerFd, ADC_CHANNEL, &value);
	if (result < -1) {
		Log_Debug("ADC_Poll failed with error: %s (%d)\n", strerror(errno), errno);
		
		return;
	}
	light_sensor = ((float)value * sampleMaxVoltage / 4095) * 1000000 / (3650 * 0.1428);

}

void closeadc() {
	CloseFdAndPrintError(adcControllerFd,"ADC");
}