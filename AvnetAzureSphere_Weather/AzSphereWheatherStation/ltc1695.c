/* Copyright (c) SC Lee. All rights reserved.
   Licensed under the GNU GPLv3 License.
   Fan control
*/

#include "ltc1695.h"
#include <errno.h>
#include "i2c.h"
#include "web_tcp_server.h"






#include "applibs_versions.h"

#include <applibs/log.h>
#include <applibs/i2c.h>

#include "mt3620_avnet_dev.h"
#include "deviceTwin.h"
#include "azure_iot_utilities.h"
#include "build_options.h"
#include "real_core.h"
#include "led.h"
#include "file.h"

uint8_t ltc1695_retreve_fan_status();
int ltc1695_save_fan_status();

uint8_t ltc1695_mode = LTC_1695_MODE_AUTO;
uint16_t ltc1695_value = 0;
uint16_t ltc1695_manual_value = 0;



void ltc1695_delay_ms(uint32_t period) {
	uint32_t second = period / 1000;
	uint32_t nsec = period % 1000 * 1000000;
	struct timespec time = { .tv_sec = second,.tv_nsec = nsec };
	while (nanosleep(&time, &time) == -1);
}

int8_t ltc1695_read(uint8_t* data) {
#ifdef ENABLE_LTC1695_READ_WRITE_DEBUG
	LogWebDebug("ltc1695_read()\n");
	LogWebDebug("reg: %0x\n", reg_addr);
	LogWebDebug("len: %d\n", len);
	;
#endif
	/*
	// Set the register address to read
	int32_t retVal = I2CMaster_Write(i2cFd, LTC1695_SEND, NULL, 0);
	if (retVal < 0) {
		LogWebDebug("ERROR: ltc1695_read(write step): errno=%d (%s)\n", errno, strerror(errno));
		return -1;
	}
	*/


	// Read the data into the provided buffer
	int32_t retVal = I2CMaster_Read(i2cFd, LTC1695_ADDRESS, data, 1);
	if (retVal < 0) {
		LogWebDebug("ERROR: ltc1695_read(read step): errno=%d (%s)\n", errno, strerror(errno));
		return -1;
	}

#ifdef ENABLE_LTC1695_READ_WRITE_DEBUG
	LogWebDebug("Read returned: ");
	for (int i = 0; i < len; i++) {
		LogWebDebug("%0x: ", data[i]);
	}
	LogWebDebug("\n\n");
#endif 	   

	return 0;


}

int8_t ltc1695_write(uint8_t* data) {

#ifdef ENABLE_LTC1695_READ_WRITE_DEBUG
	LogWebDebug("ltc1695_write()\n");
	LogWebDebug("reg: %0x\n", reg_addr);
	LogWebDebug("len: %0x\n", len);
	LogWebDebug("bufp contents: ");
	for (int i = 0; i < len; i++) {

		LogWebDebug("%0x: ", data[i]);
	}
	LogWebDebug("\n");
#endif 

	


#ifdef ENABLE_LTC1695_READ_WRITE_DEBUG
	LogWebDebug("cmdBuffer contents: ");
	for (int i = 0; i < len + 1; i++) {

		LogWebDebug("%0x: ", cmdBuffer[i]);
	}
	LogWebDebug("\n");
#endif

	// Write the data to the ltc1695device
	int32_t retVal = I2CMaster_Write(i2cFd, LTC1695_ADDRESS, data, 1);
	if (retVal < 0) {
		LogWebDebug("ERROR: ltc1695_write: errno=%d (%s)\n", errno, strerror(errno));
		return -1;
	}
#ifdef ENABLE_LTC1695_READ_WRITE_DEBUG
	LogWebDebug("Wrote %d bytes to ltc1695device.\n\n", retVal);
#endif
	return 0;
}

uint8_t ltc1695_getstatus(void) {
	uint8_t data;
	ltc1695_read(&data);
	return data;

}





uint8_t ltc1695_init(void) {
	uint8_t data = -1;
	if (ltc1695_read(&data))
		return data;
	
	if (data == LTC1695_STATUS_NO_FAULT) {
		LogWebDebug("LTC1695 fan control found\n");
	}else if(data==LTC1695_STATUS_OVERCURRENT || data==LTC1695_STATUS_THERMAL_SHUTDOWN) {
		LogWebDebug("LTC1695 fan may fault\n");
	}else {
		LogWebDebug("LTC1695 fan control not found\n");
	}


	uint8_t ledapp = 127;
	set_led(LEDAPP, &ledapp, 1);
	uint32_t ledflash = LED_ON;
	set_led(LEDAPPFLASH, &ledflash,4);

	ltc1695_retreve_fan_status();

	return data;
}


int ltc1695_save_fan_status() {

	//serialization 
	uint8_t data[LTC1695_FAN_STORAGE_SIZE];
	data[0] = ltc1695_mode;
	memcpy((uint8_t*)&data + 1, &ltc1695_manual_value, 2);

	
	 file_checksome_otherwise_save(data, FILE_FAN_STATUS_OFFSET, FILE_FAN_FILE_STATUS_LENGTH);
	//save to Permanent storage
	//file_set_mutable(data, FILE_FAN_STATUS_OFFSET, FILE_FAN_FILE_STATUS_LENGTH);
	//return file_mutable_write();

	 return 0;

}

uint8_t ltc1695_retreve_fan_status() {
	//load permanent storage data
	uint8_t data[LTC1695_FAN_STORAGE_SIZE];
	
	int result = file_retrieve_mutable(data, FILE_FAN_STATUS_OFFSET, FILE_FAN_FILE_STATUS_LENGTH);
	if (result == -1) {
		Log_Debug("No fan storage data saved");
		return result;
	}
	//Deserialization
	ltc1695_mode = data[0];
	memcpy(&ltc1695_manual_value , &data + 1,  2);
	//Force update LED
	ltc1695_setmode(ltc1695_mode);
	//Force update speed
	ltc1695_set(ltc1695_manual_value,false);
	


}

uint8_t ltc1695_set(uint16_t mV, _Bool start_boost) {
	if (mV > LTC_1695_MAX_VOLTAGE || mV < LTC_1695_MIN_VOLTAGE)
		return LTC1695_ERR_VOLTAGE_OUTOFRANGE;

	

	if (ltc1695_mode == LTC_1695_MODE_MANU)
		ltc1695_manual_value = mV;

	ltc1695_save_fan_status();

	if (ltc1695_value == mV)
		return 0;

	uint8_t command = 0x40 * start_boost;
	command += mV / 78;

	
	
	ltc1695_write(&command);
	
	ltc1695_value = mV;
#ifdef IOT_CENTRAL_APPLICATION
	updateFanDevice();
#endif // !IOT_CENTRAL_APPLICATION
	return command;

}


uint8_t ltc1695_auto(uint8_t pm, uint8_t iaq) {

	const speed[7] = { 0,3000,4200,4500,4922,4922,4922 };

	if (ltc1695_mode != LTC_1695_MODE_AUTO)
		return -1;
	uint16_t mV = speed[pm>iaq?pm:iaq];
	ltc1695_set(mV,false);
		return 0;
}


uint8_t ltc1695_setmode(_Bool automode) {
	if (automode) {
		ltc1695_mode = LTC_1695_MODE_AUTO;
		uint32_t ledflash = LED_ON;
		set_led(LEDAPPFLASH, &ledflash, 4);
		return 1;
	}
	
		ltc1695_mode = LTC_1695_MODE_MANU;
		ltc1695_set(ltc1695_manual_value, false);
		uint32_t ledflash = LED_FLASH;
		set_led(LEDAPPFLASH, &ledflash, 4);
		return 0;
	
}

uint8_t ltc1695_button_press(int pressUp)
{
	if (pressUp) {
		if (ltc1695_mode == LTC_1695_MODE_AUTO) {
			ltc1695_setmode(false);
			ltc1695_set(0,false);
			
			return;
		}

		uint16_t new = ltc1695_value + 200 + (LTC_1695_MAX_VOLTAGE - ltc1695_value) / 2;
		if (new > LTC_1695_MAX_VOLTAGE)
			new = LTC_1695_MAX_VOLTAGE;
		ltc1695_set(new,false);
	}else {
		if (ltc1695_value == 0 && ltc1695_mode == LTC_1695_MODE_MANU) {
			ltc1695_setmode(true);
			ltc1695_save_fan_status();
			return;
		}
		uint16_t new = ltc1695_value - 200 - (LTC_1695_MAX_VOLTAGE - ltc1695_value) / 2;
		if (new < LTC_1695_MIN_FAN_VOLTAGE)
			new = 0;
		ltc1695_set(new,false);
	}

	return 0;
}

