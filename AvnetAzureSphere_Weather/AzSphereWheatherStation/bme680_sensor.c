
/* Copyright (c) SC Lee. All rights reserved.
   Licensed under the GNU GPLv3 License. 

   Some of the code reference of Bosch driver integraded guide 
*/


#include <errno.h>






#include "applibs_versions.h"
#include "mt3620_avnet_dev.h"
#include "deviceTwin.h"
#include "azure_iot_utilities.h"
#include "build_options.h"
#include <applibs/log.h>
#include <applibs/i2c.h>
#include <applibs\rtc.h>

#include "bme680_sensor.h"
#include "bsec_datatypes.h"
#include "real_core.h"
#include "file.h"
#include "ltc1695.h"
#include "web_tcp_server.h"
#include "i2c.h"







struct bme680_dev bme680dev;
struct bme680_field_data bme680_sensor_data;


extern int epollFd;
static int bme680PollTimerFd = -1;

int64_t timestamp = 0;
float sea = 1013.5;

bsec_bme_settings_t setting;


/// <summary>
///     delay timer ns
/// </summary>
void bme680_sensor_delay_ns(int64_t period) {
	uint32_t second = period / 1000000000;
	uint32_t nsec = period % 1000000000;
	struct timespec time = { .tv_sec = second,.tv_nsec = nsec };
	while (nanosleep(&time, &time) == -1);
}


/// <summary>
///     delay timer ms for bme680 driver suspend 
/// </summary>
void bme680_sensor_delay_ms(uint32_t period) {
	uint32_t second = period / 1000;
	uint32_t nsec = period % 1000 * 1000000;
	struct timespec time = { .tv_sec = second,.tv_nsec = nsec };
	while (nanosleep(&time, &time) == -1);
}

/// <summary>
///     custom read I2C bus function for bme680 driver
/// </summary>
int8_t bme680_sensor_read(uint8_t dev_id, uint8_t reg_addr, uint8_t* data, uint16_t len) {
#ifdef ENABLE_BME680_READ_WRITE_DEBUG
	LogWebDebug("BME680_read()\n");
	LogWebDebug("reg: %0x\n", reg_addr);
	LogWebDebug("len: %d\n", len);
	;
#endif
	
	

	////The BME680 seen some issue by reading consecutive as batch for reading sensor data by using I2CMaster_Read,
	////so we read it once by once.

	uint8_t* data_inc = data;
	uint8_t reg_addr_inc = reg_addr;
	for (uint8_t i = 0;i < len;i++) {

		
		int32_t retVal = I2CMaster_Write(i2cFd, dev_id, &reg_addr_inc, 1);
		if (retVal < 0) {
			LogWebDebug("ERROR: BME680_read(write step): errno=%d (%s)\n", errno, strerror(errno));
			return -1;
		}
		++reg_addr_inc;
		
		I2CMaster_Read(i2cFd, dev_id, data_inc, 1);
		if (retVal < 0) {
			LogWebDebug("ERROR: BME680_read(read step): errno=%d (%s)\n", errno, strerror(errno));
			return -1;
		}
		++data_inc;
	}


#ifdef ENABLE_BME680_READ_WRITE_DEBUG 
	LogWebDebug("Read returned: ");
	for (int i = 0; i < len; i++) {
		LogWebDebug("%0x: ", data[i]);
	}
	LogWebDebug("\n\n");
#endif 	  


	return 0;

	
}

/// <summary>
///     custom write I2C bus function for bme680 driver
/// </summary>
int8_t bme680_sensor_write(uint8_t dev_id, uint8_t reg_addr, uint8_t* data, uint16_t len) {

#ifdef ENABLE_BME680_READ_WRITE_DEBUG
	LogWebDebug("BME680_write()\n");
	LogWebDebug("reg: %0x\n", reg_addr);
	LogWebDebug("len: %0x\n", len);
	LogWebDebug("bufp contents: ");
	for (int i = 0; i < len; i++) {

		LogWebDebug("%0x: ", data[i]);
	}
	LogWebDebug("\n");
#endif 

	// Construct a new command buffer that contains the register to write to, then the data to write
	uint8_t cmdBuffer[len + 1];
	cmdBuffer[0] = reg_addr;
	for (int i = 0; i < len; i++) {
		cmdBuffer[i + 1] = data[i];
	}

#ifdef ENABLE_BME680_READ_WRITE_DEBUG
	LogWebDebug("cmdBuffer contents: ");
	for (int i = 0; i < len + 1; i++) {

		LogWebDebug("%0x: ", cmdBuffer[i]);
	}
	LogWebDebug("\n");
#endif

	// Write the data to the bme680device
	int32_t retVal = I2CMaster_Write(i2cFd, dev_id, cmdBuffer, (size_t)len + 1);
	if (retVal < 0) {
		LogWebDebug("ERROR: bme680_write: errno=%d (%s)\n", errno, strerror(errno));
		return -1;
	}
#ifdef ENABLE_BME680_READ_WRITE_DEBUG
	LogWebDebug("Wrote %d bytes to bme680device.\n\n", retVal);
#endif
	return 0;
}



uint8_t bme680_sensor_init() {
	bme680dev.dev_id = BME680_I2C_ADDR_PRIMARY;
	bme680dev.intf = BME680_I2C_INTF;
	bme680dev.read = bme680_sensor_read;
	bme680dev.write = bme680_sensor_write;
	bme680dev.delay_ms = bme680_sensor_delay_ms;
	bme680dev.amb_temp = 25;
	int8_t result = BME680_OK;
	result = bme680_init(&bme680dev);
	if (result == BME680_OK)
		LogWebDebug("BME680 found!\n");
	else if (result == BME680_E_DEV_NOT_FOUND) {
		LogWebDebug("BME680 not found!\n");
		return;
	}




	bme680_get_sensor_data(&bme680_sensor_data, &bme680dev);

	

	

	//set default value 
	setting.heater_temperature = 520;
	setting.heating_duration = 150;

	setting.humidity_oversampling = BME680_OS_2X;
	setting.pressure_oversampling = BME680_OS_4X;
	setting.temperature_oversampling = BME680_OS_8X;
	setting.run_gas = BME680_ENABLE_GAS_MEAS;
	setting.trigger_measurement = 1;



	uint8_t status_data[BSEC_MAX_STATE_BLOB_SIZE];


	//read bsec status from storage and send to real core	
	result = bme680_sensor_bsec_status_read(status_data);
	if (result==0)
		send_message_to_rtcore(BME680_STATUS_SEND, status_data, BSEC_MAX_STATE_BLOB_SIZE);

	//force first reading of sensor
	bme680_sensor_force();
}




uint8_t bme680_sensor_force() {

	
	if (bme680PollTimerFd != -1) {
		CloseFdAndPrintError(bme680PollTimerFd, "BME680 close error");
		bme680PollTimerFd = -1;
	}
	uint8_t set_required_settings;

	/* Set the temperature, pressure and humidity settings */
	bme680dev.tph_sett.os_hum = setting.humidity_oversampling;
	bme680dev.tph_sett.os_pres = setting.pressure_oversampling;
	bme680dev.tph_sett.os_temp = setting.temperature_oversampling;



	/* Set the remaining gas sensor settings and link the heating profile */
	bme680dev.gas_sett.run_gas = setting.run_gas;
	/* Create a ramp heat waveform in 3 steps */
	bme680dev.gas_sett.heatr_temp = setting.heater_temperature; /* degree Celsius */
	bme680dev.gas_sett.heatr_dur = setting.heating_duration; /* milliseconds */
	if (!setting.trigger_measurement)
		return;

		/* Select the power mode */
	/* Must be set before writing the sensor configuration */
	bme680dev.power_mode = BME680_FORCED_MODE;

	/* Set the required sensor settings needed */
	set_required_settings = BME680_OST_SEL | BME680_OSP_SEL | BME680_OSH_SEL | BME680_FILTER_SEL | BME680_GAS_SENSOR_SEL;

	/* Set the desired sensor configuration */
	int8_t result = bme680_set_sensor_settings(set_required_settings, &bme680dev);

	
	// get sensor require meassurment time
	uint16_t meas_period;

	bme680_get_profile_dur(&meas_period, &bme680dev);


	//compare as bsec next_call time, and delay to that time if necessary 
	struct timespec currentTime;
	if (clock_gettime(CLOCK_REALTIME, &currentTime) == -1) {
	}
	int64_t current = (int64_t)currentTime.tv_sec * 1000000000 + currentTime.tv_nsec;
	int64_t ns = setting.next_call - current;
	if (ns > 1000)
		bme680_sensor_delay_ns(ns);
	

	/* Set the bme680 mode, force start meassurement */
	result = bme680_set_sensor_mode(&bme680dev);

	

	//update meassurment timestamp
	if (clock_gettime(CLOCK_REALTIME, &currentTime) == -1) {
	
	}
	timestamp = currentTime.tv_sec * 1e9 + currentTime.tv_nsec;

	//waiting bme680 working 
	bme680_sensor_delay_ms(meas_period);

	//get bme680 reading
	bme680_sensor_get();

	




}


static EventData bme680EventData = { .eventHandler = &bme680_sensor_force };

uint8_t bme680_sensor_bsec_feedback(uint8_t* data, uint16_t len) {

	

	int8_t virtual_sensor_size = (len - 21) / 5;
	
	///copy received setting from BSEC 
	memcpy(&setting, data + virtual_sensor_size * 5, 21);

	///reorganise data to virtual sensor struture
	union {
		float f;
		uint8_t u8[4];
	}utof;
	int8_t j = virtual_sensor_size*5-1;
	while (--virtual_sensor_size >= 0) {
		bme680_virtual_sensor_data[virtual_sensor_size].accuracy = data[j--];
		for (int8_t i=3;i >=0 ;i--)
			utof.u8[i] = data[j--];
		bme680_virtual_sensor_data[virtual_sensor_size].value = utof.f;
		
	
	}


	LogWebDebug("BSEC T: %.2f,IAQ: %d / %d, eCO2: %dppm, eVOC: %dppm\n", bme680_temperature, bme680_iaq,bme680_siaq, bme680_eco2, bme680_evoc);
	LogWebDebug("BSEC: H: %.2f,P: %.2fhPa, accuracy: iaq:%d temp:%d\n", bme680_humidity, bme680_pressure /100.0f, bme680_virtual_sensor_data[BME680_VIR_SENSOR_IAQ].accuracy, bme680_virtual_sensor_data[BME680_VIR_SENSOR_TEMP].accuracy);
	


	if (setting.trigger_measurement != 1)
		return;
	
	

	/// build timer for start of next meassurement

	struct timespec currentTime;
	if (clock_gettime(CLOCK_REALTIME, &currentTime) == -1) {

	}
	int64_t current = (int64_t)currentTime.tv_sec * 1000000000 + currentTime.tv_nsec;

	//Last 50ms should require nanostop hold for more precision meassure
	int64_t period = setting.next_call - current - 50000000;
	struct timespec bme680period = { .tv_sec = period / 1e9,.tv_nsec = period % 1000000000 };

	
	if (period > 0) {
		bme680PollTimerFd = CreateTimerFdAndAddToEpoll(epollFd, &bme680period, &bme680EventData, EPOLLIN);
		if (bme680PollTimerFd < 0) {
			return -1;
		}

		
	}




}


uint8_t bme680_sensor_bsec_status_write(uint8_t* data) {
	file_set_mutable(data,FILE_BME680_FILE_STATUS_OFFSET, FILE_BME680_FILE_STATUS_LENGTH);
	return file_mutable_write();

}


uint8_t bme680_sensor_bsec_status_read(uint8_t* data) {
	
	return file_retrieve_mutable(data, FILE_BME680_FILE_STATUS_OFFSET, FILE_BME680_FILE_STATUS_LENGTH);
	
	
}


uint8_t bme680_sensor_get() {

	uint8_t result = BME680_OK;

	

	
	result = bme680_get_sensor_mode(&bme680dev);
	
	//wait until sleep if the sensor under forced mode
	while (bme680dev.power_mode == BME680_FORCED_MODE)
		bme680_sensor_delay_ms(20);

	
	result = bme680_get_sensor_data(&bme680_sensor_data, &bme680dev);
	
	/* Trigger the next measurement if you would like to read data out continuously */
	
	LogWebDebug("BME680: %#02X  \n", bme680_sensor_data.status);
	

	if (bme680dev.new_fields) {
		uint8_t test;
		

		LogWebDebug("Time:%jd T: %.2f degC, P: %.2f hPa, H %.2f %%rH ", timestamp, bme680_sensor_data.temperature / 100.0f,
			bme680_sensor_data.pressure / 100.0f, bme680_sensor_data.humidity / 1000.0f);
		/* Avoid using measurements from an unstable heating setup */
		if (bme680_sensor_data.status & BME680_GASM_VALID_MSK)
			LogWebDebug(", G: %d ohms", bme680_sensor_data.gas_resistance);

		LogWebDebug("\n");
		
		//send meassured value to real core
		uint16_t len = 8 + 2 + 4 * 3;
		uint8_t send[len];
		memcpy(send, &timestamp,8);

		
		memcpy(send + 8, &bme680_sensor_data.temperature, 2);
		memcpy(send + 10, &bme680_sensor_data.pressure, 4);
		memcpy(send + 14, &bme680_sensor_data.humidity, 4);
		memcpy(send + 18, &bme680_sensor_data.gas_resistance, 4);
	
	

	
		send_message_to_rtcore(BME680_SEND, send, len);
	
		
		
	}

	





	

}
