/* Copyright (c) SC Lee. All rights reserved.
   Licensed under the GNU GPLv3 License.

   Some of the code copied from Avnet 
   @link https://github.com/CloudConnectKits/Azure_Sphere_SK_ADC_RTApp
*/


/*
 *  Some of the code in this file was copied from ST Micro.  Below is their required information.
 *
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2018 STMicroelectronics</center></h2>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in the
 *      documentation and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its
 *      contributors may be used to endorse or promote products derived from
 *      this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */
#include "i2c.h"
#include <errno.h>
#include <signal.h>
#include <stdbool.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>


// applibs_versions.h defines the API struct versions to use for applibs APIs.
#include "applibs_versions.h"

#include <applibs/log.h>
#include <applibs/i2c.h>

#include "mt3620_avnet_dev.h"
#include "deviceTwin.h"
#include "azure_iot_utilities.h"
#include "build_options.h"
#include "i2c.h"
#include "lsm6dso_reg.h"
#include "lps22hh_reg.h"
#include "ltc1695.h"
#include "real_core.h"
#include "bme680_sensor.h"
#include "web_tcp_server.h"

#include "smuart04l.h"

#include "adc.h"
#include "led.h"





/* Private variables ---------------------------------------------------------*/
static axis3bit16_t data_raw_acceleration;
static axis3bit16_t data_raw_angular_rate;
static axis3bit16_t raw_angular_rate_calibration;
static axis1bit32_t data_raw_pressure;
static axis1bit16_t data_raw_temperature;
static float acceleration_mg[3];
static float angular_rate_dps[3];
static float lsm6dsoTemperature_degC;
static float pressure_hPa;
static float lps22hhTemperature_degC;

static uint8_t whoamI, rst;
static int accelTimerFd = -1;
const uint8_t lsm6dsOAddress = LSM6DSO_ADDRESS;     // Addr = 0x6A
lsm6dso_ctx_t dev_ctx;
lps22hh_ctx_t pressure_ctx;

float altitude;

int iaq_class = 1;


uint8_t led_status = 0;

// Status variables
uint8_t lsm6dso_status = 1;
uint8_t lps22hh_status = 1;
//uint8_t RTCore_status = 1;

//Extern variables
int i2cFd = -1;
extern int epollFd;
extern volatile sig_atomic_t terminationRequired;

int appLedGpioFd = -1;

uint32_t wififreq = 0;

//Private functions

// Routines to read/write to the LSM6DSO device
static int32_t platform_write(int *fD, uint8_t reg, uint8_t *bufp, uint16_t len);
static int32_t platform_read(int *fD, uint8_t reg, uint8_t *bufp, uint16_t len);

// Routines to read/write to the LPS22HH device connected to the LSM6DSO sensor hub
static int32_t lsm6dso_write_lps22hh_cx(void* ctx, uint8_t reg, uint8_t* data, uint16_t len);
static int32_t lsm6dso_read_lps22hh_cx(void* ctx, uint8_t reg, uint8_t* data, uint16_t len);

/// <summary>
///     Sleep for delayTime ms
/// </summary>
void HAL_Delay(int delayTime) {
	struct timespec ts;
	ts.tv_sec = 0;
	ts.tv_nsec = delayTime * 10000;
	nanosleep(&ts, NULL);
}




void led_control() {
	uint8_t color[3];

	uint8_t level = 0;
	//PM status
	uint16_t max = smuart04l_getPM1();
	if (max < smuart04l_getPM2_5())
		max = smuart04l_getPM2_5();
	if (max < smuart04l_getPM10())
		max = smuart04l_getPM10();

	int pm_class = smuart04l_pm_to_class(max);
	

	uint16_t iaq = bme680_iaq;
	iaq = 120;
	if (iaq <= 50) {
		iaq_class = 0;
	}
	else if (iaq <= 100) {
		iaq_class = 1;
	}
	else if (iaq <= 150) {
		iaq_class = 2;
	}
	else if (iaq <= 200) {
		iaq_class = 3;
	}
	else if (iaq <= 250) {
		iaq_class = 4;
	}
	else if (iaq <= 350) {
		iaq_class = 5;
	}
	else {
		iaq_class = 6;
	}

		if (led_status) {
			
			switch (pm_class) {
			case 0:
				color[0] = 67;color[1] = 106;color[2] = 150;
				break;
			case 1:
				color[0] = 250;color[1] = 89;color[2] = 222;
				break;
			case 2:
				color[0] = 214;color[1] = 60;color[2] = 58;
				break;
			case 3:
				color[0] = 187;color[1] = 57;color[2] = 39;
				break;
			case 4:
				color[0] = 93;color[1] = 147;color[2] = 21;
				break;
			case 5:
				color[0] = 255;color[1] = 15;color[2] = 53;
				break;
			}
		}else{
			switch (iaq_class) {
			case 0:
				color[0] = 104;color[1] = 67;color[2] = 224;
				break;
			case 1:
				color[0] = 160;color[1] = 99;color[2] = 206;
				break;
			case 2:
				color[0] = 255;color[1] = 84;color[2] = 254;
				break;
			case 3:
				color[0] = 239;color[1] = 50;color[2] = 132;
				break;
			case 4:
				color[0] = 234;color[1] = 35;color[2] = 51;
				break;
			case 5:
				color[0] = 140;color[1] = 75;color[2] = 27;
				break;
			case 6:
				color[0] = 255;color[1] = 15;color[2] = 53;
				break;
			}
		}


		set_led(LED1MIXCOLOR, color, 3);
		uint32_t ledflash = led_status?LED_FASTFLASH:LED_ON;
		set_led(LED1FLASH, &ledflash, 4);
		uint8_t wifi  = wififreq > 0 ? wififreq > 5000 ? 80 : 20 : 0;
		set_led(LEDWIFI,&wifi , 1);
	
		
		led_status = led_status>0?0:1;

		
	
	
	

		ltc1695_auto(pm_class, iaq_class);


		
}

/// <summary>
///     Print latest data from on-board sensors.
/// </summary>
void AccelTimerEventHandler(EventData *eventData)
{
	

	uint8_t reg;
	lps22hh_reg_t lps22hhReg;

#if (defined(IOT_CENTRAL_APPLICATION) || defined(IOT_HUB_APPLICATION))
	static bool firstPass = true;
#endif
	// Consume the event.  If we don't do this we'll come right back 
	// to process the same event again
	if (ConsumeTimerFdEvent(accelTimerFd) != 0) {
		terminationRequired = true;
		return;
	}

	//bme680_sensor_get();

	

	// Read the sensors on the lsm6dso device

	//Read output only if new xl value is available
	lsm6dso_xl_flag_data_ready_get(&dev_ctx, &reg);
	if (reg)
	{
		// Read acceleration field data
		memset(data_raw_acceleration.u8bit, 0x00, 3 * sizeof(int16_t));
		lsm6dso_acceleration_raw_get(&dev_ctx, data_raw_acceleration.u8bit);

		acceleration_mg[0] = lsm6dso_from_fs4_to_mg(data_raw_acceleration.i16bit[0]);
		acceleration_mg[1] = lsm6dso_from_fs4_to_mg(data_raw_acceleration.i16bit[1]);
		acceleration_mg[2] = lsm6dso_from_fs4_to_mg(data_raw_acceleration.i16bit[2]);

		LogWebDebug("\nLSM6DSO: Acceleration [mg]  : %.4lf, %.4lf, %.4lf\n",
			acceleration_mg[0], acceleration_mg[1], acceleration_mg[2]);
	}

	lsm6dso_gy_flag_data_ready_get(&dev_ctx, &reg);
	if (reg)
	{
		// Read angular rate field data
		memset(data_raw_angular_rate.u8bit, 0x00, 3 * sizeof(int16_t));
		lsm6dso_angular_rate_raw_get(&dev_ctx, data_raw_angular_rate.u8bit);

		// Before we store the mdps values subtract the calibration data we captured at startup.
		angular_rate_dps[0] = (lsm6dso_from_fs2000_to_mdps(data_raw_angular_rate.i16bit[0] - raw_angular_rate_calibration.i16bit[0])) / 1000.0;
		angular_rate_dps[1] = (lsm6dso_from_fs2000_to_mdps(data_raw_angular_rate.i16bit[1] - raw_angular_rate_calibration.i16bit[1])) / 1000.0;
		angular_rate_dps[2] = (lsm6dso_from_fs2000_to_mdps(data_raw_angular_rate.i16bit[2] - raw_angular_rate_calibration.i16bit[2])) / 1000.0;

		LogWebDebug("LSM6DSO: Angular rate [dps] : %4.2f, %4.2f, %4.2f\r\n",
			angular_rate_dps[0], angular_rate_dps[1], angular_rate_dps[2]);

	

	}

	lsm6dso_temp_flag_data_ready_get(&dev_ctx, &reg);
	if (reg)
	{
		// Read temperature data
		memset(data_raw_temperature.u8bit, 0x00, sizeof(int16_t));
		lsm6dso_temperature_raw_get(&dev_ctx, data_raw_temperature.u8bit);
		lsm6dsoTemperature_degC = lsm6dso_from_lsb_to_celsius(data_raw_temperature.i16bit);

		LogWebDebug("LSM6DSO: Temperature1 [degC]: %.2f\r\n", lsm6dsoTemperature_degC);
	}

	// Read the sensors on the lsm6dso device

	lps22hh_read_reg(&pressure_ctx, LPS22HH_STATUS, (uint8_t *)&lps22hhReg, 1);

	//Read output only if new value is available

	if ((lps22hhReg.status.p_da == 1) && (lps22hhReg.status.t_da == 1))
	{
		memset(data_raw_pressure.u8bit, 0x00, sizeof(int32_t));
		lps22hh_pressure_raw_get(&pressure_ctx, data_raw_pressure.u8bit);

		pressure_hPa = lps22hh_from_lsb_to_hpa((uint32_t)data_raw_pressure.i32bit);

		memset(data_raw_temperature.u8bit, 0x00, sizeof(int16_t));
		lps22hh_temperature_raw_get(&pressure_ctx, data_raw_temperature.u8bit);
		lps22hhTemperature_degC = lps22hh_from_lsb_to_celsius(data_raw_temperature.i16bit);

		LogWebDebug("LPS22HH: Pressure     [hPa] : %.2f\r\n", pressure_hPa);
		LogWebDebug("LPS22HH: Temperature2 [degC]: %.2f\r\n", lps22hhTemperature_degC);
	}


	sensor_data.acceleration_mg[0] = acceleration_mg[0];
	sensor_data.acceleration_mg[1] = acceleration_mg[1];
	sensor_data.acceleration_mg[2] = acceleration_mg[2];
	sensor_data.angular_rate_dps[0] = angular_rate_dps[0];
	sensor_data.angular_rate_dps[1] = angular_rate_dps[1];
	sensor_data.angular_rate_dps[2] = angular_rate_dps[2];
	sensor_data.lsm6dsoTemperature_degC = lsm6dsoTemperature_degC;
	sensor_data.lps22hhpressure_hPa = pressure_hPa;
	sensor_data.lps22hhTemperature_degC = lps22hhTemperature_degC;

	/*
	The ALTITUDE value calculated is actually "Pressure Altitude". This lacks correction for temperature (and humidity)
	"pressure altitude" calculator located at: https://www.weather.gov/epz/wxcalc_pressurealtitude
	"pressure altitude" formula is defined at: https://www.weather.gov/media/epz/wxcalc/pressureAltitude.pdf
	 altitude in feet = 145366.45 * (1 - (hPa / 1013.25) ^ 0.190284) feet
	 altitude in meters = 145366.45 * 0.3048 * (1 - (hPa / 1013.25) ^ 0.190284) meters
	*/
	// weather.com formula
	//altitude = 44307.69396 * (1 - powf((atm / 1013.25), 0.190284));  // pressure altitude in meters
	// Bosch's formula
	altitude = 44330 * (1 - powf((pressure_hPa / sea), 1 / 5.255));  // pressure altitude in meters



	adc_request();
	LogWebDebug("ALSPT19: Ambient Light[Lux] : %.2f\r\n", light_sensor);

	
	//// OLED
	update_oled();

	LogWebDebug("PM id:%jd value 1,2.5,10:'%d %d %d'.\n", smuart04l_getId(), smuart04l_getPM1(), smuart04l_getPM2_5(), smuart04l_getPM10());

	//// check wifi status
	WifiConfig_ConnectedNetwork network;
	int result = WifiConfig_GetCurrentNetwork(&network);
	if (result < 0)
		wififreq = 0;
	else
		wififreq = network.frequencyMHz;


	led_control();

	

#if (defined(IOT_CENTRAL_APPLICATION) || defined(IOT_HUB_APPLICATION))

	// We've seen that the first read of the Accelerometer data is garbage.  If this is the first pass
	// reading data, don't report it to Azure.  Since we're graphing data in Azure, this data point
	// will skew the data.
	if (!firstPass) {

		// Allocate memory for a telemetry message to Azure
		char *pjsonBuffer = (char *)malloc(JSON_BUFFER_SIZE);
		if (pjsonBuffer == NULL) {
			LogWebDebug("ERROR: not enough memory to send telemetry");
		}

		snprintf(pjsonBuffer, JSON_BUFFER_SIZE, "{\"gX\":\"%.2lf\", \"gY\":\"%.2lf\", \"gZ\":\"%.2lf\", \"aX\": \"%.2f\", \"aY\": \"%.2f\", \"aZ\": \"%.2f\", \"pressure\": \"%.2f\", \"light_intensity\": \"%.2f\", \"altitude\": \"%.2f\", \"temp\": \"%.2f\", \"humidity\": \"%.2f\",\"iaq\": \"%d\", \"pm1\": \"%d\",\"pm2_5\": \"%d\",\"pm10\": \"%d\",\"fan\": \"%d\",\"fanspeed\": \"%d\",\"fanmodes\": \"%d\" }",
			angular_rate_dps[0], angular_rate_dps[1], angular_rate_dps[2], acceleration_mg[0], acceleration_mg[1], acceleration_mg[2], bme680_pressure/100.0f, light_sensor, altitude, bme680_temperature,bme680_humidity,(int)bme680_iaq,smuart04l_getPM1(),smuart04l_getPM2_5(),smuart04l_getPM10(),ltc1695_value,ltc1695_manual_value,ltc1695_mode);

		
		//snprintf(pjsonBuffer, JSON_BUFFER_SIZE, "{\"gX\":\"%.2lf\", \"gY\":\"%.2lf\", \"gZ\":\"%.2lf\", \"aX\": \"%.2f\", \"aY\": \"%.2f\", \"aZ\": \"%.2f\", \"pressure\": \"%.2f\", \"light_intensity\": \"%.2f\", \"altitude\": \"%.2f\", \"temp\": \"%.2f\",  \"rssi\": \"%d\"}",
		//	angular_rate_dps[0], angular_rate_dps[1], angular_rate_dps[2], acceleration_mg[0], acceleration_mg[1], acceleration_mg[2], pressure_hPa, light_sensor, altitude, lsm6dsoTemperature_degC, network_data.rssi);

		LogWebDebug("\n[Info] Sending telemetry: %s\n", pjsonBuffer);
		AzureIoT_SendMessage(pjsonBuffer);
		free(pjsonBuffer);

}

	firstPass = false;

#endif 

}

/// <summary>
///     Initializes the I2C interface.
/// </summary>
/// <returns>0 on success, or -1 on failure</returns>
int initI2c(void) {


	
	

	// Begin MT3620 I2C init 

	i2cFd = I2CMaster_Open(MT3620_RDB_HEADER4_ISU2_I2C);
	if (i2cFd < 0) {
		LogWebDebug("ERROR: I2CMaster_Open: errno=%d (%s)\n", errno, strerror(errno));
		return -1;
	}

	int result = I2CMaster_SetBusSpeed(i2cFd, I2C_BUS_SPEED_STANDARD);
	if (result != 0) {
		LogWebDebug("ERROR: I2CMaster_SetBusSpeed: errno=%d (%s)\n", errno, strerror(errno));
		return -1;
	}

	result = I2CMaster_SetTimeout(i2cFd, 100);
	if (result != 0) {
		LogWebDebug("ERROR: I2CMaster_SetTimeout: errno=%d (%s)\n", errno, strerror(errno));
		return -1;
	}

	
	

	// Start OLED
	if (oled_init())
	{
		LogWebDebug("OLED not found!\n");
	}
	else
	{
		LogWebDebug("OLED found!\n");
	}

	// Draw AVNET logo
	//oled_draw_logo();
	oled_i2c_bus_status(0);

	

	// Start lsm6dso specific init

	// Initialize lsm6dso mems driver interface
	dev_ctx.write_reg = platform_write;
	dev_ctx.read_reg = platform_read;
	dev_ctx.handle = &i2cFd;

	// Check device ID
	lsm6dso_device_id_get(&dev_ctx, &whoamI);
	if (whoamI != LSM6DSO_ID) {
		LogWebDebug("LSM6DSO not found!\n");

		// OLED update
		lsm6dso_status = 1;
		oled_i2c_bus_status(1);
		return -1;
	}
	else {
		LogWebDebug("LSM6DSO Found!\n");

		// OLED update
		lsm6dso_status = 0;
		oled_i2c_bus_status(1);
	}
		
	 // Restore default configuration
	lsm6dso_reset_set(&dev_ctx, PROPERTY_ENABLE);
	do {
		lsm6dso_reset_get(&dev_ctx, &rst);
	} while (rst);

	 // Disable I3C interface
	lsm6dso_i3c_disable_set(&dev_ctx, LSM6DSO_I3C_DISABLE);

	// Enable Block Data Update
	lsm6dso_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);

	 // Set Output Data Rate
	lsm6dso_xl_data_rate_set(&dev_ctx, LSM6DSO_XL_ODR_12Hz5);
	lsm6dso_gy_data_rate_set(&dev_ctx, LSM6DSO_GY_ODR_12Hz5);

	 // Set full scale
	lsm6dso_xl_full_scale_set(&dev_ctx, LSM6DSO_4g);
	lsm6dso_gy_full_scale_set(&dev_ctx, LSM6DSO_2000dps);

	 // Configure filtering chain(No aux interface)
	// Accelerometer - LPF1 + LPF2 path	
	lsm6dso_xl_hp_path_on_out_set(&dev_ctx, LSM6DSO_LP_ODR_DIV_100);
	lsm6dso_xl_filter_lp2_set(&dev_ctx, PROPERTY_ENABLE);

	// lps22hh specific init

	// Initialize lps22hh mems driver interface
	pressure_ctx.read_reg = lsm6dso_read_lps22hh_cx;
	pressure_ctx.write_reg = lsm6dso_write_lps22hh_cx;
	pressure_ctx.handle = &i2cFd;

	bool lps22hhDetected = false;
	int failCount = 10;

	while (!lps22hhDetected) {
		
		// Enable pull up on master I2C interface.
		lsm6dso_sh_pin_mode_set(&dev_ctx, LSM6DSO_INTERNAL_PULL_UP);

		// Check if LPS22HH is connected to Sensor Hub
		lps22hh_device_id_get(&pressure_ctx, &whoamI);
		if (whoamI != LPS22HH_ID) {
			LogWebDebug("LPS22HH not found!\n");
			
			// OLED update
			lps22hh_status = 1;
			oled_i2c_bus_status(2);

		}
		else {
			lps22hhDetected = true;
			LogWebDebug("LPS22HH Found!\n");

			// OLED update
			lps22hh_status = 0;
			oled_i2c_bus_status(2);
		}

		// Restore the default configuration
		lps22hh_reset_set(&pressure_ctx, PROPERTY_ENABLE);
		do {
			lps22hh_reset_get(&pressure_ctx, &rst);
		} while (rst);

		// Enable Block Data Update
		lps22hh_block_data_update_set(&pressure_ctx, PROPERTY_ENABLE);

		//Set Output Data Rate
		lps22hh_data_rate_set(&pressure_ctx, LPS22HH_10_Hz_LOW_NOISE);

		// If we failed to detect the lps22hh device, then pause before trying again.
		if (!lps22hhDetected) {
			HAL_Delay(100);
		}

		if (failCount-- == 0) {
			LogWebDebug("Failed to read LSM22HH device ID, exiting\n");
			return -1;
		}


	
	}

	

	
	// Read the raw angular rate data from the device to use as offsets.  We're making the assumption that the device
	// is stationary.

	uint8_t reg;

	
	LogWebDebug("LSM6DSO: Calibrating angular rate . . .\n"); 
	LogWebDebug("LSM6DSO: Please make sure the device is stationary.\n");
	
	do {
		// Read the calibration values
		lsm6dso_gy_flag_data_ready_get(&dev_ctx, &reg);
		if (reg)
		{
			// Read angular rate field data to use for calibration offsets
			memset(data_raw_angular_rate.u8bit, 0x00, 3 * sizeof(int16_t));
			lsm6dso_angular_rate_raw_get(&dev_ctx, raw_angular_rate_calibration.u8bit);
		}

		// Read the angular data rate again and verify that after applying the calibration, we have 0 angular rate in all directions
		
		lsm6dso_gy_flag_data_ready_get(&dev_ctx, &reg);
		
		if (reg)
		{
			// Read angular rate field data
			memset(data_raw_angular_rate.u8bit, 0x00, 3 * sizeof(int16_t));
			lsm6dso_angular_rate_raw_get(&dev_ctx, data_raw_angular_rate.u8bit);

			// Before we store the mdps values subtract the calibration data we captured at startup.
			angular_rate_dps[0] = lsm6dso_from_fs2000_to_mdps(data_raw_angular_rate.i16bit[0] - raw_angular_rate_calibration.i16bit[0]);
			angular_rate_dps[1] = lsm6dso_from_fs2000_to_mdps(data_raw_angular_rate.i16bit[1] - raw_angular_rate_calibration.i16bit[1]);
			angular_rate_dps[2] = lsm6dso_from_fs2000_to_mdps(data_raw_angular_rate.i16bit[2] - raw_angular_rate_calibration.i16bit[2]);
		}

	// If the angular values after applying the offset are not zero, then do it again!
	} while (angular_rate_dps[0] == angular_rate_dps[1] == angular_rate_dps[2] == 0.0);

	LogWebDebug("LSM6DSO: Calibrating angular rate complete!\n");



	ltc1695_init();


	bme680_sensor_init();


	// Init the epoll interface to periodically run the AccelTimerEventHandler routine where we read the sensors

	// Define the period in the build_options.h file
	struct timespec accelReadPeriod = { .tv_sec = ACCEL_READ_PERIOD_SECONDS,.tv_nsec = ACCEL_READ_PERIOD_NANO_SECONDS };
	// event handler data structures. Only the event handler field needs to be populated.
	static EventData accelEventData = { .eventHandler = &AccelTimerEventHandler };
	accelTimerFd = CreateTimerFdAndAddToEpoll(epollFd, &accelReadPeriod, &accelEventData, EPOLLIN);
	if (accelTimerFd < 0) {
		return -1;
	}

	return 0;
}

/// <summary>
///     Closes the I2C interface File Descriptors.
/// </summary>
void closeI2c(void) {

	CloseFdAndPrintError(i2cFd, "i2c");
	CloseFdAndPrintError(accelTimerFd, "accelTimer");
}

/// <summary>
///     Writes data to the lsm6dso i2c device
/// </summary>
/// <returns>0</returns>

static int32_t platform_write(int *fD, uint8_t reg, uint8_t *bufp,
	uint16_t len)
{

#ifdef ENABLE_READ_WRITE_DEBUG
	LogWebDebug("platform_write()\n");
	LogWebDebug("reg: %0x\n", reg);
	LogWebDebug("len: %0x\n", len);
	LogWebDebug("bufp contents: ");
	for (int i = 0; i < len; i++) {

		LogWebDebug("%0x: ", bufp[i]);
	}
	LogWebDebug("\n");
#endif 

	// Construct a new command buffer that contains the register to write to, then the data to write
	uint8_t cmdBuffer[len + 1];
	cmdBuffer[0] = reg;
	for (int i = 0; i < len; i++) {
		cmdBuffer[i + 1] = bufp[i];
	}

#ifdef ENABLE_READ_WRITE_DEBUG
	LogWebDebug("cmdBuffer contents: ");
	for (int i = 0; i < len + 1; i++) {

		LogWebDebug("%0x: ", cmdBuffer[i]);
	}
	LogWebDebug("\n");
#endif
	



	 //Write the data to the device
	int32_t retVal = I2CMaster_Write(i2cFd, lsm6dsOAddress, cmdBuffer, (size_t)len + 1);
	if (retVal < 0) {
		LogWebDebug("ERROR: platform_write: errno=%d (%s)\n", errno, strerror(errno));
		return -1;
	}
#ifdef ENABLE_READ_WRITE_DEBUG
	LogWebDebug("Wrote %d bytes to device.\n\n", retVal);
#endif
	return 0;
}

/// <summary>
///     Reads generic device register from the i2c interface
/// </summary>
/// <returns>0</returns>

/*
 * @brief  Read generic device register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to read
 * @param  bufp      pointer to buffer that store the data read
 * @param  len       number of consecutive register to read
 *
 */
static int32_t platform_read(int *fD, uint8_t reg, uint8_t *bufp,
	uint16_t len)
{

#ifdef ENABLE_READ_WRITE_DEBUG
	LogWebDebug("platform_read()\n");
	LogWebDebug("reg: %0x\n", reg);
	LogWebDebug("len: %d\n", len);
;
#endif
uint8_t* data_inc = bufp;
uint8_t reg_addr_inc = reg;








	// Set the register address to read
	int32_t retVal = I2CMaster_Write(i2cFd, lsm6dsOAddress, &reg, 1);
	if (retVal < 0) {
		LogWebDebug("ERROR: platform_read(write step): errno=%d (%s)\n", errno, strerror(errno));
		return -1;
	}

	// Read the data into the provided buffer
	
	retVal = I2CMaster_Read(i2cFd, lsm6dsOAddress, bufp, len);
	if (retVal < 0) {
		LogWebDebug("ERROR: platform_read(read step): errno=%d (%s)\n", errno, strerror(errno));
		return -1;
	}
	
	
#ifdef ENABLE_READ_WRITE_DEBUG
	LogWebDebug("Read returned: ");
	for (int i = 0; i < len; i++) {
		LogWebDebug("%0x: ", bufp[i]);
	}
	LogWebDebug("\n\n");
#endif 	   

	return 0;
}
/*
 * @brief  Write lsm2mdl device register (used by configuration functions)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to write
 * @param  bufp      pointer to data to write in register reg
 * @param  len       number of consecutive register to write
 *
 */
static int32_t lsm6dso_write_lps22hh_cx(void* ctx, uint8_t reg, uint8_t* data,
	uint16_t len)
{
	axis3bit16_t data_raw_acceleration;
	int32_t ret;
	uint8_t drdy;
	lsm6dso_status_master_t master_status;
	lsm6dso_sh_cfg_write_t sh_cfg_write;

	// Configure Sensor Hub to write to the LPS22HH, and send the write data
	sh_cfg_write.slv0_add = (LPS22HH_I2C_ADD_L & 0xFEU) >> 1; // 7bit I2C address
	sh_cfg_write.slv0_subadd = reg,
	sh_cfg_write.slv0_data = *data,
	ret = lsm6dso_sh_cfg_write(&dev_ctx, &sh_cfg_write);

	/* Disable accelerometer. */
	lsm6dso_xl_data_rate_set(&dev_ctx, LSM6DSO_XL_ODR_OFF);

	/* Enable I2C Master. */
	lsm6dso_sh_master_set(&dev_ctx, PROPERTY_ENABLE);

	/* Enable accelerometer to trigger Sensor Hub operation. */
	lsm6dso_xl_data_rate_set(&dev_ctx, LSM6DSO_XL_ODR_104Hz);

	/* Wait Sensor Hub operation flag set. */
	lsm6dso_acceleration_raw_get(&dev_ctx, data_raw_acceleration.u8bit);
	do
	{
		HAL_Delay(20);
		lsm6dso_xl_flag_data_ready_get(&dev_ctx, &drdy);
	} while (!drdy);

	do
	{
		HAL_Delay(20);
		lsm6dso_sh_status_get(&dev_ctx, &master_status);
	} while (!master_status.sens_hub_endop);

	/* Disable I2C master and XL (trigger). */
	lsm6dso_sh_master_set(&dev_ctx, PROPERTY_DISABLE);
	lsm6dso_xl_data_rate_set(&dev_ctx, LSM6DSO_XL_ODR_OFF);

	return ret;
}

/*
 * @brief  Read lsm2mdl device register (used by configuration functions)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to read
 * @param  bufp      pointer to buffer that store the data read
 * @param  len       number of consecutive register to read
 *
 */
static int32_t lsm6dso_read_lps22hh_cx(void* ctx, uint8_t reg, uint8_t* data, uint16_t len)
{
	lsm6dso_sh_cfg_read_t sh_cfg_read;
	axis3bit16_t data_raw_acceleration;
	int32_t ret;
	uint8_t drdy;
	lsm6dso_status_master_t master_status;

	/* Disable accelerometer. */
	lsm6dso_xl_data_rate_set(&dev_ctx, LSM6DSO_XL_ODR_OFF);
	
	// For each byte we need to read from the lps22hh
	for (int i = 0; i < len; i++) {

		/* Configure Sensor Hub to read LPS22HH. */
		sh_cfg_read.slv_add = (LPS22HH_I2C_ADD_L &0xFEU) >> 1; /* 7bit I2C address */
		sh_cfg_read.slv_subadd = reg+i;
		sh_cfg_read.slv_len = 1;

		// Call the command to read the data from the sensor hub.
		// This data will be read from the device connected to the 
		// sensor hub, and saved into a register for us to read.
		ret = lsm6dso_sh_slv0_cfg_read(&dev_ctx, &sh_cfg_read);
		lsm6dso_sh_slave_connected_set(&dev_ctx, LSM6DSO_SLV_0);

		/* Enable I2C Master and I2C master. */
		lsm6dso_sh_master_set(&dev_ctx, PROPERTY_ENABLE);

		/* Enable accelerometer to trigger Sensor Hub operation. */
		lsm6dso_xl_data_rate_set(&dev_ctx, LSM6DSO_XL_ODR_104Hz);

		/* Wait Sensor Hub operation flag set. */
		lsm6dso_acceleration_raw_get(&dev_ctx, data_raw_acceleration.u8bit);
		do {
			HAL_Delay(20);
			lsm6dso_xl_flag_data_ready_get(&dev_ctx, &drdy);
		} while (!drdy);

		do {
			HAL_Delay(20);
			lsm6dso_sh_status_get(&dev_ctx, &master_status);
		} while (!master_status.sens_hub_endop);

		/* Disable I2C master and XL(trigger). */
		lsm6dso_sh_master_set(&dev_ctx, PROPERTY_DISABLE);
		lsm6dso_xl_data_rate_set(&dev_ctx, LSM6DSO_XL_ODR_OFF);

		// Read the data from the device.  The call below reads
		// all 18 sensor hub data.  We just need the data from 
		// sensor hub 1, so copy that into our data array.
		uint8_t buffer[18];
		lsm6dso_sh_read_data_raw_get(&dev_ctx, buffer);
		data[i] = buffer[0];

#ifdef ENABLE_READ_WRITE_DEBUG
		LogWebDebug("Read %d bytes: ", len);
		for (int i = 0; i < len; i++) {
			LogWebDebug("[%0x] ", data[i]);
		}
		LogWebDebug("\n", len);
#endif 
	}

	/* Re-enable accelerometer */
	lsm6dso_xl_data_rate_set(&dev_ctx, LSM6DSO_XL_ODR_104Hz);

	return ret;
}

