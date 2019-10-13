/*   Copyright (c) SC Lee. All rights reserved.
   Licensed under the GNU GPLv3 License.

   Bare Metal program for BSEC

   Some of the code copies from Microsoft sample example https://github.com/Azure/azure-sphere-samples 
  sCopyright (c) Microsoft Corporation. All rights reserved.
   Licensed under the MIT License.

   Some of the code and library from Borch Copyright (C) 2015, 2016, 2017 Robert Bosch. All Rights Reserved. 



*/




#include <ctype.h>
#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>
#include <errno.h>
#include <math.h>
#include <float.h>
#include "mt3620-baremetal.h"
#include "mt3620-intercore.h"
#include "mt3620-inter-message.h"
#include "mt3620-uart-poll.h"
//#include "mt3620-adc.h"
#include "mt3620-gpio.h"
#include "mt3620-timer.h"
#include "mt3620-timer-poll.h"
#include "bsec_interface.h"
#include "bsec_datatypes.h"
#include "bsec_serialized_configurations_iaq.h"

#define NUM_USED_OUTPUTS 10
#define TEMPOFFSET 1.0f

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

#define BME680_SAVE_STATE_INTERVAL  1200



extern uint32_t StackTop; // &StackTop == end of TCM0

static _Noreturn void DefaultExceptionHandler(void);

static void PrintBytes(const uint8_t *buf, int start, int end);
static void PrintGuid(const uint8_t *guid);

static const int led1RedGpio = 8;
static const int ledFlashMs = 50;
_Bool ledBlinking = false;



static _Noreturn void RTCoreMain(void);

//static void HandleBlinkTimerIrq(void);

//static void blink(void);



float cosf(float x) { return cos(x); }
float sinf(float x) { return sin(x); }
float sqrtf(float x) { return sqrt(x); }
float log10f(float x) { return log10(x); }
float logf(float x) { return log(x); }
float expf(float x) { return exp(x); }
float fabsf(float x) { return fabs(x); }
float fminf(float x,float y) { return fmin(x,y); }
float fmaxf(float x, float y) { return fmax(x, y); }
float powf(float x, float y) { return pow(x, y); }
float floorf(float x) { return floor(x); }
float roundf(float x) { return round(x); }
float ceilf(float x) { return ceil(x); }

// ARM DDI0403E.d SB1.5.2-3
// From SB1.5.3, "The Vector table must be naturally aligned to a power of two whose alignment
// value is greater than or equal to (Number of Exceptions supported x 4), with a minimum alignment
// of 128 bytes.". The array is aligned in linker.ld, using the dedicated section ".vector_table".

// The exception vector table contains a stack pointer, 15 exception handlers, and an entry for
// each interrupt.
#define INTERRUPT_COUNT 100 // from datasheet
#define EXCEPTION_COUNT (16 + INTERRUPT_COUNT)
#define INT_TO_EXC(i_) (16 + (i_))
static const uintptr_t ExceptionVectorTable[EXCEPTION_COUNT]
    __attribute__((section(".vector_table"))) __attribute__((used)) = {
        [0] = (uintptr_t)&StackTop,                // Main Stack Pointer (MSP)
        [1] = (uintptr_t)RTCoreMain,               // Reset
        [2] = (uintptr_t)DefaultExceptionHandler,  // NMI
        [3] = (uintptr_t)DefaultExceptionHandler,  // HardFault
        [4] = (uintptr_t)DefaultExceptionHandler,  // MPU Fault
        [5] = (uintptr_t)DefaultExceptionHandler,  // Bus Fault
        [6] = (uintptr_t)DefaultExceptionHandler,  // Usage Fault
        [11] = (uintptr_t)DefaultExceptionHandler, // SVCall
        [12] = (uintptr_t)DefaultExceptionHandler, // Debug monitor
        [14] = (uintptr_t)DefaultExceptionHandler, // PendSV
        [15] = (uintptr_t)DefaultExceptionHandler, // SysTick
		[INT_TO_EXC(0)] = (uintptr_t)DefaultExceptionHandler,
		[INT_TO_EXC(1)] = (uintptr_t)Gpt_HandleIrq1,
		[INT_TO_EXC(2)... INT_TO_EXC(INTERRUPT_COUNT - 1)] = (uintptr_t)DefaultExceptionHandler };

static _Noreturn void DefaultExceptionHandler(void)
{
    for (;;) {
        // empty.
    }
}

static void PrintBytes(const uint8_t *buf, int start, int end)
{
    int step = (end >= start) ? +1 : -1;

    for (/* nop */; start != end; start += step) {
        Uart_WriteHexBytePoll(buf[start]);
    }
    Uart_WriteHexBytePoll(buf[end]);
}

static void PrintGuid(const uint8_t *guid)
{
    PrintBytes(guid, 3, 0); // 4-byte little-endian word
    Uart_WriteStringPoll("-");
    PrintBytes(guid, 5, 4); // 2-byte little-endian half
    Uart_WriteStringPoll("-");
    PrintBytes(guid, 7, 6); // 2-byte little-endian half
    Uart_WriteStringPoll("-");
    PrintBytes(guid, 8, 9); // 2 bytes
    Uart_WriteStringPoll("-");
    PrintBytes(guid, 10, 15); // 6 bytes
}



/*!
 * @brief        Virtual sensor subscription
 *               Please call this function before processing of data using bsec_do_steps function
 *
 * @param[in]    sample_rate         mode to be used (either BSEC_SAMPLE_RATE_ULP or BSEC_SAMPLE_RATE_LP)
 *
 * @return       subscription result, zero when successful
 */
static bsec_library_return_t bme680_bsec_update_subscription(float sample_rate)
{
	bsec_sensor_configuration_t requested_virtual_sensors[NUM_USED_OUTPUTS];
	uint8_t n_requested_virtual_sensors = NUM_USED_OUTPUTS;

	bsec_sensor_configuration_t required_sensor_settings[BSEC_MAX_PHYSICAL_SENSOR];
	uint8_t n_required_sensor_settings = BSEC_MAX_PHYSICAL_SENSOR;

	bsec_library_return_t status = BSEC_OK;

	/* note: Virtual sensors as desired to be added here */
	requested_virtual_sensors[0].sensor_id = BSEC_OUTPUT_IAQ;
	requested_virtual_sensors[0].sample_rate = sample_rate;
	requested_virtual_sensors[1].sensor_id = BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE;
	requested_virtual_sensors[1].sample_rate = sample_rate;
	requested_virtual_sensors[2].sensor_id = BSEC_OUTPUT_RAW_PRESSURE;
	requested_virtual_sensors[2].sample_rate = sample_rate;
	requested_virtual_sensors[3].sensor_id = BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY;
	requested_virtual_sensors[3].sample_rate = sample_rate;
	requested_virtual_sensors[4].sensor_id = BSEC_OUTPUT_RAW_GAS;
	requested_virtual_sensors[4].sample_rate = sample_rate;
	requested_virtual_sensors[5].sensor_id = BSEC_OUTPUT_RAW_TEMPERATURE;
	requested_virtual_sensors[5].sample_rate = sample_rate;
	requested_virtual_sensors[6].sensor_id = BSEC_OUTPUT_RAW_HUMIDITY;
	requested_virtual_sensors[6].sample_rate = sample_rate;
	requested_virtual_sensors[7].sensor_id = BSEC_OUTPUT_STATIC_IAQ;
	requested_virtual_sensors[7].sample_rate = sample_rate;
	requested_virtual_sensors[8].sensor_id = BSEC_OUTPUT_CO2_EQUIVALENT;
	requested_virtual_sensors[8].sample_rate = sample_rate;
	requested_virtual_sensors[9].sensor_id = BSEC_OUTPUT_BREATH_VOC_EQUIVALENT;
	requested_virtual_sensors[9].sample_rate = sample_rate;

	/* Call bsec_update_subscription() to enable/disable the requested virtual sensors */
	status = bsec_update_subscription(requested_virtual_sensors, n_requested_virtual_sensors, required_sensor_settings,
		&n_required_sensor_settings);

	return status;
}

static void HandleBlinkTimerIrq(void) {
	Mt3620_Gpio_Write(led1RedGpio, true);
	ledBlinking = false;
	//Gpt_LaunchTimerMs(TimerGpt0, 1000, HandleBlinkTimerIrq);
}
/*
static void blink(void) {

	if (ledBlinking)
		return;

	


	Gpt_Init();

	Mt3620_Gpio_Write(led1RedGpio, false);
	ledBlinking = true;
	Gpt_LaunchTimerMs(TimerGpt0, ledFlashMs, HandleBlinkTimerIrq);
}*/
/*!
 * @brief        Message send back to high level programe 
 *               
 *
 * @param[in]    output         bsec output array
 * @param[in]    output         bsec setting
 * @param[out]    message       message send back to high level programe
 * @return        result, zero when successful
 */



static uint8_t output_message( bsec_output_t* output, bsec_bme_settings_t* setting, uint8_t* message) {
	for (uint8_t i = 0;i < NUM_USED_OUTPUTS;i++) {
		uint8_t offset = 0;
		switch (output[i].sensor_id) {
		case BSEC_OUTPUT_IAQ:
			offset = BME680_VIR_SENSOR_IAQ;
			break;
		case BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE:
			offset = BME680_VIR_SENSOR_TEMP;
			break;
		case BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY:
			offset = BME680_VIR_SENSOR_HUMIDITY;
			break;
		case BSEC_OUTPUT_RAW_PRESSURE:
			offset = BME680_VIR_SENSOR_PRESSURE;
			break;
		case BSEC_OUTPUT_RAW_GAS:
			offset = BME680_VIR_SENSOR_GAS;
			break;
		case BSEC_OUTPUT_CO2_EQUIVALENT:
			offset = BME680_VIR_SENSOR_ECO2;
			break;
		case BSEC_OUTPUT_BREATH_VOC_EQUIVALENT:
			offset = BME680_VIR_SENSOR_EVOC;
			break;
		case BSEC_OUTPUT_STATIC_IAQ:
			offset = BME680_VIR_SENSOR_STATIC_IAQ;
			break;
		case BSEC_OUTPUT_RAW_TEMPERATURE:
			offset = BME680_VIR_SENSOR_RAW_TEMP;
			break;
		case BSEC_OUTPUT_RAW_HUMIDITY:
			offset = BME680_VIR_SENSOR_RAW_HUMIDITY;
			break;
		}

		memcpy(message + offset * 5, &output[i].signal, 4);
		message[offset * 5 + 4] = output[i].accuracy;

	}

	memcpy(message + 5 * NUM_USED_OUTPUTS, setting, 21);
	
	return 0;

}

static _Noreturn void RTCoreMain(void)
{
	union Analog_data
	{
		uint32_t u32;
		uint8_t u8[4];
	} analog_data;

	union long_data
	{
		int64_t i64;
		uint8_t u8[8];
	} long_data;


	// SCB->VTOR = ExceptionVectorTable
	WriteReg32(SCB_BASE, 0x08, (uint32_t)ExceptionVectorTable);

	//Uart_Init();
	Uart_WriteStringPoll("--------------------------------\r\n");
	Uart_WriteStringPoll("IntercoreCommsADC_RTApp_MT3620_BareMetal\r\n");
	Uart_WriteStringPoll("App built on: " __DATE__ ", " __TIME__ "\r\n");



	init_inter_message();
	//// ADC Communication
	//EnableAdc();






	// Block includes led1RedGpio, GPIO8.
//	static const GpioBlock pwm2 = {
//		.baseAddr = 0x38030000,.type = GpioBlock_PWM,.firstPin = 8,.pinCount = 1 };

//	Mt3620_Gpio_AddBlock(&pwm2);



//	Mt3620_Gpio_ConfigurePinForOutput(led1RedGpio);


	//BSEC Init

	bsec_version_t  version;
	bsec_get_version(&version);
	uint8_t work_buffer[BSEC_MAX_PROPERTY_BLOB_SIZE];

	bsec_library_return_t ret = bsec_init();

	if (ret != BSEC_OK)
		return;

	// Load BSEC config data
	ret = bsec_set_configuration(bsec_config_iaq, BSEC_MAX_PROPERTY_BLOB_SIZE, work_buffer,
		BSEC_MAX_PROPERTY_BLOB_SIZE);

	if (ret != BSEC_OK)
		return;

	// Update virtaul subscription
	ret = bme680_bsec_update_subscription(BSEC_SAMPLE_RATE_LP);

	if (ret != BSEC_OK)
		return;




	interLog("BSEC version: %d.%d.%d.%d", version.major, version.minor, version.major_bugfix, version.minor_bugfix);


	uint32_t bme680_samplecount = 0;





	for (;;)
	{
		size_t datasize = INTER_BUFF_SIZE;
		uint8_t type = 0;
		uint8_t data[datasize];
		uint8_t buf[4];
		uint8_t j;
		uint32_t mV;

		
	

		int8_t result = dequeue(&type, &data, &datasize);
		if (result || !datasize)
			continue;






		if (type == BME680_RECV) {




			for (size_t i = 0; i < 8; i++)
			{

				long_data.u8[i] = data[i];
			}

			int64_t timestamp = long_data.i64;

			uint16_t temp = data[9] << 8;
			temp |= data[8];


			for (size_t i = 0; i < 4; i++)
			{

				analog_data.u8[i] = data[i + 10];
			}
			uint32_t pressure = analog_data.u32;

			for (size_t i = 0; i < 4; i++)
			{

				analog_data.u8[i] = data[i + 14];
			}
			uint32_t humidity = analog_data.u32;



			for (size_t i = 0; i < 4; i++)
			{

				analog_data.u8[i] = data[i + 18];
			}
			uint32_t gas_resistance = analog_data.u32;

			//Pack sensor data to BSEC input structure 
			bsec_input_t inputs[5];
			bsec_output_t outputs[BSEC_NUMBER_OUTPUTS];
			uint8_t num_bsec_inputs = 0;


			//pressure sensor 
			inputs[num_bsec_inputs].sensor_id = BSEC_INPUT_PRESSURE;
			inputs[num_bsec_inputs].signal = pressure;
			inputs[num_bsec_inputs].time_stamp = timestamp;
			++num_bsec_inputs;

			//temperture sensor
			inputs[num_bsec_inputs].sensor_id = BSEC_INPUT_TEMPERATURE;
			inputs[num_bsec_inputs].signal = temp / 100.0f;
			inputs[num_bsec_inputs].time_stamp = timestamp;
			++num_bsec_inputs;

			//temperture offset
			inputs[num_bsec_inputs].sensor_id = BSEC_INPUT_HEATSOURCE;
			inputs[num_bsec_inputs].signal = TEMPOFFSET;
			inputs[num_bsec_inputs].time_stamp = timestamp;
			++num_bsec_inputs;

			//humidity sensor
			inputs[num_bsec_inputs].sensor_id = BSEC_INPUT_HUMIDITY;
			inputs[num_bsec_inputs].signal = humidity / 1000.0f;
			inputs[num_bsec_inputs].time_stamp = timestamp;
			++num_bsec_inputs;

			// gas resistance
			if (gas_resistance > 0) {
				inputs[num_bsec_inputs].sensor_id = BSEC_INPUT_GASRESISTOR;
				inputs[num_bsec_inputs].signal = gas_resistance;
				inputs[num_bsec_inputs].time_stamp = timestamp;
				++num_bsec_inputs;
			}
			uint8_t num_bsec_outputs = BSEC_NUMBER_OUTPUTS;

			// Call BSEC 
			ret = bsec_do_steps(inputs, num_bsec_inputs, outputs, &num_bsec_outputs);

			if (ret != BSEC_OK)
				continue;

			// Get next messurement settings

			bsec_bme_settings_t sensor_settings;
			ret = bsec_sensor_control(timestamp, &sensor_settings);

			//BSEC_W_SC_CALL_TIMING_VIOLATION Warning from BSEC because of timestamp wrong, 
			//it normal when the system reboot or sync of time, bypass
		
			if (ret != BSEC_OK && ret!=BSEC_W_SC_CALL_TIMING_VIOLATION)
				continue;

			uint8_t sendback[21 + NUM_USED_OUTPUTS * 5];

			ret = output_message(outputs, &sensor_settings, sendback);

			// Send data back to Main Core
			enqueue(BME680_SEND, sendback, 21 + NUM_USED_OUTPUTS * 5);

			// Send BSEC status to Main Core for save 
			if (++bme680_samplecount % BME680_SAVE_STATE_INTERVAL == 0) {
				uint8_t serialized_state[BSEC_MAX_STATE_BLOB_SIZE];
				uint8_t work_buffer_state[BSEC_MAX_STATE_BLOB_SIZE];
				ret = bsec_get_state(0, serialized_state, BSEC_MAX_STATE_BLOB_SIZE,
					work_buffer_state, BSEC_MAX_STATE_BLOB_SIZE, BSEC_MAX_STATE_BLOB_SIZE);
				if (ret == BSEC_OK)
					enqueue(BME680_STATE_SEND, serialized_state, BSEC_MAX_STATE_BLOB_SIZE);

			}

		}
		else if (type == ADC_LIGHT) {
		// ADC real core, not use
		continue;
			// Read ADC channel 0
			analog_data.u32 = ReadAdc(0);

			mV = (analog_data.u32 * 2500) / 0xFFF;


			j = 0;

		

			for (size_t i = 0; i < 4; i++)
			{
				// Put ADC data to buffer
				buf[i] = analog_data.u8[j++];
			}


			sendADC(buf);

		}
		else if (type == BME680_STATE_RECV) {
			
			//Received BME680 status from Main Core after loose power restart	

			uint8_t work_buffer_state[BSEC_MAX_STATE_BLOB_SIZE];
			ret = bsec_set_state(data, BSEC_MAX_STATE_BLOB_SIZE,
				work_buffer_state, BSEC_MAX_STATE_BLOB_SIZE);
			if (ret != BSEC_OK)
				continue;
		}
	}


}


