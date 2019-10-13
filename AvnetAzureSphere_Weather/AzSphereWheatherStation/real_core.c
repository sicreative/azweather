
/* Copyright (c) SC Lee. All rights reserved.
   Licensed under the GNU GPLv3 License.

   For Inter-communication of Real Core (M4)

*/

#include "real_core.h"
#include "web_tcp_server.h"
#include "led.h"

extern int epollFd;
extern volatile sig_atomic_t terminationRequired;

static EventData timerEventData = { .eventHandler = &TimerEventHandler };
static EventData socketEventData = { .eventHandler = &SocketEventHandler };
static EventData ledSocketEventData = { .eventHandler = &LedSocketEventHandler };

static int timerFd = -1;
static int ledsockFd = -1;
static int sockFd = -1;
uint8_t RTCore_status = 0;
uint8_t LedRTCore_status = 0;



static void LedSocketEventHandler(EventData* eventData) {

}

/// <summary>
///     Handle socket event by reading incoming data from real-time capable application.
/// </summary>
static void SocketEventHandler(EventData* eventData)
{
	// Read response from real-time capable application.
	char rxBuf[256];

	union Encap
	{
		uint32_t u32;
		uint8_t u8[4];
	};


	union Encap analog_data;



	/*
	union Analog_data
	{
		uint32_t u32;
		uint8_t u8[4];
	} analog_data;*/

	int bytesReceived = recv(sockFd, rxBuf, sizeof(rxBuf), 0);

	

	if (bytesReceived == -1) {
		LogWebDebug("ERROR: Unable to receive message: %d (%s)\n", errno, strerror(errno));
		terminationRequired = true;
	}

	if (bytesReceived < 4) {
		LogWebDebug("ERROR: No enough bytes recevied: %d recevied", bytesReceived);
		terminationRequired = true;
	}

	// Type transfer
	union Encap typeEncap;
	uint32_t type;
	for (int i = 0; i < sizeof(typeEncap); i++)
	{
		typeEncap.u8[i] = rxBuf[i];
	}
	type = typeEncap.u32;
	int datalength = bytesReceived - sizeof(typeEncap);


	if (type == ADC_LIGHT) {

		// Copy data from Rx buffer to analog_data union
		for (int i = 0; i < sizeof(analog_data); i++)
		{
			analog_data.u8[i] = rxBuf[i + 4];
		}

		// get voltage (2.5*adc_reading/4096)
		// divide by 3650 (3.65 kohm) to get current (A)
		// multiply by 1000000 to get uA
		// divide by 0.1428 to get Lux (based on fluorescent light Fig. 1 datasheet)
		// divide by 0.5 to get Lux (based on incandescent light Fig. 1 datasheet)
		// We can simplify the factors, but for demostration purpose it's OK
		light_sensor = ((float)analog_data.u32 * 2.5 / 4095) * 1000000 / (3650 * 0.1428);
	}
	else if (type == LOG) {
		char log[datalength];
		for (int i = 0; i < datalength; i++) {
			log[i] = rxBuf[i + 4];
		}
		LogWebDebug(log);
	}else if (type == BME680_RECV) {
		uint8_t data[datalength];
		for (int i = 0; i < datalength; i++) {
			data[i] = rxBuf[i + 4];
		}
		
		bme680_sensor_bsec_feedback(data, datalength);
	}
	else if (type == BME680_STATUS_RECV) {
		uint8_t data[datalength];
		for (int i = 0; i < datalength; i++) {
			data[i] = rxBuf[i + 4];
		}

		uint8_t result = bme680_sensor_bsec_status_write(data);
		if (result != 0)
			LogWebDebug("Error save of BME680 BSEC status");
		else
			LogWebDebug("Error save of BME680 BSEC status");

	}


	LogWebDebug("Received %d bytes., type: %d ", datalength, type);

	LogWebDebug("\n");
}

/// <summary>
///     Handle send timer event by writing data to the real-time capable application.
/// </summary>
static void TimerEventHandler(EventData* eventData)
{
	if (ConsumeTimerFdEvent(timerFd) != 0) {
		terminationRequired = true;
		return;
	}
	
	send_request_ADC();
}





/// <summary>
///     Helper function for TimerEventHandler sends message to real-time capable application.
/// </summary>
uint8_t send_request_ADC(void)
{
	static int iter = 0;

	// Send "Read-ADC-%d" message to real-time capable application.
	 char txMessage[32];
	sprintf(txMessage, "Read-ADC-%d", iter++);
	return send_message_to_rtcore(ADC_LIGHT, txMessage, 32);
	
}


uint8_t set_led(uint8_t type, const uint8_t* message, uint16_t len) {
	if (LedRTCore_status) {
		led_set(type, message, len);
		return;
	}

	uint8_t buffer[len + 1];
	buffer[0] = type;
	for (int i = 0;i < len;i++)
		buffer[i + 1] = message[i];



	int bytesSent = send(ledsockFd, buffer, len + 1, 0);
	if (bytesSent == -1)
	{
		LogWebDebug("ERROR: Unable to send led message: %d (%s)\n", errno, strerror(errno));
		terminationRequired = true;
		return -1;
	}
	return 0;
}


 uint8_t send_message_to_rtcore(uint8_t type, const uint8_t *message, uint16_t len) {
	

	uint8_t buffer[len + 1];
	buffer[0] = type;
	for (int i = 0;i < len;i++)
		buffer[i + 1] = message[i];

	

	int bytesSent = send(sockFd, buffer,len+1, 0);
	if (bytesSent == -1)
	{
		LogWebDebug("ERROR: Unable to send message: %d (%s)\n", errno, strerror(errno));
		terminationRequired = true;
		return -1;
	}
	return 0;
}





uint8_t real_core_init() {

	
	ledsockFd = Application_Socket(ledrtAppComponentId);
	if (ledsockFd == -1)
	{
		LogWebDebug("ERROR: Unable to create LED socket: %d (%s)\n", errno, strerror(errno));
		LogWebDebug("Real Time Core disabled or Component Id is not correct.\n");
		LogWebDebug("The program will use main core led pwm facilites.\n");
		LedRTCore_status = 1;
		led_init();
	}

	// Open connection to real-time capable application.
	sockFd = Application_Socket(rtAppComponentId);
	if (sockFd == -1)
	{
		LogWebDebug("ERROR: Unable to create socket: %d (%s)\n", errno, strerror(errno));
		LogWebDebug("Real Time Core disabled or Component Id is not correct.\n");
		LogWebDebug("The program will continue without showing light sensor data.\n");
		// Communication with RT core error
		RTCore_status = 1;
		//return -1;
	}
	else
	{
		// Communication with RT core success
		RTCore_status = 0;
		// Set timeout, to handle case where real-time capable application does not respond.
		static const struct timeval recvTimeout = { .tv_sec = 5,.tv_usec = 0 };
		int result = setsockopt(sockFd, SOL_SOCKET, SO_RCVTIMEO, &recvTimeout, sizeof(recvTimeout));
		if (result == -1)
		{
			LogWebDebug("ERROR: Unable to set socket timeout: %d (%s)\n", errno, strerror(errno));
			return -1;
		}

		// Register handler for incoming messages from real-time capable application.
		if (RegisterEventHandlerToEpoll(epollFd, sockFd, &socketEventData, EPOLLIN) != 0)
		{
			return -1;
		}


	}

	return 0;

	//// end ADC Connection


}

void closerealcore() {
	CloseFdAndPrintError(sockFd, "Real Core BSEC");
	if (LedRTCore_status)
		closeled();
	else
		CloseFdAndPrintError(ledsockFd, "Real Core LED");
}