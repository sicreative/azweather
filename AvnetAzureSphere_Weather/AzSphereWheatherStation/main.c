/*
 
 Copyright (c) SC Lee GNU GPLv3
 
 Some code copies from 
 Copyright (c) Microsoft Corporation. All rights reserved.
   Licensed under the MIT License. */

  
   *************************************************************************************************/

#include <errno.h>
#include <signal.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h> 
#include <time.h>
#include <unistd.h>
#include <stdio.h>
#include <math.h>


// applibs_versions.h defines the API struct versions to use for applibs APIs.
#include "applibs_versions.h"
#include "epoll_timerfd_utilities.h"


#include "deviceTwin.h"
#include "azure_iot_utilities.h"
#include "connection_strings.h"
#include "build_options.h"
#include <applibs/rtc.h>
#include <applibs/log.h>
#include <applibs/i2c.h>
#include <applibs/gpio.h>
#include <applibs/wificonfig.h>
#include <applibs/adc.h>
#include <applibs/uart.h>

#include <azureiot/iothub_device_client_ll.h>

#include "adc.h"
#include "i2c.h"
#include "web_tcp_server.h"

//// Real core
#include "real_core.h"
#include "mt3620_avnet_dev.h"
//// OLED
#include "oled.h"

//// PM 2.5 sensor
#include "smuart04l.h"

#include "file.h"



//Delay of check and start wifi web interface becase may be freezing after restart if too quick  
int delay = 1000;




// Provide local access to variables in other files
extern twin_t twinArray[];
extern int twinArraySize;
extern IOTHUB_DEVICE_CLIENT_LL_HANDLE iothubClientHandle;

// Support functions.
static void TerminationHandler(int signalNumber);
static int InitPeripheralsAndHandlers(void);
static void ClosePeripheralsAndHandlers(void);
static int CheckNetworkStackStatusAndLaunchServers(void);

// File descriptors - initialized to invalid value
int epollFd = -1;
static int buttonPollTimerFd = -1;
static int buttonAGpioFd = -1;
static int buttonBGpioFd = -1;
static int uartFd = -1;


int clickSocket1Relay1Fd = -1;
int clickSocket1Relay2Fd = -1;








// Button state variables, initilize them to button not-pressed (High)
static GPIO_Value_Type buttonAState = GPIO_Value_High;
static GPIO_Value_Type buttonBState = GPIO_Value_High;

#if (defined(IOT_CENTRAL_APPLICATION) || defined(IOT_HUB_APPLICATION))
	bool versionStringSent = false;
#endif

// Define the Json string format for the accelerator button press data
static const char cstrButtonTelemetryJson[] = "{\"%s\":\"%d\"}";

// Termination state
volatile sig_atomic_t terminationRequired = false;

/// <summary>
///     Signal handler for termination requests. This handler must be async-signal-safe.
/// </summary>
static void TerminationHandler(int signalNumber)
{
    // Don't use Log_Debug here, as it is not guaranteed to be async-signal-safe.
    terminationRequired = true;
}

/// <summary>
///     Handle button, network and Led timer event: 
/// </summary>
static void ButtonLedNetworkTimerEventHandler(EventData* eventData) {


	// Check whether the network stack is ready.
	
	if ( !isNetworkStackReady  && --delay <= 0 ) {
		delay = 2000;
		if (CheckNetworkStackStatusAndLaunchServers() != 0) {
		
			
			//terminationRequired = true;
		}
	}


	if (ConsumeTimerFdEvent(buttonPollTimerFd) != 0) {
		terminationRequired = true;
		return;
	}

	// Check for button A press
	GPIO_Value_Type newButtonAState;
	int result = GPIO_GetValue(buttonAGpioFd, &newButtonAState);
	if (result != 0) {
		LogWebDebug("ERROR: Could not read button GPIO: %s (%d).\n", strerror(errno), errno);
		terminationRequired = true;
		return;
	}

	// If the A button has just been pressed, send a telemetry message
	// The button has GPIO_Value_Low when pressed and GPIO_Value_High when released
	if (newButtonAState != buttonAState) {
		if (newButtonAState == GPIO_Value_Low) {
			LogWebDebug("Button A pressed!\n");
			ltc1695_button_press(true);
		}
		else {
			LogWebDebug("Button A released!\n");
		}

		// Update the static variable to use next time we enter this routine
		buttonAState = newButtonAState;
	}

	// Check for button B press
	GPIO_Value_Type newButtonBState;
	result = GPIO_GetValue(buttonBGpioFd, &newButtonBState);
	if (result != 0) {
		LogWebDebug("ERROR: Could not read button GPIO: %s (%d).\n", strerror(errno), errno);
		terminationRequired = true;
		return;
	}

	// If the B button has just been pressed/released, send a telemetry message
	// The button has GPIO_Value_Low when pressed and GPIO_Value_High when released
	if (newButtonBState != buttonBState) {
		if (newButtonBState == GPIO_Value_Low) {
			// Send Telemetry here
			LogWebDebug("Button B pressed!\n");
			ltc1695_button_press(false);

			//// OLED

			oled_state++;

			if (oled_state > OLED_NUM_SCREEN)
			{
				oled_state = 0;
			}
		}
		else {
			LogWebDebug("Button B released!\n");

		}

		// Update the static variable to use next time we enter this routine
		buttonBState = newButtonBState;

	
	}
}





/// <summary>
///     Handle UART event: if there is incoming data, print it.
/// </summary>
static void UartEventHandler(EventData* eventData)
{
	const size_t receiveBufferSize = 256;
	uint8_t receiveBuffer[receiveBufferSize + 1]; // allow extra byte for string termination
	ssize_t bytesRead;

	// Read incoming UART data. It is expected behavior that messages may be received in multiple
	// partial chunks.
	bytesRead = read(uartFd, receiveBuffer, receiveBufferSize);
	if (bytesRead < 0) {
		LogWebDebug("ERROR: Could not read UART: %s (%d).\n", strerror(errno), errno);
		terminationRequired = true;
		return;
	}

	if (bytesRead > 0) {
		// Null terminate the buffer to make it a valid string, and print it
		receiveBuffer[bytesRead] = 0;
	
		if (smuart04l_update(receiveBuffer, bytesRead) != SMUART04L_STATUS_UPDATE)
			return;
		
	}
}

// event handler data structures. Only the event handler field needs to be populated.
static EventData buttonLedEventData = { .eventHandler = &ButtonLedNetworkTimerEventHandler };

// event handler for PM2.5 sensor 
static EventData uartEventData = { .eventHandler = &UartEventHandler };



/// <summary>
///     Set up SIGTERM termination handler, initialize peripherals, and set up event handlers.
/// </summary>
/// <returns>0 on success, or -1 on failure</returns>
static int InitPeripheralsAndHandlers(void)
{
    struct sigaction action;
    memset(&action, 0, sizeof(struct sigaction));
    action.sa_handler = TerminationHandler;
    sigaction(SIGTERM, &action, NULL);

    epollFd = CreateEpollFd();
    if (epollFd < 0) {
        return -1;
    }

	real_core_init();
	
	if (initI2c() == -1) {
		return -1;
	}
	
	// Traverse the twin Array and for each GPIO item in the list open the file descriptor
	for (int i = 0; i < twinArraySize; i++) {

		// Verify that this entry is a GPIO entry
		if (twinArray[i].twinGPIO != NO_GPIO_ASSOCIATED_WITH_TWIN) {

			*twinArray[i].twinFd = -1;

			// For each item in the data structure, initialize the file descriptor and open the GPIO for output.  Initilize each GPIO to its specific inactive state.
			*twinArray[i].twinFd = (int)GPIO_OpenAsOutput(twinArray[i].twinGPIO, GPIO_OutputMode_PushPull, twinArray[i].active_high ? GPIO_Value_Low : GPIO_Value_High);

			if (*twinArray[i].twinFd < 0) {
				LogWebDebug("ERROR: Could not open LED %d: %s (%d).\n", twinArray[i].twinGPIO, strerror(errno), errno);
				return -1;
			}
		}
	}

	

	// Open button A GPIO as input
	LogWebDebug("Opening Starter Kit Button A as input.\n");
	buttonAGpioFd = GPIO_OpenAsInput(MT3620_RDB_BUTTON_A);
	if (buttonAGpioFd < 0) {
		LogWebDebug("ERROR: Could not open button A GPIO: %s (%d).\n", strerror(errno), errno);
		return -1;
	}
	// Open button B GPIO as input
	LogWebDebug("Opening Starter Kit Button B as input.\n");
	buttonBGpioFd = GPIO_OpenAsInput(MT3620_RDB_BUTTON_B);
	if (buttonBGpioFd < 0) {
		LogWebDebug("ERROR: Could not open button B GPIO: %s (%d).\n", strerror(errno), errno);
		return -1;
	}

	// Set up a timer to poll the buttons
	
	struct timespec buttonPressCheckPeriod = { 0, 1000000 };
	buttonPollTimerFd =
		CreateTimerFdAndAddToEpoll(epollFd, &buttonPressCheckPeriod, &buttonLedEventData, EPOLLIN);
	if (buttonPollTimerFd < 0) {
		return -1;
	}


	// PM 2.5 Sensor
	// Create a UART_Config object, open the UART and set up UART event handler
	UART_Config uartConfig;
	UART_InitConfig(&uartConfig);
	uartConfig.baudRate = 9600;
	uartConfig.flowControl = UART_FlowControl_None;
	uartFd = UART_Open(AVT_SK_CM1_ISU0_UART, &uartConfig);
	if (uartFd < 0) {
		LogWebDebug("ERROR: Could not open UART: %s (%d).\n", strerror(errno), errno);
		return -1;
	}
	if (RegisterEventHandlerToEpoll(epollFd, uartFd, &uartEventData, EPOLLIN) != 0) {
		return -1;
	}
	// PM 2.5 sensor end

	//init adc
	adc_init();


	// Tell the system about the callback function that gets called when we receive a device twin update message from Azure
	AzureIoT_SetDeviceTwinUpdateCallback(&deviceTwinChangedHandler);

	

    return 0;
}

/// <summary>
///     Close peripherals and handlers.
/// </summary>
static void ClosePeripheralsAndHandlers(void)
{
    LogWebDebug("Closing file descriptors.\n");
	webServer_ShutDown(serverState);
	closeI2c();
	closerealcore();
	closeadc();
 
	CloseFdAndPrintError(buttonPollTimerFd, "buttonPoll");
	CloseFdAndPrintError(buttonAGpioFd, "buttonA");
	CloseFdAndPrintError(buttonBGpioFd, "buttonB");
	

	CloseFdAndPrintError(uartFd, "UART");
	CloseFdAndPrintError(epollFd, "Epoll");


	// Traverse the twin Array and for each GPIO item in the list the close the file descriptor
	for (int i = 0; i < twinArraySize; i++) {

		// Verify that this entry has an open file descriptor
		if (twinArray[i].twinGPIO != NO_GPIO_ASSOCIATED_WITH_TWIN) {

			CloseFdAndPrintError(*twinArray[i].twinFd, twinArray[i].twinKey);
		}
	}
}

////
////    Support basic web interface, reference by PrivateNetworkServices
////




static int CheckNetworkStatus(void)
{
	// Ensure the necessary network interface is enabled.
	int result = Networking_SetInterfaceState(NetworkInterface, true);
	if (result != 0) {
		if (errno == EAGAIN) {
			LogWebDebug("INFO: The networking stack isn't ready yet, will try again later.\n");
			return 0;
		}
		else {
			LogWebDebug(
				"ERROR: Networking_SetInterfaceState for interface '%s' failed: errno=%d (%s)\n",
				NetworkInterface, errno, strerror(errno));
			return -1;
		}
	}
	isNetworkStackReady = true;

	// Display total number of network interfaces.
	ssize_t count = Networking_GetInterfaceCount();
	if (count == -1) {
		LogWebDebug("ERROR: Networking_GetInterfaceCount: errno=%d (%s)\n", errno, strerror(errno));
		return -1;
	}
	LogWebDebug("INFO: Networking_GetInterfaceCount: count=%zd\n", count);

	// Read current status of all interfaces.
	size_t bytesRequired = ((size_t)count) * sizeof(Networking_NetworkInterface);
	Networking_NetworkInterface* interfaces = malloc(bytesRequired);
	if (!interfaces) {
		abort();
	}

	ssize_t actualCount = Networking_GetInterfaces(interfaces, (size_t)count);
	if (actualCount == -1) {
		LogWebDebug("ERROR: Networking_GetInterfaces: errno=%d (%s)\n", errno, strerror(errno));
	}
	LogWebDebug("INFO: Networking_GetInterfaces: actualCount=%zd\n", actualCount);

	// Print detailed description of each interface.
	for (ssize_t i = 0; i < actualCount; ++i) {
		LogWebDebug("INFO: interface #%zd\n", i);

		// Print the interface's name.
		char printName[IF_NAMESIZE + 1];
		memcpy(printName, interfaces[i].interfaceName, interfaces[i].interfaceNameLength);
		printName[interfaces[i].interfaceNameLength] = '\0';
		LogWebDebug("INFO:   interfaceName=\"%s\"\n", interfaces[i].interfaceName);

		// Print whether the interface is enabled.
		LogWebDebug("INFO:   isEnabled=\"%d\"\n", interfaces[i].isEnabled);

		// Print the interface's configuration type.
		Networking_IpType ipType = interfaces[i].ipConfigurationType;
		const char* typeText;
		switch (ipType) {
		case Networking_IpType_DhcpNone:
			typeText = "DhcpNone";
			break;
		case Networking_IpType_DhcpClient:
			typeText = "DhcpClient";
			break;
		default:
			typeText = "unknown-configuration-type";
			break;
		}
		LogWebDebug("INFO:   ipConfigurationType=%d (%s)\n", ipType, typeText);

		// Print the interface's medium.
		Networking_InterfaceMedium_Type mediumType = interfaces[i].interfaceMediumType;
		const char* mediumText;
		switch (mediumType) {
		case Networking_InterfaceMedium_Unspecified:
			mediumText = "unspecified";
			break;
		case Networking_InterfaceMedium_Wifi:
			mediumText = "Wi-Fi";
			break;
		case Networking_InterfaceMedium_Ethernet:
			mediumText = "Ethernet";
			break;
		default:
			mediumText = "unknown-medium";
			break;
		}
		LogWebDebug("INFO:   interfaceMediumType=%d (%s)\n", mediumType, mediumText);

		// Print the interface connection status
		Networking_InterfaceConnectionStatus status;
		int result = Networking_GetInterfaceConnectionStatus(interfaces[i].interfaceName, &status);
		if (result != 0) {
			LogWebDebug("ERROR: Networking_GetInterfaceConnectionStatus: errno=%d (%s)\n", errno,
				strerror(errno));
			return -1;
		}
		LogWebDebug("INFO:   interfaceStatus=0x%02x\n", status);
	}

	free(interfaces);

	return 0;
}







static int CheckNetworkStackStatusAndLaunchServers(void)
{
	// Check the network stack readiness and display available interfaces when it's ready.
	if (CheckNetworkStatus() != 0) {
		return -1;
	}



	// The network stack is ready, so unregister the timer event handler and launch servers.
	if (isNetworkStackReady) {
		//UnregisterEventHandlerFromEpoll(epollFd, timerFd);

		Networking_IpConfig ipConfig;
		Networking_IpConfig_Init(&ipConfig);
		Networking_IpConfig_EnableDynamicIp(&ipConfig);
		int result = Networking_IpConfig_Apply(NetworkInterface, &ipConfig);
		Networking_IpConfig_Destroy(&ipConfig);
		
		if (serverState != NULL) {
			webServer_ShutDown(serverState);
		}

		serverState = webServer_Start(epollFd, localServerIpAddress.s_addr, LocalTcpServerPort,
			serverBacklogSize, ServerStoppedHandler);
		if (serverState == NULL) {
			return -1;
		}
	}

	return 0;
}


/// <summary>
///     Main entry point for this application.
/// </summary>
int main(int argc, char *argv[])
{

	setenv("TZ", "GMT-8", 1);
	tzset();

	file_mutable_read();


	// Variable to help us send the version string up only once
	bool networkConfigSent = false;
	char ssid[128];
	uint32_t frequency;
	char bssid[20];
	
	// Clear the ssid array
	memset(ssid, 0, 128);

	LogWebDebug("Version String: %s\n", argv[1]);
	LogWebDebug("Weather station starting.\n");
	
    if (InitPeripheralsAndHandlers() != 0) {
        terminationRequired = true;
    }

    // Use epoll to wait for events and trigger handlers, until an error or SIGTERM happens
    while (!terminationRequired) {
        if (WaitForEventAndCallHandler(epollFd) != 0) {
            terminationRequired = true;
        }

#if (defined(IOT_CENTRAL_APPLICATION) || defined(IOT_HUB_APPLICATION))
		// Setup the IoT Hub client.
		// Notes:
		// - it is safe to call this function even if the client has already been set up, as in
		//   this case it would have no effect;
		// - a failure to setup the client is a fatal error.
		if (!AzureIoT_SetupClient()) {
			LogWebDebug("ERROR: Failed to set up IoT Hub client\n");
			break;
		}
#endif 
		
		WifiConfig_ConnectedNetwork network;
		int result = WifiConfig_GetCurrentNetwork(&network);

		if (result < 0) 
		{
			// LogWebDebug("INFO: Not currently connected to a WiFi network.\n");
			//// OLED
			strncpy(network_data.SSID, "Not Connected", 20);

			network_data.frequency_MHz = 0;

			network_data.rssi = 0;
		}
		else 
		{

			frequency = network.frequencyMHz;
			snprintf(bssid, JSON_BUFFER_SIZE, "%02x:%02x:%02x:%02x:%02x:%02x",
				network.bssid[0], network.bssid[1], network.bssid[2], 
				network.bssid[3], network.bssid[4], network.bssid[5]);
		

			if ((strncmp(ssid, (char*)&network.ssid, network.ssidLength)!=0) || !networkConfigSent) {
				
				memset(ssid, 0, 128);
				strncpy(ssid, network.ssid, network.ssidLength);
				LogWebDebug("SSID: %s\n", ssid);
				LogWebDebug("Frequency: %dMHz\n", frequency);
				LogWebDebug("bssid: %s\n", bssid);
				networkConfigSent = true;

#if (defined(IOT_CENTRAL_APPLICATION) || defined(IOT_HUB_APPLICATION))
				// Note that we send up this data to Azure if it changes, but the IoT Central Properties elements only 
				// show the data that was currenet when the device first connected to Azure.
				checkAndUpdateDeviceTwin("ssid", &ssid, TYPE_STRING, false);
				checkAndUpdateDeviceTwin("freq", &frequency, TYPE_INT, false);
				checkAndUpdateDeviceTwin("bssid", &bssid, TYPE_STRING, false);
#endif 
			}

			//// OLED

			memset(network_data.SSID, 0, WIFICONFIG_SSID_MAX_LENGTH);
			if (network.ssidLength <= SSID_MAX_LEGTH)
			{
				strncpy(network_data.SSID, network.ssid, network.ssidLength);
			}
			else
			{
				strncpy(network_data.SSID, network.ssid, SSID_MAX_LEGTH);
			}

			network_data.frequency_MHz = network.frequencyMHz;

			network_data.rssi = network.signalRssi;
		} 		 	  	  	
#if (defined(IOT_CENTRAL_APPLICATION) || defined(IOT_HUB_APPLICATION))
		if (iothubClientHandle != NULL && !versionStringSent) {

			checkAndUpdateDeviceTwin("versionString", argv[1], TYPE_STRING, false);
			versionStringSent = true;
		}

		// AzureIoT_DoPeriodicTasks() needs to be called frequently in order to keep active
		// the flow of data with the Azure IoT Hub
		AzureIoT_DoPeriodicTasks();
#endif
    }

    ClosePeripheralsAndHandlers();
    LogWebDebug("Application exiting.\n");
    return 0;
}


