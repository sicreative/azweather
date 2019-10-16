/* Copyright (c) SC Lee. All rights reserved.
	Licensed under the GNU GPLv3 License.

	part of code copies from Microsoft Sample Private Network Services
	@link https://github.com/Azure/azure-sphere-samples/tree/master/Samples/PrivateNetworkServices
	 Copyright (c) Microsoft Corporation. All rights reserved.
		  Licensed under the MIT License.
	*/

/* Copyright (c) Microsoft Corporation. All rights reserved.
   Licensed under the MIT License. */

#define _GNU_SOURCE // required for asprintf
#include <stdbool.h>
#include <ctype.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <errno.h>
#include <stddef.h>
#include <stdarg.h>  

#include <sys/socket.h>

#include <applibs/log.h>
#include <applibs/networking.h>

#include "web_tcp_server.h"



#include "bme680_sensor.h"
#include "smuart04l.h"
#include "i2c.h"
#include "ltc1695.h"


static bool isNetworkStackReady = false;
webServer_ServerState* serverState = NULL;

#define MAX_WEBLOG 2048

char weblogBuffer[MAX_WEBLOG];

int weblogBuffer_n = 0;
uint8_t isWebDebug = 0;


// Support functions.
static void HandleListenEvent(EventData *eventData);
static void LaunchRead(webServer_ServerState *serverState);
static void HandleClientReadEvent(EventData *eventData);
static void LaunchWrite(webServer_ServerState *serverState);
static void HandleClientWriteEvent(EventData *eventData);
static int OpenIpV4Socket(in_addr_t ipAddr, uint16_t port, int sockType);
static void ReportError(const char *desc);
static void StopServer(webServer_ServerState *serverState, webServer_StopReason reason);
static webServer_ServerState *EventDataToServerState(EventData *eventData, size_t offset);


/// <summary>
///     Called when the TCP server stops processing messages from clients.
/// </summary>
 void ServerStoppedHandler(webServer_StopReason reason)
{
	const char* reasonText;
	switch (reason) {
	case EchoServer_StopReason_ClientClosed:
		reasonText = "client closed the connection.";

		break;

	case EchoServer_StopReason_Error:
		//	terminationRequired = true;
		reasonText = "an error occurred. See previous log output for more information.";
		break;

	default:
		//	terminationRequired = true;
		reasonText = "unknown reason.";
		break;
	}

	//Restart server
	isNetworkStackReady = false;

	LogWebDebug("INFO: TCP server stopped: %s\n", reasonText);

}



webServer_ServerState *webServer_Start(int epollFd, in_addr_t ipAddr, uint16_t port,
                                         int backlogSize,
                                         void (*shutdownCallback)(webServer_StopReason))
{
    webServer_ServerState *serverState = malloc(sizeof(*serverState));
    if (!serverState) {
        abort();
    }

    // Set EchoServer_ServerState state to unused values so it can be safely cleaned up if only a
    // subset of the resources are successfully allocated.
    serverState->epollFd = epollFd;
    serverState->listenFd = -1;
    serverState->clientFd = -1;
    serverState->listenEvent.eventHandler = HandleListenEvent;
    serverState->clientReadEvent.eventHandler = HandleClientReadEvent;
    serverState->epollInEnabled = false;
    serverState->clientWriteEvent.eventHandler = HandleClientWriteEvent;
    serverState->epollOutEnabled = false;
    serverState->txPayload = NULL;
    serverState->shutdownCallback = shutdownCallback;

    int sockType = SOCK_STREAM | SOCK_CLOEXEC | SOCK_NONBLOCK;
    serverState->listenFd = OpenIpV4Socket(ipAddr, port, sockType);
    if (serverState->listenFd < 0) {
        ReportError("open socket");
        goto fail;
    }

    // Be notified asynchronously when a client connects.
    RegisterEventHandlerToEpoll(epollFd, serverState->listenFd, &serverState->listenEvent, EPOLLIN);

    int result = listen(serverState->listenFd, backlogSize);
    if (result != 0) {
        ReportError("listen");
        goto fail;
    }

    LogWebDebug("INFO: TCP server: Listening for client connection (fd %d).\n",
              serverState->listenFd);

    return serverState;

fail:
    webServer_ShutDown(serverState);
    return NULL;
}

void webServer_ShutDown(webServer_ServerState *serverState)
{
    if (!serverState) {
        return;
    }

    CloseFdAndPrintError(serverState->clientFd, "clientFd");
    CloseFdAndPrintError(serverState->listenFd, "listenFd");

    free(serverState->txPayload);

    free(serverState);
}

void webServer_Restart(webServer_ServerState* serverState){
	int theEpollFd = serverState->epollFd;
	webServer_ShutDown(serverState);
	serverState = webServer_Start(theEpollFd, localServerIpAddress.s_addr, LocalTcpServerPort,
		serverBacklogSize, ServerStoppedHandler);
}


static void HandleListenEvent(EventData *eventData)
{
    webServer_ServerState *serverState =
        EventDataToServerState(eventData, offsetof(webServer_ServerState, listenEvent));
    int localFd = -1;

    do {
        // Create a new accepted socket to connect to the client.
        // The newly-accepted sockets should be opened in non-blocking mode, and use
        // EPOLLIN and EPOLLOUT to transfer data.
        struct sockaddr in_addr;
        socklen_t sockLen = sizeof(in_addr);
        localFd = accept4(serverState->listenFd, &in_addr, &sockLen, SOCK_NONBLOCK | SOCK_CLOEXEC);
        if (localFd < 0) {
            ReportError("accept");
            break;
        }

        LogWebDebug("INFO: TCP server: Accepted client connection (fd %d).\n", localFd);

        // If already have a client, then close the newly-accepted socket.
        if (serverState->clientFd >= 0) {
            LogWebDebug(
                "INFO: TCP server: Closing incoming client connection: only one client supported "
                "at a time.\n");
            break;
        }

        // Socket opened successfully, so transfer ownership to EchoServer_ServerState object.
        serverState->clientFd = localFd;
        localFd = -1;

        LaunchRead(serverState);
    } while (0);

    close(localFd);
}

static void LaunchRead(webServer_ServerState *serverState)
{
    serverState->inLineSize = 0;
    RegisterEventHandlerToEpoll(serverState->epollFd, serverState->clientFd,
                                &serverState->clientReadEvent, EPOLLIN);
}



static void HandleClientReadEvent(EventData *eventData)
{
    webServer_ServerState *serverState =
        EventDataToServerState(eventData, offsetof(webServer_ServerState, clientReadEvent));

    if (serverState->epollInEnabled) {
        UnregisterEventHandlerFromEpoll(serverState->epollFd, serverState->clientFd);
        serverState->epollInEnabled = false;
    }

    // Continue until no immediately available input or until an error occurs.
    size_t maxChars = sizeof(serverState->input) - 1;
	uint8_t last;

    while (true) {
        // Read a single byte from the client and add it to the buffered line.
        uint8_t b;
		
        ssize_t bytesReadOneSysCall = recv(serverState->clientFd, &b, 1, /* flags */ 0);
		
        // If successfully read a single byte then process it.
        if (bytesReadOneSysCall == 1) {
            // If received newline then print received line to debug log.
			if (b == '\r') {
				serverState->input[serverState->inLineSize] = '\0';
				serverState->inLineSize = 0;
				
			}else if (b== '\n' && last=='\r')
			{
				
                
				//Check the hearder recevied include GET and HTTP string as a http reqest 
				char pos = strstr(serverState->input, "GET");
				if ( pos != NULL && strstr(serverState->input, "HTTP") != NULL) {
					serverState->isHttp = 1;
					int begin = 4;
					int end = 5;
				
					if (serverState->input[begin] == '/') {
						while (serverState->input[end] != ' ')
							end++;
						strncpy(serverState->post, &serverState->input[begin], end - begin);
						serverState->post[end - begin] = '\0';

					
					}
						
				
					 

				}


                LogWebDebug("INFO: TCP server: Received \"%s\"\n", serverState->input);
				//serverState->inLineSize = 0;
                //LaunchWrite(serverState);
                
            }

            // If new character is not printable then discard.
            else if (!isprint(b)) {
                // Special case '\n' to avoid printing a message for every line of input.
                if (b != '\n') {
                    LogWebDebug("INFO: TCP server: Discarding unprintable character 0x%02x\n", b);
                }
            }

            // If new character would leave no space for NUL terminator then reset buffer.
            else if (serverState->inLineSize == maxChars) {
                LogWebDebug("INFO: TCP server: Input data overflow. Discarding %zu characters.\n",
                          maxChars);
                serverState->input[0] = b;
                serverState->inLineSize = 1;
            }

            // Else append character to buffer.
            else {
                serverState->input[serverState->inLineSize] = b;
                ++serverState->inLineSize;
            }

			last = b;
        }

        // If client has shut down restart the webServer.
        else if (bytesReadOneSysCall == 0) {
            LogWebDebug("INFO: TCP server: Client has closed connection.\n");
			webServer_Restart(serverState);
			
        //StopServer(serverState, EchoServer_StopReason_ClientClosed);
            break;
        }

        // If receive buffer is empty then wait for EPOLLIN event.
        else if (bytesReadOneSysCall == -1 && errno == EAGAIN) {
            RegisterEventHandlerToEpoll(serverState->epollFd, serverState->clientFd,
                                        &serverState->clientReadEvent, EPOLLIN);
            serverState->epollInEnabled = true;

			//Launch send after received hearder 
			if (serverState->isHttp==1)
				LaunchWrite(serverState);
			
            break;
        }

        // Another error occured so abort the program.
        else {
            ReportError("recv");
		
			webServer_Restart(serverState);
			


         //  StopServer(serverState, EchoServer_StopReason_Error);
            break;
        }
    }
}

static void LaunchWrite(webServer_ServerState *serverState)
{
  

	int begin = 0;
	int end = 0;
	while (serverState->post[++end] != '\0' && serverState->post[end] != '?');
		
	char path[end+1];
	strncpy(&path, &serverState->post[begin], end);
	path[end - begin] = '\0';
	int update = 3;
	
	
	uint8_t willDebug = 0;
	//Set value for GET query 
	if (serverState->post[end++] == '?') {
		begin = end;
		while (serverState->post[end++] != '\0') {
			if (serverState->post[end] == '=') {
				char name[end - begin+1];
				
				strncpy(name, &serverState->post[begin], end-begin);
				name[end - begin] = '\0';
				begin = ++end;
				while (serverState->post[end] != '\0' && serverState->post[end] != '&')
					end++;
				char value[end - begin+1];
				value[end - begin] = '\0';
				strncpy(value, &serverState->post[begin], end-begin);
				
				/// HTTP GET query 
			
				if (!strcmp(name,"update")) {
					update = atoi(value);
				}
				else if (!strcmp(name, "debug")) {
					willDebug = 1;
				}
				else if (!strcmp(name, "sea")) {
					sea = atof(value);
				}

				begin = ++end;

				
			}
		}
	}

	if (willDebug != isWebDebug) {
		if (willDebug == 0) {
			weblogBuffer[0] = '\0';
			weblogBuffer_n = 0;

		}
		isWebDebug = willDebug;
	}

	float pressure = bme680_pressure / 100.0f;
	float altitude = 44330 * (1 - powf((pressure / sea), 1 / 5.255));
	int pmcolor[6] = {0x335912,0xC2C435,0xD5AF38,0xE17634,0x9E4713,0x45176F};
	int iaqcolor[7] = { 0x7FBD26,0x335912,0xC2C435,0xD5AF38,0xE17634,0x9E4713,0x45176F };
	char* timestr;
	time_t t = time(NULL);
	struct tm tm = *localtime(&t);
	
	int hour = tm.tm_hour == 0 ? 12 : tm.tm_hour > 12 ? tm.tm_hour-12 : tm.tm_hour;

	
	asprintf(&timestr, "Time: %d-%d-%d   %s %02d:%02d:%02d",
		tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,tm.tm_hour>=12?"PM":"AM", hour,tm.tm_min, tm.tm_sec);

	


	char* body;
	char* html;
	int result;
	int code;




	
	//First page, simple show weather data
	if (!strcmp(path, "/") || !strcmp(path, "/index.htm") | !strcmp(path, "/index.html")) {
		code = 200;

		result = asprintf(&body, "<!DOCTYPE html>\n\
<html>\n\
<head>\n\
<style>\
div {\
margin-bottom: 15px;\
	padding: 4px 12px;\
	}\
.info {\
background-color: #f2f2f2;\
		border-left: 8px solid #2196F3;\
		border-bottom: 2px solid #2196F3;\
	}\
table{\
 font-family: arial, sans-serif;\
 border-collapse:collapse;\
 width:95%%;\
 \
}\
td, th{ border: 1px solid #ba9797;\
			  text-align: center;\
			  padding: 8px;\
			}\
tr:nth-child(even) {\
			background-color: #ffe6e6;\
			}</style>\
			<link  id=\"favicon\" rel=\"shortcut icon\" href = \"#\">\
			<meta charset=\"UTF-8\">\
			<meta http-equiv=\"Expires\" content = \"0\" >\
			<meta http-equiv=\"refresh\" content=\"%d\" />\n\
			<meta http-equiv= \"Content-type\" content=\"text/html\">\n\
			<title>Weather Station </title>\n\
			 </head><body style=\"background-color:powderblue;\">\n\
<tb><h1 style=\"color: #5e9ca0;\">Weather Station</h1>\
<div class=\"info\">\
<p><strong>Az Sphere + Bosch BME680 Gas+ Amphenol Laser PM sensor control a purifier fan</strong></div>\n\
<table align=\"center\">\
<tr>\
<th>Sensor</th>\
<th>Value</th>\
<th>Unit</th>\
</tr>\
<tr>\
<td>Temperature</td>\
<td>%.2f</td>\
<td>C</td>\
</tr>\
<tr>\
<td>Humidity</td>\
<td>%.2f</td>\
<td>%%</td>\
</tr>\
<tr>\
<td>Pressure</td>\
<td>%.2f</td>\
<td>hPa</td>\
</tr>\
<tr>\
<td>Altitude</td>\
<td>%.2f</td>\
<td>m</td>\
</tr>\
<tr>\
<td>PM1</td>\
<td bgcolor=\"#%06x\"><font color=\"ddeedd\">%d</font></td>\
<td>μg/㎥</td>\
</tr>\
<tr>\
<td >PM2.5</td>\
<td bgcolor=\"#%06x\"><font color=\"ddeedd\">%d</font></td>\
<td>μg/㎥</td>\
</tr>\
<tr>\
<td >PM10</td>\
<td bgcolor=\"#%06x\"><font color=\"ddeedd\">%d</font></td>\
<td>μg/㎥</td>\
</tr>\
<tr>\
<td>Lighting</td>\
<td>%.2f</td>\
<td>lux</td>\
</tr>\
<tr>\
<td >IAQ</td>\
<td bgcolor=\"#%06x\"><font color=\"ddeedd\">%d</font></td>\
<td>1-500</td>\
</tr>\
<tr>\
<td>eCO2</td>\
<td>%d</td>\
<td>ppm</td>\
</tr>\
<tr>\
<td>eVOC</td>\
<td>%d</td>\
<td>ppm</td>\
</tr>\
<tr>\
<td>Accuracy</td>\
<td>%d</td>\
<td>1-3</td>\
</tr>\
<tr>\
<td>Gas Resistance</td>\
<td>%d</td>\
<td>ohm</td>\
</tr>\
<tr>\
<td>Fan</td>\
<td>%d</td>\
<td>mV</td>\
</tr>\
</table><p>%s</p><br><br><pre>\
%s</pre>\
</body></html>",update,
			bme680_temperature,
			bme680_humidity,
			pressure,
			altitude,
			pmcolor[smuart04l_pm_to_class(smuart04l_getPM1())],
			smuart04l_getPM1(),
			pmcolor[smuart04l_pm_to_class(smuart04l_getPM2_5())],
			smuart04l_getPM2_5(),
			pmcolor[smuart04l_pm_to_class(smuart04l_getPM10())],
			smuart04l_getPM10(),
			light_sensor,
			iaqcolor[iaq_class],
			bme680_iaq,
			bme680_eco2,
			bme680_evoc,
			bme680_virtual_sensor_data[BME680_VIR_SENSOR_IAQ].accuracy,
			(int)bme680_virtual_sensor_data[BME680_VIR_SENSOR_GAS].value,
			ltc1695_value,
			timestr,
			weblogBuffer

			);

			free(timestr);
			//free(debug_str);

		/*result = asprintf(&body,"<!DOCTYPE html PUBLIC \"-//W3C//DTD HTML 4.01 Transitional//EN\" \"http://www.w3.org/TR/html4/loose.dtd\">\n\
			<html>\n\
			<head>\n\
			<meta http-equiv=\"refresh\" content=\"%d\" />\	
			 <meta http-equiv=\"Content-type\" content=\"text/html;charset=UTF-8\">\n\
			<title>Weather </title>\n\
			 </head>\n\<!DOCTYPE html PUBLIC \"-//W3C//DTD HTML 4.01 Transitional//EN\" \"http://www.w3.org/TR/html4/loose.dtd\">\n\
			<html>\n\
			<head>\n\
			<meta http-equiv=\"refresh\" content=\"%d\" />\	
			 <meta http-equiv=\"Content-type\" content=\"text/html;charset=UTF-8\">\n\
			<title>Weather </title>\n\
			 </head>\n\
			  <body bgcolor=\"#cc9999\">\n\
				<h4>Test</h4></body></html>",update);*/
		if (result == -1) {
			ReportError("asprintf");
			StopServer(serverState, EchoServer_StopReason_Error);
			return;
		}
	}
	else {
		code = 404;
		result = asprintf(&body, "<!DOCTYPE html PUBLIC \"-//W3C//DTD HTML 4.01 Transitional//EN\" \"http://www.w3.org/TR/html4/loose.dtd\">\n\
			<html>\n\
			<head>\n\
			 <meta http-equiv=\"Content-type\" content=\"text/html;charset=UTF-8\">\n\
			<title>Weather </title>\n\
			 </head>\n\
			  <body >\n\
				<h4>404 Not found</h4></body></html><br>azsphere weather webInterface", localServerIpAddress.s_addr, LocalTcpServerPort);
		if (result == -1) {
			ReportError("asprintf");
			StopServer(serverState, EchoServer_StopReason_Error);
			return;
		}
	}



	int bodylen = strlen(body);

	//header
	char* status;
	if (code == 202)
		asprintf(&status, "%d %s", code, "Ok");
	else
		asprintf(&status, "%d %s", code, "Not found");
	
	result = asprintf(&html, "HTTP/1.1 %s \015\012\
Server: AzSphere\015\012\
Cache-Control: private, max-age=0\015\012\
Content-Length:%d\
Content-Type: text/html\015\012\
Connection:close\015\012\
\015\012%s",status,bodylen,body);

	free(body);
	free(status);

	
    // Start to send the response.
    serverState->txPayloadSize = (size_t)strlen(html);
    serverState->txPayload = (uint8_t *)html;
    serverState->txBytesSent = 0;
    HandleClientWriteEvent(&serverState->clientWriteEvent);

	
	
	free(serverState->txPayload);

	
	
}

static void HandleClientWriteEvent(EventData *eventData)
{
    webServer_ServerState *serverState =
        EventDataToServerState(eventData, offsetof(webServer_ServerState, clientWriteEvent));

    if (serverState->epollOutEnabled) {
        UnregisterEventHandlerFromEpoll(serverState->epollFd, serverState->clientFd);
        serverState->epollOutEnabled = false;
    }

    // Continue until have written entire response, error occurs, or OS TX buffer is full.
    while (serverState->txBytesSent < serverState->txPayloadSize) {
        size_t remainingBytes = serverState->txPayloadSize - serverState->txBytesSent;
        const uint8_t *data = &serverState->txPayload[serverState->txBytesSent];
        ssize_t bytesSentOneSysCall =
            send(serverState->clientFd, data, remainingBytes, /* flags */ 0);

        // If successfully sent data then stay in loop and try to send more data.
        if (bytesSentOneSysCall > 0) {
            serverState->txBytesSent += (size_t)bytesSentOneSysCall;
        }

        // If OS TX buffer is full then wait for next EPOLLOUT.
        else if (bytesSentOneSysCall < 0 && errno == EAGAIN) {
            RegisterEventHandlerToEpoll(serverState->epollFd, serverState->clientFd,
                                        &serverState->clientWriteEvent, EPOLLOUT);
            serverState->epollOutEnabled = true;
            return;
        }

        // Another error occurred so terminate the program.
        else {
            ReportError("send");
           // StopServer(serverState, EchoServer_StopReason_Error);
			webServer_Restart(serverState);
			
            return;
        }
    }

    // If reached here then successfully sent entire payload so clean up and read next line from
    // client.
    free(serverState->txPayload);
    serverState->txPayload = NULL;
	int fd = serverState->epollFd;
	
	

	//Restart for next process 
	webServer_Restart(serverState);
	
   // LaunchRead(serverState);
}

static int OpenIpV4Socket(in_addr_t ipAddr, uint16_t port, int sockType)
{
    int localFd = -1;
    int retFd = -1;

    do {
        // Create a TCP / IPv4 socket. This will form the listen socket.
        localFd = socket(AF_INET, sockType, /* protocol */ 0);
        if (localFd < 0) {
            ReportError("socket");
            break;
        }

        // Enable rebinding soon after a socket has been closed.
        int enableReuseAddr = 1;
        int r = setsockopt(localFd, SOL_SOCKET, SO_REUSEADDR, &enableReuseAddr,
                           sizeof(enableReuseAddr));
        if (r != 0) {
            ReportError("setsockopt/SO_REUSEADDR");
            break;
        }

        // Bind to a well-known IP address.
        struct sockaddr_in addr;
        memset(&addr, 0, sizeof(addr));
        addr.sin_family = AF_INET;
        addr.sin_addr.s_addr = ipAddr;
        addr.sin_port = htons(port);

        r = bind(localFd, (const struct sockaddr *)&addr, sizeof(addr));
        if (r != 0) {
            ReportError("bind");
            break;
        }

        // Port opened successfully.
        retFd = localFd;
        localFd = -1;
    } while (0);

    close(localFd);

    return retFd;
}

static void ReportError(const char *desc)
{
    LogWebDebug("ERROR: TCP server: \"%s\", errno=%d (%s)\n", desc, errno, strerror(errno));
}

static void StopServer(webServer_ServerState *serverState, webServer_StopReason reason)
{
    // Stop listening for incoming connections.
    if (serverState->listenFd != -1) {
        UnregisterEventHandlerFromEpoll(serverState->epollFd, serverState->listenFd);
    }

    serverState->shutdownCallback(reason);
}

static webServer_ServerState *EventDataToServerState(EventData *eventData, size_t offset)
{
    uint8_t *eventData8 = (uint8_t *)eventData;
    uint8_t *serverState8 = eventData8 - offset;
    return (webServer_ServerState *)serverState8;
}


int LogWebDebug(const char* fmt, ...) {

	va_list argptr;
	va_start(argptr, fmt);

	Log_DebugVarArgs(fmt, argptr);

	if (!isWebDebug)
		return;


	char buffer[256];


	int total = vsprintf(buffer, fmt, argptr);

	if (total == 0)
		return;

	if (buffer[total - 1] != '\n') {
		buffer[total++] = '\n';
		buffer[total] = '\0';
	}
	
	for (int i = 0;i < total;i++) {
		if (buffer[i] == '<') {
			// replace to "(" if not enough buffer
			if (total + 3 > 256) {
				buffer[i] = '(';
				continue;
			}
			// otherwise replace html escape
			for (int j = total;j > i + 3;j--)
				buffer[j] = buffer[j - 3];
			buffer[i] = '&';
			buffer[i + 1] = 'l';
			buffer[i + 2] = 't';
			buffer[i + 3] = ';';
			total += 3;
		}
		else if (buffer[i] == '>') {
			// replace to ")" if not enough buffer
			if (total + 3 > 256) {
				buffer[i] = ')';
				continue;
			}
			// otherwise replace html escape
			for (int j = total;j > i + 3;j--)
				buffer[j] = buffer[j - 3];
			buffer[i] = '&';
			buffer[i + 1] = 'g';
			buffer[i + 2] = 't';
			buffer[i + 3] = ';';
			total += 3;

		}
		else if (buffer[i] == '&') {
			// replace to "A" if not enough buffer
			if (total + 4 > 256) {
				buffer[i] = 'A';
				continue;
			}
			// otherwise replace html escape
			for (int j = total;j > i + 4;j--)
				buffer[j] = buffer[j - 4];
			buffer[i] = '&';
			buffer[i + 1] = 'a';
			buffer[i + 2] = 'm';
			buffer[i + 3] = 'p';
			buffer[i + 4] = ';';
			total += 4;
		}

	}


	weblogBuffer_n += total;

	while (weblogBuffer_n > MAX_WEBLOG) {
		// find first next line char 
		char* firstnextline = strchr(weblogBuffer, '\n');
		if (firstnextline == NULL) {
			weblogBuffer_n = 0;
				break;
		}

		//shift content to begin

		int i = 0;
		do {
			weblogBuffer[i++] = *++firstnextline;
		} while (*(firstnextline) != '\0');
		weblogBuffer_n = i+total-1;


	}

    int j = 0;
	int i = weblogBuffer_n - total;
	if (i > 0)
		i--;

	for (;i < weblogBuffer_n;i++)
		weblogBuffer[i] = buffer[j++];







	
	

}
