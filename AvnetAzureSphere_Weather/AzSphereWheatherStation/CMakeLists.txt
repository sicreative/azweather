#  Copyright(c) Microsoft Corporation.All rights reserved.
#  Licensed under the MIT License.

CMAKE_MINIMUM_REQUIRED(VERSION 3.8)
PROJECT(AzSphereWheatherStation C)





# Create executable
ADD_EXECUTABLE(${PROJECT_NAME} main.c adc.c azure_iot_utilities.c bme680.c bme680_sensor.c device_twin.c epoll_timerfd_utilities.c file.c i2c.c led.c lps22hh_reg.c lsm6dso_reg.c ltc1695.c oled.c parson.c real_core.c sd1306.c smuart04l.c web_client.c web_tcp_server.c)
TARGET_LINK_LIBRARIES(${PROJECT_NAME} applibs pthread gcc_s c azureiot)

# Add MakeImage post - build command
INCLUDE("${AZURE_SPHERE_MAKE_IMAGE_FILE}")