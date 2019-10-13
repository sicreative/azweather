
# Weather Station and Air quality monitor / logging/ purifer under Avnet's starter kit and Microsoft Azure Sphere platform.
Borch BME680 (Seed Studio Groove Card) + Amphenol SM-UART-04-L PM2.5 Laser Dust Sensor + Mikroe Fan for Click 4 (LTC1695)

This is the code for project work for Sensing the World by element14 and Secure Everything by Hackster.io. 

The Main High Core (A7) working for 
1. Monitoring PM value by SM-UART04-L
2. Monitoring Indoor air qualitly (IAQ), temperture and humidity by BME680 gas sensor, apply BSEC library on M4 core.
3. Fan Controll for air purifer based on IAQ and PM value
4. Dimming LED by Hardware PWM for color indicate of current air quality
5. Web interface for show sensor value
6. Microsoft Azure IoT Central for logging, monitoring the data as well as control of FAN speed.

The device require upgraded to OS version 19.09.

This M4 applications work together with AvnetAzureSphere_Wether (High core) for

1. BSEC library for BME680 reading precision adjustment and calcuate the Indoor air quality (IAQ) score. 
2. SoftPWM and dimming for LED (not in use, some function as high core PWM dimming) 

## Code of Conduct
This project has adopted the [Microsoft Open Source Code of Conduct](https://opensource.microsoft.com/codeofconduct/).
