/***************************************************************************************************
   Name: SD1306.c
   Sphere OS: 19.05
****************************************************************************************************/
#include "smuart04l.h"


inline static uint16_t smuart04l_combine(uint8_t high, uint8_t low) {
	uint16_t combine = high << 8;
	combine += low;
	return combine;
	
}

uint8_t smuart04l_pm_to_class(uint16_t pm) {
	if (pm <= 50)
		return 0;
	if (pm <= 100)
		return 1;
	if (pm <= 150)
		return 2;
	if (pm <= 200)
		return 3;
	if (pm <= 300)
		return 4;

	return 5;
}

struct smuart04l_pm_data data, temp_buffer;
uint32_t temp_checksum = 0;
uint8_t read_pos = 0;
uint8_t end = 2*13 + 2;
uint64_t currentid = 0;


uint8_t smuart04l_update(uint8_t* receiveBuffer, size_t length) {
	uint8_t status = SMUART04L_STATUS_WAITING_NEXT_BUFFER;
	uint8_t data_high;
	uint8_t data_low;
	for (int i = 0;i < length;i++) {
		

		switch (read_pos) {
			case 0:
				data_high = receiveBuffer[i];
				if (data_high != 0x42) {
					continue;
				}
				break;
			case 1:
				data_low = receiveBuffer[i];
				if (data_low != 0x4D) {
					temp_checksum = 0;
					read_pos = 0;
					continue;
				}
				break;
			case 2:
			case 4:
			case 6:
			case 8:
			case 10:
			case 12:
			case 14:
			
				data_high = receiveBuffer[i];
				
				break;
			case 3:
				data_low = receiveBuffer[i];
				if (smuart04l_combine(data_high, data_low)!=end) {
					temp_checksum = 0;
					read_pos = 0;
					continue;
				}
			case 5:
				data_low = receiveBuffer[i];
				temp_buffer.pm_1 = smuart04l_combine(data_high, data_low);
				break;
			case 7:
				data_low = receiveBuffer[i];
				temp_buffer.pm_2_5 = smuart04l_combine(data_high, data_low);
				break;
			case 9:
				data_low = receiveBuffer[i];
				temp_buffer.pm_10 = smuart04l_combine(data_high, data_low);
				break;
			case 11:
				data_low = receiveBuffer[i];
				temp_buffer.pm_1_e = smuart04l_combine(data_high, data_low);
				break;
			case 13:
				data_low = receiveBuffer[i];
				temp_buffer.pm_2_5_e = smuart04l_combine(data_high, data_low);
				break;
			case 15:
				data_low = receiveBuffer[i];
				temp_buffer.pm_10_e = smuart04l_combine(data_high, data_low);
				break;
			case 16 ... 27:
				if (receiveBuffer[i]!=0) {
					temp_checksum = 0;
					read_pos = 0;
					continue;
				}
				break;
			case 28:
				temp_buffer.version = receiveBuffer[i];
				break;
			case 29:
				temp_buffer.numError = 0;
		
				if (receiveBuffer[i] == 0)
					break;

				temp_buffer.error.fan_speed = (receiveBuffer[i] | 0x1);
				temp_buffer.error.fan_speed_comp = (receiveBuffer[i] | 0x2);
				temp_buffer.error.fan = (receiveBuffer[i] | 0x4);
				temp_buffer.error.low_temp = (receiveBuffer[i] | 0x8);
				temp_buffer.error.high_temp = (receiveBuffer[i] | 0x10);
				temp_buffer.error.laser_alarm = (receiveBuffer[i] | 0x20);
				temp_buffer.error.laser_err = (receiveBuffer[i] | 0x40);
				break;
			case 30:
				data_high = receiveBuffer[i];
				++read_pos;
				continue;
			case 31:
				data_low = receiveBuffer[i];
				if (smuart04l_combine(data_high, data_low) != temp_checksum) {
					if (status != SMUART04L_STATUS_UPDATE) 
						status = SMUART04L_STATUS_ERR_CHECKSUM;
					break;
				}
				temp_buffer.id = currentid++;
				data = temp_buffer;
				
				status = SMUART04L_STATUS_UPDATE;
				break;
			 

		}
		temp_checksum += receiveBuffer[i];
		++read_pos;
		
		if (read_pos >= 31) {
			temp_checksum = 0;
			read_pos = 0;
		}
	


		
	}

	if (read_pos == 0 && status!=SMUART04L_STATUS_UPDATE) {
		status = SMUART04L_STATUS_NO_PM;
	}

	return status;
	
}


struct smuart04l_pm_data smuart04l_get() { return data; };


uint16_t smuart04l_getPM1() { return data.pm_1; };

uint16_t smuart04l_getPM2_5() { return data.pm_2_5; };

uint16_t smuart04l_getPM10() { return data.pm_10; };


uint16_t smuart04l_getPM1_e() { return data.pm_1_e; };

uint16_t smuart04l_getPM2_5_e() { return data.pm_2_5_e; };

uint16_t smuart04l_getPM10_e() { return data.pm_10_e; };;


uint64_t smuart04l_getId() { return data.id; };

uint8_t smuart04l_numoferror() { return data.numError; };

struct smuart04l_error smuart04l_getError() { return data.error; };