/* Copyright (c) SC Lee. All rights reserved.
   Licensed under the GNU GPLv3 License.

*/

/* Some of the code in this file was copied from  microsoft MutableStorage example
/* @link https://github.com/Azure/azure-sphere-samples/tree/master/Samples/MutableStorage
/* Copyright (c) Microsoft Corporation. All rights reserved.
   Licensed under the MIT License. */


#include "file.h"
#include "web_tcp_server.h"

// storage size 
#define MutableSize 2048




uint8_t storage[MutableSize];




 int WriteToMutableFile(uint8_t *value,uint16_t len)
{
	int fd = Storage_OpenMutableFile();
	if (fd < 0) {
		LogWebDebug("ERROR: Could not open mutable file:  %s (%d).\n", strerror(errno), errno);
		return -1;
	}
	ssize_t ret = write(fd, value, len);
	close(fd);
	if (ret < 0) {
		// If the file has reached the maximum size specified in the application manifest,
		// then -1 will be returned with errno EDQUOT (122)
		LogWebDebug("ERROR: An error occurred while writing to mutable file:  %s (%d).\n",
			strerror(errno), errno);
		return -1;
	}
	if (ret < sizeof(value)) {
		// For simplicity, this sample logs an error here. In the general case, this should be
		// handled by retrying the write with the remaining data until all the data has been
		// written.
		LogWebDebug("ERROR: Only wrote %d of %d bytes requested\n", ret, (int)sizeof(value));
		return -1;
	}
	
	return 0;
}


 int ReadMutableFile(uint8_t* value, uint16_t len)
{
	int fd = Storage_OpenMutableFile();
	if (fd < 0) {
		LogWebDebug("ERROR: Could not open mutable file:  %s (%d).\n", strerror(errno), errno);
		return -1;
	}
	
	ssize_t ret = read(fd, value, len);
	if (ret < 0) {
		LogWebDebug("ERROR: An error occurred while reading file:  %s (%d).\n", strerror(errno),
			errno);
	}
	close(fd);

	if (ret < sizeof(value)) {
		return -1;
	}

	return ret;
}


 int file_mutable_write() {
	 return WriteToMutableFile(storage, MutableSize);
 }

 int file_mutable_read() {
	 return ReadMutableFile(storage, MutableSize);
 }

 int file_checksome_otherwise_save(uint8_t* data, uint16_t pos, uint16_t len) {
	 uint8_t lastdata[len];
	 uint8_t result = file_retrieve_mutable(&lastdata, pos, len);
	 if (!result)
		 if (!memcmp(data, lastdata, len))
			 return 1;

	  file_set_mutable(data, pos, len);
	  file_mutable_write();
	  return 0;
 }

 int file_set_mutable(uint8_t *data, uint16_t pos, uint16_t len) {
	 //Each storage field started with the start flag and ended with checksum 
	//end checksum is total sum of read data excluded start_flag

	 uint32_t crc = 0;
		storage[pos++] = 0xA;
		storage[pos++] = 0xD;
		



		for (uint16_t i = pos;i < len + pos;i++) {
			storage[i] = data[i - pos];
			crc += storage[i];
		}
		memcpy(storage+len+pos, &crc, 4);

	
	
		

		return 0;

 }

 int file_retrieve_mutable(uint8_t *data, uint16_t pos, uint16_t len) {
	 
	 if (storage[pos++] != 0xA || storage[pos++] !=0xD)
		 return -1;

	 uint32_t crc = 0;
	 for (uint16_t i = pos;i < len + pos;i++) {
		 data[i - pos] = storage[i];
		 crc += storage[i];
	 }

	 uint32_t storagecrc =0;
	 memcpy(&storagecrc,storage +len + pos, 4);
	 if (storagecrc != crc)
		 return -1;
	 return 0;

 }