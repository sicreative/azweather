
/* Copyright (c) SC Lee. All rights reserved.
   Licensed under the GNU GPLv3 License.

*/

/* Some of the code in this file was copied from  microsoft MutableStorage example
/* @link https://github.com/Azure/azure-sphere-samples/tree/master/Samples/MutableStorage
/* Copyright (c) Microsoft Corporation. All rights reserved.
   Licensed under the MIT License. */


#pragma once
#include <unistd.h>
#include <signal.h>
#include <errno.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "applibs_versions.h"
#include <applibs/log.h>
#include <applibs/storage.h>
#include "epoll_timerfd_utilities.h"
#include "ltc1695.h"

#define FILE_FAN_STATUS_OFFSET 0
#define FILE_FAN_FILE_STATUS_LENGTH LTC1695_FAN_STORAGE_SIZE 

#define FILE_BME680_FILE_STATUS_OFFSET 1024
#define FILE_BME680_FILE_STATUS_LENGTH BSEC_MAX_STATE_BLOB_SIZE

/**
  *@brief  write whole mutable data to storage

  * @retval i : status
  */
int file_mutable_write(void);
/**
  *@brief  read whole mutable data from storage
 
  * @retval i : status
  */
int file_mutable_read(void);
/**
  * @brief  check the storaged data equal to data wish write, if no write the data
  * @param[in]  data: the write data
  * @param[in]  pos:  the start position 
  * @param[in]  len:  length of data write
  * @retval i : status
  */
int file_checksome_otherwise_save(uint8_t* data, uint16_t pos, uint16_t len);
/**
  * @brief  set the temp storage field data, need call @see file_mutable_write for final write
  * @param[in]  data: the write data
  * @param[in]  pos:  the start position
  * @param[in]  len:  length of data write
  * @retval i : status
  */
int file_set_mutable(uint8_t* data, uint16_t pos, uint16_t len);
/**
  * @brief read the temp storage field data, need call  @see file_mutable_read from storage before use
  * @param[out]  data: the read data
  * @param[in]  pos:  the start position
  * @param[in]  len:  length of data write
  * @retval i : status
  */
int file_retrieve_mutable(uint8_t* data, uint16_t pos, uint16_t len);