/**
 ******************************************************************************
 * @file           	: flash.h
 * @brief          	: Functions to read and write the flash memory
 * @author			: Marcel Lammerskitten
 * @date			: 24/05/23
 * @version			: 0.1
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 MobileTronics
 *
 ******************************************************************************
 */

#ifndef FLASH_H_
#define FLASH_H_

#include "lwip.h"
#include "stm32f7xx_hal.h"

#define FLASH_USER_SECTOR_11	0x081C0000
#define FLASH_USER_SECTOR_10	0x08180000
/* struts */
typedef struct
{
    ip4_addr_t 	ownIP;
    ip4_addr_t 	vehicleIP;
    ip4_addr_t 	rtcmBroadcastIP;
    ip4_addr_t 	gateway;
    ip4_addr_t 	netmask;
    uint16_t 	vehicleUdpPort;
    uint16_t 	reserved1;
    uint16_t	rtcmUdpPort;
    uint16_t	reserved2;
    uint32_t	UART2Baud;
    uint8_t 	isBase;
    uint8_t		reserved3;
    uint16_t	reserved4;
} FlashData_t;

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 		Write data into the flash memory
 * @param [in] 	StartPageAddress	Set start address of the wanted flash sector
 * @param [in] 	Data				Pointer to data that should be saved in the flash memory
 * @param [in] 	size				Size of the data
 * @return 		HAL error codes
 */
uint32_t Flash_Write_Data(uint32_t StartPageAddress, uint8_t *Data, uint16_t size);

/**
 * @brief 		Read data into the flash memory
 * @param [in] 	StartPageAddress	Set start address of the wanted flash sector
 * @param [in] 	RxBuf				Pointer to variable where the data should be stored
 * @param [in] 	size				Size of the data
 * @return 		HAL error codes
 */
void Flash_Read_Data(uint32_t StartSectorAddress, uint8_t *RxBuf, uint16_t size);

/**
 * @brief 		Get the flash memory sector
 * @param [in] 	Address				Sector address
 * @return 		Flash memory sector
 */
uint32_t GetSector(uint32_t Address);

#ifdef __cplusplus
}
#endif

#endif /* FLASH_H_ */
