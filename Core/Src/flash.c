/*
 * flash.c
 *
 *  Created on: 23 May 2023
 *      Author: marcel
 */

/*
 *	Example:
 *
 *	typedef struct
 *	{
 *	  ip4_addr_t ownIP;
 *	  ip4_addr_t vehicleIP;
 *	  ip4_addr_t baseIP;
 *	  uint16_t ecef_x;
 *	  uint16_t ecef_y;
 *	  uint16_t ecef_z;
 *	} MyFlashStruct;
 *
 *	MyFlashStruct myStruct;
 *
 *	// Initialisiere deine Struktur
 *	IP4_ADDR(&myStruct.ownIP, 192, 168, 0, 1);
 *	IP4_ADDR(&myStruct.vehicleIP, 192, 168, 0, 2);
 *	IP4_ADDR(&myStruct.baseIP, 192, 168, 0, 3);
 *	myStruct.ecef_x = 12345;
 *	myStruct.ecef_y = 57890;
 *	myStruct.ecef_z = 11121;
 *
 *	uint32_t StartSectorAddress = 0x081C0000;
 *
 *	// Schreibe deine Struktur in den Flash-Speicher
 *	Flash_Write_Data(StartSectorAddress, (uint32_t *)&myStruct, sizeof(myStruct) / sizeof(uint32_t));
 *
 *	// Erstelle eine neue Struktur, um die Daten aus dem Flash-Speicher zu lesen
 *	MyFlashStruct newStruct;
 *
 *	// Lies die Daten aus dem Flash-Speicher
 *	Flash_Read_Data(StartSectorAddress, (uint32_t *)&newStruct, sizeof(newStruct) / sizeof(uint32_t));
 *
 */

#include "flash.h"

#define FLASH_USER_START_ADDR   0x08000000 	/* Start @ of user Flash area */
#define FLASH_USER_END_ADDR     0x081FFFFF 	/* End @ of user Flash area */
#define FLASH_SECTOR_SIZE   	((uint32_t)0x40000)  /* Sector size: 256KB */

// ToDO: Only write Flash if new data
uint32_t Flash_Write_Data(uint32_t StartSectorAddress, uint8_t *Data, uint16_t size)
{
  /* Allow Access to Flash control registers and user Flash */
  if (HAL_FLASH_Unlock() == HAL_OK)
  {
	  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR);


    FLASH_EraseInitTypeDef EraseInitStruct;
    EraseInitStruct.TypeErase   = FLASH_TYPEERASE_SECTORS;
    EraseInitStruct.Sector      = GetSector(StartSectorAddress);
    EraseInitStruct.NbSectors   = 1;
    EraseInitStruct.VoltageRange= FLASH_VOLTAGE_RANGE_3;

    uint32_t SectorError = 0;
    /* Erase the Flash sectors */
    if (HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError) == HAL_OK)
    {
    	for (uint16_t i = 0; i < size; i += 4)
		{
			uint32_t dataWord = 0;
			memcpy(&dataWord, &Data[i], sizeof(uint32_t));
			if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, StartSectorAddress, dataWord) != HAL_OK)
			{
				/* Error occurred while writing data in Flash memory */
				return HAL_FLASH_GetError ();
			}
			StartSectorAddress += 4;
		}
    }
    else
    {
      /* Error occurred while sector erase */
      return HAL_FLASH_GetError ();
    }
  }
  else
  {
    /* Error occurred while unlocking the Flash */
    return HAL_FLASH_GetError ();
  }

  HAL_FLASH_Lock();
  return HAL_OK;
}

void Flash_Read_Data(uint32_t StartSectorAddress, uint8_t *RxBuf, uint16_t size)
{
    for (uint16_t i = 0; i < size; i += 4)
    {
        uint32_t dataWord = *(uint32_t *)StartSectorAddress;
        memcpy(&RxBuf[i], &dataWord, sizeof(uint32_t));
        StartSectorAddress += 4;
    }
}

uint32_t GetSector(uint32_t Address)
{
  if((Address < FLASH_USER_START_ADDR) || (Address > FLASH_USER_END_ADDR))
  {
    return -1; // invalid address
  }

  uint32_t sector = 0;

  Address = Address - FLASH_USER_START_ADDR;

  if(Address < 0x08000) sector = 0;        // 32KB
  else if(Address < 0x10000) sector = 1;   // 32KB
  else if(Address < 0x18000) sector = 2;   // 32KB
  else if(Address < 0x20000) sector = 3;   // 32KB
  else if(Address < 0x40000) sector = 4;   // 128KB
  else if(Address < 0x80000) sector = 5;   // 256KB
  else if(Address < 0xC0000) sector = 6;   // 256KB
  else if(Address < 0x100000) sector = 7;  // 256KB
  else if(Address < 0x140000) sector = 8;  // 256KB
  else if(Address < 0x180000) sector = 9;  // 256KB
  else if(Address < 0x1C0000) sector = 10; // 256KB
  else sector = 11;                        // 256KB

  return sector;
}
