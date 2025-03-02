/*
 * flash_utils.c
 *
 *  Created on: Mar 2, 2025
 *      Author: HeroPC
 */

#include "flash_utils.h"

SFlash_data_storage FS;



int flash_erase_sec_5()
{
	FLASH_EraseInitTypeDef er = {
		.TypeErase = FLASH_TYPEERASE_SECTORS,
		.Banks = FLASH_BANK_1,
		.Sector = FLASH_SECTOR_5,
		.NbSectors = 1,
		.VoltageRange = FLASH_VOLTAGE_RANGE_3
	};
	uint32_t fault_sec = 0;
	HAL_FLASH_Unlock();
	HAL_StatusTypeDef res = HAL_FLASHEx_Erase(&er, &fault_sec);
	HAL_FLASH_Lock();
	return res == HAL_OK ? 0 : -1;
}

void FLASH_SaveSetting()
{
	unsigned int FlashAddr=STM32F401_EE_START_ADDR;
	unsigned short FSDataSize;

	FSDataSize = sizeof(FS)/4;
	unsigned int *DataAddr=(unsigned int*)&FS;
	flash_erase_sec_5();
	for (int i=0; i<FSDataSize; i++)
	  {
	   FlashSaveUint32(FlashAddr,*DataAddr);
	   FlashAddr+=4;
	   DataAddr++;
	  };
}

void FLASH_LoadSetting()
{
	unsigned int FlashAddr=STM32F401_EE_START_ADDR;
	unsigned short FSDataSize;
	FSDataSize = sizeof(FS)/4;
	unsigned int *DataAddr=(unsigned int*)&FS;
    for (int i=0; i<FSDataSize; i++)
    {
      *DataAddr=(*(unsigned int*)(FlashAddr));
      FlashAddr+=4;
      DataAddr++;
    };

}

void FlashSaveUint32(uint32_t addr, uint32_t data)
{

	HAL_FLASH_Unlock();
	HAL_FLASH_Program(TYPEPROGRAM_WORD, addr, data);
	HAL_FLASH_Lock();
}
