#include "VirtAdrTable.h"
#include "eeprom.h"


uint16_t Data;
uint16_t Status;

uint16_t FLASH_GetValue(uint16_t VirtAddress)
{
	Status = EE_ReadVariable(VirtAddress, &Data);
	return Data;
}

void FLASH_SetValue(uint16_t VirtAddress, uint16_t Data)
{
	Status = EE_WriteVariable(VirtAddress, Data);
}
