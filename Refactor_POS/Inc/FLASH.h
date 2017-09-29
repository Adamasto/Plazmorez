#ifndef FLASH_H
#define FLASH_H


void FLASH_InitValue();

uint16_t FLASH_GetValue(uint16_t VirtAddress);

void FLASH_SetValue(uint16_t VirtAddress, uint16_t Data);










#endif /*FLASH_H*/
