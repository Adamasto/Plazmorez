#include "flash_control.h"
#include "stm32f7xx_hal.h"
#include "posicioner_config.h"

uint8_t flash_setNetworkAddress(uint8_t ad0, uint8_t ad1, uint8_t ad2, uint8_t ad3)
{
	if((ad0 >= 0 && ad0 <= 255) && (ad1 >= 0 && ad1 <= 255) && (ad2 >= 0 && ad2 <= 255) && (ad3 >= 0 && ad3 <= 255))
	{
		uint32_t value = 0;
		value |= (ad0);
		value |= (ad1) << 8;	
		value |= (ad2) << 16;
		value |= (ad3) << 24;
		
		uint16_t port = *(__IO uint16_t*)NETWORK_PORT_FLASH_ADDR;
		
		HAL_FLASH_Unlock();
		FLASH_Erase_Sector(NETWORK_ADDRESS_FLASH_SECTOR, FLASH_VOLTAGE_RANGE_3);
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, NETWORK_ADDRESS_FLASH_ADDR, value);	
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, NETWORK_PORT_FLASH_ADDR, port);	
		HAL_FLASH_Lock(); 	
		return 1;
	}
	else return 0;
}
uint32_t flash_getNetworkAddress()
{
	return *(__IO uint32_t*)NETWORK_ADDRESS_FLASH_ADDR;
}


uint8_t flash_setPort(uint16_t port)
{
	if(port >= 0 && port <= 65535)
	{
		uint32_t networkAddress = *(__IO uint32_t*)NETWORK_ADDRESS_FLASH_ADDR;
		HAL_FLASH_Unlock();
		FLASH_Erase_Sector(NETWORK_PORT_FLASH_SECTOR, FLASH_VOLTAGE_RANGE_3);
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, NETWORK_ADDRESS_FLASH_ADDR, networkAddress);	
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, NETWORK_PORT_FLASH_ADDR, port);	
		HAL_FLASH_Lock();
		return 1;	
	}
	else return 0;
}
uint16_t flash_getPort()
{
	return *(__IO uint16_t*)NETWORK_PORT_FLASH_ADDR;
}


uint8_t flash_setUdpTrAddress(uint8_t ad0, uint8_t ad1, uint8_t ad2, uint8_t ad3)
{
	if((ad0 >= 0 && ad0 <= 255) && (ad1 >= 0 && ad1 <= 255) && (ad2 >= 0 && ad2 <= 255) && (ad3 >= 0 && ad3 <= 255))
	{
		uint32_t value = 0;
		value |= (ad0);
		value |= (ad1) << 8;	
		value |= (ad2) << 16;
		value |= (ad3) << 24;
		
		uint16_t port = *(__IO uint16_t*)UDPTR_PORT_FLASH_ADDR;
		uint16_t interval = *(__IO uint16_t*)UDPTR_INTERVAL_FLASH_ADDR;
		HAL_FLASH_Unlock();
		FLASH_Erase_Sector(UDPTR_ADDRESS_FLASH_SECTOR, FLASH_VOLTAGE_RANGE_3);
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, UDPTR_ADDRESS_FLASH_ADDR, value);
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, UDPTR_PORT_FLASH_ADDR, port);	
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, UDPTR_INTERVAL_FLASH_ADDR, interval);	
		HAL_FLASH_Lock(); 
		
		return 1;	
	}
	else return 0;
}
uint32_t flash_getUdpTrAddress()
{
		return *(__IO uint32_t*)UDPTR_ADDRESS_FLASH_ADDR;
}

uint8_t flash_setUdpTrPort(uint16_t port)
{
	if(port >= 0 && port <= 65535)
	{
		uint32_t networkAddress = *(__IO uint32_t*)UDPTR_ADDRESS_FLASH_ADDR;
		uint16_t interval = *(__IO uint16_t*)UDPTR_INTERVAL_FLASH_ADDR;
		HAL_FLASH_Unlock();
		FLASH_Erase_Sector(UDPTR_PORT_FLASH_SECTOR, FLASH_VOLTAGE_RANGE_3);
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, UDPTR_ADDRESS_FLASH_ADDR, networkAddress);
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, UDPTR_PORT_FLASH_ADDR, port);	
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, UDPTR_INTERVAL_FLASH_ADDR, interval);
		HAL_FLASH_Lock();
		return 1;
	}
	else return 0;
}
uint16_t flash_getUdpTrPort()
{
	return *(__IO uint16_t*)UDPTR_PORT_FLASH_ADDR;
}

uint8_t flash_setUdpTrInterval(uint16_t interval)
{
	if(interval >=10)
	{		
		uint32_t networkAddress = *(__IO uint32_t*)UDPTR_ADDRESS_FLASH_ADDR;
		uint16_t port = *(__IO uint16_t*)UDPTR_PORT_FLASH_ADDR;
		HAL_FLASH_Unlock();
		FLASH_Erase_Sector(UDPTR_INTERVAL_FLASH_SECTOR, FLASH_VOLTAGE_RANGE_3);
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, UDPTR_ADDRESS_FLASH_ADDR, networkAddress);
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, UDPTR_PORT_FLASH_ADDR, port);	
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, UDPTR_INTERVAL_FLASH_ADDR, interval);
		HAL_FLASH_Lock();	
		return 1;
	}
	else return 0;
}
uint16_t flash_getUdpTrInterval()
{
	if(*(__IO uint16_t*)UDPTR_INTERVAL_FLASH_ADDR < 10)
	{
		return 10;
	}
	else return *(__IO uint16_t*)UDPTR_INTERVAL_FLASH_ADDR;
}

void IpReset()
{
	if(HAL_GPIO_ReadPin(button_GPIO_Port, button_Pin))
	{
		uint8_t IP_ADDRESS[4];
		IP_ADDRESS[0] = 192;
		IP_ADDRESS[1] = 168;
		IP_ADDRESS[2] = 16;
		IP_ADDRESS[3] = 251;
		uint16_t IP_PORT = 4000;
		flash_setNetworkAddress(IP_ADDRESS[0], IP_ADDRESS[1], IP_ADDRESS[2], IP_ADDRESS[3]);
		flash_setPort(IP_PORT);
		NVIC_SystemReset();	
	}
}
