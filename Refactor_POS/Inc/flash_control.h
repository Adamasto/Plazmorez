#ifndef FLASH_CONTROL_H
#define FLASH_CONTROL_H

#include <stdint.h>

uint8_t flash_setNetworkAddress(uint8_t ad0, uint8_t ad1, uint8_t ad2, uint8_t ad3);
uint32_t flash_getNetworkAddress();

uint8_t flash_setPort(uint16_t port);
uint16_t flash_getPort();

uint8_t flash_setUdpTrAddress(uint8_t ad0, uint8_t ad1, uint8_t ad2, uint8_t ad3);
uint32_t flash_getUdpTrAddress();

uint8_t flash_setUdpTrPort(uint16_t port);
uint16_t flash_getUdpTrPort();

uint8_t flash_setUdpTrInterval(uint16_t port);
uint16_t flash_getUdpTrInterval();

void IpReset();


#endif /* FLASH_CONTROL_H*/