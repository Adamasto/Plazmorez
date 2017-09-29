#ifndef UDP_SEND_H
#define UDP_SEND_H
#include "stm32f7xx_hal.h"

void udp_client_init(void);
void udp_client_send_buf(float *buf, uint16_t bufSize);

void udp_send_data();

#endif
