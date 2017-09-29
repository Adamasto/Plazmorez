#ifndef ENCODER_CONTROL_H
#define ENCODER_CONTROL_H

#include <stdint.h>




void GetPosition(uint8_t numOfAxis);
void GetSpeed(uint8_t numOfAxis);
void GetAccel(uint8_t numOfAxis);

void GetEncSpeed(uint8_t numOfAxis);
void GetPWMSpeed(uint8_t numOfAxis);

void GetEncAccel(uint8_t numOfAxis);
void GetPWMAccel(uint8_t numOfAxis);


















#endif /* ENCODER_CONTROL_H */