#ifndef SPEED_CONTROL_H
#define SPEED_CONTROL_H

#include "comm_pars.h"


void SetSpeed(uint8_t numOfAxis, int speed);
void Start(uint8_t numOfAxis);
void Acceleration(uint8_t numOfAxis);
void Deceleration(uint8_t numOfAxis);
void Stop(uint8_t numOfAxis);

void RoundCalibration(uint8_t numOfAxis);

void RunForward(uint8_t numOfAxis);
void RunBackward(uint8_t numOfAxis);

void ExStop(uint8_t numOfAxis);

void SlStop(uint8_t numOfAxis);


void ButtonControl(uint8_t numOfAxis);
void EngineTest(uint8_t numOfAxis);

void GoHome(uint8_t numOfAxis);
void setHomePosition(uint8_t numOfAxis);

void accuracyGo(uint8_t numOfAxis);


#endif /* SPEED_CONTROL_H */
