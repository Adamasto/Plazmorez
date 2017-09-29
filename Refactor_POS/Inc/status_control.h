#ifndef STATUS_CONTROL_H
#define STATUS_CONTROL_H

#include <stdint.h>

struct switcherBitMask
{
	uint8_t X_backwardSwitcher:1;
	uint8_t X_forwardSwitcher:1;
	uint8_t Y_backwardLeftSwitcher:1;
	uint8_t Y_backwardRightSwitcher:1;
	uint8_t Y_forwardLeftSwitcher:1;
	uint8_t Y_forwardRightSwitcher:1;
	uint8_t reserved:2;
};	


void status_SetError(uint8_t numOfAxis, uint8_t error);
void status_SetBitMaskEngineState(uint8_t numOfAxis, uint8_t state, uint8_t switcher);
uint8_t status_GetBitEngineState(uint8_t numOfAxis, uint8_t state);

int status_GetCurrentPosition(uint8_t numOfAxis);
int status_GetCurrentSpeed(uint8_t numOfAxis);
int status_GetAimPosition(uint8_t numOfAxis);
int status_GetCurrentErrorPosition(uint8_t numOfAxis);
int status_GetLastError(uint8_t numOfAxis);
uint8_t status_GetCurrentMotionStateMask(uint8_t numOfAxis);
int status_GetLastStopCause(uint8_t numOfAxis);

void waitSetBit(uint8_t numOfAxis, uint8_t command);

void status_stateControl(uint8_t numOfAxis);
void switcherStateControl();

void bigAxisEncoderErrorControl();

uint8_t getSwitcherState();

#endif /* STATUS_CONTROL_H */
