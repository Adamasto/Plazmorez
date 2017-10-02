
#include "status_control.h"
#include "comm_pars.h"
#include "posicioner_config.h"
#include "math.h"
#include "convertor.h"
#include "stm32f7xx_hal.h"
#include "flash_control.h"
#include "speed_control.h"

extern AxisParam Axx[AXIS_NUM];
struct switcherBitMask switcherState;


int status_GetCurrentPosition(uint8_t numOfAxis)
{
	return (int)(Axx[numOfAxis].counter.position.mm * 100);
}

int status_GetCurrentSpeed(uint8_t numOfAxis)
{
		return (int)(Axx[numOfAxis].counter.speed.mm * 100);
}
int status_GetAimPosition(uint8_t numOfAxis)
{
		return (int)(Axx[numOfAxis].target.position.setsValue.mm * 100);
}
int status_GetCurrentErrorPosition(uint8_t numOfAxis)
{
//		return (int)(Axx[numOfAxis].calculating.errorPositionValue);// * 100);
	return (int)((Axx[numOfAxis].encoder.position.mm - Axx[numOfAxis].counter.position.mm) * 100);
}
int status_GetLastError(uint8_t numOfAxis)
{
		return Axx[numOfAxis].status.causeOfLastErrorMF;
}
uint8_t status_GetCurrentMotionStateMask(uint8_t numOfAxis)
{
	uint8_t retValue = 0;
	retValue |= Axx[numOfAxis].status.currentEngineStatusMS.inRun;
	retValue |= Axx[numOfAxis].status.currentEngineStatusMS.stop<<1;
	retValue |= Axx[numOfAxis].status.currentEngineStatusMS.inAcceleration<<2;
	retValue |= Axx[numOfAxis].status.currentEngineStatusMS.inDeceleration<<3;
	retValue |= Axx[numOfAxis].status.currentEngineStatusMS.wait<<4;
	retValue |= Axx[numOfAxis].status.currentEngineStatusMS.inTarget<<5;
	retValue |= Axx[numOfAxis].status.currentEngineStatusMS.calibration<<6;
	retValue |= Axx[numOfAxis].status.currentEngineStatusMS.reserved<<7;
	return retValue;
}
int status_GetLastStopCause(uint8_t numOfAxis)
{
		return Axx[numOfAxis].status.causeOfLastStopEM;
}




void waitSetBit(uint8_t numOfAxis, uint8_t command)
{
	switch(command)
	{
		case 0:
			Axx[numOfAxis].status.waitBitMask.spCom = 1;
			break;
		case 1:
			Axx[numOfAxis].status.waitBitMask.acCom = 1;
			break;
		case 2:
			Axx[numOfAxis].status.waitBitMask.dcCom = 1;
			break;
		case 3:
			Axx[numOfAxis].status.waitBitMask.apCom = 1;
			break;
		default:
			break;
	}
}


void status_stateControl(uint8_t numOfAxis)
{
	// set RUN/STOP bitstatus
	if(Axx[numOfAxis].status.currentCommand == run || Axx[numOfAxis].status.currentCommand == calibration || Axx[numOfAxis].status.currentCommand == goHome || Axx[numOfAxis].status.currentCommand == setAxisInOriginPosition)
	{
		Axx[numOfAxis].status.currentEngineStatusMS.inRun = 1;
		Axx[numOfAxis].status.currentEngineStatusMS.stop = 0;
	}
	else 
	{
		Axx[numOfAxis].status.currentEngineStatusMS.inRun = 0;
		Axx[numOfAxis].status.currentEngineStatusMS.stop = 1;
	}
	
	// set acceleration bitstatus
	if(Axx[numOfAxis].status.currentSpeedStateOfEngine == accelerate)
	{
		Axx[numOfAxis].status.currentEngineStatusMS.inAcceleration = 1;
	}
	else Axx[numOfAxis].status.currentEngineStatusMS.inAcceleration = 0;
	
	// set deceleration bitstatus
	if(Axx[numOfAxis].status.currentSpeedStateOfEngine == decelerate)
	{
		Axx[numOfAxis].status.currentEngineStatusMS.inDeceleration = 1;
	}
	else Axx[numOfAxis].status.currentEngineStatusMS.inDeceleration = 0;
	
  // set wait bitstatus
	if(Axx[numOfAxis].status.currentEngineStatusMS.stop && Axx[numOfAxis].status.currentCommand == none_command)
	{
		if(Axx[numOfAxis].status.waitBitMask.spCom == 1 && Axx[numOfAxis].status.waitBitMask.acCom == 1 && Axx[numOfAxis].status.waitBitMask.dcCom == 1 && Axx[numOfAxis].status.waitBitMask.apCom == 1)
		{
			Axx[numOfAxis].status.currentEngineStatusMS.wait = 1;
		}
		else Axx[numOfAxis].status.currentEngineStatusMS.wait = 0;
	}
	else Axx[numOfAxis].status.currentEngineStatusMS.wait = 0;
	
	// set inTarget bitstatus
	if((Axx[numOfAxis].encoder.position.steps >= (Axx[numOfAxis].target.position.setsValue.steps - Axx[numOfAxis].target.admitErrorPosition.steps)) && (Axx[numOfAxis].encoder.position.steps <= (Axx[numOfAxis].target.position.setsValue.steps + Axx[numOfAxis].target.admitErrorPosition.steps)))
	{
		Axx[numOfAxis].status.currentEngineStatusMS.inTarget = 1;
	}
	else 
	{
		Axx[numOfAxis].status.currentEngineStatusMS.inTarget = 0;
	}
	
	// 	position error
	Axx[numOfAxis].status.positionError.mm = fabs(Axx[numOfAxis].encoder.position.mm - Axx[numOfAxis].counter.position.mm);
	Axx[numOfAxis].status.positionError.steps = mmToSteps(Axx[numOfAxis].status.positionError.mm, numOfAxis);
	if(Axx[numOfAxis].status.positionError.steps >= Axx[numOfAxis].target.admitErrorPosition.steps)
	{
		Axx[numOfAxis].status.causeOfLastErrorMF = errorPosition;
	}
	// encoder error
	if(Axx[numOfAxis].encoder.position.steps == Axx[numOfAxis].calculating.startPosition.steps && Axx[numOfAxis].counter.position.steps != Axx[numOfAxis].calculating.startPosition.steps)
	{
		Axx[numOfAxis].status.causeOfLastErrorMF = errorEncoder;
	}
	
	if((Axx[numOfAxis].encoder.position.steps > 0 && Axx[numOfAxis].counter.position.steps < 0) || (Axx[numOfAxis].encoder.position.steps < 0 && Axx[numOfAxis].counter.position.steps > 0))
	{
		Axx[numOfAxis].status.causeOfLastErrorMF = errorImproperConnectionEncoder;
	}
}

void switcherStateControl()
{
	if(HAL_GPIO_ReadPin(X_Backward_Switcher_GPIO_Port, X_Backward_Switcher_Pin) == 0)
	{
		switcherState.X_backwardSwitcher = 1;
		Axx[0].status.switchers.backwardSwitcher = 1;
	}
	else 
	{
		switcherState.X_backwardSwitcher = 0;
		Axx[0].status.switchers.backwardSwitcher = 0;
	}
	
//	if(HAL_GPIO_ReadPin(X_Forward_Switcher_GPIO_Port, X_Forward_Switcher_Pin) == 0)
//	{
//		switcherState.X_forwardSwitcher = 1;
//		Axx[0].status.switchers.forwardSwitcher = 1;
//	}
//	else 
//	{
//		switcherState.X_forwardSwitcher = 0;
//		Axx[0].status.switchers.forwardSwitcher = 0;
//	}
	
	if(HAL_GPIO_ReadPin(Y_Left_Backward_Switcher_GPIO_Port, Y_Left_Backward_Switcher_Pin) == 0)
	{
		switcherState.Y_backwardLeftSwitcher = 1;
		Axx[1].status.switchers.backwardSwitcher = 1;
	}
	else 
	{
		switcherState.Y_backwardLeftSwitcher = 0;
		Axx[1].status.switchers.backwardSwitcher = 0;
	}
	
	if(HAL_GPIO_ReadPin(Y_Left_Forward_Switcher_GPIO_Port, Y_Left_Forward_Switcher_Pin) == 0)
	{
		switcherState.Y_forwardLeftSwitcher = 1;
		Axx[1].status.switchers.forwardSwitcher = 1;
	}
	else 
	{
		switcherState.Y_forwardLeftSwitcher = 0;
		Axx[1].status.switchers.forwardSwitcher = 0;
	}
	
	if(HAL_GPIO_ReadPin(Y_Right_Backward_Switcher_GPIO_Port, Y_Right_Backward_Switcher_Pin) == 0)
	{
		switcherState.Y_backwardRightSwitcher = 1;
		Axx[1].status.switchers.backwardSwitcher = 1;
	}
	else 
	{
		switcherState.Y_backwardRightSwitcher = 0;
		Axx[1].status.switchers.backwardSwitcher = 0;
	}
	
	if(HAL_GPIO_ReadPin(Y_Right_Forward_Switcher_GPIO_Port, Y_Right_Forward_Switcher_Pin) == 0)
	{
		switcherState.Y_forwardRightSwitcher = 1;
		Axx[1].status.switchers.forwardSwitcher = 1;
	}
	else 
	{
		switcherState.Y_forwardRightSwitcher = 0;	
		Axx[1].status.switchers.forwardSwitcher = 0;
	}
}

void bigAxisEncoderErrorControl()
{
	if(Axx[1].calculating.encoder2LastValue >= MAX_ERROR_OF_ENCODER_MM)
	{
		ExStop(1);
		Axx[1].status.currentCommand = none_command;
		Axx[1].status.causeOfLastStopEM = haveError;
		Axx[1].status.causeOfLastErrorMF = errorEncoder;
	}
}
	


uint8_t getSwitcherState()
{
	uint8_t *bitMask = 0;
	bitMask = (uint8_t *)&switcherState;
	return *bitMask;
}