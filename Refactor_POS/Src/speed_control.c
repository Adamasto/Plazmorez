#include "speed_control.h"
#include "math.h"
#include "lwip/opt.h"
#include "lwip/arch.h"
#include "lwip/api.h"
#include <string.h>
#include <stdio.h>
#include <ctype.h>
#include <stdlib.h>
#include <stm32f7xx_it.h>
#include <stm32f7xx_hal_tim.h>
#include "udp_send.h"
#include "math.h"
#include "comm_pars.h"
#include "semphr.h"
#include "status_control.h"

extern AxisParam Axx[AXIS_NUM];
extern struct switcherBitMask switcherState;


void SetSpeed(uint8_t numOfAxis, int speed)
{
	switch(numOfAxis)
	{
		case 0:				
			TIMER_X_AXIS_ENGINE->ARR = round((float)TIMER_FREQUENCY / (float)speed);
			break;
		case 1:				
			TIMER_Y1_AXIS_ENGINE->ARR = round((float)TIMER_FREQUENCY / (float)speed);		
			break;	
	}
}

void Start(uint8_t numOfAxis)
{
	switch(numOfAxis)
	{
		case 0:
			TIMER_X_AXIS_ENGINE->CCR1 = PULSE_WIDTH;
			HAL_NVIC_EnableIRQ(TIMER_X_AXIS_IRQn);
			break;
		case 1:
			TIMER_Y1_AXIS_ENGINE->CCR1 = PULSE_WIDTH;
			TIMER_Y1_AXIS_ENGINE->CCR2 = PULSE_WIDTH;
			HAL_NVIC_EnableIRQ(TIMER_Y1_AXIS_IRQn);			
			break;
	}
}

void Stop(uint8_t numOfAxis)
{
	switch(numOfAxis)
	{
		case 0:
			TIMER_X_AXIS_ENGINE->CCR1 = PULSE_OFF;	
			HAL_NVIC_DisableIRQ(TIMER_X_AXIS_IRQn);		
			break;
		case 1:
			TIMER_Y1_AXIS_ENGINE->CCR1 = PULSE_OFF;	
			TIMER_Y1_AXIS_ENGINE->CCR2 = PULSE_OFF;			
			HAL_NVIC_DisableIRQ(TIMER_Y1_AXIS_IRQn);
			break;
	}
	Axx[numOfAxis].status.directEngine = none;
	Axx[numOfAxis].status.currentSpeedStateOfEngine = constant;
}

void Acceleration(uint8_t numOfAxis)
{
	if(Axx[numOfAxis].calculating.next_speed <= TIMER_FREQUENCY/MIN_PERIOD)
	{
		Axx[numOfAxis].calculating.next_speed += Axx[numOfAxis].calculating.deltaAc;		
		SetSpeed(numOfAxis, Axx[numOfAxis].calculating.next_speed);
		osDelay(1);
	}
}

void Deceleration(uint8_t numOfAxis)
{
	if(Axx[numOfAxis].calculating.next_speed >= TIMER_FREQUENCY/MAX_PERIOD)
	{
		Axx[numOfAxis].calculating.next_speed -= Axx[numOfAxis].calculating.deltaDc;						
		SetSpeed(numOfAxis, Axx[numOfAxis].calculating.next_speed);
		osDelay(1);
	}
}

void SlStop(uint8_t numOfAxis)
{	
	if(Axx[numOfAxis].counter.speed.steps <= MIN_SPEED)
	{
		Stop(numOfAxis);
		Axx[numOfAxis].status.directEngine = none;
	}
	Deceleration(numOfAxis);
}



void RunForward(uint8_t numOfAxis)
{
	if(Axx[numOfAxis].status.currentSpeedStateOfEngine == accelerate)
	{
		if((Axx[numOfAxis].counter.speed.steps >= Axx[numOfAxis].target.speed.setsValue.steps) || (Axx[numOfAxis].midddleValue.position.steps >= Axx[numOfAxis].calculating.stepsToAc))
		{			
			Axx[numOfAxis].status.currentSpeedStateOfEngine = constant;
		}				
		else Acceleration(numOfAxis);		
		
		if(Axx[numOfAxis].midddleValue.position.mm >= Axx[numOfAxis].target.position.setsValue.mm)
		{
			Stop(numOfAxis);	
			osDelay(100);
			accuracyGo(numOfAxis);
			Axx[numOfAxis].status.causeOfLastStopEM	= normalStop;
			Axx[numOfAxis].status.currentCommand = none_command;
		}
	}

	if(Axx[numOfAxis].status.currentSpeedStateOfEngine == constant)
	{
		if(Axx[numOfAxis].midddleValue.position.steps >= Axx[numOfAxis].calculating.stepsToDc)
		{			
			Axx[numOfAxis].status.currentSpeedStateOfEngine = decelerate;
		}	
		if(Axx[numOfAxis].midddleValue.position.mm >= Axx[numOfAxis].target.position.setsValue.mm)
		{
			Stop(numOfAxis);	
			osDelay(100);
			accuracyGo(numOfAxis);
			Axx[numOfAxis].status.causeOfLastStopEM	= normalStop;
			Axx[numOfAxis].status.currentCommand = none_command;
		}
	}
	if(Axx[numOfAxis].status.currentSpeedStateOfEngine == decelerate)
	{
		if(Axx[numOfAxis].midddleValue.position.mm >= Axx[numOfAxis].target.position.setsValue.mm)
		{
			Stop(numOfAxis);
			osDelay(100);
			accuracyGo(numOfAxis);
			Axx[numOfAxis].status.causeOfLastStopEM	= normalStop;
			Axx[numOfAxis].status.currentCommand = none_command;
		}	
		Deceleration(numOfAxis);			
	}
}

void RunBackward(uint8_t numOfAxis)
{
	if(Axx[numOfAxis].status.currentSpeedStateOfEngine == accelerate)
	{
		if((Axx[numOfAxis].counter.speed.steps >= Axx[numOfAxis].target.speed.setsValue.steps) || (Axx[numOfAxis].midddleValue.position.steps <= Axx[numOfAxis].calculating.stepsToAc))
		{
			Axx[numOfAxis].status.currentSpeedStateOfEngine = constant;
		}
		else	Acceleration(numOfAxis);	

		if(Axx[numOfAxis].midddleValue.position.mm <= Axx[numOfAxis].target.position.setsValue.mm)
		{
			Stop(numOfAxis);
			osDelay(100);
			accuracyGo(numOfAxis);
			Axx[numOfAxis].status.causeOfLastStopEM	= normalStop;
			Axx[numOfAxis].status.currentCommand = none_command;
		}		
	}
	if(Axx[numOfAxis].status.currentSpeedStateOfEngine == constant)
	{
		if(Axx[numOfAxis].midddleValue.position.steps <= Axx[numOfAxis].calculating.stepsToDc)
		{
			Axx[numOfAxis].status.currentSpeedStateOfEngine = decelerate;
		}	
		
		if(Axx[numOfAxis].midddleValue.position.mm <= Axx[numOfAxis].target.position.setsValue.mm)
		{
			Stop(numOfAxis);			
			osDelay(100);
			accuracyGo(numOfAxis);
			Axx[numOfAxis].status.causeOfLastStopEM	= normalStop;
			Axx[numOfAxis].status.currentCommand = none_command;
		}
	}
	if(Axx[numOfAxis].status.currentSpeedStateOfEngine == decelerate)
	{
		if(Axx[numOfAxis].midddleValue.position.mm <= Axx[numOfAxis].target.position.setsValue.mm)
		{
			Stop(numOfAxis);
			osDelay(100);
			accuracyGo(numOfAxis);
			Axx[numOfAxis].status.causeOfLastStopEM	= normalStop;
			Axx[numOfAxis].status.currentCommand = none_command;
		}	
		Deceleration(numOfAxis);			
	}
}

void ExStop(uint8_t numOfAxis)
{
	Stop(numOfAxis);
	Axx[numOfAxis].status.currentCommand = none_command;
}


void GoHome(uint8_t numOfAxis)
{
	if(Axx[numOfAxis].status.switchers.backwardSwitcher == 0)
	{
		Axx[numOfAxis].status.directEngine = backward;
		switch(numOfAxis)
		{
			case 0:
				AXIS_X_BACKWARD_DIR;
				break;
			case 1:
				AXIS_Y1_BACKWARD_DIR;
				break;
		}
		SetSpeed(numOfAxis, 5000);
		Start(numOfAxis);
	}
	while(Axx[numOfAxis].status.switchers.backwardSwitcher == 0 && Axx[numOfAxis].status.currentCommand == goHome);
	Stop(numOfAxis);
	switch(numOfAxis)
	{
		Axx[numOfAxis].status.directEngine = backward;
		AXIS_Y1_BACKWARD_DIR;
		case 1:
			if(switcherState.Y_backwardLeftSwitcher == 1)
			{
				if(switcherState.Y_backwardRightSwitcher == 0)
				{
					TIMER_Y1_AXIS_ENGINE->CCR2 = PULSE_WIDTH;
					while(switcherState.Y_backwardRightSwitcher == 0 && Axx[numOfAxis].status.currentCommand == goHome);
					Stop(numOfAxis);	
				}
			}
			else
			if(switcherState.Y_backwardRightSwitcher == 1)
			{
				if(switcherState.Y_backwardLeftSwitcher == 0)
				{				
					TIMER_Y1_AXIS_ENGINE->CCR1 = PULSE_WIDTH;
					while(switcherState.Y_backwardLeftSwitcher == 0 && Axx[numOfAxis].status.currentCommand == goHome);
					Stop(numOfAxis);	
				}
			}
			break;
	}
	osDelay(50);
	Axx[numOfAxis].status.directEngine = forward;
	switch(numOfAxis)
	{
		case 0:
			AXIS_X_FORWARD_DIR;
			break;
		case 1:
			AXIS_Y1_FORWARD_DIR;
			break;
	}

	Start(numOfAxis);
	while(Axx[numOfAxis].status.switchers.backwardSwitcher && Axx[numOfAxis].status.currentCommand == goHome);
	osDelay(300);
	Stop(numOfAxis);
	osDelay(200);
	setHomePosition(numOfAxis);
	RoundCalibration(numOfAxis);
	Axx[numOfAxis].status.directEngine = backward;
	switch(numOfAxis)
	{
		case 0:
			AXIS_X_BACKWARD_DIR;
			break;
		case 1:
			AXIS_Y1_BACKWARD_DIR;
			break;
	}
	Start(numOfAxis);
	while(Axx[numOfAxis].status.switchers.backwardSwitcher == 0 && Axx[numOfAxis].status.currentCommand == goHome);
	Stop(numOfAxis);
	osDelay(100);
	Axx[numOfAxis].status.directEngine = forward;
	switch(numOfAxis)
	{
		case 0:
			AXIS_X_FORWARD_DIR;
			break;
		case 1:
			AXIS_Y1_FORWARD_DIR;
			break;
	}
	SetSpeed(numOfAxis, 1650);
	Start(numOfAxis);
	while(Axx[numOfAxis].status.switchers.backwardSwitcher && Axx[numOfAxis].status.currentCommand == goHome);
	osDelay(100);
	Stop(numOfAxis);
	osDelay(350);
	setHomePosition(numOfAxis);
	Axx[numOfAxis].status.currentCommand = none_command;
	Axx[numOfAxis].status.currentEngineStatusMS.calibration = 1;
}

void setHomePosition(uint8_t numOfAxis)
{
	Axx[numOfAxis].calculating.encoderRotSum = 0;
	Axx[numOfAxis].calculating.encoderValueForDir1 = 0;
	Axx[numOfAxis].calculating.encoderValueForDir2 = 0;
	Axx[numOfAxis].calculating.encoderDiffFlags = 0;
	Axx[numOfAxis].encoder.position.steps = 0;
	Axx[numOfAxis].encoder.position.mm = 0;
	
	Axx[numOfAxis].calculating.encoder2RotSum = 0;
	Axx[numOfAxis].calculating.encoder2ValueForDir1 = 0;
	Axx[numOfAxis].calculating.encoder2ValueForDir2 = 0;
	Axx[numOfAxis].calculating.encoder2DiffFlags = 0;
	Axx[numOfAxis].encoder2.position.steps = 0;
	Axx[numOfAxis].encoder2.position.mm = 0;
	
	Axx[numOfAxis].counter.position.steps = 0;
	switch(numOfAxis)
	{
		case 0:
			TIMER_X_AXIS_ENCODER->CNT = 0;
			break;
		case 1:
			TIMER_Y1_AXIS_ENCODER->CNT = 0;
			TIMER_Y2_AXIS_ENCODER->CNT = 0;
			break;
	}
}

void RoundCalibration(uint8_t numOfAxis)
{
	Axx[numOfAxis].status.directEngine = forward;
	SetSpeed(numOfAxis, 2000);	
	switch(numOfAxis)
	{
		case 0:
			AXIS_X_FORWARD_DIR;
			break;
		case 1:
			AXIS_Y1_FORWARD_DIR;
			break;
	}
	Start(numOfAxis);
	while(Axx[numOfAxis].calculating.encoderRotSum != 1 && Axx[numOfAxis].calculating.encoder2RotSum != 1 && Axx[numOfAxis].status.currentCommand == goHome);
	Stop(numOfAxis);
	osDelay(300);
	
	switch(numOfAxis)
	{
		case 0:		
		Axx[numOfAxis].calculating.mmToStepKoeff = ((float)MM_IN_ONE_ENCODER_ROUND * (Axx[numOfAxis].calculating.encoderRotSum + (float)TIMER_X_AXIS_ENCODER->CNT/2 / (float)NUM_ENCODER_STEPS)) / (float)Axx[numOfAxis].counter.position.steps;
		Axx[numOfAxis].calculating.encoderKoef = (float)Axx[numOfAxis].counter.position.steps / ((Axx[numOfAxis].calculating.encoderRotSum * NUM_ENCODER_STEPS) + TIMER_X_AXIS_ENCODER->CNT/2);
			break;
		
		case 1:
			if(Axx[numOfAxis].calculating.encoderRotSum == 1 || Axx[numOfAxis].calculating.encoder2RotSum == 1)
			{
				Axx[numOfAxis].calculating.mmToStepKoeff = ((float)MM_IN_ONE_ENCODER_ROUND * (Axx[numOfAxis].calculating.encoderRotSum + (float)TIMER_Y1_AXIS_ENCODER->CNT/2 / (float)NUM_ENCODER_STEPS)) / (float)Axx[numOfAxis].counter.position.steps;
				Axx[numOfAxis].calculating.encoderKoef = (float)Axx[numOfAxis].counter.position.steps / ((Axx[numOfAxis].calculating.encoderRotSum + (float)TIMER_Y1_AXIS_ENCODER->CNT/2 / (float)NUM_ENCODER_STEPS) * (float)NUM_ENCODER_STEPS);

				Axx[numOfAxis].calculating.mmToStep2Koeff = ((float)MM_IN_ONE_ENCODER_ROUND * (Axx[numOfAxis].calculating.encoder2RotSum + (float)TIMER_Y2_AXIS_ENCODER->CNT/2 / (float)NUM_ENCODER_STEPS)) / (float)Axx[numOfAxis].counter.position.steps;
				Axx[numOfAxis].calculating.encoder2Koef = (float)Axx[numOfAxis].counter.position.steps / ((Axx[numOfAxis].calculating.encoder2RotSum + (float)TIMER_Y2_AXIS_ENCODER->CNT/2 / (float)NUM_ENCODER_STEPS) * (float)NUM_ENCODER_STEPS);
				break;
			}
	}
}


void accuracyGo(uint8_t numOfAxis)
{
	switch(numOfAxis)
	{
		case 0:
			if(Axx[numOfAxis].encoder.position.steps > Axx[numOfAxis].target.position.setsValue.steps)
			{
				Axx[numOfAxis].status.directEngine = backward;
				AXIS_X_BACKWARD_DIR;
			}
			else
			if(Axx[numOfAxis].encoder.position.steps < Axx[numOfAxis].target.position.setsValue.steps)
			{
				Axx[numOfAxis].status.directEngine = forward;
				AXIS_X_FORWARD_DIR;
			}
			else Axx[numOfAxis].status.directEngine = none;
			
			if(Axx[numOfAxis].status.directEngine == forward)
			{
				SetSpeed(numOfAxis, 200);
				Start(numOfAxis);
				while(Axx[numOfAxis].encoder.position.steps < Axx[numOfAxis].target.position.setsValue.steps && Axx[numOfAxis].status.currentCommand == run);
				Stop(numOfAxis);
			}
			if(Axx[numOfAxis].status.directEngine == backward)
			{
				SetSpeed(numOfAxis, 200);
				Start(numOfAxis);
				while(Axx[numOfAxis].encoder.position.steps > Axx[numOfAxis].target.position.setsValue.steps && Axx[numOfAxis].status.currentCommand == run);
				Stop(numOfAxis);
			}
			break;
		case 1:	
			if(Axx[numOfAxis].encoder.position.mm > Axx[numOfAxis].target.position.setsValue.mm)
			{
				Axx[numOfAxis].status.directEngine = backward;
				AXIS_Y1_BACKWARD_DIR;
			}
			else
			if(Axx[numOfAxis].encoder.position.mm < Axx[numOfAxis].target.position.setsValue.mm)
			{
				Axx[numOfAxis].status.directEngine = forward;
				AXIS_Y1_FORWARD_DIR;
			}
			else Axx[numOfAxis].status.directEngine = none;
			
			if(Axx[numOfAxis].status.directEngine == forward)
			{
				SetSpeed(numOfAxis, 200);
				TIMER_Y1_AXIS_ENGINE->CCR1 = PULSE_WIDTH;
				while(Axx[numOfAxis].encoder.position.mm < Axx[numOfAxis].target.position.setsValue.mm && Axx[numOfAxis].status.currentCommand == run);
				Stop(numOfAxis);
			}
			if(Axx[numOfAxis].status.directEngine == backward)
			{
				SetSpeed(numOfAxis, 200);
				TIMER_Y1_AXIS_ENGINE->CCR1 = PULSE_WIDTH;
				while(Axx[numOfAxis].encoder.position.mm > Axx[numOfAxis].target.position.setsValue.mm && Axx[numOfAxis].status.currentCommand == run);
				Stop(numOfAxis);
			}			
			osDelay(100);
			if(Axx[numOfAxis].encoder2.position.mm > Axx[numOfAxis].target.position.setsValue.mm)
			{
				Axx[numOfAxis].status.directEngine = backward;
			  AXIS_Y1_BACKWARD_DIR;
			}
			else
			if(Axx[numOfAxis].encoder2.position.mm < Axx[numOfAxis].target.position.setsValue.mm)
			{
				Axx[numOfAxis].status.directEngine = forward;
				AXIS_Y1_FORWARD_DIR;
			}
			else Axx[numOfAxis].status.directEngine = none;
			
			if(Axx[numOfAxis].status.directEngine == forward)
			{
				SetSpeed(numOfAxis, 200);
				TIMER_Y1_AXIS_ENGINE->CCR2 = PULSE_WIDTH;
				while(Axx[numOfAxis].encoder2.position.mm < Axx[numOfAxis].target.position.setsValue.mm && Axx[numOfAxis].status.currentCommand == run);
				Stop(numOfAxis);
			}
			if(Axx[numOfAxis].status.directEngine == backward)
			{
				SetSpeed(numOfAxis, 200);
				TIMER_Y1_AXIS_ENGINE->CCR2 = PULSE_WIDTH;
				while(Axx[numOfAxis].encoder2.position.mm > Axx[numOfAxis].target.position.setsValue.mm && Axx[numOfAxis].status.currentCommand == run);
				Stop(numOfAxis);
			}			
			break;
	}
}