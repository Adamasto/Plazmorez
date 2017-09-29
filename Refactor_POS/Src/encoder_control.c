#include "encoder_control.h"
#include "posicioner_config.h"
#include "comm_pars.h"
#include "stm32f7xx_hal.h"
#include "math.h"
#include <stdlib.h>
#include "convertor.h"

extern AxisParam Axx[AXIS_NUM];

void GetPosition(uint8_t numOfAxis)
{
	switch(numOfAxis)
	{
		case 0:
			Axx[numOfAxis].calculating.encoderValueForDir1 =  TIMER_X_AXIS_ENCODER->CNT;			
			Axx[numOfAxis].calculating.encoderDiffFlags = Axx[numOfAxis].calculating.encoderValueForDir2 - Axx[numOfAxis].calculating.encoderValueForDir1;	
			if(Axx[numOfAxis].calculating.encoderDiffFlags > 0) 
			{
				Axx[numOfAxis].calculating.encoderDitect = 1;
			}
			else 
			if(Axx[numOfAxis].calculating.encoderDiffFlags  < 0)
			{
				Axx[numOfAxis].calculating.encoderDitect = -1;
			}
			else Axx[numOfAxis].calculating.encoderDitect = 0;	
			
			if(abs(Axx[numOfAxis].calculating.encoderDiffFlags) >= 2000)
			{
				Axx[numOfAxis].calculating.encoderRotSum += Axx[numOfAxis].calculating.encoderDitect;
			}
			
			Axx[numOfAxis].calculating.encoderValueForDir2 =  Axx[numOfAxis].calculating.encoderValueForDir1;
			
			if(Axx[numOfAxis].calculating.encoderRotSum < 0) 
			{
				Axx[numOfAxis].encoder.position.steps = (float)(Axx[numOfAxis].calculating.encoderRotSum + 1) * (float)NUM_ENCODER_STEPS  - (float)((float)NUM_ENCODER_STEPS - (float)Axx[numOfAxis].calculating.encoderValueForDir1 / 2.0)* (float)Axx[numOfAxis].calculating.encoderKoef;
				Axx[numOfAxis].encoder.position.mm = stepsToMm(Axx[numOfAxis].encoder.position.steps, numOfAxis);
			}
			else
			{
				Axx[numOfAxis].encoder.position.steps = ((float)Axx[numOfAxis].calculating.encoderRotSum * (float)NUM_ENCODER_STEPS + (float)Axx[numOfAxis].calculating.encoderValueForDir1 / 2.0) * (float)Axx[numOfAxis].calculating.encoderKoef;
				Axx[numOfAxis].encoder.position.mm = stepsToMm(Axx[numOfAxis].encoder.position.steps, numOfAxis);
			}
			Axx[numOfAxis].counter.position.mm = stepsToMm(Axx[numOfAxis].counter.position.steps, numOfAxis);
			
			Axx[numOfAxis].midddleValue.position.steps = Axx[numOfAxis].encoder.position.steps;
			Axx[numOfAxis].midddleValue.position.mm = Axx[numOfAxis].encoder.position.mm;
			break;
		case 1:
			Axx[numOfAxis].calculating.encoderValueForDir1 =  TIMER_Y1_AXIS_ENCODER->CNT;		
			Axx[numOfAxis].calculating.encoderDiffFlags = Axx[numOfAxis].calculating.encoderValueForDir2 - Axx[numOfAxis].calculating.encoderValueForDir1;
			if(Axx[numOfAxis].calculating.encoderDiffFlags > 0) 
			{
				Axx[numOfAxis].calculating.encoderDitect = 1;
			}
			else 
			if(Axx[numOfAxis].calculating.encoderDiffFlags  < 0)
			{
				Axx[numOfAxis].calculating.encoderDitect = -1;
			}
			else Axx[numOfAxis].calculating.encoderDitect = 0;	
			
			if(abs(Axx[numOfAxis].calculating.encoderDiffFlags) >= 2000)
			{
				Axx[numOfAxis].calculating.encoderRotSum += Axx[numOfAxis].calculating.encoderDitect;
			}
			
			Axx[numOfAxis].calculating.encoderValueForDir2 =  Axx[numOfAxis].calculating.encoderValueForDir1;
			
			if(Axx[numOfAxis].calculating.encoderRotSum < 0) 
			{
				Axx[numOfAxis].encoder.position.steps = ((float)(Axx[numOfAxis].calculating.encoderRotSum + 1) * (float)NUM_ENCODER_STEPS  - (float)((float)NUM_ENCODER_STEPS - (float)Axx[numOfAxis].calculating.encoderValueForDir1 / 2.0))* (float)Axx[numOfAxis].calculating.encoderKoef;
				Axx[numOfAxis].encoder.position.mm = stepsToMm(Axx[numOfAxis].encoder.position.steps, numOfAxis);
			}
			else
			{
				Axx[numOfAxis].encoder.position.steps = ((float)Axx[numOfAxis].calculating.encoderRotSum * (float)NUM_ENCODER_STEPS + (float)Axx[numOfAxis].calculating.encoderValueForDir1 / 2.0) * (float)Axx[numOfAxis].calculating.encoderKoef;
				Axx[numOfAxis].encoder.position.mm = stepsToMm(Axx[numOfAxis].encoder.position.steps, numOfAxis);
			}
			
			
			////////////////////////
			Axx[numOfAxis].calculating.encoder2ValueForDir1 =  TIMER_Y2_AXIS_ENCODER->CNT;		
			Axx[numOfAxis].calculating.encoder2DiffFlags = Axx[numOfAxis].calculating.encoder2ValueForDir2 - Axx[numOfAxis].calculating.encoder2ValueForDir1;
			if(Axx[numOfAxis].calculating.encoder2DiffFlags > 0) 
			{
				Axx[numOfAxis].calculating.encoder2Ditect = 1;
			}
			else 
			if(Axx[numOfAxis].calculating.encoder2DiffFlags  < 0)
			{
				Axx[numOfAxis].calculating.encoder2Ditect = -1;
			}
			else Axx[numOfAxis].calculating.encoder2Ditect = 0;	
			
			if(abs(Axx[numOfAxis].calculating.encoder2DiffFlags) >= 2000)
			{
				Axx[numOfAxis].calculating.encoder2RotSum += Axx[numOfAxis].calculating.encoder2Ditect;
			}
			
			Axx[numOfAxis].calculating.encoder2ValueForDir2 =  Axx[numOfAxis].calculating.encoder2ValueForDir1;
			
			if(Axx[numOfAxis].calculating.encoder2RotSum < 0) 
			{
				Axx[numOfAxis].encoder2.position.steps = ((float)(Axx[numOfAxis].calculating.encoder2RotSum + 1) * (float)NUM_ENCODER_STEPS  - (float)((float)NUM_ENCODER_STEPS - (float)Axx[numOfAxis].calculating.encoder2ValueForDir1 / 2.0))* (float)Axx[numOfAxis].calculating.encoder2Koef;
				Axx[numOfAxis].encoder2.position.mm = steps2ToMm(Axx[numOfAxis].encoder2.position.steps, numOfAxis);
			}
			else
			{
				Axx[numOfAxis].encoder2.position.steps = ((float)Axx[numOfAxis].calculating.encoder2RotSum * (float)NUM_ENCODER_STEPS + (float)Axx[numOfAxis].calculating.encoder2ValueForDir1 / 2.0) * (float)Axx[numOfAxis].calculating.encoder2Koef;
				Axx[numOfAxis].encoder2.position.mm = steps2ToMm(Axx[numOfAxis].encoder2.position.steps, numOfAxis);
			}
			Axx[numOfAxis].counter.position.mm = steps2ToMm(Axx[numOfAxis].counter.position.steps, numOfAxis);
			
			Axx[numOfAxis].midddleValue.position.steps = (Axx[numOfAxis].encoder.position.steps + Axx[numOfAxis].encoder2.position.steps)/2;
			Axx[numOfAxis].midddleValue.position.mm = (Axx[numOfAxis].encoder.position.mm + Axx[numOfAxis].encoder2.position.mm)/2;
			
			Axx[numOfAxis].calculating.encoder2LastValue = fabs(Axx[numOfAxis].encoder.position.mm - Axx[numOfAxis].encoder2.position.mm);
			break;
	}
}

void GetSpeed(uint8_t numOfAxis)
{
	GetPWMSpeed(numOfAxis);
	GetPWMAccel(numOfAxis);
	
	GetEncSpeed(numOfAxis);
	GetEncAccel(numOfAxis);
	
//	switch(numOfAxis)
//	{
//		case 0:
//			Axx[numOfAxis].currentSpeed = (float)TIMER_FREQUENCY / (float)TIMER_X_AXIS_ENGINE->ARR;
//			break;
//		case 1:
//			Axx[numOfAxis].currentSpeed = (float)TIMER_FREQUENCY / (float)TIMER_Y1_AXIS_ENGINE->ARR;
//			break;
//		case 2:
//			Axx[numOfAxis].currentSpeed = (float)TIMER_FREQUENCY / (float)TIMER_Y2_AXIS_ENGINE->ARR;
//			break;
//	}
//	Axx[numOfAxis].currentSpeed_st = Axx[numOfAxis].currentSpeed / (float)DRIVER_PRESCALLER;
//	Axx[numOfAxis].currentSpeed_mm = (float)Axx[numOfAxis].currentSpeed_st * (float)Axx[numOfAxis].mm_koeff;
		
//	Axx[numOfAxis].ImpAcFlag1 = Axx[numOfAxis].currentSpeed_st;
//	Axx[numOfAxis].ImpAcceleration_st = ((float)Axx[numOfAxis].ImpAcFlag1 - (float)Axx[numOfAxis].ImpAcFlag2) * 1000;
//	Axx[numOfAxis].ImpAcceleration_mm = (float)Axx[numOfAxis].ImpAcceleration_st * (float)Axx[numOfAxis].mm_koeff; 
//	Axx[numOfAxis].ImpAcFlag2 = Axx[numOfAxis].ImpAcFlag1;
//	
	
//	Axx[numOfAxis].EncoderValue1 = Axx[numOfAxis].current_position;	
//	Axx[numOfAxis].currentSpeed_encoder_st = (Axx[numOfAxis].EncoderValue1 - Axx[numOfAxis].EncoderValue2) * 1000;
//	Axx[numOfAxis].currentSpeed_encoder_count[Axx[numOfAxis].speedCount] = Axx[numOfAxis].currentSpeed_encoder_st;
//	Axx[numOfAxis].currentSpeed_encoder_count_sum = Axx[numOfAxis].currentSpeed_encoder_count_sum + Axx[numOfAxis].currentSpeed_encoder_count[Axx[numOfAxis].speedCount] - Axx[numOfAxis].currentSpeed_encoder_count[Axx[numOfAxis].nextSpeedCount];
//	Axx[numOfAxis].currentSpeed_encoder_mm = (float)Axx[numOfAxis].currentSpeed_encoder_count_sum/(float)MID_NUM/*Axx[numOfAxis].currentSpeed_encoder_st*/ * (float)Axx[numOfAxis].mm_koeff;
//	Axx[numOfAxis].currentSpeed_encoder_ar = (float)((float)Axx[numOfAxis].EncoderValue1 - (float)Axx[numOfAxis].EncoderValue2) * 6.25; // !!!!!!!!!!!!!!
//	Axx[numOfAxis].EncoderValue2  = Axx[numOfAxis].EncoderValue1 ;	
	
	
//	Axx[numOfAxis].EncoderSpeedFlagForAccel1 = Axx[numOfAxis].currentSpeed_encoder_mm;
//	Axx[numOfAxis].EncoderAcceleration_mm = ((float)Axx[numOfAxis].currentSpeed_encoder_count[Axx[numOfAxis].speedCount] - (float)Axx[numOfAxis].currentSpeed_encoder_count[Axx[numOfAxis].nextSpeedCount]);
//	Axx[numOfAxis].EncoderAcceleration_mid_mm = Axx[numOfAxis].EncoderAcceleration_mid_mm + ((float)Axx[numOfAxis].currentSpeed_encoder_count[Axx[numOfAxis].speedCount] - (float)Axx[numOfAxis].currentSpeed_encoder_count[Axx[numOfAxis].nextSpeedCount])/MID_NUM;
//	Axx[numOfAxis].EncoderSpeedFlagForAccel2 = Axx[numOfAxis].EncoderSpeedFlagForAccel1;
	
	Axx[numOfAxis].calculating.firstCount ++;
	Axx[numOfAxis].calculating.secondCount ++;
	if(Axx[numOfAxis].calculating.firstCount >= MID_NUM) Axx[numOfAxis].calculating.firstCount = 0;
	if(Axx[numOfAxis].calculating.secondCount >= MID_NUM) Axx[numOfAxis].calculating.secondCount = 0;
}

//void GetAccel(uint8_t numOfAxis)
//{
////		Axx[numOfAxis].EncoderSpeedFlagForAccel1 = Axx[numOfAxis].currentSpeed_encoder_st;
////		Axx[numOfAxis].EncoderAcceleration_st = (Axx[numOfAxis].EncoderSpeedFlagForAccel1 - Axx[numOfAxis].EncoderSpeedFlagForAccel2);
////		Axx[numOfAxis].EncoderAcceleration_mm = (float)Axx[numOfAxis].EncoderAcceleration_st * (float)Axx[numOfAxis].enc_koef;
////		Axx[numOfAxis].EncoderSpeedFlagForAccel2 = Axx[numOfAxis].EncoderSpeedFlagForAccel1;
//	Axx[numOfAxis].encoder.acceleration.val1 = Axx[numOfAxis].encoder.speed.steps;
//	Axx[numOfAxis].encoder.acceleration.steps = ((float)Axx[numOfAxis].encoder.acceleration.val1 - (float)Axx[numOfAxis].encoder.acceleration.val2);
//	Axx[numOfAxis].encoder.acceleration.mm = stepsToMm(Axx[numOfAxis].encoder.acceleration.steps, numOfAxis);
//	Axx[numOfAxis].encoder.acceleration.val2 = Axx[numOfAxis].encoder.acceleration.val1;
//	
//	Axx[numOfAxis].encoder2.acceleration.val1 = Axx[numOfAxis].encoder2.speed.steps;
//	Axx[numOfAxis].encoder2.acceleration.steps = ((float)Axx[numOfAxis].encoder2.acceleration.val1 - (float)Axx[numOfAxis].encoder2.acceleration.val2);
//	Axx[numOfAxis].encoder2.acceleration.mm = stepsToMm(Axx[numOfAxis].encoder2.acceleration.steps, numOfAxis);
//	Axx[numOfAxis].encoder2.acceleration.val2 = Axx[numOfAxis].encoder2.acceleration.val1;
//}

void GetEncSpeed(uint8_t numOfAxis)
{
	Axx[numOfAxis].encoder.speed.val1 = Axx[numOfAxis].encoder.position.steps;	
	Axx[numOfAxis].encoder.speed.dif = (Axx[numOfAxis].encoder.speed.val1 - Axx[numOfAxis].encoder.speed.val2) * 1000;
	Axx[numOfAxis].encoder.speed.window[Axx[numOfAxis].calculating.firstCount] = Axx[numOfAxis].encoder.speed.dif;
	Axx[numOfAxis].encoder.speed.sum = Axx[numOfAxis].encoder.speed.sum + Axx[numOfAxis].encoder.speed.window[Axx[numOfAxis].calculating.firstCount] - Axx[numOfAxis].encoder.speed.window[Axx[numOfAxis].calculating.secondCount];
	Axx[numOfAxis].encoder.speed.steps = (float)Axx[numOfAxis].encoder.speed.sum/(float)MID_NUM;
	Axx[numOfAxis].encoder.speed.mm = stepsToMm(Axx[numOfAxis].encoder.speed.steps, numOfAxis);
	Axx[numOfAxis].encoder.speed.val2 = Axx[numOfAxis].encoder.speed.val1;
	Axx[numOfAxis].midddleValue.speed.steps = Axx[numOfAxis].encoder.speed.steps;
	Axx[numOfAxis].midddleValue.speed.mm = Axx[numOfAxis].encoder.speed.mm;
	if(numOfAxis == 1)
	{
		Axx[numOfAxis].encoder2.speed.val1 = Axx[numOfAxis].encoder2.position.steps;	
		Axx[numOfAxis].encoder2.speed.dif = (Axx[numOfAxis].encoder2.speed.val1 - Axx[numOfAxis].encoder2.speed.val2) * 1000;
		Axx[numOfAxis].encoder2.speed.window[Axx[numOfAxis].calculating.firstCount] = Axx[numOfAxis].encoder2.speed.dif;
		Axx[numOfAxis].encoder2.speed.sum = Axx[numOfAxis].encoder2.speed.sum + Axx[numOfAxis].encoder2.speed.window[Axx[numOfAxis].calculating.firstCount] - Axx[numOfAxis].encoder2.speed.window[Axx[numOfAxis].calculating.secondCount];
		Axx[numOfAxis].encoder2.speed.steps = (float)Axx[numOfAxis].encoder2.speed.sum/(float)MID_NUM;
		Axx[numOfAxis].encoder2.speed.mm = stepsToMm(Axx[numOfAxis].encoder2.speed.steps, numOfAxis);
		Axx[numOfAxis].encoder2.speed.val2 = Axx[numOfAxis].encoder2.speed.val1;
		Axx[numOfAxis].midddleValue.speed.steps = (Axx[numOfAxis].encoder.speed.steps + Axx[numOfAxis].encoder2.speed.steps)/2;
		Axx[numOfAxis].midddleValue.speed.mm = (Axx[numOfAxis].encoder.speed.mm + Axx[numOfAxis].encoder2.speed.mm)/2;	
	}
}

void GetPWMSpeed(uint8_t numOfAxis)
{
	switch(numOfAxis)
	{
		case 0:
			Axx[numOfAxis].counter.speed.steps = (float)TIMER_FREQUENCY / (float)TIMER_X_AXIS_ENGINE->ARR;
			break;
		case 1:
			Axx[numOfAxis].counter.speed.steps = (float)TIMER_FREQUENCY / (float)TIMER_Y1_AXIS_ENGINE->ARR;
			break;
		case 2:
			Axx[numOfAxis].counter.speed.steps = (float)TIMER_FREQUENCY / (float)TIMER_Y2_AXIS_ENGINE->ARR;
			break;
	}
	Axx[numOfAxis].counter.speed.mm = stepsToMm(Axx[numOfAxis].counter.speed.steps, numOfAxis);
}

void GetEncAccel(uint8_t numOfAxis)
{
	Axx[numOfAxis].encoder.acceleration.val1 = Axx[numOfAxis].encoder.speed.steps;
	Axx[numOfAxis].encoder.acceleration.dif = (Axx[numOfAxis].encoder.acceleration.val1 - Axx[numOfAxis].encoder.acceleration.val2) * 1000;
	
	Axx[numOfAxis].encoder.acceleration.window[Axx[numOfAxis].calculating.firstCount] = Axx[numOfAxis].encoder.acceleration.dif;
	Axx[numOfAxis].encoder.acceleration.sum = Axx[numOfAxis].encoder.acceleration.sum + Axx[numOfAxis].encoder.acceleration.window[Axx[numOfAxis].calculating.firstCount] - Axx[numOfAxis].encoder.acceleration.window[Axx[numOfAxis].calculating.secondCount];

	Axx[numOfAxis].encoder.acceleration.steps = (float)Axx[numOfAxis].encoder.acceleration.sum/(float)MID_NUM;
	Axx[numOfAxis].encoder.acceleration.mm = stepsToMm(Axx[numOfAxis].encoder.acceleration.steps, numOfAxis);
	Axx[numOfAxis].encoder.acceleration.val2 = Axx[numOfAxis].encoder.acceleration.val1;	
	Axx[numOfAxis].midddleValue.acceleration.steps = Axx[numOfAxis].encoder.acceleration.steps;
	Axx[numOfAxis].midddleValue.acceleration.mm = Axx[numOfAxis].encoder.acceleration.mm;
	if(numOfAxis == 1)
	{
		Axx[numOfAxis].encoder2.acceleration.val1 = Axx[numOfAxis].encoder2.speed.steps;
		Axx[numOfAxis].encoder2.acceleration.dif = (Axx[numOfAxis].encoder2.acceleration.val1 - Axx[numOfAxis].encoder2.acceleration.val2) * 1000;
		
		Axx[numOfAxis].encoder2.acceleration.window[Axx[numOfAxis].calculating.firstCount] = Axx[numOfAxis].encoder2.acceleration.dif;
		Axx[numOfAxis].encoder2.acceleration.sum = Axx[numOfAxis].encoder2.acceleration.sum + Axx[numOfAxis].encoder2.acceleration.window[Axx[numOfAxis].calculating.firstCount] - Axx[numOfAxis].encoder2.acceleration.window[Axx[numOfAxis].calculating.secondCount];

		Axx[numOfAxis].encoder2.acceleration.steps = (float)Axx[numOfAxis].encoder2.acceleration.sum/(float)MID_NUM;
		Axx[numOfAxis].encoder2.acceleration.val2 = Axx[numOfAxis].encoder2.acceleration.val1;			
		Axx[numOfAxis].midddleValue.acceleration.steps = (Axx[numOfAxis].encoder.acceleration.steps + Axx[numOfAxis].encoder2.acceleration.steps)/2;
		Axx[numOfAxis].midddleValue.acceleration.mm = (Axx[numOfAxis].encoder.acceleration.steps + Axx[numOfAxis].encoder2.acceleration.mm)/2;
	}
}

void GetPWMAccel(uint8_t numOfAxis)
{
	Axx[numOfAxis].counter.acceleration.val1 = Axx[numOfAxis].counter.speed.steps;
	Axx[numOfAxis].counter.acceleration.steps = Axx[numOfAxis].counter.acceleration.val1 - Axx[numOfAxis].counter.acceleration.val2;
	Axx[numOfAxis].counter.acceleration.val2 = Axx[numOfAxis].counter.acceleration.val1;
	Axx[numOfAxis].counter.acceleration.mm = stepsToMm(Axx[numOfAxis].counter.acceleration.steps, numOfAxis);	
}
