#include "convertor.h"
#include "posicioner_config.h"
#include "comm_pars.h"

extern AxisParam Axx[AXIS_NUM];

int mmToSteps(float mmValue, uint8_t numOfAxis)
{
	return mmValue / Axx[numOfAxis].calculating.mmToStepKoeff;
}
float stepsToMm(double stepsValue, uint8_t numOfAxis)
{
	return (float)stepsValue * Axx[numOfAxis].calculating.mmToStepKoeff;
}


int mmToSteps2(float mmValue, uint8_t numOfAxis)
{
	return mmValue / Axx[numOfAxis].calculating.mmToStep2Koeff;
}
float steps2ToMm(double stepsValue, uint8_t numOfAxis)
{
	return (float)stepsValue * Axx[numOfAxis].calculating.mmToStep2Koeff;
}