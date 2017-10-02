#ifndef CONVERTOR_H
#define CONVERTOR_H

#include <stdint.h>

int mmToSteps(float mmValue, uint8_t numOfAxis);
float stepsToMm(double stepsValue, uint8_t numOfAxis);

int mmToSteps2(float mmValue, uint8_t numOfAxis);
float steps2ToMm(double stepsValue, uint8_t numOfAxis);
#endif /* CONVERTOR_H */