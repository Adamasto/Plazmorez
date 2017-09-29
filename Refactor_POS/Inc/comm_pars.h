#ifndef COMM_PARS_H
#define COMM_PARS_H


#include <stdint.h>
#include "posicioner_config.h"




#pragma anon_unions
typedef union {
	struct {
		char ethAxis[SYMBOL_NUM_AXIS];
		char ethCmd[SYMBOL_NUM_CMD];
		char ethSymb;
		char ethValue[100];
		char endSymb;
	};
} TEthPacket;



enum
{
	sp = 0,
	ac,
	dc,	
	ap,
	bg,
	st,
	cl,
	ab,
	gp,
	gs,
	ao,
	pp,
	gh,
	na,
	np,
	ua,
	up,
	ui,
	ps,
	vl,
	dp,
	pe,
	mf,
	ms,
	em,
};

	struct waitMask
	{
		uint8_t spCom:1;
		uint8_t acCom:1;
		uint8_t dcCom:1;
		uint8_t apCom:1;
		uint8_t calibrationFlag:1;		
		uint8_t reserved1:3;
	};

typedef struct
{
	double steps;
	float mm;
}valuesPhysH;

typedef struct
{
	double steps;
	float mm;
	float window[MID_NUM];
	float val1;
	float val2;
	float sum;
	float dif;
}valuesPhysAndMidH;

typedef struct
{
	valuesPhysH position;
	valuesPhysAndMidH speed;
	valuesPhysAndMidH acceleration;
}encCountStructH;

typedef struct
{
	valuesPhysH setsValue;
	valuesPhysH backwardLimit;
	valuesPhysH forwardLimit;
}valuesLimitH;

typedef struct
{
	valuesLimitH speed;
	valuesLimitH acceleration;
	valuesLimitH deceleration;
	valuesLimitH position;
	valuesPhysH admitErrorPosition;
}valuesForTargetH;

typedef struct
{
	valuesPhysH startPosition;
	valuesPhysH distance;
	double stepsToAc;
	double stepsToDc;
	float deltaAc;
	float deltaDc;
	int deltaSpeed;
	float next_speed;
	float errorPositionValue;
	
	float encoderLastValue;
	int encoderDitect;
	int encoderDiffFlags;
	uint16_t encoderValueForDir1;
	uint16_t encoderValueForDir2;
	int encoderRotSum;
	float encoderKoef;
	float mmToStepKoeff;
	
	float encoder2LastValue;
	int encoder2Ditect;
	int encoder2DiffFlags;
	uint16_t encoder2ValueForDir1;
	uint16_t encoder2ValueForDir2;
	int encoder2RotSum;
	float encoder2Koef;
	float mmToStep2Koeff;
	
	uint8_t firstCount;
	uint8_t secondCount;
}valuesForCalcH;

typedef struct
{
	unsigned inRun:1;
	unsigned stop:1;
	unsigned inAcceleration:1;
	unsigned inDeceleration:1;
	unsigned wait:1;
	unsigned inTarget:1;
	unsigned calibration:1;
	unsigned reserved:1;
}statusMSbitMaskH;

typedef struct
{
	uint8_t forwardSwitcher;
	uint8_t backwardSwitcher;
}switcherStateH;

typedef struct
{
	valuesPhysH positionError;
	statusMSbitMaskH currentEngineStatusMS;
	switcherStateH switchers;
	enum
	{
		nothing = 0,
		normalStop,
		forwardSwitcher,
		backwardSwitcher,
		bacwardSoftLimit,
		forwardSoftLimit,
		haveError,
		recievStopCommand,
	}causeOfLastStopEM;
	
	enum
	{
		notError = 0,
		errorDriver,
		recievES,
		errorPosition,
		errorEngine,
		errorImproperConnectionEncoder,
		errorEncoder,
	}causeOfLastErrorMF;
	
	enum
	{		
		backward = 0,
		none = 1,
		forward = 2,		
	}directEngine;	
	
	enum
	{
		none_command = 0,
		stop,	
		am_stop,
		run,
		calibration,
		goHome,	
		setAxisInOriginPosition,		
	}currentCommand;
	
	enum
	{
		accelerate,
		constant,
		decelerate,
	}currentSpeedStateOfEngine;
	
	struct waitMask waitBitMask;
}statusValuesH;

typedef struct
{
	valuesPhysH acceleration;
	valuesPhysH speed;
	valuesPhysH position;
}midValueH;
//////////////////////
typedef struct
{
	encCountStructH encoder;
	encCountStructH encoder2;
	encCountStructH counter;
	midValueH midddleValue;
	valuesForTargetH target;
	statusValuesH status;
	valuesForCalcH calculating;

	int recievCommand[COMMAND_NUM]; // for command retrieving
}AxisParam;




char* ComPar_GetPacket(char* data, int len);
char* ComPar_GetAxis();
int ComPar_GetValue();
void AxisInit();
void ComPar_GetCommand(char *rdata, int len);

void setCommand(uint8_t axis, uint8_t command, int len);

void RunCommand(uint8_t cur_ax);
void TsCommand(uint8_t cur_ax);

uint8_t ComPar_SetComValue(uint8_t cur_ax, uint8_t cur_cmd, char* value);



#endif /* COMM_PARS_H */
