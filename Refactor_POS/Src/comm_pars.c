#include "comm_pars.h"
#include <string.h>
#include <stdio.h>
#include <ctype.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdlib.h>
#include "stm32f7xx_hal.h"
#include <math.h>
#include "status_control.h"
#include "convertor.h"
#include "flash_control.h"
//#include "posicioner_config.h"
//#include "pars_config.h"







char *ax[AXIS_NUM + 1] = {"X", "Y", "A"};	
char *com[COMMAND_NUM] = {"SP", "AC", "DC", "AP", "BG", "ST", "CL", "AB", "GP", "GS", "AO", "PP", "GH", "NA", "NP", "UA", "UP", "UI", "PS", "VL", "DP", "PE", "MF", "MS", "EM"};

char equalsymb = '=';
char endSymbol = ';';

//uint8_t axisForMain = 0;
enum
{
	error,
	ok,	
}parserState;

//TEthPacket ethPacket;
AxisParam Axx[AXIS_NUM];

TEthPacket ethPacket;
char recvDatta[256];

char* ComPar_GetPacket(char* data, int len)
{	
	
	memset(recvDatta, 0, strlen(recvDatta));
	char *lastAdr = 0;
	char *curComm = 0;
	int comLen = 0;
	lastAdr = strchr(data, ';');
	if(lastAdr != 0)
	{
		while(lastAdr != 0)
		{
			snprintf(curComm,strlen(data) - strlen(lastAdr) + 2,"%s;", data);
			sprintf(data,"%s", lastAdr + 1);
			comLen += strlen(curComm);
			ComPar_GetCommand(curComm, strlen(curComm));
			lastAdr = strchr(data, ';');	
		}
	}
	else sprintf(recvDatta, "%s?>;", recvDatta);		
	if(comLen < len) sprintf(recvDatta, "%s?>;", recvDatta);		
	return recvDatta;
}

void ComPar_GetCommand(char *rdata, int len)
{
	memset(&ethPacket, 0, sizeof(ethPacket));
	strncpy((char * )&ethPacket, rdata, len);		//perenosim dannye v strukturu	
	ethPacket.endSymb = rdata[len - 1];
	uint8_t cur_ax;
	if(strncmp(rdata, "RESET;", strlen("RESET;")) == 0)
	{
		NVIC_SystemReset();
	}
	else
	for(cur_ax = 0; cur_ax < AXIS_NUM + 1; cur_ax++)//sravnenie osey
	{
		if(strncmp(ethPacket.ethAxis, ax[cur_ax], SYMBOL_NUM_AXIS) == 0) //funkciya sravneniya. Esli 0, to takaya os' est'
		{			
			parserState = ok;
			uint8_t cur_cmd;
			for(cur_cmd = 0; cur_cmd < COMMAND_NUM; cur_cmd++)//sravnenie komand
			{
				if(strncmp(ethPacket.ethCmd, com[cur_cmd], SYMBOL_NUM_CMD) == 0) //funkciya sravneniya. Esli 0, to takaya komada est'
				{
					parserState = ok;
					if(ethPacket.endSymb == endSymbol) //est li v konce ";"
					{
						parserState = ok;
						setCommand(cur_ax, cur_cmd, len);
					} 
					else parserState = error;
					return;
				}
				else parserState = error;
			}
		}	
		else parserState = error;
	}
	if(parserState == error) sprintf(recvDatta, "%s?>;", recvDatta);			
}


void AxisInit()
{
	int i = 0;
	

	
	for(i = 0; i < AXIS_NUM; i ++)
	{
		memset(&Axx[i], 0, sizeof(Axx[i]));
		
		Axx[0].calculating.encoderKoef = 3.96679997;
		Axx[0].calculating.mmToStepKoeff = 0.0134445392;
	
		Axx[1].calculating.encoderKoef = 3.07775545;
		Axx[1].calculating.mmToStepKoeff = 0.0173281468;
	
		Axx[1].calculating.encoder2Koef = 3.06180191;
		Axx[1].calculating.mmToStep2Koeff = 0.0174184348;
		
		Axx[i].status.causeOfLastErrorMF = 0;
		Axx[i].status.causeOfLastStopEM = 1;
		Axx[i].status.currentEngineStatusMS.stop = 1;
		
		Axx[i].target.acceleration.backwardLimit.mm = MIN_SETS_ACCELERATION;
		Axx[i].target.acceleration.forwardLimit.mm = MAX_SETS_ACCELERATION;
		Axx[i].target.deceleration.backwardLimit.mm = MIN_SETS_DECELERATION;
		Axx[i].target.deceleration.forwardLimit.mm = MAX_SETS_DECELERATION;
		Axx[i].target.speed.backwardLimit.mm = 10;
		Axx[i].target.speed.forwardLimit.mm = 2000;
		Axx[i].target.position.backwardLimit.mm = MIN_SETS_POSITION;
		Axx[i].target.position.forwardLimit.mm = MAX_SETS_POSITION;
		
		Axx[i].target.acceleration.backwardLimit.steps = mmToSteps(Axx[i].target.acceleration.backwardLimit.mm, i);
		Axx[i].target.acceleration.forwardLimit.steps = mmToSteps(Axx[i].target.acceleration.forwardLimit.mm, i);
		Axx[i].target.deceleration.backwardLimit.steps = mmToSteps(Axx[i].target.deceleration.backwardLimit.mm, i);
		Axx[i].target.deceleration.forwardLimit.steps = mmToSteps(Axx[i].target.deceleration.forwardLimit.mm, i);
		Axx[i].target.speed.backwardLimit.steps = mmToSteps(Axx[i].target.speed.backwardLimit.mm, i);
		Axx[i].target.speed.forwardLimit.steps = mmToSteps(Axx[i].target.speed.forwardLimit.mm, i);
		Axx[i].target.position.backwardLimit.steps = mmToSteps(Axx[i].target.position.backwardLimit.mm, i);
		Axx[i].target.position.forwardLimit.steps = mmToSteps(Axx[i].target.position.forwardLimit.mm, i);
		
		Axx[i].target.admitErrorPosition.mm = ADMIT_ERROR_POSITION;
		Axx[i].target.admitErrorPosition.steps = mmToSteps(Axx[i].target.admitErrorPosition.mm, i);
		Axx[i].calculating.secondCount = 1;
	}	
	

}


char* ComPar_GetAxis()
{
	return ethPacket.ethAxis;
}

int ComPar_GetValue()
{
		return atoi(ethPacket.ethValue);
}

void setCommand(uint8_t cur_ax,uint8_t cur_cmd, int len)
{
	char newValue1[32] = {0};
	char newValue2[32] = {0};
	char newValue3[32] = {0};
	char newValue4[32] = {0};
	
	uint8_t i = 0;

//	ethPacket.ethValue[len-5] = '\0';
	
	char recVal[128] = {0};
	memset(recVal, 0, 128);
	memcpy(recVal, ethPacket.ethValue, sizeof(ethPacket.ethValue));
	int symbCount;
	char *lastAdrSym = 0;
	int lastValue;
	
	lastAdrSym = strchr(recVal, ',');
	if(lastAdrSym != 0)
	{
		snprintf(newValue1, strlen(recVal) - strlen(lastAdrSym) + 1, "%s", recVal);			
		sprintf(recVal, "%s", lastAdrSym + 1);				
		lastAdrSym = strchr(recVal, ',');
		if(lastAdrSym != 0)
		{
			snprintf(newValue2, strlen(recVal) - strlen(lastAdrSym) + 1, "%s", recVal);			
			sprintf(recVal, "%s", lastAdrSym + 1);				
			lastAdrSym = strchr(recVal, ',');
			if(lastAdrSym != 0)
			{
				snprintf(newValue3, strlen(recVal) - strlen(lastAdrSym) + 1, "%s", recVal);			
				sprintf(recVal, "%s", lastAdrSym + 1);				
				snprintf(newValue4, strlen(recVal), "%s", recVal);			
			}			
			else
			{
				snprintf(newValue3, strlen(recVal), "%s", recVal);						
			}						
		}			
		else
		{
			snprintf(newValue2, strlen(recVal), "%s", recVal);			
		}
	}
	else
	{
		snprintf(newValue1, strlen(recVal), "%s", recVal);			
	}
	
	
	if(cur_ax < 2)
	{
		if(ethPacket.ethSymb == equalsymb)
		{
			ethPacket.ethValue[len-5] = '\0';
			int symbCount;
			char *lastAdrSym = 0;
			int lastValue;
			if(cur_cmd == ao)
			{
				if(ComPar_SetComValue(cur_ax, 0, newValue1) && ComPar_SetComValue(cur_ax, 1, newValue2) &&	ComPar_SetComValue(cur_ax, 2, newValue3) && ComPar_SetComValue(cur_ax, 3, newValue4))
				{
					sprintf(recvDatta, "%s>;", recvDatta);
				}
				else sprintf(recvDatta, "%s?>;", recvDatta);
			}
			else
			if(cur_cmd == pp)
			{
				if(ComPar_SetComValue(0, 3, newValue1) && ComPar_SetComValue(1, 3, newValue2))
				{
					sprintf(recvDatta, "%s>;", recvDatta);
				}
				else sprintf(recvDatta, "%s?>;", recvDatta);
			}
			else
			if(cur_cmd == na)
			{
				uint8_t val[4];
				for(symbCount = 0; symbCount < 3; symbCount++)
				{
					lastAdrSym = strrchr(ethPacket.ethValue, '.');
					if(lastAdrSym != 0)
					{
						snprintf(lastAdrSym, strlen(lastAdrSym), "%s", lastAdrSym + 1);
						val[3 - symbCount] = atoi(lastAdrSym);					
						snprintf(ethPacket.ethValue, strlen(ethPacket.ethValue) - strlen(lastAdrSym) + 1, "%s", ethPacket.ethValue);				
					}
					else
					{
						parserState = error;
						return;
					}			
				}
				val[0] = atoi(ethPacket.ethValue);		
				if(flash_setNetworkAddress(val[0], val[1], val[2], val[3]))
				{
					sprintf(recvDatta, "%s>;", recvDatta);
				}
				else sprintf(recvDatta, "%s?>;", recvDatta);
			}
			else
			if(cur_cmd == np)
			{
				if(flash_setPort((uint16_t)atoi(ethPacket.ethValue)))
				{					
					sprintf(recvDatta, "%s>;", recvDatta);
				}
				else sprintf(recvDatta, "%s?>;", recvDatta);
			}
			else
			if(cur_cmd == ua)
			{
				uint8_t val[4];
				for(symbCount = 0; symbCount < 3; symbCount++)
				{
					lastAdrSym = strrchr(ethPacket.ethValue, '.');
					if(lastAdrSym != 0)
					{
						snprintf(lastAdrSym, strlen(lastAdrSym), "%s", lastAdrSym + 1);
						val[3 - symbCount] = atoi(lastAdrSym);					
						snprintf(ethPacket.ethValue, strlen(ethPacket.ethValue) - strlen(lastAdrSym) + 1, "%s", ethPacket.ethValue);				
					}
					else
					{
						parserState = error;
						return;
					}			
				}
				val[0] = atoi(ethPacket.ethValue);		
				if(flash_setUdpTrAddress(val[0], val[1], val[2], val[3]))
				{
					sprintf(recvDatta, "%s>;", recvDatta);
				}
				else sprintf(recvDatta, "%s?>;", recvDatta);
			}
			else
			if(cur_cmd == up)
			{
				if(flash_setUdpTrPort((uint16_t)atoi(ethPacket.ethValue)))
				{
					sprintf(recvDatta, "%s>;", recvDatta);
				}
				else sprintf(recvDatta, "%s?>;", recvDatta);
			}
			else
			if(cur_cmd == ui)
			{
				if(flash_setUdpTrInterval((uint16_t)atoi(ethPacket.ethValue)))
				{
					sprintf(recvDatta, "%s>;", recvDatta);
				}
				else sprintf(recvDatta, "%s?>;", recvDatta);
			}
			else
			if(cur_cmd == sp)
			{
				if(ComPar_SetComValue(cur_ax, 0, newValue1))
				{
					sprintf(recvDatta, "%s>;", recvDatta);
				}
				else sprintf(recvDatta, "%s?>;", recvDatta);
			}
			else
			if(cur_cmd == ac)
			{
				if(ComPar_SetComValue(cur_ax, 1, newValue1))
				{
					sprintf(recvDatta, "%s>;", recvDatta);
				}
				else sprintf(recvDatta, "%s?>;", recvDatta);
			}
			else
			if(cur_cmd == dc)
			{
				if(ComPar_SetComValue(cur_ax, 2, newValue1))
				{
					sprintf(recvDatta, "%s>;", recvDatta);
				}
				else sprintf(recvDatta, "%s?>;", recvDatta);
			}
			else
			if(cur_cmd == ap)
			{
				if(ComPar_SetComValue(cur_ax, 3, newValue1))
				{
					sprintf(recvDatta, "%s>;", recvDatta);
				}
				else sprintf(recvDatta, "%s?>;", recvDatta);
			}	
			else sprintf(recvDatta, "%s?>;", recvDatta);
			//else ComPar_SetComValue(cur_ax, cur_cmd, ethPacket.ethValue);
		}
		else
		if(ethPacket.ethSymb == endSymbol)
		{
			parserState = ok;
			switch(cur_cmd)
			{
				case bg:
					if(Axx[cur_ax].status.currentEngineStatusMS.wait && Axx[cur_ax].status.currentEngineStatusMS.calibration)
					{
						RunCommand(cur_ax);			
						sprintf(recvDatta, "%s>;", recvDatta);
						Axx[cur_ax].status.currentCommand = run;
						Axx[cur_ax].status.currentSpeedStateOfEngine = accelerate;					
					}		
					else sprintf(recvDatta, "%s?>;", recvDatta);
					break;	
				case st:
					Axx[cur_ax].status.currentCommand = stop;
					sprintf(recvDatta, "%s>;", recvDatta);
					break;
				case ab:
					Axx[cur_ax].status.currentCommand = am_stop;
					sprintf(recvDatta, "%s>;", recvDatta);
					break;			
				case gp:
					sprintf(recvDatta, "%s%d,%d,%d,%d,%d,%d;",recvDatta, status_GetLastStopCause(cur_ax), status_GetCurrentPosition(cur_ax), status_GetCurrentSpeed(cur_ax), status_GetAimPosition(cur_ax), status_GetCurrentErrorPosition(cur_ax), status_GetCurrentMotionStateMask(cur_ax));
					break;
				case gs:
					sprintf(recvDatta, "%s%d;", recvDatta, status_GetLastError(cur_ax));
					break;
				case gh:
					Axx[cur_ax].status.currentCommand = goHome;
					sprintf(recvDatta, "%s>;", recvDatta);
					break;
				case cl:
					Axx[cur_ax].status.currentCommand = calibration;
					sprintf(recvDatta, "%s>;", recvDatta);
					break;
				case na:			
					sprintf(recvDatta, "%s%d.%d.%d.%d;",recvDatta, (uint8_t)flash_getNetworkAddress(), (uint8_t)(flash_getNetworkAddress()>>8), (uint8_t)(flash_getNetworkAddress()>>16), (uint8_t)(flash_getNetworkAddress()>>24));
					break;
				case np:
					sprintf(recvDatta, "%s%d;", recvDatta, flash_getPort());
					break;				
				case ua:
					sprintf(recvDatta, "%s%d.%d.%d.%d;",recvDatta, (uint8_t)flash_getUdpTrAddress(), (uint8_t)(flash_getUdpTrAddress()>>8), (uint8_t)(flash_getUdpTrAddress()>>16), (uint8_t)(flash_getUdpTrAddress()>>24));
					break;
				case up:
					sprintf(recvDatta, "%s%d;", recvDatta, flash_getUdpTrPort());
					break;
				case ui:
					sprintf(recvDatta, "%s%d;", recvDatta, flash_getUdpTrInterval());
					break;					
				case ps:
					sprintf(recvDatta, "%s%d;", recvDatta, status_GetCurrentPosition(cur_ax));
					break;
				case vl:
					sprintf(recvDatta, "%s%d;", recvDatta, status_GetCurrentSpeed(cur_ax));
					break;
				case dp:
					sprintf(recvDatta, "%s%d;", recvDatta, status_GetAimPosition(cur_ax));
					break;
				case pe:
					sprintf(recvDatta, "%s%d;", recvDatta, status_GetCurrentErrorPosition(cur_ax));
					break;
				case mf:
					sprintf(recvDatta, "%s%d;", recvDatta, status_GetLastError(cur_ax));
					break;			
				case ms:
					sprintf(recvDatta, "%s%d;", recvDatta, status_GetCurrentMotionStateMask(cur_ax));
					break;
				case em:
					sprintf(recvDatta, "%s%d;", recvDatta, status_GetLastStopCause(cur_ax));
					break;	
				default:
					parserState = error;
					sprintf(recvDatta, "%s?>;", recvDatta);
					break;
			}
		}
		else 
		{
			parserState = error;
			sprintf(recvDatta, "%s?>;", recvDatta);
		}
	}
	else
		if(cur_ax == 2)
		{
			if(ethPacket.ethSymb == equalsymb)
			{						
				if(cur_cmd == ao)
				{
					if(ComPar_SetComValue(0, 0, newValue1)		
					&& ComPar_SetComValue(0, 1, newValue2)
					&& ComPar_SetComValue(0, 2, newValue3)			
					&& ComPar_SetComValue(0, 3, newValue4)
					&& ComPar_SetComValue(1, 0, newValue1)			
					&& ComPar_SetComValue(1, 1, newValue2)
					&& ComPar_SetComValue(1, 2, newValue3)			
					&& ComPar_SetComValue(1, 3, newValue4))
					{
						sprintf(recvDatta, "%s>;", recvDatta);
					}
					else sprintf(recvDatta, "%s?>;", recvDatta);
				}
				else
				if(cur_cmd == pp)
				{
					if(ComPar_SetComValue(0, 3, newValue1)		
					&& ComPar_SetComValue(0, 3, newValue2)
					&& ComPar_SetComValue(1, 3, newValue1)			
					&& ComPar_SetComValue(1, 3, newValue2))
					{
						sprintf(recvDatta, "%s>;", recvDatta);
					}
					else sprintf(recvDatta, "%s?>;", recvDatta);
				}
				else
				if(cur_cmd == na)
				{
					uint8_t val[4];
					for(symbCount = 0; symbCount < 3; symbCount++)
					{
						lastAdrSym = strrchr(ethPacket.ethValue, '.');
						if(lastAdrSym != 0)
						{
							snprintf(lastAdrSym, strlen(lastAdrSym), "%s", lastAdrSym + 1);
							val[3 - symbCount] = atoi(lastAdrSym);					
							snprintf(ethPacket.ethValue, strlen(ethPacket.ethValue) - strlen(lastAdrSym) + 1, "%s", ethPacket.ethValue);				
						}
						else
						{
							parserState = error;
							return;
						}			
					}
					val[0] = atoi(ethPacket.ethValue);		
					if(flash_setNetworkAddress(val[0], val[1], val[2], val[3]))
					{
						sprintf(recvDatta, "%s>;", recvDatta);
					}
					else sprintf(recvDatta, "%s?>;", recvDatta);
				}
				else
				if(cur_cmd == np)
				{
					if(flash_setPort((uint16_t)atoi(ethPacket.ethValue)))
					{
						sprintf(recvDatta, "%s>;", recvDatta);
					}
					else sprintf(recvDatta, "%s?>;", recvDatta);
				}
				else
				if(cur_cmd == ua)
				{
					uint8_t val[4];
					for(symbCount = 0; symbCount < 3; symbCount++)
					{
						lastAdrSym = strrchr(ethPacket.ethValue, '.');
						if(lastAdrSym != 0)
						{
							snprintf(lastAdrSym, strlen(lastAdrSym), "%s", lastAdrSym + 1);
							val[3 - symbCount] = atoi(lastAdrSym);					
							snprintf(ethPacket.ethValue, strlen(ethPacket.ethValue) - strlen(lastAdrSym) + 1, "%s", ethPacket.ethValue);				
						}
						else
						{
							parserState = error;
							return;
						}			
					}
					val[0] = atoi(ethPacket.ethValue);		
					if(flash_setUdpTrAddress(val[0], val[1], val[2], val[3]))
					{
						sprintf(recvDatta, "%s>;", recvDatta);
					}
					else sprintf(recvDatta, "%s?>;", recvDatta);
				}
				else
				if(cur_cmd == up)
				{
					if(flash_setUdpTrPort((uint16_t)atoi(ethPacket.ethValue)))
					{
						sprintf(recvDatta, "%s>;", recvDatta);
					}
					else sprintf(recvDatta, "%s?>;", recvDatta);
				}
				else
				if(cur_cmd == ui)
				{
					if(flash_setUdpTrInterval((uint16_t)atoi(ethPacket.ethValue)))
					{
						sprintf(recvDatta, "%s>;", recvDatta);
					}
					else sprintf(recvDatta, "%s?>;", recvDatta);
				}
				else
				if(cur_cmd == sp)
				{
					if(ComPar_SetComValue(0, 0, newValue1) && ComPar_SetComValue(1, 0, newValue2))
					{
						sprintf(recvDatta, "%s>;", recvDatta);
					}
					else sprintf(recvDatta, "%s?>;", recvDatta);
				}
				else
				if(cur_cmd == ac)
				{
					if(ComPar_SetComValue(0, 1, newValue1) && ComPar_SetComValue(1, 1, newValue2))
					{
						sprintf(recvDatta, "%s>;", recvDatta);
					}
					else sprintf(recvDatta, "%s?>;", recvDatta);
				}
				else
				if(cur_cmd == dc)
				{
					if(ComPar_SetComValue(0, 2, newValue1) && ComPar_SetComValue(1, 2, newValue2))
					{
						sprintf(recvDatta, "%s>;", recvDatta);
					}
					else sprintf(recvDatta, "%s?>;", recvDatta);
				}
				else
				if(cur_cmd == ap)
				{
					if(ComPar_SetComValue(0, 3, newValue1) && ComPar_SetComValue(1, 3, newValue2))
					{
						sprintf(recvDatta, "%s>;", recvDatta);
					}
					else sprintf(recvDatta, "%s?>;", recvDatta);
				}	
				else sprintf(recvDatta, "%s?>;", recvDatta);
			}
			else
			if(ethPacket.ethSymb == endSymbol)
			{
				parserState = ok;
				switch(cur_cmd)
				{
					case bg:
						if(Axx[0].status.currentEngineStatusMS.wait && Axx[1].status.currentEngineStatusMS.wait && Axx[0].status.currentEngineStatusMS.calibration && Axx[1].status.currentEngineStatusMS.calibration)/////!!!!!!!!!!!
						{
							RunCommand(0);			
							RunCommand(1);	
							sprintf(recvDatta, "%s>;", recvDatta);
							Axx[0].status.currentCommand = run;
							Axx[0].status.currentSpeedStateOfEngine = accelerate;					
							Axx[1].status.currentCommand = run;
							Axx[1].status.currentSpeedStateOfEngine = accelerate;		
						}						
						else sprintf(recvDatta, "%s?>;", recvDatta);
						break;	
					case st:
						Axx[0].status.currentCommand = stop;
						Axx[1].status.currentCommand = stop;
						sprintf(recvDatta, "%s>;", recvDatta);
						break;
					case ab:
						Axx[0].status.currentCommand = am_stop;
						Axx[1].status.currentCommand = am_stop;
						sprintf(recvDatta, "%s>;", recvDatta);
						break;			
					case gp:
						sprintf(recvDatta, "%s%d,%d,%d,%d,%d,%d;",recvDatta, status_GetLastStopCause(0), status_GetCurrentPosition(0), status_GetCurrentSpeed(0), status_GetAimPosition(0), status_GetCurrentErrorPosition(0), status_GetCurrentMotionStateMask(0));
						sprintf(recvDatta, "%s%d,%d,%d,%d,%d,%d;",recvDatta, status_GetLastStopCause(1), status_GetCurrentPosition(1), status_GetCurrentSpeed(1), status_GetAimPosition(1), status_GetCurrentErrorPosition(1), status_GetCurrentMotionStateMask(1));
					break;
					case gs:
						sprintf(recvDatta, "%s%d>", recvDatta, status_GetLastError(0));
						sprintf(recvDatta, "%s%d>", recvDatta, status_GetLastError(1));
						break;
					case gh:
						Axx[0].status.currentCommand = goHome;
						Axx[1].status.currentCommand = goHome;
						sprintf(recvDatta, "%s>;", recvDatta);
						break;
					case cl:
						Axx[0].status.currentCommand = calibration;
						Axx[1].status.currentCommand = calibration;
						sprintf(recvDatta, "%s>;", recvDatta);
						break;
					case na:			
						sprintf(recvDatta, "%s%d.%d.%d.%d;",recvDatta, (uint8_t)flash_getNetworkAddress(), (uint8_t)(flash_getNetworkAddress()>>8), (uint8_t)(flash_getNetworkAddress()>>16), (uint8_t)(flash_getNetworkAddress()>>24));
						break;
					case np:
						sprintf(recvDatta, "%s%d;", recvDatta, flash_getPort());
						break;
					case ua:
						sprintf(recvDatta, "%s%d.%d.%d.%d;",recvDatta, (uint8_t)flash_getUdpTrAddress(), (uint8_t)(flash_getUdpTrAddress()>>8), (uint8_t)(flash_getUdpTrAddress()>>16), (uint8_t)(flash_getUdpTrAddress()>>24));
						break;
					case up:
						sprintf(recvDatta, "%s%d;", recvDatta, flash_getUdpTrPort());
						break;
					case ui:
						sprintf(recvDatta, "%s%d;", recvDatta, flash_getUdpTrInterval());
						break;
					case ps:
						sprintf(recvDatta, "%s%d,", recvDatta, status_GetCurrentPosition(0));
						sprintf(recvDatta, "%s%d;>", recvDatta, status_GetCurrentPosition(1));
						break;
					case vl:
						sprintf(recvDatta, "%s%d,", recvDatta, status_GetCurrentSpeed(0));
						sprintf(recvDatta, "%s%d;>", recvDatta, status_GetCurrentSpeed(1));
						break;
					case dp:
						sprintf(recvDatta, "%s%d,", recvDatta, status_GetAimPosition(0));
						sprintf(recvDatta, "%s%d;>", recvDatta, status_GetAimPosition(1));
						break;
					case pe:
						sprintf(recvDatta, "%s%d,", recvDatta, status_GetCurrentErrorPosition(0));
						sprintf(recvDatta, "%s%d;>", recvDatta, status_GetCurrentErrorPosition(1));
						break;
					case mf:
						sprintf(recvDatta, "%s%d,", recvDatta, status_GetLastError(0));
						sprintf(recvDatta, "%s%d;>", recvDatta, status_GetLastError(1));
						break;			
					case ms:
						sprintf(recvDatta, "%s%d,", recvDatta, status_GetCurrentMotionStateMask(0));
						sprintf(recvDatta, "%s%d;>", recvDatta, status_GetCurrentMotionStateMask(1));
						break;
					case em:
						sprintf(recvDatta, "%s%d,", recvDatta, status_GetLastStopCause(0));
						sprintf(recvDatta, "%s%d;>", recvDatta, status_GetLastStopCause(1));
						break;	
					default:
						parserState = error;
						sprintf(recvDatta, "%s?>;", recvDatta);
						break;
				}
			}
			else 
			{
				parserState = error;
				sprintf(recvDatta, "%s?>;", recvDatta);
			}
			
		}
}



void RunCommand(uint8_t cur_ax)
{

	Axx[cur_ax].calculating.startPosition.steps = Axx[cur_ax].midddleValue.position.steps;
	Axx[cur_ax].calculating.startPosition.mm = stepsToMm(Axx[cur_ax].calculating.startPosition.steps, cur_ax);
	
	Axx[cur_ax].target.position.setsValue.mm = (float)(Axx[cur_ax].recievCommand[ap] / 100.0);
	Axx[cur_ax].target.position.setsValue.steps = mmToSteps(Axx[cur_ax].target.position.setsValue.mm, cur_ax);
	
	Axx[cur_ax].target.speed.setsValue.mm = (float)(Axx[cur_ax].recievCommand[sp]/100.0);							
	Axx[cur_ax].target.speed.setsValue.steps = mmToSteps(Axx[cur_ax].target.speed.setsValue.mm, cur_ax);
	
	Axx[cur_ax].target.acceleration.setsValue.mm = (float)(Axx[cur_ax].recievCommand[ac]/100.0);
	Axx[cur_ax].target.acceleration.setsValue.steps = mmToSteps(Axx[cur_ax].target.acceleration.setsValue.mm, cur_ax);
	
	Axx[cur_ax].target.deceleration.setsValue.mm = (float)(Axx[cur_ax].recievCommand[dc]/100.0);
	Axx[cur_ax].target.deceleration.setsValue.steps = mmToSteps(Axx[cur_ax].target.deceleration.setsValue.mm, cur_ax);
	

	Axx[cur_ax].calculating.distance.steps = fabs(Axx[cur_ax].target.position.setsValue.steps - Axx[cur_ax].calculating.startPosition.steps);
	Axx[cur_ax].calculating.distance.mm = stepsToMm(Axx[cur_ax].calculating.distance.steps, cur_ax);
	if(Axx[cur_ax].target.position.setsValue.steps > Axx[cur_ax].calculating.startPosition.steps)
	{
		Axx[cur_ax].status.directEngine = forward;
		switch(cur_ax)
		{
			case 0:
				AXIS_X_FORWARD_DIR;
				break;
			case 1:
				AXIS_Y1_FORWARD_DIR;
				break;
		}
	}
	else 
	if(Axx[cur_ax].target.position.setsValue.steps < Axx[cur_ax].calculating.startPosition.steps)
	{
		Axx[cur_ax].status.directEngine = backward;
		switch(cur_ax)
		{
			case 0:
				AXIS_X_BACKWARD_DIR;
				break;
			case 1:
				AXIS_Y1_BACKWARD_DIR;
				break;
		}
	}
	else 
	if(Axx[cur_ax].target.position.setsValue.steps == Axx[cur_ax].calculating.startPosition.steps)
	{
		Axx[cur_ax].status.directEngine = none;
	}

	Axx[cur_ax].calculating.deltaSpeed	= Axx[cur_ax].target.speed.setsValue.steps - TIMER_FREQUENCY/MAX_PERIOD;
	Axx[cur_ax].calculating.stepsToAc = (Axx[cur_ax].target.speed.setsValue.steps *  Axx[cur_ax].target.speed.setsValue.steps - TIMER_FREQUENCY / MAX_PERIOD * TIMER_FREQUENCY / MAX_PERIOD) / (2 * Axx[cur_ax].target.acceleration.setsValue.steps);
	Axx[cur_ax].calculating.stepsToDc = (Axx[cur_ax].target.speed.setsValue.steps * Axx[cur_ax].target.speed.setsValue.steps - TIMER_FREQUENCY / MAX_PERIOD * TIMER_FREQUENCY / MAX_PERIOD) / (2 * Axx[cur_ax].target.deceleration.setsValue.steps);
	if((Axx[cur_ax].calculating.stepsToAc + Axx[cur_ax].calculating.stepsToDc) >= Axx[cur_ax].calculating.distance.steps)
	{
		Axx[cur_ax].calculating.stepsToAc = Axx[cur_ax].calculating.startPosition.steps + (Axx[cur_ax].status.directEngine - 1) * Axx[cur_ax].calculating.distance.steps*Axx[cur_ax].target.deceleration.setsValue.steps / (Axx[cur_ax].target.deceleration.setsValue.steps + Axx[cur_ax].target.acceleration.setsValue.steps);
		Axx[cur_ax].calculating.stepsToDc = Axx[cur_ax].target.position.setsValue.steps - (Axx[cur_ax].status.directEngine - 1) * Axx[cur_ax].calculating.distance.steps*Axx[cur_ax].target.acceleration.setsValue.steps / (Axx[cur_ax].target.deceleration.setsValue.steps + Axx[cur_ax].target.deceleration.setsValue.steps);
	}
	else
	{
		Axx[cur_ax].calculating.stepsToAc = Axx[cur_ax].calculating.startPosition.steps + (Axx[cur_ax].status.directEngine - 1) * Axx[cur_ax].calculating.stepsToAc;
		Axx[cur_ax].calculating.stepsToDc = Axx[cur_ax].target.position.setsValue.steps -(Axx[cur_ax].status.directEngine - 1) * Axx[cur_ax].calculating.stepsToDc;
	}
	
	Axx[cur_ax].calculating.deltaAc = (float)((float)Axx[cur_ax].target.acceleration.setsValue.steps/1000.0);//(float)((float)Axx[cur_ax].acceleration/1000.0)/32*DRIVER_PRESCALLER*0.6344;//*0.4758;//
	Axx[cur_ax].calculating.deltaDc = (float)((float)Axx[cur_ax].target.deceleration.setsValue.steps/1000.0);//*0.4758;//(float)((float)Axx[cur_ax].deceleration/1000.0)/32*DRIVER_PRESCALLER*0.6344;//*0.4758;//
	Axx[cur_ax].calculating.next_speed = TIMER_FREQUENCY/MAX_PERIOD;//Axx[cur_ax].target.speed.backwardLimit.steps;
}

uint8_t ComPar_SetComValue(uint8_t cur_ax, uint8_t curCmd, char* value)
{
	int symbCount;
	for(symbCount = 0; symbCount < strlen(value); symbCount++)
	{
		if(isdigit(value[symbCount]) == 1)
		{
			parserState = ok;
		}
		else 
		{
			parserState = error;
			return 0;
		}
	}
	
	if(parserState == ok)
	{	
		Axx[cur_ax].recievCommand[curCmd] = atoi(value);
		waitSetBit(cur_ax, curCmd);
		return 1;
	}
}