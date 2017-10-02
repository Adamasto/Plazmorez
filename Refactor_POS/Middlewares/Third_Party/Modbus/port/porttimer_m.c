/*
 * FreeModbus Libary: RT-Thread Port
 * Copyright (C) 2013 Armink <armink.ztl@gmail.com>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 * File: $Id: porttimer_m.c,v 1.60 2013/08/13 15:07:05 Armink add Master Functions$
 */

/* ----------------------- Platform includes --------------------------------*/
#include "port.h"

/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mb_m.h"
#include "mbport.h"

#include "stm32f7xx_hal.h"

extern TIM_HandleTypeDef htim6;

#if MB_MASTER_RTU_ENABLED > 0 || MB_MASTER_ASCII_ENABLED > 0
/* ----------------------- Variables ----------------------------------------*/
static USHORT usT35TimeOut50us;

static void prvvTIMERExpiredISR(void);
static void timer_timeout_ind(void* parameter);

/* ----------------------- static functions ---------------------------------*/
static void prvvTIMERExpiredISR(void);

/* ----------------------- Start implementation -----------------------------*/
BOOL xMBMasterPortTimersInit(USHORT usTimeOut50us)
{
TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

	HAL_TIM_Base_Stop(&htim6);
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 1080;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = (uint32_t)5 * usTimeOut50us -1;
  htim6.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_Base_Init(&htim6);

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  HAL_TIM_ConfigClockSource(&htim6, &sClockSourceConfig);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig);

  return TRUE;
}

void vMBMasterPortTimersT35Enable()
{
    /* Enable the timer with the timeout passed to xMBPortTimersInit( ) */
	vMBMasterSetCurTimerMode(MB_TMODE_T35);
	__HAL_TIM_SET_AUTORELOAD(&htim6, 5*70);
	__HAL_TIM_SET_COUNTER(&htim6, 0);
	HAL_TIM_Base_Start_IT(&htim6);
}

void vMBMasterPortTimersConvertDelayEnable()
{
	 vMBMasterSetCurTimerMode(MB_TMODE_CONVERT_DELAY);
	 __HAL_TIM_SET_AUTORELOAD(&htim6, MB_MASTER_DELAY_MS_CONVERT*100);
	 __HAL_TIM_SET_COUNTER(&htim6, 0);
	 HAL_TIM_Base_Start_IT(&htim6);
}

void vMBMasterPortTimersRespondTimeoutEnable()
{
	 vMBMasterSetCurTimerMode(MB_TMODE_RESPOND_TIMEOUT);
	__HAL_TIM_SET_AUTORELOAD(&htim6, MB_MASTER_TIMEOUT_MS_RESPOND*100);
	 __HAL_TIM_SET_COUNTER(&htim6, 0);
	 HAL_TIM_Base_Start_IT(&htim6);
}

void vMBMasterPortTimersDisable()
{
    /* Disable any pending timers. */
	HAL_TIM_Base_Stop_IT(&htim6);
	__HAL_TIM_SET_COUNTER(&htim6, 0);
  __HAL_TIM_CLEAR_IT(&htim6, TIM_IT_UPDATE);
}

//void prvvTIMERExpiredISR(void)
//{
//    (void) pxMBMasterPortCBTimerExpired();
//}


//void HAL_TIM_PeriodElapsedCallback ( TIM_HandleTypeDef * htimer)
//{
//	switch((uint32_t)(htimer->Instance)){
//		case (uint32_t)TIM6:
//			prvvTIMERExpiredISR();
//			break;
//	}
//}

#endif
