/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * Copyright (c) 2017 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f7xx_hal.h"
#include "cmsis_os.h"
#include "lwip.h"
#include "udp.h"
/* USER CODE BEGIN Includes */
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
#include "speed_control.h"
#include "encoder_control.h"
#include "status_control.h"
#include "flash_control.h"
#include "packet_builder.h"

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;

osThreadId defaultTaskHandle;
osThreadId pwmStepperHandle;
osThreadId pwmStepper1Handle;
osThreadId EncoderHandle;
osThreadId udpSenderHandle;
osThreadId ErrorComparseHandler;

/* USER CODE BEGIN PV */

/* Private variables ---------------------------------------------------------*/
extern enum
{
	error,
	ok,	
}parserState;

extern enum
{
	driving,
	monitoringP,
	monitoringS,
	monitoringN,
	getPort,
}parserComm;

extern TEthPacket ethPacket;
extern AxisParam Axx[AXIS_NUM];
extern uint8_t IP_ADDRESS[4];
uint8_t TcpFlag = 0;
//extern udpPacketH *udpPacket;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);

static void MX_TIM5_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

void StartDefaultTask(void const * argument);
void StartPwmStepper(void const * argument);  
void StartPwmStepper1(void const * argument);
void StartEncoderTask(void const * argument);     
void StartUdpSenderTask(void const * argument); 
void StartErrorComparseTask(void const * argument); 

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */
//arm_pid_instance_f32 PID[6];
/* USER CODE BEGIN 0 */



void TIM5_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */
	if(Axx[0].status.directEngine == forward)
	{
		Axx[0].counter.position.steps++;
	}
	if(Axx[0].status.directEngine == backward)
	{
		Axx[0].counter.position.steps--;
	}
  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim5);
  /* USER CODE BEGIN TIM2_IRQn 1 */

  /* USER CODE END TIM2_IRQn 1 */
}

void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */
	if(Axx[1].status.directEngine == forward)
	{
		Axx[1].counter.position.steps++;
	}
	if(Axx[1].status.directEngine == backward)
	{
		Axx[1].counter.position.steps--;
	}
  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */

  /* USER CODE END TIM2_IRQn 1 */
}

int fputc(int c, FILE *stream)
{
   return(ITM_SendChar(c));
}
/* USER CODE END 0 */

int main(void)
{
  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
	
	MX_TIM1_Init();
  MX_TIM2_Init();
	MX_TIM3_Init();
	MX_TIM4_Init();
	MX_TIM5_Init();
  /* USER CODE BEGIN 2 */
		
	HAL_TIM_Encoder_Start(&htim1,  TIM_CHANNEL_1);
	HAL_TIM_Encoder_Start(&htim3,  TIM_CHANNEL_1);
	HAL_TIM_Encoder_Start(&htim4,  TIM_CHANNEL_1);	
//	HAL_TIM_Base_Start(&htim2);
	
	HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);
	HAL_NVIC_SetPriority(TIM5_IRQn, 15, 0);
	HAL_TIM_Base_Start_IT(&htim5);
	HAL_NVIC_DisableIRQ(TIM5_IRQn);

	
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
	HAL_NVIC_SetPriority(TIM2_IRQn, 15, 0);
	HAL_TIM_Base_Start_IT(&htim2);
	HAL_NVIC_DisableIRQ(TIM2_IRQn);
//	HAL_TIM_PWM_Start_IT(&htim2, TIM_CHANNEL_1);

  /* USER CODE END 2 */
//	ax_zero = xSemaphoreCreateCounting(1, 0);
  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, /*osPriorityNormal*/ osPriorityRealtime, 0, 256);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  osThreadDef(pwmStepper, StartPwmStepper, osPriorityLow, 0, 256);
  pwmStepperHandle = osThreadCreate(osThread(pwmStepper), NULL);
	
	osThreadDef(pwmStepper1, StartPwmStepper1, osPriorityLow, 0, 256);
  pwmStepper1Handle = osThreadCreate(osThread(pwmStepper1), NULL);
	
	osThreadDef(EncoderTask, StartEncoderTask, osPriorityLow, 0, 256);
  EncoderHandle = osThreadCreate(osThread(EncoderTask), NULL);
	
	osThreadDef(ErrorComparseTask, StartErrorComparseTask, osPriorityLow, 0, 256);
  EncoderHandle = osThreadCreate(osThread(ErrorComparseTask), NULL);
	
	osThreadDef(UdpSenderTask, StartUdpSenderTask, osPriorityLow, 0, 256);
  EncoderHandle = osThreadCreate(osThread(UdpSenderTask), NULL);
  /* Start scheduler */
  osKernelStart();
  
  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 216;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

    /**Activate the Over-Drive mode 
    */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
  {
    Error_Handler();
  }

  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInitStruct.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_HSI, RCC_MCODIV_2);

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 15, 0);
}


static void MX_TIM1_Init(void)
{

  TIM_Encoder_InitTypeDef sConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 4999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 2;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 2;
  if (HAL_TIM_Encoder_Init(&htim1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_TIM_MspPostInit(&htim2);

}

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

  TIM_Encoder_InitTypeDef sConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 4999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 2;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 2;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

}

/* TIM4 init function */
static void MX_TIM4_Init(void)
{

  TIM_Encoder_InitTypeDef sConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 4999;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 2;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 2;
  if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

}

/* TIM5 init function */
static void MX_TIM5_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 0;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 65535;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_TIM_MspPostInit(&htim5);

}

/* TIM3 init function */

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, X_Dir_Pin|Y_Dir_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : X_Dir_Pin Y_Dir_Pin */
  GPIO_InitStruct.Pin = X_Dir_Pin|Y_Dir_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : X_Backward_Switcher_Pin X_Forward_Switcher_Pin Y_Left_Backward_Switcher_Pin Y_Left_Forward_Switcher_Pin 
                           Y_Right_Backward_Switcher_Pin Y_Right_Forward_Switcher_Pin */
  GPIO_InitStruct.Pin = X_Backward_Switcher_Pin|X_Forward_Switcher_Pin|Y_Left_Backward_Switcher_Pin|Y_Left_Forward_Switcher_Pin 
                          |Y_Right_Backward_Switcher_Pin|Y_Right_Forward_Switcher_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
	
	HAL_GPIO_WritePin(button_GPIO_Port, button_Pin, GPIO_PIN_RESET);
	GPIO_InitStruct.Pin = button_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(button_GPIO_Port, &GPIO_InitStruct);

}


/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* StartDefaultTask function */



void StartPwmStepper(void const * argument) //For X axis
{
	
	AxisInit();
	while(1)
	{
		if(Axx[0].status.currentCommand == run)
		{			
			Start(0);
			
			if(Axx[0].status.directEngine == forward)
			{
				RunForward(0);
			}
			
			if(Axx[0].status.directEngine == none)
			{
				Stop(0);
			}
			
			if(Axx[0].status.directEngine == backward)
			{
				RunBackward(0);
			}					
		}
		
		if(Axx[0].status.currentCommand == stop)
		{
			SlStop(0);
		}	
			
		if(Axx[0].status.currentCommand == am_stop)
		{
			ExStop(0);
			Axx[0].status.causeOfLastErrorMF = recievES;
		}	

		if(Axx[0].status.currentCommand  == goHome)
		{
			GoHome(0);
		}
//		if(Axx[0].status.currentCommand == calibration)
//		{
//			RoundCalibration(0);
//		}
		if(Axx[0].status.currentCommand != goHome && Axx[1].status.currentCommand != goHome)
		{
			if(Axx[0].status.switchers.backwardSwitcher)
		  {
				ExStop(0);
				ExStop(1);
				Axx[0].status.causeOfLastStopEM = backwardSwitcher;
	  	}
			if(Axx[0].status.switchers.forwardSwitcher)
			{
				ExStop(0);
				ExStop(1);
				Axx[0].status.causeOfLastStopEM = forwardSwitcher;				
			}
		}			
	}
	for(;;)
	{
		osDelay(1);
	}
}

void StartPwmStepper1(void const * argument)
{
	
	while(1)
	{
		if(Axx[1].status.currentCommand == run)
		{			
			Start(1);
			
			if(Axx[1].status.directEngine == forward)
			{
				RunForward(1);
			}
			else
			if(Axx[1].status.directEngine == none)
			{
				Stop(1);
			}
			else
			if(Axx[1].status.directEngine == backward)
			{
				RunBackward(1);
			}					
		}
		
		if(Axx[1].status.currentCommand == stop)
		{
			SlStop(1);
		}	

		if(Axx[1].status.currentCommand == am_stop)
		{
			ExStop(1);
			Axx[1].status.causeOfLastErrorMF = recievES;
		}	

		if(Axx[1].status.currentCommand  == goHome)
		{
			GoHome(1);
		}
//		if(Axx[1].status.currentCommand == calibration)
//		{
//			RoundCalibration(1);
//		}
		if(Axx[1].status.currentCommand != goHome && Axx[0].status.currentCommand != goHome)
		{
			if(Axx[1].status.switchers.backwardSwitcher)
		  {
				ExStop(1);
				ExStop(0);
				Axx[1].status.causeOfLastStopEM = backwardSwitcher;
	  	}
			if(Axx[1].status.switchers.forwardSwitcher)
			{
				ExStop(1);
				ExStop(0);				
				Axx[1].status.causeOfLastStopEM = forwardSwitcher;				
			}
		}
	}
	
	for(;;)
  {
    osDelay(1);
  }
}

void StartDefaultTask(void const * argument)
{
  /* init code for LWIP */

  MX_LWIP_Init();
	struct netconn *conn, *newconn;
	struct netbuf *buf;
	err_t err, accept_err, recv_err;
	void *data;
	u16_t len;

	uint16_t port = flash_getPort();
	conn = netconn_new(NETCONN_TCP);

	if(conn != NULL)
	{
		err = netconn_bind(conn, NULL, port);
		if(err == ERR_OK)
		{
			netconn_listen(conn);		
	
			while(1)
			{
				accept_err = netconn_accept(conn, &newconn);
				if(accept_err == ERR_OK)
				{			
						
					while((recv_err = netconn_recv(newconn, &buf)) == ERR_OK)
					{
						TcpFlag = 1;
						do
						{
							netbuf_data(buf, &data, &len);							
							snprintf(data, len + 1, "%s", data);
							char *buf;
							buf = ComPar_GetPacket((char*)data, len);
							netconn_write(newconn, buf, strlen(buf), NETCONN_COPY);														
						} while(netbuf_next(buf) >= 0);
						netbuf_delete(buf);													
					}
					netconn_close(newconn);
					netconn_delete(newconn);
					TcpFlag = 0;
				}
				
			}
		}
		else 
		{
			netconn_delete(newconn);
		}
	}

  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */ 
}


void StartEncoderTask(void const * argument)
{
	while(1)
	{
		GetPosition(0);
		GetPosition(1);		
		switcherStateControl();
		IpReset();

	}	
	for(;;)
  {
    osDelay(1);
  }
}	


void StartUdpSenderTask(void const * argument)
{	
	err_t err, recv_err;
	ip_addr_t server_ip, mk_ip, *addr;
	
	IP_ADDRESS[0] = flash_getNetworkAddress();
	IP_ADDRESS[1] = flash_getNetworkAddress()>>8;									
	IP_ADDRESS[2] = flash_getNetworkAddress()>>16;
	IP_ADDRESS[3] = flash_getNetworkAddress()>>24;	
	
	IP4_ADDR(&mk_ip, IP_ADDRESS[0],IP_ADDRESS[1],IP_ADDRESS[2],IP_ADDRESS[3]);
	
	IP_ADDRESS[0] = flash_getUdpTrAddress();
	IP_ADDRESS[1] = flash_getUdpTrAddress()>>8;									
	IP_ADDRESS[2] = flash_getUdpTrAddress()>>16;
	IP_ADDRESS[3] = flash_getUdpTrAddress()>>24;
	
	IP4_ADDR(&server_ip, IP_ADDRESS[0],IP_ADDRESS[1],IP_ADDRESS[2],IP_ADDRESS[3]);
	struct netconn *conn2;
	while(1)
	{
		if(TcpFlag == 1)
		{
			conn2 = netconn_new(NETCONN_UDP);
			if(conn2 != NULL)
			{
				err = netconn_bind(conn2, &mk_ip, flash_getPort());
				if(err == ERR_OK)
				{	
					udpPacketH udpData;
					struct pbuf *p2;
					while(TcpFlag)
					{								
						server_ip.addr = flash_getUdpTrAddress();
						udpData = udpPacketBuilder();
						p2 = pbuf_alloc(PBUF_TRANSPORT, sizeof(udpData), PBUF_RAM);
						memcpy(p2->payload, &udpData, sizeof(udpData));
						netconn_connect(conn2, &server_ip, 55056);			
						
						udp_sendto(conn2->pcb.udp, p2, &server_ip, flash_getUdpTrPort());	
						netconn_close(conn2);
						pbuf_free(p2);
						osDelay(flash_getUdpTrInterval());																
					}
					netconn_delete(conn2);
				}
				else
				{
					netconn_delete(conn2);
				}
			}
		}
	}
	for(;;)
  {
    osDelay(1);
  }
}

void StartErrorComparseTask(void const * argument)
{
	while(1)
	{
		if(*(__IO uint32_t*)0x40028194)
		{
			NVIC_SystemReset();	
		}
		bigAxisEncoderErrorControl();
		GetSpeed(0);		
		status_stateControl(0);
		GetSpeed(1);		
		status_stateControl(1);
		osDelay(1);
	}	
	for(;;)
  {
    osDelay(1);
  }
}	

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
