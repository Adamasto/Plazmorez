/**
  ******************************************************************************
  * File Name          : main.h
  * Description        : This file contains the common defines of the application
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H
  /* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/
#define X_PWM_Pin GPIO_PIN_0
#define X_PWM_GPIO_Port GPIOA
#define Y_LEFT_PWM_Pin GPIO_PIN_5
#define Y_LEFT_PWM_GPIO_Port GPIOA
#define X_ENCODER_B_Pin GPIO_PIN_9
#define X_ENCODER_B_GPIO_Port GPIOE
#define X_ENCODER_A_Pin GPIO_PIN_11
#define X_ENCODER_A_GPIO_Port GPIOE
#define X_Dir_Pin GPIO_PIN_12
#define X_Dir_GPIO_Port GPIOF
#define Y_Dir_Pin GPIO_PIN_13
#define Y_Dir_GPIO_Port GPIOF
#define X_Backward_Switcher_Pin GPIO_PIN_0
#define X_Backward_Switcher_GPIO_Port GPIOD
#define X_Forward_Switcher_Pin GPIO_PIN_1
#define X_Forward_Switcher_GPIO_Port GPIOD
#define Y_Left_Backward_Switcher_Pin GPIO_PIN_2
#define Y_Left_Backward_Switcher_GPIO_Port GPIOD
#define Y_Left_Forward_Switcher_Pin GPIO_PIN_3
#define Y_Left_Forward_Switcher_GPIO_Port GPIOD
#define Y_Right_Backward_Switcher_Pin GPIO_PIN_4
#define Y_Right_Backward_Switcher_GPIO_Port GPIOD
#define Y_Right_Forward_Switcher_Pin GPIO_PIN_5
#define Y_Right_Forward_Switcher_GPIO_Port GPIOD
#define Y_RIGHT_PWM_Pin GPIO_PIN_3
#define Y_RIGHT_PWM_GPIO_Port GPIOB
#define Y_LEFT_ENCODER_A_Pin GPIO_PIN_4
#define Y_LEFT_ENCODER_A_GPIO_Port GPIOB
#define Y_LEFT_ENCODER_B_Pin GPIO_PIN_5
#define Y_LEFT_ENCODER_B_GPIO_Port GPIOB
#define Y_RIGHT_ENCODER_A_Pin GPIO_PIN_6
#define Y_RIGHT_ENCODER_A_GPIO_Port GPIOB
#define Y_RIGHT_ENCODER_B_Pin GPIO_PIN_7
#define Y_RIGHT_ENCODER_B_GPIO_Port GPIOB

#define button_Pin GPIO_PIN_13
#define button_GPIO_Port GPIOC

#define RMII_MDC_Pin GPIO_PIN_1
#define RMII_MDC_GPIO_Port GPIOC

#define RMII_REF_CLK_Pin GPIO_PIN_1
#define RMII_REF_CLK_GPIO_Port GPIOA
#define RMII_MDIO_Pin GPIO_PIN_2
#define RMII_MDIO_GPIO_Port GPIOA

#define RMII_CRS_DV_Pin GPIO_PIN_7
#define RMII_CRS_DV_GPIO_Port GPIOA
#define RMII_RXD0_Pin GPIO_PIN_4
#define RMII_RXD0_GPIO_Port GPIOC
#define RMII_RXD1_Pin GPIO_PIN_5
#define RMII_RXD1_GPIO_Port GPIOC

#define RMII_TXD1_Pin GPIO_PIN_13
#define RMII_TXD1_GPIO_Port GPIOB

#define RMII_TX_EN_Pin GPIO_PIN_11
#define RMII_TX_EN_GPIO_Port GPIOG

#define RMII_TXD0_Pin GPIO_PIN_13
#define RMII_TXD0_GPIO_Port GPIOG

/* USER CODE BEGIN Private defines */


/* USER CODE END Private defines */

/**
  * @}
  */ 

/**
  * @}
*/ 

#endif /* __MAIN_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
