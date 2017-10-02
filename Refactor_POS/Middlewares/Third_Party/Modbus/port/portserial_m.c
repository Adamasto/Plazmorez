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
 * File: $Id: portserial_m.c,v 1.60 2013/08/13 15:07:05 Armink add Master Functions $
 */

#include "port.h"

/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mbport.h"
#include "mbconfig.h"

extern UART_HandleTypeDef huart1;

#if MB_MASTER_RTU_ENABLED > 0 || MB_MASTER_ASCII_ENABLED > 0
/* ----------------------- Static variables ---------------------------------*/


/* ----------------------- Defines ------------------------------------------*/
/* serial transmit event */
#define EVENT_SERIAL_TRANS_START    (1<<0)

/* ----------------------- static functions ---------------------------------*/
//static void prvvUARTTxReadyISR(void);
//static void prvvUARTRxISR(void);
//static rt_err_t serial_rx_ind(rt_device_t dev, rt_size_t size);
//static void serial_soft_trans_irq(void* parameter);

/* ----------------------- Start implementation -----------------------------*/
BOOL xMBMasterPortSerialInit(UCHAR ucPORT, ULONG ulBaudRate, UCHAR ucDataBits,
        eMBParity eParity)
{

  huart1.Instance = USART1;	
  huart1.Init.BaudRate = ulBaudRate;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = eParity;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  HAL_UART_Init(&huart1);
  return TRUE;
		
    return TRUE;
}

void vMBMasterPortSerialEnable(BOOL xRxEnable, BOOL xTxEnable)
{
	if(TRUE==xRxEnable)
	{
			__HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);
	}
	else
	{
			__HAL_UART_DISABLE_IT(&huart1, UART_IT_RXNE);
	}

	if(TRUE==xTxEnable)
	{
			__HAL_UART_ENABLE_IT(&huart1, UART_IT_TXE);
	}
	else
	{
			__HAL_UART_DISABLE_IT(&huart1, UART_IT_TXE);
	}
}

void vMBMasterPortClose(void)
{

}

BOOL xMBMasterPortSerialPutByte(CHAR ucByte)
{
		huart1.Instance->TDR=ucByte;
	  return TRUE;
}

BOOL xMBMasterPortSerialGetByte(CHAR * pucByte)
{
		*pucByte=huart1.Instance->RDR;
    return TRUE;
}



BOOL UART_IRQ_Handler(USART_TypeDef * usart) 
{
	if (usart == huart1.Instance) 
	{
		if((__HAL_UART_GET_FLAG(&huart1, UART_FLAG_RXNE) != RESET) && (__HAL_UART_GET_IT_SOURCE(&huart1, UART_IT_RXNE) != RESET)) 
		{
			pxMBMasterFrameCBByteReceived();
			//__HAL_UART_SEND_REQ(&huart1, UART_RXDATA_FLUSH_REQUEST);
			return TRUE;
		}
		if((__HAL_UART_GET_FLAG(&huart1, UART_FLAG_TXE) != RESET) &&(__HAL_UART_GET_IT_SOURCE(&huart1, UART_IT_TXE) != RESET)) 
		{
			pxMBMasterFrameCBTransmitterEmpty();
			return TRUE;
		}
	}
	return FALSE;
}

#endif
