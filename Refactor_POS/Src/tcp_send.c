/* Includes ------------------------------------------------------------------*/
#include "lwip/debug.h"
#include "lwip/stats.h"
#include "lwip/tcp.h"
#include "main.h"
#include "memp.h"
#include "main.h"
#include <stdio.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "data_converter.h"
#include "adc_dcmi.h"
#include "spi_adc.h"

#if LWIP_TCP


struct tcp_pcb *client_pcb;

extern SemaphoreHandle_t xAdcBuf_Send_Semaphore;

extern uint16_t *currentSPI3_ADC_Buf;
extern uint16_t *currentSPI6_ADC_Buf;
extern uint8_t *ADC_buf_pnt;

#define ADC_RESULT_BUF_LEN (ADC_BUF_LEN/4)

float ADC_resultBuf[ADC_RESULT_BUF_LEN];

void TCP_Send_Task( void *pvParameters );
/* Private function prototypes -----------------------------------------------*/
void tcp_client_connect(void);
static void tcp_client_connection_close(struct tcp_pcb *tpcb);
static err_t tcp_client_sent(void *arg, struct tcp_pcb *tpcb, u16_t len);
static void tcp_client_send(struct tcp_pcb *tpcb);
static err_t tcp_client_connected(void *arg, struct tcp_pcb *tpcb, err_t err);

/* Private functions ---------------------------------------------------------*/

void tcp_client_init(void)
{
	tcp_client_connect();
}


void tcp_client_connect(void)
{
  ip_addr_t DestIPaddr;
  client_pcb = tcp_new();
  if (client_pcb != NULL)
  {
    IP4_ADDR( &DestIPaddr, 192, 168, 109, 140 );
    tcp_connect(client_pcb,&DestIPaddr,8080,tcp_client_connected);
  }
}


static err_t tcp_client_connected(void *arg, struct tcp_pcb *tpcb, err_t err)
{

  if (err == ERR_OK)
  {
        tcp_sent(tpcb, tcp_client_sent);
        xTaskCreate( TCP_Send_Task, "TCP Task", 1024, NULL, 2, NULL );
        return ERR_OK;
  }
  else
  {
	  	 tcp_client_connection_close(tpcb);
  }
  return err;
}


extern uint8_t *ADC_buf_pnt;
static void tcp_client_send(struct tcp_pcb *tpcb)
{
	uint16_t result_buf_len=6000;
	err_t wr_err = ERR_OK;
//	ADC_ConvertBuf(ADC_buf_pnt,(ADC_BUF_LEN>>1),currentSPI3_ADC_Buf,currentSPI3_ADC_Buf,(SPI_ADC_BUF_LEN>>1),ADC_resultBuf, &result_buf_len);
	wr_err = tcp_write(tpcb, (uint8_t*)ADC_resultBuf,sizeof(float)*result_buf_len,0);
	tcp_output(tpcb);
}

extern uint8_t buf_transmit_flag;
static err_t tcp_client_sent(void *arg, struct tcp_pcb *tpcb, u16_t len)
{
  LWIP_UNUSED_ARG(len);
//  tcp_client_connection_close(tpcb);

  return ERR_OK;
}


static void tcp_client_connection_close(struct tcp_pcb *tpcb)
{
  tcp_sent(tpcb, NULL);
  tcp_close(tpcb);
}

void TCP_Send_Task( void *pvParameters )
{
	while(1)
	{
		//xSemaphoreTake( xAdcBuf_Send_Semaphore, portMAX_DELAY );
		vTaskDelay(10);
		tcp_client_send(client_pcb);
	}
}

#endif /* LWIP_TCP */


