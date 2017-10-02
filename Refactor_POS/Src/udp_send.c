/* Includes ------------------------------------------------------------------*/
#include "lwip/debug.h"
#include "lwip/stats.h"
#include "lwip/udp.h"
#include "main.h"
#include "memp.h"
#include "netif.h"
#include <stdio.h>
#include <string.h>
#include "socket.h"



#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"



#define UDP_ADC_PACKET_SIZE		1000
#define UDP_PACKET_SEND_DELAY 1000

#define SENDER_PORT_NUM				4001

ip_addr_t DestIPaddr;
extern ip4_addr_t ipaddr;





//extern SemaphoreHandle_t xAdcBuf_Send_Semaphore;
SemaphoreHandle_t xAdcBuf_Send_Semaphore=NULL;

void UDP_Send_Task( void *pvParameters );

#define TEST_BUF_LEN			6000
#define TEST_CHANNEL_NUM	6
float test_buf[TEST_BUF_LEN];

void Set_TestBuf(float *buf, uint16_t buf_len, uint8_t chn_num);



#pragma pack(push,1)
typedef struct
{
	uint8_t 	id;
	uint64_t 	timestamp;
	uint8_t 	data[UDP_ADC_PACKET_SIZE];
}stPacket;
#pragma pack(pop)

stPacket UDPPacket;



float ADC_resultBuf[100];


int socket_fd;
struct sockaddr_in sa, ra;

uint8_t socket_err=0;
void udp_client_init(void)
{
//	ADC_buf_pnt=DCMIAdcRxBuff;
	vSemaphoreCreateBinary( xAdcBuf_Send_Semaphore );
  IP4_ADDR( &DestIPaddr, 192,168,16,251/*configInfo.IPAdress_Server.ip_addr_0, configInfo.IPAdress_Server.ip_addr_1, configInfo.IPAdress_Server.ip_addr_2, configInfo.IPAdress_Server.ip_addr_3 */);

	socket_fd = socket(PF_INET, SOCK_DGRAM, 0);

	if ( socket_fd < 0 )
	{
		socket_err=1;
		return;
	}

	memset(&sa, 0, sizeof(struct sockaddr_in));
	sa.sin_family = AF_INET;
	sa.sin_addr.s_addr = ipaddr.addr;
	sa.sin_port = htons(SENDER_PORT_NUM);

	if (bind(socket_fd, (struct sockaddr *)&sa, sizeof(struct sockaddr_in)) == -1)
	{
		close(socket_fd);
		socket_err=1;
		return;
	}

	memset(&ra, 0, sizeof(struct sockaddr_in));
	ra.sin_family = AF_INET;
	ra.sin_addr.s_addr = DestIPaddr.addr;
	ra.sin_port = htons(SENDER_PORT_NUM);


//	Set_TestBuf(test_buf,TEST_BUF_LEN,TEST_CHANNEL_NUM);
//  xTaskCreate( UDP_Send_Task, "UDP Task", 1024, NULL, 2, NULL );
}


void udp_client_send_buf(float *buf, uint16_t bufSize)
{
		err_t err;
		uint16_t adc_buf_offset=0;
		
		UDPPacket.id=0;
		UDPPacket.timestamp=1000;//DCMI_ADC_GetLastTimestamp();

		while(adc_buf_offset<(bufSize*sizeof(float)))
		{
			memcpy(&UDPPacket.data,((uint8_t*)buf+adc_buf_offset),UDP_ADC_PACKET_SIZE);			
			sendto(socket_fd, &UDPPacket,sizeof(UDPPacket),0,(struct sockaddr*)&ra,sizeof(ra));			
			adc_buf_offset+=UDP_ADC_PACKET_SIZE;
			UDPPacket.id++;
		}
}


void UDP_Send_Task( void *pvParameters )
{
	uint16_t result_buf_len=0;
//	printf("UDP Send Task started");
	while(1)
	{
		xSemaphoreTake( xAdcBuf_Send_Semaphore, portMAX_DELAY );
//		ADC_ConvertBuf(ADC_buf_pnt,(ADC_BUF_LEN>>1),currentSPI3_ADC_Buf,currentSPI3_ADC_Buf,(SPI_ADC_BUF_LEN>>1),test_buf, &result_buf_len);
		//udp_client_send_buf(ADC_resultBuf,result_buf_len);
		Set_TestBuf(ADC_resultBuf,TEST_BUF_LEN,TEST_CHANNEL_NUM);
		
		udp_client_send_buf(ADC_resultBuf,TEST_BUF_LEN);
	}
}


#define SIGNAL_VALUE_MAX 50000.0
#define SIGNAL_VALUE_INCREMENT 0.1
void Set_TestBuf(float *buf, uint16_t buf_len, uint8_t chn_num)
{
	uint16_t buf_index=0;
	uint8_t chn_index=0;
	
	static float signal_value=0.0;

	for(buf_index=0;buf_index<buf_len;buf_index+=chn_num)
	{
		signal_value+=SIGNAL_VALUE_INCREMENT;
		
		if(signal_value>=SIGNAL_VALUE_MAX)
		{
				signal_value=0.1;
		}
		
		for(chn_index=0;chn_index<chn_num;chn_index++)
		{
				buf[buf_index+chn_index]=signal_value;
		}
	}
}

void udp_send_data()
{
	sendto(socket_fd, "asd",sizeof("asd"),0,(struct sockaddr*)&ra,sizeof(ra));	
}