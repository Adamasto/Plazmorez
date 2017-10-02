#include <stdio.h>
#include <socket.h>
#include <string.h>

#include "port.h"

/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mbport.h"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"


SOCKET          xListenSocket;
static fd_set   allset;



stMB_TCPClient *Current_MB_TCPClient;

#define MB_TCP_CLIENT_NUM					2
#define MB_TCP_CLIENT_STACK_SIZE	1024
#define MB_TCP_POOL_STACK_SIZE		512

stMB_TCPClient MB_TCPClient[MB_TCP_CLIENT_NUM]={{INVALID_SOCKET,{0},0,0} , {INVALID_SOCKET,{0},0,0}};

SemaphoreHandle_t xMB_FrameRec_Mutex;

uint8_t flag_mb_ready=0;

/* ----------------------- Static functions ---------------------------------*/
BOOL            prvMBTCPPortAddressToString( SOCKET xSocket, CHAR * szAddr, USHORT usBufSize );
CHAR           *prvMBTCPPortFrameToString( UCHAR * pucFrame, USHORT usFrameLen );
static BOOL     prvbMBPortAcceptClient( stMB_TCPClient *MB_TCPClient );
static void     prvvMBPortReleaseClient( stMB_TCPClient *MB_TCPClient);
static void 		usleep(uint32_t time);
void xMBTCPPort_HandlingTask( void *pvParameters );
void xMBTCPPort_PoolTask( void *pvParameters );

/* ----------------------- Begin implementation -----------------------------*/

static void 		usleep(uint32_t time)
{
		while(time)
		{
				time--;
		}
}


BOOL
xMBTCPPortInit( USHORT usTCPPort )
{
    USHORT          usPort;
    struct sockaddr_in serveraddr;
	
		 xMB_FrameRec_Mutex = xSemaphoreCreateMutex();
	
			int timeoutTimeInMiliSeconds=100;
	
	
		

    if( usTCPPort == 0 )
    {
        usPort = MB_TCP_DEFAULT_PORT;
    }
    else
    {
        usPort = ( USHORT ) usTCPPort;
    }
    memset( &serveraddr, 0, sizeof( serveraddr ) );
    serveraddr.sin_family = AF_INET;
    serveraddr.sin_addr.s_addr = htonl( INADDR_ANY );
    serveraddr.sin_port = htons( usPort );
    if( ( xListenSocket = socket( AF_INET, SOCK_STREAM, IPPROTO_TCP ) ) == -1 )
    {
        printf( "Create socket failed.\r\n" );
        return FALSE;
    }
//		else if(setsockopt( xListenSocket, SOL_SOCKET, SO_RCVTIMEO,&timeoutTimeInMiliSeconds, sizeof(int)) == -1)
//		{
//				printf( "Set timeout socket failed.\r\n" );
//        return FALSE;
//		}
    else if( bind( xListenSocket, ( struct sockaddr * )&serveraddr, sizeof( serveraddr ) ) == -1 )
    {
        printf( "Bind socket failed.\r\n" );
        return FALSE;
    }
    else if( listen( xListenSocket, 5 ) == -1 )
    {
        printf( "Listen socket failed.\r\n" );
        return FALSE;
    }
    FD_ZERO( &allset );
    FD_SET( xListenSocket, &allset );
		
		listen(xListenSocket,5);
		

		xTaskCreate( xMBTCPPort_PoolTask, "MBTCP HANDLE", MB_TCP_POOL_STACK_SIZE, NULL, 2, NULL );
		
				
	

		int	clnt_index=0;
		for(clnt_index=0;clnt_index<MB_TCP_CLIENT_NUM;clnt_index++)
		{
				xTaskCreate( xMBTCPPort_HandlingTask, "MBTCP HANDLE", MB_TCP_CLIENT_STACK_SIZE, (void*)&MB_TCPClient[clnt_index], 3, NULL );
		}
    return TRUE;
}

//void
//vMBTCPPortClose(  )
//{
//    // Close all client sockets. 
//    if( xClientSocket != SOCKET_ERROR )
//    {
//        prvvMBPortReleaseClient(  );
//    }
//    // Close the listener socket.
//    if( xListenSocket != SOCKET_ERROR )
//    {
//        close( xListenSocket );
//    }
//}

void
vMBTCPPortDisable( void )
{
    /* Close all client sockets. */
    if( Current_MB_TCPClient->xClientSocket != SOCKET_ERROR )
    {
        prvvMBPortReleaseClient( Current_MB_TCPClient );
    }
}

BOOL
xMBPortTCPPool( void )
{
		int	clnt_index=0;
	
		BaseType_t task_ret=pdPASS;

		for(clnt_index=0;clnt_index<MB_TCP_CLIENT_NUM;clnt_index++)
		{
			if( MB_TCPClient[clnt_index].xClientSocket == INVALID_SOCKET )
			{

							if(prvbMBPortAcceptClient( &MB_TCPClient[clnt_index] )==TRUE)
							{
								  printf("New client accepted, Socket=%i\n",MB_TCPClient[clnt_index].xClientSocket);
									taskYIELD();
							}
			}
		}	
    return TRUE;
}


BOOL
xMBTCPPortGetRequest( UCHAR ** ppucMBTCPFrame, USHORT * usTCPLength )
{
    *ppucMBTCPFrame = &Current_MB_TCPClient->aucTCPBuf[0];
    *usTCPLength = Current_MB_TCPClient->usTCPBufPos;

    /* Reset the buffer. */
    Current_MB_TCPClient->usTCPBufPos = 0;
    Current_MB_TCPClient->usTCPFrameBytesLeft = MB_TCP_FUNC;
    return TRUE;
}


uint8_t activeSocket=0;

BOOL
xMBTCPPortSendResponse( const UCHAR * pucMBTCPFrame, USHORT usTCPLength )
{
    BOOL            bFrameSent = FALSE;
    BOOL            bAbort = FALSE;
    int             res;
    int             iBytesSent = 0;
    int             iTimeOut = MB_TCP_READ_TIMEOUT;
	
		if(Current_MB_TCPClient->xClientSocket==INVALID_SOCKET)
		{
				return FALSE;
		}

    do
    {
        res = send( Current_MB_TCPClient->xClientSocket, &pucMBTCPFrame[iBytesSent], usTCPLength - iBytesSent, 0 );
        switch ( res )
        {
        case -1:
            if( iTimeOut > 0 )
            {
                iTimeOut -= MB_TCP_READ_CYCLE;
                usleep( MB_TCP_READ_CYCLE );
            }
            else
            {
                bAbort = TRUE;
            }
            break;
        case 0:
            prvvMBPortReleaseClient(Current_MB_TCPClient );
            bAbort = TRUE;
            break;
        default:
            iBytesSent += res;
            break;
        }
    }
    while( ( iBytesSent != usTCPLength ) && !bAbort );

    bFrameSent = iBytesSent == usTCPLength ? TRUE : FALSE;

		flag_mb_ready=1;
    return bFrameSent;
}

void
prvvMBPortReleaseClient( stMB_TCPClient *MB_TCPClient )
{
    ( void )recv( MB_TCPClient->xClientSocket, &MB_TCPClient->aucTCPBuf[0], MB_TCP_BUF_SIZE, 0 );

    ( void )close( MB_TCPClient->xClientSocket );
    MB_TCPClient->xClientSocket = INVALID_SOCKET;
}

BOOL
prvbMBPortAcceptClient( stMB_TCPClient *MB_TCPClient )
{
    SOCKET          xNewSocket;
    BOOL            bOkay;

    /* Check if we can handle a new connection. */

    if( MB_TCPClient->xClientSocket != INVALID_SOCKET )
    {
        printf( "can't accept new client %i. all connections in use.\n",MB_TCPClient->xClientSocket );
        bOkay = FALSE;
    }
    else if( ( xNewSocket = accept( xListenSocket, NULL, NULL ) ) == INVALID_SOCKET )
    {
        bOkay = FALSE;
    }
    else
    {
        MB_TCPClient->xClientSocket = xNewSocket;
        MB_TCPClient->usTCPBufPos = 0;
        MB_TCPClient->usTCPFrameBytesLeft = MB_TCP_FUNC;
        bOkay = TRUE;
    }
    return bOkay;
}

void xMBTCPPort_PoolTask( void *pvParameters )
{
	while(1)
	{
			(void)xMBPortTCPPool(  );
	}
}




void xMBTCPPort_HandlingTask( void *pvParameters )
{
		fd_set          fread;
		int             ret;
		USHORT          usLength;
	  struct timeval  tval;

    tval.tv_sec =  1;
    tval.tv_usec = 0;
	
		
	
		stMB_TCPClient *MB_TCPClient;
		MB_TCPClient=(stMB_TCPClient*)pvParameters;
	
		printf("Task of socket %i started\r\n",MB_TCPClient->xClientSocket);
	
	  while(1)
    {		
				if(MB_TCPClient->xClientSocket==INVALID_SOCKET)
				{
						vTaskDelay(1);
						continue;
				}
				
				
				FD_ZERO( &fread );
				FD_SET( MB_TCPClient->xClientSocket, &fread );
				printf("Wait for Socket %i data... \r\n",MB_TCPClient->xClientSocket);
			  ret = select( MB_TCPClient->xClientSocket + 1, &fread, NULL, NULL, &tval );
        if(!ret)
        {
          printf("Continue for Socket %i... \r\n",MB_TCPClient->xClientSocket);  
					continue;
        }
				else if(ret == SOCKET_ERROR )
				{
						printf("Select return SOCKET_ERROR\r\n");
						printf("Socket %i closed(select)\r\n",MB_TCPClient->xClientSocket);
						MB_TCPClient->xClientSocket = INVALID_SOCKET;
						continue;
           // vTaskDelete(NULL);
				}
        else if( ret > 0 )
        {
						printf("Socket %i has new data\r\n",MB_TCPClient->xClientSocket);
					
            if( FD_ISSET( MB_TCPClient->xClientSocket, &fread ) )
            {
								ret = recv( MB_TCPClient->xClientSocket, &MB_TCPClient->aucTCPBuf[MB_TCPClient->usTCPBufPos], MB_TCPClient->usTCPFrameBytesLeft,0 );
                if(( ret == SOCKET_ERROR ) || ( !ret ) )
                {
										
                    close( MB_TCPClient->xClientSocket );
									  printf("Socket %i closed\r\n",MB_TCPClient->xClientSocket);
										MB_TCPClient->xClientSocket = INVALID_SOCKET;
                    //vTaskDelete(NULL);
										continue;
                }
								
								printf("Task of socket %i receive %i data\r\n",MB_TCPClient->xClientSocket, ret);
								
                MB_TCPClient->usTCPBufPos += ret;
                MB_TCPClient->usTCPFrameBytesLeft -= ret;
								
                if( MB_TCPClient->usTCPBufPos >= MB_TCP_FUNC )
                {
                    /* Length is a byte count of Modbus PDU (function code + data) and the
                     * unit identifier. */
                    usLength = MB_TCPClient->aucTCPBuf[MB_TCP_LEN] << 8U;
                    usLength |= MB_TCPClient->aucTCPBuf[MB_TCP_LEN + 1];

                    /* Is the frame already complete. */
                    if( MB_TCPClient->usTCPBufPos < ( MB_TCP_UID + usLength ) )
                    {
                        MB_TCPClient->usTCPFrameBytesLeft = usLength + MB_TCP_UID - MB_TCPClient->usTCPBufPos;
                    }
                    /* The frame is complete. */
                    else if( MB_TCPClient->usTCPBufPos == ( MB_TCP_UID + usLength ) )
                    {
											
												if(xSemaphoreTake( xMB_FrameRec_Mutex, ( TickType_t ) 10 ) == pdTRUE )
												{
											
													
														( void )xMBPortEventPost( EV_FRAME_RECEIVED );
														Current_MB_TCPClient=MB_TCPClient;		
														printf("Task of socket %i send command EV_FRAME_RECEIVED\r\n",MB_TCPClient->xClientSocket);
//														while(flag_mb_ready==0)
//														{
//															__NOP();
//														}

														flag_mb_ready=0;
														
														xSemaphoreGive( xMB_FrameRec_Mutex );
												}
												else
												{
													
													  close( MB_TCPClient->xClientSocket );
														MB_TCPClient->xClientSocket = INVALID_SOCKET;
														//vTaskDelete(NULL);
														continue;
												}
                    }
                    /* This can not happend because we always calculate the number of bytes
                     * to receive. */
                    else
                    {
                        assert( MB_TCPClient->usTCPBufPos <= ( MB_TCP_UID + usLength ) );
                    }
                }
            }
        }
    }
}