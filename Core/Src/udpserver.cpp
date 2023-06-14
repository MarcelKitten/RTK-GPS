/*
 * udpserver.c
 *
 *  Created on: Mar 31, 2022
 *      Author: controllerstech
 */

#include <udpserver.hpp>
#include "lwip/opt.h"

#include "lwip/api.h"
#include "lwip/sys.h"

#include "string.h"
#include "main.h"
#include "cmsis_os.h"
#include "semphr.h"
#include "FreeRTOS.h"
#include "flash.h"
#include "globals.h"

/* defines */
#define BROADCAST_INTERVAL_MS 1000 // 5 s

#define LONG_TIME 0xffff
#define TICKS_TO_WAIT    10

/* extern defines */
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart7;
extern DMA_HandleTypeDef hdma_uart7_rx;

/* forward declaration */
static void udp_thread(void *arg);
static void udp_broadcast(void *arg);
static void udp_nav_output(void *arg);

uint8_t base;
uint16_t bufPosUBX;
uint8_t isNewUBX;
uint8_t isUBX;
uint16_t UbxMsgSize;
uint16_t lastUbxMsgSize;

FlashData_t flashNetworkConfig;

/* Variables */
extern SemaphoreHandle_t xSemaphore;
BaseType_t xHigherPriorityTaskWoken = pdFALSE;

/* functions */

extern "C" void udpserver_init(void)
{
//	xSemaphore = xSemaphoreCreateBinary();

	// read configuration settings from flash memory
    Flash_Read_Data(FLASH_USER_SECTOR_11, (uint8_t *)&flashNetworkConfig, sizeof(flashNetworkConfig));


    if(flashNetworkConfig.isBase)
	{
		sys_thread_new("udp_broadcast", udp_broadcast, NULL, DEFAULT_THREAD_STACKSIZE, osPriorityNormal);
    }
    else {
		sys_thread_new("udp_thread", udp_thread, NULL, DEFAULT_THREAD_STACKSIZE, osPriorityNormal);
		sys_thread_new("udp_nav_output", udp_nav_output, NULL, DEFAULT_THREAD_STACKSIZE, osPriorityNormal);
    }

}


void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	if (huart->Instance == UART7)
	{
		if(flashNetworkConfig.isBase)
		{
			// Find RTCM3 initialisation chars
			for(size_t i = 0; i < Size; i++)
			{
				if((RxBuf[i] == 0xD3) && (RxBuf[i+1] == 0x00))
				{
					bufPos = i;
					isNewRTCM = 1;
					isRTCM = isNewRTCM;
					RtcmMsgSize = Size;
					lastRtcmMsgSize = RtcmMsgSize;
					break;
				}
			}
			if(isRTCM && (bufPos + Size) <= MainBuf_SIZE)
			{
				RtcmMsgSize -= Size;
				memcpy(MainBuf+bufPos, RxBuf, Size);
				bufPos += Size;
			}
			else
			{
				bufPos = 0;
				isRTCM = 0;
				RtcmMsgSize = 0;
			}

			if(RtcmMsgSize == 0)
				isRTCM = 0;
		}
		else
		{
			for(size_t i = 0; i < Size; i++)
			{
				if((RxBuf[i] == 0xB5) && (RxBuf[i+1] == 0x62))
				{
					bufPosUBX = i;
					isNewUBX = 1;
					isUBX = isNewUBX;
					UbxMsgSize = Size;
					lastUbxMsgSize = UbxMsgSize;
					break;
				}
			}
			if(isUBX && (bufPosUBX + Size) <= MainBuf_SIZE)
			{
				UbxMsgSize -= Size;
				memcpy(MainBuf+bufPosUBX, RxBuf, Size);
				bufPosUBX += Size;
			}
			else
			{
				bufPosUBX = 0;
				isUBX = 0;
				UbxMsgSize = 0;
			}

			if(UbxMsgSize == 0)
				isUBX = 0;
		}

//		__HAL_UART_DISABLE_IT(huart, UART_IT_IDLE); // Disable the idle line interrupt
//		/* start the DMA again */
//		HAL_UARTEx_ReceiveToIdle_DMA(&huart7, (uint8_t *) RxBuf, RxBuf_SIZE);
//		__HAL_DMA_DISABLE_IT(&hdma_uart7_rx, DMA_IT_HT);
//		__HAL_UART_ENABLE_IT(&huart7, UART_IT_IDLE);
//		HAL_UARTEx_ReceiveToIdle_DMA(&huart7, (uint8_t *)RxBuf, RxBuf_SIZE);
//		HAL_UART_Receive_DMA(&huart7, (uint8_t *)RxBuf, RxBuf_SIZE);

//		__HAL_UART_DISABLE_IT(huart, UART_IT_IDLE); // Disable the idle line interrupt
//		/* start the DMA again */
//		HAL_UARTEx_ReceiveToIdle_DMA(&huart7, (uint8_t *) RxBuf, RxBuf_SIZE);
//		__HAL_UART_ENABLE_IT(huart, UART_IT_IDLE); // Re-enable the idle line interrupt

		/* start the DMA again */
		HAL_UARTEx_ReceiveToIdle_DMA(&huart7, RxBuf, RxBuf_SIZE);
		__HAL_DMA_DISABLE_IT(&hdma_uart7_rx, DMA_IT_HT);
	}
}

// udp-server
static void udp_thread(void *arg)
{
	err_t err, recv_err;
	struct netbuf *buf;
	struct netconn *conn;
	ip_addr_t *addr;
	uint16_t port;
	uint8_t msg[1480];

	/* Create a new connection identifier */
	conn = netconn_new(NETCONN_UDP);

	if (conn != NULL)
	{
		/* Bind connection to the port 5000 */
		err = netconn_bind(conn, IP_ADDR_ANY, flashNetworkConfig.rtcmUdpPort);
		if (err == ERR_OK)
		{
			// Enter endless for loop if is no error
			for (;;)
			{
				/* Receive the data from the connection */
				recv_err = netconn_recv(conn, &buf);

				if (recv_err == ERR_OK) // if the data is received
				{
					addr = netbuf_fromaddr(buf); // get the address of the client
					port = netbuf_fromport(buf); // get the Port of the client

					// reset buffer
					memset(msg, 0, sizeof(msg));
					// Get the message from the client
					memcpy(msg, buf->p->payload, buf->p->len);
//					msg[buf->p->len] = '\0';
					HAL_UART_Transmit_IT(&huart7, (uint8_t *)msg, buf->p->len);

					// Free the buffer
					netbuf_delete(buf);
				}
			}
		}
		else
		{
			netconn_delete(conn);
		}
	}
}

// udp broadcast
void udp_nav_output(void *arg)
{
    // create a new UDP netconn for broadcasting
    struct netconn *broadcast_conn = netconn_new(NETCONN_UDP);
    if (broadcast_conn != NULL) {
        // bind to a random port
        netconn_bind(broadcast_conn, IP_ADDR_ANY, 0);
    }

	ip_addr_t receiver_addr = flashNetworkConfig.vehicleIP;
	uint16_t receiver_port = flashNetworkConfig.vehicleUdpPort;

    // create a new netbuf for holding the broadcast message
    struct netbuf *broadcast_buf = netbuf_new();

    for (;;)
    {
    	if(isNewUBX)
    	{
    		// copy the UBXmessage into a buffer for sending
			uint8_t buffer[lastUbxMsgSize];
			memcpy(buffer, MainBuf, lastUbxMsgSize);

			// allocate space in the broadcast netbuf for the message
			void *data = netbuf_alloc(broadcast_buf, lastUbxMsgSize);
			if (data != NULL) {
				// copy the message into the netbuf
				memcpy(data, buffer, lastUbxMsgSize);

				// send the netbuf as a broadcast to the broadcast address and port
				netconn_sendto(broadcast_conn, broadcast_buf, &receiver_addr, receiver_port);

				// reset the flag indicating a new RTCM message
				isNewUBX = 0;
			}
    	}
		vTaskDelay(pdMS_TO_TICKS(20));
    }

    // free the broadcast netbuf
    netbuf_delete(broadcast_buf);
}

// udp broadcast
void udp_broadcast(void *arg)
{
    // create a new UDP netconn for broadcasting
    struct netconn *broadcast_conn = netconn_new(NETCONN_UDP);
    if (broadcast_conn != NULL) {
        // bind to a random port
        netconn_bind(broadcast_conn, IP_ADDR_ANY, 0);
    }

    // set the broadcast IP address
    ip_addr_t broadcast_addr = flashNetworkConfig.rtcmBroadcastIP;

    // set the broadcast port
    uint16_t broadcast_port = flashNetworkConfig.rtcmUdpPort;

    // create a new netbuf for holding the broadcast message
    struct netbuf *broadcast_buf = netbuf_new();

    for (;;) {
        // if there is a new RTCM message, send it as a broadcast
        if (isNewRTCM) {
            // copy the RTCM message into a buffer for sending
            uint8_t buffer[lastRtcmMsgSize];
            memcpy(buffer, MainBuf, lastRtcmMsgSize);

            // allocate space in the broadcast netbuf for the message
            void *data = netbuf_alloc(broadcast_buf, lastRtcmMsgSize);
            if (data != NULL) {
                // copy the message into the netbuf
                memcpy(data, buffer, lastRtcmMsgSize);

                // send the netbuf as a broadcast to the broadcast address and port
                netconn_sendto(broadcast_conn, broadcast_buf, &broadcast_addr, broadcast_port);

                // reset the flag indicating a new RTCM message
                isNewRTCM = 0;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }

    // free the broadcast netbuf
    netbuf_delete(broadcast_buf);
}
