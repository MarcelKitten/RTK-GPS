/* includes */
#include "main.h"
#include <cmsis_os.h>
#include <FreeRTOS.h>
#include <UBX_PROTO.hpp>
#include "string.h"
#include "queue.h"

#include "lwip/opt.h"
#include "lwip/api.h"
#include "lwip/sys.h"

#include "flash.h"
#include "globals.h"

/*
 * Todo:
 * - IP Adresse einstellen
 */

/* Definitions for Task1 */
//osThreadId Task1TaskHandle;

/* Extern defines */
extern UART_HandleTypeDef huart6;
extern UART_HandleTypeDef huart3;


/* Defines */
#define UART1_UBLOX 	huart6
#define UART_ST_LINK	huart3

/* Variables */
/// Store UART1_Ublox data
uint8_t UART1_rxBuffer[1] = {0};
uint8_t ch = 0;
bool firstChar = true;

/* fifo */
//QueueHandle_t ubloxRx;

void Task1_init(void const *argument);


extern "C" void app(void)
{
	ubloxRx = xQueueCreate(255, sizeof(uint8_t));
	HAL_UART_Receive_IT(&UART1_UBLOX, &ch, 1);

	myUbx.sendUart1NMEAOutput(false, 0);
	myUbx.sendUart1NMEAOutput(false, 1);
	myUbx.sendUart1NMEAOutput(false, 2);
//	myUbx.setBaseStationSurveyIn(0, 500, 100);
//	myUbx.setBaseStationSurveyIn(1, 500, 100);
//	myUbx.setBaseStationSurveyIn(2, 500, 100);
//    osThreadDef(Task1, Task1_init, osPriorityLow, 0, 1024);
//    Task1TaskHandle = osThreadCreate(osThread(Task1), NULL);
}

//void Task1_init(void const *argument)
//{
////	UbxProtocol myUbx(&UART1_UBLOX, &ubloxRx);
//	osDelay(100);
////	myUbx.setBaseStationFixed(0, 388165013, 53638060, 501578863);
////	myUbx.setBaseStationFixed(1, 388165013, 53638060, 501578863);
////	myUbx.setBaseStationFixed(2, 388165013, 53638060, 501578863);
////	myUbx.setRover(0, 38400, 200);
////	myUbx.setRover(1, 38400, 200);
////	myUbx.setRover(2, 38400, 200);
//	myUbx.setBaseStationSurveyIn(0, 500, 100);
//	myUbx.setBaseStationSurveyIn(1, 500, 100);
//	myUbx.setBaseStationSurveyIn(2, 500, 100);
//	vTaskDelete(Task1TaskHandle);
//	for(;;)
//	{
////		myUbx.flushQueue();
//////		myUbx.sendUart1NMEAOutputEnable(false, 0);
//////		myUbx.sendUart1NMEAOutputEnable(false, 1);
//////		myUbx.sendUart1NMEAOutputEnable(false, 2);
////		myUbx.getSurveyInStatus();
////		myUbx.flushQueue();
////		myUbx.getNavPvt();
////		myUbx.flushQueue();
////		myUbx.getTmodeMode();
//
////		osDelay(10000);
//	}
//}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(firstChar)
	{
		xQueueSendFromISR(ubloxRx, &ch, 0);
		firstChar = false;
	}
//    HAL_UART_Transmit(&UART_ST_LINK, UART1_rxBuffer, 1, 100);
    HAL_UART_Receive_IT(&UART1_UBLOX, &ch, 1);
    xQueueSendFromISR(ubloxRx, &ch, 0);
//    HAL_UART_Receive_IT(&UART_ST_LINK, UART1_rxBuffer, 1);
}
