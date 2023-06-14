///**
//  ******************************************************************************
//  * @file           : HTTP_Server.cpp
//  * @brief          : HTTP website to configure the GPS-module and the microcontroller
//  * @author			: Marcel Lammerskitten
//  * @date			: 24/05/23
//  * @version		: 0.1
//  ******************************************************************************
//  * @attention
//  *
//  * Copyright (c) 2023 MobileTronics
//  *
//  ******************************************************************************
//  */
//
///* Includes */
//#include "main.h"
//#include "lwip/opt.h"
//#include "lwip/arch.h"
//#include "lwip/api.h"
//#include "lwip/apps/fs.h"
//#include "string.h"
//#include <stdio.h>
//#include "cmsis_os.h"
//#include "globals.h"
//#include "flash.h"
//
//
///* forward declaration */
//static void http_thread(void *arg);	/// FreeRTOS Task for HTTP access control
//static void http_server(struct netconn *conn); /// Function to handle data transfer
//
///**
// * @brief	HTTP-server initialisation
// *
// * Function to create the necessary FreeRTOS Task
// */
//extern "C" void http_server_init(void)
//{
//  sys_thread_new("http_thread", http_thread, NULL, 2048, osPriorityBelowNormal);
//}
//
//static void http_thread(void *arg)
//{
//
//
//  struct netconn *conn, *newconn;
//  err_t err, accept_err;
//
//  /* Create a new TCP connection handle */
//  conn = netconn_new(NETCONN_TCP);
//
//  if (conn!= NULL)
//  {
//    /* Bind to port 80 (HTTP) with any IP address */
//    err = netconn_bind(conn, IP_ADDR_ANY, 80);
//
//    if (err == ERR_OK)
//    {
//      /* Put the connection into LISTEN state */
//      netconn_listen(conn);
//
//      while(1)
//      {
//        /* accept any incoming connection */
//        accept_err = netconn_accept(conn, &newconn);
//        if(accept_err == ERR_OK)
//        {
//          /* serve connection */
//          http_server(newconn);
//
//          /* delete connection */
//          netconn_delete(newconn);
//        }
//      }
//    }
//  }
//}
//
//static void http_server(struct netconn *conn)
//{
//	struct netbuf *inbuf;
//	err_t recv_err;
//	char* buf;
//	u16_t buflen;
//	struct fs_file file;
//
//	/* Read the data from the port, blocking if nothing yet there */
//	recv_err = netconn_recv(conn, &inbuf);
//
//	if (recv_err == ERR_OK)
//	{
//		if (netconn_err(conn) == ERR_OK)
//		{
//			/* Get the data pointer and length of the data inside a netbuf */
//			netbuf_data(inbuf, (void**)&buf, &buflen);
//
//			// CORS-Header
//			const char *cors_headers = "Access-Control-Allow-Origin: *\r\nAccess-Control-Allow-Methods: GET, POST, OPTIONS\r\nAccess-Control-Allow-Headers: Content-Type, X-Requested-With\r\n";
//
//			/* Check if request to get the index.html */
//			if (strncmp((char const *)buf,"GET /index.html",15)==0)
//			{
//				fs_open(&file, "/index.html");
//				netconn_write(conn, (const unsigned char*)(file.data), (size_t)file.len, NETCONN_NOCOPY);
//				fs_close(&file);
//			}
//			else if (strncmp((char const *)buf,"GET /img/17719_mobiletronics-gmbh.png",37)==0)
//			{
//				fs_open(&file, "/img/17719_mobiletronics-gmbh.png");
//				netconn_write(conn, (const unsigned char*)(file.data), (size_t)file.len, NETCONN_NOCOPY);
//				fs_close(&file);
//			}
//			 /* Check if request is a GET request for setting configuration data */
//			else if (strncmp((char const *)buf, "GET /set_config", 15) == 0) {
//			    // Extrahieren und Verarbeiten der Parameter
//			    char device[10], baudrate[10], updateRate[10], baseMode[10], minTime[10], accuracy[10], ecefX[20], ecefY[20], ecefZ[20], nmea[10];
//			    sscanf((const char *)buf, "GET /set_config?device=%9[^&]&baudrate=%9[^&]&updateRate=%9[^&]&baseMode=%9[^&]&minTime=%9[^&]&accuracy=%9[^&]&ecefX=%19[^&]&ecefY=%19[^&]&ecefZ=%19[^&]&nmea=%9[^&]", device, baudrate, updateRate, baseMode, minTime, accuracy, ecefX, ecefY, ecefZ, nmea);
//
////			    printf("Device: %s\n", device);
////			    printf("Baudrate: %s\n", baudrate);
////			    printf("Update Rate: %s\n", updateRate);
////			    printf("Base Mode: %s\n", baseMode);
////			    printf("Min Time: %s\n", minTime);
////			    printf("Accuracy: %s\n", accuracy);
////			    printf("ECEF X: %s\n", ecefX);
////			    printf("ECEF Y: %s\n", ecefY);
////			    printf("ECEF Z: %s\n", ecefZ);
////			    printf("NMEA: %s\n", nmea);
//
//			    uint32_t uart2Baud = atoi(baudrate);
//			    uint16_t measRate = atoi(updateRate);
//				uint32_t minDur = atoi(minTime);
//				uint32_t accLimit = atoi(accuracy);
//
//			    // Send success message
//			    const char *response = "HTTP/1.1 200 OK\r\nContent-Type: text/plain\r\n\r\nConfiguration updated successfully.";
//			    netconn_write(conn, response, strlen(response), NETCONN_COPY);
//			}
//			else if (strncmp((char const *)buf,"GET /getvalue",13)==0) {
//				/* SVIN status */
//			    char status[20];
//			    if(svinVars.active) {
//			        strcpy(status, "Running");
//			    } else {
//			        strcpy(status, "Not Running");
//			    }
//			    char valid[4];
//			    if(svinVars.valid) {
//			        strcpy(valid, "Yes");
//			    } else {
//			        strcpy(valid, "No");
//			    }
//			    char sot[10];
//			    sprintf(sot, "%ld s", svinVars.dur);
//			    char spu[10];
//			    sprintf(spu, "%ld", svinVars.obsUsed);
//			    char exs[20];
//			    uint64_t ecefX = (uint64_t)svinVars.meanX * 100 + svinVars.meanXHP;
//			    sprintf(exs, "%lu.%04lu m", (uint32_t)(ecefX/10000), (uint32_t)(ecefX%10000));
//			    char eys[20];
//			    uint64_t ecefY = (uint64_t)svinVars.meanY * 100 + svinVars.meanYHP;
//			    sprintf(eys, "%lu.%04lu m", (uint32_t)(ecefY/10000), (uint32_t)(ecefY%10000));
//			    char ezs[20];
//			    uint64_t ecefZ = (uint64_t)svinVars.meanZ * 100 + svinVars.meanZHP;
//			    sprintf(ezs, "%lu.%04lu m", (uint32_t)(ecefZ/10000), (uint32_t)(ecefZ%10000));
//			    char sm[20];
//			    sprintf(sm, "%ld.%04ld m", svinVars.meanAcc/10000, svinVars.meanAcc%10000);
//			    char smd[10];
//				sprintf(smd, "%ld s", tmode.svinMinDur);
//				char sal[20];
//				sprintf(sal, "%ld,%04ld m", tmode.svinVarLimit/10000, tmode.svinVarLimit%10000);
//				char nsu[4];
//				sprintf(nsu, "%d", navPvt.numSatUsed);
//
//			    /* General status */
//			    char m[30];
//			    if(tmode.tmodeMode==0)
//			    	strcpy(m, "Rover-Mode");
//			    else if(tmode.tmodeMode == 1)
//			    	strcpy(m, "Base-Mode - Survey-In");
//			    else if(tmode.tmodeMode == 2)
//			    	strcpy(m, "Base-Mode - Fixed");
//			    else
//			    	strcpy(m, "unknown");
//			    char ft[31];
//				if(navPvt.fixType == 0)
//					strcpy(ft, "no-fix");
//				else if(navPvt.fixType == 1)
//					strcpy(ft, "dead reckoning only");
//				else if(navPvt.fixType == 2)
//					strcpy(ft, "2D-fix");
//				else if(navPvt.fixType == 3)
//					strcpy(ft, "3D-fix");
//				else if(navPvt.fixType == 4)
//					strcpy(ft, "GNSS + dead reckoning combined");
//				else if(navPvt.fixType == 5)
//					strcpy(ft, "time only fix");
//				else
//					strcpy(ft, "unknown");
//				char hmsl[10];
//				sprintf(hmsl, "%lu.%03lu m", navPvt.hMSL/1000, navPvt.hMSL/1000);
//				char hacc[10];
//				sprintf(hacc, "%lu mm", navPvt.hAcc);
//				char vacc[10];
//				sprintf(vacc, "%lu mm", navPvt.vAcc);
//
//				char *response_header;
//				response_header = static_cast<char*>(pvPortMalloc(200));
//				int header_len = sprintf(response_header, "HTTP/1.1 200 OK\r\nContent-Type: text/plain\r\n%s\r\n", cors_headers);
//
//				// Send the response header with CORS headers
//				netconn_write(conn, (const unsigned char*)response_header, (size_t)header_len, NETCONN_NOCOPY);
//
//				vPortFree(response_header);
//
//				char *pagedata;
//				pagedata = static_cast<char*>(pvPortMalloc(200));
//				int len = sprintf(pagedata, "%s|%s|%s|%s|%s|%s|%s|%s|%s|%s|%s|%s|%s|%s|%s|%s", status, sot, spu, exs, eys, ezs, sm, valid, m, smd, sal, nsu, ft, hmsl, hacc, vacc);
//
//				netconn_write(conn, (const unsigned char*)pagedata, (size_t)len, NETCONN_NOCOPY);
//
//			    vPortFree(pagedata);
//			    osDelay(20);
//			}
//			else
//			{
//				/* Load Error page */
//				fs_open(&file, "/404.html");
//				netconn_write(conn, (const unsigned char*)(file.data), (size_t)file.len, NETCONN_NOCOPY);
//				fs_close(&file);
//			}
//		}
//	}
//	/* Close the connection (server closes in HTTP) */
//	netconn_close(conn);
//
//	/* Delete the buffer (netconn_recv gives us ownership,
//   so we have to make sure to deallocate the buffer) */
//	netbuf_delete(inbuf);
//}
