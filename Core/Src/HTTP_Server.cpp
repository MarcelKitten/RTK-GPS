
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "lwip/opt.h"
#include "lwip/arch.h"
#include "lwip/api.h"
#include "lwip/apps/fs.h"
#include "string.h"
#include <stdio.h>
#include <UBX_PROTO.hpp>
#include "cmsis_os.h"
#include "globals.h"
#include "lwip/ip_addr.h"
#include "lwip/inet.h"
#include "flash.h"

/*
 * TODO:	- flash .isBase write into the flash
 */

/* extern */

static void http_server(struct netconn *conn)
{
	struct netbuf *inbuf;
	err_t recv_err;
	char* buf;
	u16_t buflen;
	struct fs_file file;

	FlashData_t flashIpConfig;

	uint8_t restartSystem = 0;

	/* Read the data from the port, blocking if nothing yet there */
	recv_err = netconn_recv(conn, &inbuf);

	if (recv_err == ERR_OK)
	{
		if (netconn_err(conn) == ERR_OK)
		{
			/* Get the data pointer and length of the data inside a netbuf */
			netbuf_data(inbuf, (void**)&buf, &buflen);

			// CORS-Header
			const char *cors_headers = "Access-Control-Allow-Origin: *\r\nAccess-Control-Allow-Methods: GET, POST, OPTIONS\r\nAccess-Control-Allow-Headers: Content-Type, X-Requested-With\r\n";

			/* Check if request to get the index.html */
			if (strncmp((char const *)buf,"GET /index.html",15)==0)
			{
				fs_open(&file, "/index.html");
				netconn_write(conn, (const unsigned char*)(file.data), (size_t)file.len, NETCONN_NOCOPY);
				fs_close(&file);
			}
			else if (strncmp((char const *)buf,"GET /img/17719_mobiletronics-gmbh.png",37)==0)
			{
				fs_open(&file, "/img/17719_mobiletronics-gmbh.png");
				netconn_write(conn, (const unsigned char*)(file.data), (size_t)file.len, NETCONN_NOCOPY);
				fs_close(&file);
			}
			else if (strncmp((char const *)buf,"GET /img/mt-silesia.png",23)==0)
			{
				fs_open(&file, "/img/mt-silesia.png");
				netconn_write(conn, (const unsigned char*)(file.data), (size_t)file.len, NETCONN_NOCOPY);
				fs_close(&file);
			}
			 /* Check if request is a GET request for setting configuration data */
			else if (strncmp((char const *)buf, "GET /set_config", 15) == 0) {
			    // get the parameters
			    char device[10], baudrate[10], updateRate[10], baseMode[10], minTime[10], accuracy[10], ecefX[20], ecefY[20], ecefZ[20], nmea[10];
			    sscanf((const char *)buf, "GET /set_config?device=%9[^&]&baudrate=%9[^&]&updateRate=%9[^&]&baseMode=%9[^&]&minTime=%9[^&]&accuracy=%9[^&]&ecefX=%19[^&]&ecefY=%19[^&]&ecefZ=%19[^&]&nmea=%9[^&]", device, baudrate, updateRate, baseMode, minTime, accuracy, ecefX, ecefY, ecefZ, nmea);

			    uint32_t uart2Baud = atoi(baudrate);
			    uint16_t measRate = atoi(updateRate);
				uint32_t minDur = atoi(minTime);
				uint32_t accLimit = atoi(accuracy)*10;
				uint32_t ecef_X = atoi(ecefX);
				uint32_t ecef_Y = atoi(ecefY);
				uint32_t ecef_Z = atoi(ecefZ);

				Flash_Read_Data(FLASH_USER_SECTOR_11, (uint8_t *)&flashIpConfig, sizeof(flashIpConfig));
				if(flashIpConfig.UART2Baud != uart2Baud)
				{
					myUbx.setUART2BaudRate(uart2Baud);
					flashIpConfig.UART2Baud = uart2Baud;
					Flash_Write_Data(FLASH_USER_SECTOR_11, (uint8_t *)&flashIpConfig, sizeof(flashIpConfig));
					restartSystem = 1;
				}

				if(!strcmp(device, "rover"))
				{
					myUbx.setRover();
					if(flashIpConfig.isBase)
					{
						// write mode into the flash
						flashIpConfig.isBase = 0;
						Flash_Write_Data(FLASH_USER_SECTOR_11, (uint8_t *)&flashIpConfig, sizeof(flashIpConfig));
						restartSystem = 1;
					}
				} else if (!strcmp(device, "base") && !strcmp(baseMode, "surveyIn"))
				{
					myUbx.setBaseStationSurveyIn(minDur, accLimit);
					if(!flashIpConfig.isBase)
					{
						// write mode into the flash
						flashIpConfig.isBase = 1;
						Flash_Write_Data(FLASH_USER_SECTOR_11, (uint8_t *)&flashIpConfig, sizeof(flashIpConfig));
						restartSystem = 1;
					}
				} else if (!strcmp(device, "base") && !strcmp(baseMode, "fixed"))
				{
					/* ToDo: fill with exif coordinates */
					myUbx.setBaseStationFixed(ecef_X, ecef_Y, ecef_Z);
					if(!flashIpConfig.isBase)
					{
						// write mode into the flash
						flashIpConfig.isBase = 1;
						Flash_Write_Data(FLASH_USER_SECTOR_11, (uint8_t *)&flashIpConfig, sizeof(flashIpConfig));
						restartSystem = 1;
					}
				}

				myUbx.setMeasRate(measRate);

				// send success message
			    const char *response = "HTTP/1.1 200 OK\r\nContent-Type: text/plain\r\n\r\nConfiguration updated successfully.";
			    netconn_write(conn, response, strlen(response), NETCONN_COPY);
			}
			else if (strncmp((char const *)buf,"GET /getvalue",13)==0) {

				/* ToDO: add ecef status information for fixed mode!
				 * maybe store ecef fixed status into the flash memory..
				 */


				/* Get values from u-blox module */
				myUbx.flushQueue();
				myUbx.getSurveyInStatus();
				myUbx.flushQueue();
				myUbx.getNavPvt();
				myUbx.flushQueue();
				myUbx.getTmodeMode();

				/* SVIN status */
			    char status[20];
			    if(svinVars.active) {
			        strcpy(status, "Running");
			    } else {
			        strcpy(status, "Not Running");
			    }
			    char valid[4];
			    if(svinVars.valid) {
			        strcpy(valid, "Yes");
			    } else {
			        strcpy(valid, "No");
			    }
			    char sot[10];
			    sprintf(sot, "%lu s", svinVars.dur);
			    char spu[10];
			    sprintf(spu, "%lu", svinVars.obsUsed);
			    char exs[20];
			    uint64_t ecefTemp = (uint64_t)svinVars.meanX * 100 + svinVars.meanXHP;
			    sprintf(exs, "%lu.%04lu m", (uint32_t)(ecefTemp/10000), (uint32_t)(ecefTemp%10000));
			    char eys[20];
			    ecefTemp = (uint64_t)svinVars.meanY * 100 + svinVars.meanYHP;
			    sprintf(eys, "%lu.%04lu m", (uint32_t)(ecefTemp/10000), (uint32_t)(ecefTemp%10000));
			    char ezs[20];
			    ecefTemp = (uint64_t)svinVars.meanZ * 100 + svinVars.meanZHP;
			    sprintf(ezs, "%lu.%04lu m", (uint32_t)(ecefTemp/10000), (uint32_t)(ecefTemp%10000));
			    char sm[20];
			    sprintf(sm, "%ld.%04ld m", svinVars.meanAcc/10000, svinVars.meanAcc%10000);
			    char smd[10];
				sprintf(smd, "%lu s", tmode.svinMinDur);
				char sal[10];
				sprintf(sal, "%lu,%04ld m", tmode.svinVarLimit/10000, tmode.svinVarLimit%10000);
				char nsu[4];
				sprintf(nsu, "%u", navPvt.numSatUsed);

				/* fixed-mode ECEF coordinates */
				char ex[20];
				ecefTemp = tmode.ecefX;
				sprintf(ex, "%lu.%02lu m", (uint32_t)(ecefTemp/100), (uint32_t)(ecefTemp%100));
				char ey[20];
				ecefTemp = tmode.ecefY;
				sprintf(ey, "%lu.%02lu m", (uint32_t)(ecefTemp/100), (uint32_t)(ecefTemp%100));
				char ez[20];
				ecefTemp = tmode.ecefZ;
				sprintf(ez, "%lu.%02lu m", (uint32_t)(ecefTemp/100), (uint32_t)(ecefTemp%100));

			    /* General status */
			    char m[24];
			    if(tmode.tmodeMode==0)
			    {
			    	strcpy(m, "Rover-Mode");
			    	isBase = 0;
			    }
			    else if(tmode.tmodeMode == 1)
			    {
			    	strcpy(m, "Base-Mode - Survey-In");
			    	isBase = 1;
			    }
			    else if(tmode.tmodeMode == 2)
			    {
			    	strcpy(m, "Base-Mode - Fixed");
			    	isBase = 1;
			    }
			    else
			    {
			    	strcpy(m, "unknown");
			    	isBase = 2;
			    }

			    char ft[32];
			    if(navPvt.fixType == 0)
					strcpy(ft, "no-fix");
				else if(navPvt.fixType == 1)
					strcpy(ft, "dead reckoning only");
				else if(navPvt.fixType == 2)
					strcpy(ft, "2D-fix");
				else if(navPvt.fixType == 3)
					strcpy(ft, "3D-fix");
				else if(navPvt.fixType == 4)
					strcpy(ft, "GNSS + dead reckoning combined");
				else if(navPvt.fixType == 5)
					strcpy(ft, "time only fix");
				else
					strcpy(ft, "unknown");

				char hmsl[16];
				sprintf(hmsl, "%li.%03li m", navPvt.hMSL/1000, navPvt.hMSL%1000);
				char hacc[16];
				sprintf(hacc, "%lu mm", navPvt.hAcc);
				char vacc[16];
				sprintf(vacc, "%lu mm", navPvt.vAcc);

				char *response_header;
				response_header = static_cast<char*>(pvPortMalloc(200));
				int header_len = sprintf(response_header, "HTTP/1.1 200 OK\r\nContent-Type: text/plain\r\n%s\r\n", cors_headers);

				// Send the response header with CORS headers
				netconn_write(conn, (const unsigned char*)response_header, (size_t)header_len, NETCONN_NOCOPY);

				vPortFree(response_header);

				char *pagedata;
				pagedata = static_cast<char*>(pvPortMalloc(200));
				int len = sprintf(pagedata, "%s|%s|%s|%s|%s|%s|%s|%s|%s|%s|%s|%s|%s|%s|%s|%s|%s|%s|%s", status, sot, spu, exs, eys, ezs, sm, valid, m, smd, sal, nsu, ft, hmsl, hacc, vacc, ex, ey, ez);

				netconn_write(conn, (const unsigned char*)pagedata, (size_t)len, NETCONN_NOCOPY);

			    vPortFree(pagedata);
			}
			else if (strncmp((char const *)buf, "GET /set_ip_config", 18) == 0) {
			    // Extrahieren und Verarbeiten der IP-Konfigurationsparameter
				char deviceIp[16], netmask[16], gateway[16], rtcmPort[6], vehiclePort[6], vehicleIp[16], rtcmIp[16];
				sscanf((const char *)buf, "GET /set_ip_config?deviceIp=%15[^&]&netmask=%15[^&]&gateway=%15[^&]&rtcmPort=%5[^&]&vehiclePort=%5[^&]&vehicleIp=%15[^&]&rtcmIp=%15[^&]",
					deviceIp, netmask, gateway, rtcmPort, vehiclePort, vehicleIp, rtcmIp);


				// Verarbeiten Sie die extrahierten IP-Konfigurationsparameter und aktualisieren Sie entsprechend
				ip4_addr_t deviceAddr;
				ip4_addr_t vehicleAddr;
				ip4_addr_t RtcmBroadcastAddr;
				ip4_addr_t netmaskAddr;
				ip4_addr_t gatewayAddr;
				uint16_t rtcmPortNum;
				uint16_t vehiclePortNum;

				uint8_t conversionSuccess = 1;

				// convert char array into uint16_t
				rtcmPortNum = atoi(rtcmPort);
				vehiclePortNum = atoi(vehiclePort);

				if(isBase == 1)
				{
					// convert from char array into ip4_addr_t
					if (!ip4addr_aton(deviceIp, &deviceAddr) ||
						!ip4addr_aton(rtcmIp, &RtcmBroadcastAddr) ||
						!ip4addr_aton(netmask, &netmaskAddr) ||
						!ip4addr_aton(gateway, &gatewayAddr))
					{
						const char *response = "HTTP/1.1 200 OK\r\nContent-Type: text/plain\r\n\r\nIP address error!\nChanges not accepted!";
						netconn_write(conn, response, strlen(response), NETCONN_COPY);
						conversionSuccess = 0;
					}
				}
				else if(isBase == 0)
				{
					// convert from char array into ip4_addr_t
					if (!ip4addr_aton(deviceIp, &deviceAddr) ||
						!ip4addr_aton(vehicleIp, &vehicleAddr) ||
						!ip4addr_aton(netmask, &netmaskAddr) ||
						!ip4addr_aton(gateway, &gatewayAddr))
					{
						const char *response = "HTTP/1.1 200 OK\r\nContent-Type: text/plain\r\n\r\nIP address error!\nChanges not accepted!";
						netconn_write(conn, response, strlen(response), NETCONN_COPY);
						conversionSuccess = 0;
					}
				}

				if(conversionSuccess)
				{
					Flash_Read_Data(FLASH_USER_SECTOR_11, (uint8_t *)&flashIpConfig, sizeof(flashIpConfig));
					if(isBase == 1)
					{
						flashIpConfig.ownIP = deviceAddr;
						flashIpConfig.rtcmBroadcastIP = RtcmBroadcastAddr;
						flashIpConfig.rtcmUdpPort = rtcmPortNum;
						flashIpConfig.netmask = netmaskAddr;
						flashIpConfig.gateway = gatewayAddr;
					}
					else
					{
						flashIpConfig.ownIP = deviceAddr;
						flashIpConfig.vehicleIP = vehicleAddr;
						flashIpConfig.rtcmUdpPort = rtcmPortNum;
						flashIpConfig.vehicleUdpPort = vehiclePortNum;
						flashIpConfig.netmask = netmaskAddr;
						flashIpConfig.gateway = gatewayAddr;
					}

					// write IP configuration into the flash
					Flash_Write_Data(FLASH_USER_SECTOR_11, (uint8_t *)&flashIpConfig, sizeof(flashIpConfig));

					// send success message
					char response[200];
					snprintf(response, sizeof(response), "HTTP/1.1 200 OK\r\nContent-Type: text/plain\r\n\r\nIP configuration updated successfully.\n"
							"System restarts in 5 seconds to apply changes!\nWebsite reachable at IP address.: %s", deviceIp);
					netconn_write(conn, response, strlen(response), NETCONN_COPY);

					/* System restart in x seconds with changed IP configuration */
					restartSystem = 1;
				}
			}
			else if (strncmp((char const *)buf, "GET /get_ip_settings", 20) == 0) {
			    FlashData_t flashIpConfig;
			    Flash_Read_Data(FLASH_USER_SECTOR_11, (uint8_t *)&flashIpConfig, sizeof(flashIpConfig));

			    char deviceIP[16];
			    sprintf(deviceIP, "%lu.%lu.%lu.%lu", (flashIpConfig.ownIP.addr) & 0xFF, (flashIpConfig.ownIP.addr >> 8) & 0xFF, (flashIpConfig.ownIP.addr >> 16) & 0xFF, (flashIpConfig.ownIP.addr >> 24) & 0xFF);

			    char netmask[16];
			    sprintf(netmask, "%lu.%lu.%lu.%lu", (flashIpConfig.netmask.addr) & 0xFF, (flashIpConfig.netmask.addr >> 8) & 0xFF, (flashIpConfig.netmask.addr >> 16) & 0xFF, (flashIpConfig.netmask.addr >> 24) & 0xFF);

			    char gateway[16];
			    sprintf(gateway, "%lu.%lu.%lu.%lu", (flashIpConfig.gateway.addr) & 0xFF, (flashIpConfig.gateway.addr >> 8) & 0xFF, (flashIpConfig.gateway.addr >> 16) & 0xFF, (flashIpConfig.gateway.addr >> 24) & 0xFF);

			    char rtcmPort[6];
			    sprintf(rtcmPort, "%u", flashIpConfig.rtcmUdpPort);

			    char rtcmIP[16];
			    sprintf(rtcmIP, "%lu.%lu.%lu.%lu", (flashIpConfig.rtcmBroadcastIP.addr) & 0xFF, (flashIpConfig.rtcmBroadcastIP.addr >> 8) & 0xFF, (flashIpConfig.rtcmBroadcastIP.addr >> 16) & 0xFF, (flashIpConfig.rtcmBroadcastIP.addr >> 24) & 0xFF);

			    char vehiclePort[6];
			    sprintf(vehiclePort, "%u", flashIpConfig.vehicleUdpPort);

			    char vehicleIP[16];
			    sprintf(vehicleIP, "%lu.%lu.%lu.%lu", (flashIpConfig.vehicleIP.addr) & 0xFF, (flashIpConfig.vehicleIP.addr >> 8) & 0xFF, (flashIpConfig.vehicleIP.addr >> 16) & 0xFF, (flashIpConfig.vehicleIP.addr >> 24) & 0xFF);

			    char *response_header;
			    response_header = static_cast<char *>(pvPortMalloc(200));
			    int header_len = sprintf(response_header, "HTTP/1.1 200 OK\r\nContent-Type: text/plain\r\n%s\r\n", cors_headers);

			    // Send the response header with CORS headers
			    netconn_write(conn, (const unsigned char *)response_header, (size_t)header_len, NETCONN_NOCOPY);

			    vPortFree(response_header);

			    char *pagedata;
			    pagedata = static_cast<char *>(pvPortMalloc(200));
			    int len = sprintf(pagedata, "%s|%s|%s|%s|%s|%s|%s", deviceIP, netmask, gateway, rtcmPort, rtcmIP, vehiclePort, vehicleIP);

			    netconn_write(conn, (const unsigned char *)pagedata, (size_t)len, NETCONN_NOCOPY);

			    vPortFree(pagedata);
			    osDelay(20);
			}
			else
			{
				/* Load Error page */
				fs_open(&file, "/404.html");
				netconn_write(conn, (const unsigned char*)(file.data), (size_t)file.len, NETCONN_NOCOPY);
				fs_close(&file);
			}
		}
	}
	/* Close the connection (server closes in HTTP) */
	netconn_close(conn);

	/* Delete the buffer (netconn_recv gives us ownership,
   so we have to make sure to deallocate the buffer) */
	netbuf_delete(inbuf);

	/* restart system if flag is set to 1 */
	if(restartSystem)
		NVIC_SystemReset();
}


static void http_thread(void *arg)
{ 
  struct netconn *conn, *newconn;
  err_t err, accept_err;
  
  /* Create a new TCP connection handle */
  conn = netconn_new(NETCONN_TCP);
  
  if (conn!= NULL)
  {
    /* Bind to port 80 (HTTP) with default IP address */
    err = netconn_bind(conn, IP_ADDR_ANY, 80);
    
    if (err == ERR_OK)
    {
      /* Put the connection into LISTEN state */
      netconn_listen(conn);
  
      while(1) 
      {
        /* accept any incoming connection */
        accept_err = netconn_accept(conn, &newconn);
        if(accept_err == ERR_OK)
        {
          /* serve connection */
          http_server(newconn);

          /* delete connection */
          netconn_delete(newconn);
        }
      }
    }
  }
}

extern "C" void http_server_init(void)
{
  sys_thread_new("http_thread", http_thread, NULL, 2048, osPriorityHigh);
}




