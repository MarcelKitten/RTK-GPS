/*
 * globals.cpp
 *
 *  Created on: Apr 13, 2023
 *      Author: marcel
 */

#include "globals.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "main.h"
#include "string.h"
#include "flash.h"

extern SemaphoreHandle_t globalsSemaphore;

QueueHandle_t ubloxRx;

SurveyInStatus svinVars;
TmodeConfig tmode;
NavPvt navPvt;
uint8_t isBase;

UbxProtocol myUbx(&UART1_UBLOX, &ubloxRx);

void getSurveyInStatus(SurveyInStatus *dst) {
    xSemaphoreTake(globalsSemaphore, portMAX_DELAY);
    memcpy(dst, &svinVars, sizeof(SurveyInStatus));
    xSemaphoreGive(globalsSemaphore);
}

void setSurveyInStatus(const SurveyInStatus *src) {
    xSemaphoreTake(globalsSemaphore, portMAX_DELAY);
    memcpy(&svinVars, src, sizeof(SurveyInStatus));
    xSemaphoreGive(globalsSemaphore);
}

void getTmodeConfig(TmodeConfig *dst) {
    xSemaphoreTake(globalsSemaphore, portMAX_DELAY);
    memcpy(dst, &tmode, sizeof(TmodeConfig));
    xSemaphoreGive(globalsSemaphore);
}

void setTmodeConfig(const TmodeConfig *src) {
    xSemaphoreTake(globalsSemaphore, portMAX_DELAY);
    memcpy(&tmode, src, sizeof(TmodeConfig));
    xSemaphoreGive(globalsSemaphore);
}

void getNavPvt(NavPvt *dst) {
    xSemaphoreTake(globalsSemaphore, portMAX_DELAY);
    memcpy(dst, &navPvt, sizeof(NavPvt));
    xSemaphoreGive(globalsSemaphore);
}

void setNavPvt(const NavPvt *src) {
    xSemaphoreTake(globalsSemaphore, portMAX_DELAY);
    memcpy(&navPvt, src, sizeof(NavPvt));
    xSemaphoreGive(globalsSemaphore);
}
