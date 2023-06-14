/**
  ******************************************************************************
  * @file           : globals.h
  * @brief          : Global variables and safe access to them
  * @author			: Marcel Lammerskitten
  * @date			: 24/05/23
  * @version		: 0.1
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 MobileTronics
  *
  ******************************************************************************
  */
#ifndef INC_GLOBALS_H_
#define INC_GLOBALS_H_

#include <UBX_PROTO.hpp>
#include "flash.h"

/* Defines */
#define UART1_UBLOX 	huart6
#define UART2_UBLOX 	huart7

/* Extern defines */
extern UART_HandleTypeDef huart6;
extern UART_HandleTypeDef huart7;
extern SurveyInStatus svinVars;
extern TmodeConfig tmode;
extern NavPvt navPvt;
extern QueueHandle_t ubloxRx;

extern UbxProtocol myUbx;

/**
 * @brief Shows Mode
 *
 * 0: Rover, 1: Base, 2: Unknown/Error
 */
extern uint8_t isBase;


/**
 * @brief			Safely get Survey-In status
 * @param [out]		dst		Pointer to variable which should store the data
 */
void getSurveyInStatus(SurveyInStatus *dst);

/**
 * @brief			Safely set Survey-In status
 * @param [out]		src		Pointer to variable which store the data
 */
void setSurveyInStatus(const SurveyInStatus *src);

/**
 * @brief			Safely get Tmode
 * @param [out]		dst		Pointer to variable which should store the data
 */
void getTmodeConfig(TmodeConfig *dst);

/**
 * @brief			Safely set Tmode
 * @param [out]		src		Pointer to variable which store the data
 */
void setTmodeConfig(const TmodeConfig *src);

/**
 * @brief			Safely get NavPvt status
 * @param [out]		dst		Pointer to variable which should store the data
 */
void getNavPvt(NavPvt *dst);

/**
 * @brief			Safely set NavPvt status
 * @param [out]		src		Pointer to variable which store the data
 */
void setNavPvt(const NavPvt *src);


#endif /* INC_GLOBALS_H_ */
