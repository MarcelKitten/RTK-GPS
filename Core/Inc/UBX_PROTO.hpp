/**
  ******************************************************************************
  * @file           : UBX_PROTO.h
  * @brief          : UBX Protocol for U-blox ZED-F9P
  *
  * With this class you can communicate with u-blox modules via the UBX-protocol and you can easily
  * build new/more messages.
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

/*
 * TODO:	- improve rover method
 * 			- delete the layer options and make changes always in a loop with all layers
 */

#ifndef INC_UBX_PROTO_HPP_
#define INC_UBX_PROTO_HPP_

/* Includes */
#include <vector>
#include "main.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

using namespace std;

struct SurveyInStatus {
    uint8_t version;
    uint8_t reserved0;
    uint32_t timeOfWeekMs;
    uint32_t dur;
    int32_t meanX;
    int32_t meanY;
    int32_t meanZ;
    int8_t meanXHP;
    int8_t meanYHP;
    int8_t meanZHP;
    uint8_t reserved1;
    uint32_t meanAcc;
    uint32_t obsUsed;
    uint8_t valid;
    uint8_t active;
    uint8_t reserved2;
};

struct TmodeConfig {
    uint8_t tmodeMode;
    uint32_t svinMinDur;
    uint32_t svinVarLimit;
    int32_t ecefX;
    int32_t ecefY;
    int32_t ecefZ;
};

struct NavPvt {
    uint8_t fixType;
    uint8_t numSatUsed;
    int32_t hMSL;		// Height above mean sea level
    uint32_t hAcc;		// Horizontal accuracy
    uint32_t vAcc;		// Vertical accuracy
};


/**
 * @class 	UbxProtocol
 * @brief	UBX protocol to communicate with u-blox modules
 * With this class you can communicate with u-blox modules via the UBX-protocol and you can easily
 * build new/more messages.
 */
class UbxProtocol {
	/* ToDo: delete copy constructor and equal operator*/

  public:
	/// Constructor
    UbxProtocol(UART_HandleTypeDef* huart, QueueHandle_t* buf);
    /// Deconstructor
    ~UbxProtocol();
    /// Clearing the queue
    void flushQueue();

    /**
     * @brief	Send UBX-messages
     * @param [in]	msgClass	transfer MessageClass
     * @param [in]	payload		transfer payload
     * @return	true if message sent and receive ack - else false
     */
    bool sendUbxMsg(vector<uint8_t> &MsgClass, vector<uint8_t> *payload);

    /**
     * @brief	Receive UBX-messages
     * @param [in]	msgClass	received MessageClass
     * @param [in]	payload		received payload
     * @return	true if calculated and received checksum are the same
     */
    bool receiveUbxMessage(vector<uint8_t> &MsgClass, vector<uint8_t> *payload);

    /**
     * @brief	Receive ACK and NAK
     * @param [in]	msgClass	transmitted MessageClass
     * @param [in]	ack			true = ACK, false = NAK
     * @return	true if no errors
     */
    bool ackNakReceived(const vector<uint8_t> &msgClass, bool &ack);


    // Todo: Set up the UART1 port
    void sendUart1SetUp(uint32_t baudrate = 38400, uint8_t stopbits = 1, uint8_t databits = 8, uint8_t parity = 0,
    					bool enabled = true);

    /**
     * @brief Set up UART1 interface
     * @param [in] enabled 	If true output enabled, else disabled
     * @param [in] layer	Set the Layer: 0 = Ram, 1 = BBR, 2 = Flash
     * @return 				true if operation successful
     */
    bool sendUart1NMEAOutput(bool enabled, uint8_t layer);

    /**
     * @brief 		Sets the GPS module as a stationary base with survey-in
     * @param [in] 	minDur		Set the minimal duration for the Survey-in
     * @param [in]	accLimit	Set the position accuracy limit
     * @return 					true if operation successful
     */
    bool setBaseStationSurveyIn(uint32_t minDur, uint32_t accLimit);

    /**
     * @brief 		Sets the GPS module as a stationary base in fixed mode
     * @param [in] 	ecef_x		ECEF X coordinate of the ARP position
     * @param [in] 	ecef_y		ECEF Y coordinate of the ARP position
     * @param [in] 	ecef_z		ECEF Z coordinate of the ARP position
     * @return 					true if operation successful
     */
    bool setBaseStationFixed(int32_t ecef_x, int32_t ecef_y, int32_t ecef_z);

    /**
     * @brief 		Sets the GPS module as a Rover
     * @return 					true if operation successful
     */
    bool setRover();

    /**
	 * @brief 		Sets the GPS modules measurement rate
	 * @param [in] 	measRate	Set the measurement rate in milliseconds
	 * @return 					true if operation successful
	 */
    bool setMeasRate(uint16_t measRate);

    /**
	 * @brief 		Sets the GPS modules UART2 baud rate
	 * @param [in] 	baudRate	Baud rate
	 * @return 					true if operation successful
	 */
    bool setUART2BaudRate(uint32_t baudRate);

    // Get status informations
    /**
     * @brief               Get survey-in status and information
     * @param [out] svin    Pointer to a struct to store the Survey-In status and information
     * @return              true if operation successful
     */
    bool getSurveyInStatus();

    /**
     * @brief               Get navigation status
     * @param [out] svin    Pointer to a struct to store the NavPvt status and information
     * @return              true if operation successful
     */
    bool getNavPvt();

    /**
     * @brief               Get survey-in status and information
     * @param [out] tmode   Pointer to a variable to store the current Time Mode
     * @return              true if operation successful
     */
    bool getTmodeMode();

  private:
    /**
     * @brief Function to calculate the 2-byte checksum
     * @param [in] message Pointer to the UBX payload
     * @param [out] checksum Returns the 2-Byte checksum
     */
    void calculateChecksum(const vector<uint8_t>& message, uint8_t* checksum);

    /**
     * @brief Function to calculate the payload length
     * @param [in] payload  Reference to the UBX payload
     * @return  vector with length of UBX message payload in Little Endian
     */
    vector<uint8_t> calculatePayloadLength(const vector<uint8_t>& payload);
    bool verifyChecksum(const uint8_t *buffer, uint16_t length);

    // variables
    UART_HandleTypeDef* m_huart;
    uint8_t m_UbxSync[2] = { 0xb5, 0x62 };
    QueueHandle_t* m_buf;
};

#endif /* INC_UBX_PROTO_HPP_ */
