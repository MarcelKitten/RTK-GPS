/*
 * UBX_PROTO.cpp
 *
 *  Created on: Mar 13, 2023
 *      Author: marcel
 */
/**
  ******************************************************************************
  * @file           : UBX_PROTO.cpp
  * @brief          : UBX Protocol for U-blox ZED-F9P
  * @author			: Marcel Lammerskitten
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 MobileTronics
  *
  ******************************************************************************
  */

/*
 * ToDo:	-
 */

#include <UBX_PROTO.hpp>
#include <vector>
#include "globals.h"


/**
 * @brief	Store important message classes
 *
 * Has to be continued..
 */
struct UBXMsgClass {
    vector<uint8_t> NAV = {0x01, 0x01};
    vector<uint8_t> RXM = {0x02, 0x01};
    vector<uint8_t> INF = {0x04, 0x01};
    vector<uint8_t> CFG = {0x06, 0x01};
    vector<uint8_t> UPD = {0x09, 0x01};
    vector<uint8_t> MON = {0x0A, 0x01};
    vector<uint8_t> MON_TMODE3 = {0x0A, 0x09};
    vector<uint8_t> AID = {0x0B, 0x01};
    vector<uint8_t> TIM = {0x0D, 0x01};
    vector<uint8_t> ESF = {0x10, 0x01};
    // ACK
    vector<uint8_t> ACK = {0x05, 0x01};
    vector<uint8_t> NACK = {0x05, 0x00};

    // Messages for configuration
    vector<uint8_t> CFG_VALSET = {0x06, 0x8A}; // Set values
    vector<uint8_t> CFG_VALGET = {0x06, 0x8B}; // Get values
    vector<uint8_t> CFG_MSG = {0x06, 0x01, 0x01}; // Set message rates
    vector<uint8_t> CFG_PRT = {0x06, 0x00}; // Configure UART, USB & co.
    vector<uint8_t> CFG_RATE = {0x06, 0x08}; // Set navigation and measurement rate
    vector<uint8_t> CFG_CFG = {0x06, 0x09}; // Set current configuration
    vector<uint8_t> CFG_RST = {0x06, 0x04}; // Reset receiver to default configuration
    vector<uint8_t> CFG_NAV5 = {0x06, 0x24}; // Set Navigation Engine Settings
    vector<uint8_t> CFG_CFG_CONFIG_DATA = {0x06, 0x0B}; // Save current configuration to flash memory

    // to be continued

} UBXMsgClass;

/**
 * @brief	Store CFG_VALSET messages
 */
struct UBX_CFG_VALSET {
	// Byte 0 & 1
    vector<uint8_t> LayerRAM 	= {0x00, 0x01}; /**< Save current configuration to RAM */
    vector<uint8_t> LayerBBR 	= {0x00, 0x02}; /**< Save current configuration to BBR */
    vector<uint8_t> LayerFlash	= {0x00, 0x04}; /**< Save current configuration to flash memory */
    // Byte 2 & 3
    vector<uint8_t> Transactionless	= {0x00, 0x00};
} UBX_CFG_VALSET;

/**
 * @brief	Store important message classes
 *
 * Has to be continued..
 */
struct UBXConfigKeys {
	/* TMODE */
	vector<uint8_t> TmodeMode = { 0x01, 0x00, 0x03, 0x20 };   				/**< Value byte: 0 = disable; 1 = survey_in; 2 = fixed */
	vector<uint8_t> TmodePosType = { 0x02, 0x00, 0x03, 0x20 };   			/**< Value byte: 0 = ECEF; 1 = LLH */
	vector<uint8_t> TmodeEcefX= { 0x03, 0x00, 0x03, 0x40 };   				/**< Value int32 */
	vector<uint8_t> TmodeEcefY= { 0x04, 0x00, 0x03, 0x40 };   				/**< Value int32 */
	vector<uint8_t> TmodeEcefZ= { 0x05, 0x00, 0x03, 0x40 };   				/**< Value int32 */
	vector<uint8_t> TmodeSvinMinDur = { 0x10, 0x00, 0x03, 0x40 };   		/**< Value uint32: min duration in seconds */
	vector<uint8_t> TmodeSvinAccLimit = { 0x11, 0x00, 0x03, 0x40 };   		/**< Value uint32: accuracy limit (mm scale 0.1) */

	/* Rate */
	vector<uint8_t> RateMeas= { 0x01, 0x00, 0x21, 0x30 };   				/**< Value uint16: in milliseconds */

	/* Message output Configuration */
	vector<uint8_t> MsgOutRtcm3x1005Uart1 = { 0xBE, 0x02, 0x91, 0x20 };		/**< Value byte: e.g. 1 output rate */
	vector<uint8_t> MsgOutRtcm3x1074Uart1 = { 0x5F, 0x03, 0x91, 0x20 };   	/**< Value byte: e.g. 1 output rate */
	vector<uint8_t> MsgOutRtcm3x1084Uart1 = { 0x64, 0x03, 0x91, 0x20 };   	/**< Value byte: e.g. 1 output rate */
	vector<uint8_t> MsgOutRtcm3x1094Uart1 = { 0x69, 0x03, 0x91, 0x20 };   	/**< Value byte: e.g. 1 output rate */
	vector<uint8_t> MsgOutRtcm3x1124Uart1 = { 0x6E, 0x03, 0x91, 0x20 };   	/**< Value byte: e.g. 1 output rate */
	vector<uint8_t> MsgOutRtcm3x1230Uart1 = { 0x04, 0x03, 0x91, 0x20 };   	/**< Value byte: e.g. 1 output rate */

	vector<uint8_t> MsgOutRtcm3x1005Uart2 = { 0xBF, 0x02, 0x91, 0x20 }; 	/**<  Output rate for RTCM3 1005 messages */
	vector<uint8_t> MsgOutRtcm3x1074Uart2 = { 0x60, 0x03, 0x91, 0x20 }; 	/**<  Output rate for RTCM3 1074 messages */
	vector<uint8_t> MsgOutRtcm3x1084Uart2 = { 0x65, 0x03, 0x91, 0x20 }; 	/**<  Output rate for RTCM3 1084 messages */
	vector<uint8_t> MsgOutRtcm3x1094Uart2 = { 0x6A, 0x03, 0x91, 0x20 }; 	/**<  Output rate for RTCM3 1094 messages */
	vector<uint8_t> MsgOutRtcm3x1124Uart2 = { 0x6F, 0x03, 0x91, 0x20 }; 	/**<  Output rate for RTCM3 1124 messages */
	vector<uint8_t> MsgOutRtcm3x1230Uart2 = { 0x05, 0x03, 0x91, 0x20 }; 	/**<  Output rate for RTCM3 1230 messages */

	vector<uint8_t> MsgOutUbxPvtUart2 = { 0x08, 0x00, 0x91, 0x20 }; 		/**< Value byte: e.g. 1 output rate */
	vector<uint8_t> MsgOutUbxRelPosNEDUart2 = { 0x8F, 0x00, 0x91, 0x20 }; 	/**< Value byte: e.g. 1 output rate */
	vector<uint8_t> MsgOutUbxSatUart2 = { 0x17, 0x00, 0x91, 0x20 }; 		/**< Value byte: e.g. 1 output rate */
	vector<uint8_t> MsgOutUbxSigUart2 = { 0x47, 0x03, 0x91, 0x20 }; 		/**< Value byte: e.g. 1 output rate */
	vector<uint8_t> MsgOutUbxStatusUart2 = { 0x1C, 0x00, 0x91, 0x20 }; 		/**< Value byte: e.g. 1 output rate */

    /* UART configuration */
	// Baud rate
	vector<uint8_t> Uart1Baudrate = { 0x01, 0x00, 0x52, 0x40 };
	vector<uint8_t> Uart2Baudrate = { 0x01, 0x00, 0x53, 0x40 };

    vector<uint8_t> Uart1InProtUBX = { 0x01, 0x00, 0x73, 0x10 };
    vector<uint8_t> Uart1InProtNMEA = { 0x02, 0x00, 0x73, 0x10 };
    vector<uint8_t> Uart1InProtRTCM3 = { 0x04, 0x00, 0x73, 0x10 };
    vector<uint8_t> Uart1OutProtUBX = { 0x01, 0x00, 0x74, 0x10 };
    vector<uint8_t> Uart1OutProtNMEA = { 0x02, 0x00, 0x74, 0x10 };
    vector<uint8_t> Uart1OutProtRTCM3 = { 0x04, 0x00, 0x74, 0x10 };

    vector<uint8_t> Uart2InProtUBX = { 0x01, 0x00, 0x75, 0x10 };
    vector<uint8_t> Uart2InProtNMEA = { 0x02, 0x00, 0x75, 0x10 };
    vector<uint8_t> Uart2InProtRTCM3 = { 0x04, 0x00, 0x75, 0x10 };
    vector<uint8_t> Uart2OutProtUBX = { 0x01, 0x00, 0x76, 0x10 };
    vector<uint8_t> Uart2OutProtNMEA = { 0x02, 0x00, 0x76, 0x10 };
    vector<uint8_t> Uart2OutProtRTCM3 = { 0x04, 0x00, 0x76, 0x10 };
} UBXConfigKeys;



// only for testing...
struct CFG_UART1_PROT
{
	vector<uint8_t> OUT_NMEA_OFF =
	{ 	0xb5, 0x62, 0x06, 0x8a, 0x09, 0x00, 0x00, 0x01, 0x00, 0x00, 0x02, 0x00, 0x74, 0x10, 0x00, 0x20, 0xb7,
		0xb5, 0x62, 0x06, 0x8a, 0x09, 0x00, 0x00, 0x02, 0x00, 0x00, 0x02, 0x00, 0x74, 0x10, 0x00, 0x21, 0xbf,
		0xb5, 0x62, 0x06, 0x8a, 0x09, 0x00, 0x00, 0x04, 0x00, 0x00, 0x02, 0x00, 0x74, 0x10, 0x00, 0x23, 0xcf
	};
	vector<uint8_t> OUT_NMEA_ON =
	{ 	0xb5, 0x62, 0x06, 0x8a, 0x09, 0x00, 0x00, 0x01, 0x00, 0x00, 0x02, 0x00, 0x74, 0x10, 0x01, 0x21, 0xb8,
		0xb5, 0x62, 0x06, 0x8a, 0x09, 0x00, 0x00, 0x02, 0x00, 0x00, 0x02, 0x00, 0x74, 0x10, 0x01, 0x22, 0xc0,
		0xb5, 0x62, 0x06, 0x8a, 0x09, 0x00, 0x00, 0x04, 0x00, 0x00, 0x02, 0x00, 0x74, 0x10, 0x01, 0x24, 0xd0
	};
} CFG_UART1_PROT;


UbxProtocol::UbxProtocol(UART_HandleTypeDef* huart, QueueHandle_t* buf) : m_huart(huart), m_buf(buf)
{

}


UbxProtocol::~UbxProtocol()
{

}

void UbxProtocol::flushQueue() {
    uint8_t data;
    while (xQueueReceive(*m_buf, &data, pdMS_TO_TICKS(1)) == pdPASS) {
        // Do nothing, just clearing the queue.
    }
}


void UbxProtocol::calculateChecksum(const vector<uint8_t>& message, uint8_t* checksum)
{
	uint8_t ck_a = 0, ck_b = 0;
    for (uint16_t i = 0; i < message.size(); i++) {
        ck_a += message[i];
        ck_b += ck_a;
    }
    checksum[0] = ck_a;
    checksum[1] = ck_b;
}


vector<uint8_t> UbxProtocol::calculatePayloadLength(const vector<uint8_t>& payload)
{
    uint16_t payloadSize = static_cast<uint16_t>(payload.size());
    vector<uint8_t> lengthBytes(2);
    lengthBytes[0] = static_cast<uint8_t>(payloadSize);
    lengthBytes[1] = static_cast<uint8_t>(payloadSize >> 8);
    return lengthBytes;
}


bool UbxProtocol::sendUbxMsg(vector<uint8_t> &msgClass, vector<uint8_t> *payload)
{
	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
	vector<uint8_t> message;

	// insert message class and message ID
	message.push_back(msgClass[0]);
	message.push_back(msgClass[1]);

	// calculate the payload length
	vector<uint8_t> payloadSize = calculatePayloadLength(*payload);

	// push back the payload length (LE)
	message.push_back(payloadSize[0]);
	message.push_back(payloadSize[1]);

	// attach payload
	message.insert(message.end(), payload->begin(), payload->end());

	// calculate checksum
	uint8_t checksum[2];
	calculateChecksum(message, checksum);

	// attach checksum
	message.push_back(checksum[0]);
	message.push_back(checksum[1]);

	// insert header in front
	message.insert(message.begin(), m_UbxSync[1]);
	message.insert(message.begin(), m_UbxSync[0]);

	// sending message
//	HAL_UART_Transmit(m_huart, message.data(), message.size(), HAL_MAX_DELAY);
	HAL_UART_Transmit_IT(m_huart, message.data(), message.size());

	// checking for ACK or NACK
	if(msgClass[0]  == 0x06 && msgClass[1] == 0x8A)
	{
		bool ack = false;
		ackNakReceived(msgClass, ack);
		HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
		if(ack)
		{
			HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
			HAL_Delay(10);
			HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
			return true;
		}
		else
		{
			HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET);
			HAL_Delay(10);
			HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);
			return false;
		}
	}
	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
	return true;
}


bool UbxProtocol::receiveUbxMessage(vector<uint8_t>& msgClass, vector<uint8_t>* payload) {
    uint8_t data;

    // searching first sync char
    do {
        if (xQueueReceive(*m_buf, &data, pdMS_TO_TICKS(500)) != pdPASS) {
            return false;
        }
    } while (data != m_UbxSync[0]);

    // searching second
    if (xQueueReceive(*m_buf, &data, pdMS_TO_TICKS(111)) != pdPASS || data != m_UbxSync[1]) {
        HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET);
        HAL_Delay(1000);
        HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);
        return false;
    }

    // read message class and message ID
    uint8_t msgClassData;
    for (int i = 0; i < 2; ++i) {
        if (xQueueReceive(*m_buf, &msgClassData, pdMS_TO_TICKS(111)) != pdPASS) {
            return false;
        }
        msgClass.push_back(msgClassData);
    }

    // read payload length.
    uint8_t lengthLow, lengthHigh;
    if (xQueueReceive(*m_buf, &lengthLow, pdMS_TO_TICKS(111)) != pdPASS || xQueueReceive(*m_buf, &lengthHigh, pdMS_TO_TICKS(111)) != pdPASS) {
        return false;
    }
    uint16_t payloadLength = (lengthHigh << 8) | lengthLow;

    // read payload
    uint8_t payloadData;
    for (uint16_t i = 0; i < payloadLength; ++i) {
        if (xQueueReceive(*m_buf, &payloadData, pdMS_TO_TICKS(111)) != pdPASS) {
            return false;
        }
        payload->push_back(payloadData);
    }

    // read and check the checksum
    uint8_t receivedChecksum[2];
    if (xQueueReceive(*m_buf, &receivedChecksum[0], pdMS_TO_TICKS(111)) != pdPASS || xQueueReceive(*m_buf, &receivedChecksum[1], pdMS_TO_TICKS(111)) != pdPASS) {
        return false;
    }

    uint8_t calculatedChecksum[2] = {0, 0};
    vector<uint8_t> checkMsg = msgClass;
    checkMsg.push_back(payloadLength & 0xFF);
    checkMsg.push_back(payloadLength >> 8);
    checkMsg.insert(checkMsg.end(), payload->begin(), payload->end());
    calculateChecksum(checkMsg, calculatedChecksum);
    if(receivedChecksum[0] == calculatedChecksum[0] && receivedChecksum[1] == calculatedChecksum[1])
    	return true;
    return false;
}


bool UbxProtocol::ackNakReceived(const vector<uint8_t> &msgClass, bool &ack) {
    vector<uint8_t> receivedMsgClass;
    vector<uint8_t> receivedPayload;

    if (receiveUbxMessage(receivedMsgClass, &receivedPayload)) {
        // check if received message is ACK or NAK
        if (receivedMsgClass[0] == 0x05 && (receivedMsgClass[1] == 0x00 || receivedMsgClass[1] == 0x01)) {
            // check if message Class is correct
            if (msgClass[0] == receivedPayload[0] && msgClass[1] == receivedPayload[1]) {
                ack = (receivedMsgClass[1] == 0x01);
                return true;
            }
        }
    }

    return false;
}


/**
 * @brief Set up UART1 interface
 * @param [in] baudrate Set baudrate
 * @param [in] stopbits Set stopbits
 * @param [in] databits Set databits
 * @param [in] parity Set parity
 * @param [in] enabled Enable or disable UART1
 */
void UbxProtocol::sendUart1SetUp(uint32_t baudrate, uint8_t stopbits, uint8_t databits, uint8_t parity, bool enabled)
{
	// Todo

}

bool UbxProtocol::sendUart1NMEAOutput(bool enabled, uint8_t layer)
{
	vector<uint8_t> msgClass = { UBXMsgClass.CFG_VALSET[0], UBXMsgClass.CFG_VALSET[1]};
	bool success = false;

	for (uint8_t currentLayer = 0; currentLayer < 3; ++currentLayer)
	{
		vector<uint8_t> payload;

		if (currentLayer == 0)
		{
			payload = {UBX_CFG_VALSET.LayerRAM[0], UBX_CFG_VALSET.LayerRAM[1], UBX_CFG_VALSET.Transactionless[0], UBX_CFG_VALSET.Transactionless[1],
					   UBXConfigKeys.Uart1OutProtNMEA[0], UBXConfigKeys.Uart1OutProtNMEA[1], UBXConfigKeys.Uart1OutProtNMEA[2], UBXConfigKeys.Uart1OutProtNMEA[3]};
		}
		else if (currentLayer == 1)
		{
			payload = {UBX_CFG_VALSET.LayerBBR[0], UBX_CFG_VALSET.LayerBBR[1], UBX_CFG_VALSET.Transactionless[0], UBX_CFG_VALSET.Transactionless[1],
					   UBXConfigKeys.Uart1OutProtNMEA[0], UBXConfigKeys.Uart1OutProtNMEA[1], UBXConfigKeys.Uart1OutProtNMEA[2], UBXConfigKeys.Uart1OutProtNMEA[3]};
		}
		else if (currentLayer == 2)
		{
			payload = {UBX_CFG_VALSET.LayerFlash[0], UBX_CFG_VALSET.LayerFlash[1], UBX_CFG_VALSET.Transactionless[0], UBX_CFG_VALSET.Transactionless[1],
					   UBXConfigKeys.Uart1OutProtNMEA[0], UBXConfigKeys.Uart1OutProtNMEA[1], UBXConfigKeys.Uart1OutProtNMEA[2], UBXConfigKeys.Uart1OutProtNMEA[3]};
		}

		if (enabled)
		{
			payload.push_back(0x01);
		}
		else
		{
			payload.push_back(0x00);
		}

		if (sendUbxMsg(msgClass, &payload))
		{
			success = true;
		}
		else
		{
			success = false;
			break;
		}
	}

	return success;

}

bool UbxProtocol::setBaseStationSurveyIn(uint32_t minDur, uint32_t accLimit)
{
	vector<uint8_t> msgClass = {UBXMsgClass.CFG_VALSET[0], UBXMsgClass.CFG_VALSET[1]};

	// Iterate through all three layers
	for (uint8_t currentLayer = 0; currentLayer <= 2; ++currentLayer)
	{
		vector<uint8_t> payload;
		if (currentLayer == 0)
		{
			payload = {UBX_CFG_VALSET.LayerRAM[0], UBX_CFG_VALSET.LayerRAM[1], UBX_CFG_VALSET.Transactionless[0], UBX_CFG_VALSET.Transactionless[1]};
		}
		else if (currentLayer == 1)
		{
			payload = {UBX_CFG_VALSET.LayerBBR[0], UBX_CFG_VALSET.LayerBBR[1], UBX_CFG_VALSET.Transactionless[0], UBX_CFG_VALSET.Transactionless[1]};
		}
		else if (currentLayer == 2)
		{
			payload = {UBX_CFG_VALSET.LayerFlash[0], UBX_CFG_VALSET.LayerFlash[1], UBX_CFG_VALSET.Transactionless[0], UBX_CFG_VALSET.Transactionless[1]};
		}

		// Copy payload with the layer information into payloadTemp
		vector<uint8_t> payloadTemp = payload;

		// Model selection (survey)
		payload.insert(payload.end(), UBXConfigKeys.TmodeMode.begin(), UBXConfigKeys.TmodeMode.end());
		payload.push_back(0x01); // Survey mode

		// Set min duration
		payload.insert(payload.end(), UBXConfigKeys.TmodeSvinMinDur.begin(), UBXConfigKeys.TmodeSvinMinDur.end());
		payload.push_back(minDur & 0xFF);
		payload.push_back((minDur >> 8) & 0xFF);
		payload.push_back((minDur >> 16) & 0xFF);
		payload.push_back((minDur >> 24) & 0xFF);

		// Set accuracy limit
		payload.insert(payload.end(), UBXConfigKeys.TmodeSvinAccLimit.begin(), UBXConfigKeys.TmodeSvinAccLimit.end());
		payload.push_back(accLimit & 0xFF);
		payload.push_back((accLimit >> 8) & 0xFF);
		payload.push_back((accLimit >> 16) & 0xFF);
		payload.push_back((accLimit >> 24) & 0xFF);

		// Send survey-in configuration
		if (!sendUbxMsg(msgClass, &payload))
			return false;

		// Enable RTCM3.X outputs
		payload.insert(payload.end(), UBXConfigKeys.MsgOutRtcm3x1005Uart2.begin(), UBXConfigKeys.MsgOutRtcm3x1005Uart2.end());
		payload.push_back(0x01);
		payload.insert(payload.end(), UBXConfigKeys.MsgOutRtcm3x1074Uart2.begin(), UBXConfigKeys.MsgOutRtcm3x1074Uart2.end());
		payload.push_back(0x01);
		payload.insert(payload.end(), UBXConfigKeys.MsgOutRtcm3x1084Uart2.begin(), UBXConfigKeys.MsgOutRtcm3x1084Uart2.end());
		payload.push_back(0x01);
		payload.insert(payload.end(), UBXConfigKeys.MsgOutRtcm3x1094Uart2.begin(), UBXConfigKeys.MsgOutRtcm3x1094Uart2.end());
		payload.push_back(0x01);
		payload.insert(payload.end(), UBXConfigKeys.MsgOutRtcm3x1124Uart2.begin(), UBXConfigKeys.MsgOutRtcm3x1124Uart2.end());
		payload.push_back(0x01);
		payload.insert(payload.end(), UBXConfigKeys.MsgOutRtcm3x1230Uart2.begin(), UBXConfigKeys.MsgOutRtcm3x1230Uart2.end());
		payload.push_back(0x01);

		// Disable UART2 RTCM input
		payload.insert(payload.end(), UBXConfigKeys.Uart2InProtRTCM3.begin(), UBXConfigKeys.Uart2InProtRTCM3.end());
		payload.push_back(0x00); // 0x01 = enable; 0x00 = disable

		// Enable UART2 RTCM output
		payload.insert(payload.end(), UBXConfigKeys.Uart2OutProtRTCM3.begin(), UBXConfigKeys.Uart2OutProtRTCM3.end());
		payload.push_back(0x01); // 0x01 = enable; 0x00 = disable

		if (!sendUbxMsg(msgClass, &payload))
			return false;

		payload = payloadTemp;

		// Disable UART2 UBX output
		payload.insert(payload.end(), UBXConfigKeys.Uart2OutProtUBX.begin(), UBXConfigKeys.Uart2OutProtUBX.end());
		payload.push_back(0x00); // 0x01 = enable; 0x00 = disable

		// Disable UBX navigation output
		payload.insert(payload.end(), UBXConfigKeys.MsgOutUbxPvtUart2.begin(), UBXConfigKeys.MsgOutUbxPvtUart2.end());
		payload.push_back(0x00);
		payload.insert(payload.end(), UBXConfigKeys.MsgOutUbxRelPosNEDUart2.begin(), UBXConfigKeys.MsgOutUbxRelPosNEDUart2.end());
		payload.push_back(0x00);
		payload.insert(payload.end(), UBXConfigKeys.MsgOutUbxSatUart2.begin(), UBXConfigKeys.MsgOutUbxSatUart2.end());
		payload.push_back(0x00);
		payload.insert(payload.end(), UBXConfigKeys.MsgOutUbxSigUart2.begin(), UBXConfigKeys.MsgOutUbxSigUart2.end());
		payload.push_back(0x00);
		payload.insert(payload.end(), UBXConfigKeys.MsgOutUbxStatusUart2.begin(), UBXConfigKeys.MsgOutUbxStatusUart2.end());
		payload.push_back(0x00);

		// Send RTCM output configuration
		if (!sendUbxMsg(msgClass, &payload))
			return false;
	}

	return true;
}



bool UbxProtocol::setBaseStationFixed(int32_t ecef_x, int32_t ecef_y, int32_t ecef_z)
{
	vector<uint8_t> msgClass = {UBXMsgClass.CFG_VALSET[0], UBXMsgClass.CFG_VALSET[1]};

	// Iterate through all three layers
	for (uint8_t currentLayer = 0; currentLayer <= 2; ++currentLayer)
	{
		vector<uint8_t> payload;
		if (currentLayer == 0)
		{
			payload = {UBX_CFG_VALSET.LayerRAM[0], UBX_CFG_VALSET.LayerRAM[1], UBX_CFG_VALSET.Transactionless[0], UBX_CFG_VALSET.Transactionless[1]};
		}
		else if (currentLayer == 1)
		{
			payload = {UBX_CFG_VALSET.LayerBBR[0], UBX_CFG_VALSET.LayerBBR[1], UBX_CFG_VALSET.Transactionless[0], UBX_CFG_VALSET.Transactionless[1]};
		}
		else if (currentLayer == 2)
		{
			payload = {UBX_CFG_VALSET.LayerFlash[0], UBX_CFG_VALSET.LayerFlash[1], UBX_CFG_VALSET.Transactionless[0], UBX_CFG_VALSET.Transactionless[1]};
		}

		// Copy payload with the layer information into payloadTemp
		vector<uint8_t> payloadTemp = payload;

		// Mode selection (fixed)
		payload.insert(payload.end(), UBXConfigKeys.TmodeMode.begin(), UBXConfigKeys.TmodeMode.end());
		payload.push_back(0x02); // Fixed mode

		// Position type
		payload.insert(payload.end(), UBXConfigKeys.TmodePosType.begin(), UBXConfigKeys.TmodePosType.end());
		payload.push_back(0x00); // 0x00 = ECEF; 0x01 = LLH

		// Set ECEF values
		payload.insert(payload.end(), UBXConfigKeys.TmodeEcefX.begin(), UBXConfigKeys.TmodeEcefX.end());
		payload.push_back(ecef_x & 0xFF);
		payload.push_back((ecef_x >> 8) & 0xFF);
		payload.push_back((ecef_x >> 16) & 0xFF);
		payload.push_back((ecef_x >> 24) & 0xFF);

		payload.insert(payload.end(), UBXConfigKeys.TmodeEcefY.begin(), UBXConfigKeys.TmodeEcefY.end());
		payload.push_back(ecef_y & 0xFF);
		payload.push_back((ecef_y >> 8) & 0xFF);
		payload.push_back((ecef_y >> 16) & 0xFF);
		payload.push_back((ecef_y >> 24) & 0xFF);

		payload.insert(payload.end(), UBXConfigKeys.TmodeEcefZ.begin(), UBXConfigKeys.TmodeEcefZ.end());
		payload.push_back(ecef_z & 0xFF);
		payload.push_back((ecef_z >> 8) & 0xFF);
		payload.push_back((ecef_z >> 16) & 0xFF);
		payload.push_back((ecef_z >> 24) & 0xFF);

		// Send fixed-mode configuration
		if (!sendUbxMsg(msgClass, &payload))
			return false;

		// Reset payload for the next iteration
		payload = payloadTemp;

		// Enable RTCM3.X outputs
		payload.insert(payload.end(), UBXConfigKeys.MsgOutRtcm3x1005Uart2.begin(), UBXConfigKeys.MsgOutRtcm3x1005Uart2.end());
		payload.push_back(0x01);
		payload.insert(payload.end(), UBXConfigKeys.MsgOutRtcm3x1074Uart2.begin(), UBXConfigKeys.MsgOutRtcm3x1074Uart2.end());
		payload.push_back(0x01);
		payload.insert(payload.end(), UBXConfigKeys.MsgOutRtcm3x1084Uart2.begin(), UBXConfigKeys.MsgOutRtcm3x1084Uart2.end());
		payload.push_back(0x01);
		payload.insert(payload.end(), UBXConfigKeys.MsgOutRtcm3x1094Uart2.begin(), UBXConfigKeys.MsgOutRtcm3x1094Uart2.end());
		payload.push_back(0x01);
		payload.insert(payload.end(), UBXConfigKeys.MsgOutRtcm3x1124Uart2.begin(), UBXConfigKeys.MsgOutRtcm3x1124Uart2.end());
		payload.push_back(0x01);
		payload.insert(payload.end(), UBXConfigKeys.MsgOutRtcm3x1230Uart2.begin(), UBXConfigKeys.MsgOutRtcm3x1230Uart2.end());
		payload.push_back(0x01);

		// Disable UART2 RTCM input
		payload.insert(payload.end(), UBXConfigKeys.Uart2InProtRTCM3.begin(), UBXConfigKeys.Uart2InProtRTCM3.end());
		payload.push_back(0x00); // 0x01 = enable; 0x00 = disable

		// Enable UART2 RTCM output
		payload.insert(payload.end(), UBXConfigKeys.Uart2OutProtRTCM3.begin(), UBXConfigKeys.Uart2OutProtRTCM3.end());
		payload.push_back(0x01); // 0x01 = enable; 0x00 = disable

		if (!sendUbxMsg(msgClass, &payload))
			return false;

		// Reset payload for the next iteration
		payload = payloadTemp;

		// Disable UART2 UBX output
		payload.insert(payload.end(), UBXConfigKeys.Uart2OutProtUBX.begin(), UBXConfigKeys.Uart2OutProtUBX.end());
		payload.push_back(0x00); // 0x01 = enable; 0x00 = disable

		// Disable UBX navigation output
		payload.insert(payload.end(), UBXConfigKeys.MsgOutUbxPvtUart2.begin(), UBXConfigKeys.MsgOutUbxPvtUart2.end());
		payload.push_back(0x00);
		payload.insert(payload.end(), UBXConfigKeys.MsgOutUbxRelPosNEDUart2.begin(), UBXConfigKeys.MsgOutUbxRelPosNEDUart2.end());
		payload.push_back(0x00);
		payload.insert(payload.end(), UBXConfigKeys.MsgOutUbxSatUart2.begin(), UBXConfigKeys.MsgOutUbxSatUart2.end());
		payload.push_back(0x00);
		payload.insert(payload.end(), UBXConfigKeys.MsgOutUbxSigUart2.begin(), UBXConfigKeys.MsgOutUbxSigUart2.end());
		payload.push_back(0x00);
		payload.insert(payload.end(), UBXConfigKeys.MsgOutUbxStatusUart2.begin(), UBXConfigKeys.MsgOutUbxStatusUart2.end());
		payload.push_back(0x00);

		// Send RTCM output configuration
		if (!sendUbxMsg(msgClass, &payload))
			return false;
	}

	return true;
}


bool UbxProtocol::setRover()
{
	vector<uint8_t> msgClass = { UBXMsgClass.CFG_VALSET[0], UBXMsgClass.CFG_VALSET[1]};

	// Iterate through all three layers
	for (uint8_t currentLayer = 0; currentLayer <= 2; ++currentLayer)
	{
		vector<uint8_t> payload;
		vector<uint8_t> payloadTemp;
		if (currentLayer == 0)
		{
			payload = { UBX_CFG_VALSET.LayerRAM[0], UBX_CFG_VALSET.LayerRAM[1], UBX_CFG_VALSET.Transactionless[0], UBX_CFG_VALSET.Transactionless[1]};
		}
		else if (currentLayer == 1)
		{
			payload = { UBX_CFG_VALSET.LayerBBR[0], UBX_CFG_VALSET.LayerBBR[1], UBX_CFG_VALSET.Transactionless[0], UBX_CFG_VALSET.Transactionless[1]};
		}
		else if (currentLayer == 2)
		{
			payload = { UBX_CFG_VALSET.LayerFlash[0], UBX_CFG_VALSET.LayerFlash[1], UBX_CFG_VALSET.Transactionless[0], UBX_CFG_VALSET.Transactionless[1]};
		}
		else
		{
			return false;
		}

		// Copy payload with the layer information into payloadTemp
		payloadTemp = payload;

		// Mode selection (disabled)
		payload.insert(payload.end(), UBXConfigKeys.TmodeMode.begin(), UBXConfigKeys.TmodeMode.end());
		payload.push_back(0x00); // Disabled

		// Send configuration
		if (!sendUbxMsg(msgClass, &payload))
			return false;

		// Enable UART2 RTCM input
		payload.insert(payload.end(), UBXConfigKeys.Uart2InProtRTCM3.begin(), UBXConfigKeys.Uart2InProtRTCM3.end());
		payload.push_back(0x01); // 0x01 = enable; 0x00 = disable

		// Disable UART2 RTCM output
		payload.insert(payload.end(), UBXConfigKeys.Uart2OutProtRTCM3.begin(), UBXConfigKeys.Uart2OutProtRTCM3.end());
		payload.push_back(0x00); // 0x01 = enable; 0x00 = disable

		if (!sendUbxMsg(msgClass, &payload))
			return false;

		payload = payloadTemp;

		// Disable RTCM output
		payload.insert(payload.end(), UBXConfigKeys.MsgOutRtcm3x1005Uart2.begin(), UBXConfigKeys.MsgOutRtcm3x1005Uart2.end());
		payload.push_back(0x00); // 0x01 = enable; 0x00 = disable
		payload.insert(payload.end(), UBXConfigKeys.MsgOutRtcm3x1074Uart2.begin(), UBXConfigKeys.MsgOutRtcm3x1074Uart2.end());
		payload.push_back(0x00);
		payload.insert(payload.end(), UBXConfigKeys.MsgOutRtcm3x1084Uart2.begin(), UBXConfigKeys.MsgOutRtcm3x1084Uart2.end());
		payload.push_back(0x00);
		payload.insert(payload.end(), UBXConfigKeys.MsgOutRtcm3x1094Uart2.begin(), UBXConfigKeys.MsgOutRtcm3x1094Uart2.end());
		payload.push_back(0x00);
		payload.insert(payload.end(), UBXConfigKeys.MsgOutRtcm3x1124Uart2.begin(), UBXConfigKeys.MsgOutRtcm3x1124Uart2.end());
		payload.push_back(0x00);
		payload.insert(payload.end(), UBXConfigKeys.MsgOutRtcm3x1230Uart2.begin(), UBXConfigKeys.MsgOutRtcm3x1230Uart2.end());
		payload.push_back(0x00);

		if (!sendUbxMsg(msgClass, &payload))
			return false;

		payload = payloadTemp;

		// Enable UART2 UBX output
		payload.insert(payload.end(), UBXConfigKeys.Uart2OutProtUBX.begin(), UBXConfigKeys.Uart2OutProtUBX.end());
		payload.push_back(0x01); // 0x01 = enable; 0x00 = disable

		// Enable UBX navigation output
//		payload.insert(payload.end(), UBXConfigKeys.MsgOutUbxPvtUart2.begin(), UBXConfigKeys.MsgOutUbxPvtUart2.end());
//		payload.push_back(0x01);
		payload.insert(payload.end(), UBXConfigKeys.MsgOutUbxRelPosNEDUart2.begin(), UBXConfigKeys.MsgOutUbxRelPosNEDUart2.end());
		payload.push_back(0x01);
		payload.insert(payload.end(), UBXConfigKeys.MsgOutUbxSatUart2.begin(), UBXConfigKeys.MsgOutUbxSatUart2.end());
		payload.push_back(0x01);
//		payload.insert(payload.end(), UBXConfigKeys.MsgOutUbxSigUart2.begin(), UBXConfigKeys.MsgOutUbxSigUart2.end());
//		payload.push_back(0x01);
//		payload.insert(payload.end(), UBXConfigKeys.MsgOutUbxStatusUart2.begin(), UBXConfigKeys.MsgOutUbxStatusUart2.end());
//		payload.push_back(0x01);

		// Send configuration
		if (!sendUbxMsg(msgClass, &payload))
			return false;
	}
	return true;
}


bool UbxProtocol::setMeasRate(uint16_t measRate)
{
	vector<uint8_t> msgClass = { UBXMsgClass.CFG_VALSET[0], UBXMsgClass.CFG_VALSET[1]};

	// Iterate through all three layers
	for (uint8_t currentLayer = 0; currentLayer <= 2; ++currentLayer)
	{
		vector<uint8_t> payload;
		if (currentLayer == 0)
		{
			payload = { UBX_CFG_VALSET.LayerRAM[0], UBX_CFG_VALSET.LayerRAM[1], UBX_CFG_VALSET.Transactionless[0], UBX_CFG_VALSET.Transactionless[1]};
		}
		else if (currentLayer == 1)
		{
			payload = { UBX_CFG_VALSET.LayerBBR[0], UBX_CFG_VALSET.LayerBBR[1], UBX_CFG_VALSET.Transactionless[0], UBX_CFG_VALSET.Transactionless[1]};
		}
		else if (currentLayer == 2)
		{
			payload = { UBX_CFG_VALSET.LayerFlash[0], UBX_CFG_VALSET.LayerFlash[1], UBX_CFG_VALSET.Transactionless[0], UBX_CFG_VALSET.Transactionless[1]};
		}
		else
		{
			return false;
		}

		// Set measurement rate
		payload.insert(payload.end(), UBXConfigKeys.RateMeas.begin(), UBXConfigKeys.RateMeas.end());
		payload.push_back(measRate & 0xFF);
		payload.push_back(measRate >> 8);

		// Send configuration
		if (!sendUbxMsg(msgClass, &payload))
			return false;
	}
	return true;
}

bool UbxProtocol::setUART2BaudRate(uint32_t baudRate)
{
	vector<uint8_t> msgClass = { UBXMsgClass.CFG_VALSET[0], UBXMsgClass.CFG_VALSET[1]};

	// Iterate through all three layers
	for (uint8_t currentLayer = 0; currentLayer <= 2; ++currentLayer)
	{
		vector<uint8_t> payload;
		if (currentLayer == 0)
		{
			payload = { UBX_CFG_VALSET.LayerRAM[0], UBX_CFG_VALSET.LayerRAM[1], UBX_CFG_VALSET.Transactionless[0], UBX_CFG_VALSET.Transactionless[1]};
		}
		else if (currentLayer == 1)
		{
			payload = { UBX_CFG_VALSET.LayerBBR[0], UBX_CFG_VALSET.LayerBBR[1], UBX_CFG_VALSET.Transactionless[0], UBX_CFG_VALSET.Transactionless[1]};
		}
		else if (currentLayer == 2)
		{
			payload = { UBX_CFG_VALSET.LayerFlash[0], UBX_CFG_VALSET.LayerFlash[1], UBX_CFG_VALSET.Transactionless[0], UBX_CFG_VALSET.Transactionless[1]};
		}
		else
		{
			return false;
		}

		// Configure UART2 baud rate
		payload.insert(payload.end(), UBXConfigKeys.Uart2Baudrate.begin(), UBXConfigKeys.Uart2Baudrate.end());
		payload.push_back(baudRate & 0xFF);
		payload.push_back((baudRate >> 8) & 0xFF);
		payload.push_back((baudRate >> 16) & 0xFF);
		payload.push_back((baudRate >> 24) & 0xFF);

		// Send configuration
		if (!sendUbxMsg(msgClass, &payload))
			return false;
	}
	return true;
}



bool UbxProtocol::getSurveyInStatus() {
    vector<uint8_t> msgClass = {0x01, 0x3B};
    vector<uint8_t> payload = {};
    vector<uint8_t> receivedMsgClass;
    vector<uint8_t> receivedPayload;

    if (sendUbxMsg(msgClass, &payload)) {
        if (receiveUbxMessage(receivedMsgClass, &receivedPayload)) {
            if (receivedMsgClass == msgClass) {
                // Extract information
                SurveyInStatus svin_temp;
                svin_temp.version = receivedPayload[0];
                svin_temp.reserved0 = receivedPayload[1];
                svin_temp.timeOfWeekMs = static_cast<uint32_t>(receivedPayload[4] | (receivedPayload[5] << 8) | (receivedPayload[6] << 16) | (receivedPayload[7] << 24));
                svin_temp.dur = static_cast<uint32_t>(receivedPayload[8] | (receivedPayload[9] << 8) | (receivedPayload[10] << 16) | (receivedPayload[11] << 24));
                svin_temp.meanX = static_cast<int32_t>(receivedPayload[12] | (receivedPayload[13] << 8) | (receivedPayload[14] << 16) | (receivedPayload[15] << 24));
                svin_temp.meanY = static_cast<int32_t>(receivedPayload[16] | (receivedPayload[17] << 8) | (receivedPayload[18] << 16) | (receivedPayload[19] << 24));
                svin_temp.meanZ = static_cast<int32_t>(receivedPayload[20] | (receivedPayload[21] << 8) | (receivedPayload[22] << 16) | (receivedPayload[23] << 24));
                svin_temp.meanXHP = static_cast<int8_t>(receivedPayload[24]);
                svin_temp.meanYHP = static_cast<int8_t>(receivedPayload[25]);
                svin_temp.meanZHP = static_cast<int8_t>(receivedPayload[26]);
                svin_temp.reserved1 = receivedPayload[27];
                svin_temp.meanAcc = static_cast<uint32_t>(receivedPayload[28] | (receivedPayload[29] << 8) | (receivedPayload[30] << 16) | (receivedPayload[31] << 24));
                svin_temp.obsUsed = static_cast<uint32_t>(receivedPayload[32] | (receivedPayload[33] << 8) | (receivedPayload[34] << 16) | (receivedPayload[35] << 24));
                svin_temp.valid = receivedPayload[36];
                svin_temp.active = receivedPayload[37];
                svin_temp.reserved2 = receivedPayload[38];

                setSurveyInStatus(&svin_temp);

                return true;
            }
        }
    }
    return false;
}


bool UbxProtocol::getNavPvt() {
    vector<uint8_t> msgClass = {0x01, 0x07};
    vector<uint8_t> payload = {};
    vector<uint8_t> receivedMsgClass;
    vector<uint8_t> receivedPayload;

    if (sendUbxMsg(msgClass, &payload)) {
        if (receiveUbxMessage(receivedMsgClass, &receivedPayload)) {
            if (receivedMsgClass == msgClass) {
                // Extract information
            	NavPvt navPvtInfo_temp;
            	navPvtInfo_temp.fixType = receivedPayload[20];
            	navPvtInfo_temp.numSatUsed = receivedPayload[23];
            	navPvtInfo_temp.hMSL = static_cast<int32_t>(receivedPayload[36] | (receivedPayload[37] << 8) | (receivedPayload[38] << 16) | (receivedPayload[39] << 24));
            	navPvtInfo_temp.hAcc = static_cast<uint32_t>(receivedPayload[40] | (receivedPayload[41] << 8) | (receivedPayload[42] << 16) | (receivedPayload[43] << 24));
            	navPvtInfo_temp.vAcc = static_cast<uint32_t>(receivedPayload[44] | (receivedPayload[45] << 8) | (receivedPayload[46] << 16) | (receivedPayload[47] << 24));
                setNavPvt(&navPvtInfo_temp);
                return true;
            }
        }
    }
    return false;
}

bool UbxProtocol::getTmodeMode() {
    vector<uint8_t> msgClass = {0x06, 0x8b}; // UBX-CFG-VALGET
    vector<uint8_t> payload = {0x00, 0x00, 0x00, 0x00};
    payload.insert(payload.end(), UBXConfigKeys.TmodeMode.begin(), UBXConfigKeys.TmodeMode.end());
    payload.insert(payload.end(), UBXConfigKeys.TmodeSvinMinDur.begin(), UBXConfigKeys.TmodeSvinMinDur.end());
    payload.insert(payload.end(), UBXConfigKeys.TmodeSvinAccLimit.begin(), UBXConfigKeys.TmodeSvinAccLimit.end());
    payload.insert(payload.end(), UBXConfigKeys.TmodeEcefX.begin(), UBXConfigKeys.TmodeEcefX.end());
    payload.insert(payload.end(), UBXConfigKeys.TmodeEcefY.begin(), UBXConfigKeys.TmodeEcefY.end());
    payload.insert(payload.end(), UBXConfigKeys.TmodeEcefZ.begin(), UBXConfigKeys.TmodeEcefZ.end());
    vector<uint8_t> receivedMsgClass;
    vector<uint8_t> receivedPayload;

    if (sendUbxMsg(msgClass, &payload)) {
        if (receiveUbxMessage(receivedMsgClass, &receivedPayload)) {
            if (receivedMsgClass == msgClass) {
                // Extract TMODE values
                TmodeConfig tmode_temp;
                tmode_temp.tmodeMode = receivedPayload[8];
                tmode_temp.svinMinDur = static_cast<uint32_t>(receivedPayload[13] | (receivedPayload[14] << 8) | (receivedPayload[15] << 16) | (receivedPayload[16] << 24));
                tmode_temp.svinVarLimit = static_cast<uint32_t>(receivedPayload[21] | (receivedPayload[22] << 8) | (receivedPayload[23] << 16) | (receivedPayload[24] << 24));
                tmode_temp.ecefX = static_cast<int32_t>(receivedPayload[29] | (receivedPayload[30] << 8) | (receivedPayload[31] << 16) | (receivedPayload[32] << 24));
                tmode_temp.ecefY = static_cast<int32_t>(receivedPayload[37] | (receivedPayload[38] << 8) | (receivedPayload[39] << 16) | (receivedPayload[40] << 24));
                tmode_temp.ecefZ = static_cast<int32_t>(receivedPayload[45] | (receivedPayload[46] << 8) | (receivedPayload[47] << 16) | (receivedPayload[48] << 24));
                setTmodeConfig(&tmode_temp);
                // Now check for ACK/NAK
                bool ack;
                if (ackNakReceived(msgClass, ack)) {
                    return ack;
                }
                else return false;
            }
        }
    }
    return false;
}

