#pragma once
#include "../../Mavlink_v2/common/mavlink.h"
#include "../../Mavlink_v2/mavlink_types.h"

#include "altimeter.h"
#include "Convertor.h"

#include "main.h"
#include <stdio.h>
#include <bitset>
#include <cstdint>
#include <functional>


class MavlinkControl{

	// This is a class that implements some basic functionalities from the Mavlink V2 library.
	// In the documentation you can find more info about the Mavlink protocol. Currently I use DMA
	// To send/Receive messages. 

	private:


		UART_HandleTypeDef* _huart_mavlink;
		Altimeter* _altimeter;
		Convertor* _convertor;

		//pointer to the class instance itself. Used in callback functions.
        static MavlinkControl* instancePtr;

		static const uint16_t MAVLINK_BUFFER_SIZE = 280;

		uint8_t _bufferSelector = 0;
		uint16_t _bufferLength = 0;
		
		//-----MAVLINK MESSAGES------ 
		//separate buffers for receiving/sending general mavlink messages, these are decoded further into 
		//more "specialized messages" like the ones below.
		//A complete list can be found in the docs @ mavlink.io

		//seperate "general" messages for sending and receiving. 
		mavlink_message_t _mavlinkReceived;
		mavlink_message_t _mavlinkSend_1;
		mavlink_message_t _mavlinkSend_2;

		//Mavlink status type messages
		mavlink_sys_status_t _system_status;
		mavlink_status_t _status;
		mavlink_system_t _mavlink_system = {

				1,		//system id
				1		//comonent id (MAV_COMPONENT value)

		};

		//Mavlink heartbeat message. This is sent periodically to make connection with the Herelink Controller
		mavlink_heartbeat_t _mavlink_heartbeat = {
				0, 		//custom mode = none
				MAV_TYPE_GROUND_ROVER,
				MAV_AUTOPILOT_GENERIC,
				MAV_MODE_FLAG_SAFETY_ARMED,
				MAV_STATE_STANDBY
		};
	
		//flight information message used for debugging, never receives this type of message though. 
        mavlink_flight_information_t flight_info;

	public:
		//buffers for receiving data
		uint8_t _receiveBuffer_1[MAVLINK_BUFFER_SIZE] = {0};
	    uint8_t* _tempBuffer;
		uint8_t* _receiveBuffer_2 = {0};
		uint32_t _bufferIndex = 0;

		//buffers for sending data
		uint8_t _bufferPackedforUart[MAVLINK_BUFFER_SIZE] = {0};
		uint8_t _bufferPackedforUart_2[MAVLINK_BUFFER_SIZE] = {0};

		uint16_t _TX_bufferLength;

		//Var for flight time. Used for testing purposes right now
		double _flight_time = 0;

		//function for callback implementation.
		void uartRxCallback(UART_HandleTypeDef *huart, uint16_t Size);
        friend void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size);


	    typedef struct {
	        uint8_t magic = 0;
	        uint8_t len = 0;
	        uint8_t incompat_flags = 0;
	        uint8_t compat_flags = 0;
	        uint8_t seq = 0;
	        uint8_t sysid = 0;
	        uint8_t compid = 0;
	        uint32_t msgid = 0;
	    } mavlink_header_t;

	    mavlink_header_t _mavlink_received_header;

		mavlink_heartbeat_t _received_heartbeat;

		MavlinkControl();
		MavlinkControl(UART_HandleTypeDef* huart);
		MavlinkControl(UART_HandleTypeDef* huart, Altimeter* altimeter, Convertor* convertor);

		mavlink_header_t getMavlinkHeader(void);
		uint8_t returnTestFunction(void);

		//Functions for sending data over Mavlink. See documentation for detailed explanation. 
		//The heartbeat function should always be sent periodically to mantain the connection. 
		void heartbeat(void);
		void sendTestMessage(void);
		void sendAltitude(void);
		void sendBattery(void);
		void sendFluids(void);

		//Functions for receving mavlink messages. See documentation for detailed explanation. 
		//Currently not much is being read except the header of the mavlink mssges in the debugger class. 
		void readFlightTime(mavlink_message_t receivedMessage);
		void decodeHeartbeat(mavlink_message_t receivedMessage);

		uint32_t getFlightTime(void);

		//Function that uses DMA to read the Mavlink Messages. 
		void update_RX(void);

		void Error_Handler(void);


};
