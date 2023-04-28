#pragma once
#include "../../Mavlink_v2/common/mavlink.h"
#include "../../Mavlink_v2/mavlink_types.h"

#include "altimeter.h"

#include "main.h"
#include <stdio.h>
#include <bitset>
#include <cstdint>
#include <functional>


class MavlinkControl{

	// This is a class that implements some basic functionalities from the Mavlink V2 library.
	// Enter small breakdown of how the mavlink protocol works here.

	private:


		UART_HandleTypeDef* _huart_mavlink;
		I2C_HandleTypeDef* _altimeter_i2c;
		Altimeter* _altimeter;

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
		mavlink_message_t _mavlinkSend;

		//Mavlink status type messages
		mavlink_sys_status_t _system_status;
		mavlink_status_t _status;
		mavlink_system_t _mavlink_system = {

				255,		//system id
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
		//Mavlink battery status message is used for sending the percentage of fluid left. 
		// "-1" means the autopilot doesn't take this into account
		mavlink_battery_status_t _mavlink_battery = { 
			-1, 	
			-1,	
			INT16_MAX,													//battery temp (INT16_MAX for unknown)
			{ 10000, INT16_MAX, INT16_MAX, INT16_MAX, INT16_MAX,
			INT16_MAX, INT16_MAX, INT16_MAX, INT16_MAX, INT16_MAX },	//Battery voltage of cells 1 to 10
			-1,															//Battery current
			MAV_COMP_ID_BATTERY, MAV_BATTERY_FUNCTION_ALL, MAV_BATTERY_TYPE_LIPO,
			69,															//remaining battery energy. This is the value you need, test and see if all the rest is actually necessary 
			0,															//remaining battery time. 0 is N/A
			MAV_BATTERY_CHARGE_STATE_OK,
			{ 0 },														//Battery voltages for cells 11 to 14. 
			MAV_BATTERY_MODE_AUTO_DISCHARGING, 							//Battery Mode
			0									
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
		MavlinkControl(UART_HandleTypeDef* huart, Altimeter* altimeter);

		mavlink_header_t getMavlinkHeader(void);
		uint8_t returnTestFunction(void);

		void heartbeat(void);
		void sendTestMessage(void);
		void sendAltitude(void);

		void readFlightTime(mavlink_message_t receivedMessage);
		void decodeHeartbeat(mavlink_message_t receivedMessage);
		bool checkConnection(void);

		uint32_t getFlightTime(void);

		void process_header(void);

		void update(void);
		void update_TX(void);
		void update_RX(void);

		void Error_Handler(void);


};
