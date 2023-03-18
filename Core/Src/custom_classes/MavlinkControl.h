#pragma once
#include "../../Mavlink_v2/common/mavlink.h"
#include "../../Mavlink_v2/mavlink_types.h"

#include "main.h"
#include <bitset>
#include <cstdint>
#include <functional>


class MavlinkControl{

	// This is a class that implements some basic functionalities from the Mavlink V2 library.
	// Enter small breakdown of how the mavlink protocol works here.


	private:

		UART_HandleTypeDef* _huart_mavlink;
	    MavlinkControl* _instance;

	    void onReceiveEvent(UART_HandleTypeDef *huart);




		static const uint16_t MAVLINK_BUFFER_SIZE = 280;

		//__IO prefix is a type qualifier that specifies that the variable is accessed by both the CPU
		//and external devices (such as DMA)
		__IO uint32_t receivedChars = 0;

		uint8_t _bufferSelector = 0;
		uint16_t _bufferLength = 0;
		//separate buffers for receiving/sending mavlink messages, here's where the info comes after encoding/decoding
		mavlink_message_t _mavlinkReceived;
		mavlink_message_t _mavlinkSend;

		//to send multiple messages in a stream, use a switch case perhaps?
		//This is the "nextBufferSendToMavlinkBus" variable in Gio's code

		mavlink_sys_status_t _system_status;
		mavlink_status_t _status;
		mavlink_system_t _mavlink_system = {

				1,		//system id
				1		//comonent id (MAV_COMPONENT value)

		};

		mavlink_heartbeat_t _mavlink_heartbeat = {
				0, 		//custom mode = none
				MAV_TYPE_GROUND_ROVER,
				MAV_AUTOPILOT_GENERIC,
				MAV_MODE_FLAG_SAFETY_ARMED,
				MAV_STATE_STANDBY
		};


	public:
		//buffers for receiving data
		uint8_t _receiveBuffer_1[MAVLINK_BUFFER_SIZE] = {0};
	    uint8_t _receiveBuffer_2[MAVLINK_BUFFER_SIZE]  = {0};
		uint8_t _bufferPackedForUart[MAVLINK_BUFFER_SIZE] = {0};

	    typedef struct {
	        uint8_t start_of_msg = 0;
	        uint8_t payload_len = 0;
	        uint8_t sequence_num = 0;
	        uint8_t sys_id = 0;
	        uint8_t comp_id = 0;
	        uint8_t msg_id = 0;
	    } mavlink_header_t;

	    mavlink_header_t _mavlink_received_header;


		MavlinkControl();
		MavlinkControl(UART_HandleTypeDef* huart);

		//Getterzzzz
		mavlink_header_t getMavlinkHeader(void);



		void heartbeat(void);


		void parseMavlinkDataRX(void);
		void prepareMavlinkPacket(void);

		void processReceivedBuffer(void);

		void update(void);
};
