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
		I2C_HandleTypeDef* _altimeter_i2c;

		//pointer to the class instance itself. Used in callback functions.
        static MavlinkControl* instancePtr;

		static const uint16_t MAVLINK_BUFFER_SIZE = 280;

		//__IO prefix is a type qualifier that specifies that the variable is accessed by both the CPU
		//and external devices (such as DMA)
		__IO uint32_t _receivedChars = 0;

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

				255,		//system id
				1		//comonent id (MAV_COMPONENT value)

		};

		mavlink_heartbeat_t _mavlink_heartbeat = {
				0, 		//custom mode = none
				MAV_TYPE_GROUND_ROVER,
				MAV_AUTOPILOT_GENERIC,
				MAV_MODE_FLAG_SAFETY_ARMED,
				MAV_STATE_STANDBY
		};

        mavlink_flight_information_t flight_info;



	public:
		//buffers for receiving data
		uint8_t _receiveBuffer_1[MAVLINK_BUFFER_SIZE] = {0};
	    uint8_t* _tempBuffer;
		uint8_t* _receiveBuffer_2 = {0};
		uint8_t _bufferIndex = 0;

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


		MavlinkControl();
		MavlinkControl(UART_HandleTypeDef* huart);
		MavlinkControl(UART_HandleTypeDef* huart, I2C_HandleTypeDef* i2c);


		mavlink_header_t getMavlinkHeader(void);
		uint8_t returnTestFunction(void);

		void heartbeat(void);
		void sendTestMessage(void);
		void sendAltitude(uint16_t altitude);

		void readFlightTime(void);
		uint32_t getFlightTime(void);

		void process_header(void);

		void update(void);
		void update_TX(void);
		void update_RX(void);
};
