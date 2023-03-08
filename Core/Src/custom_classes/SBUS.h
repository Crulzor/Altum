#pragma once
#include "main.h"
#include "../../Mavlink_v2/common/mavlink.h"



class SBUS{




	private:
		//pointer to handler in main
		UART_HandleTypeDef *_huart_sbus;

		//some hardcoded stuff for sbus protocol
		static const uint8_t SBUS_PACKET_SIZE = 25;
		static const uint8_t SBUS_START_BYTE = 0x0f;
		static const uint8_t SBUS_END_BYTE = 0x04;
		static const uint8_t SBUS_NUM_CHANNELS = 16;

		static const uint8_t _debounceTime = 50;


	public:
		SBUS(UART_HandleTypeDef *huart_sbus);
		void update(void);
		bool readSBUS(void);
		void otherReadJoysticks(void);
		float getLeftY(void);
		float getLeftX(void);
		float getRightY(void);
		float getRightX(void);
		float getJogWheel(void);
		bool A_button(void);
		bool B_button(void);
		bool C_button(void);
		bool D_button(void);
		bool home_button(void);


		//Keeping these public for debugging purposes for now.
		uint8_t _sbus_buffer[SBUS_PACKET_SIZE];
		uint16_t _channels[SBUS_NUM_CHANNELS] = {0};



};
