#pragma once

#include "main.h"
#include <math.h>
#include <stdio.h>
#include <bitset>
#include <stdint.h>

class Altimeter{


	private:

		I2C_HandleTypeDef* _i2c;

		//MPL3115A2 register definitions
		static const int16_t _altimeter_address = 0xC0;
		static const int16_t _altimeter_status_reg = 0x00;
		static const int16_t _altimeter_ctrl_reg_1 = 0x26;
		static const int16_t _altimeter_ctrl_reg_2 = 0x27;
		static const int16_t _altimeter_pressure_MSB = 0x01;
		static const int16_t _altimeter_pressure_CSB = 0x01;
		static const int16_t _altimeter_pressure_LSB = 0x03;
		static const int16_t _altimeter_temperature_MSB = 0x04;
		static const int16_t _altimeter_temperature_LSB = 0x05;
		static const int16_t _altimeter_data_event_flag_reg = 0x13;

		uint8_t _altimeter_mode;	//0x38 for altimeter mode with 128bit oversampling
		uint8_t _event_flags = 0x00;	//Keep this default 0 for now.

		uint8_t status_reg_data;
		uint8_t _pressure_data[3];

		float _altitude = 0;

	public:

		//Constructor
		Altimeter(I2C_HandleTypeDef* i2c);

		//Initialize altimeter
		void init_altimeter(void);
		void test_communication(void);
		float read_altitude(void);
		float get_altitude(void);
		void update(void);





};
