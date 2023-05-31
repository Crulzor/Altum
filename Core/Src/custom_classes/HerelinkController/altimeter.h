#pragma once

#include "main.h"
#include <math.h>
#include <stdio.h>
#include <bitset>
#include <stdint.h>

class Altimeter{

	private:
		I2C_HandleTypeDef*  _i2c;


		//MPL3115A2 register definitions
		static const int16_t _altimeter_address = 0xC0;
		static const int16_t _altimeter_read_address = 0xC1;
		static const int16_t _altimeter_whoAmI = 0x0C;
		static const int16_t _altimeter_status_reg = 0x00;
		static const int16_t _altimeter_ctrl_reg_1 = 0x26;
		static const int16_t _altimeter_ctrl_reg_2 = 0x28;
		static const int16_t _altimeter_fifo_status_reg = 0x0D;
		static const int16_t _altimeter_pressure_MSB = 0x01;
		static const int16_t _altimeter_pressure_CSB = 0x01;
		static const int16_t _altimeter_pressure_LSB = 0x03;
		static const int16_t _altimeter_delta_pressure_MSB = 0x07;
		static const int16_t _altimeter_delta_pressure_CSB = 0x08;
		static const int16_t _altimeter_delta_pressure_LSB = 0x09;
		static const int16_t _altimeter_temperature_MSB = 0x04;
		static const int16_t _altimeter_temperature_LSB = 0x05;
		static const int16_t _altimeter_data_event_flag_reg = 0x13;

		uint8_t _pressure_registers[3];

		float _altitude = 0;
		float _altitude_offset = 0;
		float _delta_altitude = 0;
		float _moving_avg = 0;
		float _tempArray[5] = {0};

		//made functions to read and set each register, might be a bit bloated
		uint8_t read_ctrl_reg_1(void);
		uint8_t read_status_reg(void);
		uint8_t read_data_reg(void);
		void toggle_one_shot(void);
		void set_mode_altimeter(void);
		void set_mode_active(void);

		//function to write to specific register
		bool write_to_register(uint8_t adress, uint8_t value);


	public:
		//Constructor
		Altimeter();
		Altimeter(I2C_HandleTypeDef* i2c);

		//Initialize altimeter
		void init_altimeter(void);
		float test_communication(void);
		float read_altitude(void);
		float debug_altitude(void);
		float get_altitude(void);
		float get_offset(void);
		bool whoAmI(void);
		void set_offset(float offset);

		float process_altitude(float altitude);

		void Error_Handler(void);

		//for debugging purposes
		uint8_t _status_reg_data = 0;
		uint8_t _ctrl_reg1_data = 0;
		uint8_t _ctrl_reg2_data = 0;
		uint8_t _flag_reg_data = 0;
		uint8_t _fifo_status_reg = 0;
		uint8_t _whoAmI = 0;



};
