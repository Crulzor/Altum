#include "altimeter.h"

Altimeter::Altimeter(I2C_HandleTypeDef* i2c) :_i2c (i2c){



}

void Altimeter::init_altimeter(void){

	//Set to altimeter mode
	_altimeter_mode = 0x38;
	if(HAL_I2C_Mem_Write(_i2c, _altimeter_adress, _altimeter_ctrl_reg_1, 1, &_altimeter_mode, 1, 100) != HAL_OK){

		//insert some error handling here.

	}
	//No event flags disabled (might change later).
	if(HAL_I2C_Mem_Write(_i2c, _altimeter_adress, _altimeter_ctrl_reg_2, 1, &_event_flags, 1, 100) != HAL_OK){

		//insert some error handling here.
	}

}


float Altimeter::get_altitude(void){

	//array contains the address of the altitude register we want to read from (0x26)
	//and the value 0x01 to put the MPL3115A2 into active mode.
	uint8_t transmit_data[2] = {_altimeter_adress, 0x01};

	if(HAL_I2C_Master_Transmit(_i2c, _altimeter_adress, transmit_data, 2, 100)!= HAL_OK){

		//insert some error handling here.

	}
	//Next receive the altitude data (3 bytes)
	if (HAL_I2C_Master_Receive(_i2c, _altimeter_adress, _pressure_data, 3, 100) != HAL_OK){

		//insert some error handling here.

	}
    this->_altitude = ((uint32_t)_pressure_data[0] << 24) | ((uint32_t)_pressure_data[1] << 16) | ((uint32_t)_pressure_data[2] << 8);

	return this->_altitude;

}
