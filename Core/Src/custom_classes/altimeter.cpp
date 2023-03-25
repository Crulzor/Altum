#include "altimeter.h"

Altimeter::Altimeter(I2C_HandleTypeDef* i2c) :_i2c (i2c){



}

void Altimeter::init_altimeter(void){

	//Set to altimeter mode
	_altimeter_mode = 0x38;
	if(HAL_I2C_Mem_Write(_i2c, _altimeter_address, _altimeter_ctrl_reg_1, 1, &_altimeter_mode, 1, 100) != HAL_OK){

		printf("error with i2c \r\n");

	}
	//No event flags disabled (might change later).
	if(HAL_I2C_Mem_Write(_i2c, _altimeter_address, _altimeter_ctrl_reg_2, 1, &_event_flags, 1, 100) != HAL_OK){

		printf("error with i2c \r\n");
	}
	uint8_t data[1] = {0x07};
	if(HAL_I2C_Mem_Write(_i2c, _altimeter_address, 0x13, 1, data, 1, 100) != HAL_OK){
		printf("error with i2c \r\n");
	}

}



float Altimeter::get_altitude(void){

	//TRY TO FOLLOW FLOW CHART FROM DATASHEET
	//IIC_RegWrite(SlaveAddressIIC, 0x26, 0xB8); SET TO ALTIMETER WITH OVERSAMPLING 128
	  uint8_t transmit_data[2];
	    uint8_t status_reg, out_p_msb, out_p_csb, out_p_lsb, out_t_msb, out_t_lsb;

	    // Set to Altimeter with an OSR = 128
	    transmit_data[0] = _altimeter_ctrl_reg_1;
	    transmit_data[1] = 0xB8;
	    HAL_I2C_Master_Transmit(_i2c, _altimeter_address, transmit_data, 2, 100);
	    while(HAL_I2C_GetState(_i2c) != HAL_I2C_STATE_READY);

	    // Enable Data Flags in PT_DATA_CFG
	    transmit_data[0] = _altimeter_data_event_flag_reg;
	    transmit_data[1] = 0x07;
	    HAL_I2C_Master_Transmit(_i2c, _altimeter_address, transmit_data, 2, 100);
	    while(HAL_I2C_GetState(_i2c) != HAL_I2C_STATE_READY);

	    // Set Active
	    transmit_data[0] = 0x26;
	    transmit_data[1] = 0xB9;
	    HAL_I2C_Master_Transmit(_i2c, _altimeter_address, transmit_data, 2, 100);
	    while(HAL_I2C_GetState(_i2c) != HAL_I2C_STATE_READY);

	    // Read STATUS Register
	    HAL_I2C_Mem_Read(_i2c, _altimeter_address, 0x00, 1, &status_reg, 1, 100);

	    // Is Data Ready
	    while(!(status_reg & 0x08)){
	        HAL_I2C_Mem_Read(_i2c, _altimeter_address, 0x00, 1, &status_reg, 1, 100);
	    }

	    // Read OUT_P and OUT_T
	    HAL_I2C_Mem_Read(_i2c, _altimeter_address, 0x01, 1, &_pressure_data[0], 1, 100);
	    HAL_I2C_Mem_Read(_i2c, _altimeter_address, 0x02, 1, &_pressure_data[1], 1, 100);
	    HAL_I2C_Mem_Read(_i2c, _altimeter_address, 0x03, 1, &_pressure_data[2], 1, 100);


	    // Combine the three bytes into a 20-bit value
	    uint32_t pressure_raw = ((uint32_t)_pressure_data[0] << 16) | ((uint32_t)_pressure_data[1] << 8) | _pressure_data[2];
	    // Convert the two's complement value to a signed integer
	    if (pressure_raw & 0x80000) {
	        pressure_raw |= 0xFFF00000;
	    }

	    // Convert the raw pressure value to a human-readable value
	    this->_altitude = (float)pressure_raw / 4.0;
	    return this->_altitude;

}
