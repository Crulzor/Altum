#include "altimeter.h"

Altimeter::Altimeter(I2C_HandleTypeDef* i2c) :_i2c (i2c){



}

void Altimeter::init_altimeter(void){

	//Set to altimeter mode
	_altimeter_mode = 0xf9;
	uint8_t reset = 0x04;
	uint8_t response;
    uint8_t transmit_data[3];

	if(HAL_I2C_Mem_Write(_i2c, _altimeter_address, _altimeter_ctrl_reg_1, 1, &_altimeter_mode, 1, 100) != HAL_OK){

		printf("error with i2c \r\n");

	}

	HAL_I2C_Mem_Read(_i2c, _altimeter_address, _altimeter_ctrl_reg_1, I2C_MEMADD_SIZE_8BIT , &response, 1, 100);
	printf("Reading from IIC address %d \r\n", response);

	//No event flags disabled (might change later).
	if(HAL_I2C_Mem_Write(_i2c, _altimeter_address, _altimeter_ctrl_reg_2, 1, &_event_flags, 1, 100) != HAL_OK){

		printf("error with i2c \r\n");
	}
	uint8_t data[1] = {0x07};
	if(HAL_I2C_Mem_Write(_i2c, _altimeter_address, 0x14, 1, data, 1, 100) != HAL_OK){
		printf("error with i2c \r\n");
	}
	_altimeter_mode = 0xf9;
	transmit_data[0] = _altimeter_address;
	transmit_data[1] = _altimeter_ctrl_reg_1;
	transmit_data[2] = _altimeter_mode;
	if(HAL_I2C_Master_Transmit(_i2c, _altimeter_address, transmit_data, 2, 100) != HAL_OK){

		printf("error with i2c \r\n");

	}
	transmit_data[0] = _altimeter_address;
	transmit_data[1] = _altimeter_ctrl_reg_1;
	transmit_data[2] = 0xFB;
	HAL_I2C_Master_Transmit(_i2c, _altimeter_address, transmit_data, 2, 100);
	while(HAL_I2C_GetState(_i2c) != HAL_I2C_STATE_READY);


	// Enable Data Flags in PT_DATA_CFG
	transmit_data[0] = _altimeter_data_event_flag_reg;
	transmit_data[1] = 0x07;
	HAL_I2C_Master_Transmit(_i2c, _altimeter_address, transmit_data, 2, 100);
	while(HAL_I2C_GetState(_i2c) != HAL_I2C_STATE_READY);

	// Set Active
	transmit_data[0] = _altimeter_address;
	transmit_data[1] = _altimeter_ctrl_reg_1;
	transmit_data[2] = 0xB9;
	HAL_I2C_Master_Transmit(_i2c, _altimeter_address, transmit_data, 2, 100);
	while(HAL_I2C_GetState(_i2c) != HAL_I2C_STATE_READY);


}




float Altimeter::read_altitude(void){

	 //TRIED TO FOLLOW FLOW-CHART FROM DATASHEET MPL3115A2
	    uint8_t status_reg;
	    uint8_t control_reg;

		// Read STATUS Register
		HAL_I2C_Mem_Read(_i2c, _altimeter_address, 0x00, 1, &status_reg, 1, 100);

		// Is Data Ready
		while(!(status_reg & 0x08)){
			HAL_I2C_Mem_Read(_i2c, _altimeter_address, 0x00, 1, &status_reg, 1, 100);
		}

		// Read OUT_P and OUT_T
		// Read pressure data from altimeter
		HAL_I2C_Mem_Read(_i2c, _altimeter_address, 0x01, 1, &_pressure_data[0], 1, 100);
		HAL_I2C_Mem_Read(_i2c, _altimeter_address, 0x02, 1, &_pressure_data[1], 1, 100);
		HAL_I2C_Mem_Read(_i2c, _altimeter_address, 0x03, 1, &_pressure_data[2], 1, 100);


		HAL_I2C_Mem_Read(_i2c, _altimeter_address, _altimeter_ctrl_reg_1, I2C_MEMADD_SIZE_8BIT , &control_reg, 1, 100);
		printf("Reading from IIC address %d \r\n", control_reg);
		printf("Reading from status register %d \r\n", status_reg);
	    printf("\r\n");

		uint8_t* rawBytes = _pressure_data;

	    std::string msb_byte = std::bitset<8>(rawBytes[0]).to_string();
	    std::string csb_byte = std::bitset<8>(rawBytes[1]).to_string();
	    std::string lsb_byte = std::bitset<8>(rawBytes[2]).to_string();

	    uint32_t shifted_msb_byte = uint32_t(rawBytes[0]) << 16;
	    uint32_t shifted_csb_byte = uint32_t(rawBytes[1]) << 8;
	    uint32_t shifted_lsb_byte = uint32_t(rawBytes[2]);

	    std::string shifted_msb_byte_str = std::bitset<32>(shifted_msb_byte).to_string();
	    std::string shifted_csb_byte_str = std::bitset<32>(shifted_csb_byte).to_string();
	    std::string shifted_lsb_byte_str = std::bitset<32>(shifted_lsb_byte).to_string();

		uint32_t pressure_raw = (shifted_msb_byte | shifted_csb_byte | shifted_lsb_byte) >> 4;

	    std::string pressure_raw_str = std::bitset<32>(pressure_raw).to_string();



	    printf("MSB byte: %s \r\n", msb_byte.c_str());
	    printf("shifted MSB byte %s \r\n", shifted_msb_byte_str.c_str());
	    printf("CSB byte: %s \r\n", csb_byte.c_str());
	    printf("shifted SB byte %s \r\n", shifted_csb_byte_str.c_str());
	    printf("LSB byte: %s \r\n", lsb_byte.c_str());
	    printf("shifted LSB byte %s \r\n", shifted_lsb_byte_str.c_str());
	    printf("\r\n");
	    printf("raw_pressure_bits %s \r\n", pressure_raw_str.c_str());
	    printf("\r\n");

	    return pressure_raw/ 64.0;

}

float Altimeter::get_altitude(void){

	return _altitude;
}

void Altimeter::update(void){




}
