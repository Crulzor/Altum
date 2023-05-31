#include "altimeter.h"

Altimeter::Altimeter(I2C_HandleTypeDef* i2c){

	_i2c = i2c;
}


void Altimeter::init_altimeter(void){

	uint8_t response;
	uint8_t transmit_data;

	//First check connection and read who_am_I register
	do{

		this->whoAmI();
		printf("Attempting to read from chip \r\n");

	}while(_whoAmI != 0xC4);

	//Next do a reset for good measure. Reset bit is bit 3 in ctrl register 1.
	//and wait a bit for device to come back online.
	transmit_data = 0x04;
	HAL_I2C_Mem_Write(_i2c, _altimeter_address, _altimeter_ctrl_reg_1, I2C_MEMADD_SIZE_8BIT, &transmit_data, 1, 100);

	int counter = 0;
	do{
		counter++;

	}while(HAL_I2C_Mem_Read(_i2c, _altimeter_address, _altimeter_ctrl_reg_1, I2C_MEMADD_SIZE_8BIT, &response, 1, 100) != HAL_OK);

	printf("ctrl_reg1 after reset %d \r\n", response);
	printf("delay after reset: %d \r\n", counter);

	//Set mode to standby (bit 2 of ctrl reg 1)
	transmit_data = 0x02;
	while(this->write_to_register(_altimeter_ctrl_reg_1, transmit_data) != HAL_OK){

		printf("could not write to ctrl register 1 \r\n");
		_ctrl_reg1_data = response;

	}
	//Set pressure & temp event flegs
	transmit_data = 0x07;
	while(this->write_to_register(_altimeter_data_event_flag_reg, transmit_data) != HAL_OK){

		printf("Cannot write to flag register\r\n");

	}

	//Set oversampling mode to x128 and keep the chip in standby mode
	transmit_data = 0x3A;
	while(this->write_to_register(_altimeter_ctrl_reg_1, transmit_data) != HAL_OK){

		printf("Cannot write to flag register\r\n");

	}



}

bool Altimeter::whoAmI(void){

	HAL_I2C_Mem_Read(_i2c, _altimeter_address, _altimeter_whoAmI, I2C_MEMADD_SIZE_8BIT, &_whoAmI, 1, 100);
	if(_whoAmI != 0xC4){
		return false;
	}

}

float Altimeter::read_altitude(void){

	float altitude;
	uint8_t dataBytes[3];

	this->set_mode_altimeter();
	this->toggle_one_shot();

	HAL_I2C_Mem_Read(_i2c, _altimeter_address, _altimeter_delta_pressure_MSB, I2C_MEMADD_SIZE_8BIT, &_pressure_registers[0], 1, 100);
	if((this->read_status_reg() & 0x04) != 0){

		HAL_I2C_Mem_Read(_i2c, _altimeter_address, 0x01, 1, dataBytes, 3, 512);

		// do some bitshifting first to get a 20bit altitude value info from datasheet:  there are some typos there though )
		//Left shifting the
		//OUT_P_MSB byte by 24 bits into a 32 variable and doing a logical OR with the OUT_P_CSB byte
		//left shifted 16 bits and a logical OR with the OUT_P_LSB byte left shifted 8 bits gives the
		//altitude in meters times 65536.

		int32_t raw_altitude = (dataBytes[0] << 24) | (dataBytes[1] << 16) | (dataBytes[2] << 8);
		//2's complement because it's always a signed value for some reason
		raw_altitude = (~raw_altitude +1) *-1;
		float raw_altitude_shifted = raw_altitude / 65536.0;

		_altitude = raw_altitude_shifted - this->get_offset();


		//calculate the moving average of the last 5 readings
		this->process_altitude(_altitude);
		return raw_altitude_shifted;




	}




}

float Altimeter::process_altitude(float altitude){


	//shift readings back by one index
	for(int i = 4 ; i > 0; i --){

		_tempArray[i] = _tempArray[i -1];

	}
	//calculate the average of the array
	_tempArray[0] = _altitude;

	float sum = 0;
	for(int i = 0; i < (sizeof(_tempArray) / sizeof(float)); i++){

		sum += _tempArray[i];

	}
	_moving_avg = sum / 5.0;
	return _moving_avg;

}




float Altimeter::debug_altitude(void){

	this->set_mode_altimeter();

	//Big debug function for checking the bitshifts etc....
	uint32_t alt;
	float altitude;
	uint8_t dataBytes[3];

	int counter;
	unsigned long start;

	//read some data to clear the flag

	HAL_I2C_Mem_Read(_i2c, _altimeter_address, _altimeter_delta_pressure_MSB, I2C_MEMADD_SIZE_8BIT, &_pressure_registers[0], 1, 100);

	//check if data is ready
	if((this->read_status_reg() & 0x04) != 0){

		HAL_I2C_Mem_Read(_i2c, _altimeter_address, 0x01, 1, dataBytes, 3, 512);

		std::string _msb_str = std::bitset<8>(dataBytes[0]).to_string();
		std::string _csb_str = std::bitset<8>(dataBytes[1]).to_string();
		std::string _lsb_str = std::bitset<8>(dataBytes[2]).to_string();

		printf("msb byte: %s \r\n", _msb_str.c_str());
		printf("csb byte: %s \r\n", _csb_str.c_str());
		printf("lsb byte: %s \r\n", _lsb_str.c_str());

		int32_t raw_altitude = (dataBytes[0] << 24) | (dataBytes[1] << 16) | (dataBytes[2] << 8);
		raw_altitude = (~raw_altitude +1) * -1;
		std::string raw_altitude_str  = std::bitset<32>(raw_altitude).to_string();
		float raw_altitude_shifted = raw_altitude / 65536.0;
		std::string raw_altitude_shifted_str = std::bitset<32>(raw_altitude_shifted).to_string();


		printf("raw altitude bits %s \r\n", raw_altitude_str.c_str());
		printf("raw altitude bits after dividing by 65536 %s \r\n", raw_altitude_shifted_str.c_str());
		printf("altitude in meters %f \r\n", raw_altitude_shifted);
		this->toggle_one_shot();
		return raw_altitude_shifted;


	}




}




void Altimeter::set_mode_altimeter(void){

	uint8_t response = this->read_ctrl_reg_1();
	uint8_t transmit_data;
	if((response & 0x80) == 0){

		transmit_data = response | 0x80;
		this->write_to_register(_altimeter_ctrl_reg_1, transmit_data);

	}

}


void Altimeter::set_mode_active(void){

	uint8_t response = this->read_ctrl_reg_1();
	uint8_t transmit_data;

	if((response & 0x01) == 0){

		transmit_data = response | 0x01;
		this->write_to_register(_altimeter_ctrl_reg_1, transmit_data);
	}


}



uint8_t Altimeter::read_ctrl_reg_1(void){

	HAL_I2C_Mem_Read(_i2c, _altimeter_address, _altimeter_ctrl_reg_1, I2C_MEMADD_SIZE_8BIT, &_ctrl_reg1_data, 1, 100);
	return _ctrl_reg1_data;
}

uint8_t Altimeter::read_status_reg(void){

	HAL_I2C_Mem_Read(_i2c, _altimeter_address, _altimeter_status_reg, I2C_MEMADD_SIZE_8BIT, &_status_reg_data, 1, 100);
	return _status_reg_data;
}

uint8_t Altimeter::read_data_reg(void){

	HAL_I2C_Mem_Read(_i2c, _altimeter_address, _altimeter_data_event_flag_reg, I2C_MEMADD_SIZE_8BIT, &_flag_reg_data, 1, 100);
	return _flag_reg_data;
}

float Altimeter::get_altitude(void){

	return _moving_avg;
}

bool Altimeter::write_to_register(uint8_t adress, uint8_t value){

	if(	HAL_I2C_Mem_Write(_i2c, _altimeter_address, adress, I2C_MEMADD_SIZE_8BIT, &value, 1, 100) != HAL_OK){

		return false;

	}

}

void Altimeter::toggle_one_shot(void){

	uint8_t response;
	uint8_t transmit_data;
	//first clear OST bit
	response = this->read_ctrl_reg_1();
	if((response & 0x02) != 0){

		response &= ~(0x02);
		this->write_to_register(_altimeter_ctrl_reg_1, (response));

	}
	//check status again and set OST bit
	response = this->read_ctrl_reg_1();
	if((response & 0x02) == 0){

		this->write_to_register(_altimeter_ctrl_reg_1, response | 0x02);
	}


}



//Crude debug offset-setter. Change this with a write to the offset register later
void Altimeter::set_offset(float offset){

	this->_altitude_offset = offset;
}

float Altimeter::get_offset(void){

	return this->_altitude_offset;
}

void Altimeter::Error_Handler(void){

	for (uint8_t i = 0; i < 30; i++){		/* Toggle LED signal for error */
		HAL_GPIO_TogglePin(gled_pc14_GPIO_Port, gled_pc14_Pin); //signal led
		HAL_Delay(50);
		printf("Problem with altimeter class \r\n");


	}

}

