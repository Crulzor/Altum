#include "HerelinkController.h"


//DONT FORGET TO ADD DEFAULT IMPLEMENTATION
HerelinkController::HerelinkController(){};


HerelinkController::HerelinkController(UART_HandleTypeDef *huart_sbus, UART_HandleTypeDef *huart_mavlink, Initializer *init, Components *components)
	:_altimeter(&hi2c2),
	 _sbus(huart_sbus),
	 _convertor(&this->_sbus, init, components, &this->_altimeter),
	 _mavlink(huart_mavlink, &this->_altimeter, &this->_convertor) {

	//_altimeter.init_altimeter();

	this->setMotorSpeed(_maxRPM);
	this->setSquareSize(_squareSize);
	this->setLedIntensity(_ledIntensity);

}

void HerelinkController::setMotorSpeed(int speed){

	this->_convertor._maxPercentage = speed;
}

void HerelinkController::setSquareSize(int size){

	this->_convertor._squarePosB = size;

}

void HerelinkController::setLedIntensity(int intensity){

	this->_convertor._max_led_intensity = intensity;

}

Altimeter HerelinkController::getAltimeter(void){

	return _altimeter;
}

MavlinkControl HerelinkController::getMavlink(void){

	return _mavlink;

}

SBUS HerelinkController::getSbus(void){

	return _sbus;
}

void HerelinkController::update(void){

	//Volgorde is belangrijker dan je denkt. Heb hier een soort van "scheduler" proberen te maken.
	//Voorlopig werkt deze oplossing goed. Beter zou zijn om met threads en een queue systeem te werken.

	this->_sbus.update();
	this->_convertor.process();


	if(HAL_GetTick() % 500 == 0){
		this->_mavlink.heartbeat();



	}if(HAL_GetTick() % 103 == 0){

		this->_mavlink.update_RX();
		//this->_mavlink.sendTestMessage();
		//this->_mavlink.sendAltitude();
		this->_mavlink.sendBattery();


	}if(HAL_GetTick() % 120 == 0){

		this->_mavlink.sendFluids();

	}
		//this->_altimeter.read_altitude();




}


