#include "HerelinkController.h"


//DONT FORGET TO ADD DEFAULT IMPLEMENTATION
HerelinkController::HerelinkController(){};


HerelinkController::HerelinkController(UART_HandleTypeDef *huart_sbus, UART_HandleTypeDef *huart_mavlink, Initializer *init, Components *components)
	:_altimeter(&hi2c2), _sbus(huart_sbus), _convertor(&this->_sbus, init, components, &this->_altimeter), _mavlink(huart_mavlink, &this->_altimeter, &this->_convertor){

	//_altimeter.init_altimeter();
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
		this->_mavlink.sendTestMessage();
		//this->_mavlink.sendAltitude();



	}if(HAL_GetTick() % 120 == 0){

		this->_mavlink.sendBattery();

	}
		//this->_altimeter.read_altitude();




}


