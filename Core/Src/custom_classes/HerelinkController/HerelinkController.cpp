#include "HerelinkController.h"


//DONT FORGET TO ADD DEFAULT IMPLEMENTATION
HerelinkController::HerelinkController(){};


HerelinkController::HerelinkController(UART_HandleTypeDef *huart_sbus, UART_HandleTypeDef *huart_mavlink, Initializer *init, Components *components)
	:_altimeter(&hi2c2), _sbus(huart_sbus), _mavlink(huart_mavlink, &this->_altimeter), _convertor(&this->_sbus, init, components, &this->_altimeter){

	_altimeter.init_altimeter();
	printf("initialization ok \r\n");
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

	this->_sbus.update();
	//this->_mavlink.update_RX();
	//this->_mavlink.update_TX();
	//this->_altimeter.read_altitude();
	this->_convertor.process();

}


