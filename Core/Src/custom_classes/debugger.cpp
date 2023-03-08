#include "debugger.h"

Debugger::Debugger(SBUS *sbus, Convertor *convertor){

	this->_sbus = sbus;
	this->_convertor = convertor;

}


void Debugger::displayDebugInfo(void){


	if(HAL_GetTick() % 100 == 0){


		printf("Left joystick Y: %f \r\n", this->_sbus->getLeftY());
		printf("\r\n");

		printf("Left joystick X: %f \r\n", this->_sbus->getLeftX());
		printf("\r\n");

		printf("Right joystick Y:%f \r\n", this->_sbus->getRightY());
		printf("\r\n");

		printf("Right joystick X: %f \r\n", this->_sbus->getRightX());

		printf("LedPWM: %d\r\n", this->_convertor->get_LEDPWM());

		printf("selectorPWM: %d\r\n", this->_convertor->get_selectorPWM());
		printf("\r\n");

		printf("Selector position %d \r\n", this->_convertor->get_selector_position());







		printf("\r\n");
		printf("\r\n");

		//HAL_GPIO_TogglePin(gled_pc14_GPIO_Port, gled_pc14_Pin);


	}


}

void Debugger::displaySBUS_channels(void){

	if(HAL_GetTick() % 500 == 0){

		for(int i = 0; i < 16 ; i++){

			printf("SBUS channel: %d \r\n", i);
			printf("value: %d \r\n", _sbus->_channels[i]);

		}

	}

}
