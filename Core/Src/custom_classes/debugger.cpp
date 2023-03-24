#include "debugger.h"

Debugger::Debugger(SBUS *sbus, MavlinkControl* mavlink, Convertor *convertor){

	this->_sbus = sbus;
	this->_convertor = convertor;
	this->_mavlink = mavlink;

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

		printf("Jogwheel: %f \r\n", this->_sbus->getJogWheel());


		printf("LedPWM: %d\r\n", this->_convertor->get_LEDPWM());
		printf("selectorPWM: %d\r\n", this->_convertor->get_selectorPWM());
		printf("push motor PWM: %d \r\n", this->_convertor->get_pushPWM());
		printf("fluid motor PWM: %d \r\n", this->_convertor->get_fluidPWM());
		printf("cleaner motor PWM: %d \r\n", this->_convertor->get_cleanerMotorPWM());
		printf("\r\n");

		printf("selector position %d \r\n", this->_convertor->get_selector_position());
		printf("fluid position %d \r\n", this->_convertor->get_fluidPosition());
		printf("push position %d \r\n", this->_convertor->get_pushPosition());

		printf("\r\n");


		printf("led current:  %d \r\n", this->_convertor->get_LEDCurrent());
		printf("fluid actuator current: %d \r\n", this->_convertor->get_fluidCurrent());
		printf("push motor current: %d \r\n", this->_convertor->get_pushCurrent());
		printf("selector motor current: %d \r\n", this->_convertor->get_selectorCurrent());

		printf("\r\n");


		printf("flight time %d \r\n", _mavlink->getFlightTime());

		printf("\r\n");

		printf("Mavlink start of message: %d \r\n", _mavlink->_mavlink_received_header.magic);
		printf("Mavlink payload length: %d \r\n", _mavlink->_mavlink_received_header.len);
		printf("Mavlink incompat flags: %d \r\n", _mavlink->_mavlink_received_header.incompat_flags);
		printf("Mavlink compat flags: %d \r\n", _mavlink->_mavlink_received_header.compat_flags);
		printf("Mavlink seq: %d \r\n", _mavlink->_mavlink_received_header.seq);
		printf("Mavlink sys id: %d \r\n", _mavlink->_mavlink_received_header.sysid);
		printf("Mavlink comp id: %d \r\n", _mavlink->_mavlink_received_header.compid);
		printf("Mavlink msg id: %d \r\n", _mavlink->_mavlink_received_header.msgid);

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

void Debugger::displayMavlink_header(void){



	if(HAL_GetTick() % 100 == 0 && _mavlink->_mavlink_received_header.magic != 0){
		printf("Mavlink start of message: %d \r\n", _mavlink->_mavlink_received_header.magic);
		printf("Mavlink payload length: %d \r\n", _mavlink->_mavlink_received_header.len);
		printf("Mavlink incompat flags: %d \r\n", _mavlink->_mavlink_received_header.incompat_flags);
		printf("Mavlink compat flags: %d \r\n", _mavlink->_mavlink_received_header.compat_flags);
		printf("Mavlink seq: %d \r\n", _mavlink->_mavlink_received_header.seq);
		printf("Mavlink sys id: %d \r\n", _mavlink->_mavlink_received_header.sysid);
		printf("Mavlink comp id: %d \r\n", _mavlink->_mavlink_received_header.compid);
		printf("Mavlink msg id: %d \r\n", _mavlink->_mavlink_received_header.msgid);

		printf("\r\n");
		printf("\r\n");

	}



}

void Debugger::displayMavlink_RAW(void){

	if(HAL_GetTick() % 	100 == 0){

		for(int i = 0; i < sizeof(_mavlink->_receiveBuffer_1) ; i++){

			printf("Mavlink bytes: %d \r\n", i);
			printf("value: %d \r\n", _mavlink->_receiveBuffer_1[i]);

			printf("\r\n");
			printf("\r\n");

		}

	}

}


