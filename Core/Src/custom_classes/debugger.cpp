#include "debugger.h"

Debugger::Debugger(HerelinkController *controller){

	this->_controller = controller;


}


void Debugger::displayDebugInfo(void){
	//BIG debug function. I'm just dumping everything here, because printf still is a bit wonky and this
	//works best...


	if(HAL_GetTick() % 100 == 0){



		printf("Left joystick Y: %f \r\n", this->_controller->_sbus.getLeftY());
		printf("\r\n");

		printf("Left joystick X: %f \r\n", this->_controller->_sbus.getLeftX());
		printf("\r\n");

		printf("Right joystick Y:%f \r\n", this->_controller->_sbus.getRightY());
		printf("\r\n");

		printf("Right joystick X: %f \r\n", this->_controller->_sbus.getRightX());

		printf("Jogwheel: %f \r\n", this->_controller->_sbus.getJogWheel());



		printf("home press: %d \r\n", this->_controller->_sbus._channels[9]);
		printf("shoulder press: %d \r\n", this->_controller->_sbus._channels[11]);

		printf("A press: %d \r\n", this->_controller->_sbus._channels[5]);
		printf("B press: %d \r\n", this->_controller->_sbus._channels[6]);
		printf("C press: %d \r\n", this->_controller->_sbus._channels[7]);
		printf("B press: %d \r\n", this->_controller->_sbus._channels[6]);


		printf("LedPWM: %d\r\n", this->_controller->_convertor.get_LEDPWM());
		printf("selectorPWM: %d\r\n", this->_controller->_convertor.get_selectorPWM());
		printf("push motor PWM: %d \r\n", this->_controller->_convertor.get_pushPWM());
		printf("fluid motor PWM: %d \r\n", this->_controller->_convertor.get_fluidPWM());
		printf("next fluid position: %d \r\n", this->_controller->_convertor._fluidPosition);
		printf("cleaner motor PWM: %d \r\n", this->_controller->_convertor.get_cleanerMotorPWM());
		printf("\r\n");

		printf("battery voltage: %d \r\n", this->_controller->_convertor.get_battery_voltage());
		printf("selector position %d \r\n", this->_controller->_convertor.get_selector_position());
		printf("fluid position %d \r\n", this->_controller->_convertor.get_fluidPosition());
		printf("push position %d \r\n", this->_controller->_convertor.get_pushPosition());
		printf("Toggle lock %d \r\n", this->_controller->_convertor.get_sleepToggle());



		printf("\r\n");


		printf("led current:  %d \r\n", this->_controller->_convertor.get_LEDCurrent());
		printf("fluid actuator current: %d \r\n", this->_controller->_convertor.get_fluidCurrent());
		printf("push motor current: %d \r\n", this->_controller->_convertor.get_pushCurrent());
		printf("selector motor current: %d \r\n", this->_controller->_convertor.get_selectorCurrent());
//
//		printf("\r\n");
//
//		printf("bits and bytes from the altimeter:  \r\n");
//
//		printf("Altitude value: %f \r\n", this->_controller->_altimeter.get_altitude());
//		printf("Altitude offset: %f, \r\n", this->_controller->_altimeter.get_offset());
//		printf("ALTIMETER CONTROL REGISTER 1: %d \r\n", this->_controller->_altimeter.read_ctrl_reg_1());
//		printf("ALTIMETER STATUS REGISTER: %d \r\n",  this->_controller->_altimeter.read_status_reg());
//		printf("ALTIMETER FIFO STATUS REGISTER %d \r\n",  this->_controller->_altimeter.read_data_reg());
//		printf("ALTIMETER WHOAMI  REGISTER %d \r\n",  this->_controller->_altimeter.whoAmI());


//
//		printf("\r\n");
//
//		printf("flight time %d \r\n", this->_controller->_mavlink.getFlightTime());
//
//		printf("\r\n");
//
//		printf("Mavlink start of message: %d \r\n",this->_controller->_mavlink._mavlink_received_header.magic);
//		printf("Mavlink payload length: %d \r\n",this->_controller->_mavlink._mavlink_received_header.len);
//		printf("Mavlink incompat flags: %d \r\n", this->_controller->_mavlink._mavlink_received_header.incompat_flags);
//		printf("Mavlink compat flags: %d \r\n",this->_controller->_mavlink._mavlink_received_header.compat_flags);
//		printf("Mavlink seq: %d \r\n", this->_controller->_mavlink._mavlink_received_header.seq);
//		printf("Mavlink sys id: %d \r\n", this->_controller->_mavlink._mavlink_received_header.sysid);
//		printf("Mavlink received heartbeat system status %d \r\n",this->_controller->_mavlink._received_heartbeat.system_status);
//		printf("Mavlink received heartbeat vehicle type %d \r\n",this->_controller->_mavlink._received_heartbeat.system_status);
//		printf("Mavlink comp id: %d \r\n", this->_controller->_mavlink._mavlink_received_header.compid);
//		printf("Mavlink msg id: %d \r\n", this->_controller->_mavlink._mavlink_received_header.msgid);

		printf("\r\n");
		printf("\r\n");


	}


}

//Functions below are mainly for debugging raw channels.

void Debugger::displaySBUS_channels(void){

	if(HAL_GetTick() % 500 == 0){

		for(int i = 0; i < 16 ; i++){

			printf("SBUS channel: %d \r\n", i);
			printf("value: %d \r\n", this->_controller->_sbus._channels[i]);


		}

	}

}

void Debugger::displayMavlink_RAW(void){

	if(HAL_GetTick() % 	100 == 0){

		for(int i = 0; i < sizeof(this->_controller->_mavlink._receiveBuffer_1) ; i++){

			printf("Mavlink bytes: %d \r\n", i);
			printf("value: %d \r\n", this->_controller->_mavlink._receiveBuffer_1[i]);

			printf("\r\n");
			printf("\r\n");

		}

	}

}

void Debugger::displayAltitude(void){


		//printf("Altitude value: %f \r\n", this->_altimeter->read_altitude());
		printf("Altitude getter value: %f \r\n", this->_controller->_altimeter.get_altitude());



}

void Debugger::debugADC(void){
//
//	uint32_t *adc_buffer_1 = this->_controller->_convertor.ADC_1_Buffer;
//	uint32_t *adc_buffer_2 = this->_controller->_convertor.ADC_2_Buffer;
//	uint32_t *adc_buffer_5 = this->_controller->_convertor.ADC_5_Buffer;
//
//	for(int i = 0; i < sizeof(adc_buffer_1); i++){
//
//			printf("adc buffer 1 no %d : contents: %d ", i, adc_buffer_1[i]);
//	}


}
