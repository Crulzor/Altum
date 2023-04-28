#include "MavlinkControl.h"
//There's a lot going on in this class to make the mavlink protocol work properly with the
//HAL_UARTEx_RxEventCallback function.


MavlinkControl* MavlinkControl::instancePtr = nullptr;


MavlinkControl::MavlinkControl(){};

MavlinkControl::MavlinkControl(UART_HandleTypeDef* huart, Altimeter* altimeter)
: _huart_mavlink(huart), _altimeter(altimeter)  {

	  instancePtr = this;
		HAL_UARTEx_ReceiveToIdle_DMA(_huart_mavlink, _receiveBuffer_1, MAVLINK_BUFFER_SIZE);

		//HAL_UART_Receive_DMA(_huart_mavlink, _receiveBuffer_1, RX_BUFFER_SIZE);


}

MavlinkControl::MavlinkControl(UART_HandleTypeDef* huart): _huart_mavlink(huart){}

void MavlinkControl::uartRxCallback(UART_HandleTypeDef *huart, uint16_t Size) {
  // Call the overridden function with the instancePtr as the first argument
  HAL_UARTEx_RxEventCallback(instancePtr->_huart_mavlink, Size);
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {

  // Access the instance of the class using the static member variable
  MavlinkControl* instance = MavlinkControl::instancePtr;
//  instance->process_header();
  mavlink_status_t status;
  mavlink_message_t msg;

  while (mavlink_parse_char(MAVLINK_COMM_0, instance->_receiveBuffer_1[instance->_bufferIndex++], &instance->_mavlinkReceived, &instance->_status) == MAVLINK_FRAMING_INCOMPLETE);
  instance->_mavlink_received_header.magic = instance->_mavlinkReceived.magic;
  instance->_mavlink_received_header.sysid = instance->_mavlinkReceived.sysid;
  instance->_mavlink_received_header.compid = instance->_mavlinkReceived.compid;
  instance->_mavlink_received_header.seq = instance->_mavlinkReceived.seq;
  instance->_mavlink_received_header.compat_flags = instance->_mavlinkReceived.compat_flags;
  instance->_mavlink_received_header.incompat_flags = instance->_mavlinkReceived.incompat_flags;
  instance->_mavlink_received_header.msgid = instance->_mavlinkReceived.msgid;
  instance->_mavlink_received_header.len = instance->_mavlinkReceived.len;

  instance->readFlightTime(instance->_mavlinkReceived);
  instance->decodeHeartbeat(instance->_mavlinkReceived);


  HAL_UART_Receive_DMA(instance->_huart_mavlink, instance->_receiveBuffer_1, instance->MAVLINK_BUFFER_SIZE);

  instance->_bufferIndex = 0;

}

void MavlinkControl::update_RX(void){

	HAL_UARTEx_ReceiveToIdle_DMA(_huart_mavlink, _receiveBuffer_1, MAVLINK_BUFFER_SIZE);

	
}





void MavlinkControl::update_TX(void){


	if(HAL_GetTick() % 100 == 0){

		this->heartbeat();


	}else if(HAL_GetTick() % 203 == 0){
		this->sendAltitude();

	}
}

MavlinkControl::mavlink_header_t MavlinkControl::getMavlinkHeader(){

	return _mavlink_received_header;

}


void MavlinkControl::heartbeat(void){


	_bufferLength = mavlink_msg_heartbeat_encode(
			_mavlink_system.sysid, _mavlink_system.compid, &_mavlinkSend,
			&_mavlink_heartbeat);

	//Check and see if you need the pack or encode function. 
	mavlink_msg_heartbeat_pack(_mavlink_system.sysid, _mavlink_system.compid,
			&_mavlinkSend, MAV_TYPE_GROUND_ROVER, MAV_AUTOPILOT_GENERIC,
			MAV_MODE_FLAG_SAFETY_ARMED, 0, MAV_STATE_STANDBY);
	mavlink_msg_to_send_buffer(_bufferPackedforUart, &_mavlinkSend);

	if(HAL_UART_Transmit_IT(_huart_mavlink,_bufferPackedforUart, _bufferLength) != HAL_OK){

				printf("NOT ABLE TO TRANSMIT MAVLINK PACKAGE \r\n");

	}


}

void MavlinkControl::sendTestMessage(void){

	float testValue = 122.0f;

	_mavlink_battery.battery_remaining = 50;

	// mavlink_msg_vfr_hud_pack function below repacks the value to a hud message. These can be configured on the QgroundControl app.
	//The hud message can contain values such as ground speed, altitude, etc... autocomplete will give you an overview
	//of the different values that can be sent. The values itself are hardcoded into the firmware of the Herelink controller though

	mavlink_msg_vfr_hud_pack(_mavlink_system.sysid, _mavlink_system.compid,&_mavlinkSend, testValue, 0.0f,0,0,0,0);
	_TX_bufferLength = mavlink_msg_to_send_buffer(_bufferPackedforUart, &_mavlinkSend);
	if( HAL_UART_Transmit(_huart_mavlink, _bufferPackedforUart, _TX_bufferLength, 100) != HAL_OK){
		printf("NOT ABLE TO TRANSMIT MAVLINK PACKAGE \r\n");
	}



}

void MavlinkControl::decodeHeartbeat(mavlink_message_t receivedMessage){


	if(receivedMessage.msgid == MAVLINK_MSG_ID_HEARTBEAT){

		mavlink_msg_heartbeat_decode(&receivedMessage, &_received_heartbeat);

	}


}


//doesn't work, never receives a proper message 
void MavlinkControl::readFlightTime(mavlink_message_t receivedMessage) {

	if(receivedMessage.msgid == MAVLINK_MSG_ID_SYS_STATUS){
		printf("inside read flight time function \r\n");
		mavlink_sys_status_t status;
		mavlink_msg_sys_status_decode(&receivedMessage,&status);
        _flight_time = status.onboard_control_sensors_present;

	}



}


void MavlinkControl::sendAltitude(void){

	// mavlink_msg_vfr_hud_pack function below repacks the value to a hud message. These can be configured on the QgroundControl app.
	//The hud message can contain values such as ground speed, altitude, etc... autocomplete will give you an overview
	//of the different values that can be sent. The values itself are hardcoded into the firmware of the Herelink controller though


	float altitude = _altimeter->get_altitude();
	//Function below repacks the value to a hud message. These can be configured on the QgroundControl app.
	mavlink_msg_vfr_hud_pack(_mavlink_system.sysid, _mavlink_system.compid,&_mavlinkSend, altitude , 0.0f,0,0,0,0);
	mavlink_msg_altitude_pack(_mavlink_system.sysid, _mavlink_system.compid, &_mavlinkSend, 0 , altitude, altitude, altitude, altitude, 0, 0);
	_TX_bufferLength = mavlink_msg_to_send_buffer(_bufferPackedforUart, &_mavlinkSend);
	if( HAL_UART_Transmit(_huart_mavlink, _bufferPackedforUart, _TX_bufferLength, 100) != HAL_OK){
		printf("NOT ABLE TO TRANSMIT MAVLINK PACKAGE \r\n");
	}

}


uint32_t MavlinkControl::getFlightTime(void){

	return this->_flight_time;
}


void MavlinkControl::Error_Handler(void){

	for (uint8_t i = 0; i < 30; i++){		/* Toggle LED signal for error */
		HAL_GPIO_TogglePin(gled_pc14_GPIO_Port, gled_pc14_Pin); //signal led
		HAL_Delay(50);
		printf("Problem with mavlink \r\n");

	}

}

