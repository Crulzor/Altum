#include "MavlinkControl.h"
//There's a lot going on in this class to make the mavlink protocol work properly with the
//HAL_UARTEx_RxEventCallback function.


MavlinkControl* MavlinkControl::instancePtr = nullptr;


MavlinkControl::MavlinkControl(){};

MavlinkControl::MavlinkControl(UART_HandleTypeDef* huart, Altimeter* altimeter, Convertor* convertor)
: _huart_mavlink(huart), _altimeter(altimeter), _convertor(convertor)  {

	  instancePtr = this;
		//HAL_UARTEx_ReceiveToIdle_DMA(_huart_mavlink, _receiveBuffer_1, MAVLINK_BUFFER_SIZE);



}

MavlinkControl::MavlinkControl(UART_HandleTypeDef* huart): _huart_mavlink(huart){}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {

  //Access the instance of the class using the static member variable
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



  instance->_bufferIndex = 0;

}

void MavlinkControl::update_RX(void){

	HAL_UARTEx_ReceiveToIdle_DMA(_huart_mavlink, _receiveBuffer_1, MAVLINK_BUFFER_SIZE);

}


MavlinkControl::mavlink_header_t MavlinkControl::getMavlinkHeader(){

	return _mavlink_received_header;

}


void MavlinkControl::heartbeat(void){
	//Encode the information and send it periodically to maintain connection. 

	_bufferLength = mavlink_msg_heartbeat_encode(
			_mavlink_system.sysid, _mavlink_system.compid, &_mavlinkSend_1,
			&_mavlink_heartbeat);

	//Check and see if you need the pack or encode function. 
	mavlink_msg_heartbeat_pack(_mavlink_system.sysid, _mavlink_system.compid,
			&_mavlinkSend_1, MAV_TYPE_GROUND_ROVER, MAV_AUTOPILOT_GENERIC,
			MAV_MODE_FLAG_SAFETY_ARMED, 0, MAV_STATE_STANDBY);
	mavlink_msg_to_send_buffer(_bufferPackedforUart, &_mavlinkSend_1);

	HAL_UART_Transmit_DMA(_huart_mavlink,_bufferPackedforUart, _bufferLength);


}

void MavlinkControl::sendTestMessage(void){

	float testValue = 122.0f;


	//mavlink_msg_battery_status_encode(_mavlink_system.sysid, _mavlink_system.compid, &_mavlinkSend_2, &_mavlink_battery);
	_TX_bufferLength = mavlink_msg_sys_status_pack(_mavlink_system.sysid, _mavlink_system.compid, &_mavlinkSend_2,-1,-1,-1,-1,-1, 69, 50,-1,-1,-1,-1,-1,-1,-1,-1,-1);

	// mavlink_msg_vfr_hud_pack function below repacks the value to a hud message. These can be configured on the QgroundControl app.
	//The hud message can contain values such as ground speed, altitude, etc... autocomplete will give you an overview
	//of the different values that can be sent. The values itself are hardcoded into the firmware of the Herelink controller though

	//mavlink_msg_vfr_hud_pack(_mavlink_system.sysid, _mavlink_system.compid,&_mavlinkSend_1, testValue, 0.0f,0,0,0,0);
	mavlink_msg_to_send_buffer(_bufferPackedforUart, &_mavlinkSend_2);
	HAL_UART_Transmit_DMA(_huart_mavlink,_bufferPackedforUart,_TX_bufferLength );


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
	mavlink_msg_altitude_pack(_mavlink_system.sysid, _mavlink_system.compid, &_mavlinkSend_2, 0 , altitude, altitude, altitude, altitude, 0, 0);
	_TX_bufferLength = mavlink_msg_to_send_buffer(_bufferPackedforUart, &_mavlinkSend_2);

	
	 HAL_UART_Transmit_DMA(_huart_mavlink, _bufferPackedforUart, _TX_bufferLength);
	

}

void MavlinkControl::sendBattery(void){
	// Ok, so for some reason (probably because of the firmware inside the HerelinkController or the Mavlink dialect of the Qground app)
	// I can't send the battery data with the battery_status_pack function. So I'm sending battery info with 
	// The system status function.

	uint16_t batteryRemaining = _convertor->get_battery_voltage();

	_TX_bufferLength = mavlink_msg_sys_status_pack(
    	_mavlink_system.sysid,      // System ID
    	_mavlink_system.compid,     // Component ID
		&_mavlinkSend_2,            // MAVLink message container
		-1,                         // Onboard control system health (-1: unknown, 0: not ok, 1: ok)
		-1,                         // Onboard sensor health (-1: unknown, 0: not ok, 1: ok)
		-1,                         // Onboard GPS health (-1: unknown, 0: not ok, 1: ok)
		-1,                         // Onboard power system health (-1: unknown, 0: not ok, 1: ok)
		-1,                         // Onboard communication system health (-1: unknown, 0: not ok, 1: ok)
		-1,           // Remaining battery (0-100%, -1: autopilot does not estimate the remaining battery)
		batteryRemaining,           // Current battery (0-100%, -1: autopilot does not measure the current battery)
		-1,                         // Battery energy consumed (-1: autopilot does not provide energy consumption estimate)
		-1,                         // Battery voltage (mV) (-1: autopilot does not measure the voltage)
		-1,                         // Battery current (cA) (-1: autopilot does not measure the current)
		-1,                         // Battery drop rate (cA/s) (-1: autopilot does not measure the current drop rate)
		-1,                         // Battery remaining time (minutes) (-1: autopilot does not estimate the remaining battery time)
		-1,                         // Battery remaining capacity (mAh) (-1: autopilot does not estimate the remaining battery capacity)
		-1,                         // Battery charging state (0: not charging, 1: charging)
		-1,                         // Battery mode (0: power source is connected, 1: power source is disconnected)
		-1                          // Fault/health indications (for future use)
);


	mavlink_msg_to_send_buffer(_bufferPackedforUart, &_mavlinkSend_2);
	HAL_UART_Transmit_DMA(_huart_mavlink,_bufferPackedforUart,_TX_bufferLength);



}

void MavlinkControl::sendFluids(void){

	float fluidPosition = _convertor->get_fluidPosition();
	float fluidPercentage = fluidPosition / 10;
	int16_t fluidAmount = _convertor->get_fluidAmount();



	_TX_bufferLength = mavlink_msg_vfr_hud_pack(_mavlink_system.sysid, _mavlink_system.compid, &_mavlinkSend_1, 0, 0, 0, fluidPercentage,0,fluidAmount);
	mavlink_msg_to_send_buffer(_bufferPackedforUart_2, &_mavlinkSend_1);
	HAL_UART_Transmit_DMA(_huart_mavlink, _bufferPackedforUart_2, _TX_bufferLength);

}


uint32_t MavlinkControl::getFlightTime(void){

	return this->_flight_time;
}


void MavlinkControl::Error_Handler(void){

	for (uint8_t i = 0; i < 30; i++){		/* Toggle LED signal for error */
		HAL_GPIO_TogglePin(gled_pc14_GPIO_Port, gled_pc14_Pin); //signal led
		HAL_Delay(50);

	}

}

