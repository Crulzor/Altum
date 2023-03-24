#include "MavlinkControl.h"
//There's a lot going on in this class to make the mavlink protocol work properly with the
//HAL_UARTEx_RxEventCallback function.


MavlinkControl* MavlinkControl::instancePtr = nullptr;


MavlinkControl::MavlinkControl(){};

MavlinkControl::MavlinkControl(UART_HandleTypeDef* huart, I2C_HandleTypeDef* i2c)
: _huart_mavlink(huart), _altimeter_i2c(i2c)  {

	  instancePtr = this;

	//HAL_UARTEx_ReceiveToIdle_DMA(_huart_mavlink, _receiveBuffer_1, MAVLINK_BUFFER_SIZE);




}

MavlinkControl::MavlinkControl(UART_HandleTypeDef* huart): _huart_mavlink(huart){}

void MavlinkControl::uartRxCallback(UART_HandleTypeDef *huart, uint16_t Size) {
  // Call the overridden function with the instancePtr as the first argument
  HAL_UARTEx_RxEventCallback(instancePtr->_huart_mavlink, Size);
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {

  // Access the instance of the class using the static member variable
  MavlinkControl* instance = MavlinkControl::instancePtr;
	instance
	->process_header();



}

void MavlinkControl::update_RX(void){
	HAL_UARTEx_ReceiveToIdle_DMA(_huart_mavlink, _receiveBuffer_1, RX_BUFFER_SIZE);


}





void MavlinkControl::update_TX(void){

	//Every 0.5 sec send a heartbeat to the herelink controller. If this runs continuously this
	//blocks the SBUS protocol

	this->sendTestMessage();

	if(HAL_GetTick() % 500 == 0){

		this->heartbeat();

	}

}

void MavlinkControl::process_header(void){


	//First get the header so we know what size the message is.
	//Mavlinkv2 headers are 6 bytes long and contain the fields defined in the header struct (see .h file).
	//first check the header for value 0xFD

	//NOT EVEN SURE IF I NEED THIS LATER


	if(_receiveBuffer_1[0] == 0xFD){
		_receiveBuffer_2 = _receiveBuffer_1;

		//printf("CHECKING IF LEN IS OK %d \r\n", _tempBuffer[1]);


		_mavlink_received_header.magic = _receiveBuffer_2[0];
		_mavlink_received_header.len = _receiveBuffer_2[1];
		_mavlink_received_header.incompat_flags = _receiveBuffer_2[2];
		_mavlink_received_header.compat_flags = _receiveBuffer_2[3];
		_mavlink_received_header.seq = _receiveBuffer_2[4];
		_mavlink_received_header.sysid = _receiveBuffer_2[5];
		_mavlink_received_header.msgid = _receiveBuffer_2[6] | _receiveBuffer_2[7] | _receiveBuffer_2[8];



	}else if(_receiveBuffer_1[0] == 0xFE){

		_receiveBuffer_2 = _receiveBuffer_1;

		_mavlink_received_header.magic = _receiveBuffer_2[0];
		_mavlink_received_header.len = _receiveBuffer_2[1];
		_mavlink_received_header.seq = _receiveBuffer_2[2];
		_mavlink_received_header.sysid = _receiveBuffer_2[3];
		_mavlink_received_header.compid = _receiveBuffer_2[4];
		_mavlink_received_header.msgid = _receiveBuffer_2[5];


	}





}


//NOT SURE IF THIS IS EVEN USEFUL, FELT CUTE MIGHT DELETE LATER
MavlinkControl::mavlink_header_t MavlinkControl::getMavlinkHeader(){

	return _mavlink_received_header;

}


void MavlinkControl::heartbeat(void){


	_bufferLength = mavlink_msg_heartbeat_encode(
			_mavlink_system.sysid, _mavlink_system.compid, &_mavlinkSend,
			&_mavlink_heartbeat);
	mavlink_msg_heartbeat_pack(_mavlink_system.sysid, _mavlink_system.compid,
			&_mavlinkSend, MAV_TYPE_GROUND_ROVER, MAV_AUTOPILOT_GENERIC,
			MAV_MODE_FLAG_SAFETY_ARMED, 0, MAV_STATE_STANDBY);
	mavlink_msg_to_send_buffer(_bufferPackedforUart, &_mavlinkSend);

	if( HAL_UART_Transmit(_huart_mavlink, _bufferPackedforUart, _bufferLength, 100) != HAL_OK){
		printf("NOT ABLE TO TRANSMIT MAVLINK PACKAGE \r\n");
	}

}

void MavlinkControl::sendTestMessage(void){

	float testValue = 122.0f;

	// mavlink_msg_vfr_hud_pack function below repacks the value to a hud message. These can be configured on the QgroundControl app.
	//The hud message can contain values such as ground speed, altitude, etc... autocomplete will give you an overview
	//of the different values that can be sent. The values itself are hardcoded into the firmware of the Herelink controller though

	mavlink_msg_vfr_hud_pack(_mavlink_system.sysid, _mavlink_system.compid,&_mavlinkSend, testValue, 0.0f,0,0,0,0);
	_TX_bufferLength = mavlink_msg_to_send_buffer(_bufferPackedforUart, &_mavlinkSend);
	if( HAL_UART_Transmit(_huart_mavlink, _bufferPackedforUart, _TX_bufferLength, 100) != HAL_OK){
		printf("NOT ABLE TO TRANSMIT MAVLINK PACKAGE \r\n");
	}

}

void MavlinkControl::readFlightTime() {



    // Loop until we receive the heartbeat message
    while (_flight_time == 0) {

        // Read one byte at a time until we get a complete mavlink message
        while (mavlink_parse_char(MAVLINK_COMM_0, _receiveBuffer_1[_bufferIndex++], &_mavlinkReceived, &_status) == MAVLINK_FRAMING_INCOMPLETE);

        // If we have received a heartbeat message
        if (_mavlinkReceived.msgid == MAVLINK_MSG_ID_HEARTBEAT) {

            // Decode the message and extract the flight time
            mavlink_heartbeat_t heartbeat;
            mavlink_msg_heartbeat_decode(&_mavlinkReceived, &heartbeat);
            _flight_time = heartbeat.custom_mode;

            // Reset the receive buffer index
            _bufferIndex = 0;
        }
    }

}

void MavlinkControl::sendAltitude(uint16_t altitude){

	float fAltitude = altitude;
	//Function below repacks the value to a hud message. These can be configured on the QgroundControl app.
	mavlink_msg_vfr_hud_pack(_mavlink_system.sysid, _mavlink_system.compid,&_mavlinkSend, fAltitude , 0.0f,0,0,0,0);
	_TX_bufferLength = mavlink_msg_to_send_buffer(_bufferPackedforUart, &_mavlinkSend);
	if( HAL_UART_Transmit(_huart_mavlink, _bufferPackedforUart, _TX_bufferLength, 100) != HAL_OK){
		printf("NOT ABLE TO TRANSMIT MAVLINK PACKAGE \r\n");
	}

}


uint32_t MavlinkControl::getFlightTime(void){

	return this->_flight_time;
}
