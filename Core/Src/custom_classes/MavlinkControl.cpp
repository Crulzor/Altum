#include "MavlinkControl.h"

MavlinkControl::MavlinkControl(){};

MavlinkControl::MavlinkControl(UART_HandleTypeDef* huart) : _huart_mavlink(huart) {

	HAL_UARTEx_ReceiveToIdle_DMA(_huart_mavlink, _receiveBuffer_1, MAVLINK_BUFFER_SIZE);

}




void MavlinkControl::update(void){

	//Every 0.5 sec send a heartbeat to the herelink controller. If this runs continuously this
	//blocks the SBUS protocol




	if(HAL_GetTick() % 500 == 0){

		this->heartbeat();

	}



}


void MavlinkControl::processReceivedBuffer(void){


	//First get the header so we know what size the message is.

	//Mavlink headers are 6 bytes long and contain the fields defined in the header struct (see .h file).



	if(_receiveBuffer_1[0] == 0xFD || _receiveBuffer_1[0] == 0xFE ){

		//printf("testing something %d \r\n", *tempBuffer);



		//printf("PROCESSING BUFFERRRR \r\n");
		_mavlink_received_header.start_of_msg = _receiveBuffer_1[0];
		_mavlink_received_header.payload_len = _receiveBuffer_1[1];
		_mavlink_received_header.sequence_num = _receiveBuffer_1[2];
		_mavlink_received_header.sys_id = _receiveBuffer_1[3];
		_mavlink_received_header.comp_id = _receiveBuffer_1[4];
		_mavlink_received_header.msg_id = _receiveBuffer_1[5];

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
	mavlink_msg_to_send_buffer(_bufferPackedForUart, &_mavlinkSend);

	if( HAL_UART_Transmit(_huart_mavlink, _bufferPackedForUart, _bufferLength, 100) != HAL_OK){
		printf("NOT ABLE TO TRANSMIT MAVLINK PACKAGE \r\n");
	}
	//sendPackage(_bufferPackedForUart, _bufferLength);


}

void MavlinkControl::onReceiveEvent(UART_HandleTypeDef *huart) {
    // Check if the receive event is for the UART that this object is handling
    if (huart == _huart_mavlink) {

    	printf("We got inside the onReceiveEvent function \r\n");
    }
}
