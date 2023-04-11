#include "./SBUS.h"

SBUS::SBUS(UART_HandleTypeDef *huart_sbus){

	this->_huart_sbus = huart_sbus;





}

void SBUS::update(void){



	HAL_UARTEx_ReceiveToIdle_DMA(this->_huart_sbus, this->_sbus_buffer, SBUS_PACKET_SIZE);
	this->readSBUS();




}


bool SBUS::readSBUS(void){

		// SBUS protocol: 25-byte packet
	    // 22 bytes in between contain channel data
	    // each channel is a 16-bit value, with 11-bit resolution and 5-bit status
	    // for the Herelink, there are 16 channels, but we only care about channels 1-8 for the sticks

		// buttons are assigned to channels on the herelink controller.

		// OK DUS
		// EERST BYTE 1 EN 2 OPTELLEN/OR, dan bitshiften met 8 posities naar links, dan ANDEN met 0111 1111 1111 (om 11 bits te krijgen
		// en zeker geen losse bits mee te pakken)

		//als iemand er ooit in slaagt om dit in een mooie for-loop of iets dergelijks te gieten, chapeau, ik vind hem niet :').


		//RIGHT JOYSTICK X
		uint16_t MSB =_sbus_buffer[1];
		uint16_t LSB = _sbus_buffer[2] << 8;
		uint16_t thirdByte;
		this->_channels[0] = (MSB | LSB) & 0x07FF;

		//RIGHT JOYSTICK Y
		MSB = _sbus_buffer[2] >> 3;
		LSB = _sbus_buffer[3] << 5;
		this->_channels[1] =(MSB | LSB ) & 0x07FF;

		//LEFT JOYSTICK X
		MSB = _sbus_buffer[3] >> 6;
		LSB = _sbus_buffer[4] << 2;
		thirdByte = _sbus_buffer[5] << 10;
		this->_channels[2] = (MSB | LSB | thirdByte) & 0x07FF;

		//LEFT JOYSTICK Y
		MSB = _sbus_buffer[5] >> 1;
		LSB = _sbus_buffer[6] << 7;
		this->_channels[3] = (MSB | LSB) & 0x07FF;

		//JOGWHEEL
		MSB = _sbus_buffer[6] >> 4;
		LSB = _sbus_buffer[7] << 4;
		this->_channels[4] = (MSB | LSB) & 0x07FF;

		//BUTTON A
		MSB = _sbus_buffer[7] >> 7;
		LSB = _sbus_buffer[8] << 1;
		thirdByte = _sbus_buffer[9] << 9;
		this->_channels[5] = (MSB | LSB | thirdByte) & 0x07FF;

		//BUTTON B
		MSB = _sbus_buffer[9] >> 2;
		LSB = _sbus_buffer[10] << 6;
		this->_channels[6] = (MSB | LSB) & 0x07FF;

		//BUTTON C
		MSB = _sbus_buffer[10] >> 5;
		LSB = _sbus_buffer[11] << 3;
		this->_channels[7] = (MSB | LSB) & 0x07FF;

		//BUTTON D
		MSB = _sbus_buffer[12];
		LSB = _sbus_buffer[13] << 8;
		this->_channels[8] = (MSB | LSB) & 0x07FF;

		//BUTTON HOME
		MSB = _sbus_buffer[13] >> 3;
		LSB = _sbus_buffer[14] << 5;
		this->_channels[9] = (MSB | LSB) & 0x07FF;

		MSB = _sbus_buffer[14] >> 6;
		LSB = _sbus_buffer[15] << 2;
		thirdByte = _sbus_buffer[16] << 10;
		this->_channels[10] = (MSB | LSB | thirdByte) & 0x07FF;

		MSB = _sbus_buffer[16] >> 1;
		LSB = _sbus_buffer[17] << 7;
		this->_channels[11] = (MSB | LSB) & 0x07FF;

		MSB = _sbus_buffer[17] >> 4;
		LSB = _sbus_buffer[18] << 4;
		this->_channels[12] = (MSB | LSB) & 0x07FF;

		MSB = _sbus_buffer[18] >> 7;
		LSB = _sbus_buffer[19] << 1;
		thirdByte = _sbus_buffer[20] >> 9;
		this->_channels[13] = (MSB | LSB | thirdByte) & 0x07FF;

		MSB = _sbus_buffer[20] >> 2;
		LSB = _sbus_buffer[21] << 6;
		this->_channels[14] = (MSB| LSB)  & 0x07FF;

		MSB = _sbus_buffer[21] >> 5;
		LSB = _sbus_buffer[22] << 3;
		this->_channels[15] = (MSB| LSB)  & 0x07FF;

		_channels[16] = _sbus_buffer[23] & 0x001 ? 2047 : 0;



		return 1;


}


//Normalizing joystick values between -100/100

// normalized value = (raw_value - center value) * 100 / (max_value - center_value)

float SBUS::getLeftY(void){

	int16_t raw_value = _channels[2];
	int16_t center_value = 1024;
	int16_t max_up = 364;
	int16_t max_down = 1684;

	if(raw_value < center_value){

		return ((raw_value - center_value) * 1000) / (max_up - center_value);

	}else if(raw_value > center_value){

		return ((raw_value - center_value) * -1000) / (max_down - center_value);

	}else{

	    return 0.0f;

	}

}

float SBUS::getLeftX(void){
  int16_t raw_value = _channels[3];
  float center_value = 1024.0f;
  float max_left = 364.0f;
  float max_right = 1684.0f;

  if (raw_value < center_value) {

    return ((raw_value - center_value) * 1000.0f) / (center_value - max_left);

  } else if (raw_value > center_value) {

    return ((raw_value - center_value) * 1000.0f) / (max_right - center_value);

  } else {

    return 0.0f;

  }
}


float SBUS::getRightY(void){

  int16_t raw_value = _channels[1];
  float center_value = 1024.0f;
  float max_up = 364.0f;
  float max_down = 1684.0f;

  if (raw_value < center_value) {

    return ((raw_value - center_value) * 1000.0f) / (center_value - max_down);

  } else if (raw_value > center_value) {

    return ((raw_value - center_value) * 1000.0f) / (max_up - center_value);

  } else {

	  return 0.0f;

  }
}


float SBUS::getRightX(void){

  int16_t raw_value = _channels[0];
  float center_value = 1024.0f;
  float max_left = 364.0f;
  float max_right = 1684.0f;


  if (raw_value < center_value) {

    return ((raw_value - center_value) * 1000.0f) / (center_value - max_left);

  } else if (raw_value > center_value) {

    return ((raw_value - center_value) * 1000.0f) / (max_right - center_value);

  } else {

    return 0.0f;

  }

}


float SBUS::getJogWheel(void){

	int16_t raw_value = _channels[4];

	float center_value = 1024.0f;
	float max_left = 1684.0f;
	float max_right = 364.0f;

	if(raw_value < center_value){

		return ((raw_value - center_value) * 1000.0f /(center_value - max_left));

	}else if (raw_value > center_value){

		return ((raw_value - center_value) * 1000.0f / (max_right - center_value));
	}else {

		return 0.0f;
	}


}

bool SBUS::A_button(void){

	//state machine to make sure the button presses debounce correctly (I know it's overkill but hey...).

	  static uint8_t debounce_state = 0;
	  static uint8_t debounce_counter = 0;

	  bool button_pressed = (_channels[5] > 500);

	  switch (debounce_state) {

	    case 0:
	      if (button_pressed) {
	        debounce_state = 1;
	        debounce_counter = 0;
	      }
	      break;

	      // button pressed, waiting for debounce
	    case 1:
	      if (!button_pressed) {
	        debounce_state = 0;
	      } else if (++debounce_counter >= _debounceTime) {
	        debounce_state = 2;
	      }
	      break;

	    case 2:  // button pressed and debounced
	      if (!button_pressed) {
	        debounce_state = 0;
	        return true;
	      }
	      break;
	  }

	  return false;

}

bool SBUS::B_button(void){


	  static uint8_t debounce_state = 0;
	  static uint8_t debounce_counter = 0;

	  bool button_pressed = (_channels[6] > 1000
			  );

	  switch (debounce_state) {
	    case 0:  // button released
	      if (button_pressed) {
	        debounce_state = 1;
	        debounce_counter = 0;
	      }
	      break;

	    case 1:  // button pressed, waiting for debounce
	      if (!button_pressed) {
	        debounce_state = 0;
	      } else if (++debounce_counter >= _debounceTime) {
	        debounce_state = 2;
	      }
	      break;

	    case 2:  // button pressed and debounced
	      if (!button_pressed) {
	        debounce_state = 0;
	        return true;
	      }
	      break;
	  }

	  return false;


}

bool SBUS::C_button(void){

	  static uint8_t debounce_state = 0;
	  static uint8_t debounce_counter = 0;

	  bool button_pressed = (_channels[7] > 1000);

	  switch (debounce_state) {
		case 0:  // button released
		  if (button_pressed) {
			debounce_state = 1;
			debounce_counter = 0;
		  }
		  break;

		case 1:  // button pressed, waiting for debounce
		  if (!button_pressed) {
			debounce_state = 0;
		  } else if (++debounce_counter >= _debounceTime) {
			debounce_state = 2;
		  }
		  break;

		case 2:  // button pressed and debounced
		  if (!button_pressed) {
			debounce_state = 0;
			return true;
		  }
		  break;
	  }

	  return false;

}

bool SBUS::D_button(void){

	  static uint8_t debounce_state = 0;
	  static uint8_t debounce_counter = 0;

	  bool button_pressed = (_channels[8] > 1000);

	  switch (debounce_state) {
		case 0:  // button released
		  if (button_pressed) {
			debounce_state = 1;
			debounce_counter = 0;
		  }
		  break;

		case 1:  // button pressed, waiting for debounce
		  if (!button_pressed) {
			debounce_state = 0;
		  } else if (++debounce_counter >= _debounceTime) {
			debounce_state = 2;
		  }
		  break;

		case 2:  // button pressed and debounced
		  if (!button_pressed) {
			debounce_state = 0;
			return true;
		  }
		  break;
	  }

	  return false;

}

bool SBUS::home_button_long(void){

    static uint8_t debounce_state = 0;
	  static uint8_t debounce_counter = 0;

	  bool button_pressed = (_channels[14] > 1000);

	  switch (debounce_state) {
		case 0:  // button released
		  if (button_pressed) {
			debounce_state = 1;
			debounce_counter = 0;
		  }
		  break;

		case 1:  // button pressed, waiting for debounce
		  if (!button_pressed) {
			debounce_state = 0;
		  } else if (++debounce_counter >= _debounceTime) {
			debounce_state = 2;
		  }
		  break;

		case 2:  // button pressed and debounced
		  if (!button_pressed) {
			debounce_state = 0;
			return true;
		  }
		  break;
	  }

	  return false;

}

bool SBUS::home_button(void){

      static uint8_t debounce_state = 0;
	  static uint8_t debounce_counter = 0;

	  bool button_pressed = (_channels[9] > 1000);

	  switch (debounce_state) {
		case 0:  // button released
		  if (button_pressed) {
			debounce_state = 1;
			debounce_counter = 0;
		  }
		  break;

		case 1:  // button pressed, waiting for debounce
		  if (!button_pressed) {
			debounce_state = 0;
		  } else if (++debounce_counter >= _debounceTime) {
			debounce_state = 2;
		  }
		  break;

		case 2:  // button pressed and debounced
		  if (!button_pressed) {
			debounce_state = 0;
			return true;
		  }
		  break;
	  }

	  return false;
}
