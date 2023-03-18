#include "Convertor.h"

Convertor::Convertor(SBUS *sbus, Initializer *init, Components* components){

	this->_sbus = sbus;
	this->_timers = init;
	this->_components = components;

	this->_ledPWM = 0;


}


void Convertor::ledOFF(void){

	__HAL_TIM_SET_COMPARE(_timers->get_LED_Timer(), TIM_CHANNEL_1, 0);


}
void Convertor::testSelector(void){

	__HAL_TIM_SET_COMPARE(_timers->get_selector_Timer(),  TIM_CHANNEL_3, 200);
}

void Convertor::getADC(void){


    HAL_ADC_Start_DMA(_components->get_ADC_1(), (uint32_t*)&ADC_1_Buffer, 3);
    HAL_ADC_Start_DMA(_components->get_ADC_2(), (uint32_t*)&ADC_2_Buffer, 2);
    HAL_ADC_Start_DMA(_components->get_ADC_5(), (uint32_t*)&ADC_5_Buffer, 2);



}

void Convertor::updateLED(void){



	if(_sbus->B_button() && _ledPWM < 1000) {
		_ledPWM += 250;
        __HAL_TIM_SET_COMPARE(_timers->get_LED_Timer(), TIM_CHANNEL_1, _ledPWM);




	}else if(_sbus->A_button() && _ledPWM > 0){

		_ledPWM -= 250;
        __HAL_TIM_SET_COMPARE(_timers->get_LED_Timer(), TIM_CHANNEL_1, _ledPWM);



	}

}


//LOW LEVEL FUNCTION FOR CONTROLLING ACTUATOR PWM
//TAKES A HANDL

void Convertor::actuatorControl(int16_t pwm_input, TIM_HandleTypeDef *tim, uint32_t channelTimPlus, uint32_t channelTimMin){

	if (pwm_input >= 0) { //if joystick is up then pwm is positive and will drive plus forward cahnnel of h bridge.
		__HAL_TIM_SET_COMPARE(tim, channelTimPlus, abs(pwm_input));
		__HAL_TIM_SET_COMPARE(tim, channelTimMin, 0);

	} else if (pwm_input < 0) {//if joystick is down then pwm is negatieve and will drive plus forward channel of h bridge.
		__HAL_TIM_SET_COMPARE(tim, channelTimPlus, 0);
		__HAL_TIM_SET_COMPARE(tim, channelTimMin, abs(pwm_input));
	}
}


void Convertor::updateCleanerMotor(void){

	this->_cleanerMotorPWM = 0;
	this->_cleanerMotorPWM = this->_sbus->getRightY();

	if(_cleanerMotorPWM > 10){
		__HAL_TIM_SET_COMPARE(_timers->get_cleanerMotor_Timer(), TIM_CHANNEL_3, abs(_cleanerMotorPWM));

	}else{

		__HAL_TIM_SET_COMPARE(_timers->get_cleanerMotor_Timer(), TIM_CHANNEL_3, 0);

	}



}



void Convertor::updateSelector(void){

	//puts the ALTUM in either cleaner position or in sensor position.
	this->_selectorPWM = this->_sbus->getRightX();

	float cleanerPos = 110;
	float probePos = 960;
	if(_selectorPWM > 0){

		this->actuatorControl(_selectorPWM, this->_timers->get_selector_Timer(), TIM_CHANNEL_4,  TIM_CHANNEL_3);
	}else if(_selectorPWM < 0){

		this->actuatorControl(_selectorPWM, this->_timers->get_selector_Timer(), TIM_CHANNEL_4,  TIM_CHANNEL_3 );

	}


}

void Convertor::updatePushMotor(void){

	//currently basic implementation for push motor


	this->_pushMotorPWM = this->_sbus->getLeftY();

	if(_pushMotorPWM > 0){

		this->actuatorControl(_pushMotorPWM, this->_timers->get_pushMotor_Timer() , TIM_CHANNEL_2, TIM_CHANNEL_3);

	}else if(_pushMotorPWM < 0){

		this->actuatorControl(_pushMotorPWM, this->_timers->get_pushMotor_Timer(), TIM_CHANNEL_2, TIM_CHANNEL_3);
	}

}

void Convertor::updateFluidMotor(void){

	//currently basic implementation for fluid motor
	//simple movement with jogwheel for debug now, fix buttons and fluidpercentage later.

	this->_fluidPWM = this->_sbus->getJogWheel();

	if(_fluidPWM > 0){

		this->actuatorControl(_fluidPWM, this->_timers->get_fluidMotor_Timer() , TIM_CHANNEL_2, TIM_CHANNEL_1);

	}else if(_fluidPWM < 0){

		this->actuatorControl(_fluidPWM, this->_timers->get_fluidMotor_Timer(), TIM_CHANNEL_2, TIM_CHANNEL_1);
	}


}


int16_t Convertor::get_selectorPWM(void){

	return this->_selectorPWM;

}


int16_t Convertor::get_fluidPWM(void){

	return this->_fluidPWM;
}


int16_t Convertor::get_pushPWM(void){


	return this->_pushMotorPWM;

}

int16_t Convertor::get_cleanerMotorPWM(void){

	return this->_cleanerMotorPWM;


}

int16_t Convertor::get_LEDPWM(void){

	return _ledPWM;
}

int16_t Convertor::get_selector_position(void){

	return ADC_1_Buffer[0] / 4.095;
}

int16_t Convertor::get_fluidPosition(void){

	return ADC_1_Buffer[3] / 4.095;

}

int16_t Convertor::get_pushPosition(void){

	return ADC_1_Buffer[2] / 4.095;
}


int16_t Convertor::get_LEDCurrent(void){

	return ADC_2_Buffer[0] / 4.095;
}

int16_t Convertor::get_fluidCurrent(void){


	return ADC_2_Buffer[1] / 4.095;
}

int16_t Convertor::get_pushCurrent(void){

	return ADC_5_Buffer[0] / 4.095;

}

int16_t Convertor::get_selectorCurrent(void){

	return ADC_5_Buffer[1] / 4.095;
}


void Convertor::process(void){


	this->updateLED();
	this->updateSelector();
	this->updatePushMotor();
	this->updateFluidMotor();
	this->updateCleanerMotor();
	this->getADC();

}




