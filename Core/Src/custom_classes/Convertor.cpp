#include "Convertor.h"

Convertor::Convertor(SBUS *sbus, Initializer *init, Components* components){

	this->_sbus = sbus;
	this->_timers = init;
	this->_components = components;

}

void Convertor::testLED(void){

	__HAL_TIM_SET_COMPARE(_timers->get_LED_Timer(), TIM_CHANNEL_1, 100);

}

void Convertor::testSelector(void){

	__HAL_TIM_SET_COMPARE(_timers->get_Selector_Timer(),  TIM_CHANNEL_3, 200);
}

void Convertor::getADC(void){


    HAL_ADC_Start_DMA(_components->get_ADC_1(), (uint32_t*)&ADC_1_Buffer, 2);



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




void Convertor::updateSelector(void){

	//puts the ALTUM in either cleaner position or in sensor position.
	this->_selectorPWM = this->_sbus->getRightX();

	float cleanerPos = 110;
	float probePos = 960;
	if(_selectorPWM > 0){

		this->actuatorControl(_selectorPWM, this->_timers->get_Selector_Timer(), TIM_CHANNEL_4,  TIM_CHANNEL_3);
	}else if(_selectorPWM < 0){

		this->actuatorControl(_selectorPWM, this->_timers->get_Selector_Timer(), TIM_CHANNEL_4,  TIM_CHANNEL_3 );

	}


}

void Convertor::updatePushMotor(void){

	this->_pushMotorPWM = this->_sbus->getLeftY();

	if(_pushMotorPWM > 0){

		this->actuatorControl(_pushMotorPWM, this->_timers->get_PushMotor_Timer() , TIM_CHANNEL_2, TIM_CHANNEL_3);

	}else if(_pushMotorPWM < 0){

		this->actuatorControl(_pushMotorPWM, this->_timers->get_PushMotor_Timer(), TIM_CHANNEL_2, TIM_CHANNEL_3);
	}

}

void Convertor::updateFluidMotor(void){



}


int16_t Convertor::get_selectorPWM(){

	return this->_selectorPWM;
}




void Convertor::process(void){


	this->updateLED();
	this->updateSelector();
	this->updatePushMotor();
	this->getADC();

}

uint16_t Convertor::get_LEDPWM(){

	return _ledPWM;
}

int16_t Convertor::get_selector_position(){


	return ADC_1_Buffer[0] / 4.095;
	//return _ADC_1_value /4.095;

}
