#include "Convertor.h"

Convertor::Convertor(SBUS *sbus, Initializer *init, Components* components, Altimeter* altimeter){

	this->_sbus = sbus;
	this->_timers = init;
	this->_components = components;
	this->_altimeter = altimeter;

	this->_ledPWM = 0;
	_pidSelector.setPoint = _cleanerPos;
	PIDControllerInit(&_pidSelector);



}


void Convertor::ledOFF(void){

	__HAL_TIM_SET_COMPARE(_timers->get_LED_Timer(), TIM_CHANNEL_1, 0);


}


uint16_t Convertor::getADC_NO_DMA(ADC_HandleTypeDef *hadc, uint32_t Channel){

	ADC_ChannelConfTypeDef sConfig = { 0 };
	sConfig.Channel = Channel; // ADC_CHANNEL_VOPAMP1; example. switching channels. Better to use built in function ch scanning and dma adc to minmize overhead.
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_47CYCLES_5; //important. Less cycles means less consistent and accurate results.
	sConfig.SingleDiff = ADC_SINGLE_ENDED;
	sConfig.OffsetNumber = ADC_OFFSET_NONE;
	sConfig.Offset = 0;

	// Do the fully config here and use local ADC_ChannelConfTypeDef sConfig.
	//it didn’t work when global (maybe because pointer misused in different adcs).
	if (HAL_ADC_ConfigChannel(hadc, &sConfig) != HAL_OK) {
		Error_Handler();
	}


	HAL_ADC_Start(hadc); //motor
	HAL_ADC_PollForConversion(hadc, 1000);
	uint16_t Value = HAL_ADC_GetValue(hadc); // - 637 for motor offset
	HAL_ADC_Stop(hadc);
	return Value;

}

void Convertor::getADC(void){

	//Could not get more than 2 channels to work over DMA for some reason, so get the adc1 values the regular way.
	ADC_1_Buffer[0] = this->getADC_NO_DMA(_components->get_ADC_1(), ADC_CHANNEL_11);
	ADC_1_Buffer[1] = this->getADC_NO_DMA(_components->get_ADC_1(), ADC_CHANNEL_14);
	ADC_1_Buffer[2] = this->getADC_NO_DMA(_components->get_ADC_1(), ADC_CHANNEL_12);
	ADC_1_Buffer[3] = this->getADC_NO_DMA(_components->get_ADC_1(), ADC_CHANNEL_1);

    //HAL_ADC_Start_DMA(_components->get_ADC_1(), (uint32_t *)&ADC_1_Buffer, 3);
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
void Convertor::actuatorControl(int16_t pwm_input, TIM_HandleTypeDef *tim, uint32_t channelTimPlus, uint32_t channelTimMin){

	if (pwm_input >= 0) { //if joystick is up then pwm is positive and will drive plus forward cahnnel of h bridge.
		__HAL_TIM_SET_COMPARE(tim, channelTimPlus, abs(pwm_input));
		__HAL_TIM_SET_COMPARE(tim, channelTimMin, 0);

	} else if (pwm_input < 0) {//if joystick is down then pwm is negatieve and will drive plus forward channel of h bridge.
		__HAL_TIM_SET_COMPARE(tim, channelTimPlus, 0);
		__HAL_TIM_SET_COMPARE(tim, channelTimMin, abs(pwm_input));
	}
}

void Convertor::stopCleanerMotor(void){

	__HAL_TIM_SET_COMPARE(_timers->get_cleanerMotor_Timer(), TIM_CHANNEL_3, 0);

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


// this function updates the selector based on SBUS input values
// it detects different SBUS input values and sets the `_indexer` and `_pidSelector.setPoint` accordingly.
// It uses debounce triggering to prevent unwanted toggling,
// Based on the `_indexer`, the `_pidSelector.setPoint` is set to specific positions like `_cleanerPos`, `_probePos`, `_squarePosA`, and `_squarePosB`.
// If none of the conditions match, `_pidSelector.setPoint` is set to `_cleanerPos` as the default value.
void Convertor::updateSelector(void){

	static bool debounceTrigger = 0;
	float tolerance = 5.0f;

	if(_sbus->getRightX() < -750 && debounceTrigger == 0){

		debounceTrigger = 1;
		_indexer = 0;

	}else if(_sbus->getRightX() > 750 && debounceTrigger == 0){

		debounceTrigger = 1;
		_indexer = 1;

	}else if(_sbus->getRightY() < -750 && debounceTrigger == 0){

		debounceTrigger = 1;
		_indexer = 2;
	}else if ((_sbus->getRightY() == 0 && _sbus->getRightX()==0) && debounceTrigger == 1){

		debounceTrigger = 0;
	}

	switch(_indexer){


		case 0:
			_pidSelector.setPoint = _cleanerPos;

			break;

		case 1:
			_pidSelector.setPoint = _probePos;

			break;

		case 2:


			if(_pidSelector.measurement <= (_squarePosA + tolerance)
					&& _pidSelector.measurement
							>= (_squarePosA - tolerance)){
				_pidSelector.setPoint = _squarePosB;
			}
			_indexer = 3;
			break;
		case 3:
			if(_pidSelector.measurement <= (_squarePosB + tolerance) && _pidSelector.measurement >= (_squarePosB - tolerance)){
				_pidSelector.setPoint = _squarePosA;


			}
			_indexer = 2;
			break;
		default:
			_pidSelector.setPoint = _cleanerPos;


	}


}


void Convertor::moveSelector(int16_t pwm){


	if(pwm > 0){

		this->actuatorControl(pwm, this->_timers->get_selector_Timer(), TIM_CHANNEL_4,  TIM_CHANNEL_3);
	}else if(pwm < 0){

		this->actuatorControl(pwm, this->_timers->get_selector_Timer(), TIM_CHANNEL_4,  TIM_CHANNEL_3 );

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

void Convertor::updateSelectorPosition(void){


		_pidSelector.measurement = this->get_selector_position();
		PIDControllerUpdate(&_pidSelector);
		_selectorPWM =(int16_t) _pidSelector.out;
		this->moveSelector(_selectorPWM);




}

void Convertor::updateFluidPosition(void){
    uint16_t long_press = 5000;
    static uint16_t timer = 0;  // Declare timer as static to preserve its value


    if(_sbus->shoulder_button() &&( _fluidPosition < 950)){

    	_fluidPosition += _fluidAmount;
    }

    if(_sbus->shoulder_button_long()){

        if(++timer <= long_press){

        }

        if(timer == 300){
        	_fluidPosition = this->get_fluidPosition();
        }

        if(timer == 599){
			_fluidPosition = 50;
        	timer = 0;

        }

    }else {
    	timer = 0;
    }



}

void Convertor::updateFluidMotor(void){

	uint8_t tolerance = 2;
	uint8_t biggerTolerance = 20;


	if((this->get_fluidPosition() <= (_fluidPosition + tolerance)) && (this->get_fluidPosition() >= (_fluidPosition - tolerance)) ){

		this->actuatorControl(0, this->_timers->get_fluidMotor_Timer(), TIM_CHANNEL_2, TIM_CHANNEL_1);

	}else if(this->get_fluidPosition() < (_fluidPosition )){

		this->actuatorControl(300, this->_timers->get_fluidMotor_Timer(), TIM_CHANNEL_2, TIM_CHANNEL_1);

	}else if(this->get_fluidPosition() > (_fluidPosition)){

		this->actuatorControl(-300, this->_timers->get_fluidMotor_Timer(), TIM_CHANNEL_2, TIM_CHANNEL_1);

	}


}

void Convertor::updateFluidAmount(void){


	if(this->_sbus->D_button() && (_fluidAmount < 100)){

		_fluidAmount += 5;
	}else if(_sbus->C_button() && (_fluidAmount > 0)){

		_fluidAmount -= 5;
	}

}

void Convertor::setAltitudeOffset(void){

	static uint8_t counter = 0;


	if(this->_sbus->home_button()){
		counter += 1;
		if (counter >= 2){
			counter = 0;
		}
	}

	switch(counter){

		case 0:
			_altimeter->set_offset(0);
			break;
		case 1:
			if(_altimeter->get_offset() == 0){

				_altimeter->set_offset(_altimeter->get_altitude());
			}
			break;
		case 2:
			_altimeter->set_offset(0);
			break;
	}

}

void Convertor::setSleepMode(void){

	static uint8_t counter = 0;


	if(this->_sbus->home_button()){
		_sleepToggle = !_sleepToggle;
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

int16_t Convertor::get_battery_voltage(void){

	uint16_t batteryInteger = ADC_1_Buffer[3];

	float supplyVoltage = 3.3;
	float adcRange = 4095; //because 12 bits
	float resistorTop = 96000.0; 	// not 100k ohm because 1% res and with this factor it's apparently 0.1V accurate
	float resistorBottom = 9500.0;
	float resistorTotal = resistorTop + resistorBottom;

	uint16_t batteryValue = (uint16_t) ((supplyVoltage * (float)batteryInteger * resistorTotal) / (adcRange * resistorBottom) * 10.0);
	return batteryValue;


}
int16_t Convertor::get_selector_position(void){

	return ADC_1_Buffer[0] / 4.095;
}

int16_t Convertor::get_fluidPosition(void){


	return ADC_1_Buffer[2] / 4.095;

}

int16_t Convertor::get_pushPosition(void){

	return ADC_1_Buffer[1]/ 4.095;
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

int16_t Convertor::get_fluidAmount(void){

	return _fluidAmount;
}

void Convertor::process(void){

	//Hier een case maken om de servos in "sleep mode" te zetten
	this->setSleepMode();
	if(_sleepToggle == 1){
		this->updateCleanerMotor();
		this->updateLED();
		this->updateFluidPosition();
		this->updateSelector();
		this->updateSelectorPosition();
		this->updatePushMotor();
		this->updateFluidMotor();
		this->getADC();
		this->updateFluidAmount();
		//this->setAltitudeOffset();

	}else if(_sleepToggle == 0){

		this->getADC();
		this->updateSelectorPosition();

	};


}


void Convertor::Error_Handler(void){

	for (uint8_t i = 0; i < 30; i++){		/* Toggle LED signal for error */
		HAL_GPIO_TogglePin(gled_pc14_GPIO_Port, gled_pc14_Pin); //signal led
		HAL_Delay(100);
		printf("Problem with convertor class \r\n");

	}

}

