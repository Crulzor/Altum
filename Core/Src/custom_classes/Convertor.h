#pragma once
#include <stdint.h>
#include "main.h"
#include "SBUS.h"
#include "Initializer.h"
#include "components.h"
#include "handlers.h"

//Class to convert SBUS protocol to PWM values etc...

class Convertor{

#define DEBOUNCE_DELAY 100

	public:
		Convertor(SBUS *sbus, Initializer *init, Components *components);

		//runs at standard at 64k hz
		void actuatorControl(int16_t pwm_input, TIM_HandleTypeDef *tim, uint32_t channelTimPlus, uint32_t channelTimMin);


		void getADC(void);

		void updateSelector(void);
		void updateLED(void);
		void updatePushMotor(void);
		void updateFluidMotor(void);
		void updateCleanerMotor(void);

		void testSelector(void);
		void ledOFF(void);


		//Getters...getters everywhere....

		//Getters for the pwm values
		int16_t get_LEDPWM(void);
		int16_t get_pushPWM(void);
		int16_t get_fluidPWM(void);
		int16_t get_selectorPWM(void);
		int16_t get_cleanerMotorPWM(void);

		//Getters for the pot positions (ADC)
		int16_t get_selector_position(void);
		int16_t get_pushPosition(void);
		int16_t get_fluidPosition(void);

		//Getters for the shunt(current) (ADC)
		int16_t get_fluidCurrent(void);
		int16_t get_LEDCurrent(void);
		int16_t get_pushCurrent(void);
		int16_t get_selectorCurrent(void);



		void process(void);



		//public buffer for now for debugging purposes.

		uint32_t ADC_1_Buffer[3] = {0};
		uint32_t ADC_2_Buffer[2] = {0};
		uint32_t ADC_5_Buffer[2] = {0};



	private:
		//SBUS and configuration objects to fetch necessary handlers
		SBUS *_sbus;
		Initializer *_timers;
		Components *_components;

		//private variables
		int16_t _ledPWM = 0;
		int16_t _cleanerMotorPWM = 0;
		int16_t _selectorPWM = 0;
		int16_t _pushMotorPWM = 0;
		int16_t _fluidPWM = 0;

		int8_t _fluidAmount = 0;






};
