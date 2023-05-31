#pragma once
#include <stdint.h>
#include "main.h"
#include "SBUS.h"
#include "../Initializer.h"
#include "../components.h"
#include "altimeter.h"
#include "../handlers.h"

extern "C"{

#include "../../Pid_Controller/pid_controller.h"

}


//Class to convert SBUS protocol to PWM values etc...

class Convertor{

#define DEBOUNCE_DELAY 100

	public:
		Convertor();
		Convertor(SBUS *sbus, Initializer *init, Components *components, Altimeter* altimeter);

		//runs at standard at 64k hz
		void actuatorControl(int16_t pwm_input, TIM_HandleTypeDef *tim, uint32_t channelTimPlus, uint32_t channelTimMin);


		void getADC(void);
		uint16_t getADC_NO_DMA(ADC_HandleTypeDef *hadc, uint32_t Channel);

		void moveSelector(int16_t pwm);
		void updateSelector(void);
		void updateLED(void);
		void updatePushMotor(void);
		void updateFluidMotor(void);
		void updateCleanerMotor(void);
		void stopCleanerMotor(void);
		void updateFluidAmount(void);
		void setAltitudeOffset(void);
		void makeSquare(void);
		void updateSelectorPosition(void);
		void resetFluidPosition(void);
		void updateFluidPosition(void);
		void setSleepMode(void);

		void testSelector(void);
		void ledOFF(void);

		void Error_Handler(void);

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
		int16_t get_fluidAmount(void);

		//Getter for the battery voltage
		int16_t get_battery_voltage(void);



		void process(void);
		bool _position = 0;


		//public buffer for now for debugging purposes.

		uint32_t ADC_1_Buffer[4] = {0};
		uint32_t ADC_2_Buffer[2] = {0};
		uint32_t ADC_5_Buffer[2] = {0};

		//for debugging purposes
		//bool _trigger = 0;
		uint16_t _fluidPosition = 120;
		uint16_t _fluidAmount = 20;

	private:
		//SBUS and configuration objects to fetch necessary handlers
		SBUS *_sbus;
		Initializer *_timers;
		Components *_components;
		Altimeter* _altimeter;
		//PIDController object, .Kp, Ki & Kd influence the "agressiveness" and overshoot of the controller
		PIDController _pidSelector = {

				.Kp = 40.0f,
				.Ki = 100.0f,
				.Kd = 5.0f,
				.tau = 0.5f,
				.limMin = -1000.0f,
				.limMax = 1000.0f,
				.limMinInt = -500.0f,
				.limMaxInt = 500.0f,
				.T = 0.005f,
				.accuracy = 1.0f
		};

		//private variables
		bool _sleepToggle = 0;
		bool _fluidToggler = 0;
		bool _debounceTrigger = 1;
		uint8_t _indexer = 0;
		uint16_t _timer = 0;
		int16_t _ledPWM = 0;
		int16_t _cleanerMotorPWM = 0;
		int16_t _selectorPWM = 0;
		int16_t _pushMotorPWM = 0;
		int16_t _fluidPWM = 0;
		float _squarePosA = 110.0f;
		float _squarePosB = 200.0f;
		float _cleanerPos = 110.0f;
		float _probePos = 980.0f;







};
