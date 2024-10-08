#pragma once
#include <stdint.h>
#include "main.h"
#include "SBUS.h"
#include "../Initializer.h"
#include "../components.h"
#include "altimeter.h"
#include "../handlers.h"

//PIDCONTROLLER IS WRITTEN IN STANDARD C SO EXTERN KEYWORD IS NECESSARY  
extern "C"{

#include "../../Pid_Controller/pid_controller.h"

}


//Class to convert SBUS protocol to PWM values etc...

class Convertor{

#define DEBOUNCE_DELAY 100

	public:
		//Default & implemnted constructor. Takes pointers to the objects made in the HerelinkController class
		Convertor();
		Convertor(SBUS *sbus, Initializer *init, Components *components, Altimeter* altimeter);

		//runs standard at 64k hz. This is the low-level function that controls the actuator movements
		void actuatorControl(int16_t pwm_input, TIM_HandleTypeDef *tim, uint32_t channelTimPlus, uint32_t channelTimMin);

		//Function that uses DMA multimode scanning to read values from ADC 2 & 3. 
		void getADC(void);

		//Function that manually fetches values from an ADC channel. Used for ADC1 because multimode scanning didn't
		//work on more than 2 ADC channels
		uint16_t getADC_NO_DMA(ADC_HandleTypeDef *hadc, uint32_t Channel);


		//Functions for controlling the selector. The first one is low-level and uses the HAL-macro
		//The second one implements a PIDController and uses the low-level function to move it to the
		//desired position. The third one is the one that "listens" to the user input and tells the 
		//PIDController what position it should move to. 
		void moveSelector(int16_t pwm);
		void updateSelectorPosition(void);
		void updateSelector(void);

		//functions that control the led, the push motor and the scrubber(cleaner) motor. 
		void updateLED(void);
		void ledOFF(void);
		void updatePushMotor(void);
		void updateCleanerMotor(void);
		void updateCleanerMotor(int16_t PWM);
		void stopCleanerMotor(void);
		
		//moves the selector motor up/down to make a small square
		void makeSquare(void);

		//functions for controlling the fluid piston and set the amount of fluid that will be applied. 
		//Implementation kinda works like the PIDController, but without a PIDcontroller :p 
		//main loop continuously checks fluidposition and moves the piston towards that position. 
		//Position increments are determined by the fluidAmount var. 
		void updateFluidMotor(void);
		void resetFluidPosition(void);
		void updateFluidPosition(void);
		void updateFluidAmount(void);
		void updateFluidMotorJogWheel(void);

		//This function switches between two "loops" to disable the sbus controls. 
		void setSleepMode(void);
	
		//Sets/resets the offset of the altimeter
		void setAltitudeOffset(void);


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
		int16_t get_sleepToggle(void);

		//Getter for the battery voltage
		uint16_t get_battery_voltage(void);


		//the main loop of the convertor class
		void process(void);
		bool _position = 0;

		void Error_Handler(void);

		//public buffer for now for debugging purposes.
		uint32_t ADC_1_Buffer[4] = {0};
		uint32_t ADC_2_Buffer[2] = {0};
		uint32_t ADC_5_Buffer[2] = {0};


		float _fluidPosition;
		uint16_t _fluidAmount = 20;

		uint16_t _max_led_intensity = 1000;
		int16_t _maxPercentage = 100;
		float _squarePosB = 200.0f;


		bool _sleepToggle = 0;



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
		float _cleanerPos = 110.0f;
		float _probePos = 980.0f;







};
