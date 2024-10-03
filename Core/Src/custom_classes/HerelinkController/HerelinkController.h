#pragma once
#include "MavlinkControl.h"
#include "SBUS.h"
#include "altimeter.h"
#include "Convertor.h"
#include "../Initializer.h"
#include "../components.h"


class HerelinkController{



	public:

		//Default Constructor
		HerelinkController();
		HerelinkController(UART_HandleTypeDef *huart_sbus, UART_HandleTypeDef *huart_mavlink, Initializer *init, Components *components);

		//functions for setting the motor speed & the square cleaner size
		void setMotorSpeed(int speed);
		void setSquareSize(int size);
		void setLedIntensity(int intensity);

		//This function contains the sequence in which the functions of the subclasses are called. 
		//The order IS important keep it as is or work with threads in the future. 
		void update(void);

		//Altimeter object contains the implementation of the MPL3115A2
		Altimeter _altimeter;

		//Mavlink object that contains the implementation for receiving/sending Mavlink messages
		MavlinkControl _mavlink;

		//SBUS object that contains the implementation for the SBUS protocol
		SBUS _sbus;
		
		//This object is responsible for converting the raw values of the other objects and/or ADC's 
		//To useful signals/values
		Convertor _convertor;

		//some getters for the objects 
		SBUS getSbus(void);
		MavlinkControl getMavlink(void);
		Altimeter getAltimeter(void);



		//THESE VALUES CAN BE CHANGED BY THE END USER IF HE WANTS TO 

		int _maxRPM = 100; //This is a percentage, keep it between 1-100
		float _squareSize = 200.0f;	//end point of the square. Maximum value should be 400 
		int _ledIntensity = 1000;	//SHOULD BE SOMEWHERE BETWEEN 750 AND 1000!!!


};
