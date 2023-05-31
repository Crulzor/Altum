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

		// NEEDS TO BE A VALUE BETWEEN 100 & 950
		void setSyringeStart(int start);

		//This function contains the sequence in which the functions of the subclasses are called. The order IS important
		void update(void);

		//some getters
		SBUS getSbus(void);
		MavlinkControl getMavlink(void);
		Altimeter getAltimeter(void);

		Altimeter _altimeter;
		MavlinkControl _mavlink;
		Convertor _convertor;
		SBUS _sbus;









};
