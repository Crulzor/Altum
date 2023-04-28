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

		void setMotorSpeed(int speed);
		void update(void);

		//some getters
		SBUS getSbus(void);
		MavlinkControl getMavlink(void);
		Altimeter getAltimeter(void);

		Altimeter _altimeter;
		SBUS _sbus;
		MavlinkControl _mavlink;
		Convertor _convertor;



	private:






};
