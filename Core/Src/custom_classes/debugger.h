#pragma once
#include "main.h"
#include "HerelinkController/HerelinkController.h"
#include "HerelinkController/SBUS.h"
#include "HerelinkController/MavlinkControl.h"
#include "HerelinkController/Convertor.h"
#include "HerelinkController/altimeter.h"

class Debugger{


	private:

		HerelinkController *_controller;



	public:
		Debugger(HerelinkController *controller);
		void displayDebugInfo(void);

		void displaySBUS_channels(void);
		void displayMavlink_RAW(void);
		void displayMavlink_header(void);
		void displayAltitude(void);
		void debugADC(void);

};
