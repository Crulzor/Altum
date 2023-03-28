#pragma once
#include "main.h"
#include "SBUS.h"
#include "MavlinkControl.h"
#include "Convertor.h"
#include "altimeter.h"

class Debugger{


	private:
		SBUS *_sbus;
		Convertor *_convertor;
		MavlinkControl* _mavlink;
		Altimeter* _altimeter;



	public:
		Debugger(SBUS *sbus, MavlinkControl* mavlink, Convertor *convertor, Altimeter* altimeter);
		void displayDebugInfo(void);

		void displaySBUS_channels(void);
		void displayMavlink_RAW(void);
		void displayMavlink_header(void);
		void displayAltitude(void);

};
