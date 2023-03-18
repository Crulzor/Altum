#pragma once
#include "main.h"
#include "SBUS.h"
#include "MavlinkControl.h"
#include "Convertor.h"

class Debugger{


	private:
		SBUS *_sbus;
		Convertor *_convertor;
		MavlinkControl* _mavlink;




	public:
		Debugger(SBUS *sbus, MavlinkControl* mavlink, Convertor *convertor);
		void displayDebugInfo(void);

		void displaySBUS_channels(void);
		void displayMavlink_RAW(void);
		void displayMavlink_header(void);

};
