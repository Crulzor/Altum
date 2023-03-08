#pragma once
#include "main.h"
#include "SBUS.h"
#include "Convertor.h"

class Debugger{


	private:
		SBUS *_sbus;
		Convertor *_convertor;


	public:
		Debugger(SBUS *sbus, Convertor *convertor);
		void displayDebugInfo(void);

		void displaySBUS_channels(void);
		void displaySBUS_RAW(void);

};
