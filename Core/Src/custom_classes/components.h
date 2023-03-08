#pragma once
#include "../../Inc/main.h"
#include "handlers.h"


// include of cmsis_gcc.h is necessary to implement error handler
// misschien best om dit later met overerving te doen?


class Components{


	private:
		//Hier nog de handlers toevoegen die je nodig zult hebben
		  COMP_HandleTypeDef* hcomp1_ptr = &hcomp1;
		  COMP_HandleTypeDef* hcomp2_ptr = &hcomp2;
		  COMP_HandleTypeDef* hcomp4_ptr = &hcomp4;
		  COMP_HandleTypeDef* hcomp5_ptr = &hcomp5;
		  COMP_HandleTypeDef* hcomp7_ptr = &hcomp7;
		  DAC_HandleTypeDef* hdac1_ptr = &hdac1;
		  DAC_HandleTypeDef* hdac2_ptr = &hdac2;
		  DAC_HandleTypeDef* hdac3_ptr = &hdac3;
		  DAC_HandleTypeDef* hdac4_ptr = &hdac4;
		  OPAMP_HandleTypeDef* hopamp1_ptr = &hopamp1;
		  OPAMP_HandleTypeDef* hopamp2_ptr = &hopamp2;
		  OPAMP_HandleTypeDef* hopamp3_ptr = &hopamp3;
		  OPAMP_HandleTypeDef* hopamp4_ptr = &hopamp4;
		  OPAMP_HandleTypeDef* hopamp5_ptr = &hopamp5;

		  ADC_HandleTypeDef* hadc1_ptr = &hadc1;
		  ADC_HandleTypeDef* hadc2_ptr = &hadc2;
		  ADC_HandleTypeDef* hadc5_ptr = &hadc5;



	public:



	//Keeping everything public for easy access for now, will add getters and setters where necessary
		void Error_Handler(void);


		void init_Components(void);

		void init_ADC_2(void);
		void init_ADC_5(void);
		void init_COMP_1(void);
		void init_COMP_2(void);
		void init_COMP_4(void);
		void init_COMP_5(void);
		void init_COMP_7(void);
		void init_DAC_1(void);
		void init_DAC_2(void);
		void init_DAC_3(void);
		void init_DAC_4(void);
		void init_OA_1(void);
		void init_OA_2(void);
		void init_OA_3(void);
		void init_OA_4(void);
		void init_OA_5(void);
		void init_ADC_1(void);



		ADC_HandleTypeDef* get_ADC_1(void);
		ADC_HandleTypeDef* get_ADC_2(void);
		ADC_HandleTypeDef* get_ADC_5(void);






};
