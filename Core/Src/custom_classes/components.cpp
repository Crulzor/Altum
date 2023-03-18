#include "components.h"




void Components::Error_Handler(void){

	//separate error handler for the components
	//adapted the handler to show the error on the status led then exit and try again.

	for (uint8_t i = 0; i < 30; i++){		/* Toggle LED signal for error */
		HAL_GPIO_TogglePin(gled_pc14_GPIO_Port, gled_pc14_Pin); //signal led
		HAL_Delay(200);
		printf("Components error handler !! \r\n");
	}


}



void Components::init_Components(void){

	//DEZE VOLGORDE VAN INITIALISEREN AANHOUDEN. ADC/OPAMPS ZIJN SOMS AAN ELKAAR GELINKT

	this->init_COMP_1();
	this->init_COMP_2();
	this->init_COMP_4();
	this->init_COMP_5();
	this->init_COMP_7();
	this->init_ADC_1();
	this->init_DAC_1();
	this->init_DAC_2();
	this->init_DAC_3();
	this->init_DAC_4();
	this->init_OA_1();
	this->init_OA_2();
	this->init_ADC_2();
	this->init_OA_3();
	this->init_OA_4();
	this->init_ADC_5();
	this->init_OA_5();


		HAL_OPAMP_Start(hopamp1_ptr);
		HAL_OPAMP_Start(hopamp2_ptr);
		HAL_OPAMP_Start(hopamp3_ptr);
		HAL_OPAMP_Start(hopamp4_ptr);
		HAL_OPAMP_Start(hopamp5_ptr);

		HAL_OPAMP_SelfCalibrate(hopamp1_ptr);
		HAL_OPAMP_SelfCalibrate(hopamp2_ptr);
		HAL_OPAMP_SelfCalibrate(hopamp3_ptr);
		HAL_OPAMP_SelfCalibrate(hopamp4_ptr);
		HAL_OPAMP_SelfCalibrate(hopamp5_ptr);
		HAL_ADCEx_Calibration_Start(hadc1_ptr, ADC_SINGLE_ENDED);
		HAL_ADCEx_Calibration_Start(hadc2_ptr, ADC_SINGLE_ENDED);
		HAL_ADCEx_Calibration_Start(hadc5_ptr, ADC_SINGLE_ENDED);

		//HAL_ADCEx_Calibration_Start(hadc2_ptr, ADC_SINGLE_ENDED);




}


void Components::init_ADC_1(void){

		  ADC_MultiModeTypeDef multimode = {0};
		  ADC_ChannelConfTypeDef sConfig = {0};

		  /** Common config
		  */
		  hadc1_ptr->Instance = ADC1;
		  hadc1_ptr->Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
		  hadc1_ptr->Init.Resolution = ADC_RESOLUTION_12B;
		  hadc1_ptr->Init.DataAlign = ADC_DATAALIGN_RIGHT;
		  hadc1_ptr->Init.GainCompensation = 0;
		  hadc1_ptr->Init.ScanConvMode = ADC_SCAN_ENABLE;		// Enable scanning mode
		  hadc1_ptr->Init.EOCSelection = ADC_EOC_SEQ_CONV;		// Use end of sequence conversion
		  hadc1_ptr->Init.LowPowerAutoWait = DISABLE;
		  hadc1_ptr->Init.ContinuousConvMode = DISABLE;
		  hadc1_ptr->Init.NbrOfConversion = 3;
		  hadc1_ptr->Init.DiscontinuousConvMode = DISABLE;
		  hadc1_ptr->Init.ExternalTrigConv = ADC_SOFTWARE_START;
		  hadc1_ptr->Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
		  hadc1_ptr->Init.DMAContinuousRequests = DISABLE;
		  hadc1_ptr->Init.Overrun = ADC_OVR_DATA_PRESERVED;
		  hadc1_ptr->Init.OversamplingMode = ENABLE;
		  hadc1_ptr->Init.Oversampling.Ratio = ADC_OVERSAMPLING_RATIO_256;
		  hadc1_ptr->Init.Oversampling.RightBitShift = ADC_RIGHTBITSHIFT_8;
		  hadc1_ptr->Init.Oversampling.TriggeredMode = ADC_TRIGGEREDMODE_SINGLE_TRIGGER;
		  hadc1_ptr->Init.Oversampling.OversamplingStopReset = ADC_REGOVERSAMPLING_CONTINUED_MODE;
		  if (HAL_ADC_Init(hadc1_ptr) != HAL_OK)
		  {
		    this->Error_Handler();
		    printf("ADC1 init failed");
		  }

		  /** Configure the ADC multi-mode
		  */
		  multimode.Mode = ADC_MODE_INDEPENDENT;
		  if (HAL_ADCEx_MultiModeConfigChannel(hadc1_ptr, &multimode) != HAL_OK)
		  {
		    this->Error_Handler();
		    printf("ADC1 HAL_ADCEx_MultiModeConfigChannel failed");

		  }

		  /** Configure Regular Channel
		  */

		  //SAMPLING TIME ORIGINEEL OP 47 CYCLES. MISSCHIEN NOG AANPASSEN


		  //KANAAL VOOR DE SELECTOR POT
		  sConfig.Channel = ADC_CHANNEL_11;
		  sConfig.Rank = ADC_REGULAR_RANK_1;
		  sConfig.SamplingTime = ADC_SAMPLETIME_47CYCLES_5;
		  sConfig.SingleDiff = ADC_SINGLE_ENDED;
		  sConfig.OffsetNumber = ADC_OFFSET_NONE;
		  sConfig.Offset = 0;
		  if (HAL_ADC_ConfigChannel(hadc1_ptr, &sConfig) != HAL_OK)
		  {
		    this->Error_Handler();
		    printf("ADC 1 HAL_ADC_ConfigChannel NOT ok");
		  }


		  //KANAAL VOOR DE PUSH POT
		  sConfig.Channel = ADC_CHANNEL_14;
		  sConfig.Rank = ADC_REGULAR_RANK_3;
		  sConfig.SamplingTime = ADC_SAMPLETIME_47CYCLES_5;
		  sConfig.SingleDiff = ADC_SINGLE_ENDED;
		  sConfig.OffsetNumber = ADC_OFFSET_NONE;
		  sConfig.Offset = 0;
		  if (HAL_ADC_ConfigChannel(hadc1_ptr, &sConfig) != HAL_OK)
		  {
		    this->Error_Handler();
		    printf("ADC 1 HAL_ADC_ConfigChannel Vopamp1 NOT ok");
		  }

		  //KANAAL VOOR DE FLUID POT
		  sConfig.Channel = ADC_CHANNEL_12;
		  sConfig.Rank = ADC_REGULAR_RANK_4;
		  sConfig.SamplingTime = ADC_SAMPLETIME_47CYCLES_5;
		  sConfig.SingleDiff = ADC_SINGLE_ENDED;
		  sConfig.OffsetNumber = ADC_OFFSET_NONE;
		  sConfig.Offset = 0;
		  if (HAL_ADC_ConfigChannel(hadc1_ptr, &sConfig) != HAL_OK)
		  {
		    this->Error_Handler();
		    printf("ADC 1 HAL_ADC_ConfigChannel Vopamp1 NOT ok");
		  }


}

void Components::init_ADC_2(void){


	  ADC_ChannelConfTypeDef sConfig = {0};


	  /** Common config
	  */
	  hadc2_ptr->Instance = ADC2;
	  hadc2_ptr->Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
	  hadc2_ptr->Init.Resolution = ADC_RESOLUTION_12B;
	  hadc2_ptr->Init.DataAlign = ADC_DATAALIGN_RIGHT;
	  hadc2_ptr->Init.GainCompensation = 0;
	  hadc2_ptr->Init.ScanConvMode = ADC_SCAN_ENABLE;
	  hadc2_ptr->Init.EOCSelection = ADC_EOC_SEQ_CONV;
	  hadc2_ptr->Init.LowPowerAutoWait = DISABLE;
	  hadc2_ptr->Init.ContinuousConvMode = DISABLE;
	  hadc2_ptr->Init.NbrOfConversion = 2;
	  hadc2_ptr->Init.DiscontinuousConvMode = DISABLE;
	  hadc2_ptr->Init.ExternalTrigConv = ADC_SOFTWARE_START;
	  hadc2_ptr->Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	  hadc2_ptr->Init.DMAContinuousRequests = DISABLE;
	  hadc2_ptr->Init.Overrun = ADC_OVR_DATA_PRESERVED;
	  hadc2_ptr->Init.OversamplingMode = ENABLE;
	  hadc2_ptr->Init.Oversampling.Ratio = ADC_OVERSAMPLING_RATIO_128;
	  hadc2_ptr->Init.Oversampling.RightBitShift = ADC_RIGHTBITSHIFT_7;
	  hadc2_ptr->Init.Oversampling.TriggeredMode = ADC_TRIGGEREDMODE_SINGLE_TRIGGER;
	  hadc2_ptr->Init.Oversampling.OversamplingStopReset = ADC_REGOVERSAMPLING_CONTINUED_MODE;
	  if (HAL_ADC_Init(hadc2_ptr) != HAL_OK)
	  {
	    this->Error_Handler();
	    printf("problem with ADC2 init \r\n");
	  }

	  /** Configure Regular Channel
	  */

	  //led current

	  sConfig.Channel = ADC_CHANNEL_VOPAMP2;
	  sConfig.Rank = ADC_REGULAR_RANK_1;
	  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
	  sConfig.SingleDiff = ADC_SINGLE_ENDED;
	  sConfig.OffsetNumber = ADC_OFFSET_NONE;
	  sConfig.Offset = 0;
	  if (HAL_ADC_ConfigChannel(hadc2_ptr, &sConfig) != HAL_OK)
	  {
	    this->Error_Handler();
	    printf("problem with ADC2 channel one init \r\n");

	  }

	  //fluid current
	  sConfig.Channel = ADC_CHANNEL_VOPAMP3_ADC2;
	  sConfig.Rank = ADC_REGULAR_RANK_2;
	  sConfig.SamplingTime = ADC_SAMPLETIME_47CYCLES_5;
	  sConfig.SingleDiff = ADC_SINGLE_ENDED;
	  sConfig.OffsetNumber = ADC_OFFSET_NONE;
	  if(HAL_ADC_ConfigChannel(hadc2_ptr, &sConfig) != HAL_OK){

		  this->Error_Handler();
		  printf("problem with ADC2 channel two init \r\n");

	  }


}

void Components::init_ADC_5(void){

	  ADC_ChannelConfTypeDef sConfig = {0};


	  /** Common config
	  */
	  hadc5_ptr->Instance = ADC5;
	  hadc5_ptr->Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
	  hadc5_ptr->Init.Resolution = ADC_RESOLUTION_12B;
	  hadc5_ptr->Init.DataAlign = ADC_DATAALIGN_RIGHT;
	  hadc5_ptr->Init.GainCompensation = 0;
	  hadc5_ptr->Init.ScanConvMode = ADC_SCAN_ENABLE;
	  hadc5_ptr->Init.EOCSelection = ADC_EOC_SEQ_CONV;
	  hadc5_ptr->Init.LowPowerAutoWait = DISABLE;
	  hadc5_ptr->Init.ContinuousConvMode = DISABLE;
	  hadc5_ptr->Init.NbrOfConversion = 2;
	  hadc5_ptr->Init.DiscontinuousConvMode = DISABLE;
	  hadc5_ptr->Init.ExternalTrigConv = ADC_SOFTWARE_START;
	  hadc5_ptr->Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	  hadc5_ptr->Init.DMAContinuousRequests = DISABLE;
	  hadc5_ptr->Init.Overrun = ADC_OVR_DATA_PRESERVED;
	  hadc5_ptr->Init.OversamplingMode = ENABLE;
	  hadc5_ptr->Init.Oversampling.Ratio = ADC_OVERSAMPLING_RATIO_128;
	  hadc5_ptr->Init.Oversampling.RightBitShift = ADC_RIGHTBITSHIFT_7;
	  hadc5_ptr->Init.Oversampling.TriggeredMode = ADC_TRIGGEREDMODE_SINGLE_TRIGGER;
	  hadc5_ptr->Init.Oversampling.OversamplingStopReset = ADC_REGOVERSAMPLING_CONTINUED_MODE;
	  if (HAL_ADC_Init(hadc5_ptr) != HAL_OK)
	  {
	    this->Error_Handler();
	    printf("problem with ADC5 init");

	  }

	  /** Configure Regular Channel
	  */
	  //push motor shunt channel (current)
	  sConfig.Channel = ADC_CHANNEL_VOPAMP4;
	  sConfig.Rank = ADC_REGULAR_RANK_1;
	  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
	  sConfig.SingleDiff = ADC_SINGLE_ENDED;
	  sConfig.OffsetNumber = ADC_OFFSET_NONE;
	  sConfig.Offset = 0;
	  if (HAL_ADC_ConfigChannel(hadc5_ptr, &sConfig) != HAL_OK)
	  {
	    this->Error_Handler();
	    printf("problem with ADC5 config channel");

	  }

	  //selector motor (current)
	  sConfig.Channel = ADC_CHANNEL_VOPAMP5;
	  sConfig.Rank = ADC_REGULAR_RANK_2;
	  sConfig.SamplingTime = ADC_SAMPLETIME_47CYCLES_5;
	  sConfig.SingleDiff = ADC_SINGLE_ENDED;
	  sConfig.OffsetNumber = ADC_OFFSET_NONE;
	  if (HAL_ADC_ConfigChannel(hadc5_ptr, &sConfig) != HAL_OK)
	  {
	    this->Error_Handler();
	    printf("problem with ADC5 config channel");

	  }




}

void Components::init_COMP_1(void){

	  hcomp1_ptr->Instance = COMP1;
	  hcomp1_ptr->Init.InputPlus = COMP_INPUT_PLUS_IO1;
	  hcomp1_ptr->Init.InputMinus = COMP_INPUT_MINUS_DAC1_CH1;
	  hcomp1_ptr->Init.OutputPol = COMP_OUTPUTPOL_NONINVERTED;
	  hcomp1_ptr->Init.Hysteresis = COMP_HYSTERESIS_NONE;
	  hcomp1_ptr->Init.BlankingSrce = COMP_BLANKINGSRC_NONE;
	  hcomp1_ptr->Init.TriggerMode = COMP_TRIGGERMODE_NONE;
	  if (HAL_COMP_Init(hcomp1_ptr) != HAL_OK)
	  {
	    this->Error_Handler();
	    printf("problem with COMP1 init");

	  }
}

void Components::init_COMP_2(void){

	    hcomp2_ptr->Instance = COMP2;
	    hcomp2_ptr->Init.InputPlus = COMP_INPUT_PLUS_IO1;
	    hcomp2_ptr->Init.InputMinus = COMP_INPUT_MINUS_DAC1_CH2;
	    hcomp2_ptr->Init.OutputPol = COMP_OUTPUTPOL_NONINVERTED;
	    hcomp2_ptr->Init.Hysteresis = COMP_HYSTERESIS_NONE;
	    hcomp2_ptr->Init.BlankingSrce = COMP_BLANKINGSRC_NONE;
	    hcomp2_ptr->Init.TriggerMode = COMP_TRIGGERMODE_NONE;
	    if (HAL_COMP_Init(hcomp2_ptr) != HAL_OK)
	    {
	      this->Error_Handler();
		  printf("problem with COMP2 init");

	    }




}

void Components::init_COMP_4(void){

    hcomp4_ptr->Instance = COMP4;
    hcomp4_ptr->Init.InputPlus = COMP_INPUT_PLUS_IO1;
    hcomp4_ptr->Init.InputMinus = COMP_INPUT_MINUS_DAC3_CH2;
    hcomp4_ptr->Init.OutputPol = COMP_HYSTERESIS_NONE;
    hcomp4_ptr->Init.BlankingSrce = COMP_BLANKINGSRC_NONE;
    hcomp4_ptr->Init.TriggerMode = COMP_TRIGGERMODE_NONE;
    if (HAL_COMP_Init(hcomp4_ptr) != HAL_OK)
    {
        this->Error_Handler();
        printf("problem with COMP4 init");
    }
}

void Components::init_COMP_5(void){

	  hcomp5_ptr->Instance = COMP5;
	  hcomp5_ptr->Init.InputPlus = COMP_INPUT_PLUS_IO1;
	  hcomp5_ptr->Init.InputMinus = COMP_INPUT_MINUS_DAC4_CH1;
	  hcomp5_ptr->Init.OutputPol = COMP_OUTPUTPOL_NONINVERTED;
	  hcomp5_ptr->Init.Hysteresis = COMP_HYSTERESIS_NONE;
	  hcomp5_ptr->Init.BlankingSrce = COMP_BLANKINGSRC_NONE;
	  hcomp5_ptr->Init.TriggerMode = COMP_TRIGGERMODE_NONE;
	  if (HAL_COMP_Init(hcomp5_ptr) != HAL_OK)
	  {
		this->Error_Handler();
	    printf("problem with COMP5 init");

	  }

}

void Components::init_COMP_7(void){


	   hcomp7_ptr->Instance = COMP7;
	   hcomp7_ptr->Init.InputPlus = COMP_INPUT_PLUS_IO1;
	   hcomp7_ptr->Init.InputMinus = COMP_INPUT_MINUS_DAC2_CH1;
	   hcomp7_ptr->Init.OutputPol = COMP_OUTPUTPOL_NONINVERTED;
	   hcomp7_ptr->Init.Hysteresis = COMP_HYSTERESIS_NONE;
	   hcomp7_ptr->Init.BlankingSrce = COMP_BLANKINGSRC_NONE;
	   hcomp7_ptr->Init.TriggerMode = COMP_TRIGGERMODE_NONE;
	   if (HAL_COMP_Init(hcomp7_ptr) != HAL_OK)
	   {
	     this->Error_Handler();
	   }



}


void Components::init_DAC_1(void){

    DAC_ChannelConfTypeDef sConfig = {0};
    /** DAC Initialization
    */
    hdac1_ptr->Instance = DAC1;
    if (HAL_DAC_Init(hdac1_ptr) != HAL_OK)
    {
        this->Error_Handler();
        printf("problem with DAC1 init");

    }

    /** DAC channel OUT1 config
    */
    sConfig.DAC_HighFrequency = DAC_HIGH_FREQUENCY_INTERFACE_MODE_AUTOMATIC;
    sConfig.DAC_DMADoubleDataMode = DISABLE;
    sConfig.DAC_SignedFormat = DISABLE;
    sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
    sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
    sConfig.DAC_Trigger2 = DAC_TRIGGER_NONE;
    sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_DISABLE;
    sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_INTERNAL;
    sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
    if (HAL_DAC_ConfigChannel(hdac1_ptr, &sConfig, DAC_CHANNEL_1) != HAL_OK)
    {
        this->Error_Handler();
        printf("problem with DAC1 init");

    }

    /** DAC channel OUT2 config
    */
    if (HAL_DAC_ConfigChannel(hdac1_ptr, &sConfig, DAC_CHANNEL_2) != HAL_OK)
    {
        this->Error_Handler();
        printf("problem with DAC1 init");

    }

}

void Components::init_DAC_2(void){

	  DAC_ChannelConfTypeDef sConfig = {0};

	  /** DAC Initialization
	  */
	  hdac2_ptr->Instance = DAC2;
	  if (HAL_DAC_Init(hdac2_ptr) != HAL_OK)
	  {
	    this->Error_Handler();
	    printf("problem with DAC2 init");

	  }

	  /** DAC channel OUT1 config
	  */
	  sConfig.DAC_HighFrequency = DAC_HIGH_FREQUENCY_INTERFACE_MODE_AUTOMATIC;
	  sConfig.DAC_DMADoubleDataMode = DISABLE;
	  sConfig.DAC_SignedFormat = DISABLE;
	  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
	  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
	  sConfig.DAC_Trigger2 = DAC_TRIGGER_NONE;
	  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_DISABLE;
	  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_INTERNAL;
	  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
	  if (HAL_DAC_ConfigChannel(&hdac2, &sConfig, DAC_CHANNEL_1) != HAL_OK)
	  {
	    this->Error_Handler();
	    printf("problem with DAC2 init");

	  }


}

void Components::init_DAC_3(void){

	  DAC_ChannelConfTypeDef sConfig = {0};


	  /** DAC Initialization
	  */
	  hdac3_ptr->Instance = DAC3;

	  if (HAL_DAC_Init(hdac3_ptr) != HAL_OK)
	  {
	    this->Error_Handler();
	    printf("problem with DAC3 init");

	  }

	  /** DAC channel OUT2 config
	  */
	  sConfig.DAC_HighFrequency = DAC_HIGH_FREQUENCY_INTERFACE_MODE_AUTOMATIC;
	  sConfig.DAC_DMADoubleDataMode = DISABLE;
	  sConfig.DAC_SignedFormat = DISABLE;
	  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
	  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
	  sConfig.DAC_Trigger2 = DAC_TRIGGER_NONE;
	  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_DISABLE;
	  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_INTERNAL;
	  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
	  if (HAL_DAC_ConfigChannel(hdac3_ptr, &sConfig, DAC_CHANNEL_2) != HAL_OK)
	  {
	    this->Error_Handler();
	    printf("problem with DAC3 init");

	  }



}

void Components::init_DAC_4(void){


	  DAC_ChannelConfTypeDef sConfig = {0};

	  /** DAC Initialization
	  */
	  hdac4_ptr->Instance = DAC4;
	  if (HAL_DAC_Init(hdac4_ptr) != HAL_OK)
	  {
	    this->Error_Handler();
	    printf("problem with DAC4 init");

	  }

	  /** DAC channel OUT1 config
	  */
	  sConfig.DAC_HighFrequency = DAC_HIGH_FREQUENCY_INTERFACE_MODE_AUTOMATIC;
	  sConfig.DAC_DMADoubleDataMode = DISABLE;
	  sConfig.DAC_SignedFormat = DISABLE;
	  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
	  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
	  sConfig.DAC_Trigger2 = DAC_TRIGGER_NONE;
	  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_DISABLE;
	  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_INTERNAL;
	  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
	  if (HAL_DAC_ConfigChannel(hdac4_ptr, &sConfig, DAC_CHANNEL_1) != HAL_OK)
	  {
	    this->Error_Handler();
	    printf("problem with DAC4 init");

	  }


}

void Components::init_OA_1(void){

	  hopamp1_ptr->Instance = OPAMP1;
	  hopamp1_ptr->Init.PowerMode = OPAMP_POWERMODE_NORMALSPEED;
	  hopamp1_ptr->Init.Mode = OPAMP_PGA_MODE;
	  hopamp1_ptr->Init.NonInvertingInput = OPAMP_NONINVERTINGINPUT_IO0;
	  hopamp1_ptr->Init.InternalOutput = ENABLE;
	  hopamp1_ptr->Init.TimerControlledMuxmode = OPAMP_TIMERCONTROLLEDMUXMODE_DISABLE;
	  hopamp1_ptr->Init.PgaConnect = OPAMP_PGA_CONNECT_INVERTINGINPUT_NO;
	  hopamp1_ptr->Init.PgaGain = OPAMP_PGA_GAIN_2_OR_MINUS_1;
	  hopamp1_ptr->Init.UserTrimming = OPAMP_TRIMMING_FACTORY;
	  if (HAL_OPAMP_Init(hopamp1_ptr) != HAL_OK)
	  {
	    this->Error_Handler();
	    printf("problem with OA1 init");
	  }


}

void Components::init_OA_2(void){

	    hopamp2_ptr->Instance = OPAMP2;
	    hopamp2_ptr->Init.PowerMode = OPAMP_POWERMODE_NORMALSPEED;
	    hopamp2_ptr->Init.Mode = OPAMP_PGA_MODE;
	    hopamp2_ptr->Init.NonInvertingInput = OPAMP_NONINVERTINGINPUT_IO0;
	    hopamp2_ptr->Init.InternalOutput = ENABLE;
	    hopamp2_ptr->Init.TimerControlledMuxmode = OPAMP_TIMERCONTROLLEDMUXMODE_DISABLE;
	    hopamp2_ptr->Init.PgaConnect = OPAMP_PGA_CONNECT_INVERTINGINPUT_NO;
	    hopamp2_ptr->Init.PgaGain = OPAMP_PGA_GAIN_16_OR_MINUS_15;
	    hopamp2_ptr->Init.UserTrimming = OPAMP_TRIMMING_FACTORY;
	    if (HAL_OPAMP_Init(hopamp2_ptr) != HAL_OK)
	    {
	      this->Error_Handler();
		    printf("problem with OA2 init");

	    }


}

void Components::init_OA_3(void){

	   hopamp3_ptr->Instance = OPAMP3;
	   hopamp3_ptr->Init.PowerMode = OPAMP_POWERMODE_NORMALSPEED;
	   hopamp3_ptr->Init.Mode = OPAMP_PGA_MODE;
	   hopamp3_ptr->Init.NonInvertingInput = OPAMP_NONINVERTINGINPUT_IO0;
	   hopamp3_ptr->Init.InternalOutput = ENABLE;
	   hopamp3_ptr->Init.TimerControlledMuxmode = OPAMP_TIMERCONTROLLEDMUXMODE_DISABLE;
	   hopamp3_ptr->Init.PgaConnect = OPAMP_PGA_CONNECT_INVERTINGINPUT_NO;
	   hopamp3_ptr->Init.PgaGain = OPAMP_PGA_GAIN_2_OR_MINUS_1;
	   hopamp3_ptr->Init.UserTrimming = OPAMP_TRIMMING_FACTORY;
	   if (HAL_OPAMP_Init(hopamp3_ptr) != HAL_OK)
	   {
	     this->Error_Handler();
		    printf("problem with OA3 init");

	   }

}

void Components::init_OA_4(void){

	  hopamp4_ptr->Instance = OPAMP4;
	  hopamp4_ptr->Init.PowerMode = OPAMP_POWERMODE_NORMALSPEED;
	  hopamp4_ptr->Init.Mode = OPAMP_PGA_MODE;
	  hopamp4_ptr->Init.NonInvertingInput = OPAMP_NONINVERTINGINPUT_IO0;
	  hopamp4_ptr->Init.InternalOutput = ENABLE;
	  hopamp4_ptr->Init.TimerControlledMuxmode = OPAMP_TIMERCONTROLLEDMUXMODE_DISABLE;
	  hopamp4_ptr->Init.PgaConnect = OPAMP_PGA_CONNECT_INVERTINGINPUT_NO;
	  hopamp4_ptr->Init.PgaGain = OPAMP_PGA_GAIN_16_OR_MINUS_15;
	  hopamp4_ptr->Init.UserTrimming = OPAMP_TRIMMING_FACTORY;
	  if (HAL_OPAMP_Init(hopamp4_ptr) != HAL_OK)
	  {
	    this->Error_Handler();
	    printf("problem with OA4 init");

	  }
}

void Components::init_OA_5(void){

	  hopamp5_ptr->Instance = OPAMP5;
	  hopamp5_ptr->Init.PowerMode = OPAMP_POWERMODE_NORMALSPEED;
	  hopamp5_ptr->Init.Mode = OPAMP_PGA_MODE;
	  hopamp5_ptr->Init.NonInvertingInput = OPAMP_NONINVERTINGINPUT_IO0;
	  hopamp5_ptr->Init.InternalOutput = ENABLE;
	  hopamp5_ptr->Init.TimerControlledMuxmode = OPAMP_TIMERCONTROLLEDMUXMODE_DISABLE;
	  hopamp5_ptr->Init.PgaConnect = OPAMP_PGA_CONNECT_INVERTINGINPUT_NO;
	  hopamp5_ptr->Init.PgaGain = OPAMP_PGA_GAIN_16_OR_MINUS_15;
	  hopamp5_ptr->Init.UserTrimming = OPAMP_TRIMMING_FACTORY;
	  if (HAL_OPAMP_Init(hopamp5_ptr) != HAL_OK)
	  {
	    this->Error_Handler();
	    printf("problem with OA5 init");

	  }

}

ADC_HandleTypeDef* Components::get_ADC_1(void){

	return hadc1_ptr;
}

ADC_HandleTypeDef* Components::get_ADC_2(void){

	return &hadc2;
}

ADC_HandleTypeDef* Components::get_ADC_5(void){

	return &hadc5;

}
