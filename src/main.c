/**
  ******************************************************************************
  * @file    main.c
  * @author  Ac6
  * @version V1.0
  * @date    01-December-2013
  * @brief   Default main function.
  ******************************************************************************
*/
#include "main.h"

uint16_t adc_val;
int main(void)
{

	MAIN_INIT();
    adc_val=0;

	for(;;)
	{
		adc_val = ADC_GetConv();

		GPIO_ByteOnPins(adc_val);

	    if(!FMSTR_DISABLE)
	    {
	      FMSTR_Poll();
	      FMSTR_Recorder();
	    }
	}
}
