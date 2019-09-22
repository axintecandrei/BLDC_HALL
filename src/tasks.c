#include "tasks.h"


void _100US_LOOP (void)
{
   static uint8_t control_word=0;
   MAIN_CLOCK++;
   LLD_ADC_GET_RAW_VAL();
   MIP_VOLT_ACQ_MAIN();
   MIP_CURR_ACQ_MAIN();
   HALL_GET_STATE();
   MIP_SPEED_EST_MAIN();
   switch (control_word)
   {
   case 0:
	   MOC_SPEED_CTRL_MAIN();
	   break;
   case 1 :
	   MOC_CURRENT_CTRL_MAIN();
	   break;
   case 2 :
	   MOC_SPEED_CTRL_MAIN();
	   MOC_CURRENT_CTRL_MAIN();
	   break;
   }

   BLDC_PWM_HANDLER();

#if(!FMSTR_DISABLE)
    {
      FMSTR_Poll();
      FMSTR_Recorder();
    }
#endif

}

void _2MS_LOOP (void)
{

}

void _10MS_LOOP (void)
{


}
