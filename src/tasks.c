#include "tasks.h"


void _100US_LOOP (void)
{
   MAIN_CLOCK++;
   LLD_ADC_GET_RAW_VAL();
   MIP_VOLT_ACQ_MAIN();
   MIP_CURR_ACQ_MAIN();
   HALL_GET_STATE();
   MIP_SPEED_EST_MAIN();
   MOC_SPEED_CTRL_MAIN();

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
