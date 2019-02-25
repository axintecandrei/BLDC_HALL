#include "tasks.h"


void _100US_LOOP (void)
{
   HALL_GET_STATE();
   //MIP_SPEED_EST_MAIN();
   //MOC_SPEED_CTRL_MAIN();

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
