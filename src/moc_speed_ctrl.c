/*
 * moc_speed_ctrl.c
 *
 *  Created on: 16 iun. 2018
 *      Author: AxinteA
 */
#include "moc_speed_ctrl.h"

void MOC_SPEED_CTRL_INIT()
{
   MOC_SPEED_CTRL.OUTPUT_VOLT=0;
   MOC_SPEED_CTRL.OUTPUT_VOLT_K1=0;
   MOC_SPEED_CTRL.ERROR_SPEED=0;
   MOC_SPEED_CTRL.P_GAIN=0.00025;/*gains for speed controller when used only*/
   MOC_SPEED_CTRL.I_GAIN=0.00125;

   Set_Moc_Req_Speed(0);

}

void MOC_SPEED_CTRL_MAIN()
{
   uint16_t to_be_applied_DTC;
   int16_t  loc_req_speed;
   float dc_link;
   uint8_t speed_gradient = 10;
   dc_link = (float)Get_Mip_Volt_DCLink()*0.0005F;
   loc_req_speed = LIM(Get_Moc_Req_Speed(),Get_Moc_Req_Speed()-speed_gradient,Get_Moc_Req_Speed()+speed_gradient);

   MOC_SPEED_CTRL.ERROR_SPEED = (float)(loc_req_speed) - (float)(Get_Mip_Est_Speed());
   MOC_SPEED_CTRL.P_PART = (MOC_SPEED_CTRL.P_GAIN * MOC_SPEED_CTRL.ERROR_SPEED);
   MOC_SPEED_CTRL.I_PART += (MOC_SPEED_CTRL.I_GAIN * MOC_SPEED_CTRL.ERROR_SPEED)/Ts;
   MOC_SPEED_CTRL.OUTPUT_VOLT = MOC_SPEED_CTRL.P_PART + MOC_SPEED_CTRL.I_PART;

   MOC_SPEED_CTRL.OUTPUT_VOLT = LIM(MOC_SPEED_CTRL.OUTPUT_VOLT,(-dc_link), (dc_link));

   to_be_applied_DTC = ((dc_link + MOC_SPEED_CTRL.OUTPUT_VOLT)/dc_link)*PWM_MAX_TICKS;

   Set_Bldc_Pwm_A(to_be_applied_DTC);
   Set_Bldc_Pwm_B(to_be_applied_DTC);
   Set_Bldc_Pwm_C(to_be_applied_DTC);

#if 0
   Set_Bldc_Pwm_A(((float)Get_Moc_Req_Speed()/100.0F)*4199);
   Set_Bldc_Pwm_B(((float)Get_Moc_Req_Speed()/100.0F)*4199);
   Set_Bldc_Pwm_C(((float)Get_Moc_Req_Speed()/100.0F)*4199);
#endif

}
