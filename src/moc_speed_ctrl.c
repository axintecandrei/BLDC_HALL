/*
 * moc_speed_ctrl.c
 *
 *  Created on: 16 iun. 2018
 *      Author: AxinteA
 */
#include "moc_speed_ctrl.h"

void MOC_SPEED_CTRL_INIT()
{
   MOC_SPEED_CTRL.OUTPUT_CURR=0;
   MOC_SPEED_CTRL.ERROR_SPEED=0;
   MOC_SPEED_CTRL.P_GAIN=0.0000046;/* 0.000014107;// gains for speed controller when used only*/
   MOC_SPEED_CTRL.I_GAIN=0.001986 ;//0.00626;;

   MOC_CURR_CTRL.OUTPUT_VOLT=0;
   MOC_CURR_CTRL.ERROR_CURRENT=0;
   MOC_CURR_CTRL.P_GAIN=0.000;/*gains for speed controller when used only*/
   MOC_CURR_CTRL.I_GAIN=0.000;

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
   MOC_SPEED_CTRL.OUTPUT_CURR = MOC_SPEED_CTRL.P_PART + MOC_SPEED_CTRL.I_PART;

   MOC_SPEED_CTRL.OUTPUT_CURR = LIM(MOC_SPEED_CTRL.OUTPUT_CURR,(-dc_link), (dc_link));

   to_be_applied_DTC = ((dc_link + MOC_SPEED_CTRL.OUTPUT_CURR)/dc_link)*PWM_MAX_TICKS;

   Set_Bldc_Pwm_A(to_be_applied_DTC);
   Set_Bldc_Pwm_B(to_be_applied_DTC);
   Set_Bldc_Pwm_C(to_be_applied_DTC);

#if 0
   Set_Bldc_Pwm_A(((float)Get_Moc_Req_Speed()/100.0F)*4199);
   Set_Bldc_Pwm_B(((float)Get_Moc_Req_Speed()/100.0F)*4199);
   Set_Bldc_Pwm_C(((float)Get_Moc_Req_Speed()/100.0F)*4199);
#endif

}

void MOC_CURRENT_CTRL_MAIN()
{
	   uint16_t to_be_applied_DTC;
	   int16_t  loc_req_current;
	   float dc_link;
	   //float curr_gradient = 0.005;

	   dc_link = (float)Get_Mip_Volt_DCLink()*0.0005F;
	   loc_req_current = MOC_SPEED_CTRL.OUTPUT_CURR/*LIM(MOC_SPEED_CTRL.OUTPUT_CURR,MOC_SPEED_CTRL.OUTPUT_CURR-curr_gradient,MOC_SPEED_CTRL.OUTPUT_CURR+curr_gradient)*/;

	   MOC_CURR_CTRL.ERROR_CURRENT = (float)(loc_req_current) - (float)(Get_Mip_Acq_Max_Is());
	   MOC_CURR_CTRL.P_PART = (MOC_CURR_CTRL.P_GAIN * MOC_CURR_CTRL.ERROR_CURRENT);
	   MOC_CURR_CTRL.I_PART += (MOC_CURR_CTRL.I_GAIN * MOC_CURR_CTRL.ERROR_CURRENT)/Ts;
	   MOC_CURR_CTRL.OUTPUT_VOLT = MOC_CURR_CTRL.P_PART + MOC_CURR_CTRL.I_PART;

	   MOC_CURR_CTRL.OUTPUT_VOLT = LIM(MOC_CURR_CTRL.OUTPUT_VOLT,(-dc_link), (dc_link));

	   to_be_applied_DTC = ((dc_link + MOC_CURR_CTRL.OUTPUT_VOLT)/dc_link)*PWM_MAX_TICKS;

	   Set_Bldc_Pwm_A(to_be_applied_DTC);
	   Set_Bldc_Pwm_B(to_be_applied_DTC);
	   Set_Bldc_Pwm_C(to_be_applied_DTC);
}

void MOC_VOLTAGE_VECT_MAIN()
{
	   uint16_t to_be_applied_DTC;
	   float dc_link;


	   dc_link = (float)Get_Mip_Volt_DCLink()*0.0005F;
	   to_be_applied_DTC = ((dc_link + MOC_CURR_CTRL.OUTPUT_VOLT)/dc_link)*PWM_MAX_TICKS;

	   Set_Bldc_Pwm_A(to_be_applied_DTC);
	   Set_Bldc_Pwm_B(to_be_applied_DTC);
	   Set_Bldc_Pwm_C(to_be_applied_DTC);
}
