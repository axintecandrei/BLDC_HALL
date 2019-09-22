/*
 * mip_current_acq.c
 *
 *  Created on: 16 iun. 2018
 *      Author: AxinteA
 */
#include "mip_current_acq.h"



void MIP_CURR_ACQ_MAIN()
{
	float temp_adc_volt;
	float temp_curr;

	temp_adc_volt = (float)Get_AdcIa() * ADC2VOLT_FACTOR;
	temp_curr = temp_adc_volt - CURRENT_OFFSET;
	temp_curr /= CURRENT_GAIN;
	temp_curr /= R_SHUNT;

	Set_Mip_Acq_Curr_U(temp_curr*100);


	temp_adc_volt = (float)Get_AdcIb() * ADC2VOLT_FACTOR;
	temp_curr = temp_adc_volt - CURRENT_OFFSET;
	temp_curr /= CURRENT_GAIN;
	temp_curr /= R_SHUNT;

	Set_Mip_Acq_Curr_V(temp_curr*100);



	temp_adc_volt = (float)Get_AdcIc() * ADC2VOLT_FACTOR;
	temp_curr = temp_adc_volt - CURRENT_OFFSET;
	temp_curr /= CURRENT_GAIN;
	temp_curr /= R_SHUNT;

	Set_Mip_Acq_Curr_W(temp_curr*100);

	Set_Mip_Acq_Max_Is(UTIL_Maxof3(Get_Mip_Acq_Curr_U(),Get_Mip_Acq_Curr_V(),Get_Mip_Acq_Curr_W()));
}
