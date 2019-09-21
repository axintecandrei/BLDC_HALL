/*
 * mip_volt_acq.c
 *
 *  Created on: 16 iun. 2018
 *      Author: AxinteA
 */
#include "mip_volt_acq.h"

void MIP_VOLT_ACQ_MAIN ()
{
	float temp_adc_volt;
	float temp_voltage;

	temp_adc_volt =  (float)Get_AdcDcLink() * ADC2VOLT_FACTOR;
	temp_voltage = temp_adc_volt*VOLT_DIV_FACTOR;
	Set_Mip_Volt_DCLink(temp_voltage*1000);

#if CFG_ADC_REG_CONV
	temp_adc_volt = (float)Get_AdcVa() * ADC2VOLT_FACTOR;
	temp_voltage = temp_adc_volt*VOLT_DIV_FACTOR;
	Set_Mip_Volt_A(temp_voltage*1000);

	temp_adc_volt = (float)Get_AdcVb() * ADC2VOLT_FACTOR;
	temp_voltage = temp_adc_volt*VOLT_DIV_FACTOR;
	Set_Mip_Volt_B(temp_voltage*1000);

	temp_adc_volt = (float)Get_AdcVc() * ADC2VOLT_FACTOR;
	temp_voltage = temp_adc_volt*VOLT_DIV_FACTOR;
	Set_Mip_Volt_C(temp_voltage*1000);

	/*normalize*/
	Set_Mip_Volt_A((float)Get_Mip_Volt_A() - (float)Get_Mip_Volt_DCLink()/2.0F);
	Set_Mip_Volt_B((float)Get_Mip_Volt_B() - (float)Get_Mip_Volt_DCLink()/2.0F);
	Set_Mip_Volt_C((float)Get_Mip_Volt_C() - (float)Get_Mip_Volt_DCLink()/2.0F);

#endif


}
