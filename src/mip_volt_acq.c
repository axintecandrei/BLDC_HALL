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
	float temp_dclink_volt;

	temp_adc_volt =  (float)Get_AdcDcLink() * ADC2VOLT_FACTOR;
	temp_dclink_volt = temp_adc_volt*VOLT_DIV_FACTOR;


	Set_Mip_Volt_DCLink(temp_dclink_volt*1000);
}
