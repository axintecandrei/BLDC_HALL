/*
 * mip_volt_acq_if.h
 *
 *  Created on: 16 iun. 2018
 *      Author: AxinteA
 */

#ifndef MIP_VOLT_ACQ_IF_H_
#define MIP_VOLT_ACQ_IF_H_
#include "../inc/lld_adc_if.h"
#include "../inc/utilities.h"
/*
 * DEFINES
 */


/*
 * TYPE-DEFs
 */


/*
 * VARIABLES
 */
uint16_t PORT_Mip_Volt_DCLink;
#define Set_Mip_Volt_DCLink(x) (PORT_Mip_Volt_DCLink = (x))
#define Get_Mip_Volt_DCLink()   (*((const uint16_t *) &PORT_Mip_Volt_DCLink))

#if CFG_ADC_REG_CONV
int16_t PORT_Mip_Volt_A;
#define Set_Mip_Volt_A(x) (PORT_Mip_Volt_A = (x))
#define Get_Mip_Volt_A()   (*((const int16_t *) &PORT_Mip_Volt_A))

int16_t PORT_Mip_Volt_B;
#define Set_Mip_Volt_B(x) (PORT_Mip_Volt_B = (x))
#define Get_Mip_Volt_B()   (*((const int16_t *) &PORT_Mip_Volt_B))

int16_t PORT_Mip_Volt_C;
#define Set_Mip_Volt_C(x) (PORT_Mip_Volt_C = (x))
#define Get_Mip_Volt_C()   (*((const int16_t *) &PORT_Mip_Volt_C))
#endif
/*
 * FUNCTIONS
 */
extern void MIP_VOLT_ACQ_MAIN ();
#endif /* MIP_VOLT_ACQ_IF_H_ */
