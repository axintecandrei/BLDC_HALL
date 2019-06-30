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

/*
 * FUNCTIONS
 */

#endif /* MIP_VOLT_ACQ_IF_H_ */
