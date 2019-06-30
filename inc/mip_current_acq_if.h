/*
 * mip_current_acq_if.h
 *
 *  Created on: 16 iun. 2018
 *      Author: AxinteA
 */

#ifndef MIP_CURRENT_ACQ_IF_H_
#define MIP_CURRENT_ACQ_IF_H_
#include "../inc/lld_adc_if.h"
/*
 * DEFINES
 */


/*
 * TYPE-DEFs
 */


/*
 * VARIABLES
 */
int16_t PORT_Mip_Acq_Curr_U;
#define Set_Mip_Acq_Curr_U(x) (PORT_Mip_Acq_Curr_U = (x))
#define Get_Mip_Acq_Curr_U()   (*((const int16_t *) &PORT_Mip_Acq_Curr_U))

int16_t PORT_Mip_Acq_Curr_V;
#define Set_Mip_Acq_Curr_V(x) (PORT_Mip_Acq_Curr_V = (x))
#define Get_Mip_Acq_Curr_V()   (*((const int16_t *) &PORT_Mip_Acq_Curr_V))

int16_t PORT_Mip_Acq_Curr_W;
#define Set_Mip_Acq_Curr_W(x) (PORT_Mip_Acq_Curr_W = (x))
#define Get_Mip_Acq_Curr_W()   (*((const int16_t *) &PORT_Mip_Acq_Curr_W))

/*
 * FUNCTIONS
 */
extern void MIP_CURR_ACQ_MAIN();
#endif /* MIP_CURRENT_ACQ_IF_H_ */
