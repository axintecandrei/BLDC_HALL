/*
 * mip_speed_est_if.h
 *
 *  Created on: 16 iun. 2018
 *      Author: AxinteA
 */

#ifndef MIP_SPEED_EST_IF_H_
#define MIP_SPEED_EST_IF_H_
#include "stm32f4xx.h"
/*
 * DEFINES
 */


/*
 * TYPE-DEFs
 */


/*
 * VARIABLES
 */
int32_t PORT_Mip_Est_Speed;
#define Set_Mip_Est_Speed(x) (PORT_Mip_Est_Speed = (x))
#define Get_Mip_Est_Speed()   (*((const int32_t *) &PORT_Mip_Est_Speed))

uint32_t PORT_Mip_Hall_InputCapture;
#define Set_Mip_Hall_InputCapture(x) (PORT_Mip_Hall_InputCapture = (x))
#define Get_Mip_Hall_InputCapture()   (*((const uint32_t *) &PORT_Mip_Hall_InputCapture))

/*
 * FUNCTIONS
 */
extern void MIP_SPEED_EST_INIT();

extern void MIP_SPEED_EST_MAIN();
#endif /* MIP_SPEED_EST_IF_H_ */
