/*
 * mip_speed_est_if.h
 *
 *  Created on: 16 iun. 2018
 *      Author: AxinteA
 */

#ifndef MIP_SPEED_EST_IF_H_
#define MIP_SPEED_EST_IF_H_
#include "stm32f4xx.h"
#include "lld_hall_if.h"
/*
 * DEFINES
 */


/*
 * TYPE-DEFs
 */


/*
 * VARIABLES
 */
int16_t PORT_Mip_Est_Speed;
#define Set_Mip_Est_Speed(x) (PORT_Mip_Est_Speed = (x))
#define Get_Mip_Est_Speed()   (*((const int16_t *) &PORT_Mip_Est_Speed))

int32_t PORT_Mip_Hall_InputCapture;
#define Set_Mip_Hall_InputCapture(x) (PORT_Mip_Hall_InputCapture = (x))
#define Get_Mip_Hall_InputCapture()   (*((const int32_t *) &PORT_Mip_Hall_InputCapture))

uint8_t PORT_Mip_New_Capture_Flag;
#define Set_Mip_New_Capture_Flag(x) (PORT_Mip_New_Capture_Flag = (x))
#define Get_Mip_New_Capture_Flag()   (*((const uint8_t *) &PORT_Mip_New_Capture_Flag))
/*
 * FUNCTIONS
 */
extern void MIP_SPEED_EST_INIT();

extern void MIP_SPEED_EST_MAIN();

extern int8_t MIP_SPEED_SIGN(uint8_t hall_state);
#endif /* MIP_SPEED_EST_IF_H_ */
