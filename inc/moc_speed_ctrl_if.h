/*
 * moc_speed_ctrl_if.h
 *
 *  Created on: 16 iun. 2018
 *      Author: AxinteA
 */

#ifndef MOC_SPEED_CTRL_IF_H_
#define MOC_SPEED_CTRL_IF_H_
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
int16_t PORT_Moc_Req_Speed;
#define Set_Moc_Req_Speed(x) (PORT_Moc_Req_Speed = (x))
#define Get_Moc_Req_Speed()   (*((const int16_t *) &PORT_Moc_Req_Speed))

/*
 * FUNCTIONS
 */
extern void MOC_SPEED_CTRL_INIT();
extern void MOC_SPEED_CTRL_MAIN();
#endif /* MOC_SPEED_CTRL_IF_H_ */
