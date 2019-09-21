/*
 * lld_hall_if.h
 *
 *  Created on: 18 iun. 2018
 *      Author: uidp7521
 */

#ifndef LLD_HALL_IF_H_
#define LLD_HALL_IF_H_
#include "stm32f4xx.h"




uint8_t PORT_Hall_State;
#define Set_Hall_State(x) (PORT_Hall_State = (x))
#define Get_Hall_State()   (*((const uint8_t *) &PORT_Hall_State))

extern void HALL_INIT();
extern void HALL_GET_STATE();
#endif /* LLD_HALL_IF_H_ */
