/*
 * hall.h
 *
 *  Created on: 2 iun. 2018
 *      Author: AxinteA
 */

#ifndef HALL_H_
#define HALL_H_

#include "stm32f4xx.h"
#include "utilities.h"

/*
 * DEFINES
 */

#define HALL_PORT_A        GPIOA
#define HALL_1             GPIO_PIN_11
#define HALL_2             GPIO_PIN_12
#define HALL_PORT_B        GPIOB
#define HALL_3             GPIO_PIN_12



/*
 * TYPE-DEFs
 */


/*
 * VARIABLES
 */


/*
 * FUNCTIONS
 */

void HALL_INIT();
void HALL_GET_STATE(uint8_t * hall_state);
#endif /* HALL_H_ */
