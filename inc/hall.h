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

#define HALL_PORT          GPIOA
#define HALL_1             GPIO_PIN_11
#define HALL_2             GPIO_PIN_12
#define HALL_3             GPIO_PIN_15



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
void HALL_GET_STATE();
#endif /* HALL_H_ */
