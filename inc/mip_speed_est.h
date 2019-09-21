/*
 * mip_speed_est.h
 *
 *  Created on: 16 iun. 2018
 *      Author: AxinteA
 */

#ifndef MIP_SPEED_EST_H_
#define MIP_SPEED_EST_H_
#include "mip_speed_est_if.h"

/*
 * DEFINES
 */
#define HALL_SPEED_IN_PIN   GPIO_PIN_15
#define HALL_SPEED_IN_PORT  GPIOA

#define CCW                 (-1)
#define CW                  (1)

/*
 * TYPE-DEFs
 */


/*
 * VARIABLES
 */
TIM_HandleTypeDef htim2;

/*
 * FUNCTIONS
 */


void HAL_TIM2_Base_MspInit(TIM_HandleTypeDef* tim_baseHandle);
#endif /* MIP_SPEED_EST_H_ */
