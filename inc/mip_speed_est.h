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


/*
 * TYPE-DEFs
 */


/*
 * VARIABLES
 */
TIM_HandleTypeDef htim2;

uint32_t IC_RAW_VAL[1];
/*
 * FUNCTIONS
 */


void HAL_TIM2_Base_MspInit(TIM_HandleTypeDef* tim_baseHandle);
#endif /* MIP_SPEED_EST_H_ */
