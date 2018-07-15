/*
 * bldc_pwm.h
 *
 *  Created on: 2 iun. 2018
 *      Author: AxinteA
 */

#ifndef BLDC_PWM_H_
#define BLDC_PWM_H_

#include "lld_bldc_pwm_if.h"
#include "lld_hall_if.h"

/*
 * DEFINES
 */
/*pins on GPIOB*/
#define PWM_EN_GATE  GPIO_PIN_3
#define PWM2L        GPIO_PIN_0
#define PWM3L        GPIO_PIN_1

/*pins on GPIOA*/
#define PWM1H        GPIO_PIN_8
#define PWM1L        GPIO_PIN_7
#define PWM2H        GPIO_PIN_9
#define PWM3H        GPIO_PIN_10

#define PWM_U_ACTIVE     (0b0101)
#define PWM_U_INACTIVE   (0b0000)
#define PWM_V_ACTIVE     ((0b0101)<<4)
#define PWM_V_INACTIVE   ((0b0000)<<4)
#define PWM_W_ACTIVE     ((0b0101)<<8)
#define PWM_W_INACTIVE   ((0b0000)<<8)

#define BLDC_PWM_MAX_DTC (4199)
/*
 * TYPE-DEFs
 */

/*
 * VARIABLES
 */

TIM_HandleTypeDef htim1;

/*
 * FUNCTIONS
 */

#endif /* BLDC_PWM_H_ */
