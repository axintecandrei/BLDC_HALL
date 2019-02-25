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
#define PWM_EN_GATE    GPIO_PIN_4
#define PWM_B_L        GPIO_PIN_0
#define PWM_C_L        GPIO_PIN_1

/*pins on GPIOA*/
#define PWM_A_H        GPIO_PIN_8
#define PWM_A_L        GPIO_PIN_7
#define PWM_B_H        GPIO_PIN_9
#define PWM_C_H        GPIO_PIN_10

#define PWM_A_ACTIVE     (0b0101)
#define PWM_A_INACTIVE   (0b0000)
#define PWM_B_ACTIVE     ((0b0101)<<4)
#define PWM_B_INACTIVE   ((0b0000)<<4)
#define PWM_C_ACTIVE     ((0b0101)<<8)
#define PWM_C_INACTIVE   ((0b0000)<<8)

#define BLDC_PWM_MAX_DTC (4199)

#define TIM_CLEAR_IT      (TIM1->SR  &= ~TIM_IT_UPDATE)
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
