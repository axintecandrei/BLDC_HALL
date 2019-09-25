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
/*
#define PWM_A_ACTIVE     (0b1111)
#define PWM_A_INACTIVE   (0b1010)
#define PWM_B_ACTIVE     ((0b1111)<<4)
#define PWM_B_INACTIVE   ((0b1010)<<4)
#define PWM_C_ACTIVE     ((0b1111)<<8)
#define PWM_C_INACTIVE   ((0b1010)<<8)*/

/*Confing PWM
 * - all frequency values are in Hz*/
#define PWM_DESIRED_PWM  (20000)

#define SYS_CLK_FREQ     (HAL_RCC_GetSysClockFreq())
#define TIM1_CLK_FREQ    (84000000.0F)
#define TIM1_PSC         ((SYS_CLK_FREQ/TIM1_CLK_FREQ) -1 )
#define TIM1_ARR         ((TIM1_CLK_FREQ/PWM_DESIRED_PWM) -1)

#define TIM1_DEAD_TIME   (2.0F*0.000001F) /*microseconds*/
#define TIM1_DTG         (TIM1_DEAD_TIME*TIM1_CLK_FREQ)

#define BLDC_PWM_MAX_DTC ((uint16_t)TIM1_ARR)

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

uint8_t BLDC_EN_GATE;
#endif /* BLDC_PWM_H_ */
