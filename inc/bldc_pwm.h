/*
 * bldc_pwm.h
 *
 *  Created on: 2 iun. 2018
 *      Author: AxinteA
 */

#ifndef BLDC_PWM_H_
#define BLDC_PWM_H_

#include "stm32f4xx.h"
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

//#define BLDC_PWM_ON_GATE  (do{ GPIOB->BSRR = (uint32_t)PWM_EN_GATE << 16U;}while(0);)
//#define BLDC_PWM_OFF_GATE  (do{GPIOB->BSRR = PWM_EN_GATE;}while(0);)


/*
 * TYPE-DEFs
 */


/*
 * VARIABLES
 */

extern TIM_HandleTypeDef htim1;

/*
 * FUNCTIONS
 */

void BLDC_PWM_INIT();

void BLDC_PWM_GPIO_INIT();

#endif /* BLDC_PWM_H_ */
