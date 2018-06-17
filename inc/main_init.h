/*
 * mcu_init.h
 *
 *  Created on: 20 mar. 2018
 *      Author: uidp7521
 */

#ifndef MCU_INIT_H_
#define MCU_INIT_H_

#include "stm32f4xx.h"
#include "stm32f4xx_nucleo.h"
#include "lld_adc.h"
#include "lld_lcd.h"
#include "lld_usart.h"
#include "lld_gpio.h"
#include "lld_bldc_pwm.h"
#include "lld_gp_timers.h"
#include "lld_hall.h"
#include "mip_speed_est.h"
#include "moc_speed_ctrl_if.h"
#include "freemaster.h"
#include "freemaster_STM32F4xx.h"



void MAIN_INIT(void);
#endif /* MCU_INIT_H_ */
