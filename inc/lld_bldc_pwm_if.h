/*
 * lld_bldc_pwm_if.h
 *
 *  Created on: 16 iun. 2018
 *      Author: AxinteA
 */

#ifndef LLD_BLDC_PWM_IF_H_
#define LLD_BLDC_PWM_IF_H_
#include "stm32f4xx.h"
/*
 * DEFINES
 */


/*
 * TYPE-DEFs
 */
typedef enum e_sectors
{
	Sector_1=1,
	Sector_2,
	Sector_3,
	Sector_4,
	Sector_5,
	Sector_6
}e_sectors_t;

typedef enum e_gate_states
{
	GATE_DISABLE,
	GATE_ENABLE
}e_gate_states_t;

/*
 * VARIABLES
 */
uint16_t PORT_Bldc_Pwm_U;
#define Set_Bldc_Pwm_U(x) (PORT_Bldc_Pwm_U = (x))
#define Get_Bldc_Pwm_U()   (*((const uint16_t *) &PORT_Bldc_Pwm_U))

uint16_t PORT_Bldc_Pwm_V;
#define Set_Bldc_Pwm_V(x) (PORT_Bldc_Pwm_V = (x))
#define Get_Bldc_Pwm_V()   (*((const uint16_t *) &PORT_Bldc_Pwm_V))

uint16_t PORT_Bldc_Pwm_W;
#define Set_Bldc_Pwm_W(x) (PORT_Bldc_Pwm_W = (x))
#define Get_Bldc_Pwm_W()   (*((const uint16_t *) &PORT_Bldc_Pwm_W))

uint8_t PORT_Bldc_En_Gate;
#define Set_Bldc_En_Gate(x) (PORT_Bldc_En_Gate = (x))
#define Get_Bldc_En_Gate()   (*((const uint8_t *) &PORT_Bldc_En_Gate))
/*
 * FUNCTIONS
 */

extern void BLDC_PWM_INIT();

extern void BLDC_PWM_GPIO_INIT();

extern void BLDC_PWM_HANDLER();
#endif /* LLD_BLDC_PWM_IF_H_ */
