/*
 * lld_bldc_pwm_if.h
 *
 *  Created on: 16 iun. 2018
 *      Author: AxinteA
 */

#ifndef LLD_BLDC_PWM_IF_H_
#define LLD_BLDC_PWM_IF_H_
#include "stm32f4xx.h"
#include "config.h"
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


uint16_t PORT_Bldc_Pwm_A;
#define Set_Bldc_Pwm_A(x) (PORT_Bldc_Pwm_A = (x))
#define Get_Bldc_Pwm_A()   (*((const uint16_t *) &PORT_Bldc_Pwm_A))

uint16_t PORT_Bldc_Pwm_B;
#define Set_Bldc_Pwm_B(x) (PORT_Bldc_Pwm_B = (x))
#define Get_Bldc_Pwm_B()   (*((const uint16_t *) &PORT_Bldc_Pwm_B))

uint16_t PORT_Bldc_Pwm_C;
#define Set_Bldc_Pwm_C(x) (PORT_Bldc_Pwm_C = (x))
#define Get_Bldc_Pwm_C()   (*((const uint16_t *) &PORT_Bldc_Pwm_C))


/*
 * FUNCTIONS
 */

extern void BLDC_PWM_INIT();

extern void BLDC_PWM_HANDLER();
#endif /* LLD_BLDC_PWM_IF_H_ */
