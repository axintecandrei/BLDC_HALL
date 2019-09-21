/*
 * mip_speed_est.c
 *
 *  Created on: 16 iun. 2018
 *      Author: AxinteA
 */
#include "mip_speed_est.h"


void MIP_SPEED_EST_INIT()
{
   GPIO_InitTypeDef GPIO_InitStruct;
   __TIM2_CLK_ENABLE();

   /**TIM2 GPIO Configuration
   PA15     ------> TIM2_CH1
   */
   /*Enable bus clock for ports*/
   __GPIOA_CLK_ENABLE();
   GPIO_InitStruct.Pin       = HALL_SPEED_IN_PIN;
   GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
   GPIO_InitStruct.Pull      = GPIO_NOPULL;
   GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
   GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
   HAL_GPIO_Init(HALL_SPEED_IN_PORT, &GPIO_InitStruct);

   /*
   * Prescaler 83 => Fcnt = Ftim_clk/(Prescaler + 1)
   *                 Fcnt = 100Khz
   *                 Tcnt = 10 us
   */
   TIM2->PSC = 83;
   /* ARR <=> period before overflow */
   TIM2->ARR = 0xFFFF;
   /*Enable IC on ch 1*/
   TIM2->CCMR1 = ((0x00<<24) | (0x00<<16) | (0x00<<8) | (0x01));

   /*Enable input on ch1*/
   TIM2->CCER = 0b0111;
   TIM2->DIER = 2;

   /* Peripheral interrupt init */
   HAL_NVIC_SetPriority(TIM2_IRQn, 0, 2);
   HAL_NVIC_EnableIRQ(TIM2_IRQn);

   TIM2->CR1  |= TIM_CR1_CEN;

   Set_Mip_Est_Speed(0);
   Set_Mip_New_Capture_Flag(0);
}

void MIP_SPEED_EST_MAIN()
{
	float temp_speed;
    static uint8_t prev_capture_flag = 0;
    static uint16_t motor_standstill_cnt = 0;
#if 0
	/*The capture value represents count of 10us steps taken
	 * between a rising and falling of hall sensor*/
	temp_speed = ((float)Get_Mip_Hall_InputCapture())*0.00001;
	/*Going from time to frequency F_hall = 1/T_hall*/
	temp_speed = 1.0F/temp_speed;
	/*Going from Hz to rpm, multiplying by 60
	 * BUT, for 1 mech rot, there are 2 captures of the hall
	 * so divinding by 2 results the mech speed*/
	temp_speed = temp_speed * 30.0F;
#endif

	if(Get_Mip_New_Capture_Flag() == prev_capture_flag)
	{
		motor_standstill_cnt++;
		if (motor_standstill_cnt >333)
		{
			motor_standstill_cnt = 333;
		}
	}
	else
	{
		motor_standstill_cnt = 0;
	}
    /*if motor does not spins within 0.0333 ms -speed less than 500 rpm
     * set speed to 0*/
	if(motor_standstill_cnt < 333)
	{
		temp_speed = 60000000/(Get_Mip_Hall_InputCapture()*6);
	}
	else
	{
		temp_speed = 0;
	}

	Set_Mip_Est_Speed(MIP_SPEED_SIGN(Get_Hall_State())*temp_speed);
	prev_capture_flag = Get_Mip_New_Capture_Flag();


}

int8_t MIP_SPEED_SIGN(uint8_t hall_state)
{
	static uint8_t prev_hall_state;
	static int8_t sign = 0;
	switch (hall_state)
	{
		case (1u):
		{
		    (prev_hall_state == 5)? (sign = CW) : ((prev_hall_state == 3)?(sign = CCW) : sign)  ;
		}
			break;
		case (2u):
		{
			(prev_hall_state == 3)? (sign = CW) : ((prev_hall_state == 6)?(sign = CCW) : sign)  ;
		}
			break;
		case (3u):
		{
			(prev_hall_state == 1)? (sign = CW) : ((prev_hall_state == 2)?(sign = CCW) : sign)  ;
		}
			break;
		case (4u):
		{
			(prev_hall_state == 6)? (sign = CW) : ((prev_hall_state == 5)?(sign = CCW) : sign)  ;
		}
			break;
		case (5u):
		{
			(prev_hall_state == 4)? (sign = CW) : ((prev_hall_state == 1)?(sign = CCW) : sign)  ;
		}
			break;
		case (6u):
		{
			(prev_hall_state == 2)? (sign = CW) : ((prev_hall_state == 4)?(sign = CCW) : sign)  ;
		}
			break;
		default:
			break;
	}

	prev_hall_state = hall_state;
    return sign;
}
