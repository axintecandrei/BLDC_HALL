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
   GPIO_InitStruct.Pin       = GPIO_PIN_15;
   GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
   GPIO_InitStruct.Pull      = GPIO_NOPULL;
   GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
   GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
   HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

   /*
   * Prescaler 83 => Fcnt = Ftim_clk/(Prescaler + 1)
   *                 Fcnt = 84Khz
   *                 Tcnt = 10 us
   */
   TIM2->PSC = 839;
   /* ARR <=> period before overflow */
   TIM2->ARR = 0xFFFF;
   /*Enable IC on ch 1*/
   TIM2->CCMR1 = ((0x00<<24) | (0x00<<16) | (0x00<<8) | (0x01));

   /*Enable input on ch1*/
   TIM2->CCER = 0b0001;
   TIM2->DIER = 2;

   /* Peripheral interrupt init */
   HAL_NVIC_SetPriority(TIM2_IRQn, 0, 2);
   HAL_NVIC_EnableIRQ(TIM2_IRQn);

   TIM2->CR1  |= TIM_CR1_CEN;

   Set_Mip_Est_Speed(0);
}

void MIP_SPEED_EST_MAIN()
{
	float temp_speed;
	temp_speed = ((float)Get_Mip_Hall_InputCapture())*0.00001;
	temp_speed = 1.0F/temp_speed;
	temp_speed = temp_speed * 60.0F;

	Set_Mip_Est_Speed(temp_speed/2);

}
