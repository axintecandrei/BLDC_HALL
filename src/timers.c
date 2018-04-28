/*
 * timers.c
 *
 *  Created on: 10 apr. 2018
 *      Author: axint
 */

#include "timers.h"

void TIMER2_INIT()
{
   __TIM2_CLK_ENABLE();
   /*
    * Prescaler 3 => Fcnt = Ftim_clk/(Prescaler + 1)
    *                Fcnt = 21Mhz
    *                Tcnt = 47.61904761905 ns
    */
   TIM2->PSC = 3;
   /*
    * ARR <=> period before overflow
    *         in run time measurements it would be a good
    *         practice to set this as large as possible.
    *
    */
   TIM2->ARR = 10000000;

   TIM2->CR1  |= TIM_CR1_CEN;
}


