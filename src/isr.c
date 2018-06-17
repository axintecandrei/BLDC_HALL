#include "isr.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx.h"




/**
  * @brief  This function handles SysTick Handler, but only if no RTOS defines it.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
	HAL_IncTick();
	HAL_SYSTICK_IRQHandler();
}

void TIM1_UP_TIM10_IRQHandler(void)
{
   task_scheduler();
   HAL_TIM_IRQHandler(&htim1);
}

void TIM2_IRQHandler ()
{

   static volatile uint32_t previous_capture_value = 0;
   static volatile uint32_t current_capture_value;

   current_capture_value = TIM2->CCR1;

   if(HAL_GPIO_ReadPin(HALL_PORT_A, HALL_1) == 1)
   {
	   previous_capture_value = current_capture_value;
	   TIM2->CCER |= TIM_CCER_CC1P;
   }
   else
   {
	   Set_Mip_Hall_InputCapture(current_capture_value - previous_capture_value);
	   if(Get_Mip_Hall_InputCapture() < 0)
	   {
		   Set_Mip_Hall_InputCapture (Get_Mip_Hall_InputCapture() + 0xFFFF);
	   }
	   TIM2->CCER &= ~TIM_CCER_CC1P;
   }
}

/**
* @brief This function handles USART2 global interrupt.
*/
void USART2_IRQHandler(void)
{

   if(!FMSTR_DISABLE)
   {
      FMSTR_ProcessSCI();
   }
  
   HAL_UART_IRQHandler(&huart2);

}
