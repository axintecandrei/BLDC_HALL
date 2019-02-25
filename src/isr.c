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


void ADC_IRQHandler(void)
{
#if CFG_DEBUG_FMSTR
	interrupt_arrived++;
    TIM1_SR = TIM1->SR;
    TIM1_CNT = TIM1->CNT;
    TIM1_CR1 = TIM1->CR1;
    TIM1_CR2 = TIM1->CR2;
    TIM1_SMCR = TIM1->SMCR;
    ADC_SR = ADC1->SR;
    ADC_CR1 = ADC1->CR1;
    ADC_CR2 = ADC1->CR2;
    TIM1_CCER = TIM1->CCER ;
    TIM1_CCR1 = TIM1->CCR1;
#endif
   task_scheduler();
   ADC_CLEAR_IT();
}

void TIM1_UP_TIM10_IRQHandler(void)
{
   HAL_TIM_IRQHandler(&htim1);
}

void TIM2_IRQHandler ()
{

   static volatile uint32_t previous_capture_value = 0;
   uint32_t current_capture_value;

   current_capture_value = TIM2->CCR1;
#if 0
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
#else
   Set_Mip_Hall_InputCapture(current_capture_value - previous_capture_value);
   if(Get_Mip_Hall_InputCapture() < 0)
   {
	   Set_Mip_Hall_InputCapture (Get_Mip_Hall_InputCapture() + 0xFFFF);
   }
   previous_capture_value = TIM2->CCR1;
#endif


}

void USART2_IRQHandler(void)
{

   if(!FMSTR_DISABLE)
   {
      FMSTR_ProcessSCI();
   }
  
   HAL_UART_IRQHandler(&huart2);

}
