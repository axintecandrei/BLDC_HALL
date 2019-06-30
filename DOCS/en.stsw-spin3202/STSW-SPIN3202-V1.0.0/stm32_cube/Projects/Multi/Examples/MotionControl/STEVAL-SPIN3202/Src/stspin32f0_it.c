/**
  ******************************************************************************
  * @file    stm32f0xx_it.c
  * @author  IPC Rennes
  * @version V1.0.0
  * @date    April 24th, 2017
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_hal.h"
#include "stm32f0xx.h"
#include "stspin32f0_it.h"
#include "6Step_Lib.h"
#ifdef UART_COMM
#include "UART_UI.h"
#endif

/* External variables --------------------------------------------------------*/
extern SIXSTEP_Base_InitTypeDef SIXSTEP_parameters; /*!< Main SixStep structure*/
extern ADC_HandleTypeDef ADCx;
extern TIM_HandleTypeDef HF_TIMx;
extern TIM_HandleTypeDef LF_TIMx;
extern UART_HandleTypeDef huart;

/******************************************************************************/
/*            Cortex-M0 Processor Interruption and Exception Handlers         */ 
/******************************************************************************/
/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
* @brief This function handles USART1 global interrupt.
*/
void USART1_IRQHandler(void)
{
#ifdef UART_COMM  
  /* UART in mode Receiver ---------------------------------------------------*/
  if((__HAL_UART_GET_IT(&huart, UART_IT_RXNE) != RESET) && (__HAL_UART_GET_IT_SOURCE(&huart, UART_IT_RXNE) != RESET))
  { 
    if ((huart.State!=HAL_UART_STATE_TIMEOUT)&&(huart.State!=HAL_UART_STATE_ERROR))
    {
      huart.State &= ~HAL_UART_STATE_READY;
      huart.State |= HAL_UART_STATE_BUSY_RX;
      if (UART_Receive_IT(&huart)==HAL_OK)
      {
        UART_Set_Value();
      }
    }
    /* Clear RXNE interrupt flag */
    __HAL_UART_SEND_REQ(&huart, UART_RXDATA_FLUSH_REQUEST);
  }
#endif
}

/**
* @brief This function handles DMA USART1 TX interrupt.
*/
void DMA1_Channel2_3_IRQHandler(void)
{
  HAL_DMA_IRQHandler(huart.hdmatx);
}

/**
* @brief This function handles TIM1 Break, Update, Trigger and Commutation Interrupts.
*/
void TIM1_BRK_UP_TRG_COM_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&HF_TIMx);
}
#if (!defined(HALL_SENSORS)||defined(POTENTIOMETER))
/**
* @brief This function handles ADC global interrupt.
*/
void ADC1_IRQHandler(void)
{
  HAL_ADC_IRQHandler(&ADCx);
}
#endif

/**
* @brief This function handles EXTI Line 0 to 1 interrupts.
*/
void EXTI0_1_IRQHandler(void)
{
  HAL_GPIO_EXTI_IRQHandler(BSP_BOARD_USER1_BUTTON_PIN);
  HAL_GPIO_EXTI_IRQHandler(BSP_BOARD_USER2_BUTTON_PIN);
}

/**
* @brief This function handles System tick timer.
*/
void SysTick_Handler(void)
{
  HAL_IncTick();
  HAL_SYSTICK_IRQHandler();
}

/**
* @brief This function handles TIM2 Interrupts.
*/
void TIM2_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&LF_TIMx);
}

/**
* @brief This function handles TIM3 global interrupt.
*/
void TIM3_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&LF_TIMx);
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
