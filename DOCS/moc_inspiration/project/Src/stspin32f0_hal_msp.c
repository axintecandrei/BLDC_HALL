/**
  ******************************************************************************
  * @file    stm32f0xx_hal_msp.c
  * @author  IPC Rennes
  * @version V1.0.0
  * @date    April 24th, 2017
  * @brief   HAL MSP module.    
  ******************************************************************************
  *
  * COPYRIGHT(c) 2015 STMicroelectronics
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
#include "main_32F0.h"
#include "stm32f0xx_hal.h"

/* Imported variables --------------------------------------------------------*/
extern SIXSTEP_Base_InitTypeDef SIXSTEP_parameters;
extern SIXSTEP_PI_PARAM_InitTypeDef_t PI_parameters;

/* Private variables ---------------------------------------------------------*/
#ifdef TEST
extern uint8_t stop;
#endif

/* Private function prototypes -----------------------------------------------*/
extern void MC_TIMx_SixStep_timebase(void);
extern void MC_SixStep_NEXT_step(int32_t);
extern void MC_SysTick_SixStep_MediumFrequencyTask(void);
#ifdef HALL_SENSORS
extern void MC_ADCx_SixStep_User(void);
extern void MC_TIMx_SixStep_CommutationEvent(void);
extern void MC_SixStep_TABLE(uint8_t step_number);
extern void MC_SixStep_Hall_Startup_Failure_Handler(void);
extern void MC_SixStep_Hall_Run_Failure_Handler(void);
#else
extern void MC_ADCx_SixStep_Bemf(void);
#endif
extern void UART_Send_Reply(void);

/** @defgroup MSP_module
  * @brief HAL MSP module.
  * @{
  */

/* Private functions ---------------------------------------------------------*/

/** @defgroup HAL_MSP_Private_Functions
  * @{
  */

/**
  * @brief     Initializes the Global MSP 
  * @retval    None
  */
void HAL_MspInit(void)
{
  /* System interrupt init*/
  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 3, 0);
}

#if (!defined(HALL_SENSORS)||defined(POTENTIOMETER))
/**
  * @brief     Initializes the ADC MSP.
  * @param[in] hadc ADC handle pointer
  * @retval    None
  */
void HAL_ADC_MspInit(ADC_HandleTypeDef* hadc)
{
  GPIO_InitTypeDef GPIO_InitStruct;
  if(hadc->Instance==BSP_SIP_ADCx)
  { 
    /* Peripheral clock enable */
    BSP_SIP_ADCx_CLK_ENABLE();
    
    /* GPIOs Clocks Enable */
    BSP_SIP_ADCx_GPIO_CLK_ENABLE();

#ifdef BSP_BOARD_ADCx_GPIOA
    /* GPIO Port Clock Enable */
    __GPIOA_CLK_ENABLE();    
    /* GPIOs Init */
    GPIO_InitStruct.Pin = BSP_BOARD_ADCx_GPIOA;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
#endif
    
#ifdef BSP_BOARD_ADCx_GPIOB
    /* GPIO Port Clock Enable */
    __GPIOB_CLK_ENABLE();
    /* GPIOs Init */
    GPIO_InitStruct.Pin = BSP_BOARD_ADCx_GPIOB;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
#endif
    
    /* System interrupt init*/ 
    HAL_NVIC_SetPriority(BSP_SIP_ADCx_IRQn, BSP_SIP_ADCx_PRIORITY, 0);
    HAL_NVIC_EnableIRQ(BSP_SIP_ADCx_IRQn);
    
    /* Configuration of ADC channels */
    BSP_BOARD_ADC_INIT();
  }
}
#endif

#if (!defined(HALL_SENSORS)||defined(POTENTIOMETER))
/**
  * @brief     DeInitializes the ADC MSP.
  * @param[in] hadc ADC handle pointer
  * @retval    None
  */
void HAL_ADC_MspDeInit(ADC_HandleTypeDef* hadc)
{
  if(hadc->Instance==BSP_SIP_ADCx)
  {
    /* Peripheral clock disable */
    BSP_SIP_ADCx_CLK_DISABLE();

#ifdef BSP_BOARD_ADCx_GPIOA    
    HAL_GPIO_DeInit(GPIOA, BSP_BOARD_ADCx_GPIOA);
#endif
    
#ifdef BSP_BOARD_ADCx_GPIOB    
    HAL_GPIO_DeInit(GPIOB, BSP_BOARD_ADCx_GPIOB);
#endif

    /* Peripheral interrupt DeInit*/
    HAL_NVIC_DisableIRQ(BSP_SIP_ADCx_IRQn);
  }
}
#endif

/**
  * @brief     Timer MSP Initialization 
  * @param[in] htim_base timer handle pointer
  * @retval    None
  */
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* htim_base)
{
  GPIO_InitTypeDef GPIO_InitStruct;
  if(htim_base->Instance==BSP_SIP_HF_TIMx)
  { 
    /* Peripheral clock enable */
    BSP_SIP_HF_TIMx_CLK_ENABLE();

    /* GPIOs Clocks Enable */
    BSP_SIP_HF_GPIO_CLK_ENABLE();    

#ifdef VOLTAGE_MODE    
    /* HF_TIMx Break Input Configuration */
    GPIO_InitStruct.Pin = BSP_SIP_HF_TIMx_BKIN_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    GPIO_InitStruct.Alternate = BSP_SIP_HF_TIMx_BKIN_AF;
    HAL_GPIO_Init(BSP_SIP_HF_TIMx_BKIN_PORT, &GPIO_InitStruct);
#endif
    
    /* HF_TIMx OUTPUTs Configuration */
    GPIO_InitStruct.Pin = BSP_SIP_HF_TIMx_OUT_PIN;
    GPIO_InitStruct.Mode = BSP_SIP_HF_TIMx_MODE;
    GPIO_InitStruct.Pull = BSP_SIP_HF_TIMx_PULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    GPIO_InitStruct.Alternate = BSP_SIP_HF_TIMx_AF;
    HAL_GPIO_Init(BSP_SIP_HF_TIMx_OUT_PORT, &GPIO_InitStruct);    

    GPIO_InitStruct.Pin = BSP_SIP_HF_TIMx_OUTN_PIN;
    GPIO_InitStruct.Mode = BSP_SIP_HF_TIMx_MODE;
    GPIO_InitStruct.Pull = BSP_SIP_HF_TIMx_PULL;    
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
#ifdef COMPLEMENTARY_DRIVE    
    GPIO_InitStruct.Alternate = BSP_SIP_HF_TIMx_AF;
#endif
    HAL_GPIO_Init(BSP_SIP_HF_TIMx_OUTN_PORT, &GPIO_InitStruct);

    /* Enable and set priority for the HF_TIMx interrupt */
    HAL_NVIC_SetPriority(BSP_SIP_HF_TIMx_IRQn, BSP_SIP_HF_TIMx_PRIORITY, 0);
    HAL_NVIC_EnableIRQ(BSP_SIP_HF_TIMx_IRQn);
    
    /* Stop TIM during Breakpoint */
    BSP_SIP_HF_TIMx_FREEZE_DBGMCU();
  }  
  else if(htim_base->Instance==BSP_BOARD_LF_TIMx)
  {
    /* Peripheral clock enable */
    BSP_BOARD_LF_TIMx_CLK_ENABLE();   

    /* System interrupt init*/
    HAL_NVIC_SetPriority(BSP_BOARD_LF_TIMx_IRQn, BSP_BOARD_LF_TIMx_PRIORITY, 0);
    HAL_NVIC_EnableIRQ(BSP_BOARD_LF_TIMx_IRQn);
    
    /* Stop TIM during Breakpoint */
    BSP_BOARD_LF_TIMx_FREEZE_DBGMCU();    
  }
#ifndef VOLTAGE_MODE
  else if(htim_base->Instance==BSP_BOARD_REFx_TIMx)
  {
    /* Peripheral clock enable */
    BSP_BOARD_REFx_CLK_ENABLE();
  
    /* REFx GPIO Configuration */
    GPIO_InitStruct.Pin = BSP_BOARD_REFx_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
    GPIO_InitStruct.Alternate = BSP_BOARD_REFx_AF;
    HAL_GPIO_Init(BSP_BOARD_REFx_PORT, &GPIO_InitStruct);
    
    /* Stop TIM during Breakpoint */
    BSP_BOARD_REFx_FREEZE_DBGMCU();        
  }
#endif  
}

/**
  * @brief     Timer MSP DeInitialization 
  * @param[in] htim_base timer handle pointer
  * @retval    None
  */
void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* htim_base)
{
  if(htim_base->Instance==BSP_SIP_HF_TIMx)
  {
    /* Peripheral clock disable */
    BSP_SIP_HF_TIMx_CLK_DISABLE();
    
    HAL_GPIO_DeInit(BSP_SIP_HF_TIMx_OUT_PORT, BSP_SIP_HF_TIMx_OUT_PIN);
    HAL_GPIO_DeInit(BSP_SIP_HF_TIMx_OUTN_PORT, BSP_SIP_HF_TIMx_OUTN_PIN);
    
#ifdef VOLTAGE_MODE    
    HAL_GPIO_DeInit(BSP_SIP_HF_TIMx_BKIN_PORT, BSP_SIP_HF_TIMx_BKIN_PIN);
#endif    

    /* Peripheral interrupt DeInit*/
    HAL_NVIC_DisableIRQ(BSP_SIP_HF_TIMx_IRQn);
  } 
  else if(htim_base->Instance==BSP_BOARD_LF_TIMx)
  {
    /* Peripheral clock disable */
    BSP_BOARD_LF_TIMx_CLK_DISABLE();

    /* Peripheral interrupt DeInit */
    HAL_NVIC_DisableIRQ(BSP_BOARD_LF_TIMx_IRQn);
  }
#ifndef VOLTAGE_MODE
  else if(htim_base->Instance==BSP_BOARD_REFx_TIMx)
  {
    /* Peripheral clock disable */
    BSP_BOARD_REFx_CLK_DISABLE();
  
    /* REFx GPIO DeInit */
    HAL_GPIO_DeInit(BSP_BOARD_REFx_PORT,BSP_BOARD_REFx_PIN);
  }
#endif
}

#ifdef HALL_SENSORS
/**
  * @brief     Initializes the TIM Hall Sensor MSP.
  * @param[in] htimex_hallsensor TIM handle pointer
  * @retval    None
  */
void HAL_TIMEx_HallSensor_MspInit(TIM_HandleTypeDef* htimex_hallsensor)
{
  GPIO_InitTypeDef GPIO_InitStruct;
  if(htimex_hallsensor->Instance==BSP_BOARD_LF_TIMx)
  {
    /* TIMx Peripheral clock enable */
    BSP_BOARD_LF_TIMx_CLK_ENABLE();
  
    /* Enable Hall interface GPIOs Clock */
    BSP_BOARD_LF_GPIO_CLK_ENABLE();

    /* LF_TIMx Hall interface GPIOs Init */
    GPIO_InitStruct.Pin = BSP_BOARD_LF_TIMx_HALL_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    GPIO_InitStruct.Alternate = BSP_BOARD_LF_TIMx_HALL_AF;
    HAL_GPIO_Init(BSP_BOARD_LF_TIMx_HALL_PORT, &GPIO_InitStruct);

    /* System interrupt init*/
    HAL_NVIC_SetPriority(BSP_BOARD_LF_TIMx_IRQn, BSP_BOARD_LF_TIMx_PRIORITY, 0);
    HAL_NVIC_EnableIRQ(BSP_BOARD_LF_TIMx_IRQn);
  }
}

/**
  * @brief     DeInitializes TIM Hall Sensor MSP.
  * @param[in] htimex_hallsensor TIM handle pointer
  * @retval    None
  */
void HAL_TIMEx_HallSensor_MspDeInit(TIM_HandleTypeDef* htimex_hallsensor)
{
  if(htimex_hallsensor->Instance==BSP_BOARD_LF_TIMx)
  {
    /* Peripheral clock disable */
    BSP_BOARD_LF_TIMx_CLK_DISABLE();
    
    /* LF_TIMx Hall interface GPIOs DeInit */
    HAL_GPIO_DeInit(BSP_BOARD_LF_TIMx_HALL_PORT, BSP_BOARD_LF_TIMx_HALL_PIN);

    /* Peripheral interrupt DeInit */
    HAL_NVIC_DisableIRQ(BSP_BOARD_LF_TIMx_IRQn);
  }
}
#endif

/**
  * @brief     UART MSP Initialization 
  * @param[in] huart UART handle pointer
  * @retval    None
  */
void HAL_UART_MspInit(UART_HandleTypeDef* huart)
{
  static DMA_HandleTypeDef hdma_tx;  
  GPIO_InitTypeDef GPIO_InitStruct;
  if(huart->Instance==BSP_SIP_UART)
  { 
    /* DMA Clock Enable */
    BSP_SIP_DMA_CLK_ENABLE();
    
    /* Peripheral clock enable */
    BSP_SIP_UART_CLK_ENABLE();
    
    /* GPIO Ports Clock Enable */
    BSP_SIP_UART_GPIO_CLK_ENABLE();

    /* USART GPIO Configuration */
    GPIO_InitStruct.Pin = BSP_SIP_UART_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
    GPIO_InitStruct.Alternate = BSP_SIP_UART_AF;
    HAL_GPIO_Init(BSP_SIP_UART_PORT, &GPIO_InitStruct);

    /* DMA TX handle configuration */    
    hdma_tx.Instance = BSP_SIP_DMA;
    hdma_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_tx.Init.Mode = DMA_NORMAL;
    hdma_tx.Init.Priority = DMA_PRIORITY_LOW;
    HAL_DMA_Init(&hdma_tx);

    /* Associate the initialized DMA handle to the UART handle */
    __HAL_LINKDMA(huart, hdmatx, hdma_tx);
    
    /* System interrupt init*/
    HAL_NVIC_SetPriority(BSP_SIP_DMA_IRQn, BSP_SIP_DMA_PRIORITY, BSP_SIP_DMA_PRIORITY_SUB);
    HAL_NVIC_EnableIRQ(BSP_SIP_DMA_IRQn);    
    HAL_NVIC_SetPriority(BSP_SIP_UART_IRQn, BSP_SIP_UART_PRIORITY, 0);
    HAL_NVIC_EnableIRQ(BSP_SIP_UART_IRQn);
  }
}

/**
  * @brief     UART MSP DeInitialization 
  * @param[in] huart UART handle pointer
  * @retval    None
  */
void HAL_UART_MspDeInit(UART_HandleTypeDef* huart)
{
  static DMA_HandleTypeDef hdma_tx;
  if(huart->Instance==BSP_SIP_UART)
  {
    /* DMA Clock Disable */
    BSP_SIP_DMA_CLK_DISABLE();
    
    /* Peripheral clock disable */
    BSP_SIP_UART_CLK_DISABLE();
  
    /* UART GPIO Deconfiguration */
    HAL_GPIO_DeInit(BSP_SIP_UART_PORT, BSP_SIP_UART_PIN);
    
    /* De-Initialize the DMA Channel associated to transmission process */
    HAL_DMA_DeInit(&hdma_tx);    

    /* Peripheral interrupt DeInit*/
    HAL_NVIC_DisableIRQ(BSP_SIP_DMA_IRQn);
    HAL_NVIC_DisableIRQ(BSP_SIP_UART_IRQn);
  }
}

#if (!defined(HALL_SENSORS)||defined(POTENTIOMETER))
/**
  * @brief     ADC conversion complete callback 
  * @param[in] hadcADC handle pointer  
  * @retval None
  */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
#ifdef HALL_SENSORS
  MC_ADCx_SixStep_User();
#else
  MC_ADCx_SixStep_Bemf();
#endif
}
#endif

#ifdef HALL_SENSORS  
/**
  * @brief  Input Capture callback in non blocking mode 
  * @param[in] htim TIM IC handle pointer  
  * @retval None
  */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
  MC_TIMx_SixStep_CommutationEvent();
#ifdef TEST
  if (stop == 0)
#endif
  {
    MC_TIMx_SixStep_timebase();
  }
}

/**
  * @brief  Output Compare callback in non blocking mode 
  * @param[in] htim : TIM OC handle
  * @retval None
  */
void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (SIXSTEP_parameters.hall_ok == 0)
  {
#ifdef TEST    
    if (stop != 0)
    {
      return;
    }
#endif
    if (SIXSTEP_parameters.start_cnt != 0)
    {
      SIXSTEP_parameters.hall_ko_successive++;
      if (SIXSTEP_parameters.hall_ko_successive > HALL_KO_SUCCESSIVE_MAX)
      {
        MC_SixStep_Hall_Startup_Failure_Handler();
        return;
      }
      SIXSTEP_parameters.STATUS = STARTUP;
      if (SIXSTEP_parameters.speed_target > 0)
      {
        if(SIXSTEP_parameters.next_step_pos >= 6)
        {
          SIXSTEP_parameters.next_step_pos = 1;
        }
        else
        {
          SIXSTEP_parameters.next_step_pos++;
        }
      }
      else
      {
        if(SIXSTEP_parameters.next_step_pos <= 1)
        {
          SIXSTEP_parameters.next_step_pos = 6;
        }
        else
        {
          SIXSTEP_parameters.next_step_pos--;
        }
      }
      MC_SixStep_TABLE(SIXSTEP_parameters.next_step_pos);
    }
    else
    {
      MC_SixStep_Hall_Run_Failure_Handler();
    }
  }
}

/**
  * @brief     Commutation event callback
  * @param[in] htim TIM handle pointer  
  * @retval None
  */
void HAL_TIMEx_CommutationCallback(TIM_HandleTypeDef *htim)
{
  SIXSTEP_parameters.step_position=SIXSTEP_parameters.next_step_pos;
  SIXSTEP_parameters.hall_ok = 0;
}

#else
/**
  * @brief     timer period elapsed callback 
  * @param[in] htim TIM handle pointer  
  * @retval    None
  */  
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  MC_TIMx_SixStep_timebase();   
}

#endif

/**
  * @brief  Hall Break detection callback in non blocking mode 
  * @param[in] htim : TIM handle
  * @retval None
  */
void HAL_TIMEx_BreakCallback(TIM_HandleTypeDef *htim)
{
  SIXSTEP_parameters.overcurrent++;
}

/**
  * @brief     Systick callback
  * @retval None
  */
void HAL_SYSTICK_Callback()
{
    MC_SysTick_SixStep_MediumFrequencyTask();
}

/**
  * @brief     EXT callback
  * @param[in] GPIO_Pin 
  * @retval    None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == BSP_BOARD_USER1_BUTTON_PIN)
  {
    // USER1 button callback function
    MC_EXT_button_SixStep();
  }
  if (GPIO_Pin == BSP_BOARD_USER2_BUTTON_PIN)
  {
    // USER2 button callback function
  }
}

/**
  * @brief  UART transmit callback 
  * @param  huart
  * @retval None
*/
#ifdef UART_COMM
/**
  * @brief     UART transmit complete callback
  * @param[in] huart UART handle pointer 
  * @retval    None
  */
  void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
  {
    if (SIXSTEP_parameters.UART_TX_REPLY != 0)
    {
      SIXSTEP_parameters.UART_TX_REPLY = 0;
    }
    else if (SIXSTEP_parameters.UART_TX_DIFFERED_REPLY != 0)
    {
      SIXSTEP_parameters.UART_TX_DIFFERED_REPLY = 0;
      UART_Send_Reply();
    }    
    if (SIXSTEP_parameters.UART_CONTINUOUS_TX_SPEED_MODE != 0)
    {
      SIXSTEP_parameters.UART_CONTINUOUS_TX_SPEED_ALLOWED = 1;
    }
    else if (SIXSTEP_parameters.UART_CONTINUOUS_TX_BEMF_MODE != 0)
    {
      SIXSTEP_parameters.UART_CONTINUOUS_TX_BEMF_ALLOWED = 1;
    }
  }
#endif

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
