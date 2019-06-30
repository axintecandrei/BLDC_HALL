/**
 ******************************************************************************
 * @file    main_32F0.c
 * @author  IPC Rennes
 * @version V1.0.0
 * @date    April 24th, 2017
 * @brief   This file provides a set of functions needed to configure STM32 MCU
 ******************************************************************************
 * @attention
 *
  * <h2><center>&copy; COPYRIGHT(c) 2017 STMicroelectronics</center></h2>
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

/* Variables -----------------------------------------------------------------*/
extern STSPIN32F0_MotorDriver_TypeDef STSPIN32F0MotorDriver;
extern TIM_HandleTypeDef HF_TIMx;
extern TIM_HandleTypeDef LF_TIMx;
#ifndef VOLTAGE_MODE
extern TIM_HandleTypeDef REFx;
#endif
extern ADC_HandleTypeDef ADCx;
extern UART_HandleTypeDef huart;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
#if (!defined(HALL_SENSORS)||defined(POTENTIOMETER))
static void MX_ADC_Init(void);
#endif
static void MX_HF_TIMx_Init(void);
#ifndef VOLTAGE_MODE
static void MX_REFx_Init(void);
#endif
static void MX_LF_TIMx_Init(void);
static void MX_UART_Init(void);

int main(void)
{

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
#if (!defined(HALL_SENSORS)||defined(POTENTIOMETER))
  MX_ADC_Init();
#endif
  MX_HF_TIMx_Init();
#ifndef VOLTAGE_MODE
  MX_REFx_Init();
#endif
  MX_LF_TIMx_Init();
  MX_UART_Init();

 /* **************************************************************************** 
  ==============================================================================   
            ###### This function initializes 6-Step lib ######
  ============================================================================== 
  **************************************************************************** */     
  MC_SixStep_INIT();
  /****************************************************************************/  
  
  /* Infinite loop */
  while (1)
  {
/*! **************************************************************************
  ==============================================================================   
            ###### How to use the 6Step FW Example project ######
  ==============================================================================     
  This workspace contains the middleware layer with Motor Control library to 
  drive a motor connected on STEVAL-SPIN3202 board performing a 6-step control
  algorithm allowing the motor speed regulation through UART commands or a
  potentiometer.
  The 6-step algorithm is configurable:
  - Voltage mode and sensorless BEMF detection
  - Current sensing mode 1shunt and sensorless BEMF detection
  - Voltage mode and hall sensors driving
  The workspace is provided for STSPIN32F0A in two different configurations:
   - Uart configuration
   - Potentiometer configuration
  The "Uart" configuration enables the communication protocol with external PC
  terminal.
  The "Potentiometer" configuration enables the ADC to set the speed according
  to the potentiometer tuned value.
  In both configurations the motor can be start and stop with the USER1 button.
    
  A list of APIs in 6Step_Lib.h is provided to send user command to the 6Step
  library, for instance:
    
    (#)  MC_StartMotor() -> Start the motor
    
    (#)  MC_StoptMotor() -> Stop the motor
    
    (#)  MC_Set_Speed(...) -> Set the new motor speed
 
  The MC_SixStep_param_32F0.h contains the files names of the MC parameters
  files. The user can add its own parameter file using one of the existing file
  as an example.
 
  ==============================================================================   
                       ###### USER SPACE ######
  ==============================================================================      
  *****************************************************************************/    
      
   
  /****************************************************************************/
  }
}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI14;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.HSI14CalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1);

  __SYSCFG_CLK_ENABLE();

}

#if (!defined(HALL_SENSORS)||defined(POTENTIOMETER))
/* ADC init function */
void MX_ADC_Init(void)
{
  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
    */
  ADCx.Instance = BSP_SIP_ADCx;
  ADCx.Init.ClockPrescaler = ADC_CLOCK_ASYNC;
  ADCx.Init.Resolution = ADC_RESOLUTION12b;
  ADCx.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  ADCx.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  ADCx.Init.EOCSelection = EOC_SINGLE_CONV;
  ADCx.Init.LowPowerAutoWait = DISABLE;
  ADCx.Init.LowPowerAutoPowerOff = DISABLE;
  ADCx.Init.ContinuousConvMode = DISABLE;
  ADCx.Init.DiscontinuousConvMode = DISABLE;
  ADCx.Init.ExternalTrigConv = BSP_SIP_ADCx_EXT_TRIG_CONV;
  ADCx.Init.ExternalTrigConvEdge = BSP_SIP_ADCx_EXT_TRIG_EDGE;
  ADCx.Init.DMAContinuousRequests = DISABLE;
  ADCx.Init.Overrun = OVR_DATA_PRESERVED;
  HAL_ADC_Init(&ADCx);

    /**Configure for the selected ADC regular channel to be converted. 
    */
  sConfig.Channel = ADC_CH_INIT;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_CH_INIT_ST;
  HAL_ADC_ConfigChannel(&ADCx, &sConfig);
}
#endif

/* HF timer init function */
void MX_HF_TIMx_Init(void)
{
  TIM_SlaveConfigTypeDef sSlaveConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;
  TIM_OC_InitTypeDef sConfigOC;
  
  HF_TIMx.Instance = BSP_SIP_HF_TIMx;
  HF_TIMx.Init.Prescaler = HF_TIMX_PSC;
  HF_TIMx.Init.CounterMode = BSP_SIP_HF_TIMx_COUNTER_MODE;
  HF_TIMx.Init.Period = HF_TIMX_ARR;
  HF_TIMx.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HF_TIMx.Init.RepetitionCounter = 0;
  HAL_TIM_Base_Init(&HF_TIMx);
  
  sSlaveConfig.SlaveMode = BSP_BOARD_HF_TIMX_SLAVE_MODE;
  sSlaveConfig.InputTrigger = BSP_BOARD_HF_TIMX_TS_ITR;
  HAL_TIM_SlaveConfigSynchronization(&HF_TIMx, &sSlaveConfig);
  
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_ENABLE;
  HAL_TIMEx_MasterConfigSynchronization(&HF_TIMx, &sMasterConfig);

  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = DEAD_TIME;
  sBreakDeadTimeConfig.BreakState = BSP_BOARD_HF_TIMX_BREAK_STATE;
  sBreakDeadTimeConfig.BreakPolarity = BSP_BOARD_HF_TIMX_BREAK_POL;
  sBreakDeadTimeConfig.AutomaticOutput = BSP_BOARD_HF_TIMX_BREAK_OUT;
  HAL_TIMEx_ConfigBreakDeadTime(&HF_TIMx, &sBreakDeadTimeConfig);

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  sConfigOC.Pulse = PULSE;
  HAL_TIM_PWM_ConfigChannel(&HF_TIMx, &sConfigOC, TIM_CHANNEL_1);
  HAL_TIM_PWM_ConfigChannel(&HF_TIMx, &sConfigOC, TIM_CHANNEL_2);
  HAL_TIM_PWM_ConfigChannel(&HF_TIMx, &sConfigOC, TIM_CHANNEL_3);
}

#ifndef VOLTAGE_MODE
/* Reference PWM init function */
void MX_REFx_Init(void)
{
  TIM_SlaveConfigTypeDef sSlaveConfig;  
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  REFx.Instance = BSP_BOARD_REFx_TIMx;
  REFx.Init.Prescaler = 0;
  REFx.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
  REFx.Init.Period = 479;
  REFx.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_Base_Init(&REFx);

  HAL_TIM_PWM_Init(&REFx);
  
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_TRIGGER;
  sSlaveConfig.InputTrigger = BSP_BOARD_REFx_TS_ITR;
  HAL_TIM_SlaveConfigSynchronization(&REFx, &sSlaveConfig);  

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&REFx, &sMasterConfig);

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  HAL_TIM_PWM_ConfigChannel(&REFx, &sConfigOC, BSP_BOARD_REFx_CHANNEL);
}
#endif

#ifdef HALL_SENSORS
/* LF timer init function */
void MX_LF_TIMx_Init(void)
{
  TIM_HallSensor_InitTypeDef sHallSensorConfig;

  LF_TIMx.Instance = BSP_BOARD_LF_TIMx;
  LF_TIMx.Init.Prescaler = LF_TIMX_PSC;
  LF_TIMx.Init.CounterMode = TIM_COUNTERMODE_UP;
  LF_TIMx.Init.Period = LF_TIMX_ARR;
  LF_TIMx.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  sHallSensorConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sHallSensorConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sHallSensorConfig.IC1Filter = 8;
  sHallSensorConfig.Commutation_Delay = LF_TIMx.Init.Period;
  HAL_TIMEx_HallSensor_Init(&LF_TIMx, &sHallSensorConfig);
}
#else
/* LF_TIMx init function */
void MX_LF_TIMx_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;  
  
  LF_TIMx.Instance = BSP_BOARD_LF_TIMx;
  LF_TIMx.Init.Prescaler = LF_TIMX_PSC;
  LF_TIMx.Init.CounterMode = TIM_COUNTERMODE_UP;
  LF_TIMx.Init.Period = LF_TIMX_ARR;
  LF_TIMx.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;  
  HAL_TIM_Base_Init(&LF_TIMx);
  
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  HAL_TIM_ConfigClockSource(&LF_TIMx, &sClockSourceConfig); 
  
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_ENABLE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_ENABLE;
  HAL_TIMEx_MasterConfigSynchronization(&LF_TIMx, &sMasterConfig);
}
#endif

/* UART init function */
void MX_UART_Init(void)
{
  huart.Instance = BSP_SIP_UART;
  huart.Init.BaudRate = BSP_SIP_UART_BAUD_RATE;
  huart.Init.WordLength = UART_WORDLENGTH_8B;
  huart.Init.StopBits = UART_STOPBITS_1;
  huart.Init.Parity = UART_PARITY_NONE;
  huart.Init.Mode = UART_MODE_TX_RX;
  huart.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart.Init.OverSampling = UART_OVERSAMPLING_16;
  huart.Init.OneBitSampling = UART_ONEBIT_SAMPLING_DISABLED ;
  huart.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  HAL_UART_Init(&huart);
}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __GPIOF_CLK_ENABLE();
  __GPIOA_CLK_ENABLE();
  __GPIOB_CLK_ENABLE();

  /*Configure the START STOP BUTTON GPIO pin (USER1) */
  GPIO_InitStruct.Pin = BSP_BOARD_START_STOP_BUTTON_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(BSP_BOARD_START_STOP_BUTTON_PORT, &GPIO_InitStruct);
  
  /*Configure the FAULT LED GPIO pin (USER2) */
  GPIO_InitStruct.Pin = BSP_BOARD_FAULT_LED_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(BSP_BOARD_FAULT_LED_PORT, &GPIO_InitStruct);
  BSP_BOARD_FAULT_LED_OFF();
  
  /*Configure overcurrent threshold GPIO pins */
  GPIO_InitStruct.Pin = BSP_SIP_OC_TH_STBY_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(BSP_SIP_OC_TH_STBY_PORT, &GPIO_InitStruct);
  
  /*Set the overcurrent threshold */
  STSPIN32F0MotorDriver.Overcurrent_Threshold_Setvalue(BSP_SIP_OC_TH_500mV);
  
  /*Configure the overcurrent selection GPIO pin  */
  GPIO_InitStruct.Pin = BSP_SIP_OC_SEL_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(BSP_SIP_OC_SEL_PORT, &GPIO_InitStruct);
  
  /*Set the overcurrent selection */
  STSPIN32F0MotorDriver.Overcurrent_Selection(BSP_SIP_SEL_VIS_FROM_MCU_AND_GATE_LOGIC);

#ifdef PWM_ON_BEMF_SENSING  
  /*Configure GPIO_BEMF */
  GPIO_InitStruct.Pin = GPIO_CH_BEMF;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIO_PORT_BEMF, &GPIO_InitStruct);
  HAL_GPIO_WritePin(GPIO_PORT_BEMF,GPIO_CH_BEMF,GPIO_PIN_SET);
#endif  
  
  /*Configure debug GPIO pins */
#if (GPIO_ZERO_CROSS!=0)
  GPIO_InitStruct.Pin = GPIO_CH_ZCR;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIO_PORT_ZCR, &GPIO_InitStruct);
#endif
  
#if (GPIO_COMM!=0)
  GPIO_InitStruct.Pin = GPIO_CH_COMM;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIO_PORT_COMM, &GPIO_InitStruct);
#endif

#ifdef PWM_ON_BEMF_SENSING  
#if (GPIO_ZCR_MODE!=0)
  GPIO_InitStruct.Pin = GPIO_CH_ZCR_MODE;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIO_PORT_ZCR_MODE, &GPIO_InitStruct);
#endif
#endif
  
  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
