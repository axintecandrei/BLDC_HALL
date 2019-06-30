/**
 ******************************************************************************
 * @file    STEVAL-SPIN3202.c
 * @author  IPC Rennes
 * @version V1.0.0
 * @date    April 24th, 2017
 * @brief   This file provides a set of functions to manage the STEVAL-SPIN3202
 * board
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
#include "STEVAL-SPIN3202.h"
#ifdef STM32F031x6
#include "stm32f0xx_hal.h"
#endif

/* External variables --------------------------------------------------------*/
extern STSPIN32F0_MotorDriver_TypeDef STSPIN32F0MotorDriver;
/// STEVAL-SPIN3202 timer handler for HF_TIMx
extern TIM_HandleTypeDef HF_TIMx;
/// STEVAL-SPIN3202 timer handler for LF_TIMx
extern TIM_HandleTypeDef LF_TIMx;
/// STEVAL-SPIN3202 timer handler for reference PWM
extern TIM_HandleTypeDef REFx;
/// STEVAL-SPIN3202 adc handler
extern ADC_HandleTypeDef ADCx;
/// STEVAL-SPIN3202 uart handler
extern UART_HandleTypeDef huart;
   
/** @addtogroup DRIVERS     DRIVERS 
  * @brief  Driver Layer
  * @{ 
  */

/** @addtogroup BSP     BSP 
  * @brief  BSP Layer
  * @{ 
  */

/** @addtogroup STEVAL-SPIN3202   STEVAL-SPIN3202
  * @brief  STSPIN32F0 evaluation board
  * @{ 
  */

/******************************************************//**
 * @brief     Turns selected LED On
 * @retval    None
 **********************************************************/
void BSP_BOARD_FAULT_LED_ON(void)
{
  HAL_GPIO_WritePin(BSP_BOARD_FAULT_LED_PORT,BSP_BOARD_FAULT_LED_PIN,GPIO_PIN_RESET);
}

/******************************************************//**
 * @brief     Turns selected LED Off
 * @retval    None
 **********************************************************/
void BSP_BOARD_FAULT_LED_OFF(void)
{
  HAL_GPIO_WritePin(BSP_BOARD_FAULT_LED_PORT,BSP_BOARD_FAULT_LED_PIN,GPIO_PIN_SET);
}

/******************************************************//**
 * @brief     Init the STM32 ADC register
 * @retval    None
 **********************************************************/
void BSP_BOARD_ADC_INIT(void)
{
#if !defined(HALL_SENSORS)||defined(POTENTIOMETER)||defined(CURRENT_SENSE_ADC)||defined(VBUS_SENSE_ADC)||defined(TEMP_SENSE_ADC)
    ADC_ChannelConfTypeDef sConfig;
#endif
  
 /******************** REGULAR CHANNELS CONFIGURATION *************************/
#ifndef HALL_SENSORS
  sConfig.Channel = ADC_Bemf_CH1; /* BEMF feedback phase A */    
  sConfig.SamplingTime = ADC_Bemf_CH1_ST;
  HAL_ADC_ConfigChannel(&ADCx, &sConfig);
  sConfig.Channel = ADC_Bemf_CH2; /* BEMF feedback phase B */
  sConfig.SamplingTime = ADC_Bemf_CH2_ST;
  HAL_ADC_ConfigChannel(&ADCx, &sConfig);
  sConfig.Channel = ADC_Bemf_CH3; /* BEMF feedback phase C */
  sConfig.SamplingTime = ADC_Bemf_CH3_ST;
  HAL_ADC_ConfigChannel(&ADCx, &sConfig);
#endif
#ifdef POTENTIOMETER     
  sConfig.Channel = ADC_CH_1; /* Potentiometer */
  sConfig.SamplingTime = ADC_CH_1_ST;
  sConfig.Rank = 1;
  HAL_ADC_ConfigChannel(&ADCx, &sConfig);
#endif
#ifdef CURRENT_SENSE_ADC  
  sConfig.Channel = ADC_CH_2; /* Current feedabck */
  sConfig.SamplingTime = ADC_CH_2_ST;
  HAL_ADC_ConfigChannel(&ADCx, &sConfig);
#endif   
#ifdef VBUS_SENSE_ADC    
  sConfig.Channel = ADC_CH_3; /* Bus voltage */
  sConfig.SamplingTime = ADC_CH_3_ST;
  HAL_ADC_ConfigChannel(&ADCx, &sConfig);
#endif
#ifdef TEMP_SENSE_ADC
  sConfig.Channel = ADC_CH_4; /* Temperature feedback */
  sConfig.SamplingTime = ADC_CH_4_ST;
  HAL_ADC_ConfigChannel(&ADCx, &sConfig);
#endif  
  /****************************************************************************/
}

#ifndef VOLTAGE_MODE
/******************************************************//**
 * @brief     Enable the Current Reference generation
 * @retval    None
 **********************************************************/
void MC_SixStep_Current_Reference_Start()
{
  HAL_TIM_PWM_Start(&REFx, BSP_BOARD_REFx_CHANNEL);
}

/******************************************************//**
 * @brief     Disable the Current Reference generation
 * @retval    None
 **********************************************************/
void MC_SixStep_Current_Reference_Stop()
{
  REFx.Instance->BSP_BOARD_REFx_CCR = 0;
  HAL_TIM_PWM_Stop(&REFx, BSP_BOARD_REFx_CHANNEL);
}

/******************************************************//**
 * @brief     Set the value for Current Reference
 * @retval    None
 **********************************************************/
void MC_SixStep_Current_Reference_Setvalue(uint16_t Iref)
{
  REFx.Instance->BSP_BOARD_REFx_CCR =
    (uint32_t)((UPPER_OUT_LIMIT-Iref) * REFx.Instance->ARR)>>UPPER_OUT_SHIFT;
}
#endif

/******************************************************//**
 * @brief     Enable channels CH1 and CH2 for STSPIN32F0
 * @retval    None
 **********************************************************/
void MC_SixStep_EnableInput_CH1_E_CH2_E_CH3_D(uint8_t steps1to3)
{
   STSPIN32F0MotorDriver.EnableInput_CH1_E_CH2_E_CH3_D(steps1to3);
}

/******************************************************//**
 * @brief     Enable channels CH1 and CH3 for STSPIN32F0
 * @retval    None
 **********************************************************/
void MC_SixStep_EnableInput_CH1_E_CH2_D_CH3_E(uint8_t steps1to3)
{
  STSPIN32F0MotorDriver.EnableInput_CH1_E_CH2_D_CH3_E(steps1to3);
}

/******************************************************//**
 * @brief     Enable channels CH2 and CH3 for STSPIN32F0
 * @retval    None
 **********************************************************/
void MC_SixStep_EnableInput_CH1_D_CH2_E_CH3_E(uint8_t steps1to3)
{
  STSPIN32F0MotorDriver.EnableInput_CH1_D_CH2_E_CH3_E(steps1to3);
}

/******************************************************//**
 * @brief     Disable channels CH1, CH2 and CH3 for STSPIN32F0
 * @retval    None
 **********************************************************/
void MC_SixStep_DisableInput_CH1_D_CH2_D_CH3_D(void)
{
  STSPIN32F0MotorDriver.DisableInput_CH1_D_CH2_D_CH3_D();
}

/******************************************************//**
 * @brief     Set the Duty Cycle value for CH1
 * @retval    None
 **********************************************************/
void MC_SixStep_HF_TIMx_SetDutyCycle_CH1(uint16_t CCR_value)
{ 
  STSPIN32F0MotorDriver.HF_TIMx_SetDutyCycle_CH1(CCR_value);
}

/******************************************************//**
 * @brief     Set the Duty Cycle value for CH2
 * @retval    None
 **********************************************************/
void MC_SixStep_HF_TIMx_SetDutyCycle_CH2(uint16_t CCR_value)
{ 
  STSPIN32F0MotorDriver.HF_TIMx_SetDutyCycle_CH2(CCR_value);
}

/******************************************************//**
 * @brief     Set the Duty Cycle value for CH3
 * @retval    None
 **********************************************************/
void MC_SixStep_HF_TIMx_SetDutyCycle_CH3(uint16_t CCR_value)
{ 
  STSPIN32F0MotorDriver.HF_TIMx_SetDutyCycle_CH3(CCR_value);
}

/******************************************************//**
 * @brief     Set the value for the voltage to be applied on motor phases
 * @retval    None
 **********************************************************/
void MC_SixStep_HF_TIMx_SetDutyCycle(uint16_t Iref, uint8_t stepNumber)
{
  STSPIN32F0MotorDriver.HF_TIMx_SetDutyCycle(Iref, stepNumber);
}

/******************************************************//**
 * @brief     Select the new ADC Channel
 * @param[in] adc_ch 
 * @retval    None
 **********************************************************/
void MC_SixStep_ADC_Channel(uint32_t adc_ch)
{ 
  STSPIN32F0MotorDriver.ADC_Channel(adc_ch);
}

/**
  * @}  end STEVAL-SPIN3202
  */

/**
  * @}  end BSP
  */

/**
  * @}  end DRIVERS
  */
