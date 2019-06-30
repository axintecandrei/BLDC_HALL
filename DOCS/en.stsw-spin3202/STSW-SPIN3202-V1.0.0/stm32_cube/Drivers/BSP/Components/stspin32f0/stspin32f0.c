/**
 ******************************************************************************
 * @file    stspin32f0.c
 * @author  IPC Rennes
 * @version V1.0.0
 * @date    April 24th, 2017
 * @brief   This file provides a set of functions to manage STSPIN32F0 driver
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
#include "stspin32f0.h"

/** @addtogroup DRIVERS     DRIVERS 
  * @brief  Driver Layer
  * @{ 
  */

/** @addtogroup BSP    BSP
  * @brief  BSP Layer
  * @{ 
  */

/** @addtogroup STSPIN32F0_SIP    STSPIN32F0_SIP
  * @brief  STSPIN32F0 SIP driver section
  * @{ 
  */

/** @defgroup STSPIN32F0_SIP_Exported_Variables STSPIN32F0 SIP Exported Variables
  * @{
  */       
/// EVALSTSPIN32F0 timer handler for HF_TIMx
TIM_HandleTypeDef HF_TIMx;

/// EVALSTSPIN32F0 timer handler for LF_TIMx
TIM_HandleTypeDef LF_TIMx;

/// EVALSTSPIN32F0 timer handler for reference PWM
TIM_HandleTypeDef REFx;

/// EVALSTSPIN32F0 adc handler
ADC_HandleTypeDef ADCx;

/// EVALSTSPIN32F0 uart handler
UART_HandleTypeDef huart;

/**
  * @}
  */     
    
/** @defgroup STSPIN32F0MotorDriver    STSPIN32F0MotorDriver
  *  @{
  */
/// STSPIN32F0 motor driver functions pointer structure 
STSPIN32F0_MotorDriver_TypeDef STSPIN32F0MotorDriver =
{
  EnableInput_CH1_E_CH2_E_CH3_D,
  EnableInput_CH1_E_CH2_D_CH3_E,
  EnableInput_CH1_D_CH2_E_CH3_E,  
  DisableInput_CH1_D_CH2_D_CH3_D,
  HF_TIMx_SetDutyCycle_CH1,
  HF_TIMx_SetDutyCycle_CH2,
  HF_TIMx_SetDutyCycle_CH3,
  HF_TIMx_SetDutyCycle,
  Overcurrent_Selection,
  Overcurrent_Threshold_Setvalue,
  ADC_Channel,
  Get_UART_Data
};

/**
  * @}
  */

/******************************************************//**
 * @brief     This function enables channel 1 and 2, disables channel 3
 * @param[in] steps1to3 non zero for steps 1 to 3, zero for steps 4 to 6
 * @retval    None
 **********************************************************/
void EnableInput_CH1_E_CH2_E_CH3_D(uint8_t steps1to3)
{
#ifdef COMPLEMENTARY_DRIVE
  HAL_TIM_PWM_Start(&HF_TIMx,BSP_SIP_HF_TIMx_CH1);           //TIM1_CH1 ENABLE
  HAL_TIMEx_PWMN_Start(&HF_TIMx,BSP_SIP_HF_TIMx_CH1) ;
  
  HAL_TIM_PWM_Start(&HF_TIMx,BSP_SIP_HF_TIMx_CH2);           //TIM1_CH2 ENABLE  
  HAL_TIMEx_PWMN_Start(&HF_TIMx,BSP_SIP_HF_TIMx_CH2) ;

  HAL_TIM_PWM_Stop(&HF_TIMx,BSP_SIP_HF_TIMx_CH3);            //TIM1_CH3 DISABLE  
  HAL_TIMEx_PWMN_Stop(&HF_TIMx,BSP_SIP_HF_TIMx_CH3) ;  
#else         
  if (steps1to3 != 0)
  {
    HAL_TIM_PWM_Start(&HF_TIMx,BSP_SIP_HF_TIMx_CH1);
    HAL_TIM_PWM_Stop(&HF_TIMx,BSP_SIP_HF_TIMx_CH2);
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13,GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,GPIO_PIN_SET);
  }
  else
  {
    HAL_TIM_PWM_Stop(&HF_TIMx,BSP_SIP_HF_TIMx_CH1);
    HAL_TIM_PWM_Start(&HF_TIMx,BSP_SIP_HF_TIMx_CH2);
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13,GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,GPIO_PIN_RESET);      
  }
  HAL_TIM_PWM_Stop(&HF_TIMx,BSP_SIP_HF_TIMx_CH3);             
  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,GPIO_PIN_RESET);  
#endif
}

/******************************************************//**
 * @brief     This function enables channel 1 and 3, disables channel 2
 * @param[in] steps1to3 non zero for steps 1 to 3, zero for steps 4 to 6
  * @retval None
 **********************************************************/
void EnableInput_CH1_E_CH2_D_CH3_E(uint8_t steps1to3)
{
#ifdef COMPLEMENTARY_DRIVE
  HAL_TIM_PWM_Start(&HF_TIMx,BSP_SIP_HF_TIMx_CH1);           //TIM1_CH1 ENABLE 
  HAL_TIMEx_PWMN_Start(&HF_TIMx,BSP_SIP_HF_TIMx_CH1) ;
  
  HAL_TIM_PWM_Stop(&HF_TIMx,BSP_SIP_HF_TIMx_CH2);            //TIM1_CH2  DISABLE 
  HAL_TIMEx_PWMN_Stop(&HF_TIMx,BSP_SIP_HF_TIMx_CH2) ;

  HAL_TIM_PWM_Start(&HF_TIMx,BSP_SIP_HF_TIMx_CH3);           //TIM1_CH3 ENABLE  
  HAL_TIMEx_PWMN_Start(&HF_TIMx,BSP_SIP_HF_TIMx_CH3) ;    
#else
  if (steps1to3 != 0)
  {
    HAL_TIM_PWM_Start(&HF_TIMx,BSP_SIP_HF_TIMx_CH1);
    HAL_TIM_PWM_Stop(&HF_TIMx,BSP_SIP_HF_TIMx_CH3);
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13,GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,GPIO_PIN_SET);    
  }
  else
  {
    HAL_TIM_PWM_Stop(&HF_TIMx,BSP_SIP_HF_TIMx_CH1);
    HAL_TIM_PWM_Start(&HF_TIMx,BSP_SIP_HF_TIMx_CH3);
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13,GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,GPIO_PIN_RESET);  
  }
  HAL_TIM_PWM_Stop(&HF_TIMx,BSP_SIP_HF_TIMx_CH2);             
  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,GPIO_PIN_RESET);     
#endif
}

/******************************************************//**
 * @brief     This function enables channel 2 and 3, disables channel 1
 * @param[in] steps1to3 non zero for steps 1 to 3, zero for steps 4 to 6
  * @retval None
 **********************************************************/
void EnableInput_CH1_D_CH2_E_CH3_E(uint8_t steps1to3)
{
#ifdef COMPLEMENTARY_DRIVE
  HAL_TIM_PWM_Stop(&HF_TIMx,BSP_SIP_HF_TIMx_CH1);           //TIM1_CH1 ENABLE 
  HAL_TIMEx_PWMN_Stop(&HF_TIMx,BSP_SIP_HF_TIMx_CH1) ;
  
  HAL_TIM_PWM_Start(&HF_TIMx,BSP_SIP_HF_TIMx_CH2);            //TIM1_CH2  DISABLE 
  HAL_TIMEx_PWMN_Start(&HF_TIMx,BSP_SIP_HF_TIMx_CH2) ;

  HAL_TIM_PWM_Start(&HF_TIMx,BSP_SIP_HF_TIMx_CH3);           //TIM1_CH3 ENABLE  
  HAL_TIMEx_PWMN_Start(&HF_TIMx,BSP_SIP_HF_TIMx_CH3) ;    
#else
  if (steps1to3 != 0)
  {
    HAL_TIM_PWM_Start(&HF_TIMx,BSP_SIP_HF_TIMx_CH2);
    HAL_TIM_PWM_Stop(&HF_TIMx,BSP_SIP_HF_TIMx_CH3);
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,GPIO_PIN_SET);    
  }
  else
  {
    HAL_TIM_PWM_Stop(&HF_TIMx,BSP_SIP_HF_TIMx_CH2);
    HAL_TIM_PWM_Start(&HF_TIMx,BSP_SIP_HF_TIMx_CH3);
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,GPIO_PIN_RESET);  
  }
  HAL_TIM_PWM_Stop(&HF_TIMx,BSP_SIP_HF_TIMx_CH1);             
  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13,GPIO_PIN_RESET);     
#endif
}

/******************************************************//**
 * @brief     This function disables channel 1, 2 and 3
  * @retval None
 **********************************************************/
void DisableInput_CH1_D_CH2_D_CH3_D(void)
{
#ifdef COMPLEMENTARY_DRIVE
  HAL_TIM_PWM_Stop(&HF_TIMx,BSP_SIP_HF_TIMx_CH1);           //TIM1_CH1 DISABLE 
  HAL_TIMEx_PWMN_Stop(&HF_TIMx,BSP_SIP_HF_TIMx_CH1);
  
  HAL_TIM_PWM_Stop(&HF_TIMx,BSP_SIP_HF_TIMx_CH2);           //TIM1_CH2  DISABLE 
  HAL_TIMEx_PWMN_Stop(&HF_TIMx,BSP_SIP_HF_TIMx_CH2);

  HAL_TIM_PWM_Stop(&HF_TIMx,BSP_SIP_HF_TIMx_CH3);           //TIM1_CH3 DISABLE  
  HAL_TIMEx_PWMN_Stop(&HF_TIMx,BSP_SIP_HF_TIMx_CH3);  
#else
  HAL_TIM_PWM_Stop(&HF_TIMx,BSP_SIP_HF_TIMx_CH1);
  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13,GPIO_PIN_RESET);
  
  HAL_TIM_PWM_Stop(&HF_TIMx,BSP_SIP_HF_TIMx_CH2); 
  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,GPIO_PIN_RESET);

  HAL_TIM_PWM_Stop(&HF_TIMx,BSP_SIP_HF_TIMx_CH3);  
  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,GPIO_PIN_RESET); 
#endif
}

/******************************************************//**
 * @brief     This function sets the Duty Cycle value for CH1
 * @param[in] CCR_value Capture Compare Register value
  * @retval None
 **********************************************************/
void HF_TIMx_SetDutyCycle_CH1(uint16_t CCR_value)
{ 
  HF_TIMx.Instance->BSP_SIP_HF_TIMx_CCR1 = CCR_value;
}

/******************************************************//**
 * @brief     This function sets the Duty Cycle value for CH2
 * @param[in] CCR_value Capture Compare Register value
  * @retval None
 **********************************************************/
void HF_TIMx_SetDutyCycle_CH2(uint16_t CCR_value)
{ 
  HF_TIMx.Instance->BSP_SIP_HF_TIMx_CCR2 = CCR_value;
}

/******************************************************//**
 * @brief     This function sets the Duty Cycle value for CH3
 * @param[in] CCR_value Capture Compare Register value
  * @retval None
 **********************************************************/
void HF_TIMx_SetDutyCycle_CH3(uint16_t CCR_value)
{ 
  HF_TIMx.Instance->BSP_SIP_HF_TIMx_CCR3 = CCR_value;
}

/******************************************************//**
 * @brief     Set the value for the voltage to be applied on motor phases
 * @param[in] Iref
 * @retval    None
 **********************************************************/
void HF_TIMx_SetDutyCycle(uint16_t Iref, uint8_t stepNumber)
{
  uint32_t volatile* ptr = &HF_TIMx.Instance->BSP_SIP_HF_TIMx_CCR1;
  ptr += ((--stepNumber)>>1);
  *ptr = Iref;
}

/******************************************************//**
  * @brief  Select the overcurrent comparator output signal visibility
  * (0) Visible only to MCU 
  * (1) Visible to MCU and acts on gate driver logic
 * @param[in] ocSel
  * @retval None
 **********************************************************/
void Overcurrent_Selection(uint8_t ocSel)
{
  HAL_GPIO_WritePin(BSP_SIP_OC_SEL_PORT,BSP_SIP_OC_SEL_PIN,(GPIO_PinState)ocSel);
}

/******************************************************//**
  * @brief  Set the overcurrent threshold value and commands standby mode
  * (0) Standy mode
  * (1) 100mV
  * (2) 250mV
  * (3) 500mV
 * @param[in] ocThres
  * @retval None
 **********************************************************/
void Overcurrent_Threshold_Setvalue(uint8_t ocThres)
{
  switch ((overcurrentThresholds_t)ocThres)
  {
    case BSP_SIP_OC_TH_STBY:
      HAL_GPIO_WritePin(GPIOF,GPIO_PIN_6,GPIO_PIN_RESET);
      HAL_GPIO_WritePin(GPIOF,GPIO_PIN_7,GPIO_PIN_RESET);
      break;
    case BSP_SIP_OC_TH_100mV:
      HAL_GPIO_WritePin(GPIOF,GPIO_PIN_6,GPIO_PIN_RESET);
      HAL_GPIO_WritePin(GPIOF,GPIO_PIN_7,GPIO_PIN_SET);
      break;
    case  BSP_SIP_OC_TH_250mV:
      HAL_GPIO_WritePin(GPIOF,GPIO_PIN_6,GPIO_PIN_SET);
      HAL_GPIO_WritePin(GPIOF,GPIO_PIN_7,GPIO_PIN_RESET);
      break;
    case  BSP_SIP_OC_TH_500mV:
      HAL_GPIO_WritePin(GPIOF,GPIO_PIN_6,GPIO_PIN_SET);
      HAL_GPIO_WritePin(GPIOF,GPIO_PIN_7,GPIO_PIN_SET);
      break;
  default:
      break;
  }
}

/******************************************************//**
 * @brief     Select the new ADC Channel
 * @param[in] adc_ch 
 * @retval    None
 **********************************************************/
void ADC_Channel(uint32_t adc_ch)
{ 
#ifdef HALL_SENSORS
  __HAL_ADC_ENABLE_IT(&ADCx, (ADC_IT_EOC | ADC_IT_EOS | ADC_IT_OVR));
#endif
  ADCx.Instance->CR |= ADC_CR_ADSTP;
  while(ADCx.Instance->CR & ADC_CR_ADSTP);   
  /* Regular sequence configuration */
  /* Set the channel selection register from the selected channel */
  ADCx.Instance->CHSELR = __HAL_ADC_CHSELR_CHANNEL(adc_ch);
  ADCx.Instance->CR |= ADC_CR_ADSTART;
}    

/******************************************************//**
 * @brief     Get the UART value from DR register
 * @retval    uint32_t
 **********************************************************/
uint32_t Get_UART_Data()
{
  return (huart.Instance->RDR);
}

/**
  * @}  end STSPIN32F0_SIP 
  */

/**
  * @}  end BSP 
  */

/**
  * @}  end DRIVERS
  */

