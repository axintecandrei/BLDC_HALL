/**
 ******************************************************************************
 * @file    stspin32f0.h
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STSPIN32F0_H
#define __STSPIN32F0_H

/* Includes ------------------------------------------------------------------*/
#include "stdint.h" 
#include "MC_Common.h"
#include "stm32f0xx_hal.h"
  
/** @addtogroup DRIVERS     DRIVERS 
 * @brief  Driver Layer
 * @{ 
 */

/** @addtogroup BSP    BSP
  * @brief  BSP Layer
  * @{ 
  */

/** @addtogroup STSPIN32F0_SIP   STSPIN32F0 SIP
  * @brief  STSPIN32F0 SIP driver section
  * @{ 
  */

/* Exported Constants --------------------------------------------------------*/
/** @defgroup STSPIN32F0_SIP_Exported_Constants STSPIN32F0 SIP Exported Constants
  * @{
  */

#define BSP_SIP_HF_TIMx                 (TIM1)
#define BSP_SIP_HF_GPIO_CLK_ENABLE()    __GPIOA_CLK_ENABLE();__GPIOB_CLK_ENABLE()
#define BSP_SIP_HF_TIMx_CLK_ENABLE()    __TIM1_CLK_ENABLE()
#define BSP_SIP_HF_TIMx_CLK_DISABLE()   __TIM1_CLK_DISABLE()
#ifdef VOLTAGE_MODE
#define BSP_SIP_HF_TIMx_IT_BRK_ENABLE() __HAL_TIM_ENABLE_IT(&HF_TIMx, TIM_IT_BREAK)
#else
#define BSP_SIP_HF_TIMx_IT_BRK_ENABLE()
#endif
#define BSP_SIP_HF_TIMx_FREEZE_DBGMCU() __HAL_FREEZE_TIM1_DBGMCU();
#define BSP_SIP_HF_TIMx_OUT_PIN         (GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10)
#define BSP_SIP_HF_TIMx_OUT_PORT        (GPIOA) 
#define BSP_SIP_HF_TIMx_OUTN_PIN        (GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15)
#define BSP_SIP_HF_TIMx_OUTN_PORT       (GPIOB)
#ifdef COMPLEMENTARY_DRIVE
#define BSP_SIP_HF_TIMx_MODE            (GPIO_MODE_AF_PP)
#define BSP_SIP_HF_TIMx_PULL            (GPIO_PULLDOWN)
#define BSP_SIP_HF_TIMx_AF              (GPIO_AF2_TIM1)
#else
#define BSP_SIP_HF_TIMx_MODE            (GPIO_MODE_OUTPUT_PP)
#define BSP_SIP_HF_TIMx_PULL            (GPIO_NOPULL)
#endif
#define BSP_SIP_HF_TIMx_BKIN_PIN        (GPIO_PIN_12)
#define BSP_SIP_HF_TIMx_BKIN_PORT       (GPIOB)
#define BSP_SIP_HF_TIMx_BKIN_AF         (GPIO_AF2_TIM1)
#define BSP_SIP_HF_TIMx_IRQn            (TIM1_BRK_UP_TRG_COM_IRQn)
#ifndef HALL_SENSORS
#define BSP_SIP_HF_TIMx_PRIORITY        (0)
#define BSP_SIP_HF_TIMx_COUNTER_MODE    (TIM_COUNTERMODE_CENTERALIGNED1)
#else
#define BSP_SIP_HF_TIMx_PRIORITY        (1)
#define BSP_SIP_HF_TIMx_COUNTER_MODE    (TIM_COUNTERMODE_UP)
#endif
#define BSP_SIP_HF_TIMx_CH1             (TIM_CHANNEL_1)
#define BSP_SIP_HF_TIMx_CH2             (TIM_CHANNEL_2)
#define BSP_SIP_HF_TIMx_CH3             (TIM_CHANNEL_3)
#define BSP_SIP_HF_TIMx_CCR1            CCR1
#define BSP_SIP_HF_TIMx_CCR2            CCR2
#define BSP_SIP_HF_TIMx_CCR3            CCR3    

#define BSP_SIP_ADCx                    (ADC1)
#define BSP_SIP_ADCx_GPIO_CLK_ENABLE()  __GPIOA_CLK_ENABLE();__GPIOB_CLK_ENABLE()
#define BSP_SIP_ADCx_CLK_ENABLE()       __ADC1_CLK_ENABLE()
#define BSP_SIP_ADCx_CLK_DISABLE()      __ADC1_CLK_DISABLE()
#define BSP_SIP_ADCx_IRQn               (ADC1_IRQn)
#ifndef HALL_SENSORS
#define BSP_SIP_ADCx_PRIORITY           (0)
#define BSP_SIP_ADCx_EXT_TRIG_CONV      (ADC_EXTERNALTRIGCONV_T1_TRGO)
#define BSP_SIP_ADCx_EXT_TRIG_EDGE      (ADC_EXTERNALTRIGCONVEDGE_RISING)
#else
#define BSP_SIP_ADCx_PRIORITY           (3)
#define BSP_SIP_ADCx_EXT_TRIG_CONV      (ADC_SOFTWARE_START)
#define BSP_SIP_ADCx_EXT_TRIG_EDGE      (ADC_EXTERNALTRIGCONVEDGE_NONE)
#endif

#define BSP_SIP_DMA                     (DMA1_Channel2)
#define BSP_SIP_DMA_CLK_ENABLE()        __DMA1_CLK_ENABLE()
#define BSP_SIP_DMA_CLK_DISABLE()       __DMA1_CLK_DISABLE()
#define BSP_SIP_DMA_IRQn                (DMA1_Channel2_3_IRQn)
#define BSP_SIP_DMA_PRIORITY            (3)
#define BSP_SIP_DMA_PRIORITY_SUB        (1)

#define BSP_SIP_UART                    (USART1)
#define BSP_SIP_UART_BAUD_RATE          (230400)
#define BSP_SIP_UART_GPIO_CLK_ENABLE()  __GPIOB_CLK_ENABLE()
#define BSP_SIP_UART_CLK_ENABLE()       __USART1_CLK_ENABLE()
#define BSP_SIP_UART_CLK_DISABLE()      __USART1_CLK_DISABLE()
#define BSP_SIP_UART_IRQn               (USART1_IRQn)
#define BSP_SIP_UART_PRIORITY           (2)
#define BSP_SIP_UART_PIN                (GPIO_PIN_6|GPIO_PIN_7)
#define BSP_SIP_UART_PORT               (GPIOB)
#define BSP_SIP_UART_AF                 (GPIO_AF0_USART1)

#define BSP_SIP_OC_SEL_PIN              (GPIO_PIN_11)
#define BSP_SIP_OC_SEL_PORT             (GPIOA)

#define BSP_SIP_OC_TH_STBY_PIN          (GPIO_PIN_6|GPIO_PIN_7)
#define BSP_SIP_OC_TH_STBY_PORT         (GPIOF)

/**
  * @} end STSPIN32F0 SIP Exported Constants
  */

/* Exported types ------------------------------------------------------------*/

/** @defgroup STSPIN32F0_SIP_Exported_Types STSPIN32F0 SIP Exported Types
  * @{
  */

typedef enum {
  BSP_SIP_SEL_VIS_FROM_MCU = 0,
  BSP_SIP_SEL_VIS_FROM_MCU_AND_GATE_LOGIC = 1,
} overcurrentSelectionVisibility_t;

typedef enum {
  BSP_SIP_OC_TH_STBY  = 0,
  BSP_SIP_OC_TH_100mV = 1,
  BSP_SIP_OC_TH_250mV = 2,
  BSP_SIP_OC_TH_500mV = 3
} overcurrentThresholds_t;

/**
  * @} end STSPIN32F0 SIP Exported Types
  */

/* Exported Functions  -------------------------------------------------------*/
/** @defgroup STSPIN32F0_SIP_Exported_Functions STSPIN32F0 SIP Exported Functions
  * @{
  */
    
void EnableInput_CH1_E_CH2_E_CH3_D(uint8_t steps1to3);
void EnableInput_CH1_E_CH2_D_CH3_E(uint8_t steps1to3);
void EnableInput_CH1_D_CH2_E_CH3_E(uint8_t steps1to3);
void DisableInput_CH1_D_CH2_D_CH3_D(void);
void HF_TIMx_SetDutyCycle_CH1(uint16_t CCR_value);
void HF_TIMx_SetDutyCycle_CH2(uint16_t CCR_value);
void HF_TIMx_SetDutyCycle_CH3(uint16_t CCR_value);
void HF_TIMx_SetDutyCycle(uint16_t Iref, uint8_t stepNumber);
void Overcurrent_Selection(uint8_t ocSel);
void Overcurrent_Threshold_Setvalue(uint8_t ocThres);
void ADC_Channel(uint32_t adc_ch);
uint32_t Get_UART_Data(void);

/**
  * @} STSPIN32F0 SIP Exported Functions 
  */

/**
  * @} end STSPIN32F0 SIP 
  */

/**
  * @} end BSP
  */

/**
  * @} end DRIVERS
  */

#endif
