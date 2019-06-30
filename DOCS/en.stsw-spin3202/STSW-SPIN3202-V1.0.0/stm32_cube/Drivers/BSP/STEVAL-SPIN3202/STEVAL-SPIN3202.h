/**
 ******************************************************************************
 * @file    STEVAL-SPIN3202.h
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STEVAL_SPIN3202_H
#define __STEVAL_SPIN3202_H

/* Includes ------------------------------------------------------------------*/
#include "MC_SixStep_param_32F0.h"
#include "stspin32f0.h"

/** @addtogroup DRIVERS     DRIVERS 
  * @brief  Driver Layer
  * @{ 
  */

/** @addtogroup BSP
  * @{
  */   

/** @addtogroup STEVAL-SPIN3202
  * @{   
  */

/* Exported Constants --------------------------------------------------------*/
/** @defgroup STEVAL-SPIN3202_Exported_Constants STEVAL-SPIN3202 Exported Constants
  * @{
  */

/** @defgroup STEVAL-SPIN3202_TIMER_Related_Constants STEVAL-SPIN3202 Timer Related Constants
  * @{
  */

#ifndef HALL_SENSORS
#define BSP_BOARD_HF_TIMX_SLAVE_MODE      (TIM_SLAVEMODE_TRIGGER)
#ifndef VOLTAGE_MODE
#define BSP_BOARD_HF_TIMX_TS_ITR          (TIM_TS_ITR1)
#define BSP_BOARD_HF_TIMX_BREAK_STATE     (TIM_BREAK_DISABLE)
#define BSP_BOARD_HF_TIMX_BREAK_POL       (TIM_BREAKPOLARITY_LOW)
#define BSP_BOARD_HF_TIMX_BREAK_OUT       (TIM_AUTOMATICOUTPUT_DISABLE)
#define BSP_BOARD_LF_TIMx                 TIM2
#define BSP_BOARD_LF_TIMx_CLK_ENABLE()    __TIM2_CLK_ENABLE()
#define BSP_BOARD_LF_TIMx_CLK_DISABLE()   __TIM2_CLK_DISABLE()
#define BSP_BOARD_LF_TIMx_FREEZE_DBGMCU() __HAL_FREEZE_TIM2_DBGMCU()    
#define BSP_BOARD_LF_TIMx_IRQn            TIM2_IRQn     
#else
#define BSP_BOARD_HF_TIMX_TS_ITR          (TIM_TS_ITR2)
#define BSP_BOARD_HF_TIMX_BREAK_STATE     (TIM_BREAK_ENABLE)
#define BSP_BOARD_HF_TIMX_BREAK_POL       (TIM_BREAKPOLARITY_HIGH)
#define BSP_BOARD_HF_TIMX_BREAK_OUT       (TIM_AUTOMATICOUTPUT_DISABLE)
#define BSP_BOARD_LF_TIMx                 TIM3
#define BSP_BOARD_LF_TIMx_CLK_ENABLE()    __TIM3_CLK_ENABLE()
#define BSP_BOARD_LF_TIMx_CLK_DISABLE()   __TIM3_CLK_DISABLE()
#define BSP_BOARD_LF_TIMx_FREEZE_DBGMCU() __HAL_FREEZE_TIM3_DBGMCU()    
#define BSP_BOARD_LF_TIMx_IRQn            TIM3_IRQn
#endif
#define BSP_BOARD_LF_TIMx_PRIORITY        (1)
#else
#define BSP_BOARD_HF_TIMX_SLAVE_MODE      (TIM_SLAVEMODE_DISABLE)
#define BSP_BOARD_HF_TIMX_TS_ITR          (TIM_TS_ITR1)
#define BSP_BOARD_HF_TIMX_BREAK_STATE     (TIM_BREAK_ENABLE)
#define BSP_BOARD_HF_TIMX_BREAK_POL       (TIM_BREAKPOLARITY_HIGH)
#define BSP_BOARD_HF_TIMX_BREAK_OUT       (TIM_AUTOMATICOUTPUT_DISABLE)
#define BSP_BOARD_LF_GPIO_CLK_ENABLE()    __GPIOA_CLK_ENABLE()
#define BSP_BOARD_LF_GPIO_CLK_DISABLE()   __GPIOA_CLK_DISABLE()
#define BSP_BOARD_LF_TIMx                 TIM2
#define BSP_BOARD_LF_TIMx_CLK_ENABLE()    __TIM2_CLK_ENABLE()
#define BSP_BOARD_LF_TIMx_CLK_DISABLE()   __TIM2_CLK_DISABLE() 
#define BSP_BOARD_LF_TIMx_FREEZE_DBGMCU() __HAL_FREEZE_TIM2_DBGMCU()
#define BSP_BOARD_LF_TIMx_IRQn            TIM2_IRQn
#define BSP_BOARD_LF_TIMx_PRIORITY        (0)
#endif

#ifndef VOLTAGE_MODE
#define BSP_BOARD_REFx_TIMx               TIM3
#define BSP_BOARD_REFx_CHANNEL            TIM_CHANNEL_1
#define BSP_BOARD_REFx_CCR                CCR1
#define BSP_BOARD_REFx_TS_ITR             TIM_TS_ITR1
#define BSP_BOARD_REFx_CLK_ENABLE()       __TIM3_CLK_ENABLE()
#define BSP_BOARD_REFx_CLK_DISABLE()      __TIM3_CLK_DISABLE()
#define BSP_BOARD_REFx_FREEZE_DBGMCU()    __HAL_FREEZE_TIM3_DBGMCU()
#endif

/**
  * @} end STEVAL-SPIN3202 TIMER Related Constants
  */

/** @defgroup STEVAL-SPIN3202_ADC_Related_Constants STEVAL-SPIN3202 ADC Related Constants
  * @{
  */

#ifdef POTENTIOMETER
#define ADC_CH_1              ADC_CHANNEL_3   /*SPEED*/
#endif

#ifdef CURRENT_SENSE_ADC
#define ADC_CH_2              ADC_CHANNEL_4   /*CURRENT*/
#endif

#ifdef VBUS_SENSE_ADC
#define ADC_CH_3              ADC_CHANNEL_9   /*VBUS*/
#endif

#ifdef TEMP_SENSE_ADC
#define ADC_CH_4              ADC_CHANNEL_16  /*TEMP*/
#endif

#define ADC_Bemf_CH1          ADC_CHANNEL_0   /*BEMF1*/
#define ADC_Bemf_CH2          ADC_CHANNEL_1   /*BEMF2*/
#define ADC_Bemf_CH3          ADC_CHANNEL_2   /*BEMF3*/

#ifdef POTENTIOMETER
#define ADC_CH_1_ST           ADC_SAMPLETIME_7CYCLES_5 /*SPEED sampling time*/
#endif

#ifdef CURRENT_SENSE_ADC
#define ADC_CH_2_ST           ADC_SAMPLETIME_1CYCLE_5 /*CURRENT sampling time */
#endif

#ifdef VBUS_SENSE_ADC
#define ADC_CH_3_ST           ADC_SAMPLETIME_1CYCLE_5 /*VBUS sampling time*/
#endif

#ifdef TEMP_SENSE_ADC
#define ADC_CH_4_ST           ADC_SAMPLETIME_1CYCLE_5 /*TEMP sampling time*/
#endif

#define ADC_Bemf_CH1_ST       ADC_SAMPLETIME_1CYCLE_5  /*BEMF1 sampling time*/
#define ADC_Bemf_CH2_ST       ADC_SAMPLETIME_1CYCLE_5  /*BEMF2 sampling time*/
#define ADC_Bemf_CH3_ST       ADC_SAMPLETIME_1CYCLE_5  /*BEMF3 sampling time*/

#ifndef HALL_SENSORS
#define ADC_CH_INIT           ADC_Bemf_CH1
#define ADC_CH_INIT_ST        ADC_Bemf_CH1_ST
#elif POTENTIOMETER
#define ADC_CH_INIT           ADC_CH_1
#define ADC_CH_INIT_ST        ADC_CH_1_ST
#endif

/**
  * @} end STEVAL-SPIN3202 ADC Related Constants
  */

/** @defgroup STEVAL-SPIN3202_GPIO_Related_Constants STEVAL-SPIN3202 GPIO Related Constants
  * @{
  */

#ifndef VOLTAGE_MODE
#define BSP_BOARD_REFx_PIN               (GPIO_PIN_6)
#define BSP_BOARD_REFx_PORT              (GPIOA)
#define BSP_BOARD_REFx_AF                (GPIO_AF1_TIM3)
#endif

#ifndef HALL_SENSORS
#define BSP_BOARD_ADCx_GPIOA             (GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4)
#else
#define BSP_BOARD_ADCx_GPIOA             (GPIO_PIN_3|GPIO_PIN_4)
#define BSP_BOARD_LF_TIMx_HALL_PIN       (GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2)
#define BSP_BOARD_LF_TIMx_HALL_PORT      (GPIOA)
#define BSP_BOARD_LF_TIMx_HALL_AF        (GPIO_AF2_TIM2)
#endif
#define BSP_BOARD_ADCx_GPIOB             (GPIO_PIN_1)

#ifdef PWM_ON_BEMF_SENSING
#define GPIO_PORT_BEMF                   (GPIOA)       /*!< GPIO port name for TON/TOFF BEMF sensing method */
#define GPIO_CH_BEMF                     (GPIO_PIN_7)  /*!< GPIO pin name for TON/TOFF BEMF sensing method */
#define GPIO_PORT_ZCR_MODE               (GPIOA)       /*!< GPIO port name for echo of TON/TOFF BEMF sensing method */
#define GPIO_CH_ZCR_MODE                 (GPIO_PIN_5)  /*!< GPIO pin name for echo of TON/TOFF BEMF sensing method */
#endif

#define GPIO_PORT_ZCR                    (GPIOA)       /*!< GPIO port name for zero crossing detection */
#define GPIO_CH_ZCR                      (GPIO_PIN_5)  /*!< GPIO pin name for zero crossing detection */
#define GPIO_PORT_COMM                   (GPIOA)       /*!< GPIO port name for 6Step commutation */
#define GPIO_CH_COMM                     (GPIO_PIN_15) /*!< GPIO pin name for 6Step commutation */

#define BSP_BOARD_USER1_BUTTON_PIN       (GPIO_PIN_0)
#define BSP_BOARD_USER1_BUTTON_PORT      (GPIOF)
#define BSP_BOARD_USER2_BUTTON_PIN       (GPIO_PIN_1)
#define BSP_BOARD_USER2_BUTTON_PORT      (GPIOF)
#define BSP_BOARD_START_STOP_BUTTON_PIN  (BSP_BOARD_USER1_BUTTON_PIN)
#define BSP_BOARD_START_STOP_BUTTON_PORT (BSP_BOARD_USER1_BUTTON_PORT)
#define BSP_BOARD_FAULT_LED_PIN          (BSP_BOARD_USER2_BUTTON_PIN)
#define BSP_BOARD_FAULT_LED_PORT         (BSP_BOARD_USER2_BUTTON_PORT)


/**
  * @} end STEVAL-SPIN3202 GPIO Related Constants
  */

/** @defgroup STEVAL-SPIN3202_UART_Related_Constants STEVAL-SPIN3202 UART Related Constants
  * @{
  */

#define STARTM_CMD             0     /*!<  Start Motor command received */
#define STOPMT_CMD             1     /*!<  Stop Motor command received */
#define SETSPD_CMD             2     /*!<  Set the new speed value command received */
#define GETSPD_CMD             3     /*!<  Get Mechanical Motor Speed command received */
#define INIREF_CMD             4     /*!<  Set the new STARUP_CURRENT_REFERENCE value command received */
#define POLESP_CMD             5     /*!<  Set the Pole Pairs value command received */
#define ACCELE_CMD             6     /*!<  Set the Accelleration for Start-up of the motor command received */
#define DMGCTR_CMD             7     /*!<  Enable the DEMAG dynamic control command received */
#define MAXDMG_CMD             8     /*!<  Set the BEMF Demagn MAX command received */
#define MINDMG_CMD             9     /*!<  Set the BEMF Demagn MIN command received */
#define KP_PRM_CMD             10    /*!<  Set the KP PI param command received */
#define KI_PRM_CMD             11    /*!<  Set the KI PI param command received */
#define MEASEL_CMD             12    /*!<  Set the continuous measurement to be performed */
#define HELP_CMD               13    /*!<  Help command received */
#define STATUS_CMD             14    /*!<  Get the Status of the system command received */
#define DIRECT_CMD             15    /*!<  Get the motor direction */

/**
  * @} end STEVAL-SPIN3202 UART Related Constants
  */

/**
  * @} end STEVAL-SPIN3202 Exported Constants
  */

/* Exported Functions  -------------------------------------------------------*/
/** @defgroup STEVAL-SPIN3202_Exported_Functions STEVAL-SPIN3202 Exported Functions
  * @{
  */

void BSP_BOARD_FAULT_LED_ON(void);
void BSP_BOARD_FAULT_LED_OFF(void);
void BSP_BOARD_ADC_INIT(void);
#ifndef VOLTAGE_MODE
void MC_SixStep_Current_Reference_Start(void);
void MC_SixStep_Current_Reference_Stop(void);
void MC_SixStep_Current_Reference_Setvalue(uint16_t);
#endif
void MC_SixStep_EnableInput_CH1_E_CH2_E_CH3_D(uint8_t);
void MC_SixStep_EnableInput_CH1_E_CH2_D_CH3_E(uint8_t);
void MC_SixStep_EnableInput_CH1_D_CH2_E_CH3_E(uint8_t);
void MC_SixStep_DisableInput_CH1_D_CH2_D_CH3_D(void);
void MC_SixStep_HF_TIMx_SetDutyCycle_CH1(uint16_t);
void MC_SixStep_HF_TIMx_SetDutyCycle_CH2(uint16_t);
void MC_SixStep_HF_TIMx_SetDutyCycle_CH3(uint16_t);
void MC_SixStep_HF_TIMx_SetDutyCycle(uint16_t, uint8_t);
void MC_SixStep_ADC_Channel(uint32_t);

/**
  * @} end STEVAL-SPIN3202 Exported Functions
  */

/**
  * @} end STEVAL-SPIN3202
  */

/**
  * @} end BSP
  */

/**
  * @} end DRIVERS
  */
  
#endif /* __STEVAL_SPIN3202_H */

