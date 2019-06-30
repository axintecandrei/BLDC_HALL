/**
 ******************************************************************************
 * @file    6Step_Lib.h
 * @author  IPC Rennes
 * @version V1.0.0
 * @date    April 24th, 2017
 * @brief   This header file provides the set of functions for Motor Control 
            library 
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2015 STMicroelectronics</center></h2>
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
#ifndef __6STEP_LIB_H
#define __6STEP_LIB_H

#include "STEVAL-SPIN3202.h"
#include "stm32f0xx_hal.h"

#include "math.h"
#include "stdlib.h"
#include "stdio.h"

/** @addtogroup MIDDLEWARES     MIDDLEWARES 
  * @brief  Middlewares Layer
  * @{ 
  */


/** @addtogroup MC_6-STEP_LIB    MC_6-STEP LIB 
  * @brief  Motor Control driver
  * @{ 
  */

/** @defgroup Exported_types  Exported_types
* @{
*/
/** 
  * @brief  Six Step parameters  
  */
typedef enum 
{
    IDLE,                               /* 0 */
    STARTUP,                            /* 1 */
    VALIDATION,                         /* 2 */
    STOP,                               /* 3 */
    START,                              /* 4 */
    RUN,                                /* 5 */
    ALIGNMENT,                          /* 6 */
    SPEEDFBKERROR,                      /* 7 */
    OVERCURRENT,                        /* 8 */
    STARTUP_FAILURE,                    /* 9 */  
    STARTUP_BEMF_FAILURE,               /* 10 */
    LF_TIMER_FAILURE                    /* 11 */
} SIXSTEP_Base_SystStatus_t;

/**
  * @} 
  */


/** @defgroup Exported_types  Exported_types
* @{
*/
/** 
  * @brief  Six Step parameters  
  */
typedef struct
{
  uint32_t LF_TIMx_PSC;                  /*!< Prescaler variable for low frequency timer */
  uint32_t LF_TIMx_ARR;                  /*!< ARR variable for low frequency timer */
  uint32_t HF_TIMx_PSC;                  /*!< Prescaler variable for high frequency timer */
  uint32_t HF_TIMx_ARR;                  /*!< ARR variable for high frequency timer */
  uint16_t pulse_value;                  /*!< Gate driver timer pulse value for SixStep algorithm */
#ifndef VOLTAGE_MODE
  uint16_t current_reference;            /*!< Current reference for SixStep algorithm*/  
#endif
  uint16_t startup_reference;            /*!< Startup value for pulse value or current reference for SixStep algorithm*/   
  uint8_t step_position;                 /*!< Step number variable for SixStep algorithm */
  SIXSTEP_Base_SystStatus_t STATUS;      /*!< Status variable for SixStep algorithm */  
#ifdef HALL_SENSORS
  uint8_t  next_step_pos;                /*!< Next step position for SixStep algorithm */
  uint16_t hall_capture;                 /*!< Input capture register value when hall status changes */
  uint16_t commutation_delay;            /*!< Delay between hall capture and step commutation */
  uint8_t hall_ok;                       /*!< Indication of hall capture */
  uint8_t hall_ko_successive;             /*!< Number of successive no hall capture before timer expiration */
  uint8_t start_attempts;                /*!< Number of start attempts */
  uint8_t run_attempts;                  /*!< Number of run attempts */
  volatile int32_t start_cnt;            /*!< Counter used to compute initial commutation delays */
#else
  uint8_t prev_step_pos;                 /*!< Previous step position for SixStep algorithm */
  uint16_t numberofitemArr;              /*!< Number of elements for motor startup */
  uint8_t ADC_BEMF_channel[3];           /*!< Array of ADC regular channels used for BEMF sensing */
  uint8_t Current_ADC_BEMF_channel;      /*!< ADC regular channel to select for BEMF sensing */  
  uint16_t ADC_BEMF_Buffer[7];           /*!< Buffer for ADC regular channels used for BEMF sensing */
  uint16_t ADC_BEMF_threshold_UP;        /*!< Voltage threshold for BEMF detection in up direction during OFF time*/  
  uint16_t ADC_BEMF_threshold_DOWN;      /*!< Voltage threshold for BEMF detection in down direction during OFF time*/
#ifdef PWM_ON_BEMF_SENSING
  uint16_t ADC_BEMF_threshold_UP_ON;     /*!< Voltage threshold for BEMF detection in up direction during ON time*/  
  uint16_t ADC_BEMF_threshold_DOWN_ON;   /*!< Voltage threshold for BEMF detection in down direction during ON time*/ 
  uint16_t ADC_Current_BEMF_thld_UP;     /*!< Voltage threshold for BEMF detection in up direction*/  
  uint16_t ADC_Current_BEMF_thld_DOWN;   /*!< Voltage threshold for BEMF detection in down direction*/
#endif
  uint16_t demagn_counter;               /*!< Demagnetization counter */
  uint16_t demagn_value;                 /*!< Demagnetization value */  
#endif
  uint8_t ADC_SEQ_Channel[4];            /*!< Array of ADC channels used for speed, current, vbus or temperature sensing */
  uint16_t ADC_SEQ_Buffer[4];            /*!< Buffer for ADC regular channels used for non BEMF measurements */
  int32_t speed_fdbk;                    /*!< Motor speed variable */  
  int32_t speed_fdbk_filtered;           /*!< Filtered Motor speed variable */
  int32_t speed_target;                  /*!< Speed target for the motor */

#ifdef PID
  int32_t error2;
  int32_t error1;
#else
  int32_t Integral_Term_sum;             /*!< Global Integral part for PI*/
#endif
  uint8_t ALIGN_OK;                      /*!< Flag control for Motor Alignment*/ 
  uint8_t ALIGNMENT;                     /*!< Flag control for Motor Alignment ongoing*/   
  uint16_t Speed_Loop_Time;              /*!< Speed loop variable for timing */ 

  uint16_t RUN_Motor;                    /*!< Flag for Motor status */ 
  uint8_t ARR_OK;                        /*!< ARR flag control for Accell status */ 
  uint8_t VALIDATION_OK;                 /*!< Validation flag for Closed loop control begin */ 
  uint8_t SPEED_VALIDATED;               /*!< Validation flag for Speed before closed loop control */ 
  uint8_t TARGET_SPEED_SWITCHED;         /*!< Target speed has been switched to final speed in closed loop */
  
  
  uint32_t SYSCLK_frequency;             /*!< System clock main frequency */ 
  uint32_t Uart_cmd_to_set;              /*!<  */ 
  uint32_t Uart_value_to_set;            /*!<  */
  uint32_t bemfMeasurements;
  uint16_t bemfIndexRx;  
  uint8_t UART_MEASUREMENT_TYPE;
  uint8_t UART_CONTINUOUS_TX_BEMF_MODE;
  uint8_t UART_CONTINUOUS_TX_BEMF_ALLOWED;  
  uint8_t UART_CONTINUOUS_TX_SPEED_MODE;
  uint8_t UART_CONTINUOUS_TX_SPEED_ALLOWED;
  uint8_t UART_TX_REPLY;
  uint8_t UART_TX_DIFFERED_REPLY;
  uint16_t UART_TX_CANCELLED;
  uint8_t Button_ready;                  /*!<  */
  uint8_t BEMF_OK;                       /*!<  */
  uint8_t CL_READY;                      /*!<  */
  uint8_t BEMF_Tdown_count;              /*!< BEMF Consecutive Threshold Falling Crossings Counter */   
  uint16_t IREFERENCE;                   /*!< Currrent reference*/ 
  uint16_t NUMPOLESPAIRS;                /*!< Number of motor pole pairs  */ 
  uint32_t ACCEL;                        /*!< Acceleration start-up parameter*/ 
  uint16_t KP;                           /*!< KP parameter for PI(D) regulator */ 
  uint16_t KI;                           /*!< KI parameter for PI(D) regulator */ 
#ifdef PID
  uint16_t KD;                           /*!< KD parameter for PID regulator */
#endif
  uint8_t CW_CCW;                        /*!< Set the motor direction */ 
  uint8_t overcurrent;
}  SIXSTEP_Base_InitTypeDef;             /*!< Six Step Data Structure */

/**
  * @} 
  */

/** @defgroup Exported_types  Exported_types
* @{
*/
/** 
  * @brief  Six PI regulator parameters  
  */

typedef struct
{
  int32_t Reference;                    /*!< Current reference value for PI regulator */
  uint16_t Kp_Gain;                     /*!< Kp value for PI(D) regulator */ 
  uint16_t Ki_Gain;                     /*!< Ki value for PI(D) regulator */
#ifdef PID
  int16_t Kd_Gain;                      /*!< Kd value for PID regulator */  
#endif
  int16_t Lower_Limit_Output;           /*!< Min output value for PI regulator */ 
  int16_t Upper_Limit_Output;           /*!< Max output value for PI regulator */ 
#ifdef SPEED_RAMP
  int8_t ReferenceToBeUpdated;          /*!< Different from 0 if the current reference has to be updated */
#endif
} SIXSTEP_PI_PARAM_InitTypeDef_t, *SIXSTEP_pi_PARAM_InitTypeDef_t;  /*!< PI Data Structure */

/**
  * @} 
  */

/** @defgroup MC_6-STEP_LIB_API  MC_6-STEP LIB API
* @{
*/
void MC_SixStep_INIT(void);
void MC_SixStep_RESET(void);
void MC_StartMotor(void);
void MC_StopMotor(void);
void MC_Set_Speed(uint16_t);
void MC_EXT_button_SixStep(void);
/**
  * @} 
  */

/**  MC_6-STEP_LIB 
  * @} 
  */ 

 
/**  MIDDLEWARES
  * @} 
  */

#endif
