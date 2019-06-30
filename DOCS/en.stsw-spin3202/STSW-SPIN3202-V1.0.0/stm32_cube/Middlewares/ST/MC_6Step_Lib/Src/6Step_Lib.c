/**
 ******************************************************************************
 * @file    6Step_Lib.c
 * @author  IPC Rennes
 * @version V1.0.0
 * @date    April 24th, 2017
 * @brief   This file provides the set of functions for Motor Control library 
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

/*! ****************************************************************************
================================================================================   
                 ###### Main functions for 6-Step algorithm ######
================================================================================     
The main function are the following:

1) MC_SixStep_TABLE(...) -> Set the peripherals (TIMx, GPIO etc.) for each step
2) MC_SixStep_ARR_step() -> Generate the ARR value for Low Frequency TIM during start-up
3) MC_SixStep_INIT()     -> Init the main variables for motor driving from MC_SixStep_param.h
4) MC_SixStep_RESET()    -> Reset all variables used for 6Step control algorithm
5) MC_SixStep_Ramp_Motor_calc() -> Calculate the acceleration profile step by step for motor during start-up 
6) MC_SixStep_NEXT_step()-> Generate the next step number according with the direction (CW or CCW)
7) MC_Task_Speed()       -> Speed Loop with PI regulator
8) MC_Set_Speed(...)     -> Set the new motor speed value
9) MC_StartMotor()       -> Start the Motor
10)MC_StopMotor()       -> Stop the Motor
*******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "6Step_Lib.h"

#include <string.h>

/** @addtogroup MIDDLEWARES     MIDDLEWARES 
  * @brief  Middlewares Layer
  * @{ 
  */


/** @addtogroup MC_6-STEP_LIB       MC_6-STEP LIB 
  * @brief  Motor Control driver
  * @{ 
  */

/* Data struct ---------------------------------------------------------------*/
SIXSTEP_Base_InitTypeDef SIXSTEP_parameters;            /*!< Main SixStep structure*/ 
SIXSTEP_PI_PARAM_InitTypeDef_t PI_parameters;           /*!< SixStep PI regulator structure*/ 

TIM_OC_InitTypeDef sConfig;

/* Variables -----------------------------------------------------------------*/
extern TIM_HandleTypeDef HF_TIMx;
extern TIM_HandleTypeDef LF_TIMx;
extern ADC_HandleTypeDef ADCx;
#ifdef TEST
uint8_t stop = 0;
#endif
#ifdef HALL_SENSORS
uint16_t H1, H2, H3;
uint8_t hallStatus;
#endif
#ifdef BEMF_RECORDING
#define BEMF_ARRAY_SIZE 400
uint16_t bemfArray[BEMF_ARRAY_SIZE+6];
#endif
uint16_t Rotor_poles_pairs;                         /*!<  Number of pole pairs of the motor */ 
uint32_t mech_accel_hz = 0;                         /*!<  Hz -- Mechanical acceleration rate */
uint32_t constant_k = 0;                            /*!<  1/3*mech_accel_hz */
uint32_t Time_vector_tmp = 0;                       /*!<  Startup variable  */
uint32_t Time_vector_prev_tmp = 0 ;                 /*!<  Startup variable  */
uint32_t T_single_step = 0;                         /*!<  Startup variable  */
uint32_t T_single_step_first_value = 0;             /*!<  Startup variable  */
int32_t  delta = 0;                                 /*!<  Startup variable  */
uint16_t index_array = 0;                           /*!<  Speed filter variable */
int32_t speed_tmp_array[FILTER_DEEP];               /*!<  Speed filter variable */
#ifdef POTENTIOMETER
uint32_t potentiometer_prev_speed_target;           /*!< Previous speed target for the motor */
uint32_t potentiometer_speed_target;
uint16_t potentiometer_buffer[POT_BUFFER_SIZE];     /*!<  Buffer for Potentiometer Value Filtering */
uint16_t potentiometer_buffer_index = 0;            /*!<  High-Frequency Buffer Index */
#endif
uint8_t  array_completed = FALSE;                   /*!<  Speed filter variable */
uint8_t  UART_FLAG_RECEIVE = FALSE;                 /*!<  UART commmunication flag */
#ifndef HALL_SENSORS
uint32_t ARR_LF = 0;                                /*!<  Autoreload LF TIM variable */
#endif
int32_t Mech_Speed_RPM = 0;                         /*!<  Mechanical motor speed */
int32_t El_Speed_Hz = 0;                            /*!<  Electrical motor speed */
uint16_t index_adc_chn = 0;                         /*!<  Index of ADC channel selector for measuring */
#ifdef DEMOMODE
uint16_t index_motor_run = 0;                       /*!<  Tmp variable for DEMO mode */
uint16_t test_motor_run = 1;                        /*!<  Tmp variable for DEMO mode */
#endif
uint8_t Enable_start_button = TRUE;                 /*!<  Start/stop button filter to avoid double command */
#ifndef HALL_SENSORS
uint16_t index_ARR_step = 1;                           
uint32_t n_zcr_startup = 0;
uint16_t cnt_bemf_event = 0;
uint8_t startup_bemf_failure = 0;
uint8_t lf_timer_failure = 0;
uint8_t speed_fdbk_error = 0;
uint16_t index_startup_motor = 1;
#ifdef PWM_ON_BEMF_SENSING
uint8_t zcr_on_ton = 0;
uint8_t zcr_on_ton_next = 0;
#endif
#endif
uint16_t shift_n_sqrt = 14;
static __IO uint32_t uwTick = 0;                        /*!<  Tick counter - 1msec updated */
uint16_t index_align = 1;
int32_t speed_sum_sp_filt = 0;
int32_t speed_sum_pot_filt = 0;
uint16_t index_pot_filt = 1; 
int16_t potent_filtered = 0;
uint32_t Tick_cnt = 0;  
uint32_t counter_ARR_Bemf = 0;
uint64_t constant_multiplier_tmp = 0;

/** @addtogroup MotorControl_Board_Linked_Functions MotorControl Board Linked Functions
  * @{
  */
void BSP_BOARD_FAULT_LED_ON(void);
void BSP_BOARD_FAULT_LED_OFF(void);
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
#ifdef VOLTAGE_MODE 
void MC_SixStep_HF_TIMx_SetDutyCycle(uint16_t, uint8_t);
#endif
void MC_SixStep_ADC_Channel(uint32_t);
/**
  * @} end MotorControl_Board_Linked_Functions
  */

/** @addtogroup UART_UI  UART UI
  * @brief  Serial communication through PC serial terminal
  * @{ 
  */ 
#ifdef UART_COMM
void MC_UI_INIT(void);
void UART_Send_Bemf(uint16_t *, uint16_t);
void UART_Send_Speed(void);
void UART_Set_Value(void);
void UART_Communication_Task(void);
void CMD_Parser(char* pCommandString);
#endif
/**
  * @} end UART_UI
  */

/** @defgroup MC_6-STEP_LIB_Exported_Functions MC_6-STEP LIB Exported Functions
  * @{
  */ 
void MC_Set_PI_param(SIXSTEP_PI_PARAM_InitTypeDef_t *);
#ifdef HALL_SENSORS
void MC_SixStep_Hall_Startup_Failure_Handler(void);
void MC_SixStep_Hall_Run_Failure_Handler(void);
void MC_TIMx_SixStep_CommutationEvent(void);
#else
void MC_ADCx_SixStep_Bemf(void);
#ifdef PWM_ON_BEMF_SENSING
void MC_Update_ADC_Ch(uint8_t current_is_BEMF);
#endif
#endif
void MC_TIMx_SixStep_timebase(void);
void MC_SysTick_SixStep_MediumFrequencyTask(void);
void MC_SixStep_TABLE(uint8_t);
/**
  * @} end MC_6-STEP_LIB_Exported_Functions
  */

/** @defgroup MC_6-STEP_LIB_Private_Functions MC_6-STEP LIB Private Functions
  * @{
  */ 
uint16_t MC_PI_Controller(SIXSTEP_PI_PARAM_InitTypeDef_t *, int32_t);
#ifdef POTENTIOMETER
uint16_t MC_Potentiometer_filter(void);
#endif
uint64_t MCM_Sqrt(uint64_t );
int32_t MC_GetElSpeedHz(void);
int32_t MC_GetMechSpeedRPM(void);
void MC_SixStep_NEXT_step(int32_t);
void MC_Speed_Filter(void);
void MC_SixStep_ARR_step(void);
void MC_Task_Speed(void);
#ifndef HALL_SENSORS
void MC_SixStep_Alignment(void);
#endif
void MC_SixStep_Ramp_Motor_calc(void);
void MC_SixStep_ARR_Bemf(uint8_t);
void MC_SixStep_Init_main_data(void);
/**
  * @} end MC_6-STEP_LIB_Private_Functions
  */

/** @defgroup MC_SixStep_TABLE    MC_SixStep_TABLE
  *  @{
    * @brief Set the peripherals (TIMx, GPIO etc.) for each step
    * @param  step_number: step number selected
    * @retval None
  */
void MC_SixStep_TABLE(uint8_t step_number)
{ 
#if (GPIO_COMM!=0)
  HAL_GPIO_TogglePin(GPIO_PORT_COMM,GPIO_CH_COMM);  
#endif
#if (defined(DELTA_6STEP_TABLE) && defined(COMPLEMENTARY_DRIVE) && !defined(HALL_SENSORS))
  switch (step_number)
  {
    case 1:
    {
      if(PI_parameters.Reference >= 0)
        HF_TIMx.Instance->CCR1 = SIXSTEP_parameters.pulse_value;
      else
        HF_TIMx.Instance->CCR2 = 0;
      HF_TIMx.Instance->CCER = TIM_CCER_CC1E | TIM_CCER_CC1NE | TIM_CCER_CC2E | TIM_CCER_CC2NE;      
      SIXSTEP_parameters.Current_ADC_BEMF_channel = SIXSTEP_parameters.ADC_BEMF_channel[2];
    }
    break;
    case 2:
    {
      if(PI_parameters.Reference >= 0)
        HF_TIMx.Instance->CCR3 = 0;
      else
        HF_TIMx.Instance->CCR1 = SIXSTEP_parameters.pulse_value;
      HF_TIMx.Instance->CCER = TIM_CCER_CC1E | TIM_CCER_CC1NE | TIM_CCER_CC3E | TIM_CCER_CC3NE;     
      SIXSTEP_parameters.Current_ADC_BEMF_channel = SIXSTEP_parameters.ADC_BEMF_channel[1];
    }
    break;
    case 3:
    {
      if(PI_parameters.Reference >= 0)
        HF_TIMx.Instance->CCR2 = SIXSTEP_parameters.pulse_value;
      else
        HF_TIMx.Instance->CCR3 = 0;
      HF_TIMx.Instance->CCER = TIM_CCER_CC2E | TIM_CCER_CC2NE | TIM_CCER_CC3E | TIM_CCER_CC3NE;      
      SIXSTEP_parameters.Current_ADC_BEMF_channel = SIXSTEP_parameters.ADC_BEMF_channel[0];
    }
    break;
    case 4:
    {
      if(PI_parameters.Reference >= 0)
        HF_TIMx.Instance->CCR1 = 0;
      else
        HF_TIMx.Instance->CCR2 = SIXSTEP_parameters.pulse_value;
      HF_TIMx.Instance->CCER = TIM_CCER_CC1E | TIM_CCER_CC1NE | TIM_CCER_CC2E | TIM_CCER_CC2NE;         
      SIXSTEP_parameters.Current_ADC_BEMF_channel = SIXSTEP_parameters.ADC_BEMF_channel[2];
    }
    break;
    case 5:
    {
      if(PI_parameters.Reference >= 0)
        HF_TIMx.Instance->CCR3 = SIXSTEP_parameters.pulse_value;
      else
        HF_TIMx.Instance->CCR1 = 0;        
      HF_TIMx.Instance->CCER = TIM_CCER_CC1E | TIM_CCER_CC1NE | TIM_CCER_CC3E | TIM_CCER_CC3NE;         
      SIXSTEP_parameters.Current_ADC_BEMF_channel = SIXSTEP_parameters.ADC_BEMF_channel[1];
    }
    break;
    case 6:
    {
      if(PI_parameters.Reference>=0)
        HF_TIMx.Instance->CCR2 = 0;
      else
        HF_TIMx.Instance->CCR3 = SIXSTEP_parameters.pulse_value;
      HF_TIMx.Instance->CCER = TIM_CCER_CC2E | TIM_CCER_CC2NE | TIM_CCER_CC3E | TIM_CCER_CC3NE;        
      SIXSTEP_parameters.Current_ADC_BEMF_channel = SIXSTEP_parameters.ADC_BEMF_channel[0];
    }
    break;
  }
#elif (defined(DELTA_6STEP_TABLE) && !defined(HALL_SENSORS))
  switch (step_number)
  {
    case 1:
    {
      if(PI_parameters.Reference >= 0)
        HF_TIMx.Instance->CCR1 = SIXSTEP_parameters.pulse_value;
      else
      {
        HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,GPIO_PIN_RESET);
      }
      HF_TIMx.Instance->CCER = TIM_CCER_CC1E;      
      SIXSTEP_parameters.Current_ADC_BEMF_channel = SIXSTEP_parameters.ADC_BEMF_channel[2];
    }
    break;
    case 2:
    {
      if(PI_parameters.Reference >= 0)
      {
        HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,GPIO_PIN_SET);
      }
      else
        HF_TIMx.Instance->CCR1 = SIXSTEP_parameters.pulse_value;
      HF_TIMx.Instance->CCER = TIM_CCER_CC1E;     
      SIXSTEP_parameters.Current_ADC_BEMF_channel = SIXSTEP_parameters.ADC_BEMF_channel[1];
    }
    break;
    case 3:
    {
      if(PI_parameters.Reference >= 0)
        HF_TIMx.Instance->CCR2 = SIXSTEP_parameters.pulse_value;
      else
      {
        HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13,GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,GPIO_PIN_SET);
      }
      HF_TIMx.Instance->CCER = TIM_CCER_CC2E;      
      SIXSTEP_parameters.Current_ADC_BEMF_channel = SIXSTEP_parameters.ADC_BEMF_channel[0];
    }
    break;
    case 4:
    {
      if(PI_parameters.Reference >= 0)
      {
        HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13,GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,GPIO_PIN_RESET);
      }
      else
        HF_TIMx.Instance->CCR2 = SIXSTEP_parameters.pulse_value;
      HF_TIMx.Instance->CCER = TIM_CCER_CC2E;         
      SIXSTEP_parameters.Current_ADC_BEMF_channel = SIXSTEP_parameters.ADC_BEMF_channel[2];
    }
    break;
    case 5:
    {
      if(PI_parameters.Reference >= 0)
        HF_TIMx.Instance->CCR3 = SIXSTEP_parameters.pulse_value;
      else
      {
        HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13,GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,GPIO_PIN_RESET);
      }
      HF_TIMx.Instance->CCER = TIM_CCER_CC3E;         
      SIXSTEP_parameters.Current_ADC_BEMF_channel = SIXSTEP_parameters.ADC_BEMF_channel[1];
    }
    break;
    case 6:
    {
      if(PI_parameters.Reference>=0)
      {
        HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13,GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,GPIO_PIN_SET);
      }
      else
        HF_TIMx.Instance->CCR3 = SIXSTEP_parameters.pulse_value;
      HF_TIMx.Instance->CCER = TIM_CCER_CC3E;        
      SIXSTEP_parameters.Current_ADC_BEMF_channel = SIXSTEP_parameters.ADC_BEMF_channel[0];
    }
    break;
  }
#elif (defined(HALL_SENSORS))
  switch (step_number)
  { 
    case 1:
      {  
          MC_SixStep_HF_TIMx_SetDutyCycle_CH1(SIXSTEP_parameters.pulse_value);
          MC_SixStep_HF_TIMx_SetDutyCycle_CH2(0);
          MC_SixStep_EnableInput_CH1_E_CH2_E_CH3_D(1);
      }
     break;
    case 2:
      {                   
          MC_SixStep_HF_TIMx_SetDutyCycle_CH1(SIXSTEP_parameters.pulse_value);
          MC_SixStep_HF_TIMx_SetDutyCycle_CH3(0);                              
          MC_SixStep_EnableInput_CH1_E_CH2_D_CH3_E(1);
      }
     break;     
    case 3:
      {     
          MC_SixStep_HF_TIMx_SetDutyCycle_CH2(SIXSTEP_parameters.pulse_value);
          MC_SixStep_HF_TIMx_SetDutyCycle_CH3(0);
          MC_SixStep_EnableInput_CH1_D_CH2_E_CH3_E(1);
      }
     break;     
    case 4:
      { 
          MC_SixStep_HF_TIMx_SetDutyCycle_CH1(0);
          MC_SixStep_HF_TIMx_SetDutyCycle_CH2(SIXSTEP_parameters.pulse_value);
          MC_SixStep_EnableInput_CH1_E_CH2_E_CH3_D(0);
      }
     break;  
    case 5:
      {
          MC_SixStep_HF_TIMx_SetDutyCycle_CH1(0);
          MC_SixStep_HF_TIMx_SetDutyCycle_CH3(SIXSTEP_parameters.pulse_value);          
          MC_SixStep_EnableInput_CH1_E_CH2_D_CH3_E(0);
      }
     break;
    case 6:
      {   
          MC_SixStep_HF_TIMx_SetDutyCycle_CH2(0);
          MC_SixStep_HF_TIMx_SetDutyCycle_CH3(SIXSTEP_parameters.pulse_value);          
          MC_SixStep_EnableInput_CH1_D_CH2_E_CH3_E(0);
      }
     break;        
  }
#else
  switch (step_number)
  { 
    case 1:
      {  
          MC_SixStep_HF_TIMx_SetDutyCycle_CH1(SIXSTEP_parameters.pulse_value);
          MC_SixStep_HF_TIMx_SetDutyCycle_CH2(0);
          MC_SixStep_EnableInput_CH1_E_CH2_E_CH3_D(1);
          SIXSTEP_parameters.Current_ADC_BEMF_channel = SIXSTEP_parameters.ADC_BEMF_channel[2];
      }
     break;
    case 2:
      {                   
          MC_SixStep_HF_TIMx_SetDutyCycle_CH1(SIXSTEP_parameters.pulse_value);
          MC_SixStep_HF_TIMx_SetDutyCycle_CH3(0);
          MC_SixStep_EnableInput_CH1_E_CH2_D_CH3_E(1);       
          SIXSTEP_parameters.Current_ADC_BEMF_channel = SIXSTEP_parameters.ADC_BEMF_channel[1];
      }
     break;     
    case 3:
      {     
          MC_SixStep_HF_TIMx_SetDutyCycle_CH2(SIXSTEP_parameters.pulse_value);
          MC_SixStep_HF_TIMx_SetDutyCycle_CH3(0);
          MC_SixStep_EnableInput_CH1_D_CH2_E_CH3_E(1);
          SIXSTEP_parameters.Current_ADC_BEMF_channel = SIXSTEP_parameters.ADC_BEMF_channel[0];
      }
     break;     
    case 4:
      { 
          MC_SixStep_HF_TIMx_SetDutyCycle_CH1(0);
          MC_SixStep_HF_TIMx_SetDutyCycle_CH2(SIXSTEP_parameters.pulse_value);
          MC_SixStep_EnableInput_CH1_E_CH2_E_CH3_D(0);       
          SIXSTEP_parameters.Current_ADC_BEMF_channel = SIXSTEP_parameters.ADC_BEMF_channel[2];
      }
     break;  
    case 5:
      {
          MC_SixStep_HF_TIMx_SetDutyCycle_CH1(0);
          MC_SixStep_HF_TIMx_SetDutyCycle_CH3(SIXSTEP_parameters.pulse_value);
          MC_SixStep_EnableInput_CH1_E_CH2_D_CH3_E(0);        
          SIXSTEP_parameters.Current_ADC_BEMF_channel = SIXSTEP_parameters.ADC_BEMF_channel[1];
      }
     break;
    case 6:
      {   
          MC_SixStep_HF_TIMx_SetDutyCycle_CH2(0);
          MC_SixStep_HF_TIMx_SetDutyCycle_CH3(SIXSTEP_parameters.pulse_value);
          MC_SixStep_EnableInput_CH1_D_CH2_E_CH3_E(0);         
          SIXSTEP_parameters.Current_ADC_BEMF_channel = SIXSTEP_parameters.ADC_BEMF_channel[0];
      }
     break;        
  }
#endif
}

/**
  * @}
  */
#ifndef HALL_SENSORS
/** @defgroup MC_SixStep_NEXT_step    MC_SixStep_NEXT_step
  *  @{
    * @brief Generate the next step number according with the direction (CW or CCW)
    * @retval uint8_t SIXSTEP_parameters.status
  */
void MC_SixStep_NEXT_step(int32_t Reference)
{
  ARR_LF = __HAL_TIM_GetAutoreload(&LF_TIMx);
   
  if ((ARR_LF == 0xFFFF) && (SIXSTEP_parameters.STATUS == RUN))
  {
    lf_timer_failure++;
  }
  else
  {
    SIXSTEP_parameters.LF_TIMx_ARR = ARR_LF;
  }

  if(SIXSTEP_parameters.ALIGN_OK != FALSE) 
  {
    SIXSTEP_parameters.demagn_counter = 1;
    if(SIXSTEP_parameters.prev_step_pos != SIXSTEP_parameters.step_position)
    {
      n_zcr_startup = 0;
    }
    if(PI_parameters.Reference>=0)
    {
      if(SIXSTEP_parameters.step_position>=6)
      { 
        SIXSTEP_parameters.step_position = 1;
      }
      else
      {
        SIXSTEP_parameters.step_position++;
      }
      SIXSTEP_parameters.speed_fdbk = MC_GetMechSpeedRPM();
    }
    else
    {
      if(SIXSTEP_parameters.step_position <= 1)
      { 
        SIXSTEP_parameters.step_position = 6; 
      }
      else
      {
        SIXSTEP_parameters.step_position--;  
      }
      SIXSTEP_parameters.speed_fdbk = -MC_GetMechSpeedRPM();
    }
    if(SIXSTEP_parameters.CL_READY != FALSE)
    {
      SIXSTEP_parameters.VALIDATION_OK = TRUE;
    }    
  }
  
  if(SIXSTEP_parameters.VALIDATION_OK != FALSE)
  { 
    /* Motor Stall condition detection and Speed-Feedback error generation */
    SIXSTEP_parameters.BEMF_Tdown_count++;
    if (SIXSTEP_parameters.BEMF_Tdown_count>BEMF_CONSEC_DOWN_MAX)
    { 
      speed_fdbk_error = 1;
    }
    else
    {
      LF_TIMx.Instance->ARR = 0xFFFF;
    }
  } 
  MC_SixStep_TABLE(SIXSTEP_parameters.step_position);
   
  /*  It controls if the changing step request appears during DOWNcounting
   *  in this case it changes the ADC channel */
  

  if((!__HAL_TIM_DIRECTION_STATUS(&HF_TIMx))&&(SIXSTEP_parameters.STATUS == RUN))
  {
   /* UP COUNTING, DIR bit of TIMxCR1 is 0 when up counting */ 
   switch (SIXSTEP_parameters.step_position)
   { 
    case 1:
    case 4:
     {
        SIXSTEP_parameters.Current_ADC_BEMF_channel = SIXSTEP_parameters.ADC_BEMF_channel[2]; 
     }  
    break;      
    case 2:
    case 5:
     {
        SIXSTEP_parameters.Current_ADC_BEMF_channel = SIXSTEP_parameters.ADC_BEMF_channel[1];   
     }  
    break;      
    case 3:
    case 6:
     {
        SIXSTEP_parameters.Current_ADC_BEMF_channel = SIXSTEP_parameters.ADC_BEMF_channel[0];      
     }     
    break;      
    } 
     if(SIXSTEP_parameters.STATUS != START && SIXSTEP_parameters.STATUS != ALIGNMENT)
       MC_SixStep_ADC_Channel(SIXSTEP_parameters.Current_ADC_BEMF_channel);
   }
  
}

/**
  * @} 
  */
#else
void MC_SixStep_NEXT_step(int32_t Reference)
{
  H1 = HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0);
  H2 = HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_1);
  H3 = HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_2);    
  hallStatus = (H1 << 2) | (H2 << 1) | H3; 
  if(SIXSTEP_parameters.ALIGN_OK != FALSE) 
  {
    if(Reference > 0)
    {
      switch (hallStatus)
      {
        case 2:
        {
          SIXSTEP_parameters.next_step_pos=1;
          HF_TIMx.Instance->CCR1 = SIXSTEP_parameters.pulse_value;
          HF_TIMx.Instance->CCER = TIM_CCER_CC1E | TIM_CCER_CC1NE | TIM_CCER_CC2E | TIM_CCER_CC2NE;
        }
        break;
        case 3:
        {
          SIXSTEP_parameters.next_step_pos=2;
          HF_TIMx.Instance->CCR3 = 0;
          HF_TIMx.Instance->CCER = TIM_CCER_CC1E | TIM_CCER_CC1NE | TIM_CCER_CC3E | TIM_CCER_CC3NE;
        }
        break;
        case 1:
        {
          SIXSTEP_parameters.next_step_pos=3;
          HF_TIMx.Instance->CCR2 = SIXSTEP_parameters.pulse_value;
          HF_TIMx.Instance->CCER = TIM_CCER_CC2E | TIM_CCER_CC2NE | TIM_CCER_CC3E | TIM_CCER_CC3NE;          
        }
        break;     
        case 5:
        {
          SIXSTEP_parameters.next_step_pos=4;
          HF_TIMx.Instance->CCR1 = 0;
          HF_TIMx.Instance->CCER = TIM_CCER_CC1E | TIM_CCER_CC1NE | TIM_CCER_CC2E | TIM_CCER_CC2NE;          
        }
        break;     
        case 4:
        {
          SIXSTEP_parameters.next_step_pos=5;
          HF_TIMx.Instance->CCR3 = SIXSTEP_parameters.pulse_value;
          HF_TIMx.Instance->CCER = TIM_CCER_CC1E | TIM_CCER_CC1NE | TIM_CCER_CC3E | TIM_CCER_CC3NE;            
        }
        break;  
        case 6:
        {
          SIXSTEP_parameters.next_step_pos=6;
          HF_TIMx.Instance->CCR2 = 0;
          HF_TIMx.Instance->CCER = TIM_CCER_CC2E | TIM_CCER_CC2NE | TIM_CCER_CC3E | TIM_CCER_CC3NE;
        }
        break;       
      }
      SIXSTEP_parameters.speed_fdbk = MC_GetMechSpeedRPM();
    }
    else
    {      
      switch (hallStatus)
        {
        case 1:
        {
          SIXSTEP_parameters.next_step_pos=6;
          HF_TIMx.Instance->CCR3 = SIXSTEP_parameters.pulse_value;
          HF_TIMx.Instance->CCER = TIM_CCER_CC2E | TIM_CCER_CC2NE | TIM_CCER_CC3E | TIM_CCER_CC3NE;          
        }
        break;
        case 3:
        { 
          SIXSTEP_parameters.next_step_pos=5;
          HF_TIMx.Instance->CCR1 = 0;
          HF_TIMx.Instance->CCER = TIM_CCER_CC1E | TIM_CCER_CC1NE | TIM_CCER_CC3E | TIM_CCER_CC3NE;          
        }
        break;
        case 2:
        {   
          SIXSTEP_parameters.next_step_pos=4;
          HF_TIMx.Instance->CCR2 = SIXSTEP_parameters.pulse_value;
          HF_TIMx.Instance->CCER = TIM_CCER_CC1E | TIM_CCER_CC1NE | TIM_CCER_CC2E | TIM_CCER_CC2NE;
        }
        break;
        case 6:
        {
          SIXSTEP_parameters.next_step_pos=3;
          HF_TIMx.Instance->CCR3 = 0;
          HF_TIMx.Instance->CCER = TIM_CCER_CC2E | TIM_CCER_CC2NE | TIM_CCER_CC3E | TIM_CCER_CC3NE;
        }
        break;        
        case 4:
        {
          SIXSTEP_parameters.next_step_pos=2;
          HF_TIMx.Instance->CCR1 = SIXSTEP_parameters.pulse_value;
          HF_TIMx.Instance->CCER = TIM_CCER_CC1E | TIM_CCER_CC1NE | TIM_CCER_CC3E | TIM_CCER_CC3NE;
        }
        break;
        case 5:
        { 
          SIXSTEP_parameters.next_step_pos=1;
          HF_TIMx.Instance->CCR2 = 0;
          HF_TIMx.Instance->CCER = TIM_CCER_CC1E | TIM_CCER_CC1NE | TIM_CCER_CC2E | TIM_CCER_CC2NE;            
        }
        break;
      }
      SIXSTEP_parameters.speed_fdbk = -MC_GetMechSpeedRPM();
    }
  }
}
#endif

/** @defgroup MC_SixStep_RESET    MC_SixStep_RESET
  *  @{
    * @brief Reset all variables used for 6Step control algorithm
    * @retval None
  */

void MC_SixStep_RESET()
{
#ifdef VOLTAGE_MODE
  SIXSTEP_parameters.startup_reference = (STARTUP_DUTY_CYCLE*SIXSTEP_parameters.HF_TIMx_ARR)/1000;
  SIXSTEP_parameters.pulse_value = SIXSTEP_parameters.startup_reference;
#else
  SIXSTEP_parameters.startup_reference = STARTUP_CURRENT_REFERENCE;
  SIXSTEP_parameters.current_reference = SIXSTEP_parameters.startup_reference;
#endif
  SIXSTEP_parameters.Speed_Loop_Time = SPEED_LOOP_TIME;
  SIXSTEP_parameters.ALIGNMENT = FALSE;

#ifndef HALL_SENSORS
  SIXSTEP_parameters.Current_ADC_BEMF_channel = 0;  
  SIXSTEP_parameters.numberofitemArr = NUMBER_OF_STEPS;  
  SIXSTEP_parameters.ADC_BEMF_threshold_UP = BEMF_THRSLD_UP_OFF;
  SIXSTEP_parameters.ADC_BEMF_threshold_DOWN = BEMF_THRSLD_DOWN_OFF;
#ifdef PWM_ON_BEMF_SENSING  
  SIXSTEP_parameters.ADC_BEMF_threshold_UP_ON = BEMF_THRSLD_UP_ON; 
  SIXSTEP_parameters.ADC_BEMF_threshold_DOWN_ON = BEMF_THRSLD_DOWN_ON;
  SIXSTEP_parameters.ADC_Current_BEMF_thld_UP = SIXSTEP_parameters.ADC_BEMF_threshold_UP;
  SIXSTEP_parameters.ADC_Current_BEMF_thld_DOWN = SIXSTEP_parameters.ADC_BEMF_threshold_DOWN;
#endif
  SIXSTEP_parameters.demagn_value = INITIAL_DEMAGN_DELAY;
  SIXSTEP_parameters.demagn_counter = 0;   
  SIXSTEP_parameters.prev_step_pos = 0;
  SIXSTEP_parameters.step_position = 0;
#else
  SIXSTEP_parameters.next_step_pos = 0;
#endif
  
  LF_TIMx.Instance->PSC = LF_TIMx.Init.Prescaler;
  LF_TIMx.Instance->ARR = LF_TIMx.Init.Period;
  
  HF_TIMx.Instance->PSC = HF_TIMx.Init.Prescaler;
  HF_TIMx.Instance->ARR = HF_TIMx.Init.Period;
 
  Rotor_poles_pairs = SIXSTEP_parameters.NUMPOLESPAIRS; 
  SIXSTEP_parameters.SYSCLK_frequency = HAL_RCC_GetSysClockFreq();
 
  MC_SixStep_HF_TIMx_SetDutyCycle_CH1(0);
  MC_SixStep_HF_TIMx_SetDutyCycle_CH2(0);
  MC_SixStep_HF_TIMx_SetDutyCycle_CH3(0); 

#ifdef POTENTIOMETER
  SIXSTEP_parameters.ADC_SEQ_Channel[0] = ADC_CH_1;       /*SPEED*/
#endif
#ifdef CURRENT_SENSE_ADC
  SIXSTEP_parameters.ADC_SEQ_Channel[1] = ADC_CH_2;       /*CURRENT*/
#endif 
#ifdef VBUS_SENSE_ADC 
  SIXSTEP_parameters.ADC_SEQ_Channel[2] = ADC_CH_3;       /*VBUS*/
#endif
#ifdef TEMP_SENSE_ADC
  SIXSTEP_parameters.ADC_SEQ_Channel[3] = ADC_CH_4;       /*TEMP*/
#endif

  SIXSTEP_parameters.ALIGN_OK = FALSE;
  SIXSTEP_parameters.VALIDATION_OK = 0;
  SIXSTEP_parameters.ARR_OK = 0;
  SIXSTEP_parameters.speed_fdbk_filtered = 0;
#ifndef PID
  SIXSTEP_parameters.Integral_Term_sum = 0;
#endif
  SIXSTEP_parameters.STATUS = STOP;
  SIXSTEP_parameters.RUN_Motor = 0;
  SIXSTEP_parameters.speed_fdbk = 0;
  SIXSTEP_parameters.BEMF_OK = FALSE;
  SIXSTEP_parameters.CL_READY = FALSE;
  SIXSTEP_parameters.SPEED_VALIDATED = FALSE;
  SIXSTEP_parameters.TARGET_SPEED_SWITCHED = FALSE;
  SIXSTEP_parameters.BEMF_Tdown_count = 0;   /* Reset of the Counter to detect Stop motor condition when a stall condition occurs*/
  uwTick = 0;
#ifdef DEMOMODE  
  index_motor_run = 0;
  test_motor_run = 1;
#endif
  Mech_Speed_RPM = 0;
  El_Speed_Hz = 0;
  index_adc_chn = 0;
  index_array = 0;
#ifndef HALL_SENSORS
  SIXSTEP_parameters.ADC_BEMF_channel[0] = ADC_Bemf_CH1;   /*BEMF1*/
  SIXSTEP_parameters.ADC_BEMF_channel[1] = ADC_Bemf_CH2;   /*BEMF2*/
  SIXSTEP_parameters.ADC_BEMF_channel[2] = ADC_Bemf_CH3;   /*BEMF3*/  
  T_single_step = 0;                      
  T_single_step_first_value = 0;          
  delta = 0;                               
  Time_vector_tmp = 0;                  
  Time_vector_prev_tmp = 0;
  mech_accel_hz = 0;           
  constant_k = 0;  
  ARR_LF = 0;
  index_ARR_step = 1;
  n_zcr_startup = 0;
  cnt_bemf_event = 0; 
  startup_bemf_failure = 0;
  speed_fdbk_error = 0;
  lf_timer_failure = 0;
#ifdef PWM_ON_BEMF_SENSING
  zcr_on_ton = 0;
  zcr_on_ton_next = 0;
  HAL_GPIO_WritePin(GPIO_PORT_BEMF,GPIO_CH_BEMF,GPIO_PIN_SET); // Disable divider
#endif
#endif
 
  index_align = 1;
  speed_sum_sp_filt = 0;
  speed_sum_pot_filt = 0; 
  index_pot_filt = 1;  
  potent_filtered = 0;
  Tick_cnt = 0;   
  counter_ARR_Bemf = 0;
  constant_multiplier_tmp = 0;
 
#ifdef POTENTIOMETER
  potentiometer_speed_target = 0;
  potentiometer_prev_speed_target = 0;
  potentiometer_buffer_index =0;
  for(uint16_t i = 0; i < POT_BUFFER_SIZE;i++)
  {
    potentiometer_buffer[i]=0;
  }
#endif
 
  for(uint16_t i = 0; i < FILTER_DEEP;i++)
  {
    speed_tmp_array[i] = 0;
  } 
  array_completed = FALSE;
  
#ifndef VOLTAGE_MODE
  MC_SixStep_Current_Reference_Start();
  MC_SixStep_Current_Reference_Setvalue(SIXSTEP_parameters.startup_reference);
#endif
  
#ifndef HALL_SENSORS
  if (SIXSTEP_parameters.CW_CCW == 0)
    SIXSTEP_parameters.speed_target = TARGET_SPEED_OPEN_LOOP;
  else
    SIXSTEP_parameters.speed_target = -TARGET_SPEED_OPEN_LOOP;
  index_startup_motor = 1;
  MC_SixStep_Ramp_Motor_calc();
#else
  if (SIXSTEP_parameters.CW_CCW == 0)
    SIXSTEP_parameters.speed_target = TARGET_SPEED;
  else
    SIXSTEP_parameters.speed_target = -TARGET_SPEED;
#endif
  
  MC_Set_PI_param(&PI_parameters);
}

/**
  * @} 
  */

/** @defgroup MC_SixStep_Ramp_Motor_calc    MC_SixStep_Ramp_Motor_calc
  *  @{
    * @brief Calculate the acceleration profile step by step for motor during start-up 
    * @retval None
*/
#ifndef HALL_SENSORS 
void MC_SixStep_Ramp_Motor_calc()
{
  static uint32_t calc = 0;
  if (calc!=0)
  {
    calc = 0;
  }
  else
  {
    calc++;
    uint32_t constant_multiplier = 100;
    uint32_t constant_multiplier_2 = 4000000000;  
  
    if(index_startup_motor == 1)
    { 
      mech_accel_hz = SIXSTEP_parameters.ACCEL * Rotor_poles_pairs / 60; 
      constant_multiplier_tmp = (uint64_t)constant_multiplier*(uint64_t)constant_multiplier_2;
      constant_k = constant_multiplier_tmp/(3*mech_accel_hz);
#ifndef VOLTAGE_MODE
      MC_SixStep_Current_Reference_Setvalue(SIXSTEP_parameters.startup_reference);
#else    
      MC_SixStep_HF_TIMx_SetDutyCycle(SIXSTEP_parameters.startup_reference,SIXSTEP_parameters.step_position);
#endif  
      Time_vector_prev_tmp = 0;
    }
    if(index_startup_motor < NUMBER_OF_STEPS)  
    {
      Time_vector_tmp = ((uint64_t) 1000 * (uint64_t)1000 * (uint64_t) MCM_Sqrt(((uint64_t)index_startup_motor * (uint64_t)constant_k)))/632455;   
      delta = Time_vector_tmp - Time_vector_prev_tmp;
      if(index_startup_motor==1)
      { 
        T_single_step_first_value = (2 * 3141)*delta/1000;
        SIXSTEP_parameters.LF_TIMx_ARR = (uint32_t)(65535); 
      }
      else 
      {
        T_single_step = (2 * 3141)*delta/1000;
        SIXSTEP_parameters.LF_TIMx_ARR = (uint32_t)(65535 * T_single_step)/(T_single_step_first_value);
      }
    }
    else index_startup_motor=1;
    if(SIXSTEP_parameters.STATUS != ALIGNMENT && SIXSTEP_parameters.STATUS != START)
    {
      index_startup_motor++;     
    }
    else Time_vector_tmp = 0;
    Time_vector_prev_tmp =  Time_vector_tmp;
    calc = 0;
  }
}
#endif
/**
  * @} 
  */

/**
  * @brief  It calculates the square root of a non-negative s64. 
  *   It returns 0 for negative s64.
  * @param  Input uint64_t number
  * @retval int32_t Square root of Input (0 if Input<0)
  */
uint64_t MCM_Sqrt(uint64_t wInput)
{
  uint8_t biter = 0u;
  uint64_t wtemproot;
  uint64_t wtemprootnew;
  
    if (wInput <= (uint64_t)((uint64_t)2097152<<shift_n_sqrt))  
    {
      wtemproot = (uint64_t)((uint64_t)128<<shift_n_sqrt);  
    }
    else
    {
      wtemproot = (uint64_t)((uint64_t)8192<<shift_n_sqrt);  
    }
    
    do
    {
      wtemprootnew = (wtemproot + wInput/wtemproot)>>1;
      if (wtemprootnew == wtemproot)
      {
        biter = (shift_n_sqrt-1);
      }
      else
      {
        biter ++;
        wtemproot = wtemprootnew;
      }
    }
    while (biter < (shift_n_sqrt-1));
  
  return (wtemprootnew); 
}


/** @defgroup MC_SixStep_ARR_step    MC_SixStep_ARR_step
  *  @{
    * @brief Generate the ARR value for Low Frequency TIM during start-up
    * @retval None
*/
#ifndef HALL_SENSORS
void MC_SixStep_ARR_step()
{ 
   
 if(SIXSTEP_parameters.ALIGNMENT == FALSE) 
 {    
   SIXSTEP_parameters.ALIGNMENT = TRUE;   
 }
 if(SIXSTEP_parameters.ALIGN_OK == TRUE)
 {
  if(PI_parameters.Reference >= 0) 
   {
     if(SIXSTEP_parameters.VALIDATION_OK == FALSE)
     {  
      SIXSTEP_parameters.STATUS = STARTUP;
      MC_SixStep_Ramp_Motor_calc();       
      if(index_ARR_step < SIXSTEP_parameters.numberofitemArr)
      {
        LF_TIMx.Instance->ARR = SIXSTEP_parameters.LF_TIMx_ARR;
        index_ARR_step++;
      } 
      else if(SIXSTEP_parameters.ARR_OK == 0)
      {  
       index_ARR_step = 1; 
       SIXSTEP_parameters.ACCEL>>=1;  
       if(SIXSTEP_parameters.ACCEL < MINIMUM_ACC)
       {
         SIXSTEP_parameters.ACCEL = MINIMUM_ACC;
       }       
       MC_StopMotor(); 
       SIXSTEP_parameters.STATUS = STARTUP_FAILURE;
       BSP_BOARD_FAULT_LED_ON();
#ifdef UART_COMM
       char *pCommandString = "STATUS\r\n";
       CMD_Parser(pCommandString);
#endif
      }
     }
     else 
     { 
       SIXSTEP_parameters.ARR_OK = 1;
       index_startup_motor = 1;       
       index_ARR_step = 1;   
     } 
   }
  else 
    {
     if(SIXSTEP_parameters.VALIDATION_OK == FALSE)
      {  
      SIXSTEP_parameters.STATUS = STARTUP;
      MC_SixStep_Ramp_Motor_calc();       
      if(index_ARR_step < SIXSTEP_parameters.numberofitemArr)
      {
        LF_TIMx.Instance->ARR = SIXSTEP_parameters.LF_TIMx_ARR;
        index_ARR_step++;
      } 
      else if(SIXSTEP_parameters.ARR_OK==0)
      {  
         index_ARR_step = 1;   
         SIXSTEP_parameters.ACCEL>>=1; 
         if(SIXSTEP_parameters.ACCEL < MINIMUM_ACC)
         {
           SIXSTEP_parameters.ACCEL = MINIMUM_ACC;
         }         
         MC_StopMotor();
         BSP_BOARD_FAULT_LED_ON();
         SIXSTEP_parameters.STATUS = STARTUP_FAILURE;  
#ifdef UART_COMM
         char *pCommandString = "STATUS\r\n";
         CMD_Parser(pCommandString);
#endif         
      }
      }
      else 
       { 
         SIXSTEP_parameters.ARR_OK = 1;
         index_startup_motor = 1;       
         index_ARR_step = 1;   
       } 
   }
 } 
}
#endif

/** @defgroup MC_TIMx_SixStep_CommutationEvent    MC_TIMx_SixStep_CommutationEvent
  *  @{
    * @brief Capture the counter value for which the hall sensors status changed
    * and set the delay at which the HF_TIMx will change according to the new
    * hall status value and hence new step
    * @retval None
*/
#ifdef HALL_SENSORS
void MC_TIMx_SixStep_CommutationEvent()
{ 
  SIXSTEP_parameters.hall_capture = __HAL_TIM_GetCompare(&LF_TIMx,TIM_CHANNEL_1);
  SIXSTEP_parameters.hall_ok = 1;
  if (SIXSTEP_parameters.start_cnt <= 0)
  {
    SIXSTEP_parameters.STATUS = RUN;
  }
  else
  {
    SIXSTEP_parameters.start_cnt -= START_COUNTER_STEPS_DECREMENTATION;
    if (SIXSTEP_parameters.start_cnt <= 0) SIXSTEP_parameters.start_cnt = 0;
#ifdef FIXED_HALL_DELAY
    SIXSTEP_parameters.commutation_delay = COMMUTATION_DELAY + (SIXSTEP_parameters.start_cnt<<6);
#endif
  }  
#ifndef FIXED_HALL_DELAY
  SIXSTEP_parameters.commutation_delay = SIXSTEP_parameters.hall_capture>>1;
#endif
  __HAL_TIM_SetCompare(&LF_TIMx,TIM_CHANNEL_2,SIXSTEP_parameters.commutation_delay);
}
#endif
/**
  * @} 
  */

/** @defgroup MC_SixStep_Alignment    MC_SixStep_Alignment
  *  @{
    * @brief Generate the motor alignment
    * @retval None
*/
#ifndef HALL_SENSORS
void MC_SixStep_Alignment()
{
   SIXSTEP_parameters.step_position = 6;
#ifdef ALL_WINDINGS_ENERGIZATION
   MC_SixStep_HF_TIMx_SetDutyCycle_CH3(SIXSTEP_parameters.pulse_value);
   MC_SixStep_HF_TIMx_SetDutyCycle_CH2(0);
   MC_SixStep_HF_TIMx_SetDutyCycle_CH1(SIXSTEP_parameters.pulse_value);  
   HAL_TIM_PWM_Start(&HF_TIMx,BSP_SIP_HF_TIMx_CH3);
   HAL_TIM_PWM_Stop(&HF_TIMx,BSP_SIP_BSP_SIP_HF_TIMx_CH2);
   HAL_TIM_PWM_Start(&HF_TIMx,BSP_SIP_HF_TIMx_CH1);   
#else
#ifndef VOLTAGE_MODE
   MC_SixStep_Current_Reference_Setvalue(SIXSTEP_parameters.startup_reference);
   MC_SixStep_HF_TIMx_SetDutyCycle(SIXSTEP_parameters.pulse_value,SIXSTEP_parameters.step_position);
#else
   MC_SixStep_HF_TIMx_SetDutyCycle(SIXSTEP_parameters.startup_reference,SIXSTEP_parameters.step_position);
#endif
   MC_SixStep_EnableInput_CH1_D_CH2_E_CH3_E(0);
#endif
   SIXSTEP_parameters.Current_ADC_BEMF_channel = SIXSTEP_parameters.ADC_BEMF_channel[0];
   SIXSTEP_parameters.STATUS = ALIGNMENT;
   
   index_align++;
   if(index_align >= TIME_FOR_ALIGN+1) 
    { 
      SIXSTEP_parameters.ALIGN_OK = TRUE;
      SIXSTEP_parameters.STATUS = STARTUP;
      HAL_ADC_Start_IT(&ADCx);
      index_startup_motor = 1;      
      MC_SixStep_Ramp_Motor_calc();       
      index_align = 0;
    }
}
#endif

/**
  * @} 
  */

/** @defgroup MC_Set_PI_param    MC_Set_PI_param
  *  @{
    * @brief Set all parameters for PI regulator
    * @param  PI_PARAM
    * @retval None
*/

void MC_Set_PI_param(SIXSTEP_PI_PARAM_InitTypeDef_t *PI_PARAM)
{
  if (((SIXSTEP_parameters.CW_CCW == 0)&&(SIXSTEP_parameters.speed_target < 0))||\
      ((SIXSTEP_parameters.CW_CCW != 0)&&(SIXSTEP_parameters.speed_target > 0)))
  {
    SIXSTEP_parameters.speed_target *= -1;
  }  
#ifdef SPEED_RAMP
  if (PI_PARAM->ReferenceToBeUpdated == 0) PI_PARAM->ReferenceToBeUpdated++;
  if(SIXSTEP_parameters.CW_CCW == 0)
    PI_PARAM->Reference = TARGET_SPEED_OPEN_LOOP;
  else
    PI_PARAM->Reference = -TARGET_SPEED_OPEN_LOOP;
#else 
  PI_PARAM->Reference = SIXSTEP_parameters.speed_target;
#endif 
  PI_PARAM->Kp_Gain = SIXSTEP_parameters.KP;   
  PI_PARAM->Ki_Gain = SIXSTEP_parameters.KI;
#ifdef PID  
  PI_PARAM->Kd_Gain = SIXSTEP_parameters.KD;
#endif
  PI_PARAM->Lower_Limit_Output = LOWER_OUT_LIMIT;       
  PI_PARAM->Upper_Limit_Output = UPPER_OUT_LIMIT;      
}

/**
  * @} 
  */

/** @defgroup MC_PI_Controller    MC_PI_Controller
  *  @{
    * @brief Compute the PI output for the Current Reference
    * @param  PI_PARAM PI parameters structure
    * @param  speed_fdb motor_speed_value
    * @retval int16_t Currente reference 
*/
#ifdef PID
uint16_t MC_PI_Controller(SIXSTEP_PI_PARAM_InitTypeDef_t *PI_PARAM, int32_t speed_fdb)
{
  int32_t wProportional_Term=0, wIntegral_Term=0, wDerivative_Term=0, wOutput_32=0;
  int32_t error0;

  if (PI_PARAM->Reference>0)
    error0 = (PI_PARAM->Reference - speed_fdb);
  else
    error0 = (speed_fdb - PI_PARAM->Reference);
  
  /* Proportional term computation*/
  wProportional_Term = PI_PARAM->Kp_Gain * error0;
    
  /* Integral term computation */
  wIntegral_Term = PI_PARAM->Ki_Gain * SIXSTEP_parameters.error1;
  
  /* Derivative computation */
  wDerivative_Term = PI_PARAM->Kd_Gain * SIXSTEP_parameters.error2;

  SIXSTEP_parameters.error2 = SIXSTEP_parameters.error1;
  SIXSTEP_parameters.error1 = error0;

#ifndef VOLTAGE_MODE  
  wOutput_32 = SIXSTEP_parameters.current_reference
#else
  wOutput_32 = SIXSTEP_parameters.pulse_value    
#endif
    + ((wProportional_Term - wIntegral_Term + wDerivative_Term)>>K_GAIN_SCALING);
  
  if (wOutput_32 > PI_PARAM->Upper_Limit_Output)
  {
    wOutput_32 = PI_PARAM->Upper_Limit_Output;
  }
  else if (wOutput_32 < PI_PARAM->Lower_Limit_Output)
  {
    wOutput_32 = PI_PARAM->Lower_Limit_Output;
  }

  return((uint16_t)(wOutput_32));
}
#else
uint16_t MC_PI_Controller(SIXSTEP_PI_PARAM_InitTypeDef_t *PI_PARAM, int32_t speed_fdb)
{
  int32_t wProportional_Term=0, wIntegral_Term=0, wOutput_32=0;
  int32_t Error;

  if (PI_PARAM->Reference>0)
    Error = (PI_PARAM->Reference - speed_fdb);
  else
    Error = (speed_fdb - PI_PARAM->Reference);
  
  /* Proportional term computation*/
  wProportional_Term = PI_PARAM->Kp_Gain * Error;
    
  /* Integral term computation */
  wIntegral_Term = PI_PARAM->Ki_Gain * SIXSTEP_parameters.Integral_Term_sum;
  SIXSTEP_parameters.Integral_Term_sum = Error;

#ifndef VOLTAGE_MODE  
  wOutput_32 = SIXSTEP_parameters.current_reference
#else
  wOutput_32 = SIXSTEP_parameters.pulse_value    
#endif
    + ((wProportional_Term - wIntegral_Term)>>K_GAIN_SCALING); 
  
  if (wOutput_32 > PI_PARAM->Upper_Limit_Output)
  {
    wOutput_32 = PI_PARAM->Upper_Limit_Output;
  }
  else if (wOutput_32 < PI_PARAM->Lower_Limit_Output)
  {
    wOutput_32 = PI_PARAM->Lower_Limit_Output;
  }

  return((uint16_t)(wOutput_32));
}
#endif
/**
  * @} 
  */


/** @defgroup MC_Task_Speed    MC_Task_Speed
  *  @{
    * @brief Main task: Speed Loop with PI regulator
    * @retval None
*/
#ifndef HALL_SENSORS
void MC_Task_Speed()
{
  if (SIXSTEP_parameters.VALIDATION_OK == FALSE)
  {
    if (SIXSTEP_parameters.speed_target < 0)
    {
      if (SIXSTEP_parameters.speed_fdbk_filtered <= SIXSTEP_parameters.speed_target)
      {
        SIXSTEP_parameters.STATUS = VALIDATION;   
        SIXSTEP_parameters.SPEED_VALIDATED = TRUE;        
      }
    }
    else
    {
      if (SIXSTEP_parameters.speed_fdbk_filtered >= SIXSTEP_parameters.speed_target)
      {
        SIXSTEP_parameters.STATUS = VALIDATION;   
        SIXSTEP_parameters.SPEED_VALIDATED = TRUE;        
      }
    }
  }
  
  if(SIXSTEP_parameters.SPEED_VALIDATED == TRUE && SIXSTEP_parameters.BEMF_OK == TRUE && SIXSTEP_parameters.CL_READY != TRUE)
  {
    SIXSTEP_parameters.CL_READY = TRUE;  
  }
   
  if(SIXSTEP_parameters.VALIDATION_OK != FALSE)
  {    
  /****************************************************************************/           
    SIXSTEP_parameters.STATUS = RUN; 
  /****************************************************************************/  
#ifdef POTENTIOMETER
    potentiometer_speed_target = (((uint32_t) MC_Potentiometer_filter() * MAX_POT_SPEED ) >> 12);
    if(potentiometer_speed_target < MIN_POT_SPEED)
      potentiometer_speed_target = MIN_POT_SPEED;
    if ((potentiometer_speed_target>(potentiometer_prev_speed_target+ADC_SPEED_TH))||
        (potentiometer_prev_speed_target>(potentiometer_speed_target+ADC_SPEED_TH)))
    {
      potentiometer_prev_speed_target = potentiometer_speed_target;
      if (SIXSTEP_parameters.CW_CCW != 0)
      {
        SIXSTEP_parameters.speed_target = -potentiometer_speed_target;
      }
      else
      {
        SIXSTEP_parameters.speed_target = potentiometer_speed_target;        
      }
#ifdef SPEED_RAMP      
      PI_parameters.ReferenceToBeUpdated++;
#else
      PI_parameters.Reference = SIXSTEP_parameters.speed_target;
#endif
    }
#else
    if(SIXSTEP_parameters.TARGET_SPEED_SWITCHED==FALSE)
    {
      if (SIXSTEP_parameters.CW_CCW == 0)
      {
        SIXSTEP_parameters.speed_target = TARGET_SPEED;       
      }
      else
      {
        SIXSTEP_parameters.speed_target = -TARGET_SPEED;        
      }
#ifndef SPEED_RAMP
      PI_parameters.Reference = SIXSTEP_parameters.speed_target;
#endif      
      SIXSTEP_parameters.TARGET_SPEED_SWITCHED=TRUE;
    }
#endif
#ifdef SPEED_RAMP
  if(PI_parameters.ReferenceToBeUpdated != 0)
  {
    if(PI_parameters.Reference < SIXSTEP_parameters.speed_target)
    {
      PI_parameters.Reference += ((SIXSTEP_parameters.Speed_Loop_Time*SIXSTEP_parameters.ACCEL) >> 10);
      if (PI_parameters.Reference > SIXSTEP_parameters.speed_target) PI_parameters.Reference = SIXSTEP_parameters.speed_target;
    }
    else if (PI_parameters.Reference > SIXSTEP_parameters.speed_target)
    {
      PI_parameters.Reference -= ((SIXSTEP_parameters.Speed_Loop_Time*SIXSTEP_parameters.ACCEL) >> 10);
      if (PI_parameters.Reference < SIXSTEP_parameters.speed_target) PI_parameters.Reference = SIXSTEP_parameters.speed_target;
    }
    else
    {
      PI_parameters.ReferenceToBeUpdated = 0;
    }        
  }
#endif
#ifndef VOLTAGE_MODE
    SIXSTEP_parameters.current_reference = MC_PI_Controller(&PI_parameters,SIXSTEP_parameters.speed_fdbk_filtered);  
    MC_SixStep_Current_Reference_Setvalue(SIXSTEP_parameters.current_reference);
#else 
    SIXSTEP_parameters.pulse_value = MC_PI_Controller(&PI_parameters,SIXSTEP_parameters.speed_fdbk_filtered);
#ifdef PWM_ON_BEMF_SENSING
    if (SIXSTEP_parameters.pulse_value > DUTY_CYCLE_50)
    {
      zcr_on_ton_next = 1; // Duty-cycle > 50 %
    }
    else
    {
      zcr_on_ton_next = 0; // Duty-cycle <= 50 %
    }
#endif
    MC_SixStep_HF_TIMx_SetDutyCycle(SIXSTEP_parameters.pulse_value,SIXSTEP_parameters.step_position);
#endif
  }
    if (SIXSTEP_parameters.speed_fdbk_filtered>0)
    {
    if (SIXSTEP_parameters.speed_fdbk_filtered>DEMAG_SPEED_THRESHOLD)
    {
      SIXSTEP_parameters.demagn_value = MINIMUM_DEMAGN_DELAY;
    }
    else if (SIXSTEP_parameters.speed_fdbk_filtered!=0)
    {
      SIXSTEP_parameters.demagn_value = K_DEMAG/SIXSTEP_parameters.speed_fdbk_filtered;
    }
  }
    else
    {
    if ((-SIXSTEP_parameters.speed_fdbk_filtered)>DEMAG_SPEED_THRESHOLD)
    {
      SIXSTEP_parameters.demagn_value = MINIMUM_DEMAGN_DELAY;
    }
    else if (SIXSTEP_parameters.speed_fdbk_filtered!=0)
    {      
      SIXSTEP_parameters.demagn_value = K_DEMAG/(-SIXSTEP_parameters.speed_fdbk_filtered);
    }
  }
}
#else
void MC_Task_Speed()
{
  if (SIXSTEP_parameters.STATUS == RUN)
  {
#ifdef POTENTIOMETER
    MC_SixStep_ADC_Channel(SIXSTEP_parameters.ADC_SEQ_Channel[0]);
    potentiometer_speed_target = (((uint32_t) MC_Potentiometer_filter() * MAX_POT_SPEED ) >> 12);
    if(potentiometer_speed_target < MIN_POT_SPEED)
      potentiometer_speed_target = MIN_POT_SPEED;
    if ((potentiometer_speed_target>(potentiometer_prev_speed_target+ADC_SPEED_TH))||
        (potentiometer_prev_speed_target>(potentiometer_speed_target+ADC_SPEED_TH)))
    {
      potentiometer_prev_speed_target = potentiometer_speed_target;
      if (SIXSTEP_parameters.CW_CCW != 0)
      {
        SIXSTEP_parameters.speed_target = -potentiometer_speed_target;
      }
      else
      {
        SIXSTEP_parameters.speed_target = potentiometer_speed_target;        
      }      
#ifdef SPEED_RAMP      
      PI_parameters.ReferenceToBeUpdated++;
#else
      PI_parameters.Reference = SIXSTEP_parameters.speed_target;
#endif
    }
#endif
#ifdef SPEED_RAMP
    if(PI_parameters.ReferenceToBeUpdated != 0)
    {
      if(PI_parameters.Reference < SIXSTEP_parameters.speed_target)
      {
        PI_parameters.Reference += ((SIXSTEP_parameters.Speed_Loop_Time*SIXSTEP_parameters.ACCEL) >> 10);
        if (PI_parameters.Reference > SIXSTEP_parameters.speed_target) PI_parameters.Reference = SIXSTEP_parameters.speed_target;
      }
      else if (PI_parameters.Reference > SIXSTEP_parameters.speed_target)
      {
        PI_parameters.Reference -= ((SIXSTEP_parameters.Speed_Loop_Time*SIXSTEP_parameters.ACCEL) >> 10);
        if (PI_parameters.Reference < SIXSTEP_parameters.speed_target) PI_parameters.Reference = SIXSTEP_parameters.speed_target;
      }
      else
      {
        PI_parameters.ReferenceToBeUpdated = 0;
      }        
    }
#endif  
#ifndef VOLTAGE_MODE
    SIXSTEP_parameters.current_reference = MC_PI_Controller(&PI_parameters,SIXSTEP_parameters.speed_fdbk_filtered);    
    MC_SixStep_Current_Reference_Setvalue(SIXSTEP_parameters.current_reference);
#else
    SIXSTEP_parameters.pulse_value = MC_PI_Controller(&PI_parameters,SIXSTEP_parameters.speed_fdbk_filtered);
    __disable_irq();
    MC_SixStep_HF_TIMx_SetDutyCycle(SIXSTEP_parameters.pulse_value,SIXSTEP_parameters.step_position);
    __enable_irq();
#endif
  }
}
#endif
     
/**
  * @} 
  */

/** @defgroup MC_Set_Speed    MC_Set_Speed
  *  @{
    * @brief Set the new motor speed value
    * @param  speed_value:  set new motor speed
    * @retval None
*/
void MC_Set_Speed(uint16_t speed_value)
{
#ifdef SPEED_RAMP  
  PI_parameters.ReferenceToBeUpdated++;
  if(SIXSTEP_parameters.CW_CCW == 0)
    SIXSTEP_parameters.speed_target = speed_value;
  else
    SIXSTEP_parameters.speed_target = -speed_value;  
#else  
  if(SIXSTEP_parameters.CW_CCW == 0)
  {
    PI_parameters.Reference = speed_value;
  }
  else
  {
    PI_parameters.Reference = -speed_value;
  }
  SIXSTEP_parameters.speed_target = PI_parameters.Reference;
#endif
}

/**
  * @} 
  */

/** @defgroup MC_StartMotor    MC_StartMotor
  *  @{
    * @brief Start the Motor
    * @retval None
*/
void MC_StartMotor()
{
  BSP_BOARD_FAULT_LED_OFF();
  Rotor_poles_pairs = SIXSTEP_parameters.NUMPOLESPAIRS;
#ifdef TEST
  stop = 0;
#endif  
#ifdef HALL_SENSORS
  SIXSTEP_parameters.hall_ok = 0;
  SIXSTEP_parameters.hall_ko_successive = 0;
  SIXSTEP_parameters.start_attempts = NUMBER_OF_STARTS;
  SIXSTEP_parameters.run_attempts = SIXSTEP_parameters.start_attempts;
  SIXSTEP_parameters.start_cnt = NUMBER_OF_STEPS;
#endif
#ifdef BEMF_RECORDING
  SIXSTEP_parameters.bemfIndexRx = 5;
  SIXSTEP_parameters.bemfMeasurements = 0;
  bemfArray[0]= 0xABBA;
  bemfArray[1]= 0xCDDC;
  bemfArray[2]= 0xEFFE;
  bemfArray[BEMF_ARRAY_SIZE+5] = 0xFACE;
#endif
  uwTick = 0;
  SIXSTEP_parameters.STATUS = START;
#ifndef HALL_SENSORS
  HAL_TIM_Base_Start_IT(&LF_TIMx);
#else
  sConfig.OCMode = TIM_OCMODE_PWM1;
  sConfig.OCFastMode = TIM_OCFAST_DISABLE;
  sConfig.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfig.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfig.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfig.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  SIXSTEP_parameters.ALIGNMENT = TRUE;
#ifndef VOLTAGE_MODE
  SIXSTEP_parameters.pulse_value = PULSE;
#else  
  SIXSTEP_parameters.pulse_value = SIXSTEP_parameters.startup_reference;
#endif
#ifdef ALL_WINDINGS_ENERGIZATION
  SIXSTEP_parameters.step_position = 1;
  MC_SixStep_HF_TIMx_SetDutyCycle(SIXSTEP_parameters.pulse_value,SIXSTEP_parameters.step_position);
  sConfig.Pulse = SIXSTEP_parameters.pulse_value;
  HAL_TIM_PWM_ConfigChannel(&HF_TIMx, &sConfig, BSP_SIP_HF_TIMx_CH1);
  HAL_TIM_PWM_Start(&HF_TIMx,BSP_SIP_HF_TIMx_CH1);
  HAL_TIMEx_PWMN_Start(&HF_TIMx,BSP_SIP_HF_TIMx_CH1);
  sConfig.Pulse = 0;
  HAL_TIM_PWM_ConfigChannel(&HF_TIMx, &sConfig, BSP_SIP_HF_TIMx_CH2);
  HAL_TIM_PWM_Start(&HF_TIMx,BSP_SIP_HF_TIMx_CH2);
  HAL_TIMEx_PWMN_Start(&HF_TIMx,BSP_SIP_HF_TIMx_CH2);
  sConfig.Pulse = 0;
  HAL_TIM_PWM_ConfigChannel(&HF_TIMx, &sConfig, BSP_SIP_HF_TIMx_CH3);
  HAL_TIM_PWM_Start(&HF_TIMx,BSP_SIP_HF_TIMx_CH3);
  HAL_TIMEx_PWMN_Start(&HF_TIMx,BSP_SIP_HF_TIMx_CH3);
#else
  SIXSTEP_parameters.step_position = 1;
  sConfig.Pulse = SIXSTEP_parameters.pulse_value;
  HAL_TIM_PWM_ConfigChannel(&HF_TIMx, &sConfig, BSP_SIP_HF_TIMx_CH1);
  HAL_TIM_PWM_Start(&HF_TIMx,BSP_SIP_HF_TIMx_CH1);
  HAL_TIMEx_PWMN_Start(&HF_TIMx,BSP_SIP_HF_TIMx_CH1);
  sConfig.Pulse = 0;
  HAL_TIM_PWM_ConfigChannel(&HF_TIMx, &sConfig, BSP_SIP_HF_TIMx_CH2);
  HAL_TIM_PWM_Start(&HF_TIMx,BSP_SIP_HF_TIMx_CH2);
  HAL_TIMEx_PWMN_Start(&HF_TIMx,BSP_SIP_HF_TIMx_CH2);
  HAL_TIM_PWM_Stop(&HF_TIMx,BSP_SIP_HF_TIMx_CH3);
  HAL_TIMEx_PWMN_Stop(&HF_TIMx,BSP_SIP_HF_TIMx_CH3);
#endif
  __HAL_TIM_ENABLE_IT(&LF_TIMx, TIM_IT_CC2);
  HAL_TIMEx_ConfigCommutationEvent_IT(&HF_TIMx, TIM_TS_ITR1, TIM_COMMUTATION_TRGI);
  SIXSTEP_parameters.ALIGN_OK = TRUE;
  SIXSTEP_parameters.STATUS = STARTUP;
  HAL_TIMEx_HallSensor_Start_IT(&LF_TIMx);
#ifdef POTENTIOMETER  
  index_adc_chn = 0;
  HAL_ADC_Start_IT(&ADCx);
  MC_SixStep_ADC_Channel(SIXSTEP_parameters.ADC_SEQ_Channel[0]);
#endif
#endif
  SIXSTEP_parameters.RUN_Motor = 1;
  Enable_start_button = FALSE;
}
/**
  * @} 
  */

/** @defgroup MC_StopMotor    MC_StopMotor
  *  @{
    * @brief Stop the Motor
    * @retval None  
*/
void MC_StopMotor()
{     
  uwTick = 0;
#ifdef TEST
  stop = 1;
  while(SIXSTEP_parameters.start_cnt<5000000)
  {
    SIXSTEP_parameters.start_cnt++;
  }
#endif
#ifdef HALL_SENSORS
  SIXSTEP_parameters.hall_ok = 0;
#endif  
  HF_TIMx.Instance->CR1 &= ~(TIM_CR1_CEN);
  HF_TIMx.Instance->CNT = 0;  
  MC_SixStep_DisableInput_CH1_D_CH2_D_CH3_D();
#ifdef HALL_SENSORS
  __HAL_TIM_DISABLE_IT(&LF_TIMx, TIM_IT_CC2);
  HAL_TIMEx_HallSensor_Stop_IT(&LF_TIMx);
#else
  HAL_TIM_Base_Stop_IT(&LF_TIMx);  
#endif
  LF_TIMx.Instance->CCR1 = 0;
  LF_TIMx.Instance->CCR2 = LF_TIMX_ARR;
#if (!defined(HALL_SENSORS)||defined(POTENTIOMETER))  
  HAL_ADC_Stop_IT(&ADCx);
#endif
#ifndef VOLTAGE_MODE  
  MC_SixStep_Current_Reference_Stop();
#endif
  MC_SixStep_RESET();
  Enable_start_button = FALSE;
}

/**
  * @} 
  */

#ifndef HALL_SENSORS
/** @defgroup MC_GetElSpeedHz    MC_GetElSpeedHz
  *  @{
    * @brief Get the Eletrical Motor Speed from ARR value of LF TIM
    * @retval int32_t Return the electrical motor speed
*/
int32_t MC_GetElSpeedHz()
{   
   if(__HAL_TIM_GetAutoreload(&LF_TIMx) != 0xFFFF)
   {
     uint16_t prsc = LF_TIMx.Instance->PSC;
     El_Speed_Hz = SIXSTEP_parameters.SYSCLK_frequency/((++__HAL_TIM_GetAutoreload(&LF_TIMx))*6*(++prsc)); 
   }
   else 
     El_Speed_Hz = 0;
   if(PI_parameters.Reference<0)
    return (-El_Speed_Hz);
   else 
    return (El_Speed_Hz);
}
/**
  * @} 
  */
#else
int32_t MC_GetElSpeedHz()
{ 
  if(SIXSTEP_parameters.hall_capture >= STEP_DURATION_MINIMUM)
  {
    uint16_t prsc = LF_TIMx.Instance->PSC;
    El_Speed_Hz = (int32_t)((SIXSTEP_parameters.SYSCLK_frequency)/((++prsc)*SIXSTEP_parameters.hall_capture*6)); 
  }
  if(PI_parameters.Reference<0)
    return (-El_Speed_Hz);
  else 
    return (El_Speed_Hz);
}
#endif

/** @defgroup MC_GetMechSpeedRPM    MC_GetMechSpeedRPM
  *  @{
    * @brief Get the Mechanical Motor Speed (RPM)
    * @retval int32_t Return the mechanical motor speed (RPM)
*/
#ifdef HALL_SENSORS
int32_t MC_GetMechSpeedRPM()
{  
  uint16_t prsc = LF_TIMx.Instance->PSC;
  if(SIXSTEP_parameters.hall_capture >= STEP_DURATION_MINIMUM)
  {
    Mech_Speed_RPM = (int32_t)(SIXSTEP_parameters.SYSCLK_frequency*10)/
      ((++prsc)*Rotor_poles_pairs*SIXSTEP_parameters.hall_capture);
  }
  return Mech_Speed_RPM;
}
#else
int32_t MC_GetMechSpeedRPM()
{  
  uint16_t prsc = LF_TIMx.Instance->PSC;
  Mech_Speed_RPM = (SIXSTEP_parameters.SYSCLK_frequency*10)/
    ((++SIXSTEP_parameters.LF_TIMx_ARR)*(++prsc)*Rotor_poles_pairs);
  return Mech_Speed_RPM;
}
#endif

/**
  * @} 
  */

/** @defgroup MC_SixStep_Init_main_data    MC_SixStep_Init_main_data
  *  @{
    * @brief Init the main variables for motor driving from MC_SixStep_param.h
    * @retval None
*/

void MC_SixStep_Init_main_data()
{ 
#ifdef VOLTAGE_MODE
  SIXSTEP_parameters.startup_reference = (STARTUP_DUTY_CYCLE*SIXSTEP_parameters.HF_TIMx_ARR)/1000;
#else
  SIXSTEP_parameters.startup_reference = STARTUP_CURRENT_REFERENCE;
#endif
  SIXSTEP_parameters.NUMPOLESPAIRS = NUM_POLE_PAIRS;
  SIXSTEP_parameters.ACCEL = ACC;
  SIXSTEP_parameters.KP = KP_GAIN;   
  SIXSTEP_parameters.KI = KI_GAIN;
#ifdef PID
  SIXSTEP_parameters.KD = KD_GAIN;
#endif
  SIXSTEP_parameters.CW_CCW = DIRECTION;
  SIXSTEP_parameters.overcurrent = 0;
}

/**
  * @} 
  */


/** @defgroup MC_SixStep_INIT    MC_SixStep_INIT
  *  @{
    * @brief Initialitation function for SixStep library
    * @retval None
*/

void MC_SixStep_INIT()
{
    BSP_SIP_HF_TIMx_IT_BRK_ENABLE();
    MC_SixStep_Init_main_data();
    
    SIXSTEP_parameters.pulse_value  = HF_TIMx.Instance->CCR1;
    
    SIXSTEP_parameters.LF_TIMx_ARR = LF_TIMx.Init.Period;
    SIXSTEP_parameters.LF_TIMx_PSC = LF_TIMx.Init.Prescaler;
    SIXSTEP_parameters.HF_TIMx_ARR = HF_TIMx.Init.Period;
    SIXSTEP_parameters.HF_TIMx_PSC = HF_TIMx.Init.Prescaler;  
    

  #ifdef UART_COMM  
    SIXSTEP_parameters.Button_ready = FALSE;
    SIXSTEP_parameters.UART_MEASUREMENT_TYPE = 0;
    SIXSTEP_parameters.UART_CONTINUOUS_TX_BEMF_MODE = FALSE;
    SIXSTEP_parameters.UART_CONTINUOUS_TX_BEMF_ALLOWED = FALSE;    
    SIXSTEP_parameters.UART_CONTINUOUS_TX_SPEED_MODE = FALSE;
    SIXSTEP_parameters.UART_CONTINUOUS_TX_SPEED_ALLOWED = FALSE;
    SIXSTEP_parameters.UART_TX_REPLY = FALSE;
    SIXSTEP_parameters.UART_TX_DIFFERED_REPLY = FALSE;
    SIXSTEP_parameters.UART_TX_CANCELLED = 0;
    MC_UI_INIT();               /*!<  Start the UART Communication Task*/
  #endif     

  #ifndef UART_COMM
    SIXSTEP_parameters.Button_ready = TRUE;
  #endif
    MC_SixStep_RESET();
}
      
/**
  * @} 
  */


/** @defgroup MC_TIMx_SixStep_timebase    MC_TIMx_SixStep_timebase
  *  @{
    * @brief Low Frequency Timer Callback - Call the next step and request the filtered speed value
    * @retval None
*/

void MC_TIMx_SixStep_timebase()
{
  MC_SixStep_NEXT_step(PI_parameters.Reference);                                /*Change STEP number  */
#ifndef HALL_SENSORS
  if(SIXSTEP_parameters.ARR_OK == 0) 
  {
    MC_SixStep_ARR_step();                                                       /*BASE TIMER - ARR modification for STEP frequency changing */ 
  }
#endif
  MC_Speed_Filter();                                                            /*Calculate SPEED filtered  */
}

/**
  * @} 
  */

/** @defgroup MC_Speed_Filter    MC_Speed_Filter
  *  @{
    * @brief Calculate the speed filtered
    * @retval None
*/

void MC_Speed_Filter()
{
     speed_sum_sp_filt = 0;
  speed_tmp_array[index_array] = SIXSTEP_parameters.speed_fdbk;    
  if(array_completed == FALSE)
  {  
    for(int16_t i = (index_array-1); i>=0; i--)
     {
       speed_sum_sp_filt = speed_sum_sp_filt + speed_tmp_array[i];
    }  
     index_array++;
      if(index_array >= FILTER_DEEP) 
       {
      index_array = 0;
         array_completed = TRUE;
       }
  }
  else
  { 
    for(int16_t i = (FILTER_DEEP-1); i >= 0; i--)
  {
      speed_sum_sp_filt = speed_sum_sp_filt + speed_tmp_array[i];
    }      
     index_array++;
    if(index_array >= FILTER_DEEP)
       {
      index_array = 0;
       }      
  }
  SIXSTEP_parameters.speed_fdbk_filtered = speed_sum_sp_filt>>FILTER_DEEP_SHIFT;  
}

/**
  * @} 
  */

#ifdef POTENTIOMETER
/** @defgroup MC_Potentiometer_filter    MC_Potentiometer_filter
  *  @{
    * @brief Calculate the filtered potentiometer value 
    * @retval uint16_t Return the averaged potentiometer value 
*/
uint16_t MC_Potentiometer_filter(void)
{ 
  uint16_t i;
  uint16_t min = 0xFFFF;
  uint16_t max = 0;
  uint32_t sum = 0;
  for (i = 0; i < POT_BUFFER_SIZE; i++)
  {
    uint16_t val = potentiometer_buffer[i];
    sum += val;
    if (val > max)
    {
      max = val;
    }
    else if (val < min)
    {
      min = val;
    }
  }
  sum -= max+min; 
  return (sum >> POT_BUFFER_SIZE_SHIFT);
}
/**
  * @} 
  */
#endif

/** @defgroup MC_SysTick_SixStep_MediumFrequencyTask    MC_SysTick_SixStep_MediumFrequencyTask
  *  @{
    * @brief Systick Callback - Call the Speed loop
    * @retval None
*/
#ifndef HALL_SENSORS
void MC_SysTick_SixStep_MediumFrequencyTask()
{
  if(SIXSTEP_parameters.ALIGNMENT != FALSE) 
  {
    if (SIXSTEP_parameters.ALIGN_OK == FALSE)
    {
      MC_SixStep_Alignment();
    }
  }
  
#ifdef UART_COMM  
  if (UART_FLAG_RECEIVE != FALSE) UART_Communication_Task();
#endif
 
#ifdef DEMOMODE
  index_motor_run++;
  if(index_motor_run >= DEMO_START_TIME && test_motor_run == 0)
  {
    MC_StopMotor();
    index_motor_run=0;
    test_motor_run=1;
  }
  if(index_motor_run >= DEMO_STOP_TIME && test_motor_run == 1)
  {
    MC_StartMotor();
    test_motor_run = 0;
    index_motor_run=0;
  }    
#endif

 /* Push button delay time to avoid double command */    
  if((HAL_GetTick() == BUTTON_DELAY) && (Enable_start_button == FALSE))
  {
    Enable_start_button = TRUE; 
  }  
  
  if(speed_fdbk_error!=0)
  {  
    MC_StopMotor();
    SIXSTEP_parameters.STATUS = SPEEDFBKERROR;
    BSP_BOARD_FAULT_LED_ON();
#ifdef UART_COMM
    char *pCommandString = "STATUS\r\n";
    CMD_Parser(pCommandString);
#endif
  }
  /* SIXSTEP_parameters.Speed_Loop_Time x 1msec */
  else if (Tick_cnt >= SIXSTEP_parameters.Speed_Loop_Time)
  {
    MC_Task_Speed();
    Tick_cnt=0;
#if (defined(SPEED_SENDING)&&defined(UART_COMM))
    if ((SIXSTEP_parameters.UART_CONTINUOUS_TX_SPEED_ALLOWED != FALSE) &&\
        (SIXSTEP_parameters.UART_CONTINUOUS_TX_BEMF_MODE == FALSE))
    {
      UART_Send_Speed();
    }
#endif    
  }
  else Tick_cnt++;

  if(startup_bemf_failure!=0)
  {
    SIXSTEP_parameters.ACCEL>>=1;
    if(SIXSTEP_parameters.ACCEL < MINIMUM_ACC)
    {
      SIXSTEP_parameters.ACCEL = MINIMUM_ACC;
    }  
    MC_StopMotor();
    cnt_bemf_event = 0;    
    SIXSTEP_parameters.STATUS = STARTUP_BEMF_FAILURE;  
    BSP_BOARD_FAULT_LED_ON();
#ifdef UART_COMM
    char *pCommandString = "STATUS\r\n";
    CMD_Parser(pCommandString);
#endif
  }

  if (lf_timer_failure!=0)
  {
    MC_StopMotor();
    SIXSTEP_parameters.STATUS = LF_TIMER_FAILURE;
    BSP_BOARD_FAULT_LED_ON();
#ifdef UART_COMM
    char *pCommandString = "STATUS\r\n";
    CMD_Parser(pCommandString);
#endif
  }
  
  if (SIXSTEP_parameters.overcurrent!=0)
  {
    MC_StopMotor();    
    SIXSTEP_parameters.STATUS = OVERCURRENT;    
    BSP_BOARD_FAULT_LED_ON();
#ifdef UART_COMM
    char *pCommandString = "STATUS\r\n";
    CMD_Parser(pCommandString);
#endif
    SIXSTEP_parameters.STATUS = STOP;
    SIXSTEP_parameters.overcurrent = 0;
  }
}
#else
void MC_SysTick_SixStep_MediumFrequencyTask()
{
  
#ifdef UART_COMM  
  if (UART_FLAG_RECEIVE != FALSE) UART_Communication_Task();
#endif
 
#ifdef DEMOMODE
  index_motor_run++;
  if(index_motor_run >= DEMO_START_TIME && test_motor_run == 0)
  {
    MC_StopMotor();
    index_motor_run=0;
    test_motor_run=1;
  }
  if(index_motor_run >= DEMO_STOP_TIME && test_motor_run == 1)
  {
    MC_StartMotor();
    test_motor_run = 0;
    index_motor_run=0;
  }    
#endif

 /* Push button delay time to avoid double command */    
  if((HAL_GetTick() == BUTTON_DELAY) && (Enable_start_button == FALSE))
  {
    Enable_start_button = TRUE; 
  }

  /* SIXSTEP_parameters.Speed_Loop_Time x 1msec */
  if (Tick_cnt >= SIXSTEP_parameters.Speed_Loop_Time)
  {
    MC_Task_Speed();
    Tick_cnt=0;
#if (defined(SPEED_SENDING)&&defined(UART_COMM))
    if ((SIXSTEP_parameters.UART_CONTINUOUS_TX_SPEED_ALLOWED != FALSE) &&\
        (SIXSTEP_parameters.UART_CONTINUOUS_TX_BEMF_MODE == FALSE))
    {
      UART_Send_Speed();
    }
#endif
  }
  else Tick_cnt++;
  
  if (SIXSTEP_parameters.overcurrent!=0)
  {
    MC_StopMotor();    
    SIXSTEP_parameters.STATUS = OVERCURRENT;    
    BSP_BOARD_FAULT_LED_ON();
#ifdef UART_COMM
    char *pCommandString = "STATUS\r\n";
    CMD_Parser(pCommandString);
#endif
    SIXSTEP_parameters.STATUS = STOP;
    SIXSTEP_parameters.overcurrent = 0;
  }
}
#endif

/**
  * @} 
  */

/** @defgroup MC_SixStep_ARR_Bemf    MC_SixStep_ARR_Bemf
  *  @{
    * @brief Calculate the new Autoreload value (ARR) for Low Frequency timer
    * @retval None
*/
#ifndef HALL_SENSORS
void MC_SixStep_ARR_Bemf(uint8_t up_bemf)
{
  if(SIXSTEP_parameters.prev_step_pos != SIXSTEP_parameters.step_position)
  { 
    if(SIXSTEP_parameters.SPEED_VALIDATED!=FALSE) 
    {
#if (GPIO_ZERO_CROSS!=0)
      HAL_GPIO_TogglePin(GPIO_PORT_ZCR,GPIO_CH_ZCR);         
#endif     
      if(cnt_bemf_event> BEMF_CNT_EVENT_MAX)
      {
        startup_bemf_failure = 1;  
      }
     
      if (SIXSTEP_parameters.BEMF_OK==FALSE)
      {
        if(up_bemf!=0)
        {
          n_zcr_startup++;
          cnt_bemf_event = 0;      
        }
        else
        {
          cnt_bemf_event++;
        }  
        if(n_zcr_startup>= NUMBER_ZCR)
        {
          SIXSTEP_parameters.BEMF_OK = TRUE;    
          n_zcr_startup = 0;
        }
      }
    }
    SIXSTEP_parameters.prev_step_pos = SIXSTEP_parameters.step_position;     
   
    if(SIXSTEP_parameters.VALIDATION_OK!=0)
    {                
      LF_TIMx.Instance->ARR = (__HAL_TIM_GetCounter(&LF_TIMx)+(ARR_LF>>1));
    }
  }
}
#endif

/**
  * @} 
  */

/** @defgroup MC_SixStep_Hall_Startup_Failure_Handler    MC_SixStep_Hall_Startup_Failure_Handler
  *  @{
    * @brief Handle motor startup failure in case of hall sensors
    * @retval None
*/
#ifdef HALL_SENSORS
void MC_SixStep_Hall_Startup_Failure_Handler(void)
{
  SIXSTEP_parameters.start_attempts--;
  if (SIXSTEP_parameters.start_attempts != 0)
  {
    __disable_irq();
#ifndef VOLTAGE_MODE
    SIXSTEP_parameters.current_reference = SIXSTEP_parameters.startup_reference;
#else
    SIXSTEP_parameters.pulse_value = SIXSTEP_parameters.startup_reference;
#endif
    LF_TIMx.Instance->CCR2 = LF_TIMX_ARR;
    SIXSTEP_parameters.STATUS = STARTUP;
    SIXSTEP_parameters.start_cnt = NUMBER_OF_STEPS;
    SIXSTEP_parameters.hall_ko_successive = 0;
    __enable_irq();
  }
  else
  {
    MC_StopMotor();
    SIXSTEP_parameters.STATUS = STARTUP_FAILURE;
    BSP_BOARD_FAULT_LED_ON();
#ifdef UART_COMM
    char *pCommandString = "STATUS\r\n";
    CMD_Parser(pCommandString);
#endif    
  }
}
#endif
/**
  * @} 
  */

/** @defgroup MC_SixStep_Hall_Run_Failure_Handler    MC_SixStep_Hall_Run_Failure_Handler
  *  @{
    * @brief Handle motor run failure in case of hall sensors
    * @retval None
*/
#ifdef HALL_SENSORS
void MC_SixStep_Hall_Run_Failure_Handler(void)
{
  SIXSTEP_parameters.run_attempts--;
  if (SIXSTEP_parameters.run_attempts != 0)
  {
    __disable_irq();
#ifndef VOLTAGE_MODE
    SIXSTEP_parameters.current_reference = SIXSTEP_parameters.startup_reference;
#else
    SIXSTEP_parameters.pulse_value = SIXSTEP_parameters.startup_reference;
#endif
    LF_TIMx.Instance->CCR2 = LF_TIMX_ARR;
    SIXSTEP_parameters.STATUS = STARTUP;
    SIXSTEP_parameters.start_cnt = NUMBER_OF_STEPS;
    __enable_irq();
  }
  else
  {
    MC_StopMotor();
    SIXSTEP_parameters.STATUS = SPEEDFBKERROR;
    BSP_BOARD_FAULT_LED_ON();
#ifdef UART_COMM
    char *pCommandString = "STATUS\r\n";
    CMD_Parser(pCommandString);
#endif
  }
}
#endif
/**
  * @} 
  */

#ifndef HALL_SENSORS
/** @defgroup MC_ADCx_SixStep_Bemf    MC_ADCx_SixStep_Bemf
  *  @{
    * @brief Compute the zero crossing detection
    * @retval None
*/

void MC_ADCx_SixStep_Bemf()
{
#ifdef PWM_ON_BEMF_SENSING
  uint8_t dir = (__HAL_TIM_DIRECTION_STATUS(&HF_TIMx));
  if(( dir && !zcr_on_ton) || (!dir && zcr_on_ton)) 
  /* DIR bit of TIMxCR1 is 1 when down counting, 0 when up counting  
   * If BEMF sensing during OFF time --> get the ADC value during down counting 
   * If BEMF sensing during ON time --> get the ADC value during up counting
   */
#else
  if(__HAL_TIM_DIRECTION_STATUS(&HF_TIMx))
#endif
  {
    /* GET the ADC value of BEMF sensing */
    SIXSTEP_parameters.ADC_BEMF_Buffer[SIXSTEP_parameters.step_position] = HAL_ADC_GetValue(&ADCx);
#ifdef BEMF_RECORDING
    SIXSTEP_parameters.bemfMeasurements++;
    if (SIXSTEP_parameters.UART_CONTINUOUS_TX_BEMF_ALLOWED != FALSE)
    {      
      bemfArray[SIXSTEP_parameters.bemfIndexRx]=(SIXSTEP_parameters.ADC_BEMF_Buffer[SIXSTEP_parameters.step_position])|(SIXSTEP_parameters.step_position<<12);
      SIXSTEP_parameters.bemfIndexRx++;
      if (SIXSTEP_parameters.bemfIndexRx==(BEMF_ARRAY_SIZE+5))
      {
        bemfArray[3]= SIXSTEP_parameters.bemfMeasurements>>16;
        bemfArray[4]= SIXSTEP_parameters.bemfMeasurements&0xFFFF;
        SIXSTEP_parameters.UART_CONTINUOUS_TX_BEMF_ALLOWED = FALSE;
        UART_Send_Bemf(&bemfArray[0], (BEMF_ARRAY_SIZE+6)<<1);
        SIXSTEP_parameters.bemfIndexRx=5;
      }
    }
#endif
    if(SIXSTEP_parameters.demagn_counter >= SIXSTEP_parameters.demagn_value) 
    {
#ifdef PWN_ON_BEMF_SENSING
      if(PI_parameters.Reference>=0)
      {
        if (((SIXSTEP_parameters.step_position&0x1)==0)&&
            (SIXSTEP_parameters.ADC_BEMF_Buffer[SIXSTEP_parameters.step_position]>SIXSTEP_parameters.ADC_Current_BEMF_thld_UP))
        {
          MC_SixStep_ARR_Bemf(1);
          SIXSTEP_parameters.BEMF_Tdown_count = 0;           
        }
        if (((SIXSTEP_parameters.step_position&0x1)!=0)&&
            (SIXSTEP_parameters.ADC_BEMF_Buffer[SIXSTEP_parameters.step_position]<SIXSTEP_parameters.ADC_Current_BEMF_thld_DOWN))
        {
          MC_SixStep_ARR_Bemf(0);
        }        
      }
      else
      {
        if (((SIXSTEP_parameters.step_position&0x1)!=0)&&
            (SIXSTEP_parameters.ADC_BEMF_Buffer[SIXSTEP_parameters.step_position]>SIXSTEP_parameters.ADC_Current_BEMF_thld_UP))
        {
          MC_SixStep_ARR_Bemf(1);
          SIXSTEP_parameters.BEMF_Tdown_count = 0;           
        }
        if (((SIXSTEP_parameters.step_position&0x1)==0)&&
            (SIXSTEP_parameters.ADC_BEMF_Buffer[SIXSTEP_parameters.step_position]<SIXSTEP_parameters.ADC_Current_BEMF_thld_DOWN))
        {
          MC_SixStep_ARR_Bemf(0);
        }
      }
#else
      if(PI_parameters.Reference>=0)
      {
        if (((SIXSTEP_parameters.step_position&0x1)==0)&&(SIXSTEP_parameters.ADC_BEMF_Buffer[SIXSTEP_parameters.step_position]>SIXSTEP_parameters.ADC_BEMF_threshold_UP))
        {
          MC_SixStep_ARR_Bemf(1);
          SIXSTEP_parameters.BEMF_Tdown_count = 0;           
        }
        if (((SIXSTEP_parameters.step_position&0x1)!=0)&&(SIXSTEP_parameters.ADC_BEMF_Buffer[SIXSTEP_parameters.step_position]<SIXSTEP_parameters.ADC_BEMF_threshold_DOWN))
        {
          MC_SixStep_ARR_Bemf(0);
        }        
      }
      else
      {
        if (((SIXSTEP_parameters.step_position&0x1)!=0)&&(SIXSTEP_parameters.ADC_BEMF_Buffer[SIXSTEP_parameters.step_position]>SIXSTEP_parameters.ADC_BEMF_threshold_UP))
        {
          MC_SixStep_ARR_Bemf(1);
          SIXSTEP_parameters.BEMF_Tdown_count = 0;           
        }
        if (((SIXSTEP_parameters.step_position&0x1)==0)&&(SIXSTEP_parameters.ADC_BEMF_Buffer[SIXSTEP_parameters.step_position]<SIXSTEP_parameters.ADC_BEMF_threshold_DOWN))
        {
          MC_SixStep_ARR_Bemf(0);
        }
      }
#endif
    }
    else SIXSTEP_parameters.demagn_counter++;
#ifdef PWM_ON_BEMF_SENSING
    /******************* SET ADC CHANNEL FOR SPEED/CURRENT/VBUS *******************/
    /* Set the channel for next ADC conversion */   
    MC_Update_ADC_Ch(1);    
    /******************************************************************************/        
#elif POTENTIOMETER
    /******************* SET ADC CHANNEL FOR SPEED/CURRENT/VBUS *******************/
    /* Set the channel for next ADC conversion */   
    MC_SixStep_ADC_Channel(SIXSTEP_parameters.ADC_SEQ_Channel[index_adc_chn]);
    /******************************************************************************/    
#endif
  }  
  else
  {
    /* UP COUNTING, DIR bit of TIMxCR1 is 0 when up counting */
#ifdef POTENTIOMETER
    SIXSTEP_parameters.ADC_SEQ_Buffer[index_adc_chn] = (uint16_t) HAL_ADC_GetValue(&ADCx);
#ifdef CURRENT_SENSE_ADC
    if (index_adc_chn == 0)
#endif
    {
      potentiometer_buffer[potentiometer_buffer_index++]=SIXSTEP_parameters.ADC_SEQ_Buffer[index_adc_chn];  //speed target from potentiometer
      if (potentiometer_buffer_index >= POT_BUFFER_SIZE)
      {
        potentiometer_buffer_index = 0;
      }
    }
#ifdef TEMP_SENSE_ADC
    index_adc_chn++;
    if (index_adc_chn>3) index_adc_chn = 0;
#elif defined(VBUS_SENSE_ADC)
    index_adc_chn++;    
    if (index_adc_chn>2) index_adc_chn = 0;
#elif defined(CURRENT_SENSE_ADC)
    index_adc_chn++;    
    if (index_adc_chn>1) index_adc_chn = 0;
#endif
#endif
    /* Set the channel for next ADC conversion */ 
#ifdef PWM_ON_BEMF_SENSING
    MC_Update_ADC_Ch(0);
#else
    if(SIXSTEP_parameters.STATUS != START && SIXSTEP_parameters.STATUS != ALIGNMENT)
    {
      MC_SixStep_ADC_Channel(SIXSTEP_parameters.Current_ADC_BEMF_channel);
    }
#endif
  }
}
/**
  * @} 
  */

#ifdef PWM_ON_BEMF_SENSING
/** @defgroup MC_Update_ADC_Ch    MC_Update_ADC_Ch
  *  @{
    * @brief Select the next ADC channel to be converted
    * @retval None
*/

void MC_Update_ADC_Ch(uint8_t current_is_BEMF)
{
  if (zcr_on_ton_next != zcr_on_ton)
  {
#if (GPIO_ZCR_MODE!=0)
    if (zcr_on_ton_next != 0)
    {
      HAL_GPIO_WritePin(GPIO_PORT_ZCR_MODE,GPIO_CH_ZCR_MODE,GPIO_PIN_SET);
    }
    else
    {
      HAL_GPIO_WritePin(GPIO_PORT_ZCR_MODE,GPIO_CH_ZCR_MODE,GPIO_PIN_RESET);
    }
#endif
    if (zcr_on_ton_next != 0)
    {
      SIXSTEP_parameters.ADC_Current_BEMF_thld_UP = SIXSTEP_parameters.ADC_BEMF_threshold_UP_ON;
      SIXSTEP_parameters.ADC_Current_BEMF_thld_DOWN = SIXSTEP_parameters.ADC_BEMF_threshold_DOWN_ON;
      
      HAL_GPIO_WritePin(GPIO_PORT_BEMF,GPIO_CH_BEMF,GPIO_PIN_RESET); // Enable divider  
    }
    else
    {
      SIXSTEP_parameters.ADC_Current_BEMF_thld_UP = SIXSTEP_parameters.ADC_BEMF_threshold_UP;
      SIXSTEP_parameters.ADC_Current_BEMF_thld_DOWN = SIXSTEP_parameters.ADC_BEMF_threshold_DOWN;
      
      HAL_GPIO_WritePin(GPIO_PORT_BEMF,GPIO_CH_BEMF,GPIO_PIN_SET); // Disable divider
    }
    // <---    
    zcr_on_ton = zcr_on_ton_next;
    // Switch the conversion sequence BEMF <--> User
    if (current_is_BEMF != 0)
    {
      MC_SixStep_ADC_Channel(SIXSTEP_parameters.Current_ADC_BEMF_channel);
    }
#ifdef POTENTIOMETER    
    else
    {
      MC_SixStep_ADC_Channel(SIXSTEP_parameters.ADC_SEQ_Channel[index_adc_chn]);
    }
#endif
  }
  else
  {
    // Keep the conversion sequence
    if (current_is_BEMF == 0)
    {
      MC_SixStep_ADC_Channel(SIXSTEP_parameters.Current_ADC_BEMF_channel);
    }
#ifdef POTENTIOMETER     
    else
    {
      MC_SixStep_ADC_Channel(SIXSTEP_parameters.ADC_SEQ_Channel[index_adc_chn]);
    }
#endif    
  }
}

/**
  * @} 
  */
#endif
#endif

#ifdef HALL_SENSORS
/** @defgroup MC_ADCx_SixStep_User    MC_ADCx_SixStep_User
  *  @{
    * @brief Get the converted value from the ADC for speed, temperature,
    * current or power supply voltage
    * @retval None
*/

void MC_ADCx_SixStep_User()
{
  {
    /* UP COUNTING, DIR bit of TIMxCR1 is 0 when up counting */
#ifdef POTENTIOMETER
    SIXSTEP_parameters.ADC_SEQ_Buffer[index_adc_chn] = (uint16_t) HAL_ADC_GetValue(&ADCx);
#ifdef CURRENT_SENSE_ADC
    if (index_adc_chn == 0)
#endif
    {
      potentiometer_buffer[potentiometer_buffer_index++]=SIXSTEP_parameters.ADC_SEQ_Buffer[index_adc_chn];  //speed target from potentiometer
      if (potentiometer_buffer_index >= POT_BUFFER_SIZE)
      {
        potentiometer_buffer_index = 0;
      }
    }
#ifdef TEMP_SENSE_ADC
    index_adc_chn++;
    if (index_adc_chn>3) index_adc_chn = 0;
#elif defined(VBUS_SENSE_ADC)
    index_adc_chn++;    
    if (index_adc_chn>2) index_adc_chn = 0;
#elif defined(CURRENT_SENSE_ADC)
    index_adc_chn++;    
    if (index_adc_chn>1) index_adc_chn = 0;
#endif
#endif
  }
}

/**
  * @} 
  */
#endif


/** @defgroup MC_EXT_button_SixStep    MC_EXT_button_SixStep
  *  @{
    * @brief GPIO EXT Callback - Start or Stop the motor through a USER button
    * @retval None
*/

void MC_EXT_button_SixStep()
{
  if(Enable_start_button == TRUE)
  {
    if(SIXSTEP_parameters.RUN_Motor == 0 && SIXSTEP_parameters.Button_ready == TRUE) 
    {    
      MC_StartMotor();   
    }
    else  
    {     
      MC_StopMotor(); 
    }
  }
}

/**
  * @} 
  */

/**
  * @brief This function is called to increment  a global variable "uwTick"
  *        used as application time base.
  * @note In the default implementation, this variable is incremented each 1ms
  *       in Systick ISR.
  * @note This function is declared as __weak to be overwritten in case of other 
  *       implementations in user file.
  * @retval None
  */
void HAL_IncTick(void)
{
  uwTick++;
}

/**
  * @brief  Povides a tick value in millisecond.
  * @note   The function is declared as __Weak  to be overwritten  in case of other 
  *       implementations in user file.
  * @retval tick value
  */
uint32_t HAL_GetTick(void)
{
  return uwTick;
}


/**
  * @}  end MC_6-STEP_LIB 
  */

/**
  * @}  end MIDDLEWARES
  */
