/**
 ******************************************************************************
 * @file    UART_UI.c
 * @author  IPC Rennes
 * @version V1.0.0
 * @date    April 24th, 2017
 * @brief   This file provides a set of functions needed to manage the UART com.
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

/* Includes ------------------------------------------------------------------*/
#include "UART_UI.h"

#ifdef UART_COMM  
extern UART_HandleTypeDef huart;
extern SIXSTEP_Base_InitTypeDef SIXSTEP_parameters;      /*!< Main SixStep structure*/ 
extern SIXSTEP_PI_PARAM_InitTypeDef_t PI_parameters;     /*!< SixStep PI regulator structure*/ 
extern void CMD_Parser(char* pCommandString);
extern void MC_Set_PI_param(SIXSTEP_PI_PARAM_InitTypeDef_t *);
extern void MC_StartMotor(void);
extern void MC_StopMotor(void);

DMA_HandleTypeDef txDmaHandle;

static uint16_t Uart_cmd_flag = 0;
const CMD_T CmdTable[] = {
  {"STARTM", CMD_STARTM},
  {"STOPMT", CMD_STOPMT}, 
  {"DIRECT", CMD_DIRECTION},   
  {"SETSPD", CMD_SETSPD}, 
  {"STATUS", CMD_STATUS},     
  {"GETSPD", CMD_GETSPD},    
  {"MEASEL", CMD_MEASEL}, 
  {"INIREF", CMD_INIREF},   
  {"POLESP", CMD_POLESP}, 
  {"ACCELE", CMD_ACCELE},   
  {"KP-PRM", CMD_KP_PRM},  
  {"KI-PRM", CMD_KI_PRM},    
  {"HELP", CMD_HELP},   
  {"\r\n", CMD_PROMPT},
  {"MEASTA", CMD_MEASTA},
  {"MEASTO", CMD_MEASTO},  
  {"", NULL}
};

const STATUS_T StatusTable[] = {
  {IDLE,">>Status: IDLE\r\n >"},
  {STARTUP,">>Status: STARTUP\r\n >"}, 
  {VALIDATION,"Status: VALIDATION\r\n >"},   
  {STOP,">>Status: STOP\r\n >"}, 
  {START,">>Status: START\r\n >"},     
  {RUN,">>Status: RUN\r\n >"},    
  {ALIGNMENT,">>Status: ALIGNMENT\r\n >"}, 
  {SPEEDFBKERROR,">>Status: SPEEDFBKERROR\r\n >"},   
  {OVERCURRENT,">>Status: OVERCURRENT\r\n >"},  
  {STARTUP_FAILURE,">>Status: STARTUP_FAILURE\r\n >"},  
  {STARTUP_BEMF_FAILURE,">>Status: STARTUP_BEMF_FAILURE\r\n >"},
  {LF_TIMER_FAILURE,">>Status: LF_TIMER_FAILURE\r\n >"}
};
uint8_t BUFF_RCV = RXBUFFERSIZE;
uint8_t aRxBuffer[10] = {'0'};     /*!< Buffer used for reception */
uint8_t aTxBuffer[50];
uint8_t row0TxBuffer[] = "\033[2J";  /*!< Buffer used for transmission */
uint8_t row1TxBuffer[] = "*******************\n\r";
uint8_t row2TxBuffer[] = "* STEVAL-SPIN3202 *\n\r";
uint8_t row4TxBuffer[] = "List of commands:\n\r\n\r";
uint8_t row5TxBuffer[] = " <STARTM> Start Motor\n\r";
uint8_t row6TxBuffer[] = " <STOPMT> Stop Motor\n\r";
uint8_t row7aTxBuffer[] = " <DIRECT> Set Motor direction CW or CCW\n\r";
uint8_t row7TxBuffer[] = " <SETSPD> Set Motor Speed\n\r";
uint8_t row7bTxBuffer[] = " <GETSPD> Get Motor Speed\n\r";
uint8_t row7cTxBuffer[] = " <STATUS> Get Status\n\r";
uint8_t row7dTxBuffer[] = " <MEASTA> Measurements transmission start\n\r";
uint8_t row7eTxBuffer[] = " <MEASTO> Measurements transmission stop\n\r";
uint8_t row71TxBuffer[] = " <MEASEL> Measurement type <0>Speed <1>Bemf\n\r";
uint8_t row71bTxBuffer[] = " <HELP> Show help menu\n\r";
uint8_t rowVTxBuffer[] = ">> Insert the value:\n\r";
uint8_t rowVTBxBuffer[] = ">> ENABLE <1> DISABLE <0>:\n\r";
uint8_t rowV1TBxBuffer[] = ">> CW <0> CCW <1>:\n\r";
uint8_t rowMxBuffer[] = ">> START MOTOR COMMAND RECEIVED ! <<\n\r >";
uint8_t rowSxBuffer[] = ">> STOP MOTOR COMMAND RECEIVED ! <<\n\r >";
uint8_t rowETxBuffer[] = ">> ERROR - PLEASE TYPE AGAIN ! <<\n\r >";
uint8_t row8TxBuffer[] = " *** MAIN MOTOR PARAMETERS *****************\n\r";
uint8_t row9TxBuffer[] = " <INIREF> Startup reference (0-4095)\n\r";
uint8_t row10TxBuffer[] = " <ACCELE> Motor acceleration\n\r";
uint8_t row11TxBuffer[] = " <POLESP> Set the Motor pole pairs\n\r";
uint8_t row14TxBuffer[] = " *** PI REGULATOR PARAMETERS - SPEED LOOP **\n\r";
uint8_t row15TxBuffer[] = " <KP-PRM> Set PI proportional term\n\r";
uint8_t row16TxBuffer[] = " <KI-PRM> Set PI integral term\n\r\n\r >";
uint8_t rowLxBuffer[] = ">> OK <<\n\r >";
extern uint8_t UART_FLAG_RECEIVE;

static char strSpeed[9];

/** @addtogroup MIDDLEWARES     MIDDLEWARES 
  * @brief  Middlewares Layer
  * @{ 
  */


/** @addtogroup UART_UI  UART UI
  * @brief  Serial communication through PC serial terminal
  * @{ 
  */


/** @defgroup UART_Communication_Task    UART_Communication_Task
  *  @{
    * @brief UART start receive function
*/
void UART_Communication_Task()
{ 
  if(HAL_UART_Receive_IT(&huart, (uint8_t *)aRxBuffer, 10) != HAL_OK) 
  {
    huart.State = HAL_UART_STATE_READY; 
  }
  else UART_FLAG_RECEIVE = FALSE;
}
/**
  * @} 
  */

/** @defgroup MC_UI_INIT    MC_UI_INIT
  *  @{
    * @brief UART User Interface init
*/
 void MC_UI_INIT()
 {
   HAL_UART_Transmit(&huart, (uint8_t *)row0TxBuffer, (COUNTOF(row0TxBuffer) - 1),5000);
   while (HAL_UART_GetState(&huart) != HAL_UART_STATE_READY)
    {
    } 
   HAL_UART_Transmit(&huart, (uint8_t *)row1TxBuffer, (COUNTOF(row1TxBuffer) - 1),5000);
   while (HAL_UART_GetState(&huart) != HAL_UART_STATE_READY)
    {
    } 
   HAL_UART_Transmit(&huart, (uint8_t *)row2TxBuffer, (COUNTOF(row2TxBuffer) - 1),5000);   
   while (HAL_UART_GetState(&huart) != HAL_UART_STATE_READY)
    {
    }    
   HAL_UART_Transmit(&huart, (uint8_t *)row1TxBuffer, (COUNTOF(row1TxBuffer) - 1),5000);
   while (HAL_UART_GetState(&huart) != HAL_UART_STATE_READY)
    {
    }   
   HAL_UART_Transmit(&huart, (uint8_t *)row4TxBuffer, (COUNTOF(row4TxBuffer) - 1),5000);    
   while (HAL_UART_GetState(&huart) != HAL_UART_STATE_READY)
    {
    }     
   HAL_UART_Transmit(&huart, (uint8_t *)row5TxBuffer, (COUNTOF(row5TxBuffer) - 1),5000);   
   while (HAL_UART_GetState(&huart) != HAL_UART_STATE_READY)
    {
    }   
   HAL_UART_Transmit(&huart, (uint8_t *)row6TxBuffer, (COUNTOF(row6TxBuffer) - 1),5000);     
   while (HAL_UART_GetState(&huart) != HAL_UART_STATE_READY)
    {
    } 
   HAL_UART_Transmit(&huart, (uint8_t *)row7aTxBuffer, (COUNTOF(row7aTxBuffer) - 1),5000);     
   while (HAL_UART_GetState(&huart) != HAL_UART_STATE_READY)
    {
    }   
   HAL_UART_Transmit(&huart, (uint8_t *)row7TxBuffer, (COUNTOF(row7TxBuffer) - 1),5000);     
   while (HAL_UART_GetState(&huart) != HAL_UART_STATE_READY)
    {
    }
   HAL_UART_Transmit(&huart, (uint8_t *)row7bTxBuffer, (COUNTOF(row7bTxBuffer) - 1),5000);     
   while (HAL_UART_GetState(&huart) != HAL_UART_STATE_READY)
    {
    }  
   HAL_UART_Transmit(&huart, (uint8_t *)row7cTxBuffer, (COUNTOF(row7cTxBuffer) - 1),5000);     
   while (HAL_UART_GetState(&huart) != HAL_UART_STATE_READY)
    {
    }    
   HAL_UART_Transmit(&huart, (uint8_t *)row7dTxBuffer, (COUNTOF(row7dTxBuffer) - 1),5000);     
   while (HAL_UART_GetState(&huart) != HAL_UART_STATE_READY)
    {
    }
   HAL_UART_Transmit(&huart, (uint8_t *)row7eTxBuffer, (COUNTOF(row7eTxBuffer) - 1),5000);     
   while (HAL_UART_GetState(&huart) != HAL_UART_STATE_READY)
    {
    }     
   HAL_UART_Transmit(&huart, (uint8_t *)row71TxBuffer, (COUNTOF(row71TxBuffer) - 1),5000);
   while (HAL_UART_GetState(&huart) != HAL_UART_STATE_READY)
    {
    } 
   HAL_UART_Transmit(&huart, (uint8_t *)row71bTxBuffer, (COUNTOF(row71bTxBuffer) - 1),5000);
   while (HAL_UART_GetState(&huart) != HAL_UART_STATE_READY)
    {
    }  
   HAL_UART_Transmit(&huart, (uint8_t *)row8TxBuffer, (COUNTOF(row8TxBuffer) - 1),5000);     
   while (HAL_UART_GetState(&huart) != HAL_UART_STATE_READY)
    {
    }     
   HAL_UART_Transmit(&huart, (uint8_t *)row9TxBuffer, (COUNTOF(row9TxBuffer) - 1),5000);     
   while (HAL_UART_GetState(&huart) != HAL_UART_STATE_READY)
    {
    }       
   HAL_UART_Transmit(&huart, (uint8_t *)row10TxBuffer, (COUNTOF(row10TxBuffer) - 1),5000);     
   while (HAL_UART_GetState(&huart) != HAL_UART_STATE_READY)
    {
    }       
   HAL_UART_Transmit(&huart, (uint8_t *)row11TxBuffer, (COUNTOF(row11TxBuffer) - 1),5000);     
   while (HAL_UART_GetState(&huart) != HAL_UART_STATE_READY)
    {
    }      
   HAL_UART_Transmit(&huart, (uint8_t *)row14TxBuffer, (COUNTOF(row14TxBuffer) - 1),5000);     
   while (HAL_UART_GetState(&huart) != HAL_UART_STATE_READY)
    {
    }
   HAL_UART_Transmit(&huart, (uint8_t *)row15TxBuffer, (COUNTOF(row15TxBuffer) - 1),5000);     
   while (HAL_UART_GetState(&huart) != HAL_UART_STATE_READY)
    {
    }   
   HAL_UART_Transmit(&huart, (uint8_t *)row16TxBuffer, (COUNTOF(row16TxBuffer) - 1),5000);     
   while (HAL_UART_GetState(&huart) != HAL_UART_STATE_READY)
    {
    }  
   UART_Communication_Task();
   SIXSTEP_parameters.Button_ready = TRUE;
 }
/**
  * @} 
  */


/** @defgroup UART_num_decode    UART_num_decode
  *  @{
    * @brief UART Value decoding function
*/
uint32_t UART_num_decode()
{
  static char Value_Buffer[RXBUFFERSIZE];
 
  for(uint8_t i=0;i<BUFF_RCV;i++)
  {
    Value_Buffer[i] = aRxBuffer[i];
  }
  return(atoi(Value_Buffer));
}
/**
  * @} 
  */

/** @defgroup UART_Send_Speed    UART_Send_Speed
  *  @{
    * @brief Transmit Speed 
*/
void UART_Send_Speed()
{ 
  if (huart.State != HAL_UART_STATE_BUSY_TX && huart.State != HAL_UART_STATE_BUSY_TX_RX)
  {  
    SIXSTEP_parameters.UART_CONTINUOUS_TX_SPEED_ALLOWED = FALSE;
    sprintf(strSpeed, "%d\r\n", (int) SIXSTEP_parameters.speed_fdbk_filtered);
    if (HAL_UART_Transmit_DMA(&huart, (uint8_t*) strSpeed, sizeof(strSpeed))!=HAL_OK)
    {
      SIXSTEP_parameters.UART_TX_CANCELLED++;
    }
  }
  else
  {
    SIXSTEP_parameters.UART_TX_CANCELLED++;
  }
}
/**
  * @} 
  */

/** @defgroup UART_Send_Bemf    UART_Send_Bemf
  *  @{
    * @brief Transmit Bemf 
*/
#ifdef BEMF_RECORDING
void UART_Send_Bemf(uint16_t * bemfArray, uint16_t size)
{
  if ((huart.State != HAL_UART_STATE_BUSY_TX) && (huart.State != HAL_UART_STATE_BUSY_TX_RX))
  { 
    if (HAL_UART_Transmit_DMA(&huart, (uint8_t*) bemfArray, size)!=HAL_OK)
    {
      SIXSTEP_parameters.UART_TX_CANCELLED++;
    }
  }
  else
  {
    SIXSTEP_parameters.UART_TX_CANCELLED++;
  }
}
#endif
/**
  * @} 
  */    
    
/** @defgroup UART_Send_Reply    UART_Send_Reply
  *  @{
    * @brief Transmit Reply 
*/
void UART_Send_Reply()
{ 
  SIXSTEP_parameters.UART_TX_REPLY = TRUE;
  HAL_UART_Transmit_DMA(&huart, (uint8_t*) aTxBuffer,sizeof(aTxBuffer));
}
/**
  * @} 
  */    
    
    
/** @defgroup UART_Set_Value    UART_Set_Value
  *  @{
    * @brief UART Main function
*/
void UART_Set_Value()
{ 
  uint8_t uart_continuous_tx_bemf_allowed = SIXSTEP_parameters.UART_CONTINUOUS_TX_BEMF_ALLOWED;
  SIXSTEP_parameters.UART_CONTINUOUS_TX_BEMF_ALLOWED = FALSE;
  if(Get_UART_Data() == '\n') /* UART parity bit must be set to none : PCE = 0 */
  {
    if(Uart_cmd_flag == 0)
    {     
      CMD_Parser((char*)aRxBuffer);
    }
    else 
    {
      SIXSTEP_parameters.Uart_value_to_set = UART_num_decode();
      switch(SIXSTEP_parameters.Uart_cmd_to_set) 
      {
        case SETSPD_CMD:  /*!<  Set the new speed value command received */
          MC_Set_Speed((uint16_t)SIXSTEP_parameters.Uart_value_to_set);
        break;
        case INIREF_CMD:  /*!<  Set the new STARTUP REFERENCE value command received */
          SIXSTEP_parameters.startup_reference = SIXSTEP_parameters.Uart_value_to_set;
        break;
        case POLESP_CMD:  /*!<  Set the Pole Pairs value command received */
          SIXSTEP_parameters.NUMPOLESPAIRS = SIXSTEP_parameters.Uart_value_to_set;
        break;
        case ACCELE_CMD:  /*!<  Set the Accelleration of the motor command received */
          SIXSTEP_parameters.ACCEL = SIXSTEP_parameters.Uart_value_to_set;
        break;
        case DIRECT_CMD:  /*!<  Set the motor direction */
          SIXSTEP_parameters.CW_CCW = SIXSTEP_parameters.Uart_value_to_set;
          MC_Set_PI_param(&PI_parameters);
        break;
        case KP_PRM_CMD:  /*!<  Set the KP PI param command received */
          PI_parameters.Kp_Gain = SIXSTEP_parameters.Uart_value_to_set;
        break;
        case KI_PRM_CMD:  /*!<  Set the KI PI param command received */
          PI_parameters.Ki_Gain = SIXSTEP_parameters.Uart_value_to_set;
        break;
        case MEASEL_CMD:  /*!<  Set the continuous measurement to be performed */
          SIXSTEP_parameters.UART_MEASUREMENT_TYPE = SIXSTEP_parameters.Uart_value_to_set;
        break;
      }  /* switch case */
      Uart_cmd_flag = 0;        
      if (huart.State != HAL_UART_STATE_BUSY_TX && huart.State != HAL_UART_STATE_BUSY_TX_RX)
      {
        SIXSTEP_parameters.UART_TX_REPLY = TRUE;
        HAL_UART_Transmit_DMA(&huart,(uint8_t *)rowLxBuffer,(COUNTOF(rowLxBuffer) - 1));
      }
      else
      {
        SIXSTEP_parameters.UART_TX_DIFFERED_REPLY = TRUE;
        strcpy((char *)aTxBuffer,(const char *)rowLxBuffer);
      }
      UART_FLAG_RECEIVE = TRUE;
    }  /* else */
    SIXSTEP_parameters.UART_CONTINUOUS_TX_BEMF_ALLOWED = uart_continuous_tx_bemf_allowed;
  }  /* if */
}
/**
  * @} 
  */

    
/** @defgroup CMD_MSG    CMD_MSG
  *  @{
    * @brief UART Transmit standard message
*/
void CMD_MSG()
{
  if (huart.State != HAL_UART_STATE_BUSY_TX && huart.State != HAL_UART_STATE_BUSY_TX_RX)
  {
    SIXSTEP_parameters.UART_TX_REPLY = TRUE;
    HAL_UART_Transmit_DMA(&huart,(uint8_t *)rowVTxBuffer,(COUNTOF(rowVTxBuffer) - 1));
  }
  else
  {
    SIXSTEP_parameters.UART_TX_DIFFERED_REPLY = TRUE;
    strcpy((char *)aTxBuffer,(const char *)rowVTxBuffer);
  }   
  Uart_cmd_flag = 1;
}
/**
  * @} 
  */

/** @defgroup CMD_MSG_BOOL    CMD_MSG_BOOL
  *  @{
    * @brief UART Transmit standard message for input data
*/
void CMD_MSG_BOOL()
{
  if (huart.State != HAL_UART_STATE_BUSY_TX && huart.State != HAL_UART_STATE_BUSY_TX_RX)
  {
    SIXSTEP_parameters.UART_TX_REPLY = TRUE;
    HAL_UART_Transmit_DMA(&huart,(uint8_t *)rowVTBxBuffer,(COUNTOF(rowVTBxBuffer) - 1));
  }
  else
  {
    SIXSTEP_parameters.UART_TX_DIFFERED_REPLY = TRUE;
    strcpy((char *)aTxBuffer,(const char *)rowVTBxBuffer);
  }   
  Uart_cmd_flag = 1;
}
/**
  * @} 
  */

/** @defgroup CMD_MSG_BOOL_DIRECT    CMD_MSG_BOOL_DIRECT
  *  @{
    * @brief UART Transmit CW or CCW message for input data
*/  
void CMD_MSG_BOOL_DIRECT()
{
  if (huart.State != HAL_UART_STATE_BUSY_TX && huart.State != HAL_UART_STATE_BUSY_TX_RX)
  {
    SIXSTEP_parameters.UART_TX_REPLY = TRUE;
    HAL_UART_Transmit_DMA(&huart,(uint8_t *)rowV1TBxBuffer,(COUNTOF(rowV1TBxBuffer) - 1));
  }
  else
  {
    SIXSTEP_parameters.UART_TX_DIFFERED_REPLY = TRUE;
    strcpy((char *)aTxBuffer,(const char *)rowV1TBxBuffer);
  }   
  Uart_cmd_flag = 1;     
}
/**
  * @} 
  */

/** @defgroup CMD_STARTM    CMD_STARTM
  *  @{
    * @brief UART Transmit Start motor message  
*/ 
void CMD_STARTM()
{     
  /*!<  Start Motor command received */
  MC_StartMotor();
  if (huart.State != HAL_UART_STATE_BUSY_TX && huart.State != HAL_UART_STATE_BUSY_TX_RX)
  {
    SIXSTEP_parameters.UART_TX_REPLY = TRUE;
    HAL_UART_Transmit_DMA(&huart, (uint8_t *)rowMxBuffer, (COUNTOF(rowMxBuffer) - 1));
  }
  else
  {
    SIXSTEP_parameters.UART_TX_DIFFERED_REPLY = TRUE;
    strcpy((char *)aTxBuffer,(const char *)rowMxBuffer);
  }
}
/**
  * @} 
  */

/** @defgroup CMD_STOPMT    CMD_STOPMT
  *  @{
    * @brief UART Transmit Stop motor message  
*/
void CMD_STOPMT()
{      
  /*!<  Stop Motor command received */
  MC_StopMotor();     
  if (huart.State != HAL_UART_STATE_BUSY_TX && huart.State != HAL_UART_STATE_BUSY_TX_RX)
  {
    SIXSTEP_parameters.UART_TX_REPLY = TRUE;
    HAL_UART_Transmit_DMA(&huart, (uint8_t *)rowSxBuffer, (COUNTOF(rowSxBuffer) - 1));
  }
  else
  {
    SIXSTEP_parameters.UART_TX_DIFFERED_REPLY = TRUE;
    strcpy((char *)aTxBuffer,(const char *)rowSxBuffer);
  }
}
/**
  * @} 
  */

/** @defgroup CMD_SETSPD    CMD_SETSPD
  *  @{
    * @brief UART Change the motor speed  
*/ 
void CMD_SETSPD()
{    
 /*!<  Set the new speed value command received */
        CMD_MSG();
        SIXSTEP_parameters.Uart_cmd_to_set = SETSPD_CMD;
}
/**
  * @} 
  */

/** @defgroup CMD_GETSPD    CMD_GETSPD
  *  @{
    * @brief UART Get the motor speed  
*/
void CMD_GETSPD()
{    
  /*!<  Get Mechanical Motor Speed command received */  
  sprintf((char *)aTxBuffer, ">>Motor Speed: %d RPM\r\n >", (int) SIXSTEP_parameters.speed_fdbk_filtered);
  if (huart.State != HAL_UART_STATE_BUSY_TX && huart.State != HAL_UART_STATE_BUSY_TX_RX)
  {
    SIXSTEP_parameters.UART_TX_REPLY = TRUE;
    HAL_UART_Transmit_DMA(&huart, aTxBuffer, (COUNTOF(aTxBuffer) - 1));
  }
  else
  {
    SIXSTEP_parameters.UART_TX_DIFFERED_REPLY = TRUE;
  }      
}
/**
  * @} 
  */

/** @defgroup CMD_HELP    CMD_HELP
  *  @{
    * @brief UART Help command
*/ 
void CMD_HELP()
{
 /*!<  Help command received */
      huart.State = HAL_UART_STATE_READY; 
      MC_UI_INIT();       
      UART_FLAG_RECEIVE = TRUE; 
}  
/**
  * @} 
  */


/** @defgroup CMD_INIREF    CMD_INIREF
  *  @{
    * @brief UART Set the current reference
*/
void CMD_INIREF()
{
 /*!<  Set the new STARUP_CURRENT_REFERENCE value command received */
      CMD_MSG();
      SIXSTEP_parameters.Uart_cmd_to_set = INIREF_CMD;
}
/**
  * @} 
  */


/** @defgroup CMD_POLESP    CMD_POLESP
  *  @{
    * @brief UART Set the motor poles pairs
*/
void CMD_POLESP()
{
 /*!<  Set the Pole Pairs value command received */
      CMD_MSG();
      SIXSTEP_parameters.Uart_cmd_to_set = POLESP_CMD;      
}
/**
  * @} 
  */

/** @defgroup CMD_ACCELE    CMD_ACCELE
  *  @{
    * @brief UART Set the accelleration of the motor at start-up
*/
void CMD_ACCELE()
{
 /*!<  Set the Accelleration for Start-up of the motor command received */  
      CMD_MSG();
      SIXSTEP_parameters.Uart_cmd_to_set = ACCELE_CMD;  
}
/**
  * @} 
  */

/** @defgroup CMD_DIRECTION   CMD_DIRECTION
  *  @{
    * @brief UART Set the motor direction
*/
void CMD_DIRECTION()
{ 
 /*!<  Enable the DEMAG dynamic control command received */
      CMD_MSG_BOOL_DIRECT();
      SIXSTEP_parameters.Uart_cmd_to_set = DIRECT_CMD;        
}
/**
  * @} 
  */

/** @defgroup CMD_KP_PRM    CMD_KP_PRM
  *  @{
    * @brief UART Set the  KP PI param
*/
void CMD_KP_PRM()
{ 
  /*!<  Set the KP PI param command received */  
      CMD_MSG();
      SIXSTEP_parameters.Uart_cmd_to_set = KP_PRM_CMD;     
}
/**
  * @} 
  */

/** @defgroup CMD_KI_PRM    CMD_KI_PRM
  *  @{
    * @brief UART Set the KI PI param
*/
void CMD_KI_PRM()
{ 
 /*!<  Set the KI PI param command received */    
      CMD_MSG();
      SIXSTEP_parameters.Uart_cmd_to_set = KI_PRM_CMD;     
}
/**
  * @} 
  */

/** @defgroup CMD_STATUS    CMD_STATUS
  *  @{
    * @brief UART View the STATUS
*/
void CMD_STATUS()
{
  /*!<  Get Motor Status */  
  if (huart.State != HAL_UART_STATE_BUSY_TX && huart.State != HAL_UART_STATE_BUSY_TX_RX)
  {
    SIXSTEP_parameters.UART_TX_REPLY = TRUE;
    HAL_UART_Transmit_DMA(&huart, (uint8_t*) StatusTable[SIXSTEP_parameters.STATUS].statusString, sizeof(StatusTable[SIXSTEP_parameters.STATUS].statusString));
  }
  else
  {
    SIXSTEP_parameters.UART_TX_DIFFERED_REPLY = TRUE;
    strcpy((char *)aTxBuffer,(const char *) StatusTable[SIXSTEP_parameters.STATUS].statusString);
  }       
}
/**
  * @} 
  */

/** @defgroup CMD_PROMPT    CMD_PROMPT
  *  @{
    * @brief UART get prompt 
*/
void CMD_PROMPT()
{ 
  static char strLineMessage[2]=" >";
  if (huart.State != HAL_UART_STATE_BUSY_TX && huart.State != HAL_UART_STATE_BUSY_TX_RX)
  {
    SIXSTEP_parameters.UART_TX_REPLY = TRUE;
    HAL_UART_Transmit_DMA(&huart, (uint8_t*) strLineMessage,2);
  }
  else
  {
    SIXSTEP_parameters.UART_TX_DIFFERED_REPLY = TRUE;
    strcpy((char *)aTxBuffer,(const char *)strLineMessage);
  }
}
/**
  * @} 
  */

/** @defgroup CMD_MEASEL    CMD_MEASEL
  *  @{
    * @brief UART Set the continuous measurement to be performed  
*/ 
void CMD_MEASEL()
{    
 /*!< Set the continuous measurement to be performed */
  CMD_MSG();
  SIXSTEP_parameters.Uart_cmd_to_set = MEASEL_CMD;
}
/**
  * @} 
  */  


/** @defgroup CMD_MEASTA    CMD_MEASTA
  *  @{
    * @brief UART start the measurements (speed, bemf, setup time ...) transmission
*/
void CMD_MEASTA()
{
  CMD_PROMPT();
  if (SIXSTEP_parameters.UART_MEASUREMENT_TYPE == 0)
  {
    SIXSTEP_parameters.UART_CONTINUOUS_TX_SPEED_MODE = TRUE;
    SIXSTEP_parameters.UART_CONTINUOUS_TX_SPEED_ALLOWED = TRUE;
  }
  else if (SIXSTEP_parameters.UART_MEASUREMENT_TYPE == 1)
  {
    SIXSTEP_parameters.UART_CONTINUOUS_TX_BEMF_MODE = TRUE;
    SIXSTEP_parameters.UART_CONTINUOUS_TX_BEMF_ALLOWED = TRUE;
  }
  SIXSTEP_parameters.UART_TX_CANCELLED = 0;
}
/**
  * @} 
  */

/** @defgroup CMD_MEASTO    CMD_MEASTO
  *  @{
    * @brief UART stop the measurements (speed, bemf, setup time ...) transmission
*/
void CMD_MEASTO()
{
  static char strTxCancelled[24];
  sprintf(strTxCancelled, ">>Tx cancelled: %d\r\n", SIXSTEP_parameters.UART_TX_CANCELLED);
  if (huart.State != HAL_UART_STATE_BUSY_TX && huart.State != HAL_UART_STATE_BUSY_TX_RX)
  {
    SIXSTEP_parameters.UART_TX_REPLY = TRUE;
    HAL_UART_Transmit_DMA(&huart, (uint8_t*) strTxCancelled,sizeof(strTxCancelled));       
  }
  else
  {
    SIXSTEP_parameters.UART_TX_DIFFERED_REPLY = TRUE;
    strcpy((char *)aTxBuffer,(const char *) strTxCancelled);
  }
  SIXSTEP_parameters.UART_CONTINUOUS_TX_SPEED_MODE = FALSE;
  SIXSTEP_parameters.UART_CONTINUOUS_TX_BEMF_MODE = FALSE;
}
/**
  * @} 
  */

/** @defgroup CMD_Parser    CMD_Parser
  *  @{
    * @brief UART String parser 
*/
void CMD_Parser(char* pCommandString)
{
    static uint8_t CmdListIndex;
    char sCmd[16];
    
    if (strncmp(pCommandString,"\r\n",2)==0) strcpy(sCmd,"\r\n");
    else strcpy(sCmd, pCommandString);
    strtok (sCmd, TOKEN);
    /* Command Callback identification */
    for(CmdListIndex=0;CmdTable[CmdListIndex].pCmdFunc!=NULL; CmdListIndex++) {
        if (strcmp(CmdTable[CmdListIndex].name, sCmd) == 0 ){
          break;
        }
    }
    if ( CmdListIndex <= CMD_NUM )
    {
        // CMD OK --> extract parameters
        /* Check for valid callback */
        if(CmdTable[CmdListIndex].pCmdFunc!=NULL) {
            CmdTable[CmdListIndex].pCmdFunc();
        }
    }
    else
    {
      if (huart.State != HAL_UART_STATE_BUSY_TX && huart.State != HAL_UART_STATE_BUSY_TX_RX)
      {
        SIXSTEP_parameters.UART_TX_REPLY = TRUE;
        HAL_UART_Transmit_DMA(&huart, (uint8_t*) rowETxBuffer,sizeof(rowETxBuffer));       
      }
      else
      {
        SIXSTEP_parameters.UART_TX_DIFFERED_REPLY = TRUE;
        strcpy((char *)aTxBuffer,(const char *) rowETxBuffer);
      }         
      Uart_cmd_flag = 0;
    }
    UART_FLAG_RECEIVE = TRUE;
}

/**
  * @} 
  */


/**
  * @}  end UART_UI 
  */

/**
  * @}  end MIDDLEWARES
  */
#endif
