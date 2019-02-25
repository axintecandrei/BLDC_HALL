/**
 ******************************************************************************
 * @file    UART_UI.h
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

#include "6Step_Lib.h"
#include "stdlib.h"
#include "stdio.h"
#include <string.h>

#define TOKEN "\r"
#define CMD_NUM 15

#define COUNTOF(__BUFFER__)   (sizeof(__BUFFER__) / sizeof(*(__BUFFER__)))
#define TXBUFFERSIZE          (COUNTOF(aTxBuffer) - 1)
#define RXBUFFERSIZE  8

typedef struct {
  char name[10];
  void (*pCmdFunc)(void);
} CMD_T;

typedef struct {
  SIXSTEP_Base_SystStatus_t status;
  char statusString[35];
} STATUS_T;

/** @addtogroup MIDDLEWARES     MIDDLEWARES 
  * @brief  Middlewares Layer
  * @{ 
  */


/** @addtogroup UART_UI  UART UI
  * @brief  Serial communication through PC serial terminal
  * @{ 
  */

/** @defgroup Exported_function_Uart  Exported_function_Uart
* @{
*/
/** 
  * @brief  UART function  
  */

void CMD_STARTM( void );
void CMD_STOPMT( void );
void CMD_DIRECTION( void );
void CMD_SETSPD( void );
void CMD_GETSPD( void );
void CMD_STATUS( void );
void CMD_MEASEL( void );
void CMD_HELP(void);
void CMD_INIREF(void);
void CMD_POLESP(void);
void CMD_ACCELE(void);  
void CMD_KP_PRM(void);  
void CMD_KI_PRM(void);
void CMD_PROMPT(void);
void CMD_MEASTA(void);
void CMD_MEASTO(void);
void UART_Set_Value(void);
void UART_Send_Bemf(uint16_t * bemfArray, uint16_t size);
void UART_Send_Speed(void);
void UART_Send_Reply(void);

/**
  * @} 
  */

/**
  * @} 
  */

/**
  * @} 
  */
