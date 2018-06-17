/*
 * lcd.c
 *
 *  Created on: 8 apr. 2018
 *      Author: axint
 */
#include "lld_lcd.h"



void LCD_INIT()
{

   GPIO_InitTypeDef GPIOx_Init;

   GPIOx_Init.Pin   = LCD_D0 | LCD_D1 | LCD_D2 | LCD_D3;
   GPIOx_Init.Mode  = GPIO_MODE_OUTPUT_PP;
   GPIOx_Init.Pull  = GPIO_NOPULL;
   GPIOx_Init.Speed = GPIO_SPEED_MEDIUM;

   /*Enable bus clock for port C*/
   __GPIOC_CLK_ENABLE();
    /*HAL func that will do the proper initialization*/
   HAL_GPIO_Init(LCD_NIBBLE_1_PORT, &GPIOx_Init);

   GPIOx_Init.Pin   = LCD_D4 | LCD_D5 | LCD_D6 | LCD_D7  | LCD_RS | LCD_EN;
   GPIOx_Init.Mode  = GPIO_MODE_OUTPUT_PP;
   GPIOx_Init.Pull  = GPIO_NOPULL;
   GPIOx_Init.Speed = GPIO_SPEED_MEDIUM;

   /*Enable bus clock for ports*/
   __GPIOB_CLK_ENABLE();
    /*HAL func that will do the proper initialization*/
   HAL_GPIO_Init(LCD_NIBBLE_2_PORT, &GPIOx_Init);

   HAL_GPIO_WritePin(LCD_CONTROL_PORT, LCD_RS, GPIO_PIN_RESET);

   UTIL_delay(15);

   LCD_SendCommand(LCD_DISPLAY_ON | LCD_CURSOR_OFF | LCD_BLINKING_OFF);
   LCD_SendCommand(LCD_CLEAR_DISPLAY);
   LCD_SendCommand(LCD_RETURN_HOME);
   LCD_SendCommand(LCD_8_BIT_MODE | LCD_2_LINES | LCD_FONT_5x10);
   LCD_SetCursorTo(0,0);
   LCD_SendString("LCD Init Done");
   LCD_SetCursorTo(1,2);
   LCD_SendString(" HAVE FUN");
   UTIL_delay(5000);
   LCD_SendCommand(LCD_CLEAR_DISPLAY);
   LCD_SendCommand(LCD_RETURN_HOME);
}

void LCD_SendFloat(float in_nr)
{
   char buffer_string[16];

   sprintf(buffer_string,"%.5f",in_nr);
   LCD_SendString(buffer_string);
}

void LCD_SendInt(int32_t in_nr)
{
   char buffer_string[16];

   sprintf(buffer_string,"%ld",in_nr);
   LCD_SendString(buffer_string);
}

void LCD_SendString(char* p_str)
{
   while(*p_str)
   {
      LCD_SendData(*p_str++);
   }
}

void LCD_SendData(uint8_t data)
{
   /*Pull UP RS in order to select Data Register*/
   HAL_GPIO_WritePin(LCD_CONTROL_PORT, LCD_RS, GPIO_PIN_SET);
   /*Send byte of command*/
   LCD_ByteToPort(data);
   /*Set , wait, then reset the EN*/
   LCD_ToogleEnable();
}

void LCD_SetCursorTo(uint8_t row, uint8_t col)
{
   LCD_SendCommand(LCD_SET_DDRAM | (col + (0x40*row)));
}

void LCD_SendCommand(uint8_t command)
{
   /*Pull down RS in order to select Instruction Register*/
   HAL_GPIO_WritePin(LCD_CONTROL_PORT, LCD_RS, GPIO_PIN_RESET);
   /*Send byte of command*/
   LCD_ByteToPort(command);
   /*Set , wait, then reset the EN*/
   LCD_ToogleEnable();

}

void LCD_ToogleEnable()
{
   HAL_GPIO_WritePin(LCD_CONTROL_PORT, LCD_EN, GPIO_PIN_SET);
   UTIL_delay(3);
   HAL_GPIO_WritePin(LCD_CONTROL_PORT, LCD_EN, GPIO_PIN_RESET);
}


void LCD_ByteToPort(uint8_t byte_to_send)
{
   /*Covert the byte to bit then Send them to pin ports*/
   HAL_GPIO_WritePin(LCD_NIBBLE_1_PORT, LCD_D0, (byte_to_send&  1)>>0);
   HAL_GPIO_WritePin(LCD_NIBBLE_1_PORT, LCD_D1, (byte_to_send&  2)>>1);
   HAL_GPIO_WritePin(LCD_NIBBLE_1_PORT, LCD_D2, (byte_to_send&  4)>>2);
   HAL_GPIO_WritePin(LCD_NIBBLE_1_PORT, LCD_D3, (byte_to_send&  8)>>3);
   HAL_GPIO_WritePin(LCD_NIBBLE_2_PORT, LCD_D4, (byte_to_send& 16)>>4);
   HAL_GPIO_WritePin(LCD_NIBBLE_2_PORT, LCD_D5, (byte_to_send& 32)>>5);
   HAL_GPIO_WritePin(LCD_NIBBLE_2_PORT, LCD_D6, (byte_to_send& 64)>>6);
   HAL_GPIO_WritePin(LCD_NIBBLE_2_PORT, LCD_D7, (byte_to_send&128)>>7);
}



