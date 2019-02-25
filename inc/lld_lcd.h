/*
 * lcd.h
 *
 *  Created on: 8 apr. 2018
 *      Author: axint
 */

#ifndef LCD_H_
#define LCD_H_

#include "utilities.h"
#if CFG_LCD_ON
/*
 * DEFINES
 */

#define LCD_NIBBLE_1_PORT  GPIOC
#define LCD_D0             GPIO_PIN_9
#define LCD_D1             GPIO_PIN_8
#define LCD_D2             GPIO_PIN_6
#define LCD_D3             GPIO_PIN_5

#define LCD_NIBBLE_2_PORT  GPIOB
#define LCD_D4             GPIO_PIN_3
#define LCD_D5             GPIO_PIN_5
#define LCD_D6             GPIO_PIN_4
#define LCD_D7             GPIO_PIN_10

#define LCD_CONTROL_PORT   GPIOB
#define LCD_RS             GPIO_PIN_9
#define LCD_EN             GPIO_PIN_8

/*
 *    LCD COMMANDS
 * */

   /*Display Control*/
#define LCD_CLEAR_DISPLAY  0x01
#define LCD_RETURN_HOME    0x02
#define LCD_DISPLAY_ON     0x0C
#define LCD_DISPLAY_OFF    0x08
#define LCD_CURSOR_ON      0x0A
#define LCD_CURSOR_OFF     0x08
#define LCD_BLINKING_ON    0x09
#define LCD_BLINKING_OFF   0x08

   /*Function Set*/
#define LCD_8_BIT_MODE     0x30
#define LCD_4_BIT_MODE     0x20
#define LCD_2_LINES        0x28
#define LCD_1_LINE         0x20
#define LCD_FONT_5x10      0x24
#define LCD_FONT_5x8       0x20

#define LCD_SET_DDRAM      0x80

/*
 * FUNCTIONS
 */

void LCD_INIT();
void LCD_ByteToPort(uint8_t byte_to_send );
void LCD_ToogleEnable();
void LCD_SendCommand(uint8_t command);
void LCD_SendData(uint8_t data);
void LCD_SendString(char* p_str);
void LCD_SetCursorTo(uint8_t row, uint8_t col);
void LCD_SendInt(int32_t in_nr);
void LCD_SendFloat(float in_nr);

#endif /*CFG_LCD_ON*/
#endif /* LCD_H_ */
