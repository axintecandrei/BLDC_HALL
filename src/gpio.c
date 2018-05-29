/*
 * gpio.c
 *
 *  Created on: 20 mar. 2018
 *      Author: uidp7521
 *
 * GPIO registers Description:
 * Config:
 * 		- GPIOx_MODER   register is used to select the I/O direction (input, output, AF, analog)
 * 			00: Input
 * 			01: General purpose output mode
 * 		    10: Alternate function mode
 * 		    11: Analog mode
 * 		- GPIOx_OTYPER  register is used to select the output type (push-pull or open-drain)
 *			 0: Output push-pull
 *			 1: Output open-drain
 * 		- GPIOx_OSPEEDR register is used to select speed (the I/O speed pins are directly connected to the
 *                      corresponding GPIOx_OSPEEDR register bits whatever the I/O direction)
 * 		- GPIOx_PUPDR   register is used to select the pull-up/pull-down whatever the I/O direction
 * 			00: No pull-up, pull-down
 * 			01: Pull-up
 * 			10: Pull-down
 * 			11: Reserved
 * Data:
 * 		- GPIOx_ODR     stores the data to be output, it is read/write accessible
 * 			- can be used to toggle the pin
 * 		- GPIOx_IDR     stores the data into this register
 *
 *      - GPIOx_BSRR
 *      	- pin_nr
 *      		 0 No action on the corresponding ODRx
 *      		 1: Resets the corresponding ODRx bit
 *      	- pin_nr<<16
 *      		 0: No action on the corresponding ODRx bit
 *      		 1: Sets the corresponding ODRx bit
 * */

#include "gpio.h"


void GPIO_INIT()
{
	uint8_t button_nr;

	/* local init struct for ports
	 * can be used for any ports
	 * if properly managed*/
	GPIO_InitTypeDef GPIOx_Init;

	GPIOx_Init.Pin   = D0 | D1 | D2 | D3;
	GPIOx_Init.Mode  = GPIO_MODE_OUTPUT_PP;
	GPIOx_Init.Pull  = GPIO_NOPULL;
	GPIOx_Init.Speed = GPIO_SPEED_MEDIUM;

	/*Enable bus clock for ports*/
	__GPIOC_CLK_ENABLE();
    /*HAL func that will do the proper initialisation*/
	HAL_GPIO_Init(NIBBLE_1_PORT, &GPIOx_Init);

	GPIOx_Init.Pin   = D4 | D5 | D6 | D7 ;
	GPIOx_Init.Mode  = GPIO_MODE_OUTPUT_PP;
	GPIOx_Init.Pull  = GPIO_NOPULL;
	GPIOx_Init.Speed = GPIO_SPEED_MEDIUM;

	/*Enable bus clock for ports*/
	__GPIOA_CLK_ENABLE();
    /*HAL func that will do the proper initialisation*/
	HAL_GPIO_Init(NIBBLE_2_PORT, &GPIOx_Init);
	GPIOx_Init.Pin   = BUTT_1 | BUTT_2 ;
	GPIOx_Init.Mode  = GPIO_MODE_INPUT;
	GPIOx_Init.Pull  = GPIO_PULLDOWN;
	GPIOx_Init.Speed = GPIO_SPEED_MEDIUM;

    /*HAL func that will do the proper initialization*/
	HAL_GPIO_Init(BUTT_PORT, &GPIOx_Init);

	/*init the debounce structure */
	for(button_nr = 0; button_nr < 2; button_nr++)
	{
		DEBOUNCE_BUTTx[button_nr].button_state                  = released;
		DEBOUNCE_BUTTx[button_nr].ButtonPressedConfidanceLevel  = 0;
		DEBOUNCE_BUTTx[button_nr].ButtonReleasedConfidanceLevel = 0;
	}
}

uint8_t GPIO_DebounceButton(GPIO_TypeDef* port, uint16_t button, debounce_state_t* port_debounce_str)
{
   /*check if the button is pressed*/
   if((pin_state_t)HAL_GPIO_ReadPin(port,button) == pressed)
   {

      if(port_debounce_str->button_state == released)
      {
         /*once the confidence level is fullfield*/
	      if(port_debounce_str->ButtonPressedConfidanceLevel > CONFIDENCE_THR)
	      {
	         /*update the buttons state with pressed*/
	         port_debounce_str->button_state = pressed;
	      }
	      else
	      {
	         port_debounce_str->ButtonPressedConfidanceLevel++;
	         port_debounce_str->ButtonReleasedConfidanceLevel = 0;
	      }
	   }
	}
	else
	{
      if(port_debounce_str->button_state == pressed)
      {
         /*once the confidence level is fullfield*/
         if(port_debounce_str->ButtonReleasedConfidanceLevel > CONFIDENCE_THR)
         {
            /*update the buttons state with pressed*/
            port_debounce_str->button_state = released;
         }
         else
         {

            port_debounce_str->ButtonReleasedConfidanceLevel++;
            port_debounce_str->ButtonPressedConfidanceLevel  = 0;
         }
      }
	}


	return port_debounce_str->button_state;
}

void GPIO_ByteOnPins(uint8_t byte)
{
	byteAsbits_t out_bits;

	UTIL_ByteToBits(byte, &out_bits);

	HAL_GPIO_WritePin(NIBBLE_1_PORT,D0,out_bits.bit_0);
	HAL_GPIO_WritePin(NIBBLE_1_PORT,D1,out_bits.bit_1);
	HAL_GPIO_WritePin(NIBBLE_1_PORT,D2,out_bits.bit_2);
	HAL_GPIO_WritePin(NIBBLE_1_PORT,D3,out_bits.bit_3);
	HAL_GPIO_WritePin(NIBBLE_2_PORT,D4,out_bits.bit_4);
	HAL_GPIO_WritePin(NIBBLE_2_PORT,D5,out_bits.bit_5);
	HAL_GPIO_WritePin(NIBBLE_2_PORT,D6,out_bits.bit_6);
	HAL_GPIO_WritePin(NIBBLE_2_PORT,D7,out_bits.bit_7);
}


