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


