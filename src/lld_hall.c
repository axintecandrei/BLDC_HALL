/*
 * hall.c
 *
 *  Created on: 2 iun. 2018
 *      Author: AxinteA
 */
#include "lld_hall.h"

void HALL_INIT()
{
	/* local init struct for ports
	 * can be used for any ports
	 * if properly managed*/
	GPIO_InitTypeDef GPIOx_Init;

	GPIOx_Init.Pin   = HALL_1 | HALL_2 ;
	GPIOx_Init.Mode  = GPIO_MODE_INPUT;
	GPIOx_Init.Pull  = GPIO_PULLDOWN;
	GPIOx_Init.Speed = GPIO_SPEED_HIGH;

	/*Enable bus clock for ports*/
	__GPIOA_CLK_ENABLE();
    /*HAL function that will do the proper initialization*/
	HAL_GPIO_Init(HALL_PORT_A, &GPIOx_Init);

	__GPIOB_CLK_ENABLE();
	GPIOx_Init.Pin   = HALL_3 ;
	GPIOx_Init.Mode  = GPIO_MODE_INPUT;
	GPIOx_Init.Pull  = GPIO_PULLDOWN;
	GPIOx_Init.Speed = GPIO_SPEED_HIGH;
	HAL_GPIO_Init(HALL_PORT_B, &GPIOx_Init);
}

void HALL_GET_STATE(uint8_t * hall_state)
{
   uint8_t h1,h2,h3;
   h1 = HAL_GPIO_ReadPin(HALL_PORT_A, HALL_1) ;
   h2 = HAL_GPIO_ReadPin(HALL_PORT_A, HALL_2) ;
   h3 = HAL_GPIO_ReadPin(HALL_PORT_B, HALL_3) ;
   *hall_state = (h1<<2) |(h2<<1) | h3;
}