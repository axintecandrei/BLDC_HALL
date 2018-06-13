/*
 * hall.c
 *
 *  Created on: 2 iun. 2018
 *      Author: AxinteA
 */
#include "hall.h"

void HALL_INIT()
{
	/* local init struct for ports
	 * can be used for any ports
	 * if properly managed*/
	GPIO_InitTypeDef GPIOx_Init;

	GPIOx_Init.Pin   = HALL_1 | HALL_2 | HALL_3;
	GPIOx_Init.Mode  = GPIO_MODE_INPUT;
	GPIOx_Init.Pull  = GPIO_PULLDOWN;
	GPIOx_Init.Speed = GPIO_SPEED_MEDIUM;

	/*Enable bus clock for ports*/
	__GPIOA_CLK_ENABLE();
    /*HAL function that will do the proper initialization*/
	HAL_GPIO_Init(HALL_PORT, &GPIOx_Init);
}

void HALL_GET_STATE(uint8_t * h1,uint8_t * h2, uint8_t * h3)
{
   *h1 = HAL_GPIO_ReadPin(HALL_PORT, HALL_1) ;
   *h2 = HAL_GPIO_ReadPin(HALL_PORT, HALL_2) ;
   *h3 = HAL_GPIO_ReadPin(HALL_PORT, HALL_3) ;
}
