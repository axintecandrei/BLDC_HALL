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

	GPIOx_Init.Pin   = HALL_A | HALL_B |HALL_C;
	GPIOx_Init.Mode  = GPIO_MODE_INPUT;
	GPIOx_Init.Pull  = GPIO_PULLDOWN;
	GPIOx_Init.Speed = GPIO_SPEED_HIGH;

	/*Enable bus clock for ports*/
	__GPIOA_CLK_ENABLE();
    /*HAL function that will do the proper initialization*/
	HAL_GPIO_Init(HALL_PORT, &GPIOx_Init);

	Set_Hall_State(0);
}

void HALL_GET_STATE()
{
	Set_Hall_State((HAL_GPIO_ReadPin(HALL_PORT, HALL_A)<<2) |
		           (HAL_GPIO_ReadPin(HALL_PORT, HALL_B)<<1) |
				    HAL_GPIO_ReadPin(HALL_PORT, HALL_C));
}
