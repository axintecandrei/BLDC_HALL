/*
 * mcu_init.c
 *
 *  Created on: 20 mar. 2018
 *      Author: uidp7521
 */
#include "mcu_init.h"

static void RCC_INIT(void);

void MAIN_INIT(void)
{
	RCC_INIT();
}

static void RCC_INIT(void)
{
	/*RCC HAL functions will return a status depending
	  if the configuration was ok or not*/

	/*Declare to be used structures of RCC*/
	RCC_OscInitTypeDef RCC_OscInit;		/*Oscilator type, osc state ON/OFF
	 	 	 	 	 	 	 	 	 	 Depending on the source selected (HSE, HSI or LSI),
	 	 	 	 	 	 	 	 	 	 the osc state will be set only for one type */
	RCC_ClkInitTypeDef RCC_ClockInit;   /*Set clocks that need to be init (SYSCLK, HCLK(for AHB,APB1 and APB2)),
	 	 	 	 	 	 	 	 	 	  and the appropiete PLL dividers*/

	__PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

	RCC_OscInit.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInit.HSEState = RCC_HSE_ON;
	RCC_OscInit.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInit.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInit.PLL.PLLM = 8;
   RCC_OscInit.PLL.PLLN = 336;
   RCC_OscInit.PLL.PLLP = RCC_PLLP_DIV4;
	RCC_OscInit.PLL.PLLQ = 7;

	RCC_ClockInit.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
							  |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
	RCC_ClockInit.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClockInit.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClockInit.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClockInit.APB2CLKDivider = RCC_HCLK_DIV1;


	HAL_RCC_DeInit();
	HAL_RCC_OscConfig(&RCC_OscInit);
	HAL_RCC_ClockConfig(&RCC_ClockInit, FLASH_LATENCY_2);
}

