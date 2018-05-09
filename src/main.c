/**
  ******************************************************************************
  * @file    main.c
  * @author  Ac6
  * @version V1.0
  * @date    01-December-2013
  * @brief   Default main function.
  ******************************************************************************
*/
#include "main.h"


int main(void)
{

	MAIN_INIT();
   unsigned char b = 65;
	for(;;)
	{
		/*BSP_LED_On(LED2);
		UTIL_delay(1);
		BSP_LED_Off(LED2);*/
		HAL_UART_Transmit(&huart2,&b,1,2000);
	}
}
