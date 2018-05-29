/*
 * adc.c
 *
 *  Created on: 20 mai 2018
 *      Author: AxinteA
 */
#include "adc.h"

void ADC_INIT(void)
{
	ADC_Common_TypeDef ADC_CC;
	/*Enable Clock APB2 */
	__ADC1_CLK_ENABLE();
	/*Set prescaler
	 * Fosc 84 MHz
	 * Presaler DIV4
	 * ADCclk 21 MHz*/

	ADC_CC.CCR = 1 << 16;

	/*Configure the PIN in AF*/
	GPIO_InitTypeDef GPIOx_Init;

	GPIOx_Init.Pin   = ADC_IN;
	GPIOx_Init.Mode  = GPIO_MODE_ANALOG;
	GPIOx_Init.Pull  = GPIO_NOPULL;
	HAL_GPIO_Init(ADC_PORT, &GPIOx_Init);


	/*ADC as independent mode on channel ADC_IN*/
	ADC1->CR1   &= ((0x00<<24) | (0x00<<16) | (0x00<<8) | (0x00));
	/*, single conversion*/
	ADC1->CR2   &= ((0x00<<24) | (0x00<<16) | (0x00<<8) | (0x00));
	/*on channel IN3 - PA3*/
	ADC1->SQR1  |= ((0x00<<24) | (0x00<<16) | (0x00<<8) | (0x00));
	ADC1->SQR3  |= ((0x00<<24) | (0x00<<16) | (0x00<<8) | (0x00));
	/*with a sampling rate of 480 cycles*/
	ADC1->SMPR2  |= ((0x00<<24) | (0x00<<16) | (0x00<<8) | (0x02));
	/*Enable ADC*/
	ADC1->CR2 |= 1;
}

uint16_t ADC_GetConv()
{
	/*Start conversion*/
	ADC1->CR2 |= 1 << 30;

	/*Wait till conversion done*/
	while(!((ADC1->SR & 0x02)>>1));

	/*Return the converted data*/
	return ADC1->DR;
}
