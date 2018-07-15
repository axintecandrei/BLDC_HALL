/*
 * adc.c
 *
 *  Created on: 20 mai 2018
 *      Author: AxinteA
 */
#include "lld_adc.h"

void ADC_INIT(void)
{
	ADC_Common_TypeDef ADC_CC;
	/*Set prescaler
	 * Fosc 84 MHz
	 * Presaler DIV4
	 * ADCclk 21 MHz*/

	GPIO_InitTypeDef GPIO_InitStruct;
	  /* ADC1 clock enable */
	__HAL_RCC_ADC1_CLK_ENABLE();

	/**ADC1 GPIO Configuration
	PA0-WKUP     ------> ADC1_IN0
    PA1     ------> ADC1_IN1
	PA4     ------> ADC1_IN4
	PA6     ------> ADC1_IN6
	    */
	GPIO_InitStruct.Pin  = Ia_Pin|Ib_Pin|Va_Pin|Vb_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(ADC_PORT_A, &GPIO_InitStruct);

	GPIO_InitStruct.Pin  = Vc_Pin|DC_Link_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(ADC_PORT_C, &GPIO_InitStruct);

	/*Configure PC11 as EXTI 11*/
    GPIO_InitStruct.Pin  = Trig_ADC_Pin;
    GPIO_InitStruct.Mode =  GPIO_MODE_IT_RISING;//GPIO_MODE_IT_RISING_FALLING;//
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(ADC_PORT_C, &GPIO_InitStruct);

	ADC_CC.CCR = 1 << 16;

	/*ADC sw triggered */
	ADC1->CR1   = ((0x00<<24) | (0x00<<16) | (0x00<<8) | (0x00));
	/*, sequence conversion*/
	ADC1->CR2   = ((0x00<<24) | (0x00<<16) | (0xC1<<8) | (0x00));

	/*on channels IN0,1,4,6,10,11 - PA0,1,4,6,PC0,1*/
	ADC1->SQR3  = ADC_SQR3_RK(11,6)| ADC_SQR3_RK(10,5)|ADC_SQR3_RK(6,4)| ADC_SQR3_RK(4 ,3) | ADC_SQR3_RK(1,2)| ADC_SQR3_RK(0,1);
	ADC1->SQR1  = ADC_SQR1(6);

	/*with a sampling rate of 3 cycles*/
	ADC1->SMPR2  = ADC_SMPR2(0,  6) | ADC_SMPR2(0, 4) | ADC_SMPR2(0, 1) | ADC_SMPR2(0, 0);
	ADC1->SMPR1  = ADC_SMPR1(0, 11) | ADC_SMPR1(0, 10);

	/*Configure DMA controller*/
	__HAL_RCC_DMA2_CLK_ENABLE();

	DMA2_Stream0->CR    = ((0x00<<24) | (0x02<<16) | (0x2C<<8) | (0x00)) ;
	DMA2_Stream0->NDTR  = ADC_BUFFER_SIZE;
	DMA2_Stream0->PAR   = (uint32_t)&ADC1->DR;
	DMA2_Stream0->M0AR  = (uint32_t)&ADC_TO_DMA_BUFFER[0];

	DMA2_Stream0->CR |= 0;

	/*Enable ADC*/
	ADC1->CR2 |= 0;
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

void ADC_GetSeqConv()
{
	/*Start conversion*/
	ADC1->CR2 |= 1 << 30;

	/*Wait till conversion done*/
	//while(!((ADC1->SR & 0x02)>>1));
	//while(DMA2_Stream0->NDTR==0);
}
