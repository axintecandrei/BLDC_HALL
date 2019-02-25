/*
 * adc.c
 *
 *  Created on: 20 mai 2018
 *      Author: AxinteA
 */
#include "../inc/lld_adc.h"
#if CFG_ADC_REG_CONV
static void DMA_INIT(void);
#endif
static void GPIO_INIT(void);


void ADC_INIT(void)
{
	uint8_t counter;
	/*Set prescaler
	 * Fosc 84 MHz
	 * Presaler DIV4
	 * ADCclk 21 MHz*/
	 GPIO_INIT();

	  /* ADC1 clock enable */
	__HAL_RCC_ADC1_CLK_ENABLE();

	ADC1_COMMON->CCR |= ADC_CCR_ADCPRE_0;

#if CFG_ADC_REG_CONV
	ADC1->CR2 |= ADC_CR2_DMA | ADC_CR2_DDS | ADC_CR2_EOCS;
#else
	ADC1->CR2 |= ADC_CR2_JEXTSEL_0 | ADC_CR2_JEXTEN_0;
#endif

	ADC1->CR1   |= ADC_CR1_SCAN	;

#if CFG_ADC_REG_CONV
	ADC1->SQR1  |= ADC_SQR1(ADC_NR_CONV);

	ADC1->SQR2  |= ADC_SQR2_RK(ADC_CHANNEL_V_DcLink,7);

	ADC1->SQR3  |= ADC_SQR3_RK(ADC_CHANNEL_Vc,      6) |
		           ADC_SQR3_RK(ADC_CHANNEL_Vb,      5) |
			       ADC_SQR3_RK(ADC_CHANNEL_Va,      4) |
				   ADC_SQR3_RK(ADC_CHANNEL_Ic,      3) |
				   ADC_SQR3_RK(ADC_CHANNEL_Ib,      2) |
				   ADC_SQR3_RK(ADC_CHANNEL_Ia,      1);


	ADC1->SMPR1  |= ADC_SMPR1(ADC_3_cycles, ADC_CHANNEL_V_DcLink) |
			        ADC_SMPR1(ADC_3_cycles, ADC_CHANNEL_Vc) |
			        ADC_SMPR1(ADC_3_cycles, ADC_CHANNEL_Vb);
#else

	ADC1->JSQR   |= ADC_SQR1(ADC_NR_CONV)               |
			        ADC_SQR3_RK(ADC_CHANNEL_Ia,      1) |
			        ADC_SQR3_RK(ADC_CHANNEL_Ib,      2) |
			        ADC_SQR3_RK(ADC_CHANNEL_Ic,      3) |
					ADC_SQR3_RK(ADC_CHANNEL_V_DcLink,4) ;
#endif



	ADC1->SMPR2  |= ADC_SMPR2(ADC_3_cycles, ADC_CHANNEL_Ia)  |
				    ADC_SMPR2(ADC_3_cycles, ADC_CHANNEL_Ib)  |
				    ADC_SMPR2(ADC_3_cycles, ADC_CHANNEL_Ic)  |
#if CFG_ADC_REG_CONV
					ADC_SMPR2(ADC_3_cycles, ADC_CHANNEL_Va) ;

	DMA_INIT();
#else
	                ADC_SMPR1(ADC_3_cycles,  ADC_CHANNEL_V_DcLink);
#endif


    HAL_NVIC_SetPriority(ADC_IRQn, 1, 1);
    HAL_NVIC_EnableIRQ(ADC_IRQn);

#if !CFG_ADC_REG_CONV
    ADC1->CR1 |= ADC_CR1_JEOCIE;
#endif
	/*Enable ADC*/
	ADC1->CR2 |= ADC_CR2_ADON;

    /* Delay for ADC stabilization time */
    /* Compute number of CPU cycles to wait for */
    counter = (ADC_STAB_DELAY_US * (SystemCoreClock / 1000000U));
    while(counter != 0U)
    {
      counter--;
    }
}
#if CFG_ADC_REG_CONV
static void DMA_INIT(void)
{
	/*Configure DMA controller*/
    __HAL_RCC_DMA2_CLK_ENABLE();
	/*If the stream is enabled, disable it*/
	DMA2_Stream0->CR &= ~DMA_SxCR_EN;
	while((DMA2_Stream0->CR & 0x01));

	DMA2_Stream0->PAR   = (uint32_t)&ADC1->DR;
	DMA2_Stream0->M0AR  = (uint32_t)&ADC_TO_DMA_BUFFER[0];
	DMA2_Stream0->NDTR  = DMA_NDTR;

	DMA2_Stream0->CR   |= DMA_SxCR_PL_0 | DMA_SxCR_PL_1 ;

	DMA2_Stream0->CR   |= DMA_SxCR_CIRC |
						  DMA_SxCR_MINC |
						  DMA_SxCR_PSIZE_0 |
						  DMA_SxCR_MSIZE_0;


	DMA2_Stream0->CR |= DMA_SxCR_EN;
}
#endif

static void GPIO_INIT(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
    /**ADC1 GPIO Configuration
    PC0     ------> ADC1_IN10
    PC1     ------> ADC1_IN11
    PC2     ------> ADC1_IN12
    PA0-WKUP     ------> ADC1_IN0
    PA1     ------> ADC1_IN1
    PA4     ------> ADC1_IN4
    PA6     ------> ADC1_IN6
    */

	__GPIOA_CLK_ENABLE();
	GPIO_InitStruct.Pin  = Ia_Pin |Ib_Pin| Ic_Pin| Va_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(ADC_PORT_A, &GPIO_InitStruct);

	__GPIOC_CLK_ENABLE();
	GPIO_InitStruct.Pin  = Vb_Pin|Vc_Pin|V_DcLink_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(ADC_PORT_C, &GPIO_InitStruct);
}

uint16_t ADC_GetSingleConv()
{
	/*Start conversion*/
	ADC1->CR2 |= 1 << 30;

	/*Wait till conversion done*/
	while(!((ADC1->SR & 0x02)>>1));

	/*Return the converted data*/
	return ADC1->DR;
}

void ADC_StartSeqConv()
{
#if CFG_ADC_REG_CONV

	ADC1->CR2 |=  ADC_CR2_SWSTART;
	while(!((ADC1->SR & ADC_SR_EOC)>>ADC_SR_EOC_Pos));

	ADC_GET_RAW_VAL(Adc_Ia_ch, ADC_Ia_raw);
	ADC_GET_RAW_VAL(Adc_Ib_ch, ADC_Ib_raw);
	ADC_GET_RAW_VAL(Adc_Ic_ch, ADC_Ic_raw);

	ADC_GET_RAW_VAL(Adc_Va_ch, ADC_Va_raw);
	ADC_GET_RAW_VAL(Adc_Ib_ch, ADC_Vb_raw);
	ADC_GET_RAW_VAL(Adc_Vb_ch, ADC_Vc_raw);
	ADC_GET_RAW_VAL(Adc_DC_Link_ch, ADC_V_DcLink_raw);
#else

	ADC1->CR2 |=  ADC_CR2_JSWSTART;
	while(!((ADC1->SR & ADC_SR_JEOC)>>ADC_SR_JEOC_Pos));

	ADC_GET_RAW_VAL(Adc_Ia_ch, ADC_Ia_raw);
	ADC_GET_RAW_VAL(Adc_Ib_ch, ADC_Ib_raw);
	ADC_GET_RAW_VAL(Adc_Ic_ch, ADC_Ic_raw);
	ADC_GET_RAW_VAL(Adc_DC_Link_ch, ADC_V_DcLink_raw);
#endif
}
