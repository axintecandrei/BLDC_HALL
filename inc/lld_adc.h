#ifndef ADC_H_
#define ADC_H_

#include "../CMSIS/device/stm32f4xx.h"
#include "../inc/lld_adc_if.h"
#include "../inc/utilities.h"

/*
 * DEFINES
 */


#define ADC_PORT_A      GPIOA
#define Ia_Pin          GPIO_PIN_0
#define Ib_Pin          GPIO_PIN_1
#define Ic_Pin          GPIO_PIN_4
#define Va_Pin          GPIO_PIN_6

#define ADC_PORT_C      GPIOC
#define Vb_Pin          GPIO_PIN_0
#define Vc_Pin          GPIO_PIN_1
#define V_DcLink_Pin    GPIO_PIN_2

#define ADC_CHANNEL_Ia       ADC_CHANNEL_0
#define ADC_CHANNEL_Ib       ADC_CHANNEL_1
#define ADC_CHANNEL_Ic       ADC_CHANNEL_4
#define ADC_CHANNEL_Va       ADC_CHANNEL_6
#define ADC_CHANNEL_Vb       ADC_CHANNEL_10
#define ADC_CHANNEL_Vc       ADC_CHANNEL_11
#define ADC_CHANNEL_V_DcLink ADC_CHANNEL_12

#define ADC_480_cycles  (7)
#define ADC_144_cycles  (6)
#define ADC_84_cycles   (4)
#define ADC_3_cycles    (0)

#if CFG_ADC_REG_CONV
#define ADC_NR_CONV     (7)
#define DMA_NDTR        (ADC_NR_CONV)
#else
#define ADC_NR_CONV     (4)
#endif

/*Buffer mapping*/
#define ADC_BUFFER_SIZE       ADC_NR_CONV
#define Adc_Ia_ch             0
#define Adc_Ib_ch             1
#define Adc_Ic_ch             2
#if CFG_ADC_REG_CONV
#define Adc_Va_ch             3
#define Adc_Vb_ch             4
#define Adc_Vc_ch             5
#define Adc_DC_Link_ch        6
#else
#define Adc_DC_Link_ch        3
#endif


#if CFG_ADC_REG_CONV
/*Macro used to fetch the conversion result based in channel*/
#define ADC_GET_RAW_VAL(channel, output)  do{\
                                               output = ADC_TO_DMA_BUFFER[channel]; \
                                             }while(0)
#else
#define ADC_GET_RAW_VAL(channel, output) do{\
											 __IO uint32_t * tmpJDR = &ADC1->JDR1;\
											 output = 	*(tmpJDR + channel);\
											}while(0)
#endif

#define ADC_CLEAR_IT()                      do{ADC1->SR &= ~(ADC_SR_JSTRT | ADC_SR_JEOC);}while(0)

/*
 * TYPE-DEFs
 */


/*
 * VARIABLES
 */
#if CFG_ADC_REG_CONV
__IO uint16_t ADC_TO_DMA_BUFFER[ADC_BUFFER_SIZE];
#endif

uint16_t ADC_Ia_raw;
uint16_t ADC_Ib_raw;
uint16_t ADC_Ic_raw;
uint16_t ADC_Va_raw;
uint16_t ADC_Vb_raw;
uint16_t ADC_Vc_raw;
uint16_t ADC_V_DcLink_raw;

/*
 * FUNCTIONS
 */


uint16_t ADC_GetSingleConv();
#endif /* ADC_H_ */
