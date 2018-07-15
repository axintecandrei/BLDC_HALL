#ifndef ADC_H_
#define ADC_H_

#include "stm32f4xx.h"
#include "lld_adc_if.h"
#include "utilities.h"

/*
 * DEFINES
 */
#define ADC_PORT_A  GPIOA
#define Ia_Pin      GPIO_PIN_0
#define Ib_Pin      GPIO_PIN_1
#define Va_Pin      GPIO_PIN_4
#define Vb_Pin      GPIO_PIN_6

#define ADC_PORT_C     GPIOC
#define Vc_Pin         GPIO_PIN_4
#define DC_Link_Pin    GPIO_PIN_6
#define Trig_ADC_Pin   GPIO_PIN_11

#define ADC_BUFFER_SIZE      6
#define Adc_Ia_ch            0
#define Adc_Ib_ch            1
#define Adc_Va_ch            2
#define Adc_Vb_ch            3
#define Adc_Vc_ch            4
#define Adc_DC_Link_ch       5

#define ADC_GET_RAW_VAL(channel) (ADC_TO_DMA_BUFFER[channel])

/*
 * TYPE-DEFs
 */


/*
 * VARIABLES
 */


__IO uint16_t ADC_TO_DMA_BUFFER[ADC_BUFFER_SIZE];

/*
 * FUNCTIONS
 */


#endif /* ADC_H_ */
