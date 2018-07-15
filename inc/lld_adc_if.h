/*
 * adc_if.h
 *
 *  Created on: 14 iun. 2018
 *      Author: uidp7521
 */

#ifndef ADC_IF_H_
#define ADC_IF_H_

int16_t PORT__AdcIa;
#define Set_AdcIa(v) (PORT__AdcIa = (v))
#define Get_AdcIa()   (*((const int16_t*) &PORT__AdcIa))

int16_t PORT__AdcIb;
#define Set_AdcIb(v) (PORT__AdcIb = (v))
#define Get_AdcIb()   (*((const int16_t*) &PORT__AdcIb))

int16_t PORT__AdcVa;
#define Set_AdcVa(v) (PORT__AdcVa = (v))
#define Get_AdcVa()   (*((const int16_t*) &PORT__AdcVa))

int16_t PORT__AdcVb;
#define Set_AdcVb(v) (PORT__AdcVb = (v))
#define Get_AdcVb()   (*((const int16_t*) &PORT__AdcVb))


ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

void ADC_INIT();
void ADC_GetSeqConv();
#endif /* ADC_IF_H_ */
