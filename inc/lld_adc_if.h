/*
 * lld_adc_if.h
 *
 *  Created on: 14 iun. 2018
 *      Author: uidp7521
 */

#ifndef ADC_IF_H_
#define ADC_IF_H_
#include "../CMSIS/device/stm32f4xx.h"
#include "../inc/utilities.h"

uint16_t PORT__AdcIa;
#define Set_AdcIa(v) (PORT__AdcIa = (v))
#define Get_AdcIa()   (*((const uint16_t*) &PORT__AdcIa))

uint16_t PORT__AdcIb;
#define Set_AdcIb(v) (PORT__AdcIb = (v))
#define Get_AdcIb()   (*((const uint16_t*) &PORT__AdcIb))

uint16_t PORT__AdcIc;
#define Set_AdcIc(v) (PORT__AdcIc = (v))
#define Get_AdcIc()   (*((const uint16_t*) &PORT__AdcIc))

uint16_t PORT__AdcVa;
#define Set_AdcVa(v) (PORT__AdcVa = (v))
#define Get_AdcVa()   (*((const uint16_t*) &PORT__AdcVa))

uint16_t PORT__AdcVb;
#define Set_AdcVb(v) (PORT__AdcVb = (v))
#define Get_AdcVb()   (*((const uint16_t*) &PORT__AdcVb))

uint16_t PORT__AdcVc;
#define Set_AdcVc(v) (PORT__AdcVc = (v))
#define Get_AdcVc()   (*((const uint16_t*) &PORT__AdcVc))

uint16_t PORT__AdcDcLink;
#define Set_AdcDcLink(v) (PORT__AdcDcLink = (v))
#define Get_AdcDcLink()   (*((const uint16_t*) &PORT__AdcDcLink))


extern void ADC_INIT();
extern void LLD_ADC_GET_RAW_VAL();
extern void ADC_StartSeqConv();

#endif /* ADC_IF_H_ */
