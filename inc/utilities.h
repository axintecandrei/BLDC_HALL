/*
 * utilities.h
 *
 *  Created on: 20 mar. 2018
 *      Author: axint
 */

#ifndef UTILITIES_H_
#define UTILITIES_H_
#include "../CMSIS/device/stm32f4xx.h"
#include "config.h"


typedef struct byteAsbits_s
{
	uint8_t bit_0 :1;
	uint8_t bit_1 :1;
	uint8_t bit_2 :1;
	uint8_t bit_3 :1;
	uint8_t bit_4 :1;
	uint8_t bit_5 :1;
	uint8_t bit_6 :1;
	uint8_t bit_7 :1;
}byteAsbits_t;


void UTIL_delay(uint32_t moment);
void UTIL_ByteToBits (uint8_t in_byte, byteAsbits_t* out_bits );
int32_t UTIL_Maxof3(int32_t n1 ,int32_t n2,int32_t n3);
#endif /* UTILITIES_H_ */
