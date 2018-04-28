/*
 * utilities.c
 *
 *  Created on: 20 mar. 2018
 *      Author: uidp7521
 */
#include "utilities.h"

/*some sort of delay*/
void UTIL_delay(uint32_t moment)
{
	uint32_t outer, inner;

	for(outer = 0; outer<moment; outer++)
	{
		for(inner = 0; inner<1515 ; inner++)
		{

		}
	}

}

void UTIL_ByteToBits (uint8_t in_byte, byteAsbits_t* out_bits )
{
	out_bits->bit_0 = (in_byte&  1)>>0;
	out_bits->bit_1 = (in_byte&  2)>>1;
	out_bits->bit_2 = (in_byte&  4)>>2;
	out_bits->bit_3 = (in_byte&  8)>>3;
	out_bits->bit_4 = (in_byte& 16)>>4;
	out_bits->bit_5 = (in_byte& 32)>>5;
	out_bits->bit_6 = (in_byte& 64)>>6;
	out_bits->bit_7 = (in_byte&128)>>7;
}
