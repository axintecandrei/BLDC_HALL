#ifndef ADC_H_
#define ADC_H_

#include "stm32f4xx.h"
#include "utilities.h"

/*
 * DEFINES
 */
#define ADC_PORT  GPIOA
#define ADC_IN   GPIO_PIN_0

/*
 * TYPE-DEFs
 */


/*
 * VARIABLES
 */



/*
 * FUNCTIONS
 */

void ADC_INIT();
uint16_t ADC_GetConv();
#endif /* ADC_H_ */
