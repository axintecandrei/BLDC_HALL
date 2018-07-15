/*
 * gpio.h
 *
 *  Created on: 20 mar. 2018
 *      Author: axint
 */

#ifndef GPIO_H_
#define GPIO_H_
#include "stm32f4xx.h"
#include "utilities.h"

/*
 * DEFINES
 */

#define NIBBLE_1_PORT GPIOC
#define D0            GPIO_PIN_2
#define D1            GPIO_PIN_3
#define D2            GPIO_PIN_4
#define D3            GPIO_PIN_5

#define BUTT_PORT     GPIOC
#define BUTT_1        GPIO_PIN_6
#define BUTT_2        GPIO_PIN_7

#define NIBBLE_2_PORT GPIOB
#define D4            GPIO_PIN_6
#define D5            GPIO_PIN_7
#define D6            GPIO_PIN_8
#define D7            GPIO_PIN_9


#define CONFIDENCE_THR    (100)
#define BUTT_NR			  (  2)

/*
 * TYPE-DEFs
 */
typedef enum pin_state_e
{
	released =0,
	pressed = 1
}pin_state_t;

typedef struct debounce_state_s
{
	pin_state_t button_state;
	uint32_t    ButtonPressedConfidanceLevel;
	uint32_t    ButtonReleasedConfidanceLevel;
}debounce_state_t;

/*
 * VARIABLES
 */

debounce_state_t DEBOUNCE_BUTTx[BUTT_NR];


/*
 * FUNCTIONS
 */

void GPIO_INIT();
uint8_t GPIO_DebounceButton(GPIO_TypeDef* port, uint16_t button, debounce_state_t* port_debounce_str);
void GPIO_ByteOnPins(uint8_t byte);
#endif /* GPIO_H_ */
