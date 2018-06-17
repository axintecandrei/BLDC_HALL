#ifndef __usart_H
#define __usart_H
#include "stm32f4xx_hal.h"


#define SCI_TX_Pin GPIO_PIN_2
#define SCI_TX_GPIO_Port GPIOA
#define SCI_RX_Pin GPIO_PIN_3
#define SCI_RX_GPIO_Port GPIOA




extern UART_HandleTypeDef huart2;

extern void Error_Handler(void);
void USART2_UART_Init(void);

#endif /*__usart_H*/
