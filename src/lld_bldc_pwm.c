/*
 * bldc_pwm.c
 *
 *  Created on: 2 iun. 2018
 *      Author: AxinteA
 */
#include "lld_bldc_pwm.h"

/*Comutation seq look up table*/
uint16_t BLDC_PWM_MASK_LKT[8]=
{
0,
PWM_A_INACTIVE | PWM_B_ACTIVE   | PWM_C_ACTIVE,  /*OFF -   +  */
PWM_A_ACTIVE   | PWM_B_ACTIVE   | PWM_C_INACTIVE,/*-   +   OFF*/
PWM_A_ACTIVE   | PWM_B_INACTIVE | PWM_C_ACTIVE,  /*-   OFF +  */
PWM_A_ACTIVE   | PWM_B_INACTIVE | PWM_C_ACTIVE,  /*+   OFF -  */
PWM_A_ACTIVE   | PWM_B_ACTIVE   | PWM_C_INACTIVE,/*+   -   OFF*/
PWM_A_INACTIVE | PWM_B_ACTIVE   | PWM_C_ACTIVE,  /*OFF +   -  */
0
};

static void BLDC_PWM_GPIO_INIT();


void BLDC_PWM_HANDLER()
{
	switch(Get_Hall_State())
	{
	case Sector_1:
		TIM1->CCR2 = BLDC_PWM_MAX_DTC - Get_Bldc_Pwm_B();
		TIM1->CCR3 = Get_Bldc_Pwm_C();

	    TIM1->CCER = BLDC_PWM_MASK_LKT[Get_Hall_State()];
		break;
	case Sector_2:
	    TIM1->CCR1 = BLDC_PWM_MAX_DTC - Get_Bldc_Pwm_A();
		TIM1->CCR2 = Get_Bldc_Pwm_B();

	    TIM1->CCER = BLDC_PWM_MASK_LKT[Get_Hall_State()];
		break;
	case Sector_3:
	    TIM1->CCR1 = BLDC_PWM_MAX_DTC - Get_Bldc_Pwm_A();
		TIM1->CCR3 = Get_Bldc_Pwm_C();

	    TIM1->CCER = BLDC_PWM_MASK_LKT[Get_Hall_State()];
		break;
	case Sector_4:
	    TIM1->CCR1 = Get_Bldc_Pwm_A();
		TIM1->CCR3 = BLDC_PWM_MAX_DTC - Get_Bldc_Pwm_C();

	    TIM1->CCER = BLDC_PWM_MASK_LKT[Get_Hall_State()];
		break;
	case Sector_5:
	    TIM1->CCR1 = Get_Bldc_Pwm_A();
		TIM1->CCR2 = BLDC_PWM_MAX_DTC - Get_Bldc_Pwm_B();

	    TIM1->CCER = BLDC_PWM_MASK_LKT[Get_Hall_State()];
		break;
	case Sector_6:
		TIM1->CCR2 = Get_Bldc_Pwm_B();
		TIM1->CCR3 = BLDC_PWM_MAX_DTC - Get_Bldc_Pwm_C();

	    TIM1->CCER = BLDC_PWM_MASK_LKT[Get_Hall_State()];
		break;
	default:
		TIM1->CCER = BLDC_PWM_MASK_LKT[0];

	}
	HAL_GPIO_WritePin(GPIOB,PWM_EN_GATE,Get_Bldc_En_Gate());
}


void BLDC_PWM_INIT()
{

	  TIM_ClockConfigTypeDef sClockSourceConfig;
	  TIM_MasterConfigTypeDef sMasterConfig;
	  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;
	  TIM_OC_InitTypeDef sConfigOC;

	  htim1.Instance = TIM1;
	  htim1.Init.Prescaler = 0; //TIM1 Freq = SysCLK / (Prescaler +1) => Prescaler = (SysCLK / TIM1(wanted)Freq) - 1
	  htim1.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED2;
	  htim1.Init.Period = 4199; // PWM Freq = TIM1 Freq /(TIM1 Period + 1) => TIM1 Period = (TIM1 Freq / PWM Freq) - 1
	  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	  htim1.Init.RepetitionCounter = 1;
	  HAL_TIM_Base_Init(&htim1);

	  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	  HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig);

	  HAL_TIM_PWM_Init(&htim1);

	  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;//TIM_TRGO_OC1;//TIM_TRGO_UPDATE;
	  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_ENABLE;
	  HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig);

	  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
	  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
	  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
	  sBreakDeadTimeConfig.DeadTime = 168;
	  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
	  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
	  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
	  HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig);

	  sConfigOC.OCMode = TIM_OCMODE_PWM2;
	  sConfigOC.Pulse = 0; //Pulse = DTC * (TIM1 Period + 1)
	  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;

	  HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1);

	  HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2);

	  HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3);

	  BLDC_PWM_GPIO_INIT();

	  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
	  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
	  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);

      TIM1->DIER = ((0x00<<24) | (0x00<<16) | (0x00<<8) | (0x00));

	  TIM1->CCER = PWM_A_ACTIVE | PWM_B_INACTIVE | PWM_C_INACTIVE; /*Disable CHxN*/

	  /*Init app ports*/
	  Set_Bldc_Pwm_A(2099);
	  Set_Bldc_Pwm_B(2099);
	  Set_Bldc_Pwm_C(2099);
	  Set_Bldc_En_Gate(GATE_DISABLE);

}

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* tim_baseHandle)
{

  if(tim_baseHandle->Instance==TIM1)
  {
	  /*Enable clock for TIM1*/
    __HAL_RCC_TIM1_CLK_ENABLE();

    /* Peripheral interrupt init */
    /*HAL_NVIC_SetPriority(TIM1_UP_TIM10_IRQn, 0, 1);
    HAL_NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);*/
  }
}

void BLDC_PWM_GPIO_INIT()
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /**TIM1 GPIO Configuration
    PA7     ------> TIM1_CH1N
    PB0     ------> TIM1_CH2N
    PB1     ------> TIM1_CH3N
    PA8     ------> TIM1_CH1
    PA9     ------> TIM1_CH2
    PA10     ------> TIM1_CH3
    */
	/*Enable bus clock for ports*/
	__GPIOA_CLK_ENABLE();
	__GPIOB_CLK_ENABLE();
    GPIO_InitStruct.Pin = PWM_A_L| PWM_A_H | PWM_C_H | PWM_C_H;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = PWM_B_L | PWM_C_L;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin   = PWM_EN_GATE;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_MEDIUM;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}


