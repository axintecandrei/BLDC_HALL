#ifndef __TASKS_H
#define __TASKS_H

#include "stm32f4xx_hal.h"
#include "mip_speed_est_if.h"
#include "moc_speed_ctrl_if.h"
#include "lld_adc_if.h"
#include "lld_bldc_pwm_if.h"
#include "lld_hall_if.h"
#include "freemaster.h"
#include "freemaster_private.h"


void _100US_LOOP (void);

void _2MS_LOOP (void);

void _10MS_LOOP (void);


#endif /*__TASKS_H*/
