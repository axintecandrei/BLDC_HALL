/*
 * moc_speed_ctrl.h
 *
 *  Created on: 16 iun. 2018
 *      Author: AxinteA
 */

#ifndef MOC_SPEED_CTRL_H_
#define MOC_SPEED_CTRL_H_
#include "util.h"
#include "moc_speed_ctrl_if.h"
#include "mip_speed_est_if.h"
#include "lld_bldc_pwm_if.h"
/*
 * DEFINES
 */
#define Ts              (10000)
#define DC_LINK         (12.0F)
#define PWM_MAX_TICKS   (4199)

/*
 * TYPE-DEFs
 */
typedef struct speed_ctrl
{
  float OUTPUT_VOLT;
  float OUTPUT_VOLT_K1;
  float ERROR_SPEED;
  float P_GAIN;
  float I_GAIN;
  float P_PART;
  float I_PART;
}speed_ctrl_t;

/*
 * VARIABLES
 */
speed_ctrl_t MOC_SPEED_CTRL;

/*
 * FUNCTIONS
 */

#endif /* MOC_SPEED_CTRL_H_ */
