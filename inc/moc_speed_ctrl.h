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
#include "mip_volt_acq_if.h"
#include "mip_current_acq_if.h"
#include "lld_bldc_pwm_if.h"
/*
 * DEFINES
 */
#define Ts              (10000)
#define DC_LINK         (12.0F)
#define PWM_MAX_TICKS   (2099)

/*
 * TYPE-DEFs
 */
typedef struct speed_ctrl
{
  float OUTPUT_CURR;
  float ERROR_SPEED;
  float P_GAIN;
  float I_GAIN;
  float P_PART;
  float I_PART;
}speed_ctrl_t;

typedef struct curr_ctrl
{
  float OUTPUT_VOLT;
  float ERROR_CURRENT;
  float P_GAIN;
  float I_GAIN;
  float P_PART;
  float I_PART;
}curr_ctrl_t;

/*
 * VARIABLES
 */
speed_ctrl_t MOC_SPEED_CTRL;
curr_ctrl_t MOC_CURR_CTRL;

/*
 * FUNCTIONS
 */

#endif /* MOC_SPEED_CTRL_H_ */
