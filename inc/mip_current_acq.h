/*
 * mip_current_acq.h
 *
 *  Created on: 16 iun. 2018
 *      Author: AxinteA
 */

#ifndef MIP_CURRENT_ACQ_H_
#define MIP_CURRENT_ACQ_H_
#include "mip_current_acq_if.h"
#include "../inc/utilities.h"
/*
 * DEFINES
 */
#define ADC2VOLT_FACTOR   (0.000805664F)    /*(3.3V/4096)*/
#define CURRENT_OFFSET (1.65F)
#define CURRENT_GAIN   (10.0F)
#define R_SHUNT        (0.01F)  /*10mOhm*/

/*
 * TYPE-DEFs
 */


/*
 * VARIABLES
 */

/*
 * FUNCTIONS
 */

#endif /* MIP_CURRENT_ACQ_H_ */
