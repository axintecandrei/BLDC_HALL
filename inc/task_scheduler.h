#ifndef __TASKSCH_H
#define __TASKSCH_H

#include "stdint.h"
#include "tasks.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_tim.h"


//#define TASK_SCH_0_1

#ifdef TASK_SCH_0_1

/*Task Structure 0*/
typedef struct task_t
{
  uint16_t period;
  uint16_t wait;
  void(*function) (void);
  uint8_t lock;
  uint8_t start;
}task_t; 


void task_scheduler (void);

#else

/*Tasks Structure 1*/
typedef struct task_t
{
  
  uint16_t period;                   /*The interval that the task should be executed*/
  uint16_t elapsedTime;              /*the amount of time that has passed since the previous execution of the task*/
  void(*function) (void);            /* pointer to the task’s function.*/
  uint8_t running;                   /*Flag that is checked to see if the task is ready to run*/

}task_t;



void task_scheduler (void);


#endif /*TASK_SCH_0_1*/

#endif /*__TASKSCH_H*/
