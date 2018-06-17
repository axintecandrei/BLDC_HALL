#include "task_scheduler.h"


#ifdef TASK_SCH_0_1


/*Task Scheduler 0*/

task_t task[3] = {{1, 0, _100US_LOOP, 0, 0},
                  {19, 19, _2MS_LOOP, 0, 0},
                  {99, 99, _10MS_LOOP, 0, 0}};

/*1ms tick function called from interrupt */
void task_scheduler(void) 
{
  
  int i;	
  for(i=0;i<3;i++)
  {
    if(task[i].wait > 1)
    {
      task[i].wait--;
    }else 
    {
      task[i].wait = task[i].period;
      task[i].start = 1;      
    }    
  }
  
  i=0;
  for(i=0;i<3;i++)
  {
    if(task[i].lock)
    {
      break;
    }else if(task[i].start)
    {
      task[i].lock = 1;
      task[i].function();
      task[i].lock = 0;
      task[i].start = 0;
    }  
  }
}

#else

/*Task Scheduler 1*/

uint8_t runningTasks[4] = {255}; //Track running tasks-[0] always idleTask
const uint8_t idleTask = 255; // 0 highest priority, 255 lowest
uint8_t currentTask = 0; // Index of highest priority task in runningTasks
const uint8_t tasksNum =3;
uint16_t tasksPeriod = 1; 

/*The priority is given by the position in the array */
task_t tasks[3]={{1, 1, _100US_LOOP, 0},
                 {20, 20, _2MS_LOOP, 0},
                 {100, 100, _10MS_LOOP, 0}};

void task_scheduler(void) 
{
  uint8_t i;
  //SaveContext(); //save temporary registers, if necessary
  
  //HAL_GPIO_TogglePin(TP1_GPIO_Port, TP1_Pin);//HAL_GPIO_WritePin(TP1_GPIO_Port, TP1_Pin, GPIO_PIN_SET);
  for (i=0; i < tasksNum; ++i) 
  { // Heart of scheduler code
    
    if ( (tasks[i].elapsedTime >= tasks[i].period)      // Task ready
        && (runningTasks[currentTask] > i)              // Task priority > current task priority
        && (!tasks[i].running)      )                    // Task not already running (no self-preemption)
          {
              
              //HAL_SuspendTick();                        //DisableInterrupts -  Critical section void 

              tasks[i].elapsedTime = 0; // Reset time since last tick
              tasks[i].running = 1; // Mark as running
              currentTask += 1;
              runningTasks[currentTask] = i; // Add to runningTasks
              
             // HAL_ResumeTick();                         //EnableInterrupts - End critical section void 
              
              tasks[i].function(); // Execute task
              
              //HAL_SuspendTick();                        //DisableInterrupts - Critical section 
              
              tasks[i].running = 0;
              runningTasks[currentTask] = idleTask; // Remove from runningTasks
              currentTask -= 1;
              
              //HAL_ResumeTick();                         //EnableInterrupts - End critical section void 
             
            }
    tasks[i].elapsedTime += tasksPeriod;
  }
   //HAL_GPIO_TogglePin(TP1_GPIO_Port, TP1_Pin);//HAL_GPIO_WritePin(TP1_GPIO_Port, TP1_Pin, GPIO_PIN_RESET);
  //RestoreContext();//restore temporary registers, if necessary
}


#endif /*TASK_SCH_0_1*/
