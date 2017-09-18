/******************************************************************************

  @file  occulowService.c

 @brief This file implements the Occulow people-counting code


 @author: Jacob Brooks

 ******************************************************************************/

#ifndef SERVICES_PCSERVICE_C_
#define SERVICES_PCSERVICE_C_

/*********************************************************************
 * INCLUDES
 */
#include <stdio.h>
#include <string.h>

#include <xdc/runtime/System.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>


/* Board Header files */
#include "Config/Board_LoRaBUG.h"

#include "io.h"

#include "grideyeService.h"
#include "pcService.h"

/*******************************************************************************
 * MACROS
 */

#define DELAY_MS(i)    Task_sleep(((i) * 1000) / Clock_tickPeriod)

/*******************************************************************************
 * CONSTANTS
 */
#define PC_TASK_PRIORITY                     5

#define PC_TASK_STACK_SIZE                   1200

/*******************************************************************************
 * TYPEDEFS
 */

/*******************************************************************************
 * GLOBAL VARIABLES
 */

/*******************************************************************************
 * LOCAL VARIABLES
 */

// Task configuration
Task_Struct pcTask;
Char pcTaskStack[PC_TASK_STACK_SIZE];

/*********************************************************************
 * @fn      occulow_taskFxn
 * @return  None.
 */
static void pc_taskFxn(UArg a0, UArg a1) {

}

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      pcService_createTask
 *
 * @brief   Task creation function for the people-counting application.
 *
 * @param   None.
 *
 * @return  None.
 */
void pcService_createTask(void)
{
    Task_Params taskParams;

    // Configure task
    Task_Params_init(&taskParams);
    taskParams.stack = pcTaskStack;
    taskParams.stackSize = PC_TASK_STACK_SIZE;
    //taskParams.priority = GRIDEYE_TASK_PRIORITY;

    Task_construct(&pcTask, pc_taskFxn, &taskParams, NULL);
}


#endif /* SERVICES_PCSERVICE_C_ */
