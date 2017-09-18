/******************************************************************************

  @file  grideyeService.c

 @brief This file implements the Occulow people-counting code


 @author: Jacob Brooks

 ******************************************************************************/

#ifndef SERVICES_OCCULOWSERVICE_C_
#define SERVICES_OCCULOWSERVICE_C_

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
#include "occulowService.h"

/*******************************************************************************
 * MACROS
 */

#define DELAY_MS(i)    Task_sleep(((i) * 1000) / Clock_tickPeriod)

/*******************************************************************************
 * CONSTANTS
 */
#define OCCULOW_TASK_PRIORITY                     5

#define OCCULOW_TASK_STACK_SIZE                   1200

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
Task_Struct occulowTask;
Char occulowTaskStack[OCCULOW_TASK_STACK_SIZE];

/*********************************************************************
 * @fn      occulow_taskFxn
 * @return  None.
 */
static void occulow_taskFxn(UArg a0, UArg a1) {

}

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      occulowService_createTask
 *
 * @brief   Task creation function for the Occulow application.
 *
 * @param   None.
 *
 * @return  None.
 */
void occulowService_createTask(void)
{
    Task_Params taskParams;

    // Configure task
    Task_Params_init(&taskParams);
    taskParams.stack = occulowTaskStack;
    taskParams.stackSize = OCCULOW_TASK_STACK_SIZE;
    //taskParams.priority = GRIDEYE_TASK_PRIORITY;

    Task_construct(&occulowTask, occulow_taskFxn, &taskParams, NULL);
}


#endif /* SERVICES_OCCULOWSERVICE_C_ */
