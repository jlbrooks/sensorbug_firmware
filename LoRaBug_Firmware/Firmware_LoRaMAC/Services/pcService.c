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
#include "pcFrameUtil.h"

/*******************************************************************************
 * MACROS
 */

#define DELAY_MS(i)    Task_sleep(((i) * 1000) / Clock_tickPeriod)

/*******************************************************************************
 * CONSTANTS
 */
#define PC_TASK_PRIORITY                     5
#define PC_TASK_STACK_SIZE                   1200

#define NUM_RAW_FRAMES 15
#define NUM_MEDIAN_FRAMES 7

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

// Frame queues
static frame_queue_t rawFrames;
static frame_queue_t medianFrames;

static void pc_new_frame(frame_t new_frame) {
    enqueue_frame(&rawFrames, new_frame);

    // Compute and enqueue new frame
    if (frame_queue_full(&rawFrames)) {
        frame_elem_t median_filtered_frame[GE_FRAME_SIZE];
        compute_median_frame(&rawFrames, median_filtered_frame);

        // Subtract median from the new frame
        for (int i = 0; i < GE_FRAME_SIZE; i++) {
            if (new_frame[i] < median_filtered_frame[i]) {
                median_filtered_frame[i] = 0;
            } else {
                median_filtered_frame[i] = new_frame[i] - median_filtered_frame[i];
            }
        }

        enqueue_frame(&medianFrames, median_filtered_frame);

        // Log new frame?
    }

    // Reset counters
}

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

    // Initialize frame queues
    frame_queue_init(&rawFrames, GE_FRAME_SIZE*sizeof(frame_elem_t), NUM_RAW_FRAMES);
    frame_queue_init(&medianFrames, GE_FRAME_SIZE*sizeof(frame_elem_t), NUM_MEDIAN_FRAMES);

    Task_construct(&pcTask, pc_taskFxn, &taskParams, NULL);
}


#endif /* SERVICES_PCSERVICE_C_ */
