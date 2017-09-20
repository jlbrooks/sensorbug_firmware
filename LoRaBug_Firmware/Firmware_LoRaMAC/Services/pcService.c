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
#include <stdbool.h>

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

#define NUM_TRIGGER_COLUMNS 2
#define TRIGGER_INDEX (NUM_MEDIAN_FRAMES/2)

#define TRIGGER_THRESHOLD 8  // Threshold to detect as a person
#define LOWER_THRESHOLD 4  // Threshold for heat signature difference in frames
#define UPPER_THRESHOLD 20  // Threshold for heat signature difference in frames

/*******************************************************************************
 * TYPEDEFS
 */

typedef struct {
    double in_count;  //< Number of counts going in
    double out_count;  //< Number of counts going out
    bool count_updated;  //< Whether or not the count has been updated since the last new frame
} pc_counter_t;

typedef struct {
    int16_t trigger_column[2];  //< Columns to detect movement on
    int16_t trigger_check_offset[2];  //< Offsets to check for direction
} pc_config_t;

typedef enum {DIR_IN, DIR_OUT, DIR_NONE} direction_t;

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

// Counter/config structs
static pc_counter_t counter;
static pc_config_t config;

static uint32_t frame_count;
static uint32_t last_frame_counted;

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
        frame_count++;

        // Log new frame?
    }

    // Reset counters
}

static double pc_get_in_count(void) {
    if (!counter.count_updated) {
        update_counter();
    }

    return counter.in_count;
}

static double pc_get_out_count(void) {
    if (!counter.count_updated) {
        update_counter();
    }

    return counter.out_count;
}

static void counter_init(pc_counter_t *counter)
{
    counter->in_count = 0;
    counter->out_count = 0;
    counter->count_updated = false;
}

static void config_init(pc_config_t *config) {
    config->trigger_column[0] = 2;
    config->trigger_column[1] = 5;
    config->trigger_check_offset[0] = 1;
    config->trigger_check_offset[1] = -1;
};

static bool within_threshold(int16_t current, int16_t next) {
    int16_t diff = abs(current - next);
    return (current > next && diff <= LOWER_THRESHOLD)
        || (current <= next && diff <= UPPER_THRESHOLD);
}

static direction_t determine_direction(uint16_t frame_index, int16_t trigger_col, int16_t offset) {
    if (trigger_col + offset < 0 || trigger_col + offset >= GE_GRID_SIZE) {
        //LOG_LINE("Trigger offset/column out of bounds: (%d, %d)", trigger_col, offset);
        return DIR_NONE;
    }
    uint16_t check_col = (uint16_t) (trigger_col + offset);
    frame_t current_frame = frame_queue_get(&medianFrames, frame_index);
    uint16_t current_max_index = get_max_index_in_col(current_frame, trigger_col);
    int16_t current_max = current_frame[GET_FRAME_INDEX(current_max_index, trigger_col)];

    if (current_max >= TRIGGER_THRESHOLD && is_local_max(current_frame, current_max_index, trigger_col)) {
        // Check the past
        for (uint16_t i = 1; i < 3; i++) { // TODO: MAGIC NUMBERS
            frame_t past_frame = frame_queue_get(&medianFrames, frame_index-i);
            uint16_t past_max_index = get_max_index_in_col(past_frame, check_col);
            int16_t past_max = past_frame[GET_FRAME_INDEX(past_max_index, check_col)];
            if (within_threshold(current_max, past_max) && is_local_max(past_frame, past_max_index, check_col)) {
                return (offset > 0) ? DIR_OUT : DIR_IN;
            }
        }
        // TODO: These loops are similar, refactor?

        // Check the future
        for (uint16_t i = 1; i < 3; i++) { // TODO: MAGIC NUMBERS
            frame_t future_frame = frame_queue_get(&medianFrames, frame_index+i);
            uint16_t future_max_index = get_max_index_in_col(future_frame, check_col);
            int16_t future_max = future_frame[GET_FRAME_INDEX(future_max_index, check_col)];
            if (within_threshold(current_max, future_max) && is_local_max(future_frame, future_max_index, check_col)) {
                return (offset > 0) ? DIR_IN : DIR_OUT;
            }
        }
    }

    return DIR_NONE;
}

static void update_counter(void) {
    // Exit early if count has already been updated or the buffer isn't full
    if (!frame_queue_full(&rawFrames) || counter.count_updated) {
        return;
    }

    for (int i = 0; i < NUM_TRIGGER_COLUMNS; i++) {
        uint16_t trigger_col = config.trigger_column[i];
        uint16_t offset = config.trigger_check_offset[i];
        direction_t direction = determine_direction(TRIGGER_INDEX, trigger_col, offset);
    }

    counter.count_updated = true;
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
    // Initialize counter/config
    counter_init(&counter);
    config_init(&config);

    Task_construct(&pcTask, pc_taskFxn, &taskParams, NULL);
}


#endif /* SERVICES_PCSERVICE_C_ */
