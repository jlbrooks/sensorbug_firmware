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
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Semaphore.h>


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
#define PC_TASK_STACK_SIZE                   2048

#define NUM_RAW_FRAMES 11
#define NUM_MEDIAN_FRAMES 5

#define MEDIAN_FRAME_CHUNK_SIZE ((NUM_MEDIAN_FRAMES) * (GE_FRAME_SIZE))
#define RAW_FRAME_CHUNK_SIZE ((NUM_RAW_FRAMES) * (GE_FRAME_SIZE))

#define NUM_TRIGGER_COLUMNS 2
#define TRIGGER_INDEX (NUM_MEDIAN_FRAMES/2)

#define TRIGGER_THRESHOLD 8  // Threshold to detect as a person
#define LOWER_THRESHOLD 4  // Threshold for heat signature difference in frames
#define UPPER_THRESHOLD 20  // Threshold for heat signature difference in frames

/*******************************************************************************
 * TYPEDEFS
 */

typedef struct {
    int16_t trigger_column[2];  //< Columns to detect movement on
    int16_t trigger_check_offset[2];  //< Offsets to check for direction
} pc_config_t;

typedef struct pc_internal_counter {
    double in_count;  //< Number of counts going in
    double out_count;  //< Number of counts going out
    bool count_updated;  //< Whether or not the count has been updated since the last new frame
} pc_internal_counter_t;

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

// Space for frame queues
static frame_elem_t medianFrameChunks[MEDIAN_FRAME_CHUNK_SIZE];
static frame_elem_t rawFrameChunks[RAW_FRAME_CHUNK_SIZE];

// Allocate lists of pointers into the chunks
static frame_t rawFramePtrs[NUM_RAW_FRAMES];
static frame_t medianFramePtrs[NUM_MEDIAN_FRAMES];

// Counter/config structs
static pc_internal_counter_t internal_counter;
static pc_counter_t counter;
static pc_config_t config;

// Counter lock
static Semaphore_Struct count_sem;

static uint32_t frame_count;
static uint32_t last_frame_counted;

// Local functions
static void pc_new_frame(frame_t new_frame);
static double pc_get_in_count(void);
static double pc_get_out_count(void);
static void counter_init(pc_counter_t *counter);
static void internal_counter_init(pc_internal_counter_t *counter);
static void config_init(pc_config_t *config);
static bool within_threshold(int16_t current, int16_t next);
static direction_t determine_direction(uint16_t frame_index, int16_t trigger_col, int16_t offset);
static void update_internal_counter(void);
static void pc_update_counts(double count_in, double count_out);


static void pc_new_frame(frame_t new_frame) {
    static frame_elem_t median_filtered_frame[GE_FRAME_SIZE];
    enqueue_frame(&rawFrames, new_frame);

    // Compute and enqueue new frame
    if (frame_queue_full(&rawFrames)) {
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
    internal_counter.count_updated = false;
    internal_counter.in_count = 0;
    internal_counter.out_count = 0;
}

static double pc_get_in_count(void) {
    if (!internal_counter.count_updated) {
        update_internal_counter();
    }

    return internal_counter.in_count;
}

static double pc_get_out_count(void) {
    if (!internal_counter.count_updated) {
        update_internal_counter();
    }

    return internal_counter.out_count;
}

static void internal_counter_init(pc_internal_counter_t *c)
{
    c->in_count = 0;
    c->out_count = 0;
    c->count_updated = false;
}

static void counter_init(pc_counter_t *c)
{
    c->in_count = 0;
    c->out_count = 0;
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

static void update_internal_counter(void) {
    // Exit early if count has already been updated or the buffer isn't full
    if (!frame_queue_full(&rawFrames) || internal_counter.count_updated) {
        return;
    }

    for (int i = 0; i < NUM_TRIGGER_COLUMNS; i++) {
        uint16_t trigger_col = config.trigger_column[i];
        uint16_t offset = config.trigger_check_offset[i];
        direction_t direction = determine_direction(TRIGGER_INDEX, trigger_col, offset);

        if (last_frame_counted < frame_count - 2) {
            switch (direction) {
            case DIR_IN:
                internal_counter.in_count = internal_counter.in_count + 0.5;
                last_frame_counted = frame_count;
                break;
            case DIR_OUT:
                internal_counter.out_count = internal_counter.out_count + 0.5;
                last_frame_counted = frame_count;
                break;
            }
        }
    }

    internal_counter.count_updated = true;
}

static void print_frame(uint16_t *frame) {
    for (int i = 0; i < GE_FRAME_SIZE; i++) {
        if (i < GE_FRAME_SIZE - 1) {
            uartprintf("%d,", frame[i]);
        } else {
            uartprintf("%d", frame[i]);
        }
    }
    uartprintf("\r\n");
}

/*********************************************************************
 * @fn      occulow_taskFxn
 * @return  None.
 */
static void pc_taskFxn(UArg a0, UArg a1) {
    static frame_elem_t frame[GE_FRAME_SIZE];
    double in_count, out_count;
    DELAY_MS(5000);
    grideye_init();

    while (1) {
        //uartputs("Starting to wait for frame...\r\n");
        //grideye_get_frame(frame);
        mailbox_receive_frame(frame);
        //uartprintf("Got frame: %d\r\n", result);
        print_frame(frame);
        pc_new_frame(frame);
        //uartputs("Done new frame\r\n");
        in_count = pc_get_in_count();
        out_count = pc_get_out_count();

        if (in_count > 0.0 || out_count > 0.0) {
            pc_update_counts(in_count, out_count);
            //uartprintf("In: %f out: %f\r\n", in_count, out_count);
        }
        toggleLed(Board_RLED);
    }
}

static void pc_update_counts(double count_in, double count_out) {
    Semaphore_pend(Semaphore_handle(&count_sem), BIOS_WAIT_FOREVER);
    counter.in_count = counter.in_count + count_in;
    counter.out_count = counter.out_count + count_out;
    Semaphore_post(Semaphore_handle(&count_sem));
}

/*********************************************************************
 * PUBLIC FUNCTIONS
 */


void pc_get_counts(pc_counter_t *out_counter, bool reset) {
    Semaphore_pend(Semaphore_handle(&count_sem), BIOS_WAIT_FOREVER);
    out_counter->in_count = counter.in_count;
    out_counter->out_count = counter.out_count;
    if (reset) {
        counter.in_count = 0;
        counter.out_count = 0;
    }
    Semaphore_post(Semaphore_handle(&count_sem));
}


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
    Semaphore_Params semaphoreParams;

    // Configure task
    Task_Params_init(&taskParams);
    taskParams.stack = pcTaskStack;
    taskParams.stackSize = PC_TASK_STACK_SIZE;
    //taskParams.priority = GRIDEYE_TASK_PRIORITY;

    // Initialize frame queues
    for (int i = 0; i < NUM_RAW_FRAMES; i++) {
        rawFramePtrs[i] = &rawFrameChunks[i * GE_FRAME_SIZE];
    }
    for (int i = 0; i < NUM_MEDIAN_FRAMES; i++) {
        medianFramePtrs[i] = &medianFrameChunks[i * GE_FRAME_SIZE];
    }
    frame_queue_init(&rawFrames, rawFramePtrs, GE_FRAME_SIZE, NUM_RAW_FRAMES);
    frame_queue_init(&medianFrames, medianFramePtrs, GE_FRAME_SIZE, NUM_MEDIAN_FRAMES);
    // Initialize counter/config
    internal_counter_init(&internal_counter);
    counter_init(&counter);
    config_init(&config);

    // Initialize semaphore
    Semaphore_Params_init(&semaphoreParams);
    semaphoreParams.mode = Semaphore_Mode_BINARY;
    Semaphore_construct(&count_sem, 1, &semaphoreParams);

    Task_construct(&pcTask, pc_taskFxn, &taskParams, NULL);
}


#endif /* SERVICES_PCSERVICE_C_ */
