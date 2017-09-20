/******************************************************************************

  @file  pcFrameUtil.c

 @brief This file implements the frame utility functions for people counting


 @author: Jacob Brooks

 ******************************************************************************/

#ifndef SERVICES_PCFRAMEUTIL_C_
#define SERVICES_PCFRAMEUTIL_C_

/*********************************************************************
 * INCLUDES
 */
#include <stdio.h>
#include "pcFrameUtil.h"
#include "grideyeService.h"

/*******************************************************************************
 * MACROS
 */

#define GET_FRAME_INDEX(ROW, COL) ((ROW) * GE_GRID_SIZE + (COL))

/*******************************************************************************
 * TYPEDEFS
 */

/*******************************************************************************
 * GLOBAL VARIABLES
 */

/*******************************************************************************
 * LOCAL VARIABLES
 */

/*******************************************************************************
 * LOCAL FUNCTION DECLARATIONS
 */

static void swap (frame_elem_t *a, frame_elem_t *b);
static int16_t partition(frame_t arr, uint16_t l, uint16_t h);
static void quick_sort(frame_t arr, uint16_t l, uint16_t h);
static uint16_t median_at_index(frame_queue_t *frames, uint16_t index);


/*********************************************************************
 * PUBLIC FUNCTIONS
 */

void frame_queue_init(frame_queue_t *queue, uint16_t size, uint16_t len) {
    queue->elem_size = size;
    queue->max_len = len;
    queue->cur_len = 0;
}

uint8_t frame_queue_full(frame_queue_t *queue) {
    return (queue->cur_len == queue->max_len);
}

void enqueue_frame(frame_queue_t *queue, frame_t new_frame) {
    // Copy new data into 0th frame
    for (int i = 0; i < queue->elem_size; i++) {
        queue->frames[0][i] = new_frame[i];
    }

    // Rotate queue pointers by 1
    frame_t newest_frame = queue->frames[0];
    for (int i = 0; i < queue->max_len - 1; i++) {
        queue->frames[i] = queue->frames[i+1];
    }

    queue->frames[queue->max_len-1] = newest_frame;
    if (queue->cur_len < queue->max_len) {
        queue->cur_len += 1;
    }
}

frame_t compute_median_frame(frame_queue_t *queue, frame_t frame_out) {
    uint16_t index;

    for (uint16_t col = 0; col < GE_GRID_SIZE; col++) {
        for (uint16_t row = 0; row < GE_GRID_SIZE; row++) {
            index = GET_FRAME_INDEX(row,col);
            frame_out[index] = median_at_index(queue, index);
        }
    }

    return frame_out;
}

/*******************************************************************************
 * LOCAL FUNCTION IMPLEMENTATIONS
 */

/**
 * @brief      Swaps two frame_elements
 *
 * @param      a     { parameter_description }
 * @param      b     { parameter_description }
 */
static void swap (frame_elem_t *a, frame_elem_t *b) {
    frame_elem_t t = *a;
    *a = *b;
    *b = t;
}

/**
 * @brief      Partitions a frame for quicksort
 *
 * @param[in]  arr   List to partition
 * @param[in]  l     Element to start partitioning from
 * @param[in]  h     Element to stop partitioning at
 *
 * @return     Index of the partition
 */
static int16_t partition(frame_t arr, uint16_t l, uint16_t h) {
    frame_elem_t x = arr[h];
    int16_t i = (l - 1);

    for (int16_t j = l; j <= h - 1; j++) {
        if (arr[j] <= x) {
            i++;
            swap(&arr[i], &arr[j]);
        }
    }
    swap(&arr[i + 1], &arr[h]);
    return (i + 1);
}

/**
 * @brief      Performs quicksort on a slice of an array from l to h
 *
 * @param[in]  arr   Array to sort
 * @param[in]  l     Index to start sorting from
 * @param[in]  h     Index to stop sorting from
 */
static void quick_sort(frame_t arr, uint16_t l, uint16_t h) {
    int16_t stack[h - l + 1];
    int16_t top = -1;

    stack[++top] = l;
    stack[++top] = h;

    while (top >= 0) {
        h = stack[top--];
        l = stack[top--];

        int16_t p = partition(arr, l, h);

        if (p-1 > l) {
            stack[++top] = l;
            stack[++top] = p - 1;
        }
        if (p+1 < h) {
            stack[++top] = p + 1;
            stack[++top] = h;
        }
    }
}

static uint16_t median_at_index(frame_queue_t *frames, uint16_t index) {
    frame_elem_t temp_arr[frames->cur_len];
    // Copy all elements at index into temp
    for (int i = 0; i < frames->cur_len; i++) {
        temp_arr[i] = frames->frames[i][index];
    }

    quick_sort(temp_arr, 0, frames->cur_len-1);

    // Return median
    if (frames->cur_len % 2 == 0) {
        return ((temp_arr[frames->cur_len/2] + temp_arr[frames->cur_len/2 -1]) / 2);
    } else {
        return temp_arr[frames->cur_len/2];
    }
}

#endif /* SERVICES_PCFRAMEUTIL_C_ */
