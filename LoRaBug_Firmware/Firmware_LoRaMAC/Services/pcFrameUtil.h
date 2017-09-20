/******************************************************************************

  @file  pcFrameUtil.h

 @brief This file defines the frame utility interface for people counting


 @author: Jacob Brooks

 ******************************************************************************/


#ifndef SERVICES_PCFRAMEUTIL_H_
#define SERVICES_PCFRAMEUTIL_H_

/*********************************************************************
 * INCLUDES
 */
#include <stdint.h>

/*********************************************************************
*  EXTERNAL VARIABLES
*/

/*********************************************************************
 * CONSTANTS
 */

/*******************************************************************************
 * TYPEDEFS
 */

typedef uint16_t frame_elem_t;

typedef frame_elem_t *frame_t;

typedef struct frame_queue {
    frame_t *frames;
    uint16_t elem_size;
    uint16_t max_len;
    uint16_t cur_len;
} frame_queue_t;

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * FUNCTIONS
 */

void frame_queue_init(frame_queue_t *queue, uint16_t size, uint16_t len);

uint8_t frame_queue_full(frame_queue_t *queue);

void enqueue_frame(frame_queue_t *queue, frame_t new_frame);

frame_t compute_median_frame(frame_queue_t *queue, frame_t frame_out);

#endif /* SERVICES_PCFRAMEUTIL_H_ */
