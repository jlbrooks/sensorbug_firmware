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
    uint16_t size;
    uint16_t len;
} frame_queue_t;

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * FUNCTIONS
 */

void enqueue_frame(frame_queue_t *queue, frame_t new_frame);

#endif /* SERVICES_PCFRAMEUTIL_H_ */
