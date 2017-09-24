/******************************************************************************

 @file  bmeService.h

 @brief This file contains the grideye Service Interface
        Created on: Jul 6, 2017

 @author: Abhinand Sukumar

 ******************************************************************************/

#ifndef SERVICES_GRIDEYESERVICE_H_
#define SERVICES_GRIDEYESERVICE_H_

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */

#include <stdbool.h>
#include "pcFrameUtil.h"

/*********************************************************************
*  EXTERNAL VARIABLES
*/

/*********************************************************************
 * CONSTANTS
 */

#define GE_FRAME_SIZE 64
#define GE_GRID_SIZE 8
#define MAX_GE_GRID_INDEX (GE_GRID_SIZE-1)
#define GE_MODE_NORMAL 0x00
#define GE_MODE_SLEEP 0x10
#define GE_MODE_STANDBY_1 0x20
#define GE_MODE_STANDBY_2 0x21

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * FUNCTIONS
 */

/*
 * Task creation function for the GRIDEYE Service.
 */
extern void grideyeService_createTask(void);

bool mailbox_receive_frame(frame_t *frame);


/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* SERVICES_GRIDEYESERVICE_H_ */
