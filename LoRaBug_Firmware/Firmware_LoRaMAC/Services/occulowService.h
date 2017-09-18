/****************************************************************
 @file occulowService.h

 @brief This file contains the light sensor service interface
        Created on: Jul 11, 2017

 @author: Jacob Brooks
 ***************************************************************/

#ifndef SERVICES_OCCULOWSERVICE_H_
#define SERVICES_OCCULOWSERVICE_H_

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */

/*********************************************************************
*  EXTERNAL VARIABLES
*/

/*********************************************************************
 * CONSTANTS
 */

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * FUNCTIONS
 */

/*
 * Task creation function for the occulow service
 */
extern void occulowService_createTask(void);


/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* SERVICES_OCCULOWSERVICE_H_ */
