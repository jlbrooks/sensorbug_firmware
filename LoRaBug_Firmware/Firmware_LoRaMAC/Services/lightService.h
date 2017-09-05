/******************************************************************************

 @file  lightService.h

 @brief This file contains the light sensor service interface
        Created on: Jul 11, 2017

 @author: Abhinand Sukumar

 ******************************************************************************/

#ifndef SERVICES_LIGHTSERVICE_H_
#define SERVICES_LIGHTSERVICE_H_

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
 * Task creation function for the light Service.
 */
extern void lightService_createTask(void);


/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* SERVICES_LIGHTSERVICE_H_ */
