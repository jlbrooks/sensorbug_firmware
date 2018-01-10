/******************************************************************************

 @file  bmeService.h

 @brief This file contains the bme680 Service Interface
        Created on: Jul 11, 2017

 @author: Abhinand Sukumar

 ******************************************************************************/

#ifndef SERVICES_BMESERVICE_H_
#define SERVICES_BMESERVICE_H_

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
 * Task creation function for the BME Service.
 */
extern void bmeService_createTask(void);

extern void getBMEData(float *temp, float *pres, float *hum);

extern void testBMEChip();
/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* SERVICES_BMESERVICE_H_ */
