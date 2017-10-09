/******************************************************************************

 @file  pir.h

 @brief This file contains the PIR interrupt interface
        Created on: October 9, 2017

 @author: Jacob Brooks

 ******************************************************************************/

#ifndef SERVICES_PIR_H_
#define SERVICES_PIR_H_

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

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * FUNCTIONS
 */

void pir_init();

void pir_enable_interrupt();

void pir_disable_interrupt();

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* SERVICES_PIR_H_ */
