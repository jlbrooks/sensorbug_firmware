/******************************************************************************

  @file  pir.c

 @brief This file contains the PIR interface
         Created on: Oct 10, 2017

 @author: Jacob Brooks

 ******************************************************************************/

#ifndef SERVICES_PIR_C_
#define SERVICES_PIR_C_

/*********************************************************************
 * INCLUDES
 */
#include <stdio.h>
#include <string.h>

#include <ti/sysbios/BIOS.h>
#include <xdc/runtime/System.h>
#include <xdc/runtime/Error.h>

/* Board Header files */
#include "Config/Board_LoRaBUG.h"

#include "io.h"
#include "board.h"


/*******************************************************************************
 * MACROS
 */

/*******************************************************************************
 * CONSTANTS
 */

#define PIR_PIN Board_HDR_ADIO6

/*******************************************************************************
 * TYPEDEFS
 */

/*******************************************************************************
 * GLOBAL VARIABLES
 */

/*******************************************************************************
 * LOCAL VARIABLES
 */

static PIN_Handle pirPinHandle;
static PIN_State pirPinState;

static PIN_Config pirPinTable[] = {
    PIR_PIN | PIN_INPUT_EN,
    PIN_TERMINATE
};

static void testCallback(PIN_Handle handle, PIN_Id pinId) {
    //uartprintf("Callback!\r\n");
    toggleLed(Board_GLED);
}


/*********************************************************************
 * PUBLIC FUNCTIONS
 */

void pir_init() {
    pirPinHandle = PIN_open(&pirPinState, pirPinTable);
    if (pirPinHandle == NULL)
    {
        uartputs("Failed to open board header pins\r\n");
    }
}

void pir_enable_interrupt() {
    if (PIN_registerIntCb(&pirPinState, testCallback) != PIN_SUCCESS) {
        uartputs("Error registering pin callback\r\n");
    }
    if (PIN_setInterrupt(&pirPinState, PIR_PIN | PIN_IRQ_POSEDGE) != PIN_SUCCESS) {
        uartputs("Error setting pin interrupt\r\n");
    }
}

void pir_disable_interrupt() {
    PIN_setInterrupt(&pirPinState, PIR_PIN | PIN_IRQ_DIS);
}


#endif /* SERVICES_PIR_C_ */
