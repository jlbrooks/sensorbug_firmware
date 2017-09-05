/******************************************************************************

  @file  grideyeService.c

 @brief This file contains a simple I2C transaction with the GRIDEYE
         Created on: Jul 6, 2017

 @author: Abhinand Sukumar

 ******************************************************************************/

#ifndef SERVICES_GRIDEYESERVICE_C_
#define SERVICES_GRIDEYESERVICE_C_

/*********************************************************************
 * INCLUDES
 */
#include <stdio.h>
#include <string.h>

#include <xdc/runtime/System.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>


/* Board Header files */
#include "Config/Board_LoRaBUG.h"

//#include "loraCommon.h"

#include <ti/drivers/I2C.h>
#include <ti/drivers/i2c/I2CCC26XX.h>

/*******************************************************************************
 * MACROS
 */

#define DELAY_MS(i)    Task_sleep(((i) * 1000) / Clock_tickPeriod)

/*******************************************************************************
 * CONSTANTS
 */
#define GRIDEYE_TASK_PRIORITY                     5

#define GRIDEYE_TASK_STACK_SIZE                   1200


#define LED_PIN_RX      Board_GLED
#define LED_PIN_TX      Board_RLED

/*******************************************************************************
 * TYPEDEFS
 */

/*******************************************************************************
 * GLOBAL VARIABLES
 */

/*******************************************************************************
 * LOCAL VARIABLES
 */

// Task configuration
Task_Struct grideyeTask;
Char grideyeTaskStack[GRIDEYE_TASK_STACK_SIZE];

static PIN_Handle ledPinHandle;
static PIN_State ledPinState;

static PIN_Config ledPinTable[] = {
    Board_GLED | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    Board_RLED | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    PIN_TERMINATE
};

static PIN_Handle enPinHandle;
static PIN_State enPinState;

static PIN_Config enPinTable[] = {
    Board_HDR_PORTF6 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    PIN_TERMINATE
};



/*********************************************************************
 * @fn      grideye_taskFxn
 * @return  None.
 */
static void grideye_taskFxn (UArg a0, UArg a1)
{
    /* Open LED pins */
    ledPinHandle = PIN_open(&ledPinState, ledPinTable);
    enPinHandle = PIN_open(&enPinState, enPinTable);
    if(!enPinHandle) {
        ;
    }
    PIN_setOutputValue(enPinHandle, Board_HDR_PORTF6, false);

    //PIN_setOutputValue(ledPinHandle, LED_PIN_TX, true);
    if(!ledPinHandle) {
        ;
    }

//    while(true){
//
//        PIN_setOutputValue(ledPinHandle, LED_PIN_TX, true);
//        DELAY_MS(1000);
//        PIN_setOutputValue(ledPinHandle, LED_PIN_TX, false);
//        DELAY_MS(1000);
//    }
    I2C_Handle handle;
    I2C_Params params;
    I2C_Transaction i2cTrans;

    I2C_Params_init(&params);     // sets custom to NULL
    params.transferMode = I2C_MODE_BLOCKING;
    I2CCC26XX_I2CPinCfg pinCfg;
    pinCfg.pinSDA = Board_I2C0_SDA0;
    pinCfg.pinSCL = Board_I2C0_SCL0;
    params.custom = &pinCfg;

    uint8_t rxBuf[32];
    uint8_t txBuf[32];



    while(1){
        handle = I2C_open(Board_I2C, &params);
        if(!handle) {
            //error opening I2C
            //PIN_setOutputValue(ledPinHandle, LED_PIN_RX, true);
        }

        i2cTrans.slaveAddress = 0x69;
        i2cTrans.writeBuf = txBuf;
        i2cTrans.writeCount = 2;
        i2cTrans.readBuf = rxBuf;
        i2cTrans.readCount = 0;
        txBuf[0] = 0x02;
        txBuf[1] = 0x01;

        bool status = false;
        status = I2C_transfer(handle, &i2cTrans);
        PIN_setOutputValue(ledPinHandle, LED_PIN_TX, false);
        PIN_setOutputValue(ledPinHandle, LED_PIN_RX, false);
        if(!status) {
            PIN_setOutputValue(ledPinHandle, LED_PIN_TX, true);
        }
        else {
            PIN_setOutputValue(ledPinHandle, LED_PIN_RX, true);
        }
        I2C_close(handle);
        DELAY_MS(10000);
    }

}


/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      grideyeService_createTask
 *
 * @brief   Task creation function for the grideye application.
 *
 * @param   None.
 *
 * @return  None.
 */
void grideyeService_createTask(void)
{
  Task_Params taskParams;

  // Configure task
  Task_Params_init(&taskParams);
  taskParams.stack = grideyeTaskStack;
  taskParams.stackSize = GRIDEYE_TASK_STACK_SIZE;
  //taskParams.priority = GRIDEYE_TASK_PRIORITY;

  Task_construct(&grideyeTask, grideye_taskFxn, &taskParams, NULL);
}



#endif /* SERVICES_GRIDEYESERVICE_C_ */
