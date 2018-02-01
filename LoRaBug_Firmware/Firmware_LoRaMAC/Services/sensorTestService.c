/*
 * Copyright (c) 2016, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *  ======== sensorTestService.c ========
 */
/* XDCtools Header files */
#include <xdc/std.h>
#include <xdc/runtime/System.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>

/* Driver Header files */
#include <ti/drivers/ADC.h>
#if defined(CC2650DK_7ID) || defined(CC1310DK_7XD)
#include <ti/drivers/PIN.h>
#endif

/* Example/Board Header files */
#include "Config/Board_LoRaBUG.h"
#include <stdio.h>
#include <stdlib.h>
#include "io.h"
#include "Services/bmeService.h"
#include "Services/bmxService.h"
#include "Services/grideyeService.h"

#include <ti/drivers/I2C.h>
#include <ti/drivers/i2c/I2CCC26XX.h>

#define DELAY_MS(i)    Task_sleep(((i) * 1000) / Clock_tickPeriod)

/*******************************************************************************
 * CONSTANTS
 */
#define SENSORTEST_TASK_PRIORITY                     5

#define SENSORTEST_TASK_STACK_SIZE                   2000


#define LED_PIN_RX      Board_GLED
#define LED_PIN_TX      Board_RLED

/* ADC sample count */
#define ADC_SAMPLE_COUNT  (10)


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
Task_Struct sensorTestTask;
Char sensorTestTaskStack[SENSORTEST_TASK_STACK_SIZE];

/*
 *  ======== taskFxn1 ========
 *  Open a ADC handle and get a array of sampling results after
 *  calling several conversions.
 */
Void sensorTest_taskFxn(UArg arg0, UArg arg1)
{
    setLed(Board_GLED, false);
    setLed(Board_RLED, false);
    //Try and get data from BMX. If anything fails, will assert red LED
    uint32_t data;
    uartputs("Going to test BMX chip now\r\n");
    getBmxData(&data);


    //Try and get data from BME. If anything fails, assert red LED
    uartputs("Going to test BME chip now\r\n");
    //Get BME readings
    testBMEChip();

    //Test the GridEye chip. If anything fails, assert red LED
    grideye_init();

    //Assert the green LED to signal completion of tests
//    setLed(Board_GLED, true);

}

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      sensorTestService_createTask
 *
 * @brief   Task creation function for the light sensor application.
 *
 * @param   None.
 *
 * @return  None.
 */

void sensorTestService_createTask(void)
{
  Task_Params taskParams;

  // Configure task
  Task_Params_init(&taskParams);
  taskParams.stack = sensorTestTaskStack;
  taskParams.stackSize = SENSORTEST_TASK_STACK_SIZE;
  //taskParams.priority = LIGHT_TASK_PRIORITY;

  Task_construct(&sensorTestTask, sensorTest_taskFxn, &taskParams, NULL);
}


