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
 *  ======== lightService.c ========
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
#include "io.h"

#define DELAY_MS(i)    Task_sleep(((i) * 1000) / Clock_tickPeriod)

/*******************************************************************************
 * CONSTANTS
 */
#define LIGHT_TASK_PRIORITY                     5

#define LIGHT_TASK_STACK_SIZE                   1200


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
Task_Struct lightTask;
Char lightTaskStack[LIGHT_TASK_STACK_SIZE];

static PIN_Handle ledPinHandle;
static PIN_State ledPinState;

static PIN_Config ledPinTable[] = {
    Board_GLED | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    Board_RLED | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    Board_HDR_HDIO1 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    PIN_TERMINATE
};

/* ADC conversion result variables */
uint16_t adcValue0;
uint16_t adcValue1[ADC_SAMPLE_COUNT];


/*
 *  ======== taskFxn1 ========
 *  Open a ADC handle and get a array of sampling results after
 *  calling several conversions.
 */
Void light_taskFxn(UArg arg0, UArg arg1)
{
    uint16_t     i;
    ADC_Handle   adc;
    ADC_Params   params;
    int_fast16_t res;

    //PIN_setOutputValue(ledPinHandle, Board_HDR_ADIO0, true);

    ADC_Params_init(&params);
    adc = ADC_open(0, &params);

    if (adc == NULL) {
        uartputs("qwertyuioed\r\n");
        DELAY_MS(2000);
        setLed(Board_RLED, true);
        System_abort("Error initializing ADC channel 1\n");
    }
    else {
        uartputs("ADC channel 1 initialized\r\n");
    }

    while(1) {
        res = ADC_convert(adc, &adcValue1[0]);

        if (res == ADC_STATUS_SUCCESS) {
            uartprintf("ADC channel 1 convert result (%d): 0x%x\r\n", adcValue1[0],
                adcValue1[0]);
            setLed(Board_GLED, true);
            DELAY_MS(2000);

        }
        else {
            uartprintf("ADC channel 1 convert failed (%d)\r\n", i);
            setLed(Board_RLED, true);
        }
        setLed(Board_GLED, false);
        DELAY_MS(2000);
    }

    ADC_close(adc);
}

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      lightService_createTask
 *
 * @brief   Task creation function for the light sensor application.
 *
 * @param   None.
 *
 * @return  None.
 */

void lightService_createTask(void)
{
  Task_Params taskParams;

  // Configure task
  Task_Params_init(&taskParams);
  taskParams.stack = lightTaskStack;
  taskParams.stackSize = LIGHT_TASK_STACK_SIZE;
  //taskParams.priority = LIGHT_TASK_PRIORITY;

  Task_construct(&lightTask, light_taskFxn, &taskParams, NULL);
}


