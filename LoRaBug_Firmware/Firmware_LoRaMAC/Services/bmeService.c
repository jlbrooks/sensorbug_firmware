/******************************************************************************

  @file  bmeService.c

 @brief This file contains a simple I2C transaction with the bme680
         Created on: Jul 11, 2017

 @author: Abhinand Sukumar

 ******************************************************************************/

#ifndef SERVICES_BMESERVICE_C_
#define SERVICES_BMESERVICE_C_

/*********************************************************************
 * INCLUDES
 */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include <xdc/runtime/System.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>
#include <io.h>


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
#define BME_TASK_PRIORITY                     5

#define BME_TASK_STACK_SIZE                   1200


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
Task_Struct bmeTask;
Char bmeTaskStack[BME_TASK_STACK_SIZE];


static int8_t user_i2c_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len)
{
    uartputs("i2c read called \r\n");
    int8_t rslt = 0; //0 for success, non-zero for failure
    I2C_Handle handle;
    I2C_Params params;
    I2C_Transaction i2cTrans1;

    I2C_Params_init(&params);     // sets custom to NULL
    params.transferMode = I2C_MODE_BLOCKING;
    I2CCC26XX_I2CPinCfg pinCfg;
    pinCfg.pinSDA = Board_I2C0_SDA0;
    pinCfg.pinSCL = Board_I2C0_SCL0;
    params.custom = &pinCfg;


    handle = I2C_open(Board_I2C, &params);
    if(!handle) {
        rslt = -1;
    }
    else {
        i2cTrans1.slaveAddress = dev_id;
        i2cTrans1.writeBuf = &reg_addr;
        i2cTrans1.writeCount = 1;
        i2cTrans1.readBuf = reg_data;
        i2cTrans1.readCount = len;


        uartprintf("The provided slave address is : %x\r\n", dev_id);
        uartprintf("The register address being written is : %d \r\n", reg_addr);

        bool status = false;
        status = I2C_transfer(handle, &i2cTrans1);
        setLed(Board_RLED, false);
        setLed(Board_GLED, false);
        if(!status) {
            setLed(Board_RLED, true);
            rslt = -1;
        }
        else {
            setLed(Board_GLED, true);
        }

        I2C_close(handle);
    }
    uartprintf("Done reading with result : %d \r\n", rslt);
    return rslt;
}



static int8_t user_i2c_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len)
{
    uartputs("i2c write called \r\n");
    int8_t rslt = 0; //0 for success, non-zero for failure
    I2C_Handle handle;
    I2C_Params params;
    I2C_Transaction i2cTrans1;

    I2C_Params_init(&params);     // sets custom to NULL
    params.transferMode = I2C_MODE_BLOCKING;
    I2CCC26XX_I2CPinCfg pinCfg;
    pinCfg.pinSDA = Board_I2C0_SDA0;
    pinCfg.pinSCL = Board_I2C0_SCL0;
    params.custom = &pinCfg;


    uartprintf("The provided slave address is : %x\r\n", dev_id);
    uartprintf("The register address being written is : %d \r\n", reg_addr);

    handle = I2C_open(Board_I2C, &params);
    if(!handle) {
        rslt = -1;
        uartputs("Opening failed \r\n");
    }
    else {
        uint8_t *txBuf = malloc((len + 1) * sizeof(uint8_t));
        i2cTrans1.slaveAddress = dev_id;
        i2cTrans1.writeBuf = txBuf;
        i2cTrans1.writeCount = len;
        i2cTrans1.readBuf = NULL;
        i2cTrans1.readCount = 0;
        txBuf[0] = reg_addr;
        for(int j = 0; j < len; j++)
        {
            txBuf[1 + j] = reg_data[j];
        }

        bool status = false;
        status = I2C_transfer(handle, &i2cTrans1);
        setLed(Board_RLED, false);
        setLed(Board_GLED, false);
        if(!status) {
            uartputs("The transfer failed \r\n");
            setLed(Board_RLED, true);
        }
        else {
            uartputs("The transfer was successful \r\n");
            setLed(Board_GLED, true);
        }
        free(txBuf);
        I2C_close(handle);
    }
    uartprintf("Done writing with result : %d \r\n", rslt);
    return rslt;
}

/*********************************************************************
 * @fn      bmx_taskFxn
 * @return  None.
 */
static void bme_taskFxn (UArg a0, UArg a1)
{
    while(1){
        uint8_t txBuf[2];
        uint8_t rxBuf[2];
        txBuf[0] = 0xFF;
        user_i2c_write((uint8_t)0x77, (uint8_t)0xD0, txBuf, 1);
        user_i2c_read((uint8_t)0x77, (uint8_t)0x1F, rxBuf, 1);
        DELAY_MS(3000);
    }

}


/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      bmeService_createTask
 *
 * @brief   Task creation function for the bme680 application.
 *
 * @param   None.
 *
 * @return  None.
 */
void bmeService_createTask(void)
{
  Task_Params taskParams;

  // Configure task
  Task_Params_init(&taskParams);
  taskParams.stack = bmeTaskStack;
  taskParams.stackSize = BME_TASK_STACK_SIZE;
  //taskParams.priority = BME_TASK_PRIORITY;

  Task_construct(&bmeTask, bme_taskFxn, &taskParams, NULL);
}



#endif /* SERVICES_BMESERVICE_C_ */
