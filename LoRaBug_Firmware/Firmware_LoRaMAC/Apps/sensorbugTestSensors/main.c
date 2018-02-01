
/* XDCtools Header files */

//#include <Apps/sensorbugBasicDrivers/bme680.h>
//#include <Apps/sensorbugBasicDrivers/Commissioning.h>
//#include <Apps/sensorbugBasicDrivers/pb_decode.h>
//#include <Apps/sensorbugBasicDrivers/pb_encode.h>
//#include <Apps/sensorbugBasicDrivers/sensorbug.pb.h>
#include <xdc/std.h>
#include <xdc/runtime/System.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Event.h>

/* TI-RTOS Header files */
// #include <ti/drivers/I2C.h>
#include <ti/drivers/PIN.h>
#include <ti/drivers/SPI.h>
#include <ti/drivers/UART.h>
#include <ti/drivers/ADC.h>
// #include <ti/drivers/Watchdog.h>

/* Board Header files */
#include "Board_LoRaBUG.h"

#include <string.h> // strlen in uartputs and LoRaWan code

#include "board.h"
#include "io.h"

#include "LoRaMac.h"

//#include "Services/grideyeService.h"
//#include "Services/pcService.h"
#include "Services/sensorTestService.h"


#include <ti/drivers/I2C.h>
#include <ti/drivers/i2c/I2CCC26XX.h>


static int8_t user_i2c_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len)
{
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
    return rslt;
}



static int8_t user_i2c_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len)
{
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
            rslt = -1;
            setLed(Board_RLED, true);
        }
        else {
            setLed(Board_GLED, true);
        }
        free(txBuf);
        I2C_close(handle);
    }
    return rslt;
}

/*
 *  ======== main ========
 */
int main(void)
{
//    Task_Params taskParams;

    /* Call board init functions */
    Board_initGeneral();
    Board_initI2C();
    Board_initSPI();
    Board_initUART();
    //Board_initWatchdog();
    ADC_init();

    sensorTestService_createTask();

    /* Open and setup pins */
    setuppins();

    /* Open UART */
    setupuart();

    /* Start BIOS */
    BIOS_start();

    return (0);
}

