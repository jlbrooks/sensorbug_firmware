
/* XDCtools Header files */

#include <Apps/sensorbugBasicDrivers/bme680.h>
#include <Apps/sensorbugBasicDrivers/Commissioning.h>
#include <Apps/sensorbugBasicDrivers/pb_decode.h>
#include <Apps/sensorbugBasicDrivers/pb_encode.h>
#include <Apps/sensorbugBasicDrivers/sensorbug.pb.h>
#include <xdc/std.h>
#include <xdc/runtime/System.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>

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
#include <math.h>

#include "board.h"
#include "io.h"

#include "LoRaMac.h"

#include "Services/grideyeService.h"
#include "Services/bmeService.h"
#include "Services/bmxService.h"
#include "Services/lightService.h"
#include "Services/pcService.h"

#include <ti/drivers/I2C.h>
#include <ti/drivers/i2c/I2CCC26XX.h>

#define TASKSTACKSIZE   2048

Task_Struct task0Struct;
Char task0Stack[TASKSTACKSIZE];


/*------------------------------------------------------------------------*/
/*                      Start of LoRaWan Demo Code                        */
/*------------------------------------------------------------------------*/

/*!
 * Defines the application data transmission duty cycle. 15s, value in [ms].
 */
#define APP_TX_DUTYCYCLE                            3000

/*!
 * Defines a random delay for application data transmission duty cycle. 2s,
 * value in [ms].
 */
#define APP_TX_DUTYCYCLE_RND                        1000

/*!
 * Default datarate
 */
//#define LORAWAN_DEFAULT_DATARATE                    DR_0
#define LORAWAN_DEFAULT_DATARATE                    DR_4

/*!
 * LoRaWAN confirmed messages
 */
#define LORAWAN_CONFIRMED_MSG_ON                    false

/*!
 * LoRaWAN Adaptive Data Rate
 *
 * \remark Please note that when ADR is enabled the end-device should be static
 */
#define LORAWAN_ADR_ON                              0

/*!
 * LoRaWAN application port
 */
#define LORAWAN_APP_PORT                            2

/*!
 * User application data buffer size BAND_915
 */
#define LORAWAN_APP_DATA_SIZE                       11

static uint8_t DevEui[] = LORAWAN_DEVICE_EUI;
static uint8_t AppEui[] = LORAWAN_APPLICATION_EUI;
static uint8_t AppKey[] = LORAWAN_APPLICATION_KEY;

#if( OVER_THE_AIR_ACTIVATION == 0 )

static uint8_t NwkSKey[] = LORAWAN_NWKSKEY;
static uint8_t AppSKey[] = LORAWAN_APPSKEY;

/*!
 * Device address
 */
static uint32_t DevAddr = LORAWAN_DEVICE_ADDRESS;

#endif

/*!
 * Application port
 */
static uint8_t AppPort = LORAWAN_APP_PORT;

/*!
 * User application data size
 */
static uint8_t AppDataSize = LORAWAN_APP_DATA_SIZE;

/*!
 * User application data buffer size
 */
#define LORAWAN_APP_DATA_MAX_SIZE                           64

/*!
 * User application data
 */
static uint8_t AppData[LORAWAN_APP_DATA_MAX_SIZE];

/*!
 * Indicates if the node is sending confirmed or unconfirmed messages
 */
static uint8_t IsTxConfirmed = LORAWAN_CONFIRMED_MSG_ON;

/*!
 * Defines the application data transmission duty cycle
 */
static uint32_t TxDutyCycleTime;

/*!
 * Timer to handle the application data transmission duty cycle
 */
static TimerEvent_t TxNextPacketTimer;

/*!
 * Specifies the state of the application LED
 */
static bool AppLedStateOn = false;

/*!
 * Timer to handle the state of LED1
 */
static TimerEvent_t Led1Timer;

/*!
 * Timer to handle the state of LED2
 */
static TimerEvent_t Led2Timer;

/*!
 * Timer to handle the state of LED4
 */
static TimerEvent_t Led4Timer;

/*!
 * Indicates if a new packet can be sent
 */
static bool NextTx = true;

/*!
 * Device states
 */
static enum eDeviceState
{
    DEVICE_STATE_INIT,
    DEVICE_STATE_JOIN,
    DEVICE_STATE_SEND,
    DEVICE_STATE_CYCLE,
    DEVICE_STATE_SLEEP
}DeviceState;

/*!
 * LoRaWAN compliance tests support data
 */
struct ComplianceTest_s
{
    bool Running;
    uint8_t State;
    bool IsTxConfirmed;
    uint8_t AppPort;
    uint8_t AppDataSize;
    uint8_t *AppDataBuffer;
    uint16_t DownLinkCounter;
    bool LinkCheck;
    uint8_t DemodMargin;
    uint8_t NbGateways;
}ComplianceTest;


static int8_t user_i2c_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len)
{
    //uartputs("i2c read called \r\n");
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
        uartputs("Read opening failed \r\n");
    }
    else {
        i2cTrans1.slaveAddress = dev_id;
        i2cTrans1.writeBuf = &reg_addr;
        i2cTrans1.writeCount = 1;
        i2cTrans1.readBuf = reg_data;
        i2cTrans1.readCount = len;


        //uartprintf("The provided slave address is : %x\r\n", dev_id);
        //uartprintf("The register address being written is : %d \r\n", reg_addr);

        bool status = false;
        status = I2C_transfer(handle, &i2cTrans1);
        setLed(Board_RLED, false);
        setLed(Board_GLED, false);
        if(!status) {
            setLed(Board_RLED, true);
            rslt = -1;
            uartprintf("The read failed\r\n");
        }
        else {
            setLed(Board_GLED, true);
        }

        I2C_close(handle);
    }
    //uartprintf("Done reading with result : %d \r\n", rslt);
    return rslt;
}



static int8_t user_i2c_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len)
{
    //uartputs("i2c write called \r\n");
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


    //uartprintf("The provided slave address is : %x\r\n", dev_id);
    //uartprintf("The register address being written is : %d \r\n", reg_addr);

    handle = I2C_open(Board_I2C, &params);
    if(!handle) {
        rslt = -1;
        uartputs("Write opening failed \r\n");
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
            uartputs("The transfer failed \r\n");
            setLed(Board_RLED, true);
        }
        else {
            //uartputs("The transfer was successful \r\n");
            setLed(Board_GLED, true);
        }
        free(txBuf);
        I2C_close(handle);
    }
    //uartprintf("Done writing with result : %d \r\n", rslt);
    return rslt;
}

void user_delay_ms(uint32_t period)
{
    DELAY_MS(period);
}

/*!
 * \brief   Prepares the payload of the frame
 */
static void PrepareTxFrame( uint8_t port )
{
    static uint32_t counter = 0;
    uint32_t batteryVoltage = 0;
    uint8_t batteryLevel = 0;

    size_t message_length;
    bool status;

    printf("# PrepareTxFrame\n");

    switch( port )
    {
    case 2:

        batteryVoltage = BoardGetBatteryVoltage();
        batteryLevel = BoardGetBatteryLevel();

        memset(AppData, '\0', sizeof(AppData));

        // Copy Counter
        memcpy(AppData, &counter, sizeof(counter));
        AppDataSize = sizeof(counter);
        counter++;

        // Copy Battery Voltage
        memcpy(AppData + AppDataSize, &batteryVoltage, sizeof(batteryVoltage));
        AppDataSize += sizeof(batteryVoltage);

        // Copy Battery Level
        memcpy(AppData + AppDataSize, &batteryLevel, sizeof(batteryLevel));
        AppDataSize += sizeof(batteryLevel);

        //Insert your driver code here

        //Getting MIC reading

        setPin(Board_HDR_HDIO1, true);
        //DELAY_MS(100);
        ADC_Handle   adc, adc1;
        ADC_Params   params, params1;
        int_fast16_t res, res1;

        ADC_Params_init(&params);
        adc = ADC_open(2, &params);

        if (adc == NULL) {
            uartputs("Error initializing ADC channel \r\n");
            DELAY_MS(100);
            System_abort("Error initializing ADC channel \n");
        }
        else {
            uartputs("ADC channel initialized\r\n");

        }


        uint16_t adcValue0, adcValue1;
        uint16_t minV, maxV;

        minV = 0xFFFF;
        maxV = 0;

        uint32_t currTicks, startTicks;

        startTicks = Clock_getTicks();
        currTicks = startTicks;

        while((currTicks - startTicks) < 5000) {
            currTicks = Clock_getTicks();
            res = ADC_convert(adc, &adcValue0);
            if (res == ADC_STATUS_SUCCESS) {
                if(maxV < adcValue0)
                    maxV = adcValue0;
                if(minV > adcValue0)
                    minV = adcValue0;
            }
            else {
                uartprintf("ADC channel convert failed \r\n");
            }

        }
        ADC_close(adc);

        ADC_Params_init(&params1);
        adc = ADC_open(0, &params1);

        if (adc == NULL) {
            uartputs("Error initializing ADC channel \r\n");
            DELAY_MS(100);
            System_abort("Error initializing ADC channel \n");
        }
        else {
            uartputs("ADC channel initialized\r\n");

        }


        startTicks = Clock_getTicks();
        currTicks = startTicks;
        uint32_t lightAvg = 0, count = 0;


        while((currTicks - startTicks) < 5000) {
            currTicks = Clock_getTicks();
            res = ADC_convert(adc, &adcValue1);
            if (res == ADC_STATUS_SUCCESS) {
                lightAvg += adcValue1;
                count++;
            }
            else {
                uartprintf("ADC channel convert failed \r\n");
            }

        }
        ADC_close(adc);
        lightAvg = lightAvg/count;

        uint32_t pir_status;

        //Get PIR status
        startTicks = Clock_getTicks();
        currTicks = startTicks;
        while((currTicks - startTicks) < 5000){
            currTicks = Clock_getTicks();
            pir_status = getPin(Board_HDR_ADIO6);
            if(pir_status == 1)
                break;
        }

        //Get BME readings
        struct bme680_dev gas_sensor;

        //Configuring I2C communication interface
        gas_sensor.dev_id = BME680_I2C_ADDR_SECONDARY;
        gas_sensor.intf = BME680_I2C_INTF;
        gas_sensor.read = user_i2c_read;
        gas_sensor.write = user_i2c_write;
        gas_sensor.delay_ms = user_delay_ms;


        //Initializing object to default values
        int8_t rslt = BME680_OK;
        rslt = bme680_init(&gas_sensor);
        if(rslt != BME680_OK)
            uartprintf("Initialization failed with error code : %d \r\n", rslt);

        uint8_t set_required_settings;

        /* Set the temperature, pressure and humidity settings */
        gas_sensor.tph_sett.os_hum = BME680_OS_2X;
        gas_sensor.tph_sett.os_pres = BME680_OS_4X;
        gas_sensor.tph_sett.os_temp = BME680_OS_8X;
        gas_sensor.tph_sett.filter = BME680_FILTER_SIZE_3;

        /* Set the remaining gas sensor settings and link the heating profile */
        gas_sensor.gas_sett.run_gas = BME680_ENABLE_GAS_MEAS;
        /* Create a ramp heat waveform in 3 steps */
        gas_sensor.gas_sett.heatr_temp = 320; /* degree Celsius */
        gas_sensor.gas_sett.heatr_dur = 150; /* milliseconds */

        /* Select the power mode */
        /* Must be set before writing the sensor configuration */
        gas_sensor.power_mode = BME680_FORCED_MODE;

        /* Set the required sensor settings needed */
        set_required_settings = BME680_OST_SEL | BME680_OSP_SEL | BME680_OSH_SEL | BME680_FILTER_SEL
            | BME680_GAS_SENSOR_SEL;

        /* Set the desired sensor configuration */
        rslt = bme680_set_sensor_settings(set_required_settings,&gas_sensor);
        if(rslt != 0) {
            setLed(Board_RLED, true);
            uartprintf("sensor settings failed with error code : %d\r\n", rslt);
        }


        /* Set the power mode */
        rslt = bme680_set_sensor_mode(&gas_sensor);
        if(rslt != 0) {
            setLed(Board_RLED, true);
            uartprintf("power mode setting failed with error code : %d \r\n", rslt);
        }


        /* Get the total measurement duration so as to sleep or wait till the
         * measurement is complete */
        uint16_t meas_period;
        bme680_get_profile_dur(&meas_period, &gas_sensor);
        DELAY_MS(meas_period); /* Delay till the measurement is ready */

        struct bme680_field_data data;
//        while(1) {
//            rslt = bme680_get_sensor_data(&data, &gas_sensor);
//            if(rslt != 0) {
//                setLed(Board_RLED, true);
//                uartprintf("Getting sensor data failed with error code : %d \r\n", rslt);
//            }
//
//            uartprintf("T: %.2f degC, P: %.2f hPa, H %.2f %%rH ", data.temperature / 100.0f,
//                data.pressure / 100.0f, data.humidity / 1000.0f );
//            /* Avoid using measurements from an unstable heating setup */
//            if(data.status & BME680_HEAT_STAB_MSK)
//                uartprintf(", G: %d ohms", data.gas_resistance);
//
//            uartprintf("\r\n");
//            if(!rslt)
//                break;
//        }

        //Prepare sensor readings to send over LoRa
        SensorMessage message = SensorMessage_init_zero;

        pb_ostream_t stream = pb_ostream_from_buffer(AppData, sizeof(AppData));

        message.has_mic = true;
        message.mic = maxV - minV;

        message.has_pir_status = true;
        message.pir_status = pir_status;

        message.has_light = true;
        message.light = lightAvg;

        uint32_t bmxData[20];
        getBmxData(bmxData);

        message.has_accelz = true;
        message.accelz = (bmxData[19] << 8) | bmxData[18];

        message.has_accely = true;
        message.accely = (bmxData[17] << 8) | bmxData[16];

        message.has_accelx = true;
        message.accelx = (bmxData[15] << 8) | bmxData[14];

        message.has_gyrz = true;
        message.gyrz = (bmxData[13] << 8) | bmxData[12];

        message.has_gyry = true;
        message.gyry = (bmxData[11] << 8) | bmxData[10];

        message.has_gyrx = true;
        message.gyrx = (bmxData[9] << 8) | bmxData[8];


        message.has_magz = true;
        message.magz = (bmxData[5] << 8) | bmxData[4];

        message.has_magy = true;
        message.magy = (bmxData[3] << 8) | bmxData[2];

        message.has_magx = true;
        message.magx = (bmxData[1] << 8) | bmxData[0];


        status = pb_encode(&stream, SensorMessage_fields, &message);
        message_length = stream.bytes_written;

        AppDataSize = message_length;

        if(!status) {
            printf("Encoding failed: %s\n", PB_GET_ERROR(&stream));
        }


        break;

    case 224:
        if( ComplianceTest.LinkCheck == true )
        {
            ComplianceTest.LinkCheck = false;
            AppDataSize = 3;
            AppData[0] = 5;
            AppData[1] = ComplianceTest.DemodMargin;
            AppData[2] = ComplianceTest.NbGateways;
            ComplianceTest.State = 1;
        }
        else
        {
            switch( ComplianceTest.State )
            {
            case 4:
                ComplianceTest.State = 1;
                break;
            case 1:
                AppDataSize = 2;
                AppData[0] = ComplianceTest.DownLinkCounter >> 8;
                AppData[1] = ComplianceTest.DownLinkCounter;
                break;
            }
        }
        break;
    default:
        break;
    }
}

/*!
 * \brief   Prepares the payload of the frame
 *
 * \retval  [0: frame could be send, 1: error]
 */
static bool SendFrame( void )
{
    McpsReq_t mcpsReq;
    LoRaMacTxInfo_t txInfo;

    printf("# SendFrame\n");

    if( LoRaMacQueryTxPossible( AppDataSize, &txInfo ) != LORAMAC_STATUS_OK )
    {
        // Send empty frame in order to flush MAC commands
        mcpsReq.Type = MCPS_UNCONFIRMED;
        mcpsReq.Req.Unconfirmed.fBuffer = NULL;
        mcpsReq.Req.Unconfirmed.fBufferSize = 0;
        mcpsReq.Req.Unconfirmed.Datarate = LORAWAN_DEFAULT_DATARATE;
    }
    else
    {
        if( IsTxConfirmed == false )
        {
            mcpsReq.Type = MCPS_UNCONFIRMED;
            mcpsReq.Req.Unconfirmed.fPort = AppPort;
            mcpsReq.Req.Unconfirmed.fBuffer = AppData;
            mcpsReq.Req.Unconfirmed.fBufferSize = AppDataSize;
            mcpsReq.Req.Unconfirmed.Datarate = LORAWAN_DEFAULT_DATARATE;
        }
        else
        {
            mcpsReq.Type = MCPS_CONFIRMED;
            mcpsReq.Req.Confirmed.fPort = AppPort;
            mcpsReq.Req.Confirmed.fBuffer = AppData;
            mcpsReq.Req.Confirmed.fBufferSize = AppDataSize;
            mcpsReq.Req.Confirmed.NbTrials = 8;
            mcpsReq.Req.Confirmed.Datarate = LORAWAN_DEFAULT_DATARATE;
        }
    }

    if( LoRaMacMcpsRequest( &mcpsReq ) == LORAMAC_STATUS_OK )
    {
        return false;
    }
    return true;
}

/*!
 * \brief Function executed on TxNextPacket Timeout event
 */
static void OnTxNextPacketTimerEvent( void )
{
    printf("# OnTxNextPacketTimerEvent\n");
    MibRequestConfirm_t mibReq;
    LoRaMacStatus_t status;

    TimerStop( &TxNextPacketTimer );

    mibReq.Type = MIB_NETWORK_JOINED;
    status = LoRaMacMibGetRequestConfirm( &mibReq );

    if( status == LORAMAC_STATUS_OK )
    {
        if( mibReq.Param.IsNetworkJoined == true )
        {
            DeviceState = DEVICE_STATE_SEND;
            NextTx = true;
        }
        else
        {
            DeviceState = DEVICE_STATE_JOIN;
        }
    }
}

/*!
 * \brief Function executed on Led 1 Timeout event
 */
static void OnLed1TimerEvent( void )
{
    TimerStop( &Led1Timer );
    // Switch LED 1 OFF
//    GpioWrite( &Led1, 1 );
    setLed(Board_GLED, 0);
}

/*!
 * \brief Function executed on Led 2 Timeout event
 */
static void OnLed2TimerEvent( void )
{
    TimerStop( &Led2Timer );
    // Switch LED 2 OFF
//    GpioWrite( &Led2, 1 );
    setLed(Board_RLED, 0);
}

/*!
 * \brief Function executed on Led 4 Timeout event
 */
static void OnLed4TimerEvent( void )
{
    TimerStop( &Led4Timer );
    // Switch LED 4 OFF
//    GpioWrite( &Led4, 1 );
}

/*!
 * \brief   MCPS-Confirm event function
 *
 * \param   [IN] mcpsConfirm - Pointer to the confirm structure,
 *               containing confirm attributes.
 */
static void McpsConfirm( McpsConfirm_t *mcpsConfirm )
{
    printf("# McpsConfirm\n");
    uartputs("# McpsConfirm\n");
    if( mcpsConfirm->Status == LORAMAC_EVENT_INFO_STATUS_OK )
    {
        switch( mcpsConfirm->McpsRequest )
        {
            case MCPS_UNCONFIRMED:
            {
                // Check Datarate
                // Check TxPower
                uartputs("# Got McpsConfirm: MCPS_UNCONFIRMED\n");
                break;
            }
            case MCPS_CONFIRMED:
            {
                // Check Datarate
                // Check TxPower
                // Check AckReceived
                // Check NbTrials
                uartputs("# Got McpsConfirm: MCPS_CONFIRMED\n");
                break;
            }
            case MCPS_PROPRIETARY:
            {
                break;
            }
            default:
                break;
        }

        // Switch LED 1 ON
//        GpioWrite( &Led1, 0 );
        setLed(Board_GLED, 1);
        TimerStart( &Led1Timer );
    }
    NextTx = true;
}

/*!
 * \brief   MCPS-Indication event function
 *
 * \param   [IN] mcpsIndication - Pointer to the indication structure,
 *               containing indication attributes.
 */
static void McpsIndication( McpsIndication_t *mcpsIndication )
{
    printf("# McpsIndication\n");
    if( mcpsIndication->Status != LORAMAC_EVENT_INFO_STATUS_OK )
    {
        return;
    }

    switch( mcpsIndication->McpsIndication )
    {
        case MCPS_UNCONFIRMED:
        {
            uartputs("# Got McpsIndication: MCPS_UNCONFIRMED\n");
            break;
        }
        case MCPS_CONFIRMED:
        {
            uartputs("# Got McpsIndication: MCPS_CONFIRMED\n");
            break;
        }
        case MCPS_PROPRIETARY:
        {
            break;
        }
        case MCPS_MULTICAST:
        {
            break;
        }
        default:
            break;
    }

    // Check Multicast
    // Check Port
    // Check Datarate
    // Check FramePending
    // Check Buffer
    // Check BufferSize
    // Check Rssi
    // Check Snr
    // Check RxSlot

    if( ComplianceTest.Running == true )
    {
        ComplianceTest.DownLinkCounter++;
    }

    if( mcpsIndication->RxData == true )
    {
        switch( mcpsIndication->Port )
        {
        case 1: // The application LED can be controlled on port 1 or 2
        case 2:
            if( mcpsIndication->BufferSize == 1 )
            {
                AppLedStateOn = mcpsIndication->Buffer[0] & 0x01;
//                GpioWrite( &Led3, ( ( AppLedStateOn & 0x01 ) != 0 ) ? 0 : 1 );
                setLed(Board_RLED, ( ( AppLedStateOn & 0x01 ) != 0 ) ? 1 : 0);
            }
            break;
        case 224:
            if( ComplianceTest.Running == false )
            {
                // Check compliance test enable command (i)
                if( ( mcpsIndication->BufferSize == 4 ) &&
                    ( mcpsIndication->Buffer[0] == 0x01 ) &&
                    ( mcpsIndication->Buffer[1] == 0x01 ) &&
                    ( mcpsIndication->Buffer[2] == 0x01 ) &&
                    ( mcpsIndication->Buffer[3] == 0x01 ) )
                {
                    IsTxConfirmed = false;
                    AppPort = 224;
                    AppDataSize = 2;
                    ComplianceTest.DownLinkCounter = 0;
                    ComplianceTest.LinkCheck = false;
                    ComplianceTest.DemodMargin = 0;
                    ComplianceTest.NbGateways = 0;
                    ComplianceTest.Running = true;
                    ComplianceTest.State = 1;

                    MibRequestConfirm_t mibReq;
                    mibReq.Type = MIB_ADR;
                    mibReq.Param.AdrEnable = true;
                    LoRaMacMibSetRequestConfirm( &mibReq );
                }
            }
            else
            {
                ComplianceTest.State = mcpsIndication->Buffer[0];
                switch( ComplianceTest.State )
                {
                case 0: // Check compliance test disable command (ii)
                    IsTxConfirmed = LORAWAN_CONFIRMED_MSG_ON;
                    AppPort = LORAWAN_APP_PORT;
                    AppDataSize = LORAWAN_APP_DATA_SIZE;
                    ComplianceTest.DownLinkCounter = 0;
                    ComplianceTest.Running = false;

                    MibRequestConfirm_t mibReq;
                    mibReq.Type = MIB_ADR;
                    mibReq.Param.AdrEnable = LORAWAN_ADR_ON;
                    LoRaMacMibSetRequestConfirm( &mibReq );
                    break;
                case 1: // (iii, iv)
                    AppDataSize = 2;
                    break;
                case 2: // Enable confirmed messages (v)
                    IsTxConfirmed = true;
                    ComplianceTest.State = 1;
                    break;
                case 3:  // Disable confirmed messages (vi)
                    IsTxConfirmed = false;
                    ComplianceTest.State = 1;
                    break;
                case 4: // (vii)
                    AppDataSize = mcpsIndication->BufferSize;

                    AppData[0] = 4;
                    for( uint8_t i = 1; i < AppDataSize; i++ )
                    {
                        AppData[i] = mcpsIndication->Buffer[i] + 1;
                    }
                    break;
                case 5: // (viii)
                    {
                        MlmeReq_t mlmeReq;
                        mlmeReq.Type = MLME_LINK_CHECK;
                        LoRaMacMlmeRequest( &mlmeReq );
                    }
                    break;
                case 6: // (ix)
                    {
                        MlmeReq_t mlmeReq;

                        // Disable TestMode and revert back to normal operation
                        IsTxConfirmed = LORAWAN_CONFIRMED_MSG_ON;
                        AppPort = LORAWAN_APP_PORT;
                        AppDataSize = LORAWAN_APP_DATA_SIZE;
                        ComplianceTest.DownLinkCounter = 0;
                        ComplianceTest.Running = false;

                        MibRequestConfirm_t mibReq;
                        mibReq.Type = MIB_ADR;
                        mibReq.Param.AdrEnable = LORAWAN_ADR_ON;
                        LoRaMacMibSetRequestConfirm( &mibReq );

                        mlmeReq.Type = MLME_JOIN;

                        mlmeReq.Req.Join.DevEui = DevEui;
                        mlmeReq.Req.Join.AppEui = AppEui;
                        mlmeReq.Req.Join.AppKey = AppKey;
                        mlmeReq.Req.Join.NbTrials = 3;

                        LoRaMacMlmeRequest( &mlmeReq );
                        DeviceState = DEVICE_STATE_SLEEP;
                    }
                    break;
                case 7: // (x)
                    {
                        if( mcpsIndication->BufferSize == 3 )
                        {
                            MlmeReq_t mlmeReq;
                            mlmeReq.Type = MLME_TXCW;
                            mlmeReq.Req.TxCw.Timeout = ( uint16_t )( ( mcpsIndication->Buffer[1] << 8 ) | mcpsIndication->Buffer[2] );
                            LoRaMacMlmeRequest( &mlmeReq );
                        }
                        ComplianceTest.State = 1;
                    }
                    break;
                default:
                    break;
                }
            }
            break;
        default:
            break;
        }
    }

    // Switch LED 2 ON for each received downlink
//    GpioWrite( &Led2, 0 );
    setLed(Board_RLED, 1);
    TimerStart( &Led2Timer );
}

/*!
 * \brief   MLME-Confirm event function
 *
 * \param   [IN] mlmeConfirm - Pointer to the confirm structure,
 *               containing confirm attributes.
 */
static void MlmeConfirm( MlmeConfirm_t *mlmeConfirm )
{
    printf("# MlmeConfirm\n");
    switch( mlmeConfirm->MlmeRequest )
    {
        case MLME_JOIN:
        {
            printf("# MlmeConfirm: Join\n");
            if( mlmeConfirm->Status == LORAMAC_EVENT_INFO_STATUS_OK )
            {
                // Status is OK, node has joined the network
                DeviceState = DEVICE_STATE_SEND;
            }
            else
            {
                // Join was not successful. Try to join again
                DeviceState = DEVICE_STATE_JOIN;
            }
            break;
        }
        case MLME_LINK_CHECK:
        {
            printf("# MlmeConfirm: Link Check\n");
            if( mlmeConfirm->Status == LORAMAC_EVENT_INFO_STATUS_OK )
            {
                // Check DemodMargin
                // Check NbGateways
                if( ComplianceTest.Running == true )
                {
                    ComplianceTest.LinkCheck = true;
                    ComplianceTest.DemodMargin = mlmeConfirm->DemodMargin;
                    ComplianceTest.NbGateways = mlmeConfirm->NbGateways;
                }
            }
            break;
        }
        default:
            break;
    }
    NextTx = true;
}

void maintask(UArg arg0, UArg arg1)
{
    LoRaMacPrimitives_t LoRaMacPrimitives;
    LoRaMacCallback_t LoRaMacCallbacks;
    MibRequestConfirm_t mibReq;

    BoardInitMcu( );
    BoardInitPeriph( );
    printf("# Board initialized\n");

    DeviceState = DEVICE_STATE_INIT;

    while( 1 )
    {
        switch( DeviceState )
        {
            case DEVICE_STATE_INIT:
            {
                printf("# DeviceState: DEVICE_STATE_INIT\n");
                LoRaMacPrimitives.MacMcpsConfirm = McpsConfirm;
                LoRaMacPrimitives.MacMcpsIndication = McpsIndication;
                LoRaMacPrimitives.MacMlmeConfirm = MlmeConfirm;
                LoRaMacCallbacks.GetBatteryLevel = BoardGetBatteryLevel;
                LoRaMacInitialization( &LoRaMacPrimitives, &LoRaMacCallbacks );

                TimerInit( &TxNextPacketTimer, OnTxNextPacketTimerEvent );

                TimerInit( &Led1Timer, OnLed1TimerEvent );
                TimerSetValue( &Led1Timer, 25 );

                TimerInit( &Led2Timer, OnLed2TimerEvent );
                TimerSetValue( &Led2Timer, 25 );

                TimerInit( &Led4Timer, OnLed4TimerEvent );
                TimerSetValue( &Led4Timer, 25 );

                mibReq.Type = MIB_ADR;
                mibReq.Param.AdrEnable = LORAWAN_ADR_ON;
                LoRaMacMibSetRequestConfirm( &mibReq );

                mibReq.Type = MIB_PUBLIC_NETWORK;
                mibReq.Param.EnablePublicNetwork = LORAWAN_PUBLIC_NETWORK;
                LoRaMacMibSetRequestConfirm( &mibReq );

                DeviceState = DEVICE_STATE_JOIN;
                break;
            }
            case DEVICE_STATE_JOIN:
            {
                printf("# DeviceState: DEVICE_STATE_JOIN\n");
#if( OVER_THE_AIR_ACTIVATION != 0 )
                MlmeReq_t mlmeReq;

                // Initialize LoRaMac device unique ID
//                BoardGetUniqueId( DevEui );

                mlmeReq.Type = MLME_JOIN;

                mlmeReq.Req.Join.DevEui = DevEui;
                mlmeReq.Req.Join.AppEui = AppEui;
                mlmeReq.Req.Join.AppKey = AppKey;
                mlmeReq.Req.Join.NbTrials = 3;

                if( NextTx == true )
                {
                    LoRaMacMlmeRequest( &mlmeReq );
                }
                DeviceState = DEVICE_STATE_SLEEP;
#else
                // Choose a random device address if not already defined in Commissioning.h
                if( DevAddr == 0 )
                {
                    // Random seed initialization
                    srand1( BoardGetRandomSeed( ) );

                    // Choose a random device address
                    DevAddr = randr( 0, 0x01FFFFFF );
                }

                mibReq.Type = MIB_NET_ID;
                mibReq.Param.NetID = LORAWAN_NETWORK_ID;
                LoRaMacMibSetRequestConfirm( &mibReq );

                mibReq.Type = MIB_DEV_ADDR;
                mibReq.Param.DevAddr = DevAddr;
                LoRaMacMibSetRequestConfirm( &mibReq );

                mibReq.Type = MIB_NWK_SKEY;
                mibReq.Param.NwkSKey = NwkSKey;
                LoRaMacMibSetRequestConfirm( &mibReq );

                mibReq.Type = MIB_APP_SKEY;
                mibReq.Param.AppSKey = AppSKey;
                LoRaMacMibSetRequestConfirm( &mibReq );

                mibReq.Type = MIB_NETWORK_JOINED;
                mibReq.Param.IsNetworkJoined = true;
                LoRaMacMibSetRequestConfirm( &mibReq );

                DeviceState = DEVICE_STATE_SEND;
#endif
                break;
            }
            case DEVICE_STATE_SEND:
            {
                printf("# DeviceState: DEVICE_STATE_SEND\n");
                if( NextTx == true )
                {
                    PrepareTxFrame( AppPort );

                    NextTx = SendFrame( );
                }
                if( ComplianceTest.Running == true )
                {
                    // Schedule next packet transmission
                    TxDutyCycleTime = 5000; // 5000 ms
                }
                else
                {
                    // Schedule next packet transmission
                    TxDutyCycleTime = APP_TX_DUTYCYCLE + randr( -APP_TX_DUTYCYCLE_RND, APP_TX_DUTYCYCLE_RND );
                }
                DeviceState = DEVICE_STATE_CYCLE;
                break;
            }
            case DEVICE_STATE_CYCLE:
            {
                printf("# DeviceState: DEVICE_STATE_CYCLE\n");
                DeviceState = DEVICE_STATE_SLEEP;

                // Schedule next packet transmission
                TimerSetValue( &TxNextPacketTimer, TxDutyCycleTime );
                TimerStart( &TxNextPacketTimer );
                break;
            }
            case DEVICE_STATE_SLEEP:
            {
               // printf("# DeviceState: DEVICE_STATE_SLEEP\n");
                // Wake up through events
//                TimerLowPowerHandler( );
                Task_sleep(TIME_MS * 10);
//                Task_yield();
                break;
            }
            default:
            {
                DeviceState = DEVICE_STATE_INIT;
                break;
            }
        }
    }

}

void maintask2(UArg arg0, UArg arg1)
{
    BoardInitMcu( );
    BoardInitPeriph( );
    while (1) {
        Task_yield();
    }
}

/*
 *  ======== main ========
 */
int main(void)


{
    Task_Params taskParams;

    /* Call board init functions */
    Board_initGeneral();
    Board_initI2C();
    Board_initSPI();
    Board_initUART();
    //Board_initWatchdog();
    ADC_init();

    /* Construct heartBeat Task  thread */
    Task_Params_init(&taskParams);
    taskParams.arg0 = 1000000 / Clock_tickPeriod;
    taskParams.stackSize = TASKSTACKSIZE;
    taskParams.stack = &task0Stack;
    Task_construct(&task0Struct, (Task_FuncPtr) maintask2, &taskParams, NULL);

    //bmxService_createTask();
    //lightService_createTask();
    grideyeService_createTask();
    pcService_createTask();

    /* Open and setup pins */
    setuppins();

    /* Open UART */
    setupuart();

    /* Start BIOS */
    BIOS_start();

    return (0);
}

