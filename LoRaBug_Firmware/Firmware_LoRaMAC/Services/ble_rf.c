/*
 * ble_rf.c
 *
 *  Created on: Nov 22, 2017
 *      Author: Jacob Brooks
 */

#include <ti/drivers/rf/RF.h>

#include "ble_rf.h"

void send_advertisement() {
    RF_Object ble_rfObj;
    RF_Handle ble_handle;
    RF_Params ble_params;

    RF_Mode RF_ble =
    {
        .rfMode      = RF_MODE_MULTIPLE,
        .cpePatchFxn = &rf_patch_cpe_ble,
        .mcePatchFxn = 0,
        .rfePatchFxn = &rf_patch_rfe_ble,
    };

    RF_Params_init(&ble_params);
    ble_params.nInactivityTimeout = 200;

    ble_handle = RF_open(ble_rfObj, &RF_ble, (RF_RadioSetup*)&RF_cmdRadioSetup, &ble_params);
}
