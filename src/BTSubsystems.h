#ifndef BT_SUBSYSTEMS_H
#define BT_SUBSYSTEMS_H

#include <esp_bt.h>
#include <esp_bt_main.h>
#include <esp_bt_device.h>
#include <esp_gap_bt_api.h>
#include <esp_a2dp_api.h>
#include "I2STask.h"

#define BTSUBSYSTEMS_TAG "BTSUBSYSTEMS"

class BTSubsystems
{
public:
    const char *A2SStateStrings[4] = {"Disconnected", "Connecting", "Connected", "Disconnecting"};    
    const char *A2DAudioStateStrings[3] = {"Suspended", "Stopped", "Started"};

    BTSubsystems() { }
    esp_err_t initializeBT();
    void a2dCallback(uint16_t event, void *param);
    void gapCallback(uint16_t event, void *param);

private:

    bool btInitialized_;
    I2STask taskI2SFeed_;

    void handleA2DConnectionStates(esp_a2d_cb_param_t::a2d_conn_stat_param StateParam);
    void beginI2SProcessing();
    void endI2SProcessing();
    static void installI2SDriver();
    static void uninstallI2SDriver();
};

#endif
