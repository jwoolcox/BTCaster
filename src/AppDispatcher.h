#ifndef APP_DISPATCHER_H
#define APP_DISPATCHER_H

#include <freertos/FreeRTOS.h>
#include <freertos/FreeRTOSConfig.h>
#include <freertos/queue.h>

#include <esp_log.h>
#include <esp_task_wdt.h>
#include <soc/rtc_wdt.h>

#include "AC101.h"

#include "ESPTask.h"
#include "Dispatching.h"
#include "BTSubsystems.h"

#define DISPATCHER_TAG "APP_DISPATCHER"

class AppDispatcher: public ESPTask
{
public:
    AppDispatcher() { esp_log_level_set(DISPATCHER_TAG, ESP_LOG_DEBUG); }
    void processMessages();

    virtual void instanceCbHandler(void *) override;

    static void setQueueHandle(QueueHandle_t hQueue);
    static bool postMessage(DispatchMessage_t *pMessage);
    static bool postMessage(DispatchTarget_t dispatchTarget, uint16_t dispatchEvent, void* dispatchParam, int dispatchParmaLen);

private:
    BTSubsystems btSubsystems;
    AC101 audioAC101_;
};

#endif
