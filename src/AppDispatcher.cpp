#include "AppDispatcher.h"
#include <esp_log.h>
#include <string.h>

static QueueHandle_t hAppMsgs_;

void AppDispatcher::setQueueHandle(QueueHandle_t hQueue)
{
    hAppMsgs_ = hQueue;
}

void AppDispatcher::processMessages()
{
    DispatchMessage_t msg;

    if (pdTRUE == xQueueReceive(hAppMsgs_, &msg, (TickType_t)portMAX_DELAY))
    {
        switch (msg.eventTarget)
        {
        case DispatchTarget_t::APP_CORE:
            switch (msg.event)
            {
            case 99:
                ESP_LOGW(DISPATCHER_TAG, "Application startup proceeding");
                btSubsystems.initializeBT();
                audioAC101_.begin();
                break;
            default:
                ESP_LOGW(DISPATCHER_TAG, "Unhandled message to APP CORE [%d]", msg.event);
                break;
            }
            break;
        case DispatchTarget_t::BT_GAP:
            btSubsystems.gapCallback(msg.event, msg.param);
            break;
        case DispatchTarget_t::AVRC:
            // avrcHandler_.callback(msg.event, msg.param);
            break;
        case DispatchTarget_t::A2DP:
            btSubsystems.a2dCallback(msg.event, msg.param);
            break;
        case DispatchTarget_t::DAC:
            audioAC101_.callback(msg.event, msg.param);
            break;
        }

        if (NULL != msg.param)
            free(msg.param);
    }
}

void AppDispatcher::instanceCbHandler(void *pCbParam)
{
    for (;;)
    {
        processMessages();
    }
}

bool AppDispatcher::postMessage(DispatchMessage_t *pMessage)
{
    bool result = false;
    
    ESP_LOGD(DISPATCHER_TAG, "%s, Target: 0x%x, Event: 0x%x", __func__, pMessage->eventTarget, pMessage->event);

    if (NULL != pMessage)
    {
        if (xQueueSend(hAppMsgs_, pMessage, 10 / portTICK_PERIOD_MS) != pdTRUE)
        {
            ESP_LOGE(DISPATCHER_TAG, "xQueueSend failed");
            result = false;
        }
    }

    return result;
}

bool AppDispatcher::postMessage(DispatchTarget_t dispatchTarget, uint16_t dispatchEvent, void *dispatchParam, int dispatchParamLen)
{
    bool result = false;
    DispatchMessage_t msg;

    msg.eventTarget = dispatchTarget;
    msg.event = dispatchEvent;
    msg.param = NULL;

    if (dispatchParam && dispatchParamLen > 0)
    {
        if ((msg.param = malloc(dispatchParamLen)) != NULL)
        {
            // no deep copy
            memcpy(msg.param, dispatchParam, dispatchParamLen);
            result = postMessage(&msg);
        }
    } else {
        result = postMessage(&msg);
    }

    return result;
}