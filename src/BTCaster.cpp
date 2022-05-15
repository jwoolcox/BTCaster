#include <nvs.h>
#include <nvs_flash.h>

#include "BTCaster.h"

#define CTAG "BTCaster"

static QueueHandle_t hAppMsgs_;

BTCaster::BTCaster() {}

void BTCaster::begin()
{
    esp_log_level_set(CTAG, ESP_LOG_DEBUG);

    // esp core initialize
    initESP();

    if ((hAppMsgs_ = xQueueCreate(10, sizeof(DispatchMessage_t))) == NULL)
    {
        ESP_ERROR_CHECK(ESP_FAIL);
    }

    taskApplication_.setQueueHandle(hAppMsgs_);

    BTCaster::postMessage(DispatchTarget_t::APP_CORE, 99, NULL, 0);

    // run application thread
    taskApplication_.begin("BT App Dispatcher", NULL);
  

    while (1) {
        rtc_wdt_feed();
        vTaskDelay(900/portTICK_PERIOD_MS);
    }
}

void BTCaster::end()
{
    taskApplication_.end();
}

esp_err_t BTCaster::initESP()
{
    ESP_LOGI(CTAG, " Initialize NVS..");

    esp_err_t err = ESP_FAIL;
    err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_LOGI(CTAG, " Erase before initialize NVS..");
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }

    return err;
}

bool BTCaster::postMessage(DispatchTarget_t dispatchTarget, uint16_t dispatchEvent, void *dispatchParam, int dispatchParamLen)
{
    return AppDispatcher::postMessage(dispatchTarget, dispatchEvent, dispatchParam, dispatchParamLen);
}