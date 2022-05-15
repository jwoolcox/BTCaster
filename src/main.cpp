#include <esp_log.h>

#include <freertos/FreeRTOS.h>
#include <freertos/FreeRTOSConfig.h>
#include <freertos/task.h>

#include "BTCaster.h"

static const char *TAG = "ESPBT";

BTCaster btcApp;

extern "C" void app_main() {
    esp_log_level_set("*", ESP_LOG_INFO);
    esp_log_level_set(TAG, ESP_LOG_DEBUG);

    ESP_LOGI(TAG, "[ ** ] Running BTCaster.app..");
    btcApp.begin();
}
