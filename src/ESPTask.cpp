#include "ESPTask.h"
#include <array>

ESPTask::ESPTask() {}

esp_err_t ESPTask::begin(const char *taskName, void *pCbParams)
{
    esp_err_t result = ESP_FAIL;

    std::array<void *, 2> *cbParams = new std::array<void *, 2>({this, pCbParams});

    result = xTaskCreate(cbTaskHandler, taskName, 3072, cbParams, configMAX_PRIORITIES - 3, &taskHandle_);

    return result;
}

void ESPTask::end()
{
    vTaskDelete(taskHandle_);
}
