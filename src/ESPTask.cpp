#include "ESPTask.h"
#include <array>

ESPTask::ESPTask() {}

esp_err_t ESPTask::begin(const char *taskName, void *pCbParams)
{
    esp_err_t result = ESP_FAIL;

    std::array<void *, 2> *cbParams = new std::array<void *, 2>({this, pCbParams});

    result = xTaskCreate([] (void *cbParam) {
        std::array<void *, 2> params = *static_cast<std::array<void *, 2> *>(cbParam);

        ESPTask *task = static_cast<ESPTask *>(params[0]);

        task->instanceCbHandler(params[1]);
    }, taskName, 3072, cbParams, configMAX_PRIORITIES - 3, &taskHandle_);

    return result;
}

void ESPTask::end()
{
    vTaskDelete(taskHandle_);
}
