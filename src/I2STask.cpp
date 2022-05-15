#include <esp_log.h>
#include <esp_task_wdt.h>

#include <driver/i2s.h>
#include <freertos/FreeRTOS.h>
#include <freertos/FreeRTOSConfig.h>
#include <freertos/ringbuf.h>

#include "I2STask.h"

void I2STask::instanceCbHandler(void *pCbParam)
{
    const char *CTAG = "- I2STask -";
    //esp_log_level_set(CTAG, ESP_LOG_DEBUG);

    RingbufHandle_t hRingBuffer = *static_cast<RingbufHandle_t *>(pCbParam);

    uint8_t *data = NULL;
    size_t rxSize = 0;
    size_t txSize = 0;

    for (;;)
    {
        data = (uint8_t *)xRingbufferReceive(hRingBuffer, &rxSize, (TickType_t)portMAX_DELAY);

        if (rxSize != 0)
        {
            if (i2s_write(i2s_port_t(0), data, rxSize, &txSize, portMAX_DELAY) != ESP_OK)
                ESP_LOGE(CTAG, "I2S Write error");
            else
                ESP_LOGD(CTAG, "-->> write [%d]", txSize);

            vRingbufferReturnItem(hRingBuffer, (void *)data);
        }

        vTaskDelay(100 / portMAX_DELAY);
    }
}

