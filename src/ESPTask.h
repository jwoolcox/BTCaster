#ifndef ESPTASK_H
#define ESPTASK_H

#include <freertos/FreeRTOS.h>
#include <freertos/FreeRTOSConfig.h>
#include <freertos/task.h>
#include <array>

class ESPTask
{
public:
    ESPTask();

    esp_err_t begin(const char *taskName, void *cbParams);
    void end();

    virtual void instanceCbHandler(void *pCbParam) = 0;

    static void cbTaskHandler(void *cbParam)
    {
        std::array<void *, 2> params = *static_cast<std::array<void *, 2> *>(cbParam);

        ESPTask *task = static_cast<ESPTask *>(params[0]);

        task->instanceCbHandler(params[1]);
    }

private:
    char *taskTitle_;
    TaskHandle_t taskHandle_;
};

#endif
