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

private:
    char *taskTitle_;
    TaskHandle_t taskHandle_;
};

#endif
