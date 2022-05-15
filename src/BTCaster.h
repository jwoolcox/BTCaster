#ifndef BTCASTER_H
#define BTCASTER_H

#include <AC101.h>

#include "AppDispatcher.h"

#define PIN_KEY_1 GPIO_NUM_36       // KEY 1
#define PIN_KEY_2 GPIO_NUM_13       // KEY 2
#define PIN_WIFI_TOGGLE GPIO_NUM_19 // KEY 3
#define PIN_BT_PAIR GPIO_NUM_23     // KEY 4
#define PIN_VOL_UP GPIO_NUM_18      // KEY 5
#define PIN_VOL_DOWN GPIO_NUM_5     // KEY 6

#define PIN_LED_4 GPIO_NUM_22       // LED 4
#define PIN_LEDX  GPIO_NUM_21       // LED ?
#define PIN_LED_5 GPIO_NUM_19       // LED 5

class BTCaster
{
public:
    BTCaster();

    void begin();
    void end();

    static bool postMessage(DispatchTarget_t dispatchTarget, uint16_t dispatchEvent, void* dispatchParam, int dispatchParmaLen);

private:
    AppDispatcher taskApplication_;
    
    esp_err_t initESP();
};

#endif
