#ifndef DISPATCHING_H
#define DISPATCHING_H

enum DispatchTarget_t
{
    APP_CORE,
    BT_GAP,
    A2DP,
    AVRC,
    DAC
};

typedef enum DispatchTarget_t DispatchTarget_t;

typedef struct
{
    DispatchTarget_t eventTarget;
    uint16_t event;
    void *param;
} DispatchMessage_t;

#endif
