#ifndef I2STASK_H
#define I2STASK_H

#include "ESPTask.h"

class I2STask : public ESPTask
{
public:
    virtual void instanceCbHandler(void *pCbParam) override;
protected:
    static void installI2SDriver();
    static void uninstallI2SDriver();
};

#endif
