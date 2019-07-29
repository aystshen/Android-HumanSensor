/*********************************************************************************
* Copyright 2019 Bob Shen
* FileName: HumanSensor.h
* Author: Bob Shen
* Version: 1.0.0
* Date: 2019-7-26
* Description:
*     Custom human sensor for detecting human proximity.
*
* Revision:
*     Date:
*     Reviser:
*     Description:
*********************************************************************************/

#ifndef ANDROID_HUMAN_SENSOR_H
#define ANDROID_HUMAN_SENSOR_H

#include <stdint.h>
#include <errno.h>
#include <sys/cdefs.h>
#include <sys/types.h>

#include "nusensors.h"
#include "SensorBase.h"
#include "InputEventReader.h"

struct input_event;

class HumanSensor : public SensorBase
{
    private:
        int mEnabled;
        InputEventCircularReader mInputReader;
        sensors_event_t mPendingEvent;

    public:
        HumanSensor();
        virtual ~HumanSensor();
        virtual int readEvents(sensors_event_t* data, int count);
        void processEvent(int code, int value);
        virtual int setDelay(int32_t handle, int64_t ns);
        virtual int enable(int32_t handle, int enabled);
        virtual int isActivated(int handle);
};

#endif  // ANDROID_HUMAN_SENSOR_H
