/*********************************************************************************
* Copyright 2019 Bob Shen
* FileName: HumanSensor.cpp
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

#include <fcntl.h>
#include <errno.h>
#include <math.h>
#include <poll.h>
#include <unistd.h>
#include <dirent.h>
#include <math.h>
#include <sys/select.h>
#include <cutils/log.h>
#include <utils/BitSet.h>
#include <cutils/properties.h>
#include <linux/ioctl.h>

#include "HumanSensor.h"

// ioctl cmd
#define MW_SENSOR_IOC_MAGIC  'm'

#define MW_SENSOR_IOC_ENABLE _IOW(MW_SENSOR_IOC_MAGIC, 1, int)
#define MW_SENSOR_IOC_SET_RATE _IOW(MW_SENSOR_IOC_MAGIC, 2, int)

HumanSensor::HumanSensor()
    : SensorBase(HUMAN_DEVICE_NAME, "mwsensor"),
      mEnabled(0),
      mInputReader(32)
{
    mPendingEvent.version = sizeof(sensors_event_t);
    mPendingEvent.sensor = ID_HUMAN;
    mPendingEvent.type = SENSOR_TYPE_HUMAN;
    memset(mPendingEvent.data, 0x00, sizeof(mPendingEvent.data));

    int err = 0;
    err = open_device();
    err = err < 0 ? -errno : 0;
    if (err) {
        LOGE("%s:%s\n", __func__, strerror(-err));
        return;
    }
}

HumanSensor::~HumanSensor()
{
    if (mEnabled) {
        enable(0, 0);
    }

    if (dev_fd > 0) {
        close(dev_fd);
        dev_fd = -1;
    }
}

int HumanSensor::enable(int32_t, int en)
{
    int newState = en ? 1 : 0;
    int err = 0;

    if (newState != mEnabled) {
        if (dev_fd < 0) {
            open_device();
        }

        if (0 > (err = ioctl(dev_fd, MW_SENSOR_IOC_ENABLE, &newState))) {
            LOGE("fail to perform MW_SENSOR_IOC_ENABLE, err = %d, error is '%s'", err, strerror(errno));
            goto EXIT;
        }

        mEnabled = newState;
    }

EXIT:
    return err;
}

int HumanSensor::setDelay(int32_t handle, int64_t ns)
{
    int err = 0;

    if (ns < 0) {
        return -EINVAL;
    }

    if (dev_fd < 0) {
        open_device();
    }

    short delay = ns / 1000000;

    if ((err = ioctl(dev_fd, MW_SENSOR_IOC_SET_RATE, &delay)) < 0) {
        LOGE("fail to perform MW_SENSOR_IOC_SET_RATE, result = %d, error is '%s'", err, strerror(errno));
    }

    return err;
}

int HumanSensor::isActivated(int /* handle */)
{
    return mEnabled;
}

int HumanSensor::readEvents(sensors_event_t* data, int count)
{
    if (count < 1) {
        return -EINVAL;
    }

    ssize_t n = mInputReader.fill(data_fd);
    if (n < 0) {
        return n;
    }

    int numEventReceived = 0;
    input_event const* event;

    while (count && mInputReader.readEvent(&event)) {
        int type = event->type;

        LOGI("HumanSensor: read event (type=%d, code=%d)", type, event->code);

        if (type == EV_REL) {
            processEvent(event->code, event->value);
        } else if (type == EV_SYN) {
            mPendingEvent.timestamp = getTimestamp();
            *data++ = mPendingEvent;
            count--;
            numEventReceived++;
        } else {
            LOGE("HumanSensor: unknown event (type=%d, code=%d)", type, event->code);
        }

        mInputReader.next();
    }

    return numEventReceived;
}

void HumanSensor::processEvent(int code, int value)
{
    switch (code) {
        case EVENT_TYPE_HUMAN:
            mPendingEvent.human = value;
            break;
    }
}
