# Android custom Sensor -- HumanSensor
Custom human sensor for detecting human proximity. 

This project will show you how to add a custom Sensor on the Android system, including how to modify the driver, hardware, framework, and how the app uses this Sensor.

## Integration
1. Copy mwsensor to kernel/drivers/input/sensors/ directory.
2. Modify the kernel/drivers/input/sensors/Kconfig as follows:
```
source "drivers/input/sensors/mwsensor/Kconfig"
```
3. Modify the kernel/drivers/input/sensors/Makefile as follows:
```
obj-$(CONFIG_SENSORS_MICROWAVE) += mwsensor/
```
4. Modify the dts as follows:  
```
mwsensor: mwsensor { 
	status = "okay";
	compatible = "topband,mwsensor";
	irq-gpios = <&gpio0 GPIO_D4 IRQ_TYPE_LEVEL_HIGH>;
	
	mwsensor,is_delay = <0>;
	mwsensor,delay_time = <3>;
	mwsensor,is_poll = <1>;
};
```
5. Copy hardware/* to hardware/rockchip/sensor/st/ or hardware/libhardware/modules/sensors/ directory.
6. Modify the hardware/rockchip/sensor/st/Android.mk as follows:
```
LOCAL_SRC_FILES := 						\
				sensors.c 				\
				nusensors.cpp 			\
				GyroSensor.cpp			\
				InputEventReader.cpp	\
				SensorBase.cpp			\
				AkmSensor.cpp			\
				MmaSensor.cpp	\
				LightSensor.cpp	\
				ProximitySensor.cpp		\
				PressureSensor.cpp		\
				TemperatureSensor.cpp		\
				AdcSensor.cpp \
				HumanSensor.cpp
```
7. For how to modify hardware and framework, please refer to HumanSensor.patch.

## Usage
[Download demo](https://fir.im/1a4h)  

After Android adds a custom Sensor, the APP is used in the same way as Android's other default Sensors. You only need to change the Sensor type to Sensor.TYPE_HUMAN.

The following is the implementation of monitoring the Human Sensor status change, for reference only:    
```
mSensorManager = (SensorManager) getSystemService(Context.SENSOR_SERVICE);
if (null != mSensorManager) {
    // Human Sensor
    Sensor humanSensor = mSensorManager.getDefaultSensor(27); // 27: Sensor.TYPE_HUMAN
    if (humanSensor != null) {
        mHumanSensorEventListener = new SensorEventListener() {
            @Override
            public void onSensorChanged(SensorEvent event) {
                float human = event.values[0]; // 1: Near 2: Far
                mSensorView.updateHumanSensorData(human);
            }

            @Override
            public void onAccuracyChanged(Sensor sensor, int accuracy) {

            }
        };
        mSensorManager.registerListener(mHumanSensorEventListener, humanSensor, SensorManager.SENSOR_DELAY_NORMAL);
    }
}
```
## Developed By
* ayst.shen@foxmail.com

## License
```
Copyright 2019 Bob Shen.

Licensed under the Apache License, Version 2.0 (the "License"); you 
may not use this file except in compliance with the License. You may 
obtain a copy of the License at

http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software 
distributed under the License is distributed on an "AS IS" BASIS, 
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or 
implied. See the License for the specific language governing 
permissions and limitations under the License.
```