/*
 * sensor.h
 *
 *  Created on: 28 kwi 2022
 *      Author: Darek
 */

#ifndef SENSOR_H_
#define SENSOR_H_

#include "em_i2c.h"
#include <stdint.h>
#include <stddef.h>
#include <string.h>

#define RAM_STORAGE_CAPACITY  500
#define RAM_STORAGE_LEN       4

typedef int sensorStatus_t;

enum sensorCodes
{
  SENSOR_ERR = 0, SENSOR_OK
};

int16_t
i2cTestFunction (void);

sensorStatus_t
sensorInit (uint8_t sensorType);

sensorStatus_t
sensorGetTemperatureC (uint8_t sensorId, float *data);

sensorStatus_t
sensorGetTemperatureF (uint8_t sensorId, float *data);

sensorStatus_t
sensorGetTemperatureRaw (uint8_t sensorId, int16_t *data);

sensorStatus_t
sensorGetAccelerometerX (uint8_t sensorId, int *data);

sensorStatus_t
sensorGetAccelerometerY (uint8_t sensorId, int *data);

sensorStatus_t
sensorGetAccelerometerZ (uint8_t sensorId, int *data);

sensorStatus_t
sensorGetAccelerometer (uint8_t sensorId, int *dataX, int *dataY, int *dataZ);

sensorStatus_t
sensorGetHumidity (uint8_t sensorId, int *data);

sensorStatus_t
sensorGetPinIrq(uint8_t sensorId, uint8_t *data);

sensorStatus_t
sensorGetDataFromRamStorage (uint32_t *outputBuffer, size_t size, int idx);

sensorStatus_t
sensorPutDataToRamStorage (uint32_t *inputBuffer, size_t size);

uint32_t
sensorGetRamStorageCurrentIndex(void);

sensorStatus_t
sensorClearDataBuffer(void);

enum
{
  CUSTOM = 0, AS6212, FLOOD_SENSOR, AD7156
};

#endif /* SENSOR_H_ */
