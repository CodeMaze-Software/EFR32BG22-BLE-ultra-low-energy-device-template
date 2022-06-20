/*
 * sensor.c
 *
 *  Created on: 28 kwi 2022
 *      Author: Darek
 */

#include "sensor.h"
#include "as6212.h"

uint32_t buff[RAM_STORAGE_CAPACITY][RAM_STORAGE_LEN];
uint32_t idx;

sensorStatus_t
sensorInit (uint8_t sensorType)
{
  idx = 1;

  switch (sensorType)
    {
    case AS6212:
      as6212Init (AS6212_DEFAULT_ADDRESS);
      return SENSOR_OK;

      break;

    case FLOOD_SENSOR:
      break;

    case AD7156:
      break;

    case CUSTOM:
      // todo
      return SENSOR_OK;
      break;

    default:
      return SENSOR_ERR;
      break;
    }
}

sensorStatus_t
sensorGetTemperatureC (uint8_t sensorId, float *data)
{
  (void) sensorId;

  *data = as6212ReadTemperatureC ();
  return SENSOR_OK;
}

sensorStatus_t
sensorGetTemperatureF (uint8_t sensorId, float *data)
{
  (void) sensorId;

  *data = as6212ReadTemperatureF ();
  return SENSOR_OK;
}

sensorStatus_t
sensorGetTemperatureRaw (uint8_t sensorId, int16_t *data)
{
  (void) sensorId;

  *data = as6212ReadTemperatureRaw ();
  return SENSOR_OK;
}

sensorStatus_t
sensorGetAccelerometerX (uint8_t sensorId, int *data)
{
  // todo implement ...
  (void) data;

  return SENSOR_ERR;
}

sensorStatus_t
sensorGetAccelerometerY (uint8_t sensorId, int *data)
{
  // todo implement ...
  (void) data;

  return SENSOR_ERR;
}

sensorStatus_t
sensorGetAccelerometerZ (uint8_t sensorId, int *data)
{
  // todo implement ...
  (void) data;

  return SENSOR_ERR;
}

sensorStatus_t
sensorGetAccelerometer (uint8_t sensorId, int *dataX, int *dataY, int *dataZ)
{
  if (SENSOR_OK != sensorGetAccelerometerX (sensorId, dataX))
    return SENSOR_ERR;

  if (SENSOR_OK != sensorGetAccelerometerY (sensorId, dataY))
    return SENSOR_ERR;

  if (SENSOR_OK != sensorGetAccelerometerZ (sensorId, dataZ))
    return SENSOR_ERR;

  return SENSOR_OK;
}

sensorStatus_t
sensorGetHumidity (uint8_t sensorId, int *data)
{
  // todo implement ...
  (void) data;
  (void) sensorId;

  return SENSOR_ERR;
}

sensorStatus_t
sensorGetPinIrq(uint8_t sensorId, uint8_t *data)
{
  // todo implement ...
  (void) data;
  (void) sensorId;

  return SENSOR_ERR;
}

sensorStatus_t
sensorGetDataFromRamStorage (uint32_t *outputBuffer, size_t size, int idx)
{
  if(outputBuffer == NULL || size == 0)
    return SENSOR_ERR;

  memcpy (outputBuffer, buff + idx, size);

  return SENSOR_OK;
}

sensorStatus_t
sensorPutDataToRamStorage (uint32_t *inputBuffer, size_t size)
{
  if(inputBuffer == NULL || size == 0)
    return SENSOR_ERR;

  memcpy (buff + idx, inputBuffer, size);

  if (++idx >= RAM_STORAGE_CAPACITY)
    idx = 1;

  return SENSOR_OK;
}

uint32_t
sensorGetRamStorageCurrentIndex(void)
{
  return idx;
}

sensorStatus_t
sensorClearDataBuffer(void)
{
    for(int i = 0; i < RAM_STORAGE_CAPACITY; i++)
        for(int j =0; j < RAM_STORAGE_LEN; j++)
            buff[i][j] = 0;

    return SENSOR_OK;
}
