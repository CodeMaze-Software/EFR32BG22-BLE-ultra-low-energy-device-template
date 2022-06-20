/*
 * sensor_i2c_utils.h
 *
 *  Created on: 29 kwi 2022
 *      Author: Darek
 */

#ifndef SENSOR_I2C_UTILS_H_
#define SENSOR_I2C_UTILS_H_

#include "em_i2c.h"

typedef int sensorI2cStatus_t;

#define DEFAULT_I2C_TIMEOUT 300000

enum
{
  SENSOR_I2C_ERR = (-4),
  SENSOR_I2C_BUSY,
  SENSOR_I2C_TIMEOUT,
  SENSOR_I2C_DATA_ERR
};

enum
{
  I2C_NOT_INITIALIZED, SENSOR_I2C_OK, I2C_INITIALIZED_OK
};

enum
{
  I2C_0 = 0, I2C_1
};

int16_t
i2cTestFunction (void);

sensorI2cStatus_t
sensorI2cInit (uint8_t i2c);

sensorI2cStatus_t
sensorI2cGetInitStatus ();

sensorI2cStatus_t
sensorI2cWriteData (uint8_t addr, uint8_t *dataTxBuffer, uint8_t txCnt);

sensorI2cStatus_t
sensorI2cReadData (uint8_t addr, uint8_t cmd, uint8_t *dataRxBuffer, uint8_t rxCnt);

sensorI2cStatus_t
sensorI2cTransferData (uint8_t addr, uint8_t *dataTxBuffer, uint8_t txCnt, uint8_t *dataRxBuffer,
                   uint8_t rxCnt);

#endif /* SENSOR_I2C_UTILS_H_ */
