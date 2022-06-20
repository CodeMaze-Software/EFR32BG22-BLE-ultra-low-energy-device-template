/*
 * sensor_i2c_utils.c
 *
 *  Created on: 29 kwi 2022
 *      Author: Darek
 */

#include <stddef.h>
#include "sensor_i2c_utils.h"

sensorI2cStatus_t initialized;

int16_t
i2cTestFunction (void)
{
  I2C_TransferSeq_TypeDef i2cSeq;
  I2C_TransferReturn_TypeDef i2cStatus;
  I2C_Init_TypeDef initI2c = I2C_INIT_DEFAULT;
  I2C_Init (I2C0, &initI2c);

  uint8_t i2c_write_data[1];
  uint8_t i2c_read_data[2];

  i2cSeq.addr = (0x48 << 1);
  i2cSeq.flags = I2C_FLAG_WRITE_READ;

  i2c_write_data[0] = 0x00;
  i2cSeq.buf[0].data = i2c_write_data;
  i2cSeq.buf[0].len = 1;

  i2cSeq.buf[1].data = i2c_read_data;
  i2cSeq.buf[1].len = 2;

  i2cStatus = I2C_TransferInit (I2C0, &i2cSeq);
  while (i2cStatus == i2cTransferInProgress)
    {
      i2cStatus = I2C_Transfer (I2C0);
    }

  int16_t regValue = (i2c_read_data[0] << 8 | i2c_read_data[1]);
  return (regValue * 0.0078125);

}

sensorI2cStatus_t
sensorI2cInit (uint8_t i2c)
{
  sensorI2cStatus_t st = SENSOR_I2C_ERR;
  I2C_Init_TypeDef initI2c = I2C_INIT_DEFAULT;

  initialized = I2C_INITIALIZED_OK;

  switch (i2c)
    {
    case I2C_0:
      I2C_Init (I2C0, &initI2c);
      st = SENSOR_I2C_OK;
      break;

    case I2C_1:
      I2C_Init (I2C1, &initI2c);
      st = SENSOR_I2C_OK;
      break;

    default:
      st = SENSOR_I2C_ERR;
      initialized = I2C_NOT_INITIALIZED;
      break;
    }
  return st;
}

sensorI2cStatus_t
sensorI2cGetInitStatus ()
{
  return initialized;
}

sensorI2cStatus_t
sensorI2cWriteData (uint8_t addr, uint8_t *dataTxBuffer, uint8_t txCnt)
{
  return sensorI2cTransferData (addr, dataTxBuffer, txCnt, NULL, 0);
}

sensorI2cStatus_t
sensorI2cReadData (uint8_t addr, uint8_t cmd, uint8_t *dataRxBuffer,
                   uint8_t rxCnt)
{
  return sensorI2cTransferData (addr, &cmd, 1, dataRxBuffer, rxCnt);
}

sensorI2cStatus_t
sensorI2cTransferData (uint8_t addr, uint8_t *dataTxBuffer, uint8_t txCnt,
                       uint8_t *dataRxBuffer, uint8_t rxCnt)
{
  uint32_t timeout = DEFAULT_I2C_TIMEOUT;
  I2C_TransferSeq_TypeDef i2cSeq;
  I2C_TransferReturn_TypeDef i2cStatus;
  sensorI2cStatus_t st = SENSOR_I2C_OK;

  i2cSeq.addr = (addr << 1);
  i2cSeq.flags = I2C_FLAG_WRITE_READ;

  i2cSeq.buf[0].data = dataTxBuffer;
  i2cSeq.buf[0].len = txCnt;

  i2cSeq.buf[1].data = dataRxBuffer;
  i2cSeq.buf[1].len = rxCnt;

  i2cStatus = I2C_TransferInit (I2C0, &i2cSeq);
  while (i2cStatus == i2cTransferInProgress  && timeout--)
    {
      i2cStatus = I2C_Transfer (I2C0);
    }

  if (i2cStatus != i2cTransferDone)
    {
      st = SENSOR_I2C_ERR;
      return st;
    }

  return st;
}
