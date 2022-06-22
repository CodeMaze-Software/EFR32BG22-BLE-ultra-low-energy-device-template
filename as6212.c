/*
 * as6212.c
 *
 *  Created on: 29 kwi 2022
 *      Author: Darek
 */

#include "as6212.h"
#include "sensor_i2c_utils.h"

uint8_t txBuffer[3];
uint8_t rxBuffer[2];
static uint8_t deviceAddress;

bool
as6212Init (unsigned int as6212I2cAddr)
{
  if (I2C_INITIALIZED_OK != sensorI2cGetInitStatus ())
    sensorI2cInit (I2C_0);

  if (as6212I2cAddr >= 0x44 && as6212I2cAddr <= 0x4B)
    {
      deviceAddress = as6212I2cAddr;
      return true;
    }
  else
    return false;
}

void
as6212SetConfig (uint16_t configState)
{
  as6212WriteRegister (CONFIG, configState);
}

uint16_t
as6212GetConfig (void)
{
  return as6212ReadRegister (CONFIG, 2);
}

uint16_t
as6212ReadRegister (uint8_t reg, uint8_t size)
{
  int16_t dataC = 0;

  sensorI2cReadData (deviceAddress, reg, rxBuffer, size);

  dataC = ((rxBuffer[0] << 8) | rxBuffer[1]);

  return dataC;
}

void
as6212WriteRegister (uint8_t reg, int16_t data)
{
  txBuffer[0] = reg;
  txBuffer[1] = (data << 8);
  txBuffer[2] = data;

  sensorI2cWriteData (deviceAddress, txBuffer, 3);
}

float
as6212ReadTemperatureC (void)
{
  int16_t digitalTempC = as6212ReadRegister (TVAL, 2);

  float finalTempC;

  if (digitalTempC < 32768)
    {
      finalTempC = digitalTempC * 0.0078125;
    }

  if (digitalTempC >= 32768)
    {
      finalTempC = ((digitalTempC - 1) * 0.0078125) * -1;
    }

  return finalTempC;
}

float
as6212ReadTemperatureF (void)
{
  return as6212ReadTemperatureC () * 9.0 / 5.0 + 32.0;
}

int16_t
as6212ReadTemperatureRaw (void)
{
  return as6212ReadRegister (TVAL, 2);
}

void
as6212SleepModeEnable (void)
{
  as6212WriteRegister (CONFIG, SLEEPMODE);
}

void
as6212SleepModeDisable (void)
{
  as6212WriteRegister (CONFIG, DEFAULTM);
}

void
as6212SleepMode (bool sleepModeState)
{
  sleepModeState ? as6212SleepModeEnable () : as6212SleepModeDisable ();
}
