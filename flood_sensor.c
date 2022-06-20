/*
 * flood_sensor.c
 *
 *  Created on: 1 maj 2022
 *      Author: Darek
 */
#include "flood_sensor.h"
#include "em_gpio.h"

void
(*sensorCallback) (uint8_t pin, uint8_t state);

s_irqEventSensor_t *floodSensorStruct;

bool
irqSensorInitialization (s_irqEventSensor_t *floodSensorPtr, void
(*callbackPtr) (uint8_t pin, uint8_t state))
{
  GPIO_PinModeSet (floodSensorPtr->floodSensorPort,
                   floodSensorPtr->floodSensorPin, gpioModeInputPull, 1);

  if (callbackPtr != NULL)
    {
      sensorCallback = callbackPtr;
    }

  return true;
}

uint8_t
irqSensorGetPolling (s_irqEventSensor_t *floodSensorPtr)
{
  return GPIO_PinInGet (floodSensorPtr->floodSensorPort,
                        floodSensorPtr->floodSensorPin);
}

void
irqSensor_EVENT (s_irqEventSensor_t *floodSensorPtr)
{
  if (sensorCallback != NULL
      && !GPIO_PinInGet (floodSensorPtr->floodSensorPort,
                         floodSensorPtr->floodSensorPin))
    {
      sensorCallback (floodSensorPtr->floodSensorPort,
                      floodSensorPtr->floodSensorPin);
    }
}
