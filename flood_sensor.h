/*
 * flood_sensor.h
 *
 *  Created on: 1 maj 2022
 *      Author: Darek
 */
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

#ifndef FLOOD_SENSOR_H_
#define FLOOD_SENSOR_H_

typedef struct
{
  uint8_t floodSensorPort;
  uint8_t floodSensorPin;
} s_irqEventSensor_t;

bool
irqSensorInitialization (s_irqEventSensor_t *floodSensorPtr, void
(*callbackPtr) (uint8_t pin, uint8_t state));
uint8_t
irqSensorGetPolling (s_irqEventSensor_t *floodSensorPtr);

void
irqSensor_EVENT (s_irqEventSensor_t *floodSensorPtr);

#endif /* FLOOD_SENSOR_H_ */
