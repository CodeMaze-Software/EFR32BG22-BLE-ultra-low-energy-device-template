/*
 * as6212.h
 *
 *  Created on: 29 kwi 2022
 *      Author: Darek
 */

#ifndef AS6212_H_
#define AS6212_H_

#include <stdbool.h>
#include <stdint.h>

#define AS6212_DEFAULT_ADDRESS      0x48

enum AS6212_Register
{

  TVAL = 0x0,    //Temperature Register
  CONFIG = 0x1,    //Configuration Register
  TLOW = 0x2,    //Low Temperature Threshold
  THIGH = 0x3,    //High Temperature Threshold

  DEFAULTM = 0x40A0,   //Default state
  SLEEPMODE = 0x41A0,   //Sleep Mode
  SLEEPSS = 0xC1A0,   //Sleep Mode Single Shot

  SINGLESHOT = 0x8000, //15
  CFAULT_1 = 0x0800, //12
  CFAULT_0 = 0x0400, //11
  POLARITY = 0x0200, //10
  INTERRUPT = 0x0100, //9
  SLEEP = 0x0080, //8
  CONVER_RATE_1 = 0x0040, //7
  CONVER_RATE_0 = 0x0020, //6
  ALERT = 0x0010, //5

};

bool
as6212Init (unsigned int as6212I2cAddr);

void
as6212SetConfig (uint16_t configState);

uint16_t
as6212GetConfig (void);

uint16_t
as6212ReadRegister (uint8_t reg, uint8_t size);

void
as6212WriteRegister (uint8_t reg, int16_t data);

float
as6212ReadTemperatureC (void);

float
as6212ReadTemperatureF (void);

int16_t
as6212ReadTemperatureRaw (void);

void
as6212SleepModeEnable (void);

void
as6212SleepModeDisable (void);

void
as6212SleepMode (bool sleepModeState);

#endif /* AS6212_H_ */
