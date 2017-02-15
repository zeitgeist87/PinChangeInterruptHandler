/*
 * Copyright (C) 2015  Andreas Rohner
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef PINCHANGEINTERRUPTHANDLER_H
#define PINCHANGEINTERRUPTHANDLER_H

#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#include "pins_arduino.h"
#endif

int8_t digitalPinToPCINT(int8_t pin);

class PinChangeInterruptHandler {
public:
  virtual void handlePCInterrupt(int8_t pcIntNum, bool value) = 0;
  void attachPCInterrupt(int8_t pcIntNum);
  void detachPCInterrupt(int8_t pcIntNum);
};

#endif  /* PINCHANGEINTERRUPTHANDLER_H */
