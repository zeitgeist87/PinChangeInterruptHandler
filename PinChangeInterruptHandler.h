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

#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
#elif defined(ENERGIA)
  #include "Energia.h"
#else
  #include "WProgram.h"
#endif


// Macro copied from PinChangeInterrupt library https://github.com/NicoHood/PinChangeInterrupt
// convert a normal pin to its PCINT number (0 - max 23), used by the user
// calculates the pin by the Arduino definitions
#define digitalPinToPinChangeInterrupt(p) (digitalPinToPCICR(p) ? ((8 * digitalPinToPCICRbit(p)) + digitalPinToPCMSKbit(p)) : NOT_AN_INTERRUPT)
// alias for shorter writing
#define digitalPinToPCINT digitalPinToPinChangeInterrupt

class PinChangeInterruptHandler {
public:
  virtual void handlePCInterrupt(int8_t pcIntNum, bool value) = 0;
  void attachPCInterrupt(int8_t pcIntNum);
  void detachPCInterrupt(int8_t pcIntNum);
};

#endif  /* PINCHANGEINTERRUPTHANDLER_H */
