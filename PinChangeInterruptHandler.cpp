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

#include <avr/interrupt.h>
#include "PinChangeInterruptBoards.h"
#include "PinChangeInterruptHandler.h"

#if defined(PCINT3_vect)
  #define PORT_COUNT 4
#elif defined(PCINT2_vect)
  #define PORT_COUNT 3
#elif defined(PCINT1_vect)
  #define PORT_COUNT 2
#else
  #define PORT_COUNT 1
#endif

#ifdef PCMSK0
  #define portToPCMSK(port)   (*(&PCMSK0 + (port)))
#elif defined(PCMSK)
  #define portToPCMSK(port)   (*(&PCMSK + (port)))
#endif

#if !defined(PCICR) && defined(GIMSK)
  #define PCICR   GIMSK
#endif

#ifdef PCIE0
  #define portToPCICRbit(port)   ((port) + PCIE0)
#elif defined(PCIE)
  #define portToPCICRbit(port)   ((port) + PCIE)
#endif

static PinChangeInterruptHandler *handlers[PORT_COUNT * 8];

void PinChangeInterruptHandler::attachPCInterrupt(int8_t pcIntNum) {
  byte port = pcIntNum / 8;
  byte bits = pcIntNum % 8;

  if (pcIntNum < 0 || port >= PORT_COUNT)
    return;

  noInterrupts();

  // Enable pins
  portToPCMSK(port) |= 1 << bits;
  // Enable ports
  PCICR |= 1 << portToPCICRbit(port);

  handlers[pcIntNum] = this;

  interrupts();
}

void PinChangeInterruptHandler::detachPCInterrupt(int8_t pcIntNum) {
  byte port = pcIntNum / 8;
  byte bits = pcIntNum % 8;

  if (pcIntNum < 0 || port >= PORT_COUNT)
    return;

  noInterrupts();

  // Disable pins
  portToPCMSK(port) &= ~(1 << bits);
  if (portToPCMSK(port) == 0) {
    // Disable ports
    PCICR &= ~(1 << portToPCICRbit(port));
  }

  handlers[pcIntNum] = 0;

  interrupts();
}

static void changeInterrupt(byte port, byte state) {
  static byte oldState[4];

  byte change = oldState[port] ^ state;
  oldState[port] = state;

  byte i = 0;
  while (change) {
    if (change & 1) {
      byte pcIntNum = (port << 3) + i;
      PinChangeInterruptHandler *handler = handlers[pcIntNum];
      if (handler)
        handler->handlePCInterrupt(pcIntNum, !!(state & (1 << i)));
    }
    change >>= 1;
    ++i;
  }
}

#if defined(PCINT0_vect)
  ISR(PCINT0_vect) { changeInterrupt(0, PCINT_INPUT_PORT0); }
#endif
#if defined(PCINT1_vect)
  ISR(PCINT1_vect) { changeInterrupt(1, PCINT_INPUT_PORT1); }
#endif
#if defined(PCINT2_vect)
  ISR(PCINT2_vect) { changeInterrupt(2, PCINT_INPUT_PORT2); }
#endif
#if defined(PCINT3_vect)
  ISR(PCINT3_vect) { changeInterrupt(3, PCINT_INPUT_PORT3); }
#endif
