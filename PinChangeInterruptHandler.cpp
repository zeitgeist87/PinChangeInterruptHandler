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

#if defined(__AVR__)
#include <avr/interrupt.h>
#endif

#include "PinChangeInterruptHandler.h"
#include "PinChangeInterruptBoards.h"

#ifndef NOT_AN_INTERRUPT
  #define NOT_AN_INTERRUPT -1
#endif

#ifdef PCINT_FALLBACK

// Low performance fallback for unsupported MCUs
static PinChangeInterruptHandler *handlers[EXTERNAL_NUM_INTERRUPTS];
static bool state[EXTERNAL_NUM_INTERRUPTS];

#ifdef PCINT_USE_INTERRUPT_MAP
static byte interruptMap[EXTERNAL_NUM_INTERRUPTS];
#define interruptToDigitalPin(p) (interruptMap[p])

static inline void initInterruptMap() {
  static bool init = true;

  if (init) {
    for (byte i = 0; i < 255; ++i) {
      byte pin = digitalPinToInterrupt(i);
      if (pin != NOT_AN_INTERRUPT && pin < EXTERNAL_NUM_INTERRUPTS) {
        interruptMap[pin] = i;
      }
    }

    init = false;
  }
}

#else
#define interruptToDigitalPin(p) (p)
#endif

int8_t digitalPinToPCINT(int8_t pin) {
  return digitalPinToInterrupt(pin);
}

static void changeInterrupt() {
  for (byte i = 0; i < EXTERNAL_NUM_INTERRUPTS; ++i) {
    bool data = digitalRead(interruptToDigitalPin(i));
    if (data != state[i]) {
      state[i] = data;
      PinChangeInterruptHandler *handler = handlers[i];
      if (handler)
        handler->handlePCInterrupt(i, data);
    }
  }
}

void PinChangeInterruptHandler::attachPCInterrupt(int8_t pcIntNum) {
  #ifdef PCINT_USE_INTERRUPT_MAP
  initInterruptMap();
  #endif

  if (pcIntNum == NOT_AN_INTERRUPT || pcIntNum >= EXTERNAL_NUM_INTERRUPTS)
    return;

  state[pcIntNum] = digitalRead(interruptToDigitalPin(pcIntNum));
  attachInterrupt(pcIntNum, changeInterrupt, CHANGE);
  handlers[pcIntNum] = this;
}

void PinChangeInterruptHandler::detachPCInterrupt(int8_t pcIntNum) {
  if (pcIntNum == NOT_AN_INTERRUPT || pcIntNum >= EXTERNAL_NUM_INTERRUPTS)
    return;

  detachInterrupt(pcIntNum);
  handlers[pcIntNum] = 0;
}

#else

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

// Macro copied from PinChangeInterrupt library https://github.com/NicoHood/PinChangeInterrupt
// convert a normal pin to its PCINT number (0 - max 23), used by the user
// calculates the pin by the Arduino definitions
int8_t digitalPinToPCINT(int8_t p) {
  return digitalPinToPCICR(p) ? ((8 * digitalPinToPCICRbit(p)) + digitalPinToPCMSKbit(p)) : NOT_AN_INTERRUPT;
}

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

#endif
