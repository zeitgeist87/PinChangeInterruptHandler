PinChangeInterruptHandler
===========

An object-oriented pin-change interrupt handler library for the Arduino. It uses
a pure C++ virtual method as a callback. This
allows for a clean and object oriented design, which is especially useful for
libraries. The code is heavily influenced by the excellent [PinChangeInterrupt library](https://github.com/NicoHood/PinChangeInterrupt) of NicoHood.

Usage
-----

The usage is very simple. You just inherit from
the class PinChangeInterruptHandler and implement the method handlePCInterrupt().

```cpp
#include <PinChangeInterruptHandler.h>

// Inherit from PinChangeInterruptHandler
class MyLib : PinChangeInterruptHandler {
  byte inputPin;
  unsigned long localState1;
  unsigned long localState2;

public:
  MyLib(byte inputPin) : inputPin(pin) {}
  
  void begin() {
    pinMode(inputPin, INPUT);
    attachPCInterrupt(digitalPinToPCINT(inputPin));
  }

  void stop() {
    detachPCInterrupt(digitalPinToPCINT(inputPin));
  }

  // Overwrite handlePCInterrupt() method
  virtual void handlePCInterrupt(int8_t interruptNum, bool value) {
    // Do something
    localState1 = value;
    // ...
  }
};
```