#include <PinChangeInterruptHandler.h>

// Inherit from PinChangeInterruptHandler
class ChangeCounter : PinChangeInterruptHandler {
  byte inputPin;
  unsigned long riseCounter;
  unsigned long fallCounter;

public:
  ChangeCounter(byte pin) : riseCounter(0), fallCounter(0), inputPin(pin) {}
  
  void begin() {
    pinMode(inputPin, INPUT);
    riseCounter = 0;
    fallCounter = 0;
    attachPCInterrupt(digitalPinToPCINT(inputPin));
  }

  void stop() {
    detachPCInterrupt(digitalPinToPCINT(inputPin));
  }

  unsigned long getRises() {
    return riseCounter;
  }

  unsigned long getFalls() {
    return fallCounter;
  }

  // Overwrite handlePCInterrupt() method
  virtual void handlePCInterrupt(int8_t interruptNum, bool value) {
    if (value)
      ++riseCounter;
    else
      ++fallCounter;
  }
};

ChangeCounter counter(2);

void setup() {
  Serial.begin(9600);
}

void loop() {
  counter.begin();
  delay(5000);
  counter.stop();

  Serial.println("");
  Serial.print("Rises: ");
  Serial.println(counter.getRises());
  Serial.print("Falls: ");
  Serial.println(counter.getFalls());
}
