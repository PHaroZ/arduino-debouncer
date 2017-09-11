#ifndef Debouncer_h
#define Debouncer_h

#include "Arduino.h"

template <byte dataWidth, typename ShiftType>
class _Debouncer  {
private:
  uint8_t debounceTime;
  ShiftType lastRawStates;
  uint16_t nextRawTimes[dataWidth];
  ShiftType lastStates;
  inline boolean lastState(int index) { return bitRead(this->lastStates, index); }

  inline boolean lastRawState(int index) { return bitRead(this->lastRawStates, index); }

  inline boolean nextRawTime(int index) { return this->nextRawTimes[index]; }

public:
  _Debouncer() : debounceTime(30) { }

  inline void setDebounceTime(uint8_t value) { this->debounceTime = value; }

  inline ShiftType read() { return this->lastStates; }

  inline uint16_t getDataWidth() { return dataWidth; }

  inline boolean state(int index) { return this->lastState(index); }

  bool debounce(ShiftType states) {
    uint16_t now = millis();
    uint8_t state;
    bool changed = false;

    for (uint16_t index = 0; index < dataWidth; index++) {
      state = bitRead(states, index);
      if (state != this->lastState(index)) {
        // don't read too quickly ->
        if (now > this->nextRawTime(index)) {
          // new state should be equals to previous read to be considered as valid ->
          if (this->lastRawState(index) == state) {
            // state as really change, save it ->
            bitWrite(this->lastStates, index, state);
            changed = true;
          }
          this->nextRawTimes[index] = now + this->debounceTime;
        }
        bitWrite(this->lastRawStates, index, state);
      }
    }

    return changed;
  }
};

// fallback with 64 bit state (up to 8 shift registers)
template <byte dataWidth>
class Debouncer : public _Debouncer<dataWidth, uint64_t> { };
// single shift register (8 bit state)
template <>
class Debouncer<8> : public _Debouncer<8, uint8_t> { };
// two shift registers (16 bit state)
template <>
class Debouncer<16> : public _Debouncer<16, uint16_t> { };
// four shift registers (32 bit state)
template <>
class Debouncer<32> : public _Debouncer<32, uint32_t> { };

#endif // ifndef Debouncer_h
