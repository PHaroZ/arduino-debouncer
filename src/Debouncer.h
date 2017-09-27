#ifndef Debouncer_h
#define Debouncer_h

#include "Arduino.h"

template <uint8_t dataWidth, typename ShiftType>
class _Debouncer  {
private:
  uint8_t debounceTime;
  ShiftType lastRawStates;
  uint16_t lastRawTimes[dataWidth];
  ShiftType lastStates;
  inline bool lastState(int index) { return bitRead(this->lastStates, index); }

  inline bool lastRawState(int index) { return bitRead(this->lastRawStates, index); }

  inline bool lastRawTime(int index) { return this->lastRawTimes[index]; }

public:
  _Debouncer() : debounceTime(30) { }

  inline void setDebounceTime(uint8_t value) { this->debounceTime = value; }

  inline ShiftType read() { return this->lastStates; }

  inline uint8_t getDataWidth() { return dataWidth; }

  inline boolean state(int index) { return this->lastState(index); }

  void begin(bool initialState) {
    for (uint8_t index = 0; index < dataWidth; index++) {
      bitWrite(this->lastStates, index, initialState);
    }
    this->lastRawStates = this->lastStates;
  }

  bool debounce(ShiftType states) {
    // greatly improse speed when nothing has changed
    if (states == this->lastRawStates && states == this->lastStates) {
      return false;
    }

    uint16_t now = millis();
    bool state;
    bool changed = false;

    for (uint8_t index = 0; index < dataWidth; index++) {
      state = bitRead(states, index);
      if (state != this->lastRawState(index)) {
        this->lastRawTimes[index] = now;
        bitWrite(this->lastRawStates, index, state);
      } else {
        // new state is equals to last read
        // is it different to last real state ?
        if (state != this->lastState(index)) {
          // yes, but rawState is it stabilized ?
          if (now - this->lastRawTime(index) >= this->debounceTime) {
            // yes ! so we can consider state as really changed
            bitWrite(this->lastStates, index, state);
            changed = true;
          }
        }
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
