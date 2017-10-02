#ifndef Debouncer_h
#define Debouncer_h

#include "Arduino.h"

// #define DEBOUNCER_CPU_OPTIMIZATION

namespace {
// data time for time comparision
// should be uint32_t but we don't need these capacity, range of 65536ms is sufficient
#ifdef DEBOUNCER_CPU_OPTIMIZATION
using TimeType = uint32_t;
#else
using TimeType = uint8_t;
#endif
}

template <size_t dataWidth, typename ShiftType>
class _Debouncer  {
private:
  // number of ms during which a switch must have the same state to be considered as stabilized
  // should be, at max, half of capacity of TimeType
  uint8_t debounceTime;
  // last read states, not debounced
  ShiftType lastRawStates;
  // last states, debounced
  ShiftType lastStates;
  TimeType lastRawTimes[dataWidth];
  bool resetTimeFlag = 0;
  // a half of TimeType capacity
  const TimeType timeTypeHalfValue = ((TimeType) (~TimeType(0))) / 2;

  inline bool lastState(int index) { return bitRead(this->lastStates, index); }

  inline bool lastRawState(int index) { return bitRead(this->lastRawStates, index); }

public:
  _Debouncer() : debounceTime(30) { }

  inline void setDebounceTime(uint8_t value) { this->debounceTime = value; }

  inline ShiftType read() { return this->lastStates; }

  inline uint8_t getDataWidth() { return dataWidth; }

  inline boolean state(int index) { return this->lastState(index); }

  void begin(ShiftType initialState) {
    this->lastStates    = initialState;
    this->lastRawStates = this->lastStates;
  }

  bool debounce(ShiftType &states) {
    #ifndef DEBOUNCER_CPU_OPTIMIZATION
    TimeType now = millis();

    {
      // manage case when millis() overflow capacity of TimeType
      // state which are older than [a half of TimeType capacity] should be shifted of [a half of TimeType capacity] to avoid false debounce detection
      bool inFirstRange = now < this->timeTypeHalfValue;

      if (inFirstRange == this->resetTimeFlag) {
        TimeType resetLimit = now - this->debounceTime;
        for (size_t index = 0; index < dataWidth; index++) {
          if (inFirstRange ? (this->lastRawTimes[index] <=
              resetLimit) : ((TimeType) (now - this->lastRawTimes[index]) >
              this->debounceTime))
          {
            this->lastRawTimes[index] = resetLimit;
          }
        }
        this->resetTimeFlag = !inFirstRange;
      }
    }
    #endif // ifndef DEBOUNCER_CPU_OPTIMIZATION

    // greatly improve speed when nothing has changed
    if (states == this->lastRawStates && states == this->lastStates) {
      return false;
    }

    #ifdef DEBOUNCER_CPU_OPTIMIZATION
    TimeType now = millis();
    #endif

    bool state;
    bool changed = false;

    for (size_t index = 0; index < dataWidth; index++) {
      state = bitRead(states, index);
      if (state != this->lastRawState(index)) {
        this->lastRawTimes[index] = now;
        bitWrite(this->lastRawStates, index, state);
      } else {
        // new state is equals to last read
        // is it different to last real/debounced state ?
        if (state != this->lastState(index)) {
          // yes, but rawState is it stabilized ?
          if ((TimeType) (now - this->lastRawTimes[index]) >= this->debounceTime) {
            // yes ! so we can consider state as really changed
            bitWrite(this->lastStates, index, state);
            changed = true;
          }
        }
      }
    }

    return changed;
  } // debounce
};

// fallback with 64 bit state (up to 8 shift registers)
template <size_t dataWidth>
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
