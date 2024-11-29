#pragma once

#include <Arduino.h>
#include <SPI.h>

namespace types {

using u8 = uint8_t;
using u16 = uint16_t;
using u32 = uint32_t;
using u64 = uint64_t;
using i8 = int8_t;
using i16 = int16_t;
using i32 = int32_t;
using i64 = int64_t;
using byte = unsigned char; // this is somehow fine, and gets ignored by the compiler when using Arduino.h and using namespace types
using f32 = float;
using f64 = double;

#ifdef ARDUINO_ARCH_RP2040
using GenericSPI = SPIClassRP2040;
#else
using GenericSPI = SPIClass;
#endif

// Return value with status code. No funny rust stuff :D
template <typename T, typename E>
struct Result {
  Result() = default;
  const T value;
  const E status;
};

// Simple return status
enum class Status: u8 {
  ok,
  error,
};

}
