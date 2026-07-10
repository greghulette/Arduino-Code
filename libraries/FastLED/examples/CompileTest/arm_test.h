#pragma once

#include "FastLED.h"

void arm_tests() {
#ifndef FL_IS_ARM
#error "FL_IS_ARM should be defined for ARM platforms"
#endif

#if FASTLED_USE_PROGMEM != 0 && FASTLED_USE_PROGMEM != 1
#error "FASTLED_USE_PROGMEM should be either 0 or 1 for ARM platforms"
#endif

#if defined(ARDUINO_TEENSYLC) || defined(ARDUINO_TEENSY30) || defined(__MK20DX128__) || defined(__MK20DX256__) || defined(ARDUINO_ARCH_RENESAS_UNO) || defined(STM32F1)
    // Teensy LC, Teensy 3.0, Teensy 3.1/3.2, Renesas UNO, and STM32F1 have limited memory
    #if SKETCH_HAS_LARGE_MEMORY != 0
    #error "SKETCH_HAS_LARGE_MEMORY should be 0 for Teensy LC, Teensy 3.0, Teensy 3.1/3.2, Renesas UNO, and STM32F1"
    #endif
    #if SKETCH_HAS_HUGE_MEMORY != 0
    #error "SKETCH_HAS_HUGE_MEMORY should be 0 for Teensy LC, Teensy 3.0, Teensy 3.1/3.2, Renesas UNO, and STM32F1"
    #endif
#elif defined(FL_IS_TEENSY_35) || defined(FL_IS_TEENSY_36) || defined(FL_IS_TEENSY_4X) \
   || defined(ARDUINO_ARCH_RP2040) || defined(PICO_RP2040) || defined(PICO_RP2350) \
   || defined(__SAMD51__) \
   || defined(STM32F4xx) || defined(STM32H7xx) || defined(ARDUINO_GIGA)
    // Huge memory ARM platforms (>= 256KB RAM)
    #if SKETCH_HAS_LARGE_MEMORY != 1
    #error "SKETCH_HAS_LARGE_MEMORY should be 1 for huge ARM platforms"
    #endif
    #if SKETCH_HAS_HUGE_MEMORY != 1
    #error "SKETCH_HAS_HUGE_MEMORY should be 1 for huge ARM platforms"
    #endif
#else
    // Most other ARM platforms have large memory but are not "huge"
    // (e.g., SAMD21, nRF52, generic Cortex-M)
    #if SKETCH_HAS_LARGE_MEMORY != 1
    #error "SKETCH_HAS_LARGE_MEMORY should be 1 for most ARM platforms"
    #endif
    #if SKETCH_HAS_HUGE_MEMORY != 0
    #error "SKETCH_HAS_HUGE_MEMORY should be 0 for generic ARM platforms (high tier, not huge)"
    #endif
#endif

#if FASTLED_ALLOW_INTERRUPTS != 1 && FASTLED_ALLOW_INTERRUPTS != 0
#error "FASTLED_ALLOW_INTERRUPTS should be either 0 or 1 for ARM platforms"
#endif

// Check that F_CPU is defined
#ifndef F_CPU
#error "F_CPU should be defined for ARM platforms"
#endif

// Specific ARM variant checks
#if defined(ARDUINO_ARCH_STM32) || defined(STM32F1)
    #if FASTLED_ALLOW_INTERRUPTS != 0
    #error "STM32 platforms should have FASTLED_ALLOW_INTERRUPTS set to 0"
    #endif
    #if FASTLED_USE_PROGMEM != 0
    #error "STM32 platforms should have FASTLED_USE_PROGMEM set to 0"
    #endif
#endif

#if defined(ARDUINO_ARCH_RP2040) || defined(ARDUINO_RASPBERRY_PI_PICO)
    #if FASTLED_USE_PROGMEM != 0
    #error "RP2040 platforms should have FASTLED_USE_PROGMEM set to 0"
    #endif
    #if FASTLED_ALLOW_INTERRUPTS != 1
    #error "RP2040 platforms should have FASTLED_ALLOW_INTERRUPTS set to 1"
    #endif
    #ifdef FASTLED_FORCE_SOFTWARE_SPI
    // RP2040 forces software SPI - this is expected
    #endif
#endif

#if defined(__MK20DX128__) || defined(__MK20DX256__) || defined(__MK66FX1M0__) || defined(__IMXRT1062__)
    // Teensy platforms that use PROGMEM
    #if FASTLED_USE_PROGMEM != 1
    #error "Teensy K20/K66/MXRT1062 platforms should have FASTLED_USE_PROGMEM set to 1"
    #endif
#endif

#if defined(ARDUINO_ARCH_SAMD) || defined(ARDUINO_SAM_DUE)
    #if FASTLED_USE_PROGMEM != 0
    #error "SAMD/SAM platforms should have FASTLED_USE_PROGMEM set to 0"
    #endif
#endif

#if defined(NRF52_SERIES) || defined(ARDUINO_ARCH_NRF52)
    #if FASTLED_USE_PROGMEM != 0
    #error "NRF52 platforms should have FASTLED_USE_PROGMEM set to 0"
    #endif
    #ifndef CLOCKLESS_FREQUENCY
    #error "NRF52 should have CLOCKLESS_FREQUENCY defined"
    #endif
#endif

// DEFAULT macro must survive FastLED.h inclusion (used by analogReference)
#if defined(ARDUINO)
#ifndef DEFAULT
#error "DEFAULT macro should be defined after including FastLED.h on ARM Arduino platforms"
#endif
#endif
}
