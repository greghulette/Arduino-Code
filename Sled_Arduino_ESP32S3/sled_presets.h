#pragma once
// ─────────────────────────────────────────────────────────────────────────────
//  sled_presets.h — saved-animation record for the sled.
//  Kept in a header (not inline in the .ino) so the Arduino IDE's auto-generated
//  function prototypes — which it inserts above the sketch body — can see the
//  type. Defining it inline in the .ino caused "'SledPreset' does not name a type".
// ─────────────────────────────────────────────────────────────────────────────
#include <Arduino.h>

struct SledPreset {
  char     name[20];
  uint8_t  anim;          // LogicAnimation enum value (base type)
  uint16_t speed;         // frame interval ms (10..1000)
  uint8_t  bright;        // global brightness 0..255
  uint8_t  density;       // 1..100 (busy-ness of random anims)
  uint8_t  rgb[4][3];     // COLOR1..4 RGB
  uint8_t  weight[4];     // COLOR1..4 weight %
  uint8_t  cbright[4];    // COLOR1..4 per-colour brightness
};
