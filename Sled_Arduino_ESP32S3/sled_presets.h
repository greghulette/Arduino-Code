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
  uint16_t speed;         // frame interval ms (10..1000); for SCROLLTEXT = scroll speed
  uint8_t  bright;        // global brightness 0..255
  uint8_t  density;       // 1..100 (busy-ness of random anims)
  uint8_t  rgb[4][3];     // COLOR1..4 RGB
  uint8_t  weight[4];     // COLOR1..4 weight %
  uint8_t  cbright[4];    // COLOR1..4 per-colour brightness

  // 2026-06: per-preset scroll text — only used when anim == SCROLLTEXT, so each
  // text preset carries its OWN message/language (e.g. one English, one Aurebesh).
  // The editor rasterizes the message in the chosen font and ships the bitmap;
  // textMsg/textFont are stored so the editor can re-edit the preset later.
  uint16_t textCols;      // 0 = no text for this preset
  uint8_t  textRgb[3];    // text colour
  uint8_t  textScroll;    // 0/1
  uint8_t  textFont;      // 0 = English (latin), 1 = Aurebesh (editor-side only)
  char     textMsg[40];   // original message (for re-edit in the studio)
  uint8_t  textBmp[256];  // == TEXT_MAX_COLS; rasterized 8-row column bitmap
};
