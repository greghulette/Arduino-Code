#pragma once
/**
 * logic_animations.h
 * ------------------
 * Enhanced R2D2 Logic Display Animation System
 * 
 * Complete implementation with 12 animations:
 *   Phase 1 (Core):
 *     - NORMAL: Standard R2D2 random blinking
 *     - SOLIDCOLOR: Solid color display
 *     - FLASHCOLOR: Flashing on/off
 *     - LIGHTSOUT: All off (standby)
 *   
 *   Phase 2 (Advanced):
 *     - ALARM: Fast chaotic warning
 *     - FAILURE: Slow stuttering failure
 *     - FIRE: Flickering fire effect
 *     - RAINBOW: Smooth rainbow wave
 *     - ROAMINGPIXEL: K.I.T.T./Cylon scanner
 *     - REDALERT: Urgent red alert flash
 *     - COLORSWAP: Alternate between two colors
 *     - FLIPFLOPCOLOR: Split display flip-flop
 * 
 * Features:
 *   - 4 configurable colors (COLOR1-4) with weighted distribution
 *   - Per-color brightness (0-255) for background/foreground effects
 *   - Named color presets (RED, GREEN, BLUE, R2BLUE, etc.)
 *   - RGB value support (0-255 for each channel)
 *   - Authentic R2D2 patterns
 */

#include <Arduino.h>
#include <Adafruit_NeoPixel.h>

// ============================================================================
// ANIMATION ENUMERATION
// ============================================================================

enum class LogicAnimation : uint8_t {
  NORMAL = 0,
  ALARM,
  FAILURE,
  LEIA,              // Reserved for future
  MARCH,             // Reserved for future
  SOLIDCOLOR,
  FLASHCOLOR,
  FLIPFLOPCOLOR,
  FLIPFLOPALTCOLOR,  // Reserved for future
  COLORSWAP,
  RAINBOW,
  REDALERT,
  FIRE,
  ROAMINGPIXEL,
  HORIZONTALSCANLINE,  // Reserved for future
  VERTICALSCANLINE,    // Reserved for future
  RANDOM,              // Reserved for future
  LIGHTSOUT,

  // 2026-05 sled-ported animations (appended so existing enum values are
  // unchanged). Carried over from the standalone sled sketch, remapped to 8x32.
  TWINKLE,             // soft random multicolour twinkle
  PULSE,               // whole-panel sine breathe on COLOR1
  WIPE,                // column wipe across the matrix in COLOR1
  CHASE,               // theatre chase (COLOR1/COLOR2) marching across
  SPARKLE,             // sharp white/COLOR1 pops with fast fade
  HEARTBEAT,           // double lub-dub brightness throb on COLOR1
  MATRIXRAIN,          // falling "Matrix" drops down the columns (COLOR1)
  SCROLLTEXT,          // scroll/center a text bitmap authored in the editor

  COUNT
};

// Animation names — order MUST match the enum above.
const char* const ANIMATION_NAMES[] = {
  "Normal", "Alarm", "Failure", "Leia", "March",
  "Solid Color", "Flash Color", "Flip Flop Color", "Flip Flop Alt Color",
  "Color Swap", "Rainbow", "Red Alert", "Fire", "Roaming Pixel",
  "Horizontal Scan", "Vertical Scan", "Random", "Lights Out",
  "Twinkle", "Pulse", "Wipe", "Chase", "Sparkle", "Heartbeat", "Matrix Rain", "Scroll Text"
};

// Get animation name
inline const char* getAnimationName(LogicAnimation anim) {
  uint8_t idx = (uint8_t)anim;
  if (idx >= (uint8_t)LogicAnimation::COUNT) return "Unknown";
  return ANIMATION_NAMES[idx];
}

// Parse animation name
inline bool parseAnimationName(const char* name, LogicAnimation& anim) {
  String upper = String(name);
  upper.toUpperCase();
  upper.replace(" ", "");
  
  if (upper == "NORMAL") { anim = LogicAnimation::NORMAL; return true; }
  if (upper == "ALARM") { anim = LogicAnimation::ALARM; return true; }
  if (upper == "FAILURE") { anim = LogicAnimation::FAILURE; return true; }
  if (upper == "SOLID" || upper == "SOLIDCOLOR") { anim = LogicAnimation::SOLIDCOLOR; return true; }
  if (upper == "FLASH" || upper == "FLASHCOLOR") { anim = LogicAnimation::FLASHCOLOR; return true; }
  if (upper == "FLIPFLOP" || upper == "FLIPFLOPCOLOR") { anim = LogicAnimation::FLIPFLOPCOLOR; return true; }
  if (upper == "COLORSWAP" || upper == "SWAP") { anim = LogicAnimation::COLORSWAP; return true; }
  if (upper == "RAINBOW") { anim = LogicAnimation::RAINBOW; return true; }
  if (upper == "REDALERT" || upper == "ALERT") { anim = LogicAnimation::REDALERT; return true; }
  if (upper == "FIRE") { anim = LogicAnimation::FIRE; return true; }
  if (upper == "ROAMING" || upper == "ROAMINGPIXEL" || upper == "SCANNER") { 
    anim = LogicAnimation::ROAMINGPIXEL; return true; 
  }
  if (upper == "OFF" || upper == "LIGHTSOUT") { anim = LogicAnimation::LIGHTSOUT; return true; }

  // Sled-ported animations
  if (upper == "TWINKLE") { anim = LogicAnimation::TWINKLE; return true; }
  if (upper == "PULSE") { anim = LogicAnimation::PULSE; return true; }
  if (upper == "WIPE") { anim = LogicAnimation::WIPE; return true; }
  if (upper == "CHASE") { anim = LogicAnimation::CHASE; return true; }
  if (upper == "SPARKLE") { anim = LogicAnimation::SPARKLE; return true; }
  if (upper == "HEARTBEAT") { anim = LogicAnimation::HEARTBEAT; return true; }
  if (upper == "RAIN" || upper == "MATRIXRAIN") { anim = LogicAnimation::MATRIXRAIN; return true; }
  if (upper == "TEXT" || upper == "SCROLLTEXT") { anim = LogicAnimation::SCROLLTEXT; return true; }

  return false;
}

// ============================================================================
// ANIMATION STATE
// ============================================================================

struct AnimationState {
    uint32_t step;
    uint32_t substep;
    int8_t direction;
    uint16_t roam_x, roam_y;
    uint8_t flip_state;
    uint32_t last_change_ms;
    uint8_t* heat;          // ← CHANGE: Was uint8_t heat[32]
    uint16_t heat_size;     // ← ADD
    
    AnimationState() : step(0), substep(0), direction(1), 
                       roam_x(0), roam_y(0), flip_state(0), 
                       last_change_ms(0), heat(nullptr), heat_size(0) {}
    
    ~AnimationState() {     // ← ADD
      if (heat) delete[] heat;
    }
  };

// ============================================================================
// COLOR DEFINITIONS
// ============================================================================

struct ColorRGB {
  uint8_t r, g, b;
};

namespace NamedColors {
  const ColorRGB RED       = {255, 0, 0};
  const ColorRGB GREEN     = {0, 255, 0};
  const ColorRGB BLUE      = {0, 0, 255};
  const ColorRGB WHITE     = {255, 255, 255};
  const ColorRGB CYAN      = {0, 255, 255};
  const ColorRGB MAGENTA   = {255, 0, 255};
  const ColorRGB YELLOW    = {255, 255, 0};
  const ColorRGB ORANGE    = {255, 165, 0};
  const ColorRGB PURPLE    = {128, 0, 255};
  const ColorRGB PINK      = {255, 192, 203};
  const ColorRGB LIME      = {0, 255, 50};
  const ColorRGB TEAL      = {0, 128, 128};
  const ColorRGB NAVY      = {0, 0, 128};
  const ColorRGB MAROON    = {128, 0, 0};
  const ColorRGB GOLD      = {255, 215, 0};
  const ColorRGB SILVER    = {192, 192, 192};
  const ColorRGB CORAL     = {255, 127, 80};
  const ColorRGB SALMON    = {250, 128, 114};
  const ColorRGB LAVENDER  = {230, 230, 250};
  const ColorRGB TURQUOISE = {64, 224, 208};
  const ColorRGB R2_BLUE   = {0, 100, 255};
  const ColorRGB R2_GREEN  = {0, 255, 0};
  const ColorRGB R2_YELLOW = {255, 200, 0};
  const ColorRGB R2_RED    = {255, 0, 0};
  const ColorRGB R2_WHITE  = {255, 255, 255};
  const ColorRGB BLACK     = {0, 0, 0};
}

// Switch the active matrix mask at runtime (true = FULL 8x32, false = SLED border).
void sledSetMask(bool full);
// Mirror the matrix to match physical mounting (X = left/right, Y = up/down).
void sledSetFlip(bool fx, bool fy);

// ============================================================================
// MAIN CLASS
// ============================================================================

class LogicAnimations {
public:
  LogicAnimations(uint8_t pin, uint16_t count, const char* name);
  ~LogicAnimations();
  
  void begin();
  void update();
  void enable(bool on);
  bool isEnabled() const { return enabled_; }
  // Emit the current 8x32 frame as a compact "F:" + RGB444 hex line (for the
  // WebSerial editor's on-screen matrix preview — no physical LEDs required).
  void emitMirrorFrame(Print& out);
  // Blocking confirmation flash: fade the masked panel up/down `times` in the
  // given colour (used as a UI confirm, e.g. auto-cycle toggled on).
  void confirmPulse(uint8_t r, uint8_t g, uint8_t b, uint8_t times);
  void fadeAll(uint8_t fadeValue);
void fadeAll_WithClear(uint8_t fadeValue);  // Fix the typo: uint8_t not ufadeValue
  void setAnimation(LogicAnimation anim);
  LogicAnimation getAnimation() const { return animation_; }
  static const char* getAnimationName(LogicAnimation anim);
  void setColor(uint8_t slot, uint8_t r, uint8_t g, uint8_t b);
  void setColor(uint8_t slot, const char* color_name);
  void getColor(uint8_t slot, uint8_t& r, uint8_t& g, uint8_t& b);
  
  void setWeight(uint8_t slot, uint8_t weight);
  uint8_t getWeight(uint8_t slot);
  
  void setColorBrightness(uint8_t slot, uint8_t brightness);
  uint8_t getColorBrightness(uint8_t slot);
  
  void loadPresetFL();
  void loadPresetRL();
  
  void setBrightness(uint8_t brightness);
  uint8_t getBrightness() const { return brightness_; }
  
  void setSpeed(uint16_t interval_ms);
  uint16_t getSpeed() const { return anim_speed_ms_; }
  
  void setDensity(uint8_t density);
  uint8_t getDensity() const { return density_; }

  // SCROLLTEXT: load a column bitmap (1 byte/col, bit y = row y) authored in
  // the editor. color is 0xRRGGBB; scroll=false centres static text.
  void setText(const uint8_t* bmp, uint16_t cols, uint32_t color, bool scroll);
  
  bool handleCommand(const String& cmd);
  String getStatus();
  String getHelpMenu();

  static bool parseColorName(const char* name, ColorRGB& color);
  static const char* getColorName(uint8_t r, uint8_t g, uint8_t b);
  
private:
  uint8_t pin_;
  uint16_t led_count_;
  const char* name_;
  Adafruit_NeoPixel* strip_;
  bool enabled_;
  uint32_t last_update_ms_;
  LogicAnimation animation_;
  AnimationState anim_state_;
  ColorRGB colors_[4];
  uint8_t weights_[4];
  uint8_t color_brightness_[4];
  uint8_t brightness_;
  uint16_t anim_speed_ms_;
  uint8_t density_;
  
  void updateAnimation();
  
  // Phase 1
  void animNormal();
  void animSolidColor();
  void animFlash();
  void animLightsOut();
  
  // Phase 2
  void animAlarm();
  void animFailure();
  void animFire();
  void animRainbow();
  void animRoamingPixel();
  void animRedAlert();
  void animColorSwap();
  void animFlipFlopColor();

  // 2026-05 sled-ported animations (remapped to the 8x32 matrix)
  void animTwinkle();
  void animPulse();
  void animWipe();
  void animChase();
  void animSparkle();
  void animHeartbeat();
  void animMatrixRain();
  void animScrollText();

  // SCROLLTEXT state (bitmap authored in the editor)
  uint8_t*  textBmp_   = nullptr;
  uint16_t  textCols_  = 0;
  uint32_t  textColor_ = 0xFFFFFF;
  bool      textScroll_= true;

  static bool parseAnimationName(const char* name, LogicAnimation& anim);

  void applyColorWithBrightness(uint8_t r, uint8_t g, uint8_t b, uint8_t intensity, 
                                uint8_t& out_r, uint8_t& out_g, uint8_t& out_b);
  uint8_t selectColorSlot();
  void hsvToRgb(uint16_t h, uint8_t s, uint8_t v, uint8_t& r, uint8_t& g, uint8_t& b);
};