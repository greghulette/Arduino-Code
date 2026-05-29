/**
 * enhanced_logic_strip_phase1.cpp
 * --------------------------------
 * Phase 1 Implementation: Core R2D2 Logic Display Animations
 * 
 * Includes:
 *   - NORMAL: Standard R2D2 random blinking
 *   - SOLIDCOLOR: Solid color fill
 *   - FLASH: Flashing on/off
 *   - LIGHTSOUT: All off (standby)
 * 
 * Also includes:
 *   - 4-color system with weighted distribution
 *   - Per-color brightness control
 *   - Named color support (20+ colors)
 *   - Command-based control system
 */

#include "logic_animations.h"
#include "matrix_patterns.h"
// #include "animation_settings.h"
// ============================================================================
// COLOR NAME PARSING
// ============================================================================

bool LogicAnimations::parseColorName(const char* name, ColorRGB& color) {
  String upper = String(name);
  upper.toUpperCase();
  
  // Basic colors
  if (upper == "RED")       { color = NamedColors::RED; return true; }
  if (upper == "GREEN")     { color = NamedColors::GREEN; return true; }
  if (upper == "BLUE")      { color = NamedColors::BLUE; return true; }
  if (upper == "WHITE")     { color = NamedColors::WHITE; return true; }
  if (upper == "CYAN")      { color = NamedColors::CYAN; return true; }
  if (upper == "MAGENTA")   { color = NamedColors::MAGENTA; return true; }
  if (upper == "YELLOW")    { color = NamedColors::YELLOW; return true; }
  if (upper == "ORANGE")    { color = NamedColors::ORANGE; return true; }
  if (upper == "PURPLE")    { color = NamedColors::PURPLE; return true; }
  if (upper == "PINK")      { color = NamedColors::PINK; return true; }
  
  // Extended colors
  if (upper == "LIME")      { color = NamedColors::LIME; return true; }
  if (upper == "TEAL")      { color = NamedColors::TEAL; return true; }
  if (upper == "NAVY")      { color = NamedColors::NAVY; return true; }
  if (upper == "MAROON")    { color = NamedColors::MAROON; return true; }
  if (upper == "GOLD")      { color = NamedColors::GOLD; return true; }
  if (upper == "SILVER")    { color = NamedColors::SILVER; return true; }
  if (upper == "CORAL")     { color = NamedColors::CORAL; return true; }
  if (upper == "SALMON")    { color = NamedColors::SALMON; return true; }
  if (upper == "LAVENDER")  { color = NamedColors::LAVENDER; return true; }
  if (upper == "TURQUOISE") { color = NamedColors::TURQUOISE; return true; }
  
  // R2D2 specific
  if (upper == "R2BLUE")    { color = NamedColors::R2_BLUE; return true; }
  if (upper == "R2GREEN")   { color = NamedColors::R2_GREEN; return true; }
  if (upper == "R2YELLOW")  { color = NamedColors::R2_YELLOW; return true; }
  if (upper == "R2RED")     { color = NamedColors::R2_RED; return true; }
  if (upper == "R2WHITE")   { color = NamedColors::R2_WHITE; return true; }
  
  if (upper == "BLACK" || upper == "OFF") { 
    color = NamedColors::BLACK; 
    return true; 
  }
  
  return false;  // Unknown color name
}

const char* LogicAnimations::getColorName(uint8_t r, uint8_t g, uint8_t b) {
  // Check common colors
  if (r == 255 && g == 0 && b == 0) return "RED";
  if (r == 0 && g == 255 && b == 0) return "GREEN";
  if (r == 0 && g == 0 && b == 255) return "BLUE";
  if (r == 255 && g == 255 && b == 255) return "WHITE";
  if (r == 0 && g == 255 && b == 255) return "CYAN";
  if (r == 255 && g == 0 && b == 255) return "MAGENTA";
  if (r == 255 && g == 255 && b == 0) return "YELLOW";
  if (r == 255 && g == 165 && b == 0) return "ORANGE";
  if (r == 0 && g == 0 && b == 0) return "OFF";
  
  return "CUSTOM";  // RGB values don't match a named color
}

// Active matrix mask — defaults to the SLED bordered region; switchable at
// runtime to the FULL 8x32 panel via sledSetMask(). Flip state is remembered
// here so re-creating the mask doesn't lose the panel-orientation flips.
MatrixPattern pattern(8, 32, (const bool*)MatrixPatterns::SLED);
static bool gFlipX = false, gFlipY = false;

void sledSetMask(bool full) {
  pattern = MatrixPattern(8, 32, (const bool*)(full ? MatrixPatterns::FULL
                                                    : MatrixPatterns::SLED));
  pattern.setFlip(gFlipX, gFlipY);   // re-apply after rebuild
}

void sledSetFlip(bool fx, bool fy) {
  gFlipX = fx; gFlipY = fy;
  pattern.setFlip(fx, fy);
}
// ============================================================================
// CONSTRUCTOR / DESTRUCTOR
// ============================================================================

LogicAnimations::LogicAnimations(uint8_t pin, uint16_t count, const char* name)
  : pin_(pin), led_count_(count), name_(name), strip_(nullptr),
    enabled_(false), last_update_ms_(0),
    animation_(LogicAnimation::NORMAL),  // Default to NORMAL animation
    brightness_(80), anim_speed_ms_(50), density_(15)
{
  // Initialize with default colors (will be overridden by presets):
  // COLOR1 blue, COLOR2 white, COLOR3 red, COLOR4 green.
  colors_[0] = NamedColors::BLUE;
  colors_[1] = NamedColors::WHITE;
  colors_[2] = NamedColors::RED;
  colors_[3] = NamedColors::GREEN;
  
  // Default weights (equal distribution)
  weights_[0] = 40;  // COLOR1: 40%
  weights_[1] = 30;  // COLOR2: 30%
  weights_[2] = 20;  // COLOR3: 20%
  weights_[3] = 10;  // COLOR4: 10%
  
  // Default per-color brightness (all at full brightness)
  color_brightness_[0] = 255;  // COLOR1: full brightness
  color_brightness_[1] = 255;  // COLOR2: full brightness
  color_brightness_[2] = 255;  // COLOR3: full brightness
  color_brightness_[3] = 255;  // COLOR4: full brightness
}

LogicAnimations::~LogicAnimations() {
  if (strip_) delete strip_;
  if (textBmp_) delete[] textBmp_;
}

// ============================================================================
// INITIALIZATION
// ============================================================================

void LogicAnimations::begin() {
  Serial.printf("[LED:%s] Initializing enhanced logic strip...\n", name_);
  
  strip_ = new Adafruit_NeoPixel(led_count_, pin_, NEO_GRB + NEO_KHZ800);
  if (!strip_) {
    Serial.printf("[LED:%s] ERROR: NeoPixel creation failed!\n", name_);
    return;
  }
  
  strip_->begin();
  strip_->setBrightness(255);  // We handle brightness ourselves
  strip_->clear();
  strip_->show();
  
  Serial.printf("[LED:%s] Initialized %d LEDs on pin %d\n", name_, led_count_, pin_);
  Serial.printf("[LED:%s] 4-color system with weights: %d%%, %d%%, %d%%, %d%%\n",
                name_, weights_[0], weights_[1], weights_[2], weights_[3]);
  Serial.printf("[LED:%s] Starting with animation: %s\n", name_, getAnimationName(animation_));
}

// ============================================================================
// CONTROL
// ============================================================================

void LogicAnimations::enable(bool on) {
  enabled_ = on;
  
  if (!on && strip_) {
    strip_->clear();
    strip_->show();
  } else if (on && strip_) {
    // ← ADD THIS BLOCK!
    // Force immediate update when enabling
    last_update_ms_ = millis() - anim_speed_ms_;  // Trigger immediate update
    updateAnimation();  // Show something immediately
  }
  
  Serial.printf("[LED:%s] %s\n", name_, on ? "ENABLED" : "DISABLED");
}

void LogicAnimations::update() {
  if (!enabled_ || !strip_) return;
  
  uint32_t now = millis();
  if (now - last_update_ms_ >= anim_speed_ms_) {
    last_update_ms_ = now;
    updateAnimation();
  }
}

// ============================================================================
// ANIMATION CONTROL
// ============================================================================

void LogicAnimations::setAnimation(LogicAnimation anim) {
  animation_ = anim;
  
  // Reset animation state when changing animations
  anim_state_ = AnimationState();
  anim_state_.last_change_ms = millis();
  
  Serial.printf("[LED:%s] Animation set to: %s\n", name_, getAnimationName(anim));
  
  // Immediate update for LIGHTSOUT to turn off instantly
  if (anim == LogicAnimation::LIGHTSOUT && strip_) {
    strip_->clear();
    strip_->show();
  }
}

// logic_animations.cpp

const char* LogicAnimations::getAnimationName(LogicAnimation anim) {
  switch (anim) {
    case LogicAnimation::NORMAL:        return "NORMAL";
    case LogicAnimation::ALARM:         return "ALARM";
    case LogicAnimation::FAILURE:       return "FAILURE";
    case LogicAnimation::FIRE:          return "FIRE";
    case LogicAnimation::RAINBOW:       return "RAINBOW";
    case LogicAnimation::ROAMINGPIXEL:  return "ROAMINGPIXEL";
    case LogicAnimation::REDALERT:      return "REDALERT";
    case LogicAnimation::COLORSWAP:     return "COLORSWAP";
    case LogicAnimation::FLIPFLOPCOLOR: return "FLIPFLOPCOLOR";
    case LogicAnimation::SOLIDCOLOR:    return "SOLIDCOLOR";
    case LogicAnimation::FLASHCOLOR:         return "FLASHCOLOR";
    case LogicAnimation::LIGHTSOUT:     return "LIGHTSOUT";
    case LogicAnimation::TWINKLE:       return "TWINKLE";
    case LogicAnimation::PULSE:         return "PULSE";
    case LogicAnimation::WIPE:          return "WIPE";
    case LogicAnimation::CHASE:         return "CHASE";
    case LogicAnimation::SPARKLE:       return "SPARKLE";
    case LogicAnimation::HEARTBEAT:     return "HEARTBEAT";
    case LogicAnimation::MATRIXRAIN:    return "MATRIXRAIN";
    case LogicAnimation::SCROLLTEXT:    return "SCROLLTEXT";
    default:                            return "UNKNOWN";
  }
}

// ============================================================================
// COLOR MANAGEMENT
// ============================================================================

void LogicAnimations::setColor(uint8_t slot, uint8_t r, uint8_t g, uint8_t b) {
  if (slot < 1 || slot > 4) return;
  
  colors_[slot - 1].r = r;
  colors_[slot - 1].g = g;
  colors_[slot - 1].b = b;
  
  Serial.printf("[LED:%s] COLOR%d set to RGB(%d,%d,%d) [%s]\n",
                name_, slot, r, g, b, getColorName(r, g, b));
}

void LogicAnimations::setColor(uint8_t slot, const char* color_name) {
  if (slot < 1 || slot > 4) return;
  
  ColorRGB color;
  if (parseColorName(color_name, color)) {
    setColor(slot, color.r, color.g, color.b);
  } else {
    Serial.printf("[LED:%s] ERROR: Unknown color name '%s'\n", name_, color_name);
  }
}

void LogicAnimations::getColor(uint8_t slot, uint8_t& r, uint8_t& g, uint8_t& b) {
  if (slot < 1 || slot > 4) {
    r = g = b = 0;
    return;
  }
  
  r = colors_[slot - 1].r;
  g = colors_[slot - 1].g;
  b = colors_[slot - 1].b;
}

void LogicAnimations::setWeight(uint8_t slot, uint8_t weight) {
  if (slot < 1 || slot > 4) return;
  
  weights_[slot - 1] = constrain(weight, 0, 100);
  
  Serial.printf("[LED:%s] WEIGHT%d set to %d%%\n", name_, slot, weights_[slot - 1]);
}

uint8_t LogicAnimations::getWeight(uint8_t slot) {
  if (slot < 1 || slot > 4) return 0;
  return weights_[slot - 1];
}

void LogicAnimations::setColorBrightness(uint8_t slot, uint8_t brightness) {
  if (slot < 1 || slot > 4) return;
  
  color_brightness_[slot - 1] = brightness;
  
  Serial.printf("[LED:%s] COLOR%d brightness set to %d\n", name_, slot, brightness);
}

uint8_t LogicAnimations::getColorBrightness(uint8_t slot) {
  if (slot < 1 || slot > 4) return 0;
  return color_brightness_[slot - 1];
}

// ============================================================================
// PRESETS
// ============================================================================

void LogicAnimations::loadPresetFL() {
  // Front Logic: Blue (60%), White (30%), Custom accent (10%)
  Serial.printf("[LED:%s] Loading FRONT LOGIC preset...\n", name_);
  
  setColor(1, NamedColors::R2_BLUE.r, NamedColors::R2_BLUE.g, NamedColors::R2_BLUE.b);
  setColor(2, NamedColors::R2_WHITE.r, NamedColors::R2_WHITE.g, NamedColors::R2_WHITE.b);
  setColor(3, NamedColors::CYAN.r, NamedColors::CYAN.g, NamedColors::CYAN.b);  // Custom accent
  setColor(4, 0, 0, 0);  // OFF
  
  setWeight(1, 60);  // Blue dominant
  setWeight(2, 30);  // White secondary
  setWeight(3, 10);  // Accent rarely
  setWeight(4, 0);   // OFF
  
  // Reset per-color brightness to defaults
  setColorBrightness(1, 255);
  setColorBrightness(2, 255);
  setColorBrightness(3, 255);
  setColorBrightness(4, 0);
  
  Serial.printf("[LED:%s] Front Logic loaded: Blue(60%%), White(30%%), Cyan(10%%)\n", name_);
}

void LogicAnimations::loadPresetRL() {
  // Rear Logic: Green (40%), Yellow (30%), Red (20%), Custom accent (10%)
  Serial.printf("[LED:%s] Loading REAR LOGIC preset...\n", name_);
  
  setColor(1, NamedColors::R2_GREEN.r, NamedColors::R2_GREEN.g, NamedColors::R2_GREEN.b);
  setColor(2, NamedColors::R2_YELLOW.r, NamedColors::R2_YELLOW.g, NamedColors::R2_YELLOW.b);
  setColor(3, NamedColors::R2_RED.r, NamedColors::R2_RED.g, NamedColors::R2_RED.b);
  setColor(4, NamedColors::ORANGE.r, NamedColors::ORANGE.g, NamedColors::ORANGE.b);  // Custom accent
  
  setWeight(1, 40);  // Green dominant
  setWeight(2, 30);  // Yellow secondary
  setWeight(3, 20);  // Red tertiary
  setWeight(4, 10);  // Accent rarely
  
  // Reset per-color brightness to defaults
  setColorBrightness(1, 255);
  setColorBrightness(2, 255);
  setColorBrightness(3, 255);
  setColorBrightness(4, 255);
  
  Serial.printf("[LED:%s] Rear Logic loaded: Green(40%%), Yellow(30%%), Red(20%%), Orange(10%%)\n", name_);
}

// ============================================================================
// SETTINGS
// ============================================================================

void LogicAnimations::setBrightness(uint8_t brightness) {
  brightness_ = brightness;
  Serial.printf("[LED:%s] Brightness set to %d\n", name_, brightness_);
}

void LogicAnimations::setSpeed(uint16_t interval_ms) {
  anim_speed_ms_ = constrain(interval_ms, 10, 1000);
  Serial.printf("[LED:%s] Speed set to %d ms\n", name_, anim_speed_ms_);
}

void LogicAnimations::setDensity(uint8_t density) {
  density_ = constrain(density, 1, 100);
  Serial.printf("[LED:%s] Density set to %d%%\n", name_, density_);
}

// ============================================================================
// ANIMATION DISPATCHER
// ============================================================================

void LogicAnimations::updateAnimation() {
  // Dispatch to appropriate animation function based on current mode
  switch (animation_) {
    case LogicAnimation::NORMAL:
      animNormal();
      break;
      
    case LogicAnimation::SOLIDCOLOR:
      animSolidColor();
      break;
      
    case LogicAnimation::FLASHCOLOR:
      animFlash();
      break;
      
    case LogicAnimation::LIGHTSOUT:
      animLightsOut();
      break;
    case LogicAnimation::ALARM:
      animAlarm();
      break;
      
    case LogicAnimation::FAILURE:
      animFailure();
      break;
      
    case LogicAnimation::FIRE:
      animFire();
      break;
      
    case LogicAnimation::RAINBOW:
      animRainbow();
      break;
      
    case LogicAnimation::ROAMINGPIXEL:
      animRoamingPixel();
      break;
      
    case LogicAnimation::REDALERT:
      animRedAlert();
      break;
      
    case LogicAnimation::COLORSWAP:
      animColorSwap();
      break;
      
    case LogicAnimation::FLIPFLOPCOLOR:
      animFlipFlopColor();
      break;

    // 2026-05 sled-ported animations
    case LogicAnimation::TWINKLE:    animTwinkle();    break;
    case LogicAnimation::PULSE:      animPulse();      break;
    case LogicAnimation::WIPE:       animWipe();       break;
    case LogicAnimation::CHASE:      animChase();      break;
    case LogicAnimation::SPARKLE:    animSparkle();    break;
    case LogicAnimation::HEARTBEAT:  animHeartbeat();  break;
    case LogicAnimation::MATRIXRAIN: animMatrixRain(); break;
    case LogicAnimation::SCROLLTEXT: animScrollText(); break;

    // Future animations will be added here
    default:
      // Unknown animation - fall back to NORMAL
      Serial.printf("[LED:%s] WARNING: Animation %d not implemented yet, using NORMAL\n", 
                    name_, (int)animation_);
      animNormal();
      break;
  }
}

// ============================================================================
// ANIMATION IMPLEMENTATIONS - PHASE 1
// ============================================================================

// ----------------------------------------------------------------------------
// NORMAL - Standard R2D2 Random Blinking
// ----------------------------------------------------------------------------
void LogicAnimations::animNormal() {
  // 1. INSTEAD OF strip_->clear(), we fade everything slightly
  // This value (20-60) determines how fast they fade out. 
  // Lower = longer trails, Higher = faster blinking.
  fadeAll(10); 
  
  // 2. Determine how many NEW pixels to spark this frame
  // Since we aren't clearing, we only need to add a few at a time
  uint16_t num_to_spark = (led_count_ * density_) / 500; // Reduced density for "sparks"
  if (num_to_spark < 1) num_to_spark = 1;
  
  uint16_t placed_pixels = 0;
  uint16_t attempts = 0;
  const uint16_t max_attempts = num_to_spark * 5;

  // 3. Place NEW random pixels
  while (placed_pixels < num_to_spark && attempts < max_attempts) {
    attempts++;
    uint8_t x = random(32); 
    uint8_t y = random(8);

    if (pattern.isActive(x, y)) {
      uint8_t color_slot = selectColorSlot();
      ColorRGB& selected_color = colors_[color_slot];
      uint8_t intensity = random(128, 256);
      
      uint8_t color_bright = color_brightness_[color_slot];
      uint8_t adj_r = (selected_color.r * color_bright) / 255;
      uint8_t adj_g = (selected_color.g * color_bright) / 255;
      uint8_t adj_b = (selected_color.b * color_bright) / 255;
      
      uint8_t r, g, b;
      applyColorWithBrightness(adj_r, adj_g, adj_b, intensity, r, g, b);
      
      uint16_t led_idx = pattern.xyToIndex(x, y);
      strip_->setPixelColor(led_idx, r, g, b);
      
      placed_pixels++;
    }
  }
  
  strip_->show();
}

// Helper function to fade all LEDs by a certain amount
void LogicAnimations::fadeAll(uint8_t fadeValue) {
  // Option 1: Fade only pattern LEDs (recommended for cleaner look)
  for (uint8_t y = 0; y < 8; y++) {
    for (uint8_t x = 0; x < 32; x++) {
      // Skip LEDs outside the pattern
      if (!pattern.isActive(x, y)) continue;
      
      uint16_t led_idx = pattern.xyToIndex(x, y);
      if (led_idx >= led_count_) continue;
      
      uint32_t color = strip_->getPixelColor(led_idx);
      
      // Extract R, G, B
      uint8_t r = (uint8_t)(color >> 16);
      uint8_t g = (uint8_t)(color >> 8);
      uint8_t b = (uint8_t)color;

      // Subtractive fading (prevents "wrapping" around to bright colors)
      r = (r > fadeValue) ? r - fadeValue : 0;
      g = (g > fadeValue) ? g - fadeValue : 0;
      b = (b > fadeValue) ? b - fadeValue : 0;

      strip_->setPixelColor(led_idx, r, g, b);
    }
  }
}

void LogicAnimations::fadeAll_WithClear(uint8_t fadeValue) {
  for (uint8_t y = 0; y < 8; y++) {
    for (uint8_t x = 0; x < 32; x++) {
      uint16_t led_idx = pattern.xyToIndex(x, y);
      if (led_idx >= led_count_) continue;
      
      if (pattern.isActive(x, y)) {
        // Fade LEDs inside pattern
        uint32_t color = strip_->getPixelColor(led_idx);
        
        uint8_t r = (uint8_t)(color >> 16);
        uint8_t g = (uint8_t)(color >> 8);
        uint8_t b = (uint8_t)color;

        r = (r > fadeValue) ? r - fadeValue : 0;
        g = (g > fadeValue) ? g - fadeValue : 0;
        b = (b > fadeValue) ? b - fadeValue : 0;

        strip_->setPixelColor(led_idx, r, g, b);
      } else {
        // Force LEDs outside pattern to black
        strip_->setPixelColor(led_idx, 0, 0, 0);
      }
    }
  }
}
// ----------------------------------------------------------------------------
// SOLIDCOLOR - Solid Color Display
// ----------------------------------------------------------------------------
void LogicAnimations::animSolidColor() {
  // 1. Start with a blank slate
  strip_->clear();
  
  // 2. Prepare the color (using your existing logic)
  ColorRGB& color = colors_[0];  // Use COLOR1
  uint8_t color_bright = color_brightness_[0];
  
  uint8_t adj_r = (color.r * color_bright) / 255;
  uint8_t adj_g = (color.g * color_bright) / 255;
  uint8_t adj_b = (color.b * color_bright) / 255;
  
  uint8_t r, g, b;
  applyColorWithBrightness(adj_r, adj_g, adj_b, 255, r, g, b);
  
  // 3. Loop through the grid and only light pixels inside the REAR_LOGIC pattern
  for (uint8_t y = 0; y < 8; y++) {
    for (uint8_t x = 0; x < 32; x++) {
      if (pattern.isActive(x, y)) {
        // Map the valid (x,y) to the physical vertical serpentine index
        uint16_t led_idx = pattern.xyToIndex(x, y);
        strip_->setPixelColor(led_idx, r, g, b);
      }
    }
  }
  
  strip_->show();
}

// ----------------------------------------------------------------------------
// FLASH - Flashing On/Off
// ----------------------------------------------------------------------------
void LogicAnimations::animFlash() {
  // Alternate between the pattern-on and all-off
  
  // Toggle state every update
  anim_state_.flip_state = !anim_state_.flip_state;
  
  // Always clear first to ensure the "OFF" state is clean 
  // and that the "ON" state doesn't bleed outside the pattern
  strip_->clear();
  
  if (anim_state_.flip_state) {
    // ON state - use COLOR1
    ColorRGB& color = colors_[0];
    uint8_t color_bright = color_brightness_[0];
    
    uint8_t adj_r = (color.r * color_bright) / 255;
    uint8_t adj_g = (color.g * color_bright) / 255;
    uint8_t adj_b = (color.b * color_bright) / 255;
    
    uint8_t r, g, b;
    applyColorWithBrightness(adj_r, adj_g, adj_b, 255, r, g, b);
    
    // Fill ONLY the pixels defined in your REAR_LOGIC pattern
    for (uint8_t y = 0; y < 8; y++) {
      for (uint8_t x = 0; x < 32; x++) {
        if (pattern.isActive(x, y)) {
          uint16_t led_idx = pattern.xyToIndex(x, y);
          strip_->setPixelColor(led_idx, r, g, b);
        }
      }
    }
  } 
  // If flip_state is false, the strip remains cleared from the strip_->clear() above
  
  strip_->show();
}

// ----------------------------------------------------------------------------
// LIGHTSOUT - All Off (Standby)
// ----------------------------------------------------------------------------
void LogicAnimations::animLightsOut() {
  // Simply keep all LEDs off
  // This is the most power-efficient mode
  
  // Only clear once, not every update (save CPU cycles)
  if (anim_state_.step == 0) {
    strip_->clear();
    strip_->show();
    anim_state_.step = 1;  // Mark as initialized
  }
  
  // Do nothing on subsequent updates - already off!
  // This saves power and CPU cycles
}

// ============================================================================
// HELPER FUNCTIONS
// ============================================================================

uint8_t LogicAnimations::selectColorSlot() {
  // Select color based on weighted random distribution
  // Weights are relative probabilities
  
  uint8_t rand_val = random(100);
  uint8_t cumulative = 0;
  
  for (uint8_t i = 0; i < 4; i++) {
    cumulative += weights_[i];
    if (rand_val < cumulative) {
      return i;
    }
  }
  
  return 0;  // Fallback to COLOR1
}

void LogicAnimations::applyColorWithBrightness(uint8_t r, uint8_t g, uint8_t b,
                                                   uint8_t intensity,
                                                   uint8_t& out_r, uint8_t& out_g, uint8_t& out_b) {
  // Apply intensity and global brightness
  // Note: Per-color brightness is already applied by caller
  
  uint32_t scale = (uint32_t)intensity * brightness_;
  out_r = (r * scale) / 65025;  // 65025 = 255 * 255
  out_g = (g * scale) / 65025;
  out_b = (b * scale) / 65025;
}

// ============================================================================
// COMMAND HANDLING
// ============================================================================

bool LogicAnimations::handleCommand(const String& cmd) {
  String upper = cmd;
  upper.toUpperCase();
  upper.trim();

  // 1. HELP & STATUS
  if (upper == "HELP" || upper == "?") {
    Serial.println(getHelpMenu());
    return true;
  }
  if (upper == "STATUS") {
    Serial.println(getStatus());
    return true;
  }

  // 2. POWER CONTROL
  if (upper == "ON") { enable(true); return true; }
  if (upper == "OFF") { enable(false); return true; }

  // 3. BRIGHTNESS (Global: BRIGHT | Per-Color: BRIGHT1-4)
  if (upper.startsWith("BRIGHT ")) {
    setBrightness(upper.substring(7).toInt());
    return true;
  }
  for (uint8_t i = 1; i <= 4; i++) {
    String prefix = "BRIGHT" + String(i) + " ";
    if (upper.startsWith(prefix)) {
      setColorBrightness(i, upper.substring(prefix.length()).toInt());
      return true;
    }
  }

  // 4. ANIMATION SELECTION (Directly by name)
  LogicAnimation anim;
  if (parseAnimationName(upper.c_str(), anim)) {
    setAnimation(anim);
    return true;
  }

  // 5. COLOR & WEIGHTS
  for (uint8_t i = 1; i <= 4; i++) {
    String c_pre = "COLOR" + String(i) + " ";
    String w_pre = "WEIGHT" + String(i) + " ";
    
    if (upper.startsWith(c_pre)) {
      String val = cmd.substring(c_pre.length());
      val.trim();
      // Handle RGB "R G B" or Name "RED"
      if (val.indexOf(' ') > 0) {
        int r, g, b;
        if (sscanf(val.c_str(), "%d %d %d", &r, &g, &b) == 3) {
          setColor(i, r, g, b);
        }
      } else {
        setColor(i, val.c_str());
      }
      return true;
    }
    
    if (upper.startsWith(w_pre)) {
      setWeight(i, upper.substring(w_pre.length()).toInt());
      return true;
    }
  }

  // 6. GLOBAL PARAMETERS
  if (upper.startsWith("SPEED ")) { setSpeed(upper.substring(6).toInt()); return true; }
  if (upper.startsWith("DENSITY ")) { setDensity(upper.substring(8).toInt()); return true; }

  return false;
}
String LogicAnimations::getHelpMenu() {
  String h = "\n=== " + String(name_) + " HELP ===\n";
  h += "Usage: LED " + String(name_) + " <COMMAND>\n\n";
  h += "ANIMATIONS: NORMAL, ALARM, FAILURE, FIRE, RAINBOW, ROAMINGPIXEL,\n";
  h += "            REDALERT, COLORSWAP, FLIPFLOPCOLOR, SOLIDCOLOR, FLASH\n\n";
  h += "COMMANDS:\n";
  h += "  STATUS             - Show current settings\n";
  h += "  BRIGHT <0-255>     - Set Global Brightness\n";
  h += "  BRIGHT1-4 <0-255>  - Set Color 1-4 Brightness\n";
  h += "  COLOR1-4 <Name|RGB>- Set Palette Color\n";
  h += "  WEIGHT1-4 <0-100>  - Color probability weight\n";
  h += "  SPEED <ms>         - Update interval\n";
  h += "  DENSITY <1-100>    - Pixel density\n";
  h += "==========================\n";
  return h;
}

String LogicAnimations::getStatus() {
  String s = String("[LED:") + name_ + "] Enhanced Logic Status\n";
  s += "  Enabled: " + String(enabled_ ? "YES" : "NO") + "\n";
  s += "  Animation: " + String(getAnimationName(animation_)) + "\n";
  s += "  Global Brightness: " + String(brightness_) + "\n";
  s += "  Speed: " + String(anim_speed_ms_) + " ms\n";
  s += "  Density: " + String(density_) + "%\n";
  
  // Show all 4 colors with weights AND per-color brightness
  for (uint8_t i = 0; i < 4; i++) {
    uint8_t r, g, b;
    getColor(i + 1, r, g, b);
    s += "  COLOR" + String(i + 1) + ": RGB(" + String(r) + "," + String(g) + "," + String(b) + ")";
    s += " [" + String(getColorName(r, g, b)) + "]";
    s += " Weight:" + String(weights_[i]) + "%";
    s += " Bright:" + String(color_brightness_[i]) + "\n";
  }
  
  return s;
}

void LogicAnimations::animAlarm() {
  // Fast chaotic alarm (50% density, high intensity) 
  // Restricted to the REAR_LOGIC pattern
  strip_->clear();
  
  // We calculate 50% density based on the total matrix size
  uint16_t num_pixels = (led_count_ * 50) / 100;
  if (num_pixels < 1) num_pixels = 1;
  
  uint16_t placed_pixels = 0;
  uint16_t attempts = 0;
  // Safety exit to prevent infinite loops if the pattern is very small
  uint16_t max_attempts = num_pixels * 5; 

  while (placed_pixels < num_pixels && attempts < max_attempts) {
    attempts++;
    
    // Pick random coordinates
    uint8_t x = random(32);
    uint8_t y = random(8);
    
    // Only light up if it's inside our shape
    if (pattern.isActive(x, y)) {
      uint8_t intensity = random(200, 256);  // Very bright!
      
      uint8_t color_slot = selectColorSlot();
      ColorRGB& selected_color = colors_[color_slot];
      
      uint8_t color_bright = color_brightness_[color_slot];
      uint8_t adj_r = (selected_color.r * color_bright) / 255;
      uint8_t adj_g = (selected_color.g * color_bright) / 255;
      uint8_t adj_b = (selected_color.b * color_bright) / 255;
      
      uint8_t r, g, b;
      applyColorWithBrightness(adj_r, adj_g, adj_b, intensity, r, g, b);
      
      // Use the vertical serpentine mapper
      uint16_t led_idx = pattern.xyToIndex(x, y);
      strip_->setPixelColor(led_idx, r, g, b);
      
      placed_pixels++;
    }
  }
  
  strip_->show();
}

void LogicAnimations::animFailure() {
  // Slow stuttering failure (5-15% density, dim)
  if (anim_state_.substep == 0 || random(100) < 30) {
    strip_->clear();
    
    uint16_t num_pixels = (led_count_ * random(5, 15)) / 100;
    if (num_pixels < 1) num_pixels = 1;
    
    for (uint16_t i = 0; i < num_pixels; i++) {
      uint16_t idx = random(led_count_);
      uint8_t intensity = random(64, 128);  // Dim
      
      uint8_t color_slot = random(2, 4);  // Use COLOR3/COLOR4
      ColorRGB& selected_color = colors_[color_slot];
      
      uint8_t color_bright = color_brightness_[color_slot];
      uint8_t adj_r = (selected_color.r * color_bright) / 255;
      uint8_t adj_g = (selected_color.g * color_bright) / 255;
      uint8_t adj_b = (selected_color.b * color_bright) / 255;
      
      uint8_t r, g, b;
      applyColorWithBrightness(adj_r, adj_g, adj_b, intensity, r, g, b);
      
      strip_->setPixelColor(idx, r, g, b);
    }
    
    strip_->show();
    anim_state_.substep++;
  }
}

// Proper 2D matrix fire: each of the 32 columns is an independent vertical
// flame (Fire2012-style) that sparks at the bottom row and rises. heat[] is
// treated as [x*8 + i] where i=0 is the flame BASE and i=7 the top; physical
// row y = 7 - i so flames rise from the bottom of the panel. Honours the mask.
void LogicAnimations::animFire() {
  const uint8_t  W = 32, Hh = 8;
  const uint16_t cells = (uint16_t)W * Hh;            // 256

  if (anim_state_.heat == nullptr || anim_state_.heat_size != cells) {
    if (anim_state_.heat) delete[] anim_state_.heat;
    anim_state_.heat = new uint8_t[cells];
    anim_state_.heat_size = cells;
    memset(anim_state_.heat, 0, cells);
  }
  uint8_t* heat = anim_state_.heat;
  ColorRGB& base = colors_[0];
  const uint8_t cb = color_brightness_[0];

  strip_->clear();
  for (uint8_t x = 0; x < W; x++) {
    uint8_t* col = heat + (uint16_t)x * Hh;            // col[0]=base .. col[7]=top

    // 1) cool each cell a little
    for (uint8_t i = 0; i < Hh; i++) {
      uint8_t c = random(0, 28);
      col[i] = (col[i] > c) ? col[i] - c : 0;
    }
    // 2) heat drifts upward — each upper cell averages the two below it
    for (int i = Hh - 1; i >= 2; i--)
      col[i] = (uint8_t)(((int)col[i-1] + col[i-2] + col[i-2]) / 3);
    // 3) spark at the base
    if (random(255) < 150) {
      int v = col[0] + random(160, 256);
      col[0] = (v > 255) ? 255 : v;
    }
    // 4) render: base (i=0) -> physical bottom row (y = Hh-1)
    for (uint8_t i = 0; i < Hh; i++) {
      uint8_t y = (Hh - 1) - i;
      if (!pattern.isActive(x, y)) continue;
      uint16_t idx = pattern.xyToIndex(x, y);
      if (idx >= led_count_) continue;
      uint8_t t = col[i], r, g, b;
      if (t < 85)       { r = (base.r*t*3)/255; g = (base.g*t*3)/255; b = (base.b*t*3)/255; }
      else if (t < 170) { uint8_t k=t-85;  r = base.r; g = base.g + ((255-base.g)*k*3)/255; b = base.b; }
      else              { uint8_t k=t-170; r = 255;    g = 255;       b = base.b + ((255-base.b)*k*3)/255; }
      r = (uint16_t)r * cb * brightness_ / 65025;
      g = (uint16_t)g * cb * brightness_ / 65025;
      b = (uint16_t)b * cb * brightness_ / 65025;
      strip_->setPixelColor(idx, r, g, b);
    }
  }
  strip_->show();
}

void LogicAnimations::animRainbow() {
  // Prevent overflow - wrap step counter at 65535
  anim_state_.step = (anim_state_.step + 1) & 0xFFFF;
  
  // Clear ALL LEDs first to prevent ghosting
  strip_->clear();
  
  // Only iterate through active pattern coordinates
  for (uint8_t y = 0; y < 8; y++) {
    for (uint8_t x = 0; x < 32; x++) {
      // Check if this coordinate is in the active pattern
      if (!pattern.isActive(x, y)) continue;
      
      // Get the physical LED index
      uint16_t led_idx = pattern.xyToIndex(x, y);
      
      // Safety check - skip if out of bounds
      if (led_idx >= led_count_) continue;
      
      // Create smooth rainbow hue based on position and time
      // Using linear position along the strip for smooth gradient
      // Calculate position as percentage through active LEDs (0-255)
      uint8_t position = (x * 8 + y);  // 0-255 range for 8x32 matrix
      
      // Create smooth hue that shifts over time
      // Position creates the rainbow gradient, step makes it flow
      uint16_t hue = ((uint32_t)position * 256) + (anim_state_.step * 32);
      hue = hue & 0xFFFF;  // Keep in valid range
      
      // Convert HSV to RGB
      uint8_t r, g, b;
      hsvToRgb(hue, 255, 255, r, g, b);
      
      // Apply global brightness
      r = (r * brightness_) / 255;
      g = (g * brightness_) / 255;
      b = (b * brightness_) / 255;
      
      // Set the pixel
      strip_->setPixelColor(led_idx, r, g, b);
    }
  }
  
  strip_->show();
}


void LogicAnimations::animRoamingPixel() {
  strip_->clear();
  ColorRGB& scan_color = colors_[0];
  
  // Use 'step' as the X coordinate (0-31)
  int16_t centerX = anim_state_.step;

  for (int16_t xOffset = -2; xOffset <= 2; xOffset++) {
    int16_t targetX = centerX + xOffset;
    if (targetX >= 0 && targetX < 32) {
      uint8_t trail_bright = 255 - (abs(xOffset) * 100); // Sharp falloff
      
      for (uint8_t y = 0; y < 8; y++) {
        if (pattern.isActive(targetX, y)) {
          uint16_t led_idx = pattern.xyToIndex(targetX, y);
          uint8_t r, g, b;
          applyColorWithBrightness(scan_color.r, scan_color.g, scan_color.b, trail_bright, r, g, b);
          strip_->setPixelColor(led_idx, r, g, b);
        }
      }
    }
  }

  anim_state_.step += anim_state_.direction;
  if (anim_state_.step >= 31 || anim_state_.step <= 0) anim_state_.direction *= -1;
  strip_->show();
}

void LogicAnimations::animRedAlert() {
  // Urgent red alert flashing - only inside the pattern
  anim_state_.flip_state = !anim_state_.flip_state;
  
  // Start with a clean slate (essential for the pattern mask)
  strip_->clear();
  
  ColorRGB& alert_color = colors_[0];
  uint8_t color_bright = color_brightness_[0];
  
  // Calculate the color components once per frame
  uint8_t r, g, b;
  if (anim_state_.flip_state) {
    // Bright state
    r = (alert_color.r * color_bright * brightness_) / 65025;
    g = (alert_color.g * color_bright * brightness_) / 65025;
    b = (alert_color.b * color_bright * brightness_) / 65025;
  } else {
    // Dim state (20%)
    r = (alert_color.r * color_bright * brightness_ * 20) / 6502500;
    g = (alert_color.g * color_bright * brightness_ * 20) / 6502500;
    b = (alert_color.b * color_bright * brightness_ * 20) / 6502500;
  }
  
  // Apply the color only to pixels within the pattern
  for (uint8_t y = 0; y < 8; y++) {
    for (uint8_t x = 0; x < 32; x++) {
      // Check your REAR_LOGIC pattern
      if (pattern.isActive(x, y)) {
        // Map the grid coordinate to your physical vertical serpentine LED ID
        uint16_t led_idx = pattern.xyToIndex(x, y);
        strip_->setPixelColor(led_idx, r, g, b);
      }
    }
  }
  
  strip_->show();
}

void LogicAnimations::animColorSwap() {
  // Alternate between COLOR1 and COLOR2 inside the pattern
  anim_state_.flip_state = !anim_state_.flip_state;
  
  // Clear strip so background stays dark
  strip_->clear();
  
  uint8_t color_idx = anim_state_.flip_state ? 0 : 1;
  ColorRGB& swap_color = colors_[color_idx];
  uint8_t color_bright = color_brightness_[color_idx];
  
  uint8_t r = (swap_color.r * color_bright * brightness_) / 65025;
  uint8_t g = (swap_color.g * color_bright * brightness_) / 65025;
  uint8_t b = (swap_color.b * color_bright * brightness_) / 65025;
  
  for (uint8_t y = 0; y < 8; y++) {
    for (uint8_t x = 0; x < 32; x++) {
      if (pattern.isActive(x, y)) {
        uint16_t led_idx = pattern.xyToIndex(x, y);
        strip_->setPixelColor(led_idx, r, g, b);
      }
    }
  }
  
  strip_->show();
}

void LogicAnimations::animFlipFlopColor() {
  // Split display flip-flop (Left half vs Right half)
  anim_state_.flip_state = !anim_state_.flip_state;
  
  strip_->clear();
  
  for (uint8_t y = 0; y < 8; y++) {
    for (uint8_t x = 0; x < 32; x++) {
      if (pattern.isActive(x, y)) {
        // Use the X coordinate (0-31) to determine the half
        bool first_half = (x < 16); 
        
        uint8_t color_idx = (first_half ^ anim_state_.flip_state) ? 0 : 1;
        ColorRGB& pixel_color = colors_[color_idx];
        uint8_t color_bright = color_brightness_[color_idx];
        
        uint8_t r = (pixel_color.r * color_bright * brightness_) / 65025;
        uint8_t g = (pixel_color.g * color_bright * brightness_) / 65025;
        uint8_t b = (pixel_color.b * color_bright * brightness_) / 65025;
        
        uint16_t led_idx = pattern.xyToIndex(x, y);
        strip_->setPixelColor(led_idx, r, g, b);
      }
    }
  }
  
  strip_->show();
}

void LogicAnimations::hsvToRgb(uint16_t h, uint8_t s, uint8_t v, uint8_t& r, uint8_t& g, uint8_t& b) {
  if (s == 0) {
    r = g = b = v;
    return;
  }
  
  uint8_t region = h / 10923;
  uint16_t remainder = (h % 10923) * 6; // Scale to 0-65535
  
  uint8_t p = (v * (255 - s)) >> 8;
  // Use uint32_t for the multiplication to prevent overflow flickering
  uint8_t q = (v * (255 - ((uint32_t)s * remainder >> 16))) >> 8;
  uint8_t t = (v * (255 - ((uint32_t)s * (65535 - remainder) >> 16))) >> 8;
  
  switch (region) {
    case 0: r = v; g = t; b = p; break;
    case 1: r = q; g = v; b = p; break;
    case 2: r = p; g = v; b = t; break;
    case 3: r = p; g = q; b = v; break;
    case 4: r = t; g = p; b = v; break;
    default: r = v; g = p; b = q; break;
  }
}

/**
 * Maps a string name (from Serial command) to the LogicAnimation enum.
 * Supports the simplified "LED RL <ANIM_NAME>" structure.
 */
bool LogicAnimations::parseAnimationName(const char* name, LogicAnimation& anim) {
  String n = String(name);
  n.toUpperCase();
  n.trim();

  if (n == "NORMAL")        { anim = LogicAnimation::NORMAL; return true; }
  if (n == "ALARM")         { anim = LogicAnimation::ALARM; return true; }
  if (n == "FAILURE")       { anim = LogicAnimation::FAILURE; return true; }
  if (n == "FIRE")          { anim = LogicAnimation::FIRE; return true; }
  if (n == "RAINBOW")       { anim = LogicAnimation::RAINBOW; return true; }
  if (n == "ROAMINGPIXEL")  { anim = LogicAnimation::ROAMINGPIXEL; return true; }
  if (n == "REDALERT")      { anim = LogicAnimation::REDALERT; return true; }
  if (n == "COLORSWAP")     { anim = LogicAnimation::COLORSWAP; return true; }
  if (n == "FLIPFLOPCOLOR") { anim = LogicAnimation::FLIPFLOPCOLOR; return true; }
  if (n == "SOLIDCOLOR")    { anim = LogicAnimation::SOLIDCOLOR; return true; }
  if (n == "FLASHCOLOR" || n == "FLASH") { anim = LogicAnimation::FLASHCOLOR; return true; }
  if (n == "LIGHTSOUT")     { anim = LogicAnimation::LIGHTSOUT; return true; }

  return false;
}

// ============================================================================
// SLED-PORTED ANIMATIONS (2026-05) — remapped to the 8x32 matrix.
// All honour the active `pattern` mask and global brightness_, and self-pace
// off anim_speed_ms_ (one frame per update() tick). Colours use COLOR1/COLOR2.
// ============================================================================

// TWINKLE — soft random multicolour shimmer with gentle fade (the favourite).
void LogicAnimations::animTwinkle() {
  fadeAll(16);                                   // soft trails
  uint16_t num = (led_count_ * density_) / 400;  // a few new twinkles per frame
  if (num < 1) num = 1;
  for (uint16_t i = 0; i < num; i++) {
    uint8_t x = random(32), y = random(8);
    if (!pattern.isActive(x, y)) continue;
    uint8_t r, g, b;
    hsvToRgb((uint16_t)random(0, 65536), 255, 255, r, g, b);  // random hue
    r = (uint16_t)r * brightness_ / 255;
    g = (uint16_t)g * brightness_ / 255;
    b = (uint16_t)b * brightness_ / 255;
    strip_->setPixelColor(pattern.xyToIndex(x, y), r, g, b);
  }
  strip_->show();
}

// PULSE — whole masked panel breathes COLOR1 in a smooth sine.
void LogicAnimations::animPulse() {
  anim_state_.step = (anim_state_.step + 1) % 256;
  float s = (sinf(((float)anim_state_.step / 256.0f) * 2.0f * PI - PI / 2.0f) + 1.0f) * 0.5f;
  float lvl = 0.10f + s * 0.90f;                 // 10% glow floor
  ColorRGB& c = colors_[0];
  uint8_t r = (uint8_t)(((uint16_t)c.r * brightness_ / 255) * lvl);
  uint8_t g = (uint8_t)(((uint16_t)c.g * brightness_ / 255) * lvl);
  uint8_t b = (uint8_t)(((uint16_t)c.b * brightness_ / 255) * lvl);
  strip_->clear();
  for (uint8_t y = 0; y < 8; y++)
    for (uint8_t x = 0; x < 32; x++)
      if (pattern.isActive(x, y)) strip_->setPixelColor(pattern.xyToIndex(x, y), r, g, b);
  strip_->show();
}

// WIPE — fill the matrix column-by-column with COLOR1, then restart.
void LogicAnimations::animWipe() {
  uint8_t col = anim_state_.step;
  if (col == 0) strip_->clear();
  ColorRGB& c = colors_[0];
  uint8_t r = (uint16_t)c.r * brightness_ / 255;
  uint8_t g = (uint16_t)c.g * brightness_ / 255;
  uint8_t b = (uint16_t)c.b * brightness_ / 255;
  if (col < 32)
    for (uint8_t y = 0; y < 8; y++)
      if (pattern.isActive(col, y)) strip_->setPixelColor(pattern.xyToIndex(col, y), r, g, b);
  strip_->show();
  anim_state_.step++;
  if (anim_state_.step > 33) anim_state_.step = 0;   // brief full-hold then restart
}

// CHASE — theatre chase: every 3rd column COLOR1, the rest COLOR2, marching.
void LogicAnimations::animChase() {
  uint8_t phase = anim_state_.step % 3;
  ColorRGB& c1 = colors_[0];
  ColorRGB& c2 = colors_[1];
  for (uint8_t x = 0; x < 32; x++) {
    ColorRGB c = (((x + phase) % 3) == 0) ? c1 : c2;
    uint8_t r = (uint16_t)c.r * brightness_ / 255;
    uint8_t g = (uint16_t)c.g * brightness_ / 255;
    uint8_t b = (uint16_t)c.b * brightness_ / 255;
    for (uint8_t y = 0; y < 8; y++)
      if (pattern.isActive(x, y)) strip_->setPixelColor(pattern.xyToIndex(x, y), r, g, b);
  }
  strip_->show();
  anim_state_.step++;
}

// SPARKLE — sharp COLOR1 pops over a fast-decaying field (electric crackle).
void LogicAnimations::animSparkle() {
  fadeAll(40);                       // fast fade
  uint16_t num = 2 + random(3);      // 2..4 sparks/frame
  ColorRGB& c = colors_[0];
  uint8_t r = (uint16_t)c.r * brightness_ / 255;
  uint8_t g = (uint16_t)c.g * brightness_ / 255;
  uint8_t b = (uint16_t)c.b * brightness_ / 255;
  for (uint16_t i = 0; i < num; i++) {
    uint8_t x = random(32), y = random(8);
    if (!pattern.isActive(x, y)) continue;
    strip_->setPixelColor(pattern.xyToIndex(x, y), r, g, b);
  }
  strip_->show();
}

// HEARTBEAT — double "lub-dub" brightness throb on COLOR1.
void LogicAnimations::animHeartbeat() {
  anim_state_.step = (anim_state_.step + 1) % 120;
  float t  = (float)anim_state_.step / 120.0f;
  float b1 = expf(-powf((t - 0.08f) / 0.045f, 2.0f));        // strong first beat
  float b2 = 0.6f * expf(-powf((t - 0.28f) / 0.045f, 2.0f)); // softer second beat
  float lvl = 0.04f + (b1 > b2 ? b1 : b2) * 0.96f;
  ColorRGB& c = colors_[0];
  uint8_t r = (uint8_t)(((uint16_t)c.r * brightness_ / 255) * lvl);
  uint8_t g = (uint8_t)(((uint16_t)c.g * brightness_ / 255) * lvl);
  uint8_t b = (uint8_t)(((uint16_t)c.b * brightness_ / 255) * lvl);
  strip_->clear();
  for (uint8_t y = 0; y < 8; y++)
    for (uint8_t x = 0; x < 32; x++)
      if (pattern.isActive(x, y)) strip_->setPixelColor(pattern.xyToIndex(x, y), r, g, b);
  strip_->show();
}

// MATRIXRAIN — bright COLOR1 heads fall down each column, leaving fading trails.
void LogicAnimations::animMatrixRain() {
  static int8_t dropY[32];
  static bool   rain_init = false;
  if (!rain_init) { for (uint8_t i = 0; i < 32; i++) dropY[i] = -1; rain_init = true; }

  fadeAll(40);   // trails
  ColorRGB& c = colors_[0];
  uint8_t r = (uint16_t)c.r * brightness_ / 255;
  uint8_t g = (uint16_t)c.g * brightness_ / 255;
  uint8_t b = (uint16_t)c.b * brightness_ / 255;

  for (uint8_t x = 0; x < 32; x++) {
    if (dropY[x] >= 0) {
      uint8_t y = (uint8_t)dropY[x];
      if (pattern.isActive(x, y)) strip_->setPixelColor(pattern.xyToIndex(x, y), r, g, b);
      dropY[x]++;
      if (dropY[x] >= 8) dropY[x] = -1;       // fell off the bottom
    } else if (random(100) < 8) {
      dropY[x] = 0;                            // spawn a new drop at the top
    }
  }
  strip_->show();
}

// ----------------------------------------------------------------------------
// Mirror the current frame to a Print stream (the WebSerial editor's preview).
// Emits "F:" followed by 8*32 pixels in (y,x) row-major order, each as 3 hex
// nibbles (RGB444 — plenty for a preview, half the bytes of full RGB888 so it
// stays inside the 115200-baud budget at ~10 fps). No physical LEDs needed:
// this reads the in-RAM NeoPixel buffer the animation just computed.
// ----------------------------------------------------------------------------
void LogicAnimations::emitMirrorFrame(Print& out) {
  if (!strip_) return;
  static const char HEXN[] = "0123456789abcdef";
  out.print("F:");
  for (uint8_t y = 0; y < 8; y++) {
    for (uint8_t x = 0; x < 32; x++) {
      uint16_t idx = pattern.xyToIndex(x, y);
      uint32_t c = (idx < led_count_) ? strip_->getPixelColor(idx) : 0;
      out.write(HEXN[(uint8_t)((c >> 16) & 0xFF) >> 4]);   // R hi-nibble
      out.write(HEXN[(uint8_t)((c >>  8) & 0xFF) >> 4]);   // G hi-nibble
      out.write(HEXN[(uint8_t)( c        & 0xFF) >> 4]);   // B hi-nibble
    }
  }
  out.println();
}

// Blocking confirm flash: fade the masked region up then down, `times` times.
// Brief (~0.8s) — used as a UI acknowledgement. The running animation resumes
// on the next update() afterward.
void LogicAnimations::confirmPulse(uint8_t r, uint8_t g, uint8_t b, uint8_t times) {
  if (!strip_) return;
  for (uint8_t t = 0; t < times; t++) {
    for (int phase = 0; phase < 2; phase++) {
      for (int s = 0; s <= 255; s += 28) {
        int lvl = phase ? (255 - s) : s;
        uint8_t rr = (uint16_t)r * lvl / 255;
        uint8_t gg = (uint16_t)g * lvl / 255;
        uint8_t bb = (uint16_t)b * lvl / 255;
        strip_->clear();
        for (uint8_t y = 0; y < 8; y++)
          for (uint8_t x = 0; x < 32; x++)
            if (pattern.isActive(x, y)) strip_->setPixelColor(pattern.xyToIndex(x, y), rr, gg, bb);
        strip_->show();
        delay(10);
      }
    }
  }
  strip_->clear();
  strip_->show();
}

// ----------------------------------------------------------------------------
// SCROLLTEXT — render a column bitmap (1 byte/col, bit y = row y) authored in
// the editor. Scrolls right→left, or centres it statically if scroll is off
// and it fits in 32 columns.
// ----------------------------------------------------------------------------
void LogicAnimations::setText(const uint8_t* bmp, uint16_t cols, uint32_t color, bool scroll) {
  if (textBmp_) { delete[] textBmp_; textBmp_ = nullptr; }
  textCols_   = cols;
  textColor_  = color;
  textScroll_ = scroll;
  if (cols > 0) { textBmp_ = new uint8_t[cols]; memcpy(textBmp_, bmp, cols); }
  anim_state_.step = 0;
}

void LogicAnimations::animScrollText() {
  strip_->clear();
  if (!textBmp_ || textCols_ == 0) { strip_->show(); return; }

  uint8_t r = (uint16_t)((textColor_ >> 16) & 0xFF) * brightness_ / 255;
  uint8_t g = (uint16_t)((textColor_ >>  8) & 0xFF) * brightness_ / 255;
  uint8_t b = (uint16_t)( textColor_        & 0xFF) * brightness_ / 255;

  bool scroll = textScroll_ || textCols_ > 32;   // force scroll if it can't fit
  int  total  = (int)textCols_ + 32;             // trailing blank gap = panel width
  int  off    = scroll ? 0 : (32 - (int)textCols_) / 2;

  for (uint8_t x = 0; x < 32; x++) {
    int src;
    if (scroll) { src = (int)anim_state_.step + x; if (src >= total) src -= total; }
    else        { src = (int)x - off; }
    if (src < 0 || src >= (int)textCols_) continue;     // blank (gap / margins)

    uint8_t colByte = textBmp_[src];
    for (uint8_t y = 0; y < 8; y++) {
      if (colByte & (1 << y)) {
        uint16_t idx = pattern.xyToIndex(x, y);
        if (idx < led_count_) strip_->setPixelColor(idx, r, g, b);
      }
    }
  }

  if (scroll) { anim_state_.step++; if ((int)anim_state_.step >= total) anim_state_.step = 0; }
  strip_->show();
}