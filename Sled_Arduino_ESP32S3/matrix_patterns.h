#pragma once
/**
 * matrix_patterns.h
 * -----------------
 * LED Matrix Coordinate Mapping and Pattern System
 * 
 * Features:
 *   - XY coordinate to LED index mapping (handles serpentine/zigzag)
 *   - Pattern definition system (which LEDs are active)
 *   - Visual ASCII pattern editor
 *   - Multiple preset patterns
 * 
 * Your Matrix: 8 rows × 32 columns = 256 LEDs
 * Layout: SERPENTINE (every other row reversed)
 * 
 * Example:
 *   Row 0: →→→  0,  1,  2, ..., 15, 16, 17, ..., 31
 *   Row 1: ←←← 47, 46, 45, ..., 32, 31, 30, ..., 16  (REVERSED!)
 *   Row 2: →→→ 48, 49, 50, ..., 63, 64, 65, ..., 79
 *   Row 3: ←←← 95, 94, 93, ..., 80, 79, 78, ..., 64  (REVERSED!)
 *   etc.
 */

#include <Arduino.h>

// ============================================================================
// MATRIX COORDINATE MAPPER
// ============================================================================

class MatrixMapper {
public:
  MatrixMapper(uint8_t rows, uint8_t cols, bool serpentine = true)
    : rows_(rows), cols_(cols), serpentine_(serpentine), flipX_(false), flipY_(false) {}

  // Mirror the logical coordinate system to match how the panel is physically
  // mounted (so logical column 0 / row 0 land where you expect on the panel).
  void setFlip(bool fx, bool fy) { flipX_ = fx; flipY_ = fy; }

  // Convert XY coordinates to LED strip index (applying any flip first).
  uint16_t xyToIndex(uint8_t x, uint8_t y) const {
    if (x >= cols_ || y >= rows_) return 0xFFFF;  // Out of bounds
    uint8_t ax = flipX_ ? (cols_ - 1 - x) : x;
    uint8_t ay = flipY_ ? (rows_ - 1 - y) : y;

    uint16_t index;
    if (serpentine_ && (ax & 1)) {
      // Odd physical columns: bottom-to-top (reversed)
      index = (ax * rows_) + (rows_ - 1 - ay);
    } else {
      // Even physical columns: top-to-bottom (normal)
      index = (ax * rows_) + ay;
    }
    return index;
  }
  
  // Convert LED strip index to XY coordinates
  void indexToXY(uint16_t index, uint8_t& x, uint8_t& y) const {
    if (index >= rows_ * cols_) {
      x = y = 0xFF;
      return;
    }
    
    x = index / rows_;
    uint8_t rowOffset = index % rows_;
    
    if (serpentine_ && (x & 1)) {
      // Odd columns: reversed
      y = rows_ - 1 - rowOffset;
    } else {
      // Even columns: normal
      y = rowOffset;
    }
  }
  
  uint16_t size() const { return rows_ * cols_; }
  uint8_t rows() const { return rows_; }
  uint8_t cols() const { return cols_; }
  
private:
  uint8_t rows_;
  uint8_t cols_;
  bool serpentine_;
  bool flipX_;
  bool flipY_;
};

// ============================================================================
// PATTERN DEFINITION
// ============================================================================

class MatrixPattern {
public:
  // Constructor with pattern array
  MatrixPattern(uint8_t rows, uint8_t cols, const bool* pattern, bool serpentine = true)
    : mapper_(rows, cols, serpentine), pattern_(pattern) {}
  
  // Check if LED at (x, y) is active in pattern
  bool isActive(uint8_t x, uint8_t y) const {
    if (x >= mapper_.cols() || y >= mapper_.rows()) return false;
    return pattern_[y * mapper_.cols() + x];
  }
  
  // Check if LED index is active
  bool isActiveIndex(uint16_t index) const {
    uint8_t x, y;
    mapper_.indexToXY(index, x, y);
    if (x == 0xFF) return false;
    return isActive(x, y);
  }
  
  // Get LED index from (x, y)
  uint16_t xyToIndex(uint8_t x, uint8_t y) const {
    return mapper_.xyToIndex(x, y);
  }

  // Mirror to match physical mounting (passes through to the mapper).
  void setFlip(bool fx, bool fy) { mapper_.setFlip(fx, fy); }

  // Get mapper reference
  const MatrixMapper& mapper() const { return mapper_; }
  
private:
  MatrixMapper mapper_;
  const bool* pattern_;
};

// ============================================================================
// PATTERN PRESETS FOR YOUR 8×32 MATRIX
// ============================================================================

namespace MatrixPatterns {

// ============================================================================
// PATTERN: LEFT HALF (Your green highlighted area from screenshot)
// ============================================================================
// Rows 0-7, Columns 0-15 active
// This matches your green-highlighted region
const bool LEFT_HALF[8][32] = {
  {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1, 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
  {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1, 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
  {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1, 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
  {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1, 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
  {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1, 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
  {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1, 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
  {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1, 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
  {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1, 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}
};

// ============================================================================
// PATTERN: RIGHT HALF
// ============================================================================
const bool RIGHT_HALF[8][32] = {
  {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0, 1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},
  {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0, 1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},
  {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0, 1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},
  {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0, 1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},
  {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0, 1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},
  {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0, 1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},
  {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0, 1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},
  {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0, 1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1}
};

// ============================================================================
// PATTERN: FULL (All LEDs active)
// ============================================================================
const bool FULL[8][32] = {
  {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1, 1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},
  {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1, 1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},
  {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1, 1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},
  {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1, 1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},
  {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1, 1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},
  {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1, 1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},
  {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1, 1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},
  {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1, 1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1}
};
const bool SLED[8][32] = {
  {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
  {0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1, 1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0},
  {0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1, 1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0},
  {0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1, 1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0},
  {0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1, 1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0},
  {0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1, 1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0},
  {0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1, 1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0},
  {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
};
// ============================================================================
// PATTERN: CHECKERBOARD
// ============================================================================
const bool CHECKERBOARD[8][32] = {
  {1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0, 1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0},
  {0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1, 0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1},
  {1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0, 1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0},
  {0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1, 0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1},
  {1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0, 1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0},
  {0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1, 0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1},
  {1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0, 1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0},
  {0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1, 0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1}
};

// ============================================================================
// PATTERN: BORDER
// ============================================================================
const bool BORDER[8][32] = {
  {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1, 1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},  // Top
  {1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},  // Sides
  {1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
  {1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
  {1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
  {1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
  {1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
  {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1, 1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1}   // Bottom
};

// ============================================================================
// PATTERN: CENTER BOX
// ============================================================================
const bool CENTER_BOX[8][32] = {
  {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
  {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
  {0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1, 1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0},
  {0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1, 1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0},
  {0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1, 1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0},
  {0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1, 1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0},
  {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
  {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}
};

const bool REAR_LOGIC[8][32] = {
  {0,0,1,1,1,1,0,0,0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,0,0,1,1,1,1,0,0},
  {0,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,0},
  {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1, 1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},
  {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1, 1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},
  {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1, 1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},
  {0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1, 1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0},
  {0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1, 1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0},
  {0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1, 1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0}
};


} // namespace MatrixPatterns

// ============================================================================
// USAGE EXAMPLES
// ============================================================================

/*
EXAMPLE 1: Use your custom left-half pattern
  #include "matrix_patterns.h"
  
  MatrixPattern pattern(8, 32, (const bool*)MatrixPatterns::LEFT_HALF);
  
  // In your animation:
  strip_->clear();
  for (uint8_t y = 0; y < 8; y++) {
    for (uint8_t x = 0; x < 32; x++) {
      if (pattern.isActive(x, y)) {
        uint16_t led_idx = pattern.xyToIndex(x, y);
        strip_->setPixelColor(led_idx, 255, 0, 0);  // Red
      }
    }
  }
  strip_->show();

EXAMPLE 2: Create a custom pattern
  const bool MY_PATTERN[8][32] = {
    // Define your custom pattern here
    // 1 = LED active, 0 = LED inactive
    {1,1,0,0,1,1,0,0, ...},  // Row 0
    {0,1,1,0,0,1,1,0, ...},  // Row 1
    // ... etc
  };
  
  MatrixPattern pattern(8, 32, (const bool*)MY_PATTERN);

EXAMPLE 3: Just use XY mapping without patterns
  MatrixMapper mapper(8, 32, true);  // 8 rows, 32 cols, serpentine
  
  // Light up LED at position (5, 2)
  uint16_t led_idx = mapper.xyToIndex(5, 2);
  strip_->setPixelColor(led_idx, 0, 255, 0);  // Green

EXAMPLE 4: Create animations with patterns
  // Moving bar across left half
  MatrixPattern pattern(8, 32, (const bool*)MatrixPatterns::LEFT_HALF);
  
  strip_->clear();
  for (uint8_t y = 0; y < 8; y++) {
    uint8_t x = (anim_step % 16);  // Bar position
    if (pattern.isActive(x, y)) {
      uint16_t led_idx = pattern.xyToIndex(x, y);
      strip_->setPixelColor(led_idx, 0, 0, 255);  // Blue
    }
  }
  strip_->show();
*/