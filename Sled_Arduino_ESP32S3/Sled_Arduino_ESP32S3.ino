// Sled Controller for ESP32-S3 with WCB 3.2 Hardware
// Adapted from Sled_Arduino.ino for NeoPixel/LED control via button input
// WCB 3.2 Serial Pin Mapping for ESP32-S3:
//   Serial1 TX (GPIO 4)  → NeoPixel Data
//   Serial2 TX (GPIO 6)  → Push Button Input
//   Serial1 RX (GPIO 5)  → LED 1
//   Serial2 RX (GPIO 7)  → LED 2
//   Serial3 TX (GPIO 15) → LED 3
//   Serial3 RX (GPIO 16) → LED 4
//   Serial4 TX (GPIO 17) → LED 5
//   Serial4 RX (GPIO 18) → LED 6
//   Serial5 TX (GPIO 9)  → LED 7
//   Serial5 RX (GPIO 10) → LED 8
//   GPIO 21 (direct)     → LED 9
//   Serial (USB-CDC)     → Debug Output

#include <Adafruit_NeoPixel.h>
#include <WiFi.h>
#include "esp_wifi.h"
#include <esp_now.h>
#define ETM_MY_BOARD_INDEX ETM_BOARD_SL
#include <ETM_Droid.h>
#ifdef __AVR__
  #include <avr/power.h> // Required for 16 MHz Adafruit Trinket
#endif

// ==================== ESP-NOW / ETM ====================
// Must match ESPNOWPASSWORD on the Droid Gateway and Body Controller.
String ESPNOWPASSWORD = "GregsAstromech";

// ==================== PIN DEFINITIONS ====================
#define BUTTON_PIN          6   // Serial2 TX (repurposed as input) - WCB S2 TX
#define PIXEL_PIN           4   // Serial1 TX (repurposed as data line) - WCB S1 TX
#define PIXEL_COUNT         31  // Total NeoPixels on the strip

// ==================== MATRIX LAYOUT ====================
// The first 27 pixels form a 3-row x 9-column matrix wired in serpentine
// (boustrophedon) order:
//   Row 0 (bottom):  pixel  0 .. 8   — right to left
//   Row 1 (middle):  pixel  9 ..17   — left to right (so col c -> pixel 17 - c)
//   Row 2 (top):     pixel 18 ..26   — right to left
// Pixels 27..30 are "extras" outside the matrix and are left off by the
// column-aware patterns (wipe, scanner).
#define MATRIX_ROWS         3
#define MATRIX_COLS         9
#define MATRIX_PIXELS       (MATRIX_ROWS * MATRIX_COLS)   // 27

// LED pin assignments for ESP32-S3 — driven by WCB 3.2 serial headers + 1 direct
#define LEFT_BLUE           5    // WCB S1 RX
#define LEFT_WHITE          7    // WCB S2 RX
#define LEFT_GREEN          15   // WCB S3 TX
#define RIGHT_BLUE          16   // WCB S3 RX
#define RIGHT_WHITE         17   // WCB S4 TX
#define RIGHT_GREEN         18   // WCB S4 RX
#define LED7                9    // WCB S5 TX
#define LED8                10   // WCB S5 RX
#define LED9                21   // Direct to ESP32-S3 (not via WCB)

#define LED_COUNT           9
int LEDS[LED_COUNT] = {LEFT_BLUE, LEFT_WHITE, LEFT_GREEN, RIGHT_BLUE, RIGHT_WHITE, RIGHT_GREEN, LED7, LED8, LED9};

// ==================== COLOR DEFINITIONS ====================
const uint32_t red     = 0xFF0000;
const uint32_t orange  = 0xFF8000;
const uint32_t yellow  = 0xFFFF00;
const uint32_t green   = 0x00FF00;
const uint32_t cyan    = 0x00FFFF;
const uint32_t blue    = 0x0000FF;
const uint32_t magenta = 0xFF00FF;
const uint32_t white   = 0xFFFFFF;
const uint32_t off     = 0x000000;

const uint32_t basicColors[9] = {off, red, yellow, green, cyan, blue, magenta, orange, white};

// ==================== PATTERN TYPES ====================
enum pattern { NONE, RAINBOW_CYCLE, THEATER_CHASE, COLOR_WIPE, SCANNER, FADE, PULSE,
               SPARKLE, MATRIX_RAIN, FIRE, HEARTBEAT, TWINKLE };
enum direction { FORWARD, REVERSE };

// ==================== STATE VARIABLES ====================
int LEFT_BLUE_STATE = LOW;
int LEFT_WHITE_STATE = LOW;
int LEFT_GREEN_STATE = LOW;
int RIGHT_BLUE_STATE = HIGH;
int RIGHT_WHITE_STATE = LOW;
int RIGHT_GREEN_STATE = LOW;

unsigned long currentMillis;
unsigned long blinkpreviousMillis = 0;
const long blinkinterval = 400; // Interval for LED blinking in milliseconds

int randomNumberArray;
int mode = 0;
int buttonState = HIGH;
int lastButtonState = HIGH;
int stableButtonState = HIGH;
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 50;  // Increased to 50ms for better stability
bool buttonPressed = false;

// ==================== NEO PIXEL PATTERN CLASS ====================
class NeoPatterns : public Adafruit_NeoPixel
{
public:
    pattern ActivePattern;
    direction Direction;
    
    unsigned long Interval;
    unsigned long lastUpdate;
    
    uint32_t Color1, Color2;
    uint16_t TotalSteps;
    uint16_t Index;
    int16_t  dropRow[MATRIX_COLS];   // MATRIX_RAIN: per-column falling head row (-1 = idle)
    
    void (*OnComplete)();
    
    // Constructor
    NeoPatterns(uint16_t pixels, uint8_t pin, uint8_t type, void (*callback)())
        : Adafruit_NeoPixel(pixels, pin, type)
    {
        OnComplete = callback;
    }
    
    // Update the pattern
    void Update()
    {
        if ((millis() - lastUpdate) > Interval)
        {
            lastUpdate = millis();
            switch (ActivePattern)
            {
                case NONE:
                    break;
                case RAINBOW_CYCLE:
                    RainbowCycleUpdate();
                    break;
                case THEATER_CHASE:
                    TheaterChaseUpdate();
                    break;
                case COLOR_WIPE:
                    ColorWipeUpdate();
                    break;
                case SCANNER:
                    ScannerUpdate();
                    break;
                case FADE:
                    FadeUpdate();
                    break;
                case PULSE:
                    PulseUpdate();
                    break;
                case SPARKLE:
                    SparkleUpdate();
                    break;
                case MATRIX_RAIN:
                    MatrixRainUpdate();
                    break;
                case FIRE:
                    FireUpdate();
                    break;
                case HEARTBEAT:
                    HeartbeatUpdate();
                    break;
                case TWINKLE:
                    TwinkleUpdate();
                    break;
                default:
                    break;
            }
        }
    }
    
    // Increment the Index and reset at the end
    void Increment()
    {
        if (Direction == FORWARD)
        {
            Index++;
            if (Index >= TotalSteps)
            {
                Index = 0;
                if (OnComplete != NULL)
                {
                    OnComplete();
                }
            }
        }
        else
        {
            --Index;
            if (Index <= 0)
            {
                Index = TotalSteps - 1;
                if (OnComplete != NULL)
                {
                    OnComplete();
                }
            }
        }
    }
    
    // Reverse pattern direction
    void Reverse()
    {
        if (Direction == FORWARD)
        {
            Direction = REVERSE;
            Index = TotalSteps - 1;
        }
        else
        {
            Direction = FORWARD;
            Index = 0;
        }
    }
    
    // Initialize for a RainbowCycle
    void RainbowCycle(uint8_t interval, direction dir = FORWARD)
    {
        ActivePattern = RAINBOW_CYCLE;
        Interval = interval;
        TotalSteps = 255;
        Index = 0;
        Direction = dir;
    }
    
    // Update the Rainbow Cycle Pattern
    void RainbowCycleUpdate()
    {
        for (int i = 0; i < numPixels(); i++)
        {
            setPixelColor(i, Wheel(((i * 256 / numPixels()) + Index) & 255));
        }
        show();
        Increment();
    }
    
    // Initialize for a Theater Chase
    void TheaterChase(uint32_t color1, uint32_t color2, uint8_t interval, direction dir = FORWARD)
    {
        ActivePattern = THEATER_CHASE;
        Interval = interval;
        TotalSteps = numPixels();
        Color1 = color1;
        Color2 = color2;
        Index = 0;
        Direction = dir;
    }
    
    // Update the Theater Chase Pattern
    void TheaterChaseUpdate()
    {
        for (int i = 0; i < numPixels(); i++)
        {
            if ((i + Index) % 3 == 0)
            {
                setPixelColor(i, Color1);
            }
            else
            {
                setPixelColor(i, Color2);
            }
        }
        show();
        Increment();
    }
    
    // Initialize for a ColorWipe
    void ColorWipe(uint32_t color, uint8_t interval, direction dir = FORWARD)
    {
        ActivePattern = COLOR_WIPE;
        Interval = interval;
        TotalSteps = numPixels();
        Color1 = color;
        Index = 0;
        Direction = dir;
    }
    
    // ===== Matrix layout helpers (serpentine 3x9) =====
    // Returns the pixel index for (row, col), or 0xFFFF if out of range.
    //
    // The strip IS wired serpentine — bottom and top rows run one direction,
    // middle row runs the opposite direction so the data line snakes through
    // all 27 pixels in one continuous chain.
    uint16_t pixelAt(uint8_t row, uint8_t col)
    {
        if (row >= MATRIX_ROWS || col >= MATRIX_COLS) return 0xFFFF;
        switch (row)
        {
            case 0: return col;                                  // bottom:  0..8  (left → right)
            case 1: return (uint16_t)(2 * MATRIX_COLS - 1) - col; // middle: 17..9  (right → left)
            case 2: return (uint16_t)(2 * MATRIX_COLS) + col;     // top:    18..26 (left → right)
        }
        return 0xFFFF;
    }

    // Sets every pixel in a column to the same color.
    void setColumnColor(uint8_t col, uint32_t color)
    {
        for (uint8_t row = 0; row < MATRIX_ROWS; row++)
        {
            uint16_t p = pixelAt(row, col);
            if (p != 0xFFFF) setPixelColor(p, color);
        }
    }

    // Update the Color Wipe Pattern (column-based — fills one whole column per step)
    void ColorWipeUpdate()
    {
        setColumnColor((uint8_t)Index, Color1);
        show();
        Increment();
    }
    
    // Initialize for a SCANNER
    void Scanner(uint32_t color1, uint8_t interval)
    {
        ActivePattern = SCANNER;
        Interval = interval;
        TotalSteps = (numPixels() - 1) * 2;
        Color1 = color1;
        Index = 0;
    }
    
    // Update the Scanner Pattern — Knight Rider / "KITT" Larson scanner.
    //
    // A bright head column sweeps left↔right across the 3x9 matrix with a
    // multi-column comet (head at full, ±1 and ±2 neighbours stepped down) and
    // a decaying trail behind it, so the back columns fade out smoothly.
    //
    // The bounce is computed entirely from `Index` via `scanCol`, so this
    // pattern does NOT use Direction / Reverse(). (It used to call the generic
    // Increment(), whose OnComplete=Reverse() flipped Direction to REVERSE;
    // combined with the unsigned `--Index` underflow that left the matrix
    // stuck dark — which is why the red scanner often refused to light unless
    // a FORWARD-resetting pattern like rainbow ran first.) We self-increment
    // here exactly like PulseUpdate(), so direction can never get corrupted.
    //
    // TotalSteps = (MATRIX_COLS - 1) * 2 = 16 → scanCol walks
    // 0,1,..,8,7,6,..,1 then wraps, a clean continuous bounce.
    void ScannerUpdate()
    {
        uint8_t scanCol = (Index < MATRIX_COLS)
            ? (uint8_t)Index                        // forward sweep:  0..MATRIX_COLS-1
            : (uint8_t)(TotalSteps - Index);        // backward sweep: MATRIX_COLS-2 .. 1

        for (uint8_t col = 0; col < MATRIX_COLS; col++)
        {
            // Comet shape: brightness by distance from the head column.
            int d = (int)col - (int)scanCol;
            if (d < 0) d = -d;
            uint8_t scale = (d == 0) ? 255 :   // head — full bright
                            (d == 1) ? 90  :   // first neighbour
                            (d == 2) ? 25  :   // second neighbour (faint)
                                       0;      // beyond the comet: trail only
            uint32_t comet = ScaleColor(Color1, scale);

            for (uint8_t row = 0; row < MATRIX_ROWS; row++)
            {
                uint16_t p = pixelAt(row, col);
                if (p == 0xFFFF) continue;
                // Brighter of the comet shape vs. the decaying trail left by
                // previous frames — gives a longer, naturally-fading tail.
                uint32_t decayed = DimColor(getPixelColor(p));
                setPixelColor(p, MaxColor(comet, decayed));
            }
        }
        show();

        // Self-advance and wrap (no Reverse) — scanCol provides the bounce.
        Index++;
        if (Index >= TotalSteps) Index = 0;
    }
    
    // Initialize for a Fade
    void Fade(uint32_t color1, uint32_t color2, uint16_t steps, uint8_t interval, direction dir = FORWARD)
    {
        ActivePattern = FADE;
        Interval = interval;
        TotalSteps = steps;
        Color1 = color1;
        Color2 = color2;
        Index = 0;
        Direction = dir;
    }
    
    // Update the Fade Pattern
    void FadeUpdate()
    {
        uint8_t red = ((Red(Color1) * (TotalSteps - Index)) + (Red(Color2) * Index)) / TotalSteps;
        uint8_t green = ((Green(Color1) * (TotalSteps - Index)) + (Green(Color2) * Index)) / TotalSteps;
        uint8_t blue = ((Blue(Color1) * (TotalSteps - Index)) + (Blue(Color2) * Index)) / TotalSteps;
        
        ColorSet(Color(red, green, blue));
        show();
        Increment();
    }

    // Initialize for a Pulse (smooth sine breathing on a single color)
    // steps    = samples per full breath cycle (higher = smoother)
    // interval = ms between samples (steps * interval = period in ms)
    void Pulse(uint32_t color, uint16_t steps, uint8_t interval)
    {
        ActivePattern = PULSE;
        Interval = interval;
        TotalSteps = steps;
        Color1 = color;
        Index = 0;
        Direction = FORWARD;
    }

    // Update the Pulse Pattern (sine-shaped brightness with a small glow floor)
    void PulseUpdate()
    {
        // phase walks 0..2π across TotalSteps; offset so we start dim and breathe up
        float phase = ((float)Index / (float)TotalSteps) * 2.0f * (float)PI;
        float s = (sinf(phase - (float)PI / 2.0f) + 1.0f) * 0.5f;  // 0..1
        // Keep a 15% glow floor so it pulses instead of cutting fully off
        float b = 0.15f + s * 0.85f;

        uint8_t r = (uint8_t)(Red(Color1)   * b);
        uint8_t g = (uint8_t)(Green(Color1) * b);
        uint8_t bl = (uint8_t)(Blue(Color1)  * b);

        ColorSet(Color(r, g, bl));

        Index++;
        if (Index >= TotalSteps) Index = 0;
    }

    // Calculate 50% dimmed version of a color
    uint32_t DimColor(uint32_t color)
    {
        uint32_t dimColor = Color(Red(color) >> 1, Green(color) >> 1, Blue(color) >> 1);
        return dimColor;
    }

    // Scale a color's brightness by scale/255 (0 = off, 255 = full).
    uint32_t ScaleColor(uint32_t color, uint8_t scale)
    {
        uint16_t r = ((uint16_t)Red(color)   * scale) / 255;
        uint16_t g = ((uint16_t)Green(color) * scale) / 255;
        uint16_t b = ((uint16_t)Blue(color)  * scale) / 255;
        return Color((uint8_t)r, (uint8_t)g, (uint8_t)b);
    }

    // Per-channel max of two colors — lets the comet head sit on top of a
    // decaying trail without either one clobbering the other.
    uint32_t MaxColor(uint32_t a, uint32_t b)
    {
        uint8_t r = Red(a)   > Red(b)   ? Red(a)   : Red(b);
        uint8_t g = Green(a) > Green(b) ? Green(a) : Green(b);
        uint8_t bl = Blue(a) > Blue(b)  ? Blue(a)  : Blue(b);
        return Color(r, g, bl);
    }

    // Map a heat value (0..255) onto a black→red→orange→yellow→white flame
    // ramp (the classic FastLED HeatColor algorithm).
    uint32_t HeatColor(uint8_t temperature)
    {
        uint8_t t192 = (uint8_t)(((uint16_t)temperature * 191) / 255);  // 0..191
        uint8_t ramp = t192 & 0x3F;                                     // 0..63
        ramp <<= 2;                                                     // 0..252
        if (t192 & 0x80)      return Color(255, 255, ramp);             // hottest: toward white
        else if (t192 & 0x40) return Color(255, ramp, 0);              // middle: orange→yellow
        else                  return Color(ramp, 0, 0);                 // coolest: dim red
    }

    // ===== SPARKLE — sharp white/colour pops over a fast-decaying field =====
    void SparkleUpdate()
    {
        for (int i = 0; i < numPixels(); i++)
            setPixelColor(i, DimColor(getPixelColor(i)));   // fast 50% decay → crackle
        uint8_t sparks = 2 + random(2);                     // 2..3 new sparks/frame
        for (uint8_t s = 0; s < sparks; s++)
            setPixelColor(random(numPixels()), Color1);
        show();
    }

    // ===== TWINKLE — gentle multi-colour shimmer with a slow fade =====
    void TwinkleUpdate()
    {
        for (int i = 0; i < numPixels(); i++)
            setPixelColor(i, ScaleColor(getPixelColor(i), 200));   // ~78%/frame: slow, soft fade
        if (random(100) < 40)                                       // occasional new twinkle
        {
            uint32_t c = basicColors[1 + random(8)];                // any colour except OFF
            setPixelColor(random(numPixels()), ScaleColor(c, 170)); // start soft
        }
        show();
    }

    // ===== FIRE — warm flicker, hotter at the bottom row =====
    void FireUpdate()
    {
        for (uint8_t col = 0; col < MATRIX_COLS; col++)
        {
            for (uint8_t row = 0; row < MATRIX_ROWS; row++)
            {
                uint16_t p = pixelAt(row, col);
                if (p == 0xFFFF) continue;
                int heat = 255 - (int)row * 70;   // row0 bottom hottest, row2 top coolest
                heat -= (int)random(70);          // random flicker downward
                if (heat < 0) heat = 0;
                setPixelColor(p, HeatColor((uint8_t)heat));
            }
        }
        show();
    }

    // ===== HEARTBEAT — double "lub-dub" throb on Color1 =====
    void HeartbeatUpdate()
    {
        float t = (float)Index / (float)TotalSteps;                 // 0..1 over the cycle
        float bump1 = expf(-powf((t - 0.08f) / 0.045f, 2.0f));      // strong first beat
        float bump2 = 0.6f * expf(-powf((t - 0.28f) / 0.045f, 2.0f)); // softer second beat
        float b = bump1 > bump2 ? bump1 : bump2;                    // (rest of cycle ≈ 0)
        float level = 0.04f + b * 0.96f;                            // small glow floor
        ColorSet(ScaleColor(Color1, (uint8_t)(level * 255.0f)));
        Index++;
        if (Index >= TotalSteps) Index = 0;
    }

    // ===== MATRIX RAIN — bright heads fall down each column, leaving trails =====
    void MatrixRainUpdate()
    {
        // Fade the matrix to leave fading tails behind each falling head.
        for (uint8_t col = 0; col < MATRIX_COLS; col++)
            for (uint8_t row = 0; row < MATRIX_ROWS; row++)
            {
                uint16_t p = pixelAt(row, col);
                if (p != 0xFFFF) setPixelColor(p, DimColor(getPixelColor(p)));
            }
        for (uint8_t col = 0; col < MATRIX_COLS; col++)
        {
            if (dropRow[col] >= 0)
            {
                uint16_t p = pixelAt((uint8_t)dropRow[col], col);
                if (p != 0xFFFF) setPixelColor(p, Color1);  // bright head
                dropRow[col]--;                              // fall toward the bottom
            }
            else if (random(100) < 12)                       // chance to spawn a new drop
            {
                dropRow[col] = MATRIX_ROWS - 1;              // start at the top row
            }
        }
        show();
    }
    
    // Set all pixels to a color (synchronously)
    void ColorSet(uint32_t color)
    {
        for (int i = 0; i < numPixels(); i++)
        {
            setPixelColor(i, color);
        }
        show();
    }
    
    // Returns the Red component of a 32-bit color
    uint8_t Red(uint32_t color)
    {
        return (color >> 16) & 0xFF;
    }
    
    // Returns the Green component of a 32-bit color
    uint8_t Green(uint32_t color)
    {
        return (color >> 8) & 0xFF;
    }
    
    // Returns the Blue component of a 32-bit color
    uint8_t Blue(uint32_t color)
    {
        return color & 0xFF;
    }
    
    // Input a value 0 to 255 to get a color value
    uint32_t Wheel(byte WheelPos)
    {
        WheelPos = 255 - WheelPos;
        if (WheelPos < 85)
        {
            return Color(255 - WheelPos * 3, 0, WheelPos * 3);
        }
        else if (WheelPos < 170)
        {
            WheelPos -= 85;
            return Color(0, WheelPos * 3, 255 - WheelPos * 3);
        }
        else
        {
            WheelPos -= 170;
            return Color(WheelPos * 3, 255 - WheelPos * 3, 0);
        }
    }
};

// ==================== FORWARD DECLARATIONS ====================
NeoPatterns* stripPtr = nullptr;

// ==================== STRIP COMPLETION CALLBACK ====================
void stripComplete()
{
    if (stripPtr != nullptr)
        stripPtr->Reverse();
}

// ==================== GLOBAL OBJECTS ====================
NeoPatterns strip(PIXEL_COUNT, PIXEL_PIN, NEO_GRB + NEO_KHZ800, &stripComplete);

// ==================== PATTERN HELPERS ====================
// Centralised setup for each pattern so the button cycle and ESP-NOW command
// parser both produce identical behaviour.

void applyOff()
{
    strip.ActivePattern = NONE;
    strip.ColorSet(off);
}

void applySolid(uint32_t color)
{
    strip.ActivePattern = NONE;
    strip.ColorSet(color);
}

void applyPulse(uint32_t color)
{
    strip.Color1        = color;
    strip.TotalSteps    = 150;          // 150 * 20ms = 3s breath cycle
    strip.Interval      = 20;
    strip.ActivePattern = PULSE;
    strip.Index         = 0;
    strip.Direction     = FORWARD;
    strip.lastUpdate    = millis() - strip.Interval;
}

void applyWipe(uint32_t color)
{
    strip.Color1        = color;
    strip.Interval      = 90;                // a bit slower since columns travel less distance
    strip.ActivePattern = COLOR_WIPE;
    strip.TotalSteps    = MATRIX_COLS;       // one step per column
    strip.Index         = 0;
    strip.Direction     = FORWARD;
    strip.clear();                           // wipe the slate (matrix + extras) before filling
    strip.show();
}

void applyChase(uint32_t c1, uint32_t c2)
{
    strip.Color1        = c1;
    strip.Color2        = c2;
    strip.Interval      = 550;
    strip.ActivePattern = THEATER_CHASE;
    strip.TotalSteps    = strip.numPixels();
    strip.Index         = 0;
    strip.Direction     = FORWARD;
}

void applyRainbow()
{
    strip.Interval      = 10;
    strip.ActivePattern = RAINBOW_CYCLE;
    strip.TotalSteps    = 255;
    strip.Index         = 0;
    strip.Direction     = FORWARD;
}

void applyScanner(uint32_t color)
{
    strip.Color1        = color;
    strip.Interval      = 55;                          // brisk KITT sweep (comet stays readable via the trail)
    strip.ActivePattern = SCANNER;
    strip.TotalSteps    = (MATRIX_COLS - 1) * 2;       // bounce across columns
    strip.Index         = 0;
    strip.Direction     = FORWARD;                      // scanner self-increments; keep state sane regardless
    strip.lastUpdate    = millis() - strip.Interval;    // render the first frame immediately
    strip.clear();                                      // extras stay dark
    strip.show();
}

void applyFade(uint32_t c1, uint32_t c2)
{
    strip.Color1        = c1;
    strip.Color2        = c2;
    strip.Interval      = 75;
    strip.ActivePattern = FADE;
    strip.TotalSteps    = 256;
    strip.Index         = 0;
    strip.Direction     = FORWARD;
    strip.lastUpdate    = millis() - strip.Interval;
}

// Red Alert — reuse the smooth PULSE engine, just fast + red for an urgent throb.
void applyAlert()
{
    strip.Color1        = red;
    strip.TotalSteps    = 40;                          // 40 * 18ms ≈ 0.72s throb
    strip.Interval      = 18;
    strip.ActivePattern = PULSE;
    strip.Index         = 0;
    strip.Direction     = FORWARD;
    strip.lastUpdate    = millis() - strip.Interval;
}

void applySparkle(uint32_t color)
{
    strip.Color1        = color;
    strip.Interval      = 40;
    strip.ActivePattern = SPARKLE;
    strip.Index         = 0;
    strip.lastUpdate    = millis() - strip.Interval;
    strip.clear();
    strip.show();
}

void applyTwinkle()
{
    strip.Interval      = 60;
    strip.ActivePattern = TWINKLE;
    strip.Index         = 0;
    strip.lastUpdate    = millis() - strip.Interval;
    strip.clear();
    strip.show();
}

void applyFire()
{
    strip.Interval      = 50;
    strip.ActivePattern = FIRE;
    strip.Index         = 0;
    strip.lastUpdate    = millis() - strip.Interval;
    strip.clear();                                      // extras stay dark
    strip.show();
}

void applyHeartbeat(uint32_t color)
{
    strip.Color1        = color;
    strip.TotalSteps    = 120;                          // 120 * 10ms = 1.2s per heartbeat
    strip.Interval      = 10;
    strip.ActivePattern = HEARTBEAT;
    strip.Index         = 0;
    strip.lastUpdate    = millis() - strip.Interval;
}

void applyMatrixRain(uint32_t color)
{
    strip.Color1        = color;
    strip.Interval      = 110;
    strip.ActivePattern = MATRIX_RAIN;
    strip.Index         = 0;
    for (uint8_t c = 0; c < MATRIX_COLS; c++) strip.dropRow[c] = -1;
    strip.lastUpdate    = millis() - strip.Interval;
    strip.clear();
    strip.show();
}

// ==================== MODE CYCLE ====================
// Single source of truth for "next animation" — used by the local button AND
// by the :NEXT / :PREV ESP-NOW commands so all paths produce identical results.
#define SLED_MODE_COUNT  14

void applyModeByIndex(int n)
{
    switch (n)
    {
        case 0:  Serial.println("[MODE 0] PULSE (Blue breathing)");   applyPulse(blue);       break;
        case 1:  Serial.println("[MODE 1] PULSE (Red breathing)");    applyPulse(red);        break;
        case 2:  Serial.println("[MODE 2] COLOR_WIPE (White)");       applyWipe(white);       break;
        case 3:  Serial.println("[MODE 3] COLOR_WIPE (Red)");         applyWipe(red);         break;
        case 4:  Serial.println("[MODE 4] COLOR_WIPE (Blue)");        applyWipe(blue);        break;
        case 5:  Serial.println("[MODE 5] THEATER_CHASE (Blue/Red)"); applyChase(blue, red);  break;
        case 6:  Serial.println("[MODE 6] SCANNER (Blue)");           applyScanner(blue);     break;
        case 7:  Serial.println("[MODE 7] SCANNER (Red)");            applyScanner(red);      break;
        case 8:  Serial.println("[MODE 8] RED ALERT");                applyAlert();           break;
        case 9:  Serial.println("[MODE 9] SPARKLE (White)");          applySparkle(white);    break;
        case 10: Serial.println("[MODE 10] TWINKLE (multi)");         applyTwinkle();         break;
        case 11: Serial.println("[MODE 11] FIRE");                    applyFire();            break;
        case 12: Serial.println("[MODE 12] HEARTBEAT (Red)");         applyHeartbeat(red);    break;
        case 13: Serial.println("[MODE 13] MATRIX RAIN (Green)");     applyMatrixRain(green); break;
        default: return;
    }
}

void cycleNext()
{
    applyModeByIndex(mode);
    mode = (mode + 1) % SLED_MODE_COUNT;
}

void cyclePrev()
{
    // Step back one (mode points at NEXT, so back up by 2 to land on previous)
    mode = (mode + SLED_MODE_COUNT - 2) % SLED_MODE_COUNT;
    applyModeByIndex(mode);
    mode = (mode + 1) % SLED_MODE_COUNT;
}

// ==================== COLOR PARSER ====================
// Accepts case-insensitive color names ("BLUE", "red", ...) and "#RRGGBB".
// Returns "off" (0x000000) if unrecognised.
uint32_t parseColor(const char* s)
{
    if (s == nullptr || s[0] == '\0') return off;

    // Hex form: "#RRGGBB" or "RRGGBB"
    const char* hex = (s[0] == '#') ? s + 1 : s;
    bool looksHex = (strlen(hex) == 6);
    for (int i = 0; looksHex && i < 6; i++)
    {
        char c = hex[i];
        if (!((c >= '0' && c <= '9') ||
              (c >= 'a' && c <= 'f') ||
              (c >= 'A' && c <= 'F'))) { looksHex = false; }
    }
    if (looksHex)
    {
        return (uint32_t)strtoul(hex, nullptr, 16) & 0x00FFFFFF;
    }

    // Named colors
    if (strcasecmp(s, "RED")     == 0) return red;
    if (strcasecmp(s, "ORANGE")  == 0) return orange;
    if (strcasecmp(s, "YELLOW")  == 0) return yellow;
    if (strcasecmp(s, "GREEN")   == 0) return green;
    if (strcasecmp(s, "CYAN")    == 0) return cyan;
    if (strcasecmp(s, "BLUE")    == 0) return blue;
    if (strcasecmp(s, "MAGENTA") == 0) return magenta;
    if (strcasecmp(s, "WHITE")   == 0) return white;
    if (strcasecmp(s, "OFF")     == 0) return off;
    return off;
}

// ==================== COMMAND DISPATCH ====================
// Parses commands of the form ":VERB" or ":VERB,ARG1[,ARG2]" and applies the
// matching pattern. Returns true if recognised.
//
// Supported:
//   :OFF
//   :COLOR,<name|#RRGGBB>
//   :PULSE,<color>
//   :WIPE,<color>
//   :CHASE,<color1>,<color2>
//   :SCAN,<color>
//   :FADE,<color1>,<color2>
//   :RAINBOW
//   :ALERT                 (fast red throb)
//   :SPARKLE,<color>       (sharp pops, default white)
//   :TWINKLE               (soft multi-colour shimmer)
//   :FIRE                  (warm flame flicker)
//   :HEARTBEAT,<color>     (double lub-dub throb, default red)
//   :RAIN,<color>          (matrix rain, default green)
//   :MODE,<0..13>          (cycle-style shortcut — matches the local button)
bool runSledCommand(const char* cmdIn)
{
    if (cmdIn == nullptr) return false;

    // Skip leading ':' if present
    const char* src = (cmdIn[0] == ':') ? cmdIn + 1 : cmdIn;

    // Copy into a mutable buffer so strtok can chew it up
    char buf[100];
    strncpy(buf, src, sizeof(buf) - 1);
    buf[sizeof(buf) - 1] = '\0';

    char* verb = strtok(buf, ",");
    if (verb == nullptr) return false;

    char* a1 = strtok(nullptr, ",");
    char* a2 = strtok(nullptr, ",");

    Serial.printf("[SLED] Command verb=%s a1=%s a2=%s\n",
                  verb, a1 ? a1 : "-", a2 ? a2 : "-");

    if      (strcasecmp(verb, "OFF")     == 0) { applyOff(); return true; }
    else if (strcasecmp(verb, "COLOR")   == 0) { applySolid(parseColor(a1)); return true; }
    else if (strcasecmp(verb, "PULSE")   == 0) { applyPulse(parseColor(a1 ? a1 : "BLUE")); return true; }
    else if (strcasecmp(verb, "WIPE")    == 0) { applyWipe(parseColor(a1 ? a1 : "WHITE")); return true; }
    else if (strcasecmp(verb, "CHASE")   == 0) { applyChase(parseColor(a1 ? a1 : "BLUE"),
                                                            parseColor(a2 ? a2 : "RED")); return true; }
    else if (strcasecmp(verb, "SCAN")    == 0) { applyScanner(parseColor(a1 ? a1 : "RED")); return true; }
    else if (strcasecmp(verb, "FADE")    == 0) { applyFade(parseColor(a1 ? a1 : "BLUE"),
                                                           parseColor(a2 ? a2 : "OFF")); return true; }
    else if (strcasecmp(verb, "RAINBOW") == 0) { applyRainbow(); return true; }
    else if (strcasecmp(verb, "ALERT")     == 0) { applyAlert(); return true; }
    else if (strcasecmp(verb, "SPARKLE")   == 0) { applySparkle(parseColor(a1 ? a1 : "WHITE")); return true; }
    else if (strcasecmp(verb, "TWINKLE")   == 0) { applyTwinkle(); return true; }
    else if (strcasecmp(verb, "FIRE")      == 0) { applyFire(); return true; }
    else if (strcasecmp(verb, "HEARTBEAT") == 0) { applyHeartbeat(parseColor(a1 ? a1 : "RED")); return true; }
    else if (strcasecmp(verb, "RAIN")      == 0) { applyMatrixRain(parseColor(a1 ? a1 : "GREEN")); return true; }
    else if (strcasecmp(verb, "NEXT")    == 0) { cycleNext(); return true; }
    else if (strcasecmp(verb, "PREV")    == 0) { cyclePrev(); return true; }
    else if (strcasecmp(verb, "MODE")    == 0)
    {
        int n = a1 ? atoi(a1) : 0;
        if (n < 0 || n >= SLED_MODE_COUNT) return false;
        applyModeByIndex(n);
        mode = (n + 1) % SLED_MODE_COUNT;   // keep button cycle in sync with remote selection
        return true;
    }

    Serial.printf("[SLED] Unknown verb: %s\n", verb);
    return false;
}

// ==================== ESP-NOW RECEIVE CALLBACK ====================
void OnDataRecv(const esp_now_recv_info_t* info, const uint8_t* incomingData, int len)
{
    if (len < (int)sizeof(espnow_struct_message)) return;

    espnow_struct_message msg;
    memcpy(&msg, incomingData, sizeof(msg));

    // Password gate — silently drop foreign-network packets
    if (strncmp(msg.structPassword, ESPNOWPASSWORD.c_str(), sizeof(msg.structPassword)) != 0) return;

    int senderIdx = etmBoardIndexFromMAC(info->src_addr);
    if (senderIdx < 0) return;   // unknown peer

    // Any inbound packet refreshes the sender's heartbeat timestamp
    etmHandleHeartbeat(senderIdx);

    switch (msg.structPacketType)
    {
        case PACKET_TYPE_HEARTBEAT:
            // Already handled by etmHandleHeartbeat above
            return;

        case PACKET_TYPE_ACK:
            etmProcessAck(senderIdx, msg.structSequenceNumber);
            return;

        case PACKET_TYPE_COMMAND:
        {
            // Only act on commands targeted at us or broadcast
            bool forUs       = (strncmp(msg.structTargetID, "SL", 2) == 0);
            bool isBroadcast = (strncmp(msg.structTargetID, "BR", 2) == 0);
            if (!forUs && !isBroadcast) return;

            if (msg.structCommandIncluded)
            {
                Serial.printf("[ESP-NOW] %s -> %s : %s (seq %u)\n",
                              ETM_BOARD_IDS[senderIdx],
                              msg.structTargetID,
                              msg.structCommand,
                              msg.structSequenceNumber);
                runSledCommand(msg.structCommand);
            }

            // ACK back to sender (unicast targeting only — broadcasts don't need ACKs from every receiver)
            if (forUs)
            {
                etmSendAck(senderIdx, msg.structSequenceNumber);
            }
            return;
        }
    }
}

// ==================== SETUP ====================
void setup()
{
    Serial.begin(115200);
    delay(500); // Give serial time to initialize
    
    Serial.println("\n\n=== Sled Controller (ESP32-S3) Starting ===");
    
    // Initialize strip pointer for callback
    stripPtr = &strip;
    
    // Initialize NeoPixel strip — boot into a slow blue breathing pulse so
    // there's clear visual confirmation the firmware came up cleanly. This
    // also leaves `mode` consistent with the cycle (mode 0 = PULSE blue).
    strip.begin();
    randomSeed(esp_random());   // vary Sparkle / Fire / Twinkle / Rain between boots
    applyPulse(blue);
    
    // Initialize button pin
    pinMode(BUTTON_PIN, INPUT_PULLUP);
    
    // Initialize LED pins
    for (int i = 0; i < LED_COUNT; i++)
    {
        pinMode(LEDS[i], OUTPUT);
        digitalWrite(LEDS[i], LOW);
    }

    // ===== ESP-NOW (ETM_Droid) =====
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();   // Pure ESP-NOW — no AP/STA join needed
    // Set our MAC so other ETM peers recognise us as ETM_BOARD_SL
    esp_wifi_set_mac(WIFI_IF_STA, (uint8_t*)ETM_BOARD_MACS[ETM_BOARD_SL]);
    Serial.print("[ESP-NOW] MAC: "); Serial.println(WiFi.macAddress());

    if (esp_now_init() != ESP_OK)
    {
        Serial.println("[ESP-NOW] init FAILED");
    }
    else
    {
        esp_now_register_recv_cb(OnDataRecv);

        // Register broadcast peer + all known ETM boards as peers
        esp_now_peer_info_t peerInfo = {};
        peerInfo.channel = 0;
        peerInfo.encrypt = false;

        memcpy(peerInfo.peer_addr, ETM_BROADCAST_MAC, 6);
        esp_now_add_peer(&peerInfo);

        for (int i = 0; i < ETM_NUM_BOARDS; i++)
        {
            if (i == ETM_BOARD_SL) continue;
            memcpy(peerInfo.peer_addr, ETM_BOARD_MACS[i], 6);
            esp_now_add_peer(&peerInfo);
        }

        etmInit(ESPNOWPASSWORD.c_str(), ETM_BOARD_SL);
        Serial.println("[ESP-NOW] Ready as SL");
    }

    Serial.println("Setup complete. Press button to cycle, or send :E SL <cmd> from the webpage.");
}

// ==================== MAIN LOOP ====================
// ==================== USB SERIAL COMMAND INPUT ====================
// Accumulates characters from USB Serial into a line buffer, then dispatches
// the complete line through runSledCommand() — same parser the ESP-NOW
// callback uses. Lets you drive the Sled directly from a USB serial monitor
// at 115200 baud for bench testing:
//
//   :PULSE,BLUE
//   :CHASE,GREEN,WHITE
//   :OFF
//   :MODE,3
//
// Lines may end in CR ('\r'), LF ('\n'), or CRLF. Empty lines and lines
// longer than the buffer are silently dropped (buffer is reset on overflow).
// The leading ':' is optional — runSledCommand() strips it if present.
static char     serialCmdBuf[100];
static uint8_t  serialCmdLen = 0;

static void serialPoll()
{
    while (Serial.available() > 0)
    {
        char c = (char)Serial.read();
        if (c == '\r' || c == '\n')
        {
            if (serialCmdLen > 0)
            {
                serialCmdBuf[serialCmdLen] = '\0';
                Serial.printf("[USB] received: %s\n", serialCmdBuf);
                bool ok = runSledCommand(serialCmdBuf);
                if (!ok) Serial.printf("[USB] command rejected\n");
                serialCmdLen = 0;
            }
        }
        else if (serialCmdLen < sizeof(serialCmdBuf) - 1)
        {
            serialCmdBuf[serialCmdLen++] = c;
        }
        else
        {
            // Overflow — drop the partial line so we resync on the next CR/LF.
            Serial.println("[USB] line too long, discarded");
            serialCmdLen = 0;
        }
    }
}

void loop()
{
    // Drive ETM heartbeats / retries / offline detection
    etmProcess();

    // Drain any pending USB serial commands (bench-testing convenience)
    serialPoll();

    // Update NeoPixel animation
    strip.Update();
    
    // ===== BUTTON DEBOUNCING (improved) =====
    int reading = digitalRead(BUTTON_PIN);
    
    // If the reading has changed from last time, reset the debounce timer
    if (reading != lastButtonState)
    {
        lastDebounceTime = millis();
        lastButtonState = reading;
    }
    
    // If the reading has been stable for longer than debounceDelay
    if ((millis() - lastDebounceTime) > debounceDelay)
    {
        // If the stable state has changed
        if (reading != stableButtonState)
        {
            stableButtonState = reading;
            
            // Detect HIGH->LOW transition (button press)
            if (stableButtonState == LOW && !buttonPressed)
            {
                buttonPressed = true;
                Serial.print("Button pressed. Advancing from mode: ");
                Serial.println(mode);
                cycleNext();   // shared with :NEXT ESP-NOW command
            }
            // Detect LOW->HIGH transition (button release)
            else if (stableButtonState == HIGH)
            {
                buttonPressed = false;
            }
        }
    }
    
    // ===== RANDOM LED BLINKING =====
    currentMillis = millis();
    
    if (currentMillis - blinkpreviousMillis >= blinkinterval)
    {
        blinkpreviousMillis = currentMillis;
        randomNumberArray = random(0, LED_COUNT);
        digitalWrite(LEDS[randomNumberArray], HIGH);
        randomNumberArray = random(0, LED_COUNT);
        digitalWrite(LEDS[randomNumberArray], LOW);
    }
}
