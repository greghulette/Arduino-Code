// Simple demonstration on using an input device to trigger changes on your
// NeoPixels. Wire a momentary push button to connect from ground to a
// digital IO pin. When the button is pressed it will change to a new pixel
// animation. Initial state has all pixels off -- press the button once to
// start the first animation. As written, the button does not interrupt an
// animation in-progress, it works only when idle.
#include <Adafruit_DotStar.h>

#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
 #include <avr/power.h> // Required for 16 MHz Adafruit Trinket
#endif
#include <Toggle.h>

// Digital IO pin connected to the button. This will be driven with a
// pull-up resistor so the switch pulls the pin to ground momentarily.
// On a high -> low transition the button press logic will execute.
#define BUTTON_PIN   2
#define PIXEL_PIN    3  // Digital IO pin connected to the NeoPixels.
#define LEFT_BLUE 13
#define LEFT_WHITE 12
#define LEFT_GREEN 11
#define RIGHT_BLUE 4 
#define RIGHT_WHITE 5
#define RIGHT_GREEN 6
#define LED7 7
#define LED8 8
#define LED9 9
#define LED10 10 
#define PIXEL_COUNT 27 // Number of NeoPixels

int  LEDS[10] = {LEFT_BLUE, LEFT_WHITE,LEFT_GREEN,RIGHT_BLUE, RIGHT_WHITE, RIGHT_GREEN,LED7,LED8,LED9,LED10};
// Declare our NeoPixel strip object:
// Adafruit_NeoPixel strip(PIXEL_COUNT, PIXEL_PIN, NEO_GRB + NEO_KHZ800);
//Adafruit_DotStar strip(PIXEL_COUNT, DATAPIN, CLOCKPIN, DOTSTAR_BRG);
//
//#define DATAPIN    4
//#define CLOCKPIN   5
unsigned long currentMillis;
int randomNumberArray;

// Pattern types supported:
enum  pattern { NONE, RAINBOW_CYCLE, THEATER_CHASE, COLOR_WIPE, SCANNER, FADE };

// Patern directions supported:
enum  direction { FORWARD, REVERSE };



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

// Argument 1 = Number of pixels in NeoPixel strip
// Argument 2 = Arduino pin number (most are valid)
// Argument 3 = Pixel type flags, add together as needed:
//   NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
//   NEO_KHZ400  400 KHz (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)
//   NEO_GRB     Pixels are wired for GRB bitstream (most NeoPixel products)
//   NEO_RGB     Pixels are wired for RGB bitstream (v1 FLORA pixels, not v2)
//   NEO_RGBW    Pixels are wired for RGBW bitstream (NeoPixel RGBW products)

// boolean oldState = HIGH;
// int     mode     = 0;    // Currently-active animation mode, 0-9



int LEFT_BLUE_STATE = LOW;
int LEFT_WHITE_STATE = LOW;
int LEFT_GREEN_STATE = LOW;

int RIGHT_BLUE_STATE = HIGH;
int RIGHT_WHITE_STATE = LOW;
int RIGHT_GREEN_STATE = LOW;

// Generally, you should use "unsigned long" for variables that hold time
// The value will quickly become too large for an int to store
unsigned long blinkpreviousMillis = 0;        // will store last time LED was updated

// constants won't change:
const long blinkinterval = 400;           // interval at which to blink (milliseconds)

// NeoPattern Class - derived from the Adafruit_NeoPixel class
class NeoPatterns : public Adafruit_NeoPixel
{
    public:

    // Member Variables:  
    pattern  ActivePattern;  // which pattern is running
    direction Direction;     // direction to run the pattern
    
    unsigned long Interval;   // milliseconds between updates
    unsigned long lastUpdate; // last update of position
    
    uint32_t Color1, Color2;  // What colors are in use
    uint16_t TotalSteps;  // total number of steps in the pattern
    uint16_t Index;  // current step within the pattern
    
    void (*OnComplete)();  // Callback on completion of pattern
    
    // Constructor - calls base-class constructor to initialize strip
    NeoPatterns(uint16_t pixels, uint8_t pin, uint8_t type, void (*callback)())
    :Adafruit_NeoPixel(pixels, pin, type)
    {
        OnComplete = callback;
    }
    
    // Update the pattern
    void Update()
    {
        if((millis() - lastUpdate) > Interval) // time to update
        {
            lastUpdate = millis();
            switch(ActivePattern)
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
                    OnComplete(); // call the comlpetion callback
                }
            }
        }
        else // Direction == REVERSE
        {
            --Index;
            if (Index <= 0)
            {
                Index = TotalSteps-1;
                if (OnComplete != NULL)
                {
                    OnComplete(); // call the comlpetion callback
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
            Index = TotalSteps-1;
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
        for(int i=0; i< numPixels(); i++)
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
        for(int i=0; i< numPixels(); i++)
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
    
    // Update the Color Wipe Pattern
    void ColorWipeUpdate()
    {
        setPixelColor(Index, Color1);
        show();
        Increment();
    }
    
    // Initialize for a SCANNNER
    void Scanner(uint32_t color1, uint8_t interval)
    {
        ActivePattern = SCANNER;
        Interval = interval;
        TotalSteps = (numPixels() - 1) * 2;
        Color1 = color1;
        Index = 0;
    }

    // Update the Scanner Pattern
    void ScannerUpdate()
    { 
        for (int i = 0; i < numPixels(); i++)
        {
            if (i == Index)  // Scan Pixel to the right
            {
                 setPixelColor(i, Color1);
            }
            else if (i == TotalSteps - Index) // Scan Pixel to the left
            {
                 setPixelColor(i, Color1);
            }
            else // Fading tail
            {
                 setPixelColor(i, DimColor(getPixelColor(i)));
            }
        }
        show();
        Increment();
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
        // Calculate linear interpolation between Color1 and Color2
        // Optimise order of operations to minimize truncation error
        uint8_t red = ((Red(Color1) * (TotalSteps - Index)) + (Red(Color2) * Index)) / TotalSteps;
        uint8_t green = ((Green(Color1) * (TotalSteps - Index)) + (Green(Color2) * Index)) / TotalSteps;
        uint8_t blue = ((Blue(Color1) * (TotalSteps - Index)) + (Blue(Color2) * Index)) / TotalSteps;
        
        ColorSet(Color(red, green, blue));
        show();
        Increment();
    }
   
    // Calculate 50% dimmed version of a color (used by ScannerUpdate)
    uint32_t DimColor(uint32_t color)
    {
        // Shift R, G and B components one bit to the right
        uint32_t dimColor = Color(Red(color) >> 1, Green(color) >> 1, Blue(color) >> 1);
        return dimColor;
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
    
    // Input a value 0 to 255 to get a color value.
    // The colours are a transition r - g - b - back to r.
    uint32_t Wheel(byte WheelPos)
    {
        WheelPos = 255 - WheelPos;
        if(WheelPos < 85)
        {
            return Color(255 - WheelPos * 3, 0, WheelPos * 3);
        }
        else if(WheelPos < 170)
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


void stripComplete();

Toggle button(BUTTON_PIN);
NeoPatterns strip(27, 3, NEO_GRB + NEO_KHZ800, &stripComplete);
int mode =0;

void setup() {
  Serial.begin(115200);
  button.begin(BUTTON_PIN);
  button.setToggleTrigger(0);
  // pinMode(BUTTON_PIN, INPUT_PULLUP);
  strip.begin(); // Initialize NeoPixel strip object (REQUIRED)
  strip.ColorWipe(blue, 25);
  // strip.show();  // Initialize all pixels to 'off'
  // colorWipe(white, 50);    // Blue

  pinMode(LEFT_BLUE, OUTPUT);
  pinMode(LEFT_WHITE, OUTPUT);
  pinMode(LEFT_GREEN, OUTPUT);

  pinMode(RIGHT_BLUE, OUTPUT);
  pinMode(RIGHT_WHITE, OUTPUT);
  pinMode(RIGHT_GREEN, OUTPUT);
  pinMode(LED7, OUTPUT);
  pinMode(LED8, OUTPUT);
  pinMode(LED9, OUTPUT);
  pinMode(LED10, OUTPUT);
  
//  digitalWrite(RIGHT_BLUE, HIGH);
//  digitalWrite(LEFT_WHITE, HIGH);
//  digitalWrite(LEFT_GREEN, HIGH);
}

void loop() {
  // Get current button state.
  // boolean newState = digitalRead(BUTTON_PIN);

  // Check if state changed from high to low (button press).
  // if((newState == LOW) && (oldState == HIGH)) {
    // Short delay to debounce button.
    // delay(20);
    // Check if button is still low after debounce.
    // newState = digitalRead(BUTTON_PIN);
    // if(newState == LOW) {      // Yes, still low
    strip.Update();
    button.poll();
    if(button.onPress()){
       Serial.println("button pressed");
      switch(mode) {           // Start the new animation...
             Serial.println(mode);

        case 0:
          Serial.println(mode);
          strip.Interval = 75;
          strip.Color1= blue;
          strip.Color2= off;
          strip.ActivePattern = FADE;
          break;
        case 1:
          Serial.println(mode);
          strip.Interval = 75;
          strip.Color1= red;
          strip.Color2= off;
          strip.ActivePattern = FADE;
          break;
        case 2:
           Serial.println(mode);
          strip.Color1= white;
          // strip.Color2 = white;
          strip.Interval = 40;
          strip.ActivePattern = COLOR_WIPE;
          break;
        case 3:
          Serial.println(mode);
          strip.Color1= red;
          strip.Interval = 40;
          strip.ActivePattern = COLOR_WIPE;
          break;
        case 4:
          Serial.println(mode);
          strip.Color1= blue;
          strip.Interval = 40;
          strip.ActivePattern = COLOR_WIPE;
          break;
        case 5:
          Serial.println(mode);
          strip.Interval = 550;
          strip.Color1= blue;
          strip.Color2= red;
          strip.ActivePattern = THEATER_CHASE;
          break;
      }       
        if(++mode >5) mode = 0; // Advance to next mode, wrap around after #8


    }
  

  // Set the last-read button state to the old state.
  // oldState = newState;
 currentMillis = millis();

  if (currentMillis - blinkpreviousMillis >= blinkinterval) {
    // save the last time you blinked the LED
    blinkpreviousMillis = currentMillis;
    randomNumberArray = random(0,11);
    digitalWrite(LEDS[randomNumberArray], HIGH);
    randomNumberArray = random(0,11);
    digitalWrite(LEDS[randomNumberArray], LOW);
  
}
}
// Fill strip pixels one after another with a color. Strip is NOT cleared
// first; anything there will be covered pixel by pixel. Pass in color
// (as a single 'packed' 32-bit value, which you can get by calling
// strip.Color(red, green, blue) as shown in the loop() function above),
// and a delay time (in milliseconds) between pixels.
// void colorWipe(uint32_t color, int wait) {
//   for(int i=0; i<strip.numPixels(); i++) { // For each pixel in strip...
//     strip.setPixelColor(i, color);         //  Set pixel's color (in RAM)
//     strip.show();                          //  Update strip to match
//     delay(wait);                           //  Pause for a moment
//   }
// }

// // Theater-marquee-style chasing lights. Pass in a color (32-bit value,
// // a la strip.Color(r,g,b) as mentioned above), and a delay time (in ms)
// // between frames.
// void theaterChase(uint32_t color, int wait) {
//   for(int a=0; a<10; a++) {  // Repeat 10 times...
//     for(int b=0; b<3; b++) { //  'b' counts from 0 to 2...
//       strip.clear();         //   Set all pixels in RAM to 0 (off)
//       // 'c' counts up from 'b' to end of strip in steps of 3...
//       for(int c=b; c<strip.numPixels(); c += 3) {
//         strip.setPixelColor(c, color); // Set pixel 'c' to value 'color'
//       }
//       strip.show(); // Update strip with new contents
//       delay(wait);  // Pause for a moment
//     }
//   }
// }

// // Rainbow cycle along whole strip. Pass delay time (in ms) between frames.
// void rainbow(int wait) {
//   // Hue of first pixel runs 3 complete loops through the color wheel.
//   // Color wheel has a range of 65536 but it's OK if we roll over, so
//   // just count from 0 to 3*65536. Adding 256 to firstPixelHue each time
//   // means we'll make 3*65536/256 = 768 passes through this outer loop:
//   for(long firstPixelHue = 0; firstPixelHue < 3*65536; firstPixelHue += 256) {
//     for(int i=0; i<strip.numPixels(); i++) { // For each pixel in strip...
//       // Offset pixel hue by an amount to make one full revolution of the
//       // color wheel (range of 65536) along the length of the strip
//       // (strip.numPixels() steps):
//       int pixelHue = firstPixelHue + (i * 65536L / strip.numPixels());
//       // strip.ColorHSV() can take 1 or 3 arguments: a hue (0 to 65535) or
//       // optionally add saturation and value (brightness) (each 0 to 255).
//       // Here we're using just the single-argument hue variant. The result
//       // is passed through strip.gamma32() to provide 'truer' colors
//       // before assigning to each pixel:
//       strip.setPixelColor(i, strip.gamma32(strip.ColorHSV(pixelHue)));
//     }
//     strip.show(); // Update strip with new contents
//     delay(wait);  // Pause for a moment
//   }
// }

// // Rainbow-enhanced theater marquee. Pass delay time (in ms) between frames.
// void theaterChaseRainbow(int wait) {
//   int firstPixelHue = 0;     // First pixel starts at red (hue 0)
//   for(int a=0; a<30; a++) {  // Repeat 30 times...
//     for(int b=0; b<3; b++) { //  'b' counts from 0 to 2...
//       strip.clear();         //   Set all pixels in RAM to 0 (off)
//       // 'c' counts up from 'b' to end of strip in increments of 3...
//       for(int c=b; c<strip.numPixels(); c += 3) {
//         // hue of pixel 'c' is offset by an amount to make one full
//         // revolution of the color wheel (range 65536) along the length
//         // of the strip (strip.numPixels() steps):
//         int      hue   = firstPixelHue + c * 65536L / strip.numPixels();
//         uint32_t color = strip.gamma32(strip.ColorHSV(hue)); // hue -> RGB
//         strip.setPixelColor(c, color); // Set pixel 'c' to value 'color'
//       }
//       strip.show();                // Update strip with new contents
//       delay(wait);                 // Pause for a moment
//       firstPixelHue += 65536 / 90; // One cycle of color wheel over 90 frames
//     }
//   }
// }

// strip Completion Callback
void stripComplete()
{

      strip.Reverse();

}
