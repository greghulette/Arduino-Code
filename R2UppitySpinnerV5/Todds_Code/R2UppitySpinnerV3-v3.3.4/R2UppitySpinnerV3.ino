/*
 * --------------------------------------------------------------------
 * R2UppitySpinnerV3 (https://github.com/reeltwo/R2UppitySpinnerV3)
 * --------------------------------------------------------------------
 * Written by Mimir Reynisson (skelmir)
 *
 * R2UppitySpinnerV3 is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * R2UppitySpinnerV3 is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with R2UppitySpinnerV3; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wimplicit-fallthrough"

#ifdef SOC_LEDC_SUPPORT_HS_MODE
#define LEDC_CHANNELS           (SOC_LEDC_CHANNEL_NUM<<1)
#else
#define LEDC_CHANNELS           (SOC_LEDC_CHANNEL_NUM)
#endif

extern uint8_t channels_resolution[LEDC_CHANNELS];

///////////////////////////////////

#if __has_include("build_version.h")
#include "build_version.h"
#else
#define BUILD_VERSION "custom"
#endif

#if __has_include("reeltwo_build_version.h")
#include "reeltwo_build_version.h"
#endif

///////////////////////////////////

///////////////////////////////////
// FEATURE FLAGS
// Comment out any line to disable that feature.
///////////////////////////////////

#define USE_DEBUG           // Print diagnostic messages to the USB serial console

// Pre-declare the debug output functions BEFORE ReelTwo.h so that every
// DEBUG_PRINT / DEBUG_PRINTLN in Reeltwo's inline headers (WifiWebServer,
// WifiSerialBridge, etc.) goes through our filter instead of straight to
// Serial.  The actual function bodies are defined later, after WebLogBuffer.
//
// sDebugSuppressNewClient is declared here (defined later) so templates can
// check it.  The webLogBuffer forward ref isn't possible (it's a class instance),
// so template overloads for non-string types only go to Serial — that's fine,
// the "New client:" first arg is always a string which triggers suppression.
extern volatile bool sDebugSuppressNewClient;
void _debugPrint(const char* s);
void _debugPrint(const String& s);
void _debugPrintln(const char* s);
void _debugPrintln(const String& s);
template<typename T> void _debugPrint(const T& v) {
    if (!sDebugSuppressNewClient) Serial.print(v);
}
template<typename T> void _debugPrintln(const T& v) {
    if (!sDebugSuppressNewClient) Serial.println(v);
    sDebugSuppressNewClient = false;
}
// Zero-arg println
inline void _debugPrintln() { Serial.println(); sDebugSuppressNewClient = false; }

#define DEBUG_SERIAL Serial
#define DEBUG_SERIAL_READY() Serial.begin(115200)
#define DEBUG_PRINT(s)       _debugPrint(s)
#define DEBUG_PRINTLN(...)   _debugPrintln(__VA_ARGS__)
#define DEBUG_PRINTLN_HEX(s) Serial.println(s, HEX)
#define DEBUG_PRINT_HEX(s)   Serial.print(s, HEX)
#define DEBUG_PRINTF(...)    Serial.printf(__VA_ARGS__)
// #define USE_SMQDEBUG     // Extra debug output for the wireless remote protocol (very verbose)
#define USE_DROID_REMOTE    // Enable the handheld Droid Remote controller (wireless)
#define USE_WIFI            // Enable WiFi so R2 can be controlled from a phone or browser
#define USE_WIFI_WEB        // Enable the built-in web configuration page
#define USE_WIFI_MARCDUINO  // Accept Marcduino commands over WiFi (TCP port 2000)
#define USE_MDNS            // Advertise R2’s name on the network (lets you use "R2Uppity.local")
#define USE_OTA             // Allow firmware updates over WiFi (no USB cable needed)

// Uncomment only if your PCB has an SD card slot (used for firmware updates).
//#define USE_SDCARD

///////////////////////////////////
// FIRMWARE VERSION
// Bump this string with each release so serial, web, and #PCONFIG all show
// which build is running.
///////////////////////////////////
#define FIRMWARE_VERSION "3.3.4"

///////////////////////////////////
// CONFIGURABLE OPTIONS
///////////////////////////////////

// --- Droid Remote wireless credentials ---
// The remote and R2 must share the same hostname and secret to pair.
#ifdef USE_DROID_REMOTE
#define REMOTE_ENABLED                      true        // Remote support on by default
#define SMQ_HOSTNAME                        "R2Uppity"  // Name R2 announces over the air
#define SMQ_SECRET                          "Astromech" // Shared passphrase for pairing
#endif

// --- WiFi credentials ---
// true = R2 creates its own hotspot; false = R2 joins your existing network.
#ifdef USE_WIFI
#define WIFI_ENABLED                        true
#define WIFI_AP_NAME                        "R2Uppity"   // WiFi network name R2 broadcasts
#define WIFI_AP_PASSPHRASE                  "Astromech"  // WiFi password
#define WIFI_ACCESS_POINT                   true  // true = R2 is the hotspot; false = R2 joins your network
#endif

// --- Marcduino serial command settings ---
#define MARC_SERIAL_ENABLED                 true   // Accept Marcduino commands via serial
#define MARC_WIFI_ENABLED                   false  // Accept Marcduino commands via WiFi

///////////////////////////////////
// TIMING AND BUFFER SIZES
///////////////////////////////////

#define SERIAL_BAUD_RATE                    9600
#define CONSOLE_BUFFER_SIZE                 300
#define COMMAND_BUFFER_SIZE                 256

// --- Rotary motor ramp-up / ramp-down tuning ---
// Higher acceleration scale = slower ramp-up; lower deceleration scale = snappier stop.
#define ROTARY_THROTTLE_ACCELERATION_SCALE  100
#define ROTARY_THROTTLE_DECELERATION_SCALE  20
#define ROTARY_THROTTLE_LATENCY             25   // Milliseconds between each speed step

// Acceptable angular error before the rotary is considered "at target" (degrees).
#define ROTARY_FUDGE_POSITION               5

// --- Random movement mode ("M" command) timing ---
#define MOVEMODE_MAX_INTERVAL               5

// --- Calibration ---
// Pause between speed levels during calibration so the motor can cool.
// If your motor runs hot during calibration, increase this value.
#define CALIBRATION_COOLDOWN_MS             2500
// Maximum PID-output-limit search attempts per speed level.
// Prevents excessive motor run time when a level is borderline.
#define CALIBRATION_MAX_TRIES               4

///////////////////////////////////
// MOTOR POWER AND TRAVEL DEFAULTS
// The calibration routine sets the right values for your build automatically.
// Change the DEFAULT_* lines to switch between lifter types.
///////////////////////////////////

// --- IA-Parts periscope lifter ---
// "Minimum power" is the lowest throttle that actually moves the motor.
#define IAPARTS_LIFTER_MINIMUM_POWER        30
#define IAPARTS_LIFTER_SEEKBOTTTOM_POWER    30
#define IAPARTS_ROTARY_MINIMUM_POWER        20
#define IAPARTS_LIFTER_DISTANCE             845  // encoder ticks, full travel (set by calibration)

// --- Greg Hulette periscope lifter (heavier, needs more power) ---
#define GREG_LIFTER_MINIMUM_POWER           65
#define GREG_LIFTER_SEEKBOTTTOM_POWER       40
#define GREG_ROTARY_MINIMUM_POWER           40
#define GREG_LIFTER_DISTANCE                450

// --- Active defaults (Greg Hulette lifter) ---
#define DEFAULT_LIFTER_MINIMUM_POWER        GREG_LIFTER_MINIMUM_POWER
#define DEFAULT_LIFTER_SEEKBOTTTOM_POWER    GREG_LIFTER_SEEKBOTTTOM_POWER
#define DEFAULT_ROTARY_MINIMUM_POWER        GREG_ROTARY_MINIMUM_POWER
#define DEFAULT_LIFTER_DISTANCE             GREG_LIFTER_DISTANCE

// Rotary is blocked below this height to avoid hitting the dome.
#define DEFAULT_ROTARY_MINIMUM_HEIGHT       DEFAULT_LIFTER_DISTANCE/2

#define MOTOR_TIMEOUT                       2000  // ms before idle motors are powered off

// Internal PID constants — you should not need to change these.
#define OUTPUT_LIMIT_PRESCALE               3.1
#define DISTANCE_OUTPUT_SCALE               3

#define MAX_COMMANDS                        100
#define COMMAND_SERIAL                      Serial2

// Uncomment only if the rotary unit has been physically removed.
// NEVER enable this with the rotary installed — it bypasses spin safety checks.
//#define DISABLE_ROTARY

#ifdef DISABLE_ROTARY
#define DISABLE_SAFETY_MANEUVER
#endif

#include "pin-map.h"

//////////////////////////////
// LIGHT KIT TRI-STATE
//////////////////////////////
//   A    B    C
// OPEN OPEN OPEN  (Switch Position 0): Full Cycle (default): This routine will randomly select the LED
//                                      color, pattern, and speed for a random period of time.
// OPEN OPEN GND   (Switch Position 1): Off: This setting turns ALL lights OFF. I added this to allow a
//                                      microcontroller to turn off the lights without having to kill
//                                      the supply power.
// OPEN GND  OPEN  (Switch Position 2): Obi Wan: The Top LED’s flash Blue, the Side LED’s are Blue,
//                                      and the Main White LED’s are Random.
// OPEN GND  GND   (Switch Position 3): Yoda: The Top LED’s and Side LED’s fade Green On and Off.
// GND  OPEN OPEN  (Switch Position 4): Sith: The Top LED’s and Side LED’s flash Red.
// GND  OPEN GND   (Switch Position 5): Search Light: All LED’s are White, the Center Bright LED is ON.
// GND  GND  OPEN  (Switch Position 6): Dagobah: This is the most screen accurate mode.
//                                      The Main White LED’s are ON, the side LED’s are White, the Lower 
//                                      Rectangular Red LED’s are All On, and the Rear LED’s are Blinking Red.
// GND GND GND     (Switch Position 7): Sparkle: All White LED’s randomly Flash
///////////////////////////////////

#define PREFERENCE_REMOTE_ENABLED       "remote"
#define PREFERENCE_REMOTE_HOSTNAME      "rhost"
#define PREFERENCE_REMOTE_SECRET        "rsecret"
#define PREFERENCE_REMOTE_PAIRED        "rpaired"
#define PREFERENCE_REMOTE_LMK           "rlmk"
#define PREFERENCE_WIFI_ENABLED         "wifi"
#define PREFERENCE_WIFI_SSID            "ssid"
#define PREFERENCE_WIFI_PASS            "pass"
#define PREFERENCE_WIFI_AP              "ap"

#define PREFERENCE_MARCSERIAL           "mserial"
#define PREFERENCE_MARCWIFI_ENABLED     "mwifi"

#define PREFERENCES_PARAM_LIFTER_MINIMUM_POWER  "lftminpwr"
#define PREFERENCES_PARAM_LIFTER_SEEKBOT_POWER  "lftseekbotpwr"
#define PREFERENCES_PARAM_ROTARY_MINIMUM_POWER  "rotminpwr"
#define PREFERENCES_PARAM_LIFTER_DISTANCE        "lftdist"
#define PREFERENCES_PARAM_ROTARY_MININUM_HEIGHT  "minheight"
#define PREFERENCES_PARAM_ROTARY_ENCODER_COUNT   "rotenccount"
#define PREFERENCES_PARAM_DRIFT_CORRECTION_PCT  "driftpct"
#define DEFAULT_DRIFT_CORRECTION_PCT            5   // re-seek if drifted more than 5%

///////////////////////////////////

#ifdef USE_DROID_REMOTE
#include "ReelTwoSMQ32.h"
#else
#include "ReelTwo.h"
#endif
#include "core/SetupEvent.h"
#include "core/AnimatedEvent.h"
//#include "core/AnalogWrite.h"
#include "encoder/PPMReader.h"
#include "Wire.h"

// ReelTwo.h just redefined DEBUG_PRINT/DEBUG_PRINTLN to go straight to Serial.
// Re-override them before including WifiWebServer.h / WifiMarcduinoReceiver.h
// so the "New client:" messages in those headers go through our filter.
#undef DEBUG_PRINT
#undef DEBUG_PRINTLN
#define DEBUG_PRINT(s)       _debugPrint(s)
#define DEBUG_PRINTLN(...)   _debugPrintln(__VA_ARGS__)

#ifdef USE_WIFI
 #include "wifi/WifiAccess.h"
 #include <ESPmDNS.h>
 #ifdef USE_WIFI_WEB
  #include "wifi/WifiWebServer.h"
 #endif
 #ifdef USE_WIFI_MARCDUINO
  #include "wifi/WifiMarcduinoReceiver.h"
 #endif
#endif
#ifdef USE_OTA
#include <ArduinoOTA.h>
#endif
#include <Preferences.h>

///////////////////////////////////////////////////////////////////////////////
// WEB LOG BUFFER
// Captures DEBUG output and streams it to the /log web page via AJAX polling.
// Stores the last MAX_LINES log lines keyed by a monotonic index so the
// browser can ask "give me everything after index N" without missing entries.
///////////////////////////////////////////////////////////////////////////////

#ifdef USE_WIFI_WEB
class WebLogBuffer : public Print {
public:
    static const int MAX_LINES = 50;

    WebLogBuffer() : fNextIdx(0), fHead(0), fCount(0) {}

    size_t write(uint8_t c) override {
        if (c == '\n') {
            if (fCurrentLine.length() > 0) {
                fLines[fHead].fIdx = fNextIdx++;
                fLines[fHead].fMsg = fCurrentLine;
                fHead = (fHead + 1) % MAX_LINES;
                if (fCount < MAX_LINES) fCount++;
                fCurrentLine = "";
            }
        } else if (c != '\r') {
            if (fCurrentLine.length() < 120) fCurrentLine += (char)c;
        }
        return 1;
    }

    size_t write(const uint8_t* buf, size_t size) override {
        for (size_t i = 0; i < size; i++) write(buf[i]);
        return size;
    }

    // Returns log lines with idx > since, formatted as "idx\tmsg\n"
    String getLinesAfter(int since) {
        String result;
        int start = (fCount >= MAX_LINES) ? fHead : 0;
        for (int i = 0; i < fCount; i++) {
            int pos = (start + i) % MAX_LINES;
            if ((int)fLines[pos].fIdx > since) {
                result += String(fLines[pos].fIdx);
                result += '\t';
                result += fLines[pos].fMsg;
                result += '\n';
            }
        }
        return result;
    }

    int getLatestIdx() const { return (int)fNextIdx - 1; }

private:
    struct { uint32_t fIdx; String fMsg; } fLines[MAX_LINES];
    uint32_t fNextIdx;
    int fHead;
    int fCount;
    String fCurrentLine;
};

WebLogBuffer webLogBuffer;

// Implement the debug output functions declared before the ReelTwo includes.
// These tee output to both USB serial and the web log buffer, and suppress
// noisy "New client:" messages from the Reeltwo web server library.
volatile bool sDebugSuppressNewClient;

static bool _checkSuppress(const char* s)
{
    if (!sDebugSuppressNewClient && strncmp(s, "New client", 10) == 0)
        sDebugSuppressNewClient = true;
    return sDebugSuppressNewClient;
}

void _debugPrint(const char* s)
{
    if (!_checkSuppress(s)) { webLogBuffer.print(s); Serial.print(s); }
}
void _debugPrint(const String& s)
{
    if (!_checkSuppress(s.c_str())) { webLogBuffer.print(s); Serial.print(s); }
}
void _debugPrintln(const char* s)
{
    if (!_checkSuppress(s)) { webLogBuffer.println(s); Serial.println(s); }
    sDebugSuppressNewClient = false;
}
void _debugPrintln(const String& s)
{
    if (!_checkSuppress(s.c_str())) { webLogBuffer.println(s); Serial.println(s); }
    sDebugSuppressNewClient = false;
}
#endif // USE_WIFI_WEB

#ifdef USE_SDCARD
#include "FS.h"
#include "SD.h"
#endif

#ifdef USE_DROID_REMOTE
#define USE_MENUS
#endif

Preferences preferences;

///////////////////////////////////

#include "core/PinManager.h"

#ifdef USE_I2C_GPIO_EXPANDER
#include "PCF8574.h"
#ifndef GPIO_EXPANDER_ADDRESS
#define GPIO_EXPANDER_ADDRESS 0x20
#endif

static volatile bool sDigitalReadAll = true;
static void IRAM_ATTR flagDigitalReadAll()
{
    sDigitalReadAll = true;
}

class CustomPinManager : public PinManager
{
public:
    typedef PCF8574::DigitalInput DigitalInput;

    CustomPinManager(byte i2cAddress = GPIO_EXPANDER_ADDRESS) :
        fGPIOExpander(i2cAddress, PIN_GPIO_INTERRUPT, flagDigitalReadAll)
    {}

    virtual bool digitalRead(uint8_t pin) override
    {
        if (pin >= GPIO_PIN_BASE)
        {
            return fGPIOExpander.digitalRead(pin-GPIO_PIN_BASE, sDigitalReadAll);
        }
        return PinManager::digitalRead(pin);
    }
    virtual void digitalWrite(uint8_t pin, uint8_t val) override
    {
        if (pin >= GPIO_PIN_BASE)
        {
            fGPIOExpander.digitalWrite(pin-GPIO_PIN_BASE, val);
        }
        else
        {
            PinManager::digitalWrite(pin, val);
        }
    }
    virtual void pinMode(uint8_t pin, uint8_t mode) override
    {
        if (pin >= GPIO_PIN_BASE)
        {
            fGPIOExpander.pinMode(pin-GPIO_PIN_BASE, mode);
        }
        else
        {
            PinManager::pinMode(pin, mode);
        }
    }

    virtual void begin() override
    {
        fGPIOExpander.begin();
    }

    DigitalInput digitalReadAll()
    {
        return fGPIOExpander.digitalReadAll();
    }

protected:
    PCF8574 fGPIOExpander;
};
CustomPinManager sPinManager;

#else
PinManager sPinManager;
#endif

///////////////////////////////////
// STATUS LED — board RGB LED showing current operating state
///////////////////////////////////

#ifdef PIN_STATUSLED
#include "core/SingleStatusLED.h"

// LED mode codes — each maps to a color or blink pattern.
enum {
    kNormalModeHome = 0,   // solid green
    kWifiModeHome = 1,     // solid blue
    kNormalModeAway = 2,   // solid red
    kWifiModeAway = 3,     // alternating blue/red
    kNormalModeMoving = 4, // alternating green/yellow
    kWifiModeMoving = 5,   // alternating blue/yellow
    kSafetyMode = 6,       // solid yellow
};
unsigned sCurrentMode = kNormalModeHome;

// 4-frame color animation table per mode {R, G, B}. Values capped at 2 to stay dim.
static constexpr uint8_t kStatusColors[][4][3] = {
      { {  0,   2,    0} , {   0,    2,    0} , {  0,   2,    0} , {   0,    2,    0}  },  // normal mode home    (all green)
      { {  0,   0,    2} , {   0,    0,    2} , {  0,   0,    2} , {   0,    0,    2}  },  // wifi mode home      (all blue)
      { {  2,   0,    0} , {   2,    0,    0} , {  2,   0,    0} , {   2,    0,    0}  },  // normal mode away    (all red)
      { {  0,   0,    2} , {   2,    0,    0} , {  0,   0,    2} , {   2,    0,    0}  },  // wifi mode away      (blue, red, blue, red)
      { {  0,   2,    0} , {   2,    2,    0} , {  0,   2,    0} , {   2,    2,    0}  },  // normal mode moving  (green, yellow, green, yellow)
      { {  0,   0,    2} , {   2,    2,    0} , {  0,   0,    2} , {   2,    2,    0}  },  // wifi mode moving    (blue, yellow, blue, yellow)
      { {  2,   2,    0} , {   2,    2,    0} , {  2,   2,    0} , {   2,    2,    0}  }   // safety mode         (all yellow)
};
typedef SingleStatusLED<PIN_STATUSLED> StatusLED;
StatusLED statusLED(kStatusColors, SizeOfArray(kStatusColors));
#endif

#include "drive/TargetSteering.h"
#include "core/EEPROMSettings.h"

///////////////////////////////////
// USER-ADJUSTABLE MOTOR PARAMETERS
// Stored in flash; updated by the calibration routine or web UI.
///////////////////////////////////

struct LifterParameters
{
    int fLifterMinPower;        // Minimum throttle % to lift
    int fLifterMinSeekBotPower; // Minimum throttle % to lower (usually less than up)
    int fRotaryMinPower;        // Minimum throttle % to spin
    int fLifterDistance;        // Encoder ticks for full travel from bottom to top (set by calibration)
    int fRotaryMinHeight;       // Lifter must be above this tick count before spinning is allowed
    int fRotaryEncoderCount;    // Ticks per full 360° rotation, cached across reboots
    int fDriftCorrectionPct;    // Re-seek if lifter drifts more than this % (0 = disabled)

    void load()
    {
        fLifterMinPower = preferences.getInt(PREFERENCES_PARAM_LIFTER_MINIMUM_POWER, DEFAULT_LIFTER_MINIMUM_POWER);
        fLifterMinSeekBotPower = preferences.getInt(PREFERENCES_PARAM_LIFTER_SEEKBOT_POWER, DEFAULT_LIFTER_SEEKBOTTTOM_POWER);
        fRotaryMinPower = preferences.getInt(PREFERENCES_PARAM_ROTARY_MINIMUM_POWER, DEFAULT_ROTARY_MINIMUM_POWER);
        fLifterDistance = preferences.getInt(PREFERENCES_PARAM_LIFTER_DISTANCE, DEFAULT_LIFTER_DISTANCE);
        fRotaryMinHeight = preferences.getInt(PREFERENCES_PARAM_ROTARY_MININUM_HEIGHT, DEFAULT_ROTARY_MINIMUM_HEIGHT);
        fRotaryEncoderCount = preferences.getInt(PREFERENCES_PARAM_ROTARY_ENCODER_COUNT, 0);
        fDriftCorrectionPct = preferences.getInt(PREFERENCES_PARAM_DRIFT_CORRECTION_PCT, DEFAULT_DRIFT_CORRECTION_PCT);
    }

    void save()
    {
        preferences.putInt(PREFERENCES_PARAM_LIFTER_MINIMUM_POWER, fLifterMinPower);
        preferences.putInt(PREFERENCES_PARAM_LIFTER_SEEKBOT_POWER, fLifterMinSeekBotPower);
        preferences.putInt(PREFERENCES_PARAM_ROTARY_MINIMUM_POWER, fRotaryMinPower);
        preferences.putInt(PREFERENCES_PARAM_LIFTER_DISTANCE, fLifterDistance);
        preferences.putInt(PREFERENCES_PARAM_ROTARY_MININUM_HEIGHT, fRotaryMinHeight);
        preferences.putInt(PREFERENCES_PARAM_ROTARY_ENCODER_COUNT, fRotaryEncoderCount);
        preferences.putInt(PREFERENCES_PARAM_DRIFT_CORRECTION_PCT, fDriftCorrectionPct);
    }
};

LifterParameters sLifterParameters;

#define LIFTER_MINIMUM_POWER        sLifterParameters.fLifterMinPower
#define LIFTER_SEEKBOTTTOM_POWER    sLifterParameters.fLifterMinSeekBotPower
#define ROTARY_MINIMUM_POWER        sLifterParameters.fRotaryMinPower
#define LIFTER_DISTANCE             sLifterParameters.fLifterDistance
#define ROTARY_MINIMUM_HEIGHT       sLifterParameters.fRotaryMinHeight

///////////////////////////////////
// CALIBRATION DATA (saved to flash)
// The calibration routine fills these PID limit tables for each 5% speed step.
///////////////////////////////////

#define ENCODER_STATUS_RATE 200 // milliseconds between stall checks

struct OutputLimit
{
    bool valid;
    unsigned outputLimit;
};

// All settings saved to flash between power cycles.
struct LifterSettings
{
    // Calibration tables: one entry per 5% speed step, filled by the calibration routine.
    OutputLimit fUpLimits[100/5+1];
    OutputLimit fDownLimits[sizeof(fUpLimits)/sizeof(fUpLimits[0])];

    unsigned fMinimumPower = 0;    // Lowest speed % that actually moves the lifter
    unsigned fLifterDistance = 0;  // Encoder ticks from bottom to top

    union
    {
        struct
        {
            bool fLifterLimitSetting:1;  // Logic level of lifter limit switches (NO vs NC wiring)
            bool fRotaryLimitSetting:1;  // Logic level of the rotary home switch
            bool fUpLimitsCalibrated:1;
            bool fDownLimitsCalibrated:1;
            bool fSafetyManeuver:1;      // (unused)
            bool fDisableRotary:1;       // Set true via config command to disable rotary
        };
        uint8_t fFlags;
    };

    unsigned fBaudRate = SERIAL_BAUD_RATE;
    unsigned fID = 0;        // Optional unit ID for multi-board setups
    unsigned fReserved1 = 0;
    unsigned fReserved2 = 0;
    unsigned fReserved3 = 0;

    unsigned getLifterDistance()
    {
        return fLifterDistance != 0 ? fLifterDistance : sLifterParameters.fLifterDistance;
    }

    static constexpr size_t limitCount()
    {
        return sizeof(fUpLimits)/sizeof(fUpLimits[0]);
    }
};
EEPROMSettings<LifterSettings> sSettings;

///////////////////////////////////
// GLOBAL STATE VARIABLES
///////////////////////////////////

static volatile bool sCalibrating;
static volatile bool sSafetyManeuver;        // true once the startup safety maneuver succeeds
static volatile bool sSafetyManeuverFailed;  // volatile: written on core 1, read on core 0 (web)
static volatile bool sWebAbort;              // set by E-STOP from core 0; checked by seek loops on core 1
static volatile uint32_t sRescueOverrideExpiry;  // millis() timestamp when safety override expires (0 = off)

// Encoder ticks per full 360° rotation, measured during the safety maneuver.
static unsigned sRotaryCircleEncoderCount;

///////////////////////////////////
// COMMAND BUFFER — incoming serial characters accumulate here until a newline
///////////////////////////////////

static unsigned sPos;
static unsigned sCopyPos;
static bool sProcessing;
static bool sNextCommand;
static uint32_t sWaitNextSerialCommand;
static char sBuffer[CONSOLE_BUFFER_SIZE];
static char sCopyBuffer[sizeof(sBuffer)+4];    // Copy of last command, shown on the web diagnostics page
static bool sCmdNextCommand;
static char sCmdBuffer[COMMAND_BUFFER_SIZE];
static bool sRCMode;
static bool sVerboseDebug;

static void runSerialCommand()
{
    sWaitNextSerialCommand = 0;
    sProcessing = (sPos != 0);
}

static void resetSerialCommand()
{
    sWaitNextSerialCommand = 0;
    sNextCommand = false;
    sProcessing = (sCmdBuffer[0] == ':');
    sPos = 0;
}

// Format a printf-style command string and queue it for immediate execution.
static void executeCommand(const char* cmd, ...)
{
    va_list targ;
    sPos = 0;
    va_start(targ, cmd);
    vsnprintf(sBuffer, sizeof(sBuffer), cmd, targ);
    va_end(targ);
    sPos = strlen(sBuffer);
    // Copy command into sCopyBuffer so the web CMD widget shows web-originated commands too
    strncpy(sCopyBuffer, sBuffer, sizeof(sCopyBuffer) - 1);
    sCopyBuffer[sizeof(sCopyBuffer) - 1] = '\0';
    runSerialCommand();
}

///////////////////////////////////

PPMReader sPPM(PIN_PPMIN_RC, 6);

#ifdef USE_SDCARD
static bool sSDCardMounted;
#endif

bool mountReadOnlyFileSystem()
{
#ifdef USE_SPIFFS
    return (SPIFFS.begin(true));
#endif
    return false;
}

bool mountWritableFileSystem()
{
#ifdef USE_FATFS
    return (FFat.begin(true, "/fatfs"));
#endif
    return false;
}

bool getSDCardMounted()
{
#ifdef USE_SDCARD
    return sSDCardMounted;
#else
    return false;
#endif
}

bool mountSDFileSystem()
{
#ifdef USE_SDCARD
    if (SD.begin(PIN_SD_CS))
    {
        DEBUG_PRINTLN("Card Mount Success");
        sSDCardMounted = true;
        return true;
    }
    DEBUG_PRINTLN("Card Mount Failed");
#endif
    return false;
}

void unmountSDFileSystem()
{
#ifdef USE_SDCARD
    if (sSDCardMounted)
    {
        sSDCardMounted = false;
        SD.end();
    }
#endif
}

void unmountFileSystems()
{
    unmountSDFileSystem();
#ifdef USE_FATFS
    FFat.end();
#endif
#ifdef USE_SPIFFS
    SPIFFS.end();
#endif
}

///////////////////////////////////

// Cleanly shut down connections and restart the ESP32 (used after settings changes).
void reboot()
{
    Serial.println(F("Restarting..."));
#ifdef USE_DROID_REMOTE
    DisconnectRemote();
#endif
    if (sRCMode)
        sPPM.end();
    unmountFileSystems();
    preferences.end();
    ESP.restart();
}

///////////////////////////////////
// PERISCOPELIFTER CLASS
// Controls the lifter motor, rotary motor, and light kit.
// Position is tracked via encoder ticks; limit switches mark the endpoints.
///////////////////////////////////

class PeriscopeLifter : public SetupEvent
{
public:
    // Light kit modes — three output pins (A, B, C) form a 3-bit mode select.
    enum
    {
        kLightKit_FullCycle = 0,   // Random color/pattern cycling
        kLightKit_Off = 1,         // All lights off
        kLightKit_ObiWan = 2,      // Blue top LEDs, random white
        kLightKit_Yoda = 3,        // Fading green
        kLightKit_Sith = 4,        // Flashing red
        kLightKit_SearchLight = 5, // All white
        kLightKit_Dagobah = 6,     // Most screen-accurate mode
        kLightKit_Sparkle = 7
    };

    // Limit switch readers — return true when the switch is triggered.
    // fLifterLimitSetting/fRotaryLimitSetting handle NO vs NC wiring.

    static bool lifterTopLimit()
    {
        // First read: quickly reject the common case (limit not reached).
        if (sPinManager.digitalRead(PIN_LIFTER_TOPLIMIT) != sSettings.fLifterLimitSetting)
            return false;
        // Confirm with a second read after a delay to filter motor EMI noise.
        // Sustained stall-current EMI can last milliseconds; 3ms clears it while a real
        // switch activation (many ms of contact) still reads true on the second check.
        delayMicroseconds(3000);
        return (sPinManager.digitalRead(PIN_LIFTER_TOPLIMIT) == sSettings.fLifterLimitSetting);
    }

    static bool lifterBottomLimit()
    {
        bool limit = (sPinManager.digitalRead(PIN_LIFTER_BOTLIMIT) == sSettings.fLifterLimitSetting);
        return limit;
    }

    // Returns lifter height in encoder ticks (0 = bottom). Interrupts briefly disabled for atomic read.
    static long getLifterPosition()
    {
        long pos;
        cli();
        pos = encoder_lifter_ticks;
        sei();
        return pos;
    }

    // Clamped version: negative encoder positions are never valid (bottom = 0,
    // top = fLifterDistance).  After a bottom-limit reset the encoder can drift
    // a few ticks negative from motor coast or EMI phantom edges, which cascades
    // into broken height %, rotaryAllowed() failures, and ROTARY NOT ALLOWED spam.
    // Use this everywhere except inside seek loops where raw ticks matter.
    static long getLifterPositionClamped()
    {
        long pos = getLifterPosition();
        return (pos < 0) ? 0 : pos;
    }

    // Returns true when the rotary head is at its home (0°) position.
    static bool rotaryHomeLimit()
    {
    #ifdef DISABLE_ROTARY
        return true;
    #else
        bool limit = sSettings.fDisableRotary || (sPinManager.digitalRead(PIN_ROTARY_LIMIT) == sSettings.fRotaryLimitSetting);
        return limit;
    #endif
    }

    // Returns rotary position in encoder ticks (0 = home).
    static long getRotaryPosition()
    {
        long pos;
        cli();
        pos = encoder_rotary_ticks;
        sei();
        return pos;
    }

    // Motor fault detection — DIAG pin goes LOW on overcurrent or driver fault.

    static bool lifterMotorFault()
    {
        return !digitalRead(PIN_LIFTER_DIAG);
    }

    static bool rotaryMotorFault()
    {
        return !digitalRead(PIN_ROTARY_DIAG);
    }

    // Returns true when the periscope is fully down and motors have timed out.
    static bool isIdle()
    {
        if (lifterBottomLimit())
        {
            if (!fMotorsEnabled ||
                fMotorsEnabledTime + MOTOR_TIMEOUT < millis())
            {
                return true;
            }
        }
        return false;
    }

    // Motor power enable/disable — drivers are powered off when idle to save current.

    static bool motorsEnabled()
    {
        return fMotorsEnabled;
    }

    static void disableMotors()
    {
        sPinManager.digitalWrite(PIN_MOTOR_EN, LOW);
        sPinManager.digitalWrite(PIN_MOTOR_ENB, HIGH);

        fMotorsEnabled = false;
        fLifterTargetPos = -1;
        fLifterHolding = false;
    }

    static void enableMotors()
    {
        sPinManager.digitalWrite(PIN_MOTOR_EN, HIGH);
        sPinManager.digitalWrite(PIN_MOTOR_ENB, LOW);

        fMotorsEnabled = true;
        fMotorsEnabledTime = millis();
    #ifdef PIN_STATUSLED
        statusLED.setMode(sSafetyManeuver ? sCurrentMode + kNormalModeMoving : unsigned(kSafetyMode));
    #endif
    }

    // Write PWM to both motor direction pins. One high + one low = run; both low = stop.
    static inline void dualAnalogWrite(uint8_t m1, float v1, uint8_t m2, float v2)
    {
        const auto output1{static_cast<int32_t>(abs(v1) * 255)};
        const auto output2{static_cast<int32_t>(abs(v2) * 255)};
        ::analogWrite(m1, output1);
        ::analogWrite(m2, output2);
    }

    // Lifter motor control. throttle: -1.0 = full down, +1.0 = full up; values within ±0.10 treated as zero.

    static void lifterMotorMove(float throttle)
    {
        bool reverse = (throttle < 0);
        throttle = min(max(abs(throttle), 0.0f), 1.0f);

        if (throttle < 0.10)
            throttle = 0;
        if (fLifterThrottle != throttle)
        {
            enableMotors();
            if (reverse)
            {
                dualAnalogWrite(PIN_LIFTER_PWM1, 0, PIN_LIFTER_PWM2, fabs(throttle));
                fLifterThrottle = -throttle;
            }
            else
            {
                dualAnalogWrite(PIN_LIFTER_PWM1, fabs(throttle), PIN_LIFTER_PWM2, 0);
                fLifterThrottle = throttle;
            }
        }
    }

    static void lifterMotorStop()
    {
        enableMotors();
        dualAnalogWrite(PIN_LIFTER_PWM1, 0, PIN_LIFTER_PWM2, 0);
        fLifterThrottle = 0;
    }

    // Active brake: both PWMs HIGH shorts motor windings, resisting back-driving.
    // Used to hold position against vibration drift on lead screw lifters.
    static void lifterMotorBrake()
    {
        enableMotors();
        dualAnalogWrite(PIN_LIFTER_PWM1, 1.0, PIN_LIFTER_PWM2, 1.0);
        fLifterThrottle = 0;
    }

    ///////////////////////////////////

    // Breakaway boost: overcome static friction on a tight belt/lead screw,
    // then ramp into pulsed drive so the mechanism reaches steady-state speed.
    //
    // seekDistance: total ticks the caller intends to travel.  The ramp is
    //   scaled to at most 1/3 of this so momentum doesn't cause limit-slam.
    //   Pass 0 to use defaults (seekToTop/seekToBottom don't know distance).
    //
    // Phase 1 — CONTINUOUS BURST (up to maxMs):
    //   Run the motor without pulsing until the encoder confirms real movement
    //   (>= burstTicks).  This overcomes static friction.
    //
    // Phase 2 — RAMP TO PULSED:
    //   Gradually increase the off-period from 0ms to the normal 1ms over
    //   rampTicks of encoder travel.  This lets the lead screw reach a steady
    //   rotational speed where each 3ms pulse can sustain motion on its own.
    //
    // After both phases the caller's normal pulsed drive loop takes over.
    // Returns true if movement was detected, false if timed out (probable stall).
    static bool breakawayBoost(float power, long seekDistance = 0,
                               int burstTicks = 10, uint32_t maxMs = 500)
    {
        // Scale ramp to seek distance: never more than 1/3 of total travel,
        // capped at 80 ticks.  This prevents momentum buildup that causes
        // the motor to slam into limit switches.
        // For very short seeks (< 30 ticks), skip breakaway entirely.
        if (seekDistance > 0 && seekDistance < 30)
            return true;  // too short to need a boost
        int rampTicks;
        if (seekDistance > 0)
            rampTicks = min(80L, seekDistance / 3);
        else
            rampTicks = 80;  // default for seekToTop/seekToBottom (unknown distance)
        rampTicks = max(rampTicks, 20);  // minimum useful ramp

        // Only check the limit switch in the DIRECTION OF TRAVEL.
        // When starting from a limit (e.g. UP from bottom), the departing limit
        // switch is still active and would cause an immediate false exit.
        bool goingUp = (power > 0);

        long startPos = getLifterPosition();
        uint32_t startMs = millis();
        bool limitHit = false;

        // Phase 1: continuous burst until burstTicks of movement
        while (millis() - startMs < maxMs)
        {
            limitHit = goingUp ? lifterTopLimit() : lifterBottomLimit();
            if (limitHit || serialAbort())
                break;
            lifterMotorMove(power);  // continuous — no stop pulse
            delay(1);                // yield, ~1ms per iteration
            if (abs(getLifterPosition() - startPos) >= burstTicks)
                break;
        }
        long burstEnd = getLifterPosition();
        long burstDist = abs(burstEnd - startPos);

        if (burstDist < burstTicks && !limitHit)
        {
            lifterMotorStop();
            Serial.print("BREAKAWAY timeout (");
            Serial.print(burstDist);
            Serial.println(" ticks)");
            return false;
        }

        Serial.print("BREAKAWAY after ");
        Serial.print(millis() - startMs);
        Serial.print("ms");

        if (limitHit)
        {
            Serial.println(" (limit)");
            return true;
        }

        // Phase 2: ramp from continuous into pulsed drive.
        //   0–75%:  3ms on, no stop  (nearly continuous — builds inertia)
        //  75–90%:  3ms on, stop call only (micro-brake, ~0ms off)
        //  90–100%: 3ms on, stop + 1ms off (matches caller's pulse pattern)
        long rampStart = burstEnd;
        bool rampLimit = false;
        while (!serialAbort())
        {
            rampLimit = goingUp ? lifterTopLimit() : lifterBottomLimit();
            if (rampLimit) break;
            long traveled = abs(getLifterPosition() - rampStart);
            if (traveled >= rampTicks)
                break;
            lifterMotorMove(power);
            delay(3);
            if (traveled > rampTicks * 3 / 4)
            {
                // Last 25%: introduce braking
                lifterMotorStop();
                if (traveled > rampTicks * 9 / 10)
                    delay(1);  // last 10%: full 1ms off (matches normal pulse)
            }
            // 0–75%: no stop — motor runs continuously in 3ms chunks
        }
        Serial.print(" ramp="); Serial.print(abs(getLifterPosition() - rampStart));
        Serial.println(" ticks");
        return true;
    }

    // Returns true if an abort was requested (serial char during calibration,
    // or E-STOP from the web interface).
    static bool serialAbort()
    {
        if (sWebAbort)
        {
            DEBUG_PRINTLN("WEB ABORT");
            sWebAbort = false;
            return true;
        }
        if (sCalibrating && Serial.available())
        {
            DEBUG_PRINTLN("SERIAL ABORT");
            while (Serial.available())
                Serial.read();
            sCalibrating = false;
            return true;
        }
        return false;
    }

    static bool seekToPosition(float pos, float speed)
    {
        if (!ensureSafetyManeuver())
            return false;

        // ensure position is in the range of 0.0 [bottom] - 1.0 [top]
        pos = min(max(abs(pos), 0.0f), 1.0f);
        if (isRotarySpinning() || !rotaryHomeLimit())
        {
            // Cannot go below safe rotary height if spinning or not at home position
            float minSafePos = (float)ROTARY_MINIMUM_HEIGHT / (float)sSettings.getLifterDistance();
            pos = min(max(pos, minSafePos), 1.0f);
        }
        // Floor speed at fMinimumPower — below this the motor stalls mid-travel.
        // Allow higher speeds for manual control; pulsed drive + gravity (UP) or
        // pulsed ramp zone (DOWN) handle deceleration at limits.
        if (speed < sSettings.fMinimumPower / 100.0f)
            speed = sSettings.fMinimumPower / 100.0f;

        long maxlen = sSettings.getLifterDistance();
        long current = getLifterPositionClamped();
        long target_ticks = pos * maxlen;

        long distance = abs(target_ticks - current);
        // Skip trivially short seeks (< 5 ticks) unless targeting a limit switch.
        // EMI phantom ticks often put the encoder a few ticks off, producing
        // 0-3 tick "seeks" that are invisible but waste time and flood the log.
        if (distance < 5 && pos > 0.01f && pos < 0.99f)
        {
            return true;  // already there
        }
        // Pulsed drive (3ms on / 1ms off) the entire travel for mechanical braking.
        // Power ramp only in the last 20% of distance-to-target: from fMinimumPower
        // down to approach power (~40%). Full power for the first 80% so the heavy
        // periscope can still climb against gravity.
        float approachPower = LIFTER_SEEKBOTTTOM_POWER / 100.0f;  // ~0.40
        TargetSteering steering(target_ticks);
        steering.setSampleTime(1);
        float minpower = (1.0f - ((float)distance / (float)maxlen)) + 0.1;
        steering.setDistanceTunings(1.0, minpower, minpower);

        float limit;
        bool success = false;
        if (target_ticks > getLifterPosition())
        {
            // seek up
            if (!getUpOutputLimit(speed, limit))
            {
                Serial.print("SEEK UP ABORT: uncalibrated speed="); Serial.println(int(speed*100));
                return false;
            }
            // Do not use PID deceleration: the lead screw is self-locking so no deceleration
            // is needed, and calibrated outputLimits cause a long minimum-power zone that
            // makes the motor strain audibly. Run at constant speed; stop at target or limit.
            limit = 0;
            Serial.print("SEEK UP "); Serial.print(current); Serial.print("->"); Serial.print(target_ticks);
            Serial.print(" ("); Serial.print(int(pos*100)); Serial.print("%) speed="); Serial.println(int(speed*100));
            bool topLimit;
            bool reachedTarget = false;
            // Breakaway boost: continuous drive to overcome static friction on
            // tight belts / lead screws before switching to pulsed drive.
            breakawayBoost(speed, distance);
            // Use a position-change timeout instead of LifterStatus tick-counting,
            // which requires 20 ticks/200ms and falsely trips on slow motors (Greg lifter
            // at minimum power produces only ~15 ticks/200ms).
            long seekPrevPos = getLifterPosition();
            uint32_t seekPosChangeMs = millis();
            uint32_t seekLogMs = millis();
            // Slow-progress motor protection state
            long slowCheckPos = seekPrevPos;
            uint32_t slowCheckMs = millis();
            uint32_t slowDurationMs = 0;
            bool needsReboost = false;
            for (;;)
            {
                long encoder_ticks = getLifterPosition();
                topLimit = lifterTopLimit();
                reachedTarget = (encoder_ticks >= target_ticks);
                // When seeking full travel (pos=1.0), the physical top-limit switch is the
                // real stopping condition. Stopping at the stored fLifterDistance (reachedTarget)
                // leaves the lifter just short of the switch due to per-run encoder variability.
                // For full-travel seeks, rely on topLimit or the stall timeout instead.
                bool atFullTravel = (target_ticks >= maxlen);
                if (topLimit || (!atFullTravel && reachedTarget) || serialAbort())
                    break;
                // Periodic progress log every 500ms
                if (millis() - seekLogMs > 500)
                {
                    seekLogMs = millis();
                    Serial.print("  UP pos="); Serial.print(encoder_ticks);
                    Serial.print("/"); Serial.print(target_ticks);
                    Serial.print(" top="); Serial.print(topLimit ? "1" : "0");
                    Serial.print(" home="); Serial.print(rotaryHomeLimit() ? "1" : "0");
                    Serial.print(" liftFault="); Serial.println(lifterMotorFault() ? "YES" : "no");
                }
                if (encoder_ticks != seekPrevPos)
                {
                    seekPrevPos = encoder_ticks;
                    seekPosChangeMs = millis();
                    needsReboost = false;
                }
                if (millis() - seekPosChangeMs > 2000)
                {
                    Serial.print("SEEK UP ABORT: stall at pos="); Serial.println(encoder_ticks);
                    break;
                }
                // Slow-progress motor protection: if moving less than 10 ticks
                // per second for 3+ seconds, the motor is borderline stalling and
                // drawing near-stall current. Abort to prevent motor burnout.
                if (millis() - slowCheckMs > 1000)
                {
                    long ticksPerSec = abs(encoder_ticks - slowCheckPos) * 1000L / max(1UL, millis() - slowCheckMs);
                    slowCheckPos = encoder_ticks;
                    slowCheckMs = millis();
                    if (ticksPerSec < 10 && (target_ticks - encoder_ticks) > 20)
                    {
                        slowDurationMs += 1000;
                        if (slowDurationMs >= 3000)
                        {
                            Serial.print("SEEK UP ABORT: slow progress (");
                            Serial.print(ticksPerSec);
                            Serial.print(" ticks/s) at pos=");
                            Serial.println(encoder_ticks);
                            break;
                        }
                        // Re-boost: pulsed drive lost momentum on a sticky spot.
                        // Try another continuous burst before giving up.
                        if (!needsReboost)
                        {
                            needsReboost = true;
                            Serial.println("  RE-BOOST (slow progress)");
                            breakawayBoost(speed, target_ticks - encoder_ticks);
                            seekPrevPos = getLifterPosition();
                            seekPosChangeMs = millis();
                            continue;
                        }
                    }
                    else
                    {
                        slowDurationMs = 0;
                        needsReboost = false;
                    }
                }
                float throttle = 1.0f;
                if (limit > 0)
                {
                    steering.setCurrentDistance(encoder_ticks);
                    throttle = steering.getThrottle();
                    // Clamp so motor power never drops below fMinimumPower — the lead screw
                    // is self-locking, so any throttle below minimum just stalls the motor.
                    float minThrottle = sSettings.fMinimumPower / (100.0f * speed);
                    throttle = max(throttle, minThrottle);
                }
                // Pulsed drive (3ms on / 1ms off) for the ENTIRE ascent at
                // constant speed. No ramp needed: gravity is the brake on UP.
                // Each 3ms pulse moves ~1 encoder tick. Between pulses the motor
                // is stopped and the self-locking lead screw + gravity hold position.
                // Overshoot when target is reached = ~1 tick (one pulse of coast).
                {
                    lifterMotorMove(speed);
                    delay(3);
                    lifterMotorStop();
                    delay(1);
                }
            }
            lifterMotorStop();
            if (topLimit)
            {
                delay(100);                 // let motor coast/EMI settle
                resetLifterPositionTop();   // re-sync encoder to fLifterDistance
            }
            Serial.print("SEEK UP DONE pos="); Serial.print(getLifterPosition());
            Serial.print("/"); Serial.print(maxlen);
            Serial.print(topLimit ? " (topLimit)" : reachedTarget ? " (target)" : " (abort)");
            Serial.println();
            success = topLimit || reachedTarget;
        }
        else
        {
            // seek down
            if (!getDownOutputLimit(speed, limit))
            {
                Serial.print("SEEK DOWN ABORT: uncalibrated speed="); Serial.println(int(speed*100));
                return false;
            }
            // Do not use PID deceleration: same reasoning as seek up above.
            limit = 0;
            Serial.print("SEEK DOWN "); Serial.print(current); Serial.print("->"); Serial.print(target_ticks);
            Serial.print(" ("); Serial.print(int(pos*100)); Serial.print("%) speed="); Serial.println(int(speed*100));
            bool botLimit;
            bool reachedTarget = false;
            // Breakaway boost for descent
            breakawayBoost(-speed, distance);
            // Use a position-change timeout instead of LifterStatus tick-counting,
            // which requires 20 ticks/200ms and falsely trips on slow motors (Greg lifter
            // at minimum power produces only ~15 ticks/200ms).
            long seekPrevPos = getLifterPosition();
            uint32_t seekPosChangeMs = millis();
            uint32_t seekLogMs = millis();
            // Slow-progress motor protection state
            long slowCheckPos = seekPrevPos;
            uint32_t slowCheckMs = millis();
            uint32_t slowDurationMs = 0;
            bool needsReboost = false;
            for (;;)
            {
                long encoder_ticks = getLifterPosition();
                botLimit = lifterBottomLimit();
                reachedTarget = (encoder_ticks <= target_ticks);
                if (botLimit || reachedTarget || serialAbort())
                    break;
                // Periodic progress log every 500ms
                if (millis() - seekLogMs > 500)
                {
                    seekLogMs = millis();
                    Serial.print("  DN pos="); Serial.print(encoder_ticks);
                    Serial.print("/"); Serial.print(target_ticks);
                    Serial.print(" bot="); Serial.print(botLimit ? "1" : "0");
                    Serial.print(" home="); Serial.print(rotaryHomeLimit() ? "1" : "0");
                    Serial.print(" liftFault="); Serial.println(lifterMotorFault() ? "YES" : "no");
                }
                if (encoder_ticks != seekPrevPos)
                {
                    seekPrevPos = encoder_ticks;
                    seekPosChangeMs = millis();
                    needsReboost = false;
                }
                if (millis() - seekPosChangeMs > 2000)
                {
                    Serial.print("SEEK DOWN ABORT: stall at pos="); Serial.println(encoder_ticks);
                    break;
                }
                // Slow-progress motor protection (same as UP seek)
                if (millis() - slowCheckMs > 1000)
                {
                    long ticksPerSec = abs(encoder_ticks - slowCheckPos) * 1000L / max(1UL, millis() - slowCheckMs);
                    slowCheckPos = encoder_ticks;
                    slowCheckMs = millis();
                    if (ticksPerSec < 10 && (encoder_ticks - target_ticks) > 20)
                    {
                        slowDurationMs += 1000;
                        if (slowDurationMs >= 3000)
                        {
                            Serial.print("SEEK DOWN ABORT: slow progress (");
                            Serial.print(ticksPerSec);
                            Serial.print(" ticks/s) at pos=");
                            Serial.println(encoder_ticks);
                            break;
                        }
                        if (!needsReboost)
                        {
                            needsReboost = true;
                            Serial.println("  RE-BOOST (slow progress)");
                            breakawayBoost(-speed, encoder_ticks - target_ticks);
                            seekPrevPos = getLifterPosition();
                            seekPosChangeMs = millis();
                            continue;
                        }
                    }
                    else
                    {
                        slowDurationMs = 0;
                        needsReboost = false;
                    }
                }
                float throttle = -1.0f;
                if (limit > 0)
                {
                    steering.setCurrentDistance(encoder_ticks);
                    throttle = steering.getThrottle();
                    // Clamp so motor power never drops below fMinimumPower — the lead screw
                    // is self-locking, so any throttle below minimum just stalls the motor.
                    float minThrottle = sSettings.fMinimumPower / (100.0f * speed);
                    throttle = min(throttle, -minThrottle);
                }
                // Pulsed drive (3ms on / 1ms off) for the ENTIRE descent.
                // Full power for most of travel; ramp from speed to approachPower
                // in the last 20% of distance-to-target.
                {
                    long distToTarget = encoder_ticks - target_ticks;
                    long rampZone = max(30L, (current - target_ticks) / 5);  // last 20%
                    float power = speed;
                    if (distToTarget <= rampZone)
                    {
                        float ratio = max(0.0f, (float)distToTarget / (float)rampZone);
                        power = approachPower + ratio * (speed - approachPower);
                    }
                    lifterMotorMove(-power);
                    delay(3);
                    lifterMotorStop();
                    delay(1);
                }
            }
            lifterMotorStop();
            if (botLimit)
            {
                resetLifterPosition();
                delay(100);             // let motor coast/EMI settle
                resetLifterPosition();  // re-zero after settling
                long postReset = getLifterPosition();
                if (postReset != 0)
                {
                    Serial.print("WARNING: encoder non-zero after bottom reset: ");
                    Serial.println(postReset);
                }
            }
            Serial.print("SEEK DOWN DONE pos="); Serial.print(getLifterPosition());
            Serial.print("/"); Serial.print(maxlen);
            Serial.print(botLimit ? " (botLimit)" : reachedTarget ? " (target)" : " (abort)");
            Serial.println();
            success = botLimit || reachedTarget;
        }
        // Track target for drift correction. If going to bottom (pos ≈ 0),
        // clear the target — no point holding at the home position.
        if (success)
        {
            if (pos <= 0.01f)
            {
                fLifterTargetPos = -1;
                fLifterHolding = false;
                lifterMotorStop();
            }
            else
            {
                fLifterTargetPos = pos;
                fLifterHolding = true;
                lifterMotorStop();
            }
        }
        return success;
    }

    // Returns true when it is safe to spin: safety passed, rotary enabled, lifter above minimum height.
    static bool rotaryAllowed()
    {
    #ifdef DISABLE_ROTARY
        return false;
    #else
        return !sSafetyManeuverFailed && !sSettings.fDisableRotary && (getLifterPositionClamped() > sLifterParameters.fRotaryMinHeight);
    #endif
    }

    // Set continuous spin speed (-1.0 to +1.0); actual ramp is handled by rotaryMotorUpdate().
    static void rotaryMotorSpeed(float speed)
    {
        fRotarySpeed = (rotaryAllowed()) ? speed : 0;
        fRotaryEncoderLastStatus = millis();
        rotaryMotorUpdate();
    }

    // Ramps the rotary toward its target speed each loop; stops on stall detection.
    static void rotaryMotorUpdate()
    {
    #ifndef DISABLE_ROTARY
        if (sSettings.fDisableRotary)
            return;

        // SAFETY: if the lifter has descended below the safe spin height while the
        // rotary is running (continuous spin mode), cut motor power immediately.
        // Without this check the PWM output keeps driving the motor even though
        // rotaryAllowed() returns false, so the periscope head could hit the dome.
        if (!rotaryAllowed() && fRotaryThrottle != 0)
        {
            Serial.print("ROTARY STOPPED: safetyFail=");
            Serial.print(sSafetyManeuverFailed ? "1" : "0");
            Serial.print(" disabled=");
            Serial.print(sSettings.fDisableRotary ? "1" : "0");
            Serial.print(" pos=");
            Serial.print(getLifterPosition());
            Serial.print(" minH=");
            Serial.print(sLifterParameters.fRotaryMinHeight);
            Serial.print(" home=");
            Serial.println(rotaryHomeLimit() ? "1" : "0");
            rotaryMotorStop();
            fRotarySpeed = 0;
        }

        uint32_t currentMillis = millis();
        if (currentMillis - fRotaryThrottleUpdate > ROTARY_THROTTLE_LATENCY)
        {
            if (rotaryAllowed() && (fRotarySpeed != 0 || fRotaryThrottle != 0))
            {
                float throttle = 0;
                long encoder_ticks = getRotaryPosition();
                if (fRotarySpeed > fRotaryThrottle)
                {
                    // Ramp up; use deceleration scale when crossing through zero.
                    float scale = ROTARY_THROTTLE_ACCELERATION_SCALE;
                    if (fRotaryThrottle < 0)
                        scale = ROTARY_THROTTLE_DECELERATION_SCALE;
                    float val = max(abs(fRotarySpeed - fRotaryThrottle) / scale, 0.01f);
                    throttle = ((int)round(min(fRotaryThrottle + val, fRotarySpeed)*100))/100.0f;
                    if (sVerboseDebug)
                    {
                        DEBUG_PRINTLN(throttle);
                    }
                    rotaryMotorMove(throttle);
                    fRotaryThrottle = throttle;
                }
                else if (fRotarySpeed < fRotaryThrottle)
                {
                    // Ramp down.
                    float scale = ROTARY_THROTTLE_ACCELERATION_SCALE;
                    if (fRotaryThrottle > 0)
                        scale = ROTARY_THROTTLE_DECELERATION_SCALE;
                    float val = abs(fRotarySpeed - fRotaryThrottle) / scale;
                    throttle = ((int)floor(max(fRotaryThrottle - val, fRotarySpeed)*100))/100.0f;
                    if (sVerboseDebug)
                    {
                        DEBUG_PRINTLN(throttle);
                    }
                    rotaryMotorMove(throttle);
                    fRotaryThrottle = throttle;
                }
                // Stall check: stop the motor if no encoder ticks arrived in the last interval.
                if (millis() - fRotaryEncoderLastStatus > ENCODER_STATUS_RATE*2)
                {
                    fRotaryEncoderLastStatus = millis();
                    if (fRotaryEncoderLastTick == encoder_ticks)
                    {
                        DEBUG_PRINTLN("ROTARY NOT MOVING - ABORT");
                        rotaryMotorStop();
                    }
                    fRotaryEncoderLastTick = encoder_ticks;
                }
                fRotaryThrottleUpdate = currentMillis;
            }
        }
    #endif
    }

    static void rotaryMotorMove(float throttle, bool skipSafety = false)
    {
    #ifndef DISABLE_ROTARY
        if (sSettings.fDisableRotary)
            return;
        bool reverse = (throttle < 0);
        throttle = min(max(abs(throttle), 0.0f), 1.0f);
        if (throttle < 0.10)
            throttle = 0;

        // Ensure lifter is higher than minimum
        if (rotaryAllowed() || skipSafety)
        {
            if (fRotaryThrottle != throttle)
            {
                fRotaryEncoderLastStatus = millis();
                fRotaryEncoderLastTick = getRotaryPosition();
                fRotaryMoving = (throttle != 0);
                enableMotors();
                if (reverse)
                {
                    dualAnalogWrite(PIN_ROTARY_PWM1, 0, PIN_ROTARY_PWM2, fabs(throttle));
                    fRotaryThrottle = -throttle;
                }
                else
                {
                    dualAnalogWrite(PIN_ROTARY_PWM1, fabs(throttle), PIN_ROTARY_PWM2, 0);
                    fRotaryThrottle = throttle;
                }
            }
        }
        else
        {
            {
                static uint32_t sLastRotaryWarn;
                if (millis() - sLastRotaryWarn > 2000)
                {
                    sLastRotaryWarn = millis();
                    DEBUG_PRINT("ROTARY NOT ALLOWED: pos=");
                    DEBUG_PRINTLN(getLifterPosition());
                }
            }
        }
    #endif
    }

    // Circular geometry helpers — angles wrap 0°–359°, so shortest-path math is non-trivial.

    // True if p3 lies within the clockwise arc from p1 to p2.
    static bool withinArc(double p1, double p2, double p3)
    {
        return fmod(p2 - p1 + 2*360, 360) >= fmod(p3 - p1 + 2*360, 360);
    }

    // Wraps any angle into 0–359 (e.g. 370→10, -10→350).
    static int normalize(int degrees)
    {
        degrees = fmod(degrees, 360);
        if (degrees < 0)
            degrees += 360;
        return degrees;
    }

    // Signed shortest rotation from origin to target (positive = clockwise).
    static int shortestDistance(int origin, int target)
    {
        int result = 0;
        int diff = abs(origin - target) % 360;

        if (diff > 180)
        {
            result = (360 - diff);
            if (target > origin)
                result *= -1;
        }
        else
        {
            result = diff;
            if (origin > target)
                result *= -1;
        }
        return result;
    }

    // Steers rotary toward target; returns true once within fudge degrees.
    // Speed ramps up when far away if maxspeed > speed.
    // flipCount persists across loop iterations to implement direction-flip hysteresis.
    static bool moveScopeToTarget(int pos, int target, int fudge, float speed, float maxspeed, float &m, int &flipCount)
    {
    #ifndef DISABLE_ROTARY
        if (sSettings.fDisableRotary)
            return true;
        if (sVerboseDebug)
        {
            DEBUG_PRINT("MOVE raw="); DEBUG_PRINT(getRotaryPosition());
            DEBUG_PRINT(" pos="); DEBUG_PRINT(pos);
            DEBUG_PRINT(" target="); DEBUG_PRINT(target);
        }
        if (!withinArc(target - fudge, target + fudge, pos))
        {
            int dist = shortestDistance(pos, target);
            if (sVerboseDebug)
            {
                DEBUG_PRINT(" dist="); DEBUG_PRINT(dist);
            }
            if (maxspeed > speed && abs(dist) > fudge*2)
            {
                speed += (maxspeed - speed) * float(abs(dist)) / 180;
                speed = min(speed, maxspeed);  // clamp: dist > 180 could exceed maxspeed
            }
            if (sVerboseDebug)
            {
                DEBUG_PRINT(" speed1="); DEBUG_PRINT(speed);
            }
            if (speed < (ROTARY_MINIMUM_POWER/100.0))
                speed = (ROTARY_MINIMUM_POWER/100.0);
            if (sVerboseDebug)
            {
                DEBUG_PRINT(" speed2="); DEBUG_PRINT(speed);
            }
            float nm = (dist > 0) ? -speed : speed;
            if (m != 0 && ((m < 0 && nm > 0) || (m > 0 && nm < 0)))
            {
                // Direction flipped.  A single flip can be encoder noise (especially
                // in electrically busy event environments), so require two consecutive
                // flips before declaring overshoot and stopping.
                if (++flipCount >= 2)
                {
                    DEBUG_PRINTLN("DIRECTION CHANGE x2 — overshoot, stopping");
                    DEBUG_PRINTLN(m);
                    DEBUG_PRINTLN(nm);
                    return true;
                }
                DEBUG_PRINTLN("DIRECTION CHANGE x1 — waiting for confirmation");
            }
            else
            {
                flipCount = 0;  // consistent direction — reset the hysteresis counter
            }
            m = nm;
            if (sVerboseDebug)
            {
                DEBUG_PRINT(" m="); DEBUG_PRINTLN(m);
            }
            return false;
        }
        if (sVerboseDebug)
        {
            DEBUG_PRINTLN();
        }
    #endif
        return true;
    }

    // Backward-compatible overload: callers that don't need flip hysteresis
    // (e.g., single-shot moves) get a fresh counter per call.
    static bool moveScopeToTarget(int pos, int target, int fudge, float speed, float maxspeed, float &m)
    {
        int flipCount = 0;
        return moveScopeToTarget(pos, target, fudge, speed, maxspeed, m, flipCount);
    }

    static unsigned rotaryMotorCurrentPosition()
    {
        if (sRotaryCircleEncoderCount == 0)
            return 0;
        return normalize(getRotaryPosition() * (360.0 / sRotaryCircleEncoderCount));
    }

    static void rotaryMotorAbsolutePosition(int degrees, float speed = 0, float maxspeed = 0)
    {
    #ifndef DISABLE_ROTARY
        if (sSettings.fDisableRotary)
            return;
        float m = 0;
        int flipCount = 0;   // persistent across loop iterations for proper hysteresis
        if (speed == 0)
            speed = (ROTARY_MINIMUM_POWER/100.0);
        if (maxspeed == 0)
            maxspeed = speed;
        // degrees = -degrees;
        RotaryStatus rotaryStatus;
        while (!moveScopeToTarget(rotaryMotorCurrentPosition(), normalize(degrees), ROTARY_FUDGE_POSITION, speed, maxspeed, m, flipCount))
        {
            if (sWebAbort) break;
            rotaryMotorMove(m);
            if (!rotaryStatus.isMoving())
            {
                DEBUG_PRINTLN("ABORT");
                break;
            }
        }
        rotaryMotorStop();
    #endif
    }

    static void rotaryMotorRelativePosition(int relativeDegrees)
    {
        rotaryMotorAbsolutePosition(rotaryMotorCurrentPosition() + relativeDegrees);
    }

    // Return head to 0°: coarse move via encoder, then fine-align to the limit switch.
    // Checks sWebAbort between each pass so ESTOP halts rotation immediately.
    static void rotateHome()
    {
    #ifndef DISABLE_ROTARY
        if (sSettings.fDisableRotary)
            return;
        Serial.print("ROTATE HOME: curDeg=");
        Serial.print(rotaryMotorCurrentPosition());
        Serial.print(" encTicks=");
        Serial.print(getRotaryPosition());
        Serial.print(" homeSwitch=");
        Serial.print(rotaryHomeLimit() ? "1" : "0");
        Serial.print(" circleEnc=");
        Serial.println(sRotaryCircleEncoderCount);
        // First pass: encoder-based move to 0°. This gets us very close to home
        // and is reliable once sRotaryCircleEncoderCount is known.
        rotaryMotorAbsolutePosition(0, 0.5);
        if (sWebAbort) { rotaryMotorStop(); return; }
        Serial.print("  pass1 done: deg=");
        Serial.print(rotaryMotorCurrentPosition());
        Serial.print(" home=");
        Serial.println(rotaryHomeLimit() ? "1" : "0");
        // Second pass: creep to limit switch for a precise encoder re-zero.
        // This corrects any small drift that accumulated over multiple rotations.
        if (shortestDistance(rotaryMotorCurrentPosition(), 0) > 0)
            rotateLeftHome();
        else
            rotateRightHome();
        if (sWebAbort) { rotaryMotorStop(); return; }
        Serial.print("  pass2 done: deg=");
        Serial.print(rotaryMotorCurrentPosition());
        Serial.print(" home=");
        Serial.println(rotaryHomeLimit() ? "1" : "0");

        // Third pass: back off the switch and re-approach at minimum speed for
        // a repeatable final position.  The first creep may overshoot by a
        // variable amount; this second touch at lower speed tightens the spread.
        if (rotaryHomeLimit() && sRotaryCircleEncoderCount >= 1000)
        {
            // Back off: move away from the switch by a small amount
            float backoffSpeed = (ROTARY_MINIMUM_POWER/100.0) + 0.05;
            // Determine which direction moves away from the switch
            // (opposite of whichever direction brought us here)
            if (shortestDistance(rotaryMotorCurrentPosition(), 0) >= 0)
                backoffSpeed = -backoffSpeed;
            uint32_t backoffEnd = millis() + 300;
            while (millis() < backoffEnd && rotaryHomeLimit() && !sWebAbort)
            {
                rotaryMotorMove(backoffSpeed);
                delay(3);
                rotaryMotorStop();
                delay(1);
            }
            rotaryMotorStop();
            if (sWebAbort) return;
            delay(100);
            // Re-approach at minimum creep speed
            if (!rotaryHomeLimit())
            {
                rotateUntilHome(-backoffSpeed < 0 ? -0.01 : 0.01);
                if (rotaryHomeLimit())
                {
                    resetRotaryPosition();
                    DEBUG_PRINTLN("HOME (precision re-approach)");
                }
            }
        }

        // If the limit switch never triggered (unreliable hardware), accept the
        // encoder position as home rather than leaving the position unknown.
        if (!rotaryHomeLimit() && sRotaryCircleEncoderCount >= 1000)
        {
            DEBUG_PRINTLN("HOME (encoder only - limit switch did not trigger)");
            resetRotaryPosition();
        }
    #endif
    }

    // Inch the motor until the home limit switch fires.
    // Uses active braking on limit contact for minimal overshoot.
    static void rotateUntilHome(float speed)
    {
    #ifndef DISABLE_ROTARY
        if (sSettings.fDisableRotary)
            return;
        bool neg = (speed < 0);
        // Add the caller's speed above the minimum power floor.
        // Previously 0.1*abs(speed) squashed the range so 0.01 and 0.1
        // produced nearly identical speeds (0.401 vs 0.41).  Now 0.01
        // gives 0.41 and 0.1 gives 0.50 — a meaningful difference for
        // precision re-approach vs normal creep.
        speed = (ROTARY_MINIMUM_POWER/100.0) + abs(speed);
        if (neg)
            speed = -speed;
        DEBUG_PRINTLN(speed);
        RotaryStatus rotaryStatus;
        encoder_rotary_stop_limit = true;
        uint32_t homeTimeout = millis() + 10000;  // give up after 10 seconds
        while (!rotaryHomeLimit() && millis() < homeTimeout && !sWebAbort)
        {
            rotaryMotorMove(speed);
            uint32_t startMillis = millis();
            while (millis() < startMillis + 3)
            {
                if (rotaryHomeLimit())
                    goto home;
            }
            rotaryMotorStop();
            startMillis = millis();
            while (millis() < startMillis + 1 && !rotaryHomeLimit())
                ;
            if (!rotaryStatus.isMoving())
            {
                DEBUG_PRINTLN("ABORT");
                break;
            }
        }
    home:
        // Active brake to minimize overshoot past the limit switch
        rotaryMotorBrake();
        delay(50);
        rotaryMotorStop();
        encoder_rotary_stop_limit = false;
    #endif
    }

    // Returns true if the rotary home limit switch was found and position was zeroed.
    static bool rotateLeftHome()
    {
    #ifndef DISABLE_ROTARY
        // Ensure lifter is higher than minimum
        if (rotaryAllowed())
        {
            if (!rotaryHomeLimit() && !sWebAbort)
            {
                rotateUntilHome(-0.1);
            }
            if (sWebAbort) { rotaryMotorStop(); return false; }
            delay(200);
            if (!rotaryHomeLimit() && !sWebAbort)
            {
                rotateUntilHome(0.1);
            }
            if (sWebAbort) { rotaryMotorStop(); return false; }
            if (rotaryHomeLimit())
            {
                // DEBUG_PRINTLN("FOUND HOME");
                resetRotaryPosition();
                return true;
            }
        }
        return false;
    #else
        return true;
    #endif
    }

    // Returns true if the rotary home limit switch was found and position was zeroed.
    static bool rotateRightHome()
    {
    #ifndef DISABLE_ROTARY
        // Ensure lifter is higher than minimum
        if (rotaryAllowed())
        {
            if (!rotaryHomeLimit() && !sWebAbort)
            {
                rotateUntilHome(0.1);
            }
            if (sWebAbort) { rotaryMotorStop(); return false; }
            delay(200);
            // Try up to two passes in the reverse direction if the first attempt missed.
            for (int attempt = 0; attempt < 2 && !rotaryHomeLimit() && !sWebAbort; attempt++)
            {
                rotateUntilHome(-0.1);
            }
            if (sWebAbort) { rotaryMotorStop(); return false; }
            if (rotaryHomeLimit())
            {
                DEBUG_PRINTLN("FOUND HOME");
                resetRotaryPosition();
                return true;
            }
            else
            {
                DEBUG_PRINTLN("NOT AT HOME");
            }
        }
        return false;
    #else
        return true;
    #endif
    }

    static void rotaryMotorStop()
    {
        fRotarySpeed = 0;
        enableMotors();
        dualAnalogWrite(PIN_ROTARY_PWM1, 0, PIN_ROTARY_PWM2, 0);
        fRotaryThrottle = 0;
        fRotaryMoving = false;
    }

    // Active brake: both PWMs HIGH shorts motor windings through the H-bridge,
    // converting momentum into heat. Much faster stop than coast (both PWMs LOW).
    static void rotaryMotorBrake()
    {
        fRotarySpeed = 0;
        enableMotors();
        dualAnalogWrite(PIN_ROTARY_PWM1, 1.0, PIN_ROTARY_PWM2, 1.0);
        fRotaryThrottle = 0;
        fRotaryMoving = false;
    }

    static bool isRotarySpinning()
    {
    #ifdef DISABLE_ROTARY
        // Rotary is not moving
        return false;
    #else
        return !sSettings.fDisableRotary && fRotaryMoving;
    #endif
    }

    static bool isRotaryAtRest()
    {
    #ifdef DISABLE_ROTARY
        return true;
    #else
        return sSettings.fDisableRotary || ((rotaryHomeLimit() || rotaryMotorCurrentPosition() == 0) && !fRotaryMoving);
    #endif
    }

    // Light show getter/setter — mode numbers match the kLightKit_* enum above.
    static int getLightShow()
    {
        return fLightShow;
    }

    static void setLightShow(int show)
    {
        fLightShow = show;
        // Light kit pins are active-low: the bit is inverted before writing.
        sPinManager.digitalWrite(PIN_LIGHTKIT_A, !((show>>2)&1));
        sPinManager.digitalWrite(PIN_LIGHTKIT_B, !((show>>1)&1));
        sPinManager.digitalWrite(PIN_LIGHTKIT_C, !((show>>0)&1));
    }

    static void IRAM_ATTR measureLifterEncoder();
    static void IRAM_ATTR measureRotaryEncoder();

    virtual void setup() override
    {
        //////////////////////////
        // ENCODER PINS
        //////////////////////////

        sPinManager.pinMode(PIN_LIFTER_ENCODER_A, INPUT);
        sPinManager.pinMode(PIN_LIFTER_ENCODER_B, INPUT);

        sPinManager.pinMode(PIN_ROTARY_ENCODER_A, INPUT);
        sPinManager.pinMode(PIN_ROTARY_ENCODER_B, INPUT);

        attachInterrupt(
            digitalPinToInterrupt(PIN_LIFTER_ENCODER_A),
                measureLifterEncoder, CHANGE);
        attachInterrupt(
            digitalPinToInterrupt(PIN_ROTARY_ENCODER_A),
                measureRotaryEncoder, CHANGE);

        //////////////////////////
        // MOTOR DRIVER PINS
        //////////////////////////

        // LIFTER
        sPinManager.pinMode(PIN_LIFTER_PWM1, OUTPUT);
        sPinManager.pinMode(PIN_LIFTER_PWM2, OUTPUT);
        sPinManager.pinMode(PIN_LIFTER_DIAG, INPUT_PULLUP);

        // ROTARY
        sPinManager.pinMode(PIN_ROTARY_PWM1, OUTPUT);
        sPinManager.pinMode(PIN_ROTARY_PWM2, OUTPUT);
        sPinManager.pinMode(PIN_ROTARY_DIAG, INPUT_PULLUP);

        sPinManager.pinMode(PIN_ROTARY_LIMIT, INPUT_PULLUP);
        sPinManager.pinMode(PIN_LIFTER_TOPLIMIT, INPUT_PULLUP);
        sPinManager.pinMode(PIN_LIFTER_BOTLIMIT, INPUT_PULLUP);

        //////////////////////////
        // MOTOR ENABLE PINS
        //////////////////////////

        sPinManager.pinMode(PIN_MOTOR_EN, OUTPUT);
        sPinManager.pinMode(PIN_MOTOR_ENB, OUTPUT);

        //////////////////////////
        // LIGHT KIT PINS
        //////////////////////////

        sPinManager.pinMode(PIN_LIGHTKIT_A, OUTPUT);
        sPinManager.pinMode(PIN_LIGHTKIT_B, OUTPUT);
        sPinManager.pinMode(PIN_LIGHTKIT_C, OUTPUT);

    #ifndef USE_SDCARD
        //////////////////////////////
        // ANALOG SEQUENCE SELECT PINS
        //////////////////////////////

        sPinManager.pinMode(PIN_INPUT_A, INPUT_PULLUP);
        sPinManager.pinMode(PIN_INPUT_B, INPUT_PULLUP);
        sPinManager.pinMode(PIN_INPUT_C, INPUT_PULLUP);
    #endif

        sPinManager.begin();

        setLightShow(kLightKit_Off);

        // analogWriteFrequencyResolution(PIN_LIFTER_PWM1, 30000, 8);
        // analogWriteFrequencyResolution(PIN_LIFTER_PWM2, 30000, 8);
        // analogWriteFrequencyResolution(PIN_ROTARY_PWM1, 30000, 8);
        // analogWriteFrequencyResolution(PIN_ROTARY_PWM2, 30000, 8);
    }

    ///////////////////////////////////

    static bool isCalibrated()
    {
        return sSettings.fUpLimitsCalibrated && sSettings.fDownLimitsCalibrated;
    }

    static void seekToBottom(bool usePID = true)
    {
        if (!isRotaryAtRest())
        {
            DEBUG_PRINTLN("ABORT: ROTARY NOT HOME");
            DEBUG_PRINTLN(rotaryMotorCurrentPosition());
            return;
        }
        if (!usePID)
        {
            DEBUG_PRINTLN("SEEK TO BOTTOM");

            // seek down
            float mpower = LIFTER_SEEKBOTTTOM_POWER / 100.0;
            bool botLimit;
            // Breakaway boost before pulsed drive
            breakawayBoost(-mpower);
            // Use a 2-second position-change timeout instead of LifterStatus.
            // LifterStatus requires 20 ticks/200ms which the Greg lifter can't
            // sustain at SEEKBOTTOM_POWER (40%), causing false stall aborts that
            // skip the encoder re-zero and corrupt position tracking.
            long prevPos = getLifterPosition();
            uint32_t posChangeMs = millis();
            for (;;)
            {
                botLimit = lifterBottomLimit();
                if (botLimit || serialAbort())
                    break;
                lifterMotorMove(-mpower);
                delay(3);
                lifterMotorStop();
                delay(1);

                long curPos = getLifterPosition();
                if (curPos != prevPos)
                {
                    prevPos = curPos;
                    posChangeMs = millis();
                }
                if (millis() - posChangeMs > 2000)
                {
                    Serial.print("SEEK TO BOTTOM ABORT: stall at pos=");
                    Serial.println(curPos);
                    break;
                }
            }
            lifterMotorStop();
            delay(500);
            if (botLimit)
            {
                DEBUG_PRINTLN("BOTTOM LIMIT REACHED");
                if (sSettings.fLifterDistance == 0)
                {
                    sSettings.fLifterDistance = abs(getLifterPosition()) + 10;
                    DEBUG_PRINT("LIFTER DISTANCE: ");
                    DEBUG_PRINTLN(sSettings.fLifterDistance);
                }
                resetLifterPosition();
                delay(100);
                resetLifterPosition();
            }
        }
        else
        {
            float fDIn = 0;
            float fDOut = 0;
            float fDSet = 0;
            PID<float> distance(fDIn, fDOut, fDSet, 1.0, 0.01, 0.01);
            // TargetSteering steering(0);
            distance.setAutomatic(true);
            distance.setSampleTime(1);
            distance.setOutputLimits(-350, 350);

            DEBUG_PRINTLN("SEEK TO BOTTOM PID");
        #ifdef USE_DEBUG
            uint32_t start = millis();
        #endif
            bool botLimit;
            LifterStatus lifterStatus;
            for (;;)
            {
                long encoder_ticks = getLifterPosition();
                botLimit = lifterBottomLimit();
                if (botLimit || serialAbort())
                    break;
                fDSet = encoder_ticks;
                distance.process();
                float throttle = (fDOut / 350.0);
                if (throttle > 1.0)
                    break;
                lifterMotorMove(-throttle * 0.7);

                if (!lifterStatus.isMoving())
                {
                    DEBUG_PRINT("LIFTER ABORTED AT "); DEBUG_PRINTLN(encoder_ticks);
                    break;
                }
            }
            lifterMotorStop();
        #ifdef USE_DEBUG
            uint32_t stop = millis();
        #endif
            if (botLimit)
            {
                DEBUG_PRINTLN("BOTTOM LIMIT REACHED");
                DEBUG_PRINT("DISTANCE: ");
                DEBUG_PRINTLN(getLifterPosition());
                DEBUG_PRINT("TIME: ");
                DEBUG_PRINTLN(stop - start);
                // resetLifterPosition();
            }
        }
    }

#undef INTERNAL_LIFTER_DEBUG
#ifdef INTERNAL_LIFTER_DEBUG
    static bool lifterMotorTest(float speed = 0.5, bool usePID = false)
    {
        if (!usePID)
        {
            float mpower = sSettings.fMinimumPower/100.0 + 0.05 + 0.1 * speed;
            LifterStatus lifterStatus;
            uint32_t startPos = getLifterPosition();
            printf("startPos: %u\n", startPos);
            for (;;)
            {
                if (Serial.available())
                    break;
                lifterMotorMove(mpower);
                delay(3);
                lifterMotorStop();
                delay(1);
                if (!lifterStatus.isMoving())
                {
                    Serial.println("ABORT");
                    break;
                }
            }
            uint32_t stopPos = getLifterPosition();
            printf("stopPos: %u\n", stopPos);
        }
        Serial.read();
        return false;
    }

    static bool rotaryMotorTest(float speed = 0.5, bool usePID = false)
    {
        // Find home position
        if (!usePID)
        {
            RotaryStatus rotaryStatus;
            long startPos = getRotaryPosition();
            printf("startPos: %ld\n", startPos);

            rotaryMotorMove(speed, true);
            delay(200);

            while (!rotaryHomeLimit())
            {
                // long rotaryPos = getRotaryPosition();
                rotaryMotorMove(speed, true);
                // uint32_t startMillis = millis();
                // while (startMillis + 3 < millis())
                // {
                //     if (rotaryHomeLimit())
                //         goto stop;
                // }
                // rotaryMotorStop();
                // startMillis = millis();
                // while (startMillis + 1 < millis() && !rotaryHomeLimit())
                //     ;
                if (!rotaryStatus.isMoving())
                {
                    DEBUG_PRINTLN("ABORT");
                    break;
                }
            }
        // stop:
            sRotaryCircleEncoderCount = abs(getRotaryPosition());
            long rotaryHomePos = getRotaryPosition();
            // Active braking
            // rotaryMotorMove((speed < 0) ? 1 : -1, true);
            // delay((speed < 0) ? 3 : 10);
            rotaryMotorStop();
            delay(1000);

            long stopPos = getRotaryPosition();
            printf("home: %ld\n", rotaryHomePos);
            printf("stopPos: %ld\n", stopPos);
            printf("diff: %ld\n", abs(stopPos - rotaryHomePos));
            printf("limit: %d\n", rotaryHomeLimit());
        }
        Serial.read();
        return false;
    }
#endif
    // Startup safety maneuver: lifts to top, homes rotary and measures one full revolution,
    // then lowers back down. Most commands are blocked until this succeeds.
    static bool safetyManeuver()
    {
    #ifdef INTERNAL_LIFTER_DEBUG
        lifterMotorStop();
        rotaryMotorStop();
        for (int i = 0; i < LEDC_CHANNELS; i++)
        {
            printf("channel[%d]: %d\n", i, channels_resolution[i]);
        }

        if (!analogWriteFrequencyResolution(PIN_ROTARY_PWM1, 21762, 10))
            printf("NOT SUPPORTED\n");
        analogWriteFrequencyResolution(PIN_ROTARY_PWM2, 21762, 10);
        rotaryMotorMove(-(ROTARY_MINIMUM_POWER/100.0), true);
        delay(2000);
        analogWriteFrequencyResolution(PIN_ROTARY_PWM1, 1000, 8);
        analogWriteFrequencyResolution(PIN_ROTARY_PWM2, 1000, 8);
        rotaryMotorMove(-(ROTARY_MINIMUM_POWER/100.0), true);
        delay(2000);


            // rotaryMotorTest((ROTARY_MINIMUM_POWER/100.0));
        // }
        rotaryMotorStop();
        return false;
    #endif
        Serial.println("SAFETY");
        sSafetyManeuverFailed = false;
        // updateDistance=false: safety maneuver runs with dome on where EMI
        // corrupts encoder counts. Only #PSC calibration (dome off) should
        // update fLifterDistance.
        if (seekToTop(0.8, false, false))
        {
        #ifndef DISABLE_SAFETY_MANEUVER
            if (!sSettings.fDisableRotary)
            {
                setLightShow(kLightKit_Dagobah);

                if (sRotaryCircleEncoderCount >= 1000)
                {
                    // We have a cached encoder count from a previous run.
                    // Just find home — no need to measure a full revolution again.
                    Serial.print("ROTARY ENCODER COUNT (cached) = ");
                    Serial.println(sRotaryCircleEncoderCount);
                    bool homed = rotateLeftHome();
                    if (!homed)
                        homed = rotateRightHome();
                    if (!homed)
                    {
                        Serial.println("ABORT: ROTARY HOME NOT FOUND");
                        sSafetyManeuverFailed = true;
                        return false;
                    }
                }
                else
                {
                    // No cached count — do the full measurement: find home, then
                    // spin exactly one revolution and count the encoder ticks.
                    int attempt = 0;
                    while (sRotaryCircleEncoderCount == 0 && attempt++ < 5)
                    {
                        // Ensure rotary in home position
                        rotaryMotorMove(-(ROTARY_MINIMUM_POWER/100.0));
                        delay(200);

                        rotateLeftHome();
                        delay(100);

                        if (!rotaryHomeLimit())
                        {
                            Serial.println("NOT HOME TRY SPIN AROUND");
                            bool rotaryWasHome = true;
                            RotaryStatus rotaryStatus;
                            rotaryMotorMove(-(ROTARY_MINIMUM_POWER/100.0));
                            for (;;)
                            {
                                if (rotaryHomeLimit())
                                {
                                    if (!rotaryWasHome)
                                        break;
                                }
                                else if (rotaryWasHome)
                                {
                                    rotaryWasHome = false;
                                }
                                if (!rotaryStatus.isMoving())
                                    break;
                            }
                            rotaryMotorStop();
                        }
                        if (rotaryHomeLimit())
                        {
                            Serial.println("FIND ENCODER LENGTH");
                            resetRotaryPosition();
                            encoder_rotary_last_status = millis();
                            bool rotaryWasHome = true;
                            rotaryMotorMove(-((ROTARY_MINIMUM_POWER+5)/100.0));
                            delay(100);
                            RotaryStatus rotaryStatus;
                            encoder_rotary_stop_limit = true;
                            for (;;)
                            {
                                if (rotaryHomeLimit())
                                {
                                    if (!rotaryWasHome)
                                        break;
                                }
                                else if (rotaryWasHome)
                                {
                                    rotaryWasHome = false;
                                }
                                if (!rotaryStatus.isMoving())
                                    break;
                            }
                            encoder_rotary_stop_limit = false;
                            rotaryMotorStop();
                            delay(100);
                            sRotaryCircleEncoderCount = abs(getRotaryPosition());
                            Serial.print("ROTARY ENCODER COUNT = ");
                            Serial.println(sRotaryCircleEncoderCount);
                            if (sRotaryCircleEncoderCount < 1000)
                            {
                                sRotaryCircleEncoderCount = 0;  // bad reading, retry
                            }
                            else
                            {
                                // Good reading — save it so future reboots skip this step.
                                sLifterParameters.fRotaryEncoderCount = sRotaryCircleEncoderCount;
                                sLifterParameters.save();
                            }
                        }
                        else
                        {
                            DEBUG_PRINTLN("ROTARY NOT HOME TRY AGAIN");
                        }
                    }
                    // If we still have no valid count, the limit switch is too
                    // unreliable to establish a reference. Abort the safety maneuver.
                    if (!sRotaryCircleEncoderCount)
                    {
                        return false;
                    }
                }
            }
        #endif

            setLightShow(kLightKit_Off);
            if (!isRotaryAtRest())
            {
                Serial.println("ABORT: ROTARY NOT HOME");
                Serial.println(rotaryMotorCurrentPosition());
                return false;
            }
            // seekToTop already set fLifterDistance from the UP-direction encoder count.
            // Do NOT reset it to 0 here — seekToBottom's fLifterDistance==0 guard would
            // then overwrite it with the DOWN-direction count, which is smaller due to
            // EMI overcounting going UP vs DOWN.
            resetLifterPosition();
        #ifdef INTERNAL_LIFTER_DEBUG
            return false;
        #else
            seekToBottom(false);
            delay(10);  // debounce: let limit switch settle before re-reading
            sSafetyManeuver = lifterBottomLimit();
            if (!sSafetyManeuver)
            {
                Serial.print("SAFETY: bottom limit FALSE after seek (switch bounce?) pos=");
                Serial.println(getLifterPosition());
                sSafetyManeuverFailed = true;
            }
            return sSafetyManeuver;
        #endif
        }
        else
        {
            DEBUG_PRINTLN("ABORT: FAILED SEEK TO TOP");
            sSafetyManeuverFailed = true;
            return false;
        }
    }

    // Runs the safety maneuver if it hasn't succeeded yet. Disables motors on failure.
    static bool ensureSafetyManeuver()
    {
        if (!sSafetyManeuver)
        {
            Serial.print("ENSURE SAFETY: running maneuver (safetyFail=");
            Serial.print(sSafetyManeuverFailed ? "1" : "0");
            Serial.print(" pos=");
            Serial.print(getLifterPosition());
            Serial.println(")");
            if (!safetyManeuver())
            {
                Serial.println("ENSURE SAFETY: FAILED — disabling motors");
                disableMotors();
            }
            else
            {
                Serial.println("ENSURE SAFETY: OK");
            }
        }
        return sSafetyManeuver;
    }

    // Calibration routine: lifts and lowers at each 5% speed step to measure PID braking limits.
    // Trigger via #PSC or the web UI. Results are saved to flash.
    static bool calibrate()
    {
        bool success = true;
        int retryCount = 0;
        sCalibrating = true;
        sSettings.fMinimumPower = max(int(sSettings.fMinimumPower), LIFTER_MINIMUM_POWER);
        if (!safetyManeuver())
        {
            Serial.println("ABORT: FAILED SAFETY MANEUVER");
            sCalibrating = false;
            return false;
        }
    retry:
        if (++retryCount > 3)
        {
            Serial.println("ABORT: ROTARY NOT AT REST AFTER 3 RETRIES");
            sSafetyManeuverFailed = true;
            sCalibrating = false;
            return false;
        }
        if (!isRotaryAtRest())
        {
            if (!safetyManeuver())
            {
                Serial.println("ABORT: FAILED SAFETY MANEUVER");
                sCalibrating = false;
                return false;
            }
        }
        // Clear all limits
        memset(sSettings.fUpLimits, '\0', sizeof(sSettings.fUpLimits));
        memset(sSettings.fDownLimits, '\0', sizeof(sSettings.fDownLimits));
        sSettings.fUpLimitsCalibrated = false;
        sSettings.fDownLimitsCalibrated = false;
        sSettings.fMinimumPower = 0;
        // NOTE: do NOT reset fLifterDistance here. safetyManeuver() already resets it
        // to 0 internally (before its own seekToBottom) so it always remeasures fresh.
        // Resetting it again here would erase the correct measurement and corrupt
        // targetDistance for the calibration sweep.

        long homePosition = getLifterPosition();
        int topSpeed = LIFTER_MINIMUM_POWER;
        for (topSpeed = LIFTER_MINIMUM_POWER; topSpeed <= 100; topSpeed += 5)
        {
            LifterStatus lifterStatus;
            lifterMotorMove(topSpeed / 100.0);
            delay(ENCODER_STATUS_RATE*2);
            lifterMotorStop();
            // Stop as soon as the top limit fires — do not strain the motor against it.
            // Driving into the top limit generates sustained stall current that corrupts
            // the encoder via EMI, making targetDistance wrong for the whole sweep.
            if (lifterTopLimit())
                break;
            if (lifterStatus.isMoving())
                break;
        }
        seekToBottom(false);

        long targetDistance = sSettings.getLifterDistance();

        Serial.println("SEEK TO TOP");
        for (; sCalibrating && topSpeed <= 100; topSpeed += 5)
        {
            unsigned tries = 0;
            if (!isRotaryAtRest())
            {
                DEBUG_PRINTLN("ABORT: ROTARY NOT AT HOME POSITION");
                DEBUG_PRINTLN(rotaryMotorCurrentPosition());
                goto retry;
            }
            long outputLimit = max(long(topSpeed * OUTPUT_LIMIT_PRESCALE), 10L);
            Serial.print("SPEED: "); Serial.println(topSpeed);
            while (outputLimit >= 0 && tries < CALIBRATION_MAX_TRIES)
            {
                tries++;
                long maxEncoderVal = 0;
                seekToBottom(false);
                delay(1000);

                TargetSteering steering(targetDistance);
                if (outputLimit > 0)
                    steering.setDistanceOutputLimits(outputLimit);
                steering.setSampleTime(1);
                bool topLimit;
                long start_ticks = getLifterPosition();
                DEBUG_PRINT("OUTPUT LIMIT: "); DEBUG_PRINTLN(outputLimit);
                long calPrevPos = getLifterPosition();
                uint32_t calPosChangeMs = millis();
                for (;;)
                {
                    long encoder_ticks = getLifterPosition();
                    maxEncoderVal = max(maxEncoderVal, encoder_ticks);
                    topLimit = lifterTopLimit();
                    // Exit if top limit hit, target reached (use >= to handle encoder overshoot),
                    // or abort requested.
                    // NOTE: we intentionally do NOT guard against topLimit based on encoder position.
                    // The encoder can undercount vs. the measured fLifterDistance due to motor EMI
                    // at different speeds, so a topLimit at a "low" encoder value may be real.
                    // The 3ms debounce in lifterTopLimit() is the primary anti-EMI protection.
                    if (topLimit || encoder_ticks >= targetDistance || serialAbort())
                        break;
                    if (encoder_ticks != calPrevPos) { calPrevPos = encoder_ticks; calPosChangeMs = millis(); }
                    if (millis() - calPosChangeMs > 2000)
                    {
                        Serial.println("ABORT");
                        break;
                    }
                    float throttle = 1.0;
                    if (outputLimit > 0)
                    {
                        steering.setCurrentDistance(encoder_ticks);
                        throttle = steering.getThrottle();
                    }
                    lifterMotorMove(throttle * (topSpeed / 100.0));
                }
                lifterMotorStop();
                if (start_ticks == getLifterPosition())
                {
                    // Motor did not move at all.
                    if (outputLimit == 0)
                    {
                        // Full throttle and still no movement — motor truly underpowered.
                        Serial.println(" SPEED TOO LOW");
                        break;
                    }
                    // PID outputLimit is too large relative to travel distance,
                    // throttling power below minimum before the motor can overcome inertia.
                    // Drop to 0 (full throttle) and retry.
                    Serial.print(" DID NOT MOVE (outputLimit="); Serial.print(outputLimit);
                    Serial.println(") - RETRYING AT FULL THROTTLE");
                    outputLimit = 0;
                    delay(1000);
                    continue;
                }
                // Success: top limit switch fired OR encoder reached/passed the target distance.
                if (topLimit || maxEncoderVal >= targetDistance - 5)
                {
                    size_t index = topSpeed/5;
                    sSettings.fUpLimits[index].valid = true;
                    sSettings.fUpLimits[index].outputLimit = outputLimit;
                    if (sSettings.fMinimumPower == 0)
                        sSettings.fMinimumPower = topSpeed;
                    Serial.print("UP LIMIT RECORDED speed="); Serial.print(topSpeed);
                    Serial.print(" outputLimit="); Serial.println(outputLimit);
                    break;
                }
                else
                {
                    long encoder_ticks = getLifterPosition();
                    if (outputLimit == 0)
                    {
                        Serial.print(" MAX ENCODER="); Serial.print(maxEncoderVal);
                        Serial.println(" LIMIT NOT REACHED AT FULL THROTTLE - GIVING UP");
                        break;
                    }
                    Serial.print(encoder_ticks);
                    Serial.print(" LIMIT NOT REACHED - RETRYING ");
                    Serial.print(targetDistance - encoder_ticks);
                    // Reduce by the larger of: the distance-based amount OR half the
                    // current value. This ensures we converge in at most ~4 attempts
                    // even when the motor stalls just short of the target.
                    long reduction = max((targetDistance - encoder_ticks) * 2, outputLimit / 2);
                    outputLimit = max(outputLimit - reduction, 0L);
                    Serial.print(" NEW LIMIT : "); Serial.println(outputLimit);
                }
                delay(1000);
                if (serialAbort())
                    break;
            }
            // Pause between speed levels to let the motor cool down.
            lifterMotorStop();
            delay(CALIBRATION_COOLDOWN_MS);
            if (!sCalibrating || serialAbort())
            {
                Serial.println("SERIAL ABORT");
                success = false;
                break;
            }
        }
        // Abort calibration
        if (!success)
        {
            Serial.println("CALIBRATION ABORTED");
            sCalibrating = false;
            return false;
        }

        sSettings.fUpLimitsCalibrated = true;
        Serial.println("SEEK TO BOTTOM");
        DEBUG_PRINT("TOP LIMIT SWITCH: ");
        DEBUG_PRINTLN(lifterTopLimit());
        DEBUG_PRINT("BOTTOM LIMIT SWITCH: ");
        DEBUG_PRINTLN(lifterBottomLimit());
        seekToBottom(false);

        // After up-calibration stall cycles the motor driver may be in thermal
        // shutdown (TSD).  Wait up to 15 s for the fault to clear before
        // starting the down phase.  Toggle enableMotors() periodically to
        // re-assert the EN pin and clear any latched OCP state.
        {
            Serial.print("FAULT CHECK: "); Serial.println(lifterMotorFault() ? "FAULTED - WAITING" : "OK");
            uint32_t faultWaitStart = millis();
            while (lifterMotorFault() && (millis() - faultWaitStart) < 15000)
            {
                enableMotors();
                delay(500);
            }
            if (lifterMotorFault())
            {
                Serial.println("MOTOR FAULT NOT CLEARED - ABORTING DOWN CALIBRATION");
                sCalibrating = false;
                return false;
            }
            Serial.println("FAULT CLEARED - STARTING DOWN CALIBRATION");
        }
        delay(500);

        for (topSpeed = sSettings.fMinimumPower; sCalibrating && topSpeed <= 100; topSpeed += 5)
        {
            float limit;
            if (!getUpOutputLimit(topSpeed/100.0f, limit))
                continue;
            long outputLimit = max(long(limit*1.5), 160L);
            unsigned downTries = 0;
            while (success && outputLimit >= 0 && downTries < CALIBRATION_MAX_TRIES)
            {
                downTries++;
                long minEncoderVal = 0x7FFFFFFFL;
                if (!seekToTop(0.8, false))
                {
                    Serial.println("SEEK TO TOP FAILED - ABORT");
                    topSpeed = 200;
                    success = false;
                    break;
                }

                TargetSteering steering(homePosition);
                if (outputLimit > 0)
                    steering.setDistanceOutputLimits(outputLimit);
                steering.setSampleTime(1);
                bool botLimit;
                long start_ticks = getLifterPosition();
                long calPrevPos = getLifterPosition();
                uint32_t calPosChangeMs = millis();
                for (;;)
                {
                    long encoder_ticks = getLifterPosition();
                    minEncoderVal = min(minEncoderVal, encoder_ticks);
                    botLimit = lifterBottomLimit();
                    if (botLimit || serialAbort())
                        break;
                    if (encoder_ticks != calPrevPos) { calPrevPos = encoder_ticks; calPosChangeMs = millis(); }
                    if (millis() - calPosChangeMs > 2000)
                    {
                        Serial.println("ABORT");
                        break;
                    }
                    float throttle = -1.0f;
                    if (outputLimit > 0)
                    {
                        steering.setCurrentDistance(encoder_ticks);
                        throttle = steering.getThrottle();
                    }
                    lifterMotorMove(throttle * (topSpeed / 100.0));
                }
                lifterMotorStop();
                if (start_ticks == getLifterPosition())
                {
                    if (outputLimit == 0)
                    {
                        DEBUG_PRINTLN(" SPEED TOO LOW");
                        success = false;
                        break;
                    }
                    DEBUG_PRINT(" DID NOT MOVE DOWN (outputLimit="); DEBUG_PRINT(outputLimit);
                    DEBUG_PRINTLN(") - RETRYING AT FULL THROTTLE");
                    outputLimit = 0;
                    delay(1000);
                    continue;
                }
                if (botLimit)
                {
                    // Reset encoder to 0 at the physical bottom so the next seekToTop
                    // starts from a known reference. Without this, each iteration's
                    // UP count accumulates (337 → 573 → 772 → ...) because resetLifterPositionTop
                    // sets the encoder to fLifterDistance at the top, but going DOWN doesn't
                    // return it to 0 — the encoder undercounts going DOWN vs UP due to EMI.
                    resetLifterPosition();
                    size_t index = topSpeed/5;
                    sSettings.fDownLimits[index].valid = true;
                    sSettings.fDownLimits[index].outputLimit = outputLimit;
                    Serial.print("DOWN LIMIT RECORDED speed="); Serial.print(topSpeed);
                    Serial.print(" outputLimit="); Serial.println(outputLimit);
                    break;
                }
                else
                {
                    long encoder_ticks = getLifterPosition();
                    if (outputLimit == 0)
                    {
                        DEBUG_PRINTLN(" DOWN LIMIT NOT REACHED AT FULL THROTTLE - GIVING UP");
                        break;
                    }
                    // Same convergence fix as Phase 1: reduce by at least half
                    // to cap retries at CALIBRATION_MAX_TRIES.
                    long reduction = max(encoder_ticks * 2, outputLimit / 2);
                    outputLimit = max(outputLimit - reduction, 0L);
                    DEBUG_PRINT(" DOWN LIMIT NOT REACHED AT "); DEBUG_PRINT(encoder_ticks);
                    DEBUG_PRINT(" NEW LIMIT: "); DEBUG_PRINTLN(outputLimit);
                }
                delay(2000);
                if (serialAbort())
                    break;
            }
            // Pause between speed levels to let the motor cool down.
            lifterMotorStop();
            delay(CALIBRATION_COOLDOWN_MS);
            if (!sCalibrating || serialAbort())
            {
                Serial.println("SERIAL ABORT");
                success = false;
                break;
            }
        }
        seekToBottom(false);
        if (!success)
        {
            Serial.println("CALIBRATION ABORTED");
            sSettings.fMinimumPower = max(int(sSettings.fMinimumPower), LIFTER_MINIMUM_POWER);
            sCalibrating = false;
            return false;
        }

        sSettings.fDownLimitsCalibrated = true;
        sSettings.write();
        Serial.println("SUCCESS");
        // Recommend a minimum operating speed.  The calibrated fMinimumPower is
        // the lowest speed that can START the motor, but pulsed drive needs extra
        // headroom to sustain motion through sticky spots.  Add 5% as a margin.
        unsigned recSpeed = min((unsigned)100, sSettings.fMinimumPower + 5);
        Serial.print("RECOMMENDED min speed for this mechanism: ");
        Serial.print(recSpeed);
        Serial.println("%");
        Serial.print("  (calibrated minimum power: ");
        Serial.print(sSettings.fMinimumPower);
        Serial.println("%)");
        if (LIFTER_MINIMUM_POWER < recSpeed)
        {
            Serial.print("  NOTE: current 'Min lifter power' is ");
            Serial.print(LIFTER_MINIMUM_POWER);
            Serial.print("% — consider raising to ");
            Serial.print(recSpeed);
            Serial.println("% via Parameters page or #PMINPWR command");
        }
        sCalibrating = false;
        return true;
    }

    // Called every loop. Runs rotary ramp logic and, in move mode, fires random lift/spin actions.
    static void animate()
    {
        // If the safety maneuver has failed, cancel move mode and stop all motors.
        // This prevents random motion commands from firing when the droid is in
        // an unknown mechanical state.
        if (sSafetyManeuverFailed && fMoveMode)
        {
            moveModeEnd();
            lifterMotorStop();
            rotaryMotorStop();
        }
        rotaryMotorUpdate();
        if (fMoveMode && fMoveModeNextCmd < millis())
        {
            if (fMoveModeAuto)
                animateAuto();
            else
                animateParameterized();
        }
    }

    // Fully-randomized auto mode (bare :PM). Picks from a variety of visually
    // dramatic actions designed for short-travel periscopes.
    static void animateAuto()
    {
        // Compute usable position range.  safeMin is the lowest the lifter can
        // go while the rotary is away from home (fRotaryMinHeight as a fraction).
        float safeMin = (float)ROTARY_MINIMUM_HEIGHT / (float)max(1, (int)sSettings.getLifterDistance());
        float curPos = getLifterPositionClamped() / (float)max(1, (int)sSettings.getLifterDistance());

        // Randomize a lift speed for this action: 65-85% — fast enough to look
        // lively, slow enough for the pulsed soft stops to work well.
        float liftSpeed = (sSettings.fMinimumPower + random(21)) / 100.0;

        // Random rotary speed: minimum+10 to 90%
        int rotSpeed = ROTARY_MINIMUM_POWER + 10 + random(max(1, 90 - ROTARY_MINIMUM_POWER - 10));

        switch (random(6))
        {
            case 0:
            {
                // POP UP / DOWN — home rotary first so the lifter can travel the
                // full range safely, then pop to top, pause, drop to bottom.
                // This is one of the most visually dramatic moves.
                Serial.println("AUTO: pop up/down");
                rotateHome();
                if (serialAbort()) return;
                seekToPosition(1.0, liftSpeed);
                if (serialAbort()) return;
                delay(500 + random(1500));
                if (serialAbort()) return;
                seekToPosition(0.0, liftSpeed);
                if (serialAbort()) return;
                delay(300 + random(700));
                if (serialAbort()) return;
                // Pop back up to a random high position for the next action
                seekToPosition(0.7 + random(31) / 100.0, liftSpeed);
                fMoveModeNextCmd = millis() + (2 + random(3)) * 1000L;
                break;
            }
            case 1:
            {
                // BIG LIFT — seek to a position far from current, using the full
                // safe range.  Ensures visually obvious movement every time.
                Serial.println("AUTO: big lift");
                // Home rotary first so the periscope head doesn't hit the dome
                // when lowering. Without this, a previous "random angle" case
                // leaves the head rotated and lowering into the dome.
                rotateHome();
                if (serialAbort()) { fMoveModeNextCmd = millis() + 2000; break; }
                float lo = 0.0;
                float target;
                // Pick a target at least 30% away from current position
                int attempts = 0;
                do {
                    target = lo + random(101 - (int)(lo * 100)) / 100.0;
                } while (fabsf(target - curPos) < 0.30f && ++attempts < 10);
                // If we couldn't find a target far enough away, skip the seek
                // rather than making a tiny imperceptible movement.
                if (fabsf(target - curPos) >= 0.10f)
                    seekToPosition(target, liftSpeed);
                fMoveModeNextCmd = millis() + (2 + random(4)) * 1000L;
                break;
            }
            case 2:
            {
                // SPIN AT TOP — raise to full height, spin a random direction
                // for 2-5 seconds, then stop.
                Serial.println("AUTO: spin at top");
                if (!seekToPosition(1.0, liftSpeed) || serialAbort())
                {
                    fMoveModeNextCmd = millis() + 2000;
                    break;
                }
                {
                    int dir = random(2) ? 1 : -1;
                    rotaryMotorSpeed(dir * rotSpeed / 100.0);
                    uint32_t spinEnd = millis() + 2000 + random(3000);
                    while (millis() < spinEnd && !serialAbort())
                        delay(50);
                    rotaryMotorSpeed(0);
                    delay(200);
                }
                fMoveModeNextCmd = millis() + (1 + random(3)) * 1000L;
                break;
            }
            case 3:
            {
                // RANDOM ANGLE — raise, rotate to a random angle, hold.
                Serial.println("AUTO: random angle");
                if (!seekToPosition(1.0, liftSpeed) || serialAbort())
                {
                    fMoveModeNextCmd = millis() + 2000;
                    break;
                }
                {
                    int degrees = 30 + random(300);  // avoid tiny angles near 0
                    float spd = (ROTARY_MINIMUM_POWER + 5 + random(30)) / 100.0;
                    rotaryMotorAbsolutePosition(degrees, spd);
                }
                fMoveModeNextCmd = millis() + (3 + random(4)) * 1000L;
                break;
            }
            case 4:
            {
                // LOOK AROUND — raise to top, do 2-3 partial rotations as if
                // scanning, then come home.
                Serial.println("AUTO: look around");
                if (!seekToPosition(1.0, liftSpeed) || serialAbort())
                {
                    fMoveModeNextCmd = millis() + 2000;
                    break;
                }
                {
                    int numLooks = 2 + random(2);
                    for (int i = 0; i < numLooks && !serialAbort(); i++)
                    {
                        int degrees = 45 + random(270);
                        float spd = (ROTARY_MINIMUM_POWER + 5 + random(20)) / 100.0;
                        rotaryMotorAbsolutePosition(degrees, spd);
                        delay(800 + random(1200));
                    }
                    if (!serialAbort())
                        rotateHome();
                }
                fMoveModeNextCmd = millis() + (2 + random(3)) * 1000L;
                break;
            }
            case 5:
            {
                // BOB AND WEAVE — with rotary homed, bob up/down 2-3 times at
                // varying heights.  Uses the full travel range safely.
                Serial.println("AUTO: bob and weave");
                rotateHome();
                if (serialAbort()) return;
                {
                    int numBobs = 2 + random(2);
                    for (int i = 0; i < numBobs && !serialAbort(); i++)
                    {
                        float hi = 0.7 + random(31) / 100.0;
                        float lo2 = random(20) / 100.0;  // can go low since rotary is homed
                        seekToPosition(hi, liftSpeed);
                        if (serialAbort()) break;
                        delay(200 + random(500));
                        seekToPosition(lo2, liftSpeed);
                        if (serialAbort()) break;
                        delay(200 + random(500));
                    }
                    // End at a reasonable height for the next action
                    if (!serialAbort())
                        seekToPosition(0.6 + random(41) / 100.0, liftSpeed);
                }
                fMoveModeNextCmd = millis() + (1 + random(3)) * 1000L;
                break;
            }
        }
    }

    // Original parameterized move mode (e.g. :PM70,50,2,4)
    static void animateParameterized()
    {
        bool retry = false;
        do
        {
            switch (random(3))
            {
                case 0:
                {
                    uint8_t pos = random(100);
                    uint8_t speedpercentage = min(max(int(fMoveModeNextLifterSpeed), int(sSettings.fMinimumPower)), 100);
                    if (!seekToPosition(pos/100.0, speedpercentage/100.0))
                    {
                        retry = !retry;
                        continue;
                    }
                    retry = false;
                    break;
                }
                case 1:
                {
                    if (!rotaryAllowed())
                        seekToPosition(1.0, fMoveModeNextLifterSpeed/100.0);

                    int32_t speed = fMoveModeNextRotarySpeed;
                    if (speed == 0)
                        speed = 80;
                    speed = max(speed, (int32_t)ROTARY_MINIMUM_POWER);
                    speed = -speed + random(speed*2);
                    if (abs(speed) < ROTARY_MINIMUM_POWER+5)
                        speed = (speed < 0) ? -ROTARY_MINIMUM_POWER-5 : ROTARY_MINIMUM_POWER+5;
                    speed = min(max(speed, (int32_t)-100), (int32_t)100);
                    if (speed == 0)
                    {
                        rotateHome();
                    }
                    else
                    {
                        rotaryMotorSpeed(speed / 100.0);
                    }
                    retry = false;
                    break;
                }
                case 2:
                {
                    if (!rotaryAllowed())
                        seekToPosition(1.0, fMoveModeNextLifterSpeed/100.0);

                    uint32_t speedpercentage = max(int(fMoveModeNextRotarySpeed), ROTARY_MINIMUM_POWER);
                    speedpercentage = random(fMoveModeNextRotarySpeed - ROTARY_MINIMUM_POWER) + ROTARY_MINIMUM_POWER;
                    float speed = speedpercentage/100.0;
                    float maxspeed = speed;
                    if (speedpercentage < fMoveModeNextRotarySpeed)
                    {
                        speedpercentage = random(fMoveModeNextRotarySpeed - speedpercentage) + speedpercentage;
                        maxspeed = speedpercentage/100.0;
                    }
                    rotaryMotorAbsolutePosition(random(360), speed, maxspeed);
                    retry = false;
                    break;
                }
            }
        } while (retry);
        fMoveModeNextCmd = millis() + random(fMoveModeNextIntervalMin, fMoveModeNextIntervalMax) * 1000L;
    }

    static void moveMode(uint8_t nextLifterSpeed, uint8_t nextRotarySpeed, uint8_t nextIntervalMin, uint8_t nextIntervalMax)
    {
        fMoveMode = true;
        fMoveModeNextLifterSpeed = nextLifterSpeed;
        fMoveModeNextRotarySpeed = nextRotarySpeed;
        fMoveModeNextIntervalMin = nextIntervalMin;
        fMoveModeNextIntervalMax = nextIntervalMax;
        fMoveModeNextCmd = millis();
    }

    static void moveModeEnd()
    {
        fMoveMode = false;
        fMoveModeAuto = false;
        fMoveModeNextLifterSpeed = 0;
        fMoveModeNextRotarySpeed = 0;
        fMoveModeNextIntervalMin = 0;
        fMoveModeNextIntervalMax = 0;
        fMoveModeNextCmd = 0;
        // Don't clear fLifterTargetPos here — drift correction should
        // continue holding the last position after random mode ends.
    }

    // Start fully-randomized auto mode (bare :PM with no arguments)
    static void moveModeAutoStart()
    {
        fMoveMode = true;
        fMoveModeAuto = true;
        fMoveModeNextLifterSpeed = 0;   // unused in auto mode
        fMoveModeNextRotarySpeed = 0;   // unused in auto mode
        fMoveModeNextIntervalMin = 0;   // unused in auto mode
        fMoveModeNextIntervalMax = 0;   // unused in auto mode
        fMoveModeNextCmd = millis();
    }

private:

    ///////////////////////////////////

    static void resetLifterChangedState()
    {
        cli();
        encoder_lifter_changed = 0;
        sei();
    }

    static void resetRotaryChangedState()
    {
        cli();
        encoder_rotary_changed = 0;
        sei();
    }

    static void resetLifterPosition()
    {
        cli();
        encoder_lifter_val = 0;
        encoder_lifter_pin_A_last = 0;
        encoder_lifter_ticks = 0;    
        sei();
    }

    static void resetLifterPositionTop()
    {
        cli();
        encoder_lifter_val = 0;
        encoder_lifter_pin_A_last = 0;
        encoder_lifter_ticks = sSettings.getLifterDistance();
        sei();
    }

    static void resetRotaryPosition()
    {
        cli();
        encoder_rotary_val = 0;
        encoder_rotary_pin_A_last = 0;
        encoder_rotary_ticks = 0;    
        sei();
    }

    ///////////////////////////////////

    static bool getDownOutputLimit(float speed, float &limit)
    {
        speed = min(max(speed, 0.0f), 1.0f);
        size_t index = size_t(speed*100/5);
        limit = 0;
        if (index < sSettings.limitCount() && sSettings.fDownLimits[index].valid)
        {
            limit = sSettings.fDownLimits[index].outputLimit;
            return true;
        }
        DEBUG_PRINT("UNCALIBRATED SPEED - ");
        DEBUG_PRINT(speed);
        DEBUG_PRINT(" - ");
        DEBUG_PRINTLN(index);
        return false;
    }

    static bool getUpOutputLimit(float speed, float &limit)
    {
        speed = min(max(speed, 0.0f), 1.0f);
        size_t index = size_t(speed*100/5);
        limit = 0;
        if (index < sSettings.limitCount() && sSettings.fUpLimits[index].valid)
        {
            limit = sSettings.fUpLimits[index].outputLimit;
            return true;
        }
        DEBUG_PRINT("UNCALIBRATED SPEED - ");
        DEBUG_PRINT(speed);
        DEBUG_PRINT(" - ");
        DEBUG_PRINTLN(index);
        return false;
    }

    ///////////////////////////////////

public:
    // Mark the current rotary position as home (encoder zero).
    // Called by the rescue page's "Set Home" button.
    static void setRotaryHome() { resetRotaryPosition(); }

    // Returns the current lifter height as a percentage of full travel (0–100).
    static int getLifterHeightPercent()
    {
        // Use the calibrated travel distance so 100% = top limit switch position.
        // sLifterParameters.fLifterDistance is the generic default (e.g. 450 for Greg lifter)
        // and does not reflect the actual mechanical setup measured during calibration.
        int dist = (int)sSettings.getLifterDistance();
        if (dist <= 0) dist = sLifterParameters.fLifterDistance;
        if (dist <= 0) return 0;
        return (int)(getLifterPositionClamped() * 100L / dist);
    }

    // Saves the current lifter height as the safe spin threshold and persists it.
    // Rejects heights below 10 % of full travel to prevent accidentally setting
    // a value so low that the periscope head could hit the dome.
    static void setSafeSpinHeightToCurrent()
    {
        int dist = max(1, (int)sLifterParameters.fLifterDistance);
        int minAllowed = dist / 10;  // 10 % floor
        int current = (int)getLifterPosition();
        if (current < minAllowed)
        {
            Serial.println("Safe spin height too low — raise periscope higher first.");
            return;
        }
        sLifterParameters.fRotaryMinHeight = current;
        sLifterParameters.save();
        Serial.print("Safe spin height set to ");
        Serial.print(getLifterHeightPercent());
        Serial.println("%");
    }

    // Clear the cached encoder-ticks-per-revolution so it is re-measured on
    // the next safety maneuver. Use this if the limit switch position was
    // physically changed or a new motor was installed.
    static void clearRotaryEncoderCount()
    {
        sRotaryCircleEncoderCount = 0;
        sLifterParameters.fRotaryEncoderCount = 0;
        sLifterParameters.save();
        Serial.println("Rotary encoder count cleared — will re-measure on next boot.");
    }

    // Force rotary spin, bypassing all safety checks.
    // For rescue use only — normal code should use rotaryMotorSpeed().
    //
    // Why the two-step: rotaryMotorMove() only writes PWM when fRotaryThrottle
    // differs from the requested value.  If we set fRotaryThrottle = speed
    // first the change-detection guard sees no difference and the PWM is never
    // updated.  Resetting fRotaryThrottle to 0 first guarantees the write fires.
    static void rotaryMotorSpeedForce(float speed)
    {
        enableMotors();
        fRotaryThrottle = 0;              // reset so change-detection in rotaryMotorMove fires
        rotaryMotorMove(speed, true);     // write PWM directly, skipSafety=true
        fRotarySpeed = speed;             // keep target in sync so rotaryMotorUpdate does not ramp
    }

    static bool seekToTop(float speed = 0.5, bool usePID = true, bool updateDistance = true)
    {
        if (!usePID)
        {
            float mpower = sSettings.fMinimumPower/100.0 + 0.05 + 0.1 * speed;
            Serial.print(" MOTOR: "); Serial.println(mpower);

            // seek up
            bool topLimit;
            // Breakaway boost before pulsed drive
            breakawayBoost(mpower);
            long seekPrevPos = getLifterPosition();
            uint32_t seekPosChangeMs = millis();
            for (;;)
            {
                topLimit = lifterTopLimit();
                if (topLimit || serialAbort())
                    break;
                long cur = getLifterPosition();
                if (cur != seekPrevPos) { seekPrevPos = cur; seekPosChangeMs = millis(); }
                if (millis() - seekPosChangeMs > 2000)
                {
                    Serial.print("ABORT fault="); Serial.println(lifterMotorFault());
                    break;
                }
                lifterMotorMove(mpower);
                delay(3);
                lifterMotorStop();
                delay(1);
            }
            if (topLimit)
            {
                Serial.println("TOP LIMIT REACHED");
                // Capture the encoder count from the last bottom-reset to this top-limit
                // hit as the authoritative UP-direction travel distance. The encoder
                // overcounts going UP vs DOWN at higher motor speeds (EMI phantom edges),
                // so fLifterDistance must be set from an UP seek — not from seekToBottom.
                // Only update if the encoder is positive (i.e. started from a bottom ref).
                long upCount = getLifterPosition();
                if (updateDistance)
                {
                    unsigned stored = sLifterParameters.fLifterDistance;  // flash/default value
                    if (upCount < 50)
                    {
                        Serial.print("LIFTER DISTANCE REJECTED (too small): ");
                        Serial.println(upCount);
                        Serial.println("Keeping stored distance. Check top limit switch wiring.");
                    }
                    else if (stored >= 100 && abs(upCount - (long)stored) > (long)stored / 10)
                    {
                        // New measurement is >10% different from stored calibration.
                        // Dome EMI causes encoder undercounting; don't corrupt the
                        // good calibrated value.  Use the stored value instead.
                        Serial.print("LIFTER DISTANCE REJECTED (>10% from stored): measured=");
                        Serial.print(upCount);
                        Serial.print(" stored=");
                        Serial.println(stored);
                        sSettings.fLifterDistance = stored;
                    }
                    else
                    {
                        sSettings.fLifterDistance = (unsigned)upCount;
                        Serial.print("LIFTER DISTANCE (up): ");
                        Serial.println(sSettings.fLifterDistance);
                    }
                }
                else
                {
                    Serial.print("LIFTER DISTANCE (safety, not updating): measured=");
                    Serial.println(upCount);
                    // Use stored calibrated distance — safety maneuver runs with
                    // dome on where EMI corrupts encoder counts.
                    sSettings.fLifterDistance = sLifterParameters.fLifterDistance;
                }
                resetLifterPositionTop();
            }
            else
            {
                Serial.println("NOT TOP LIMIT");
            }
            return topLimit;
        }
        if (!isRotaryAtRest())
        {
            DEBUG_PRINTLN("ABORT: ROTARY NOT HOME");
            DEBUG_PRINTLN(rotaryMotorCurrentPosition());
            return false;
        }
        speed = min(max(speed, 0.0f), 1.0f);
        TargetSteering steering(sSettings.getLifterDistance());
        steering.setSampleTime(1);
        float limit;
        if (!getUpOutputLimit(speed, limit))
            return false;
        steering.setDistanceOutputLimits(limit);
    #ifdef USE_DEBUG
        uint32_t start = millis();
    #endif
        bool topLimit;
        long seekPrevPos2 = getLifterPosition();
        uint32_t seekPosChangeMs2 = millis();
        for (;;)
        {
            long encoder_ticks = getLifterPosition();
            topLimit = lifterTopLimit();
            if (topLimit || serialAbort())
                break;
            if (encoder_ticks != seekPrevPos2) { seekPrevPos2 = encoder_ticks; seekPosChangeMs2 = millis(); }
            if (millis() - seekPosChangeMs2 > 2000)
            {
                printf("LIFTER ABORTED AT %ld (POWER=%f)\n", encoder_ticks, steering.getThrottle() * speed);
                break;
            }
            steering.setCurrentDistance(encoder_ticks);
            lifterMotorMove(steering.getThrottle() * speed);
        }
        lifterMotorStop();
    #ifdef USE_DEBUG
        uint32_t stop = millis();
    #endif
        if (topLimit)
        {
            DEBUG_PRINTLN("TOP LIMIT REACHED");
            DEBUG_PRINT("DISTANCE: ");
            DEBUG_PRINTLN(getLifterPosition());
            DEBUG_PRINT("TIME: ");
            DEBUG_PRINTLN(stop - start);
            return true;
        }
        return false;
    }

    static bool seekToPositionSlow(float pos, float speed)
    {
        DEBUG_PRINT("SLOW: "); DEBUG_PRINT(speed);
        // ensure position is in the range of 0.0 [bottom] - 1.0 [top]
        pos = min(max(abs(pos), 0.0f), 1.0f);
        if (isRotarySpinning() || !rotaryHomeLimit())
        {
            // Cannot go below safe rotary height if spinning or not at home position
            float minSafePos = (float)ROTARY_MINIMUM_HEIGHT / (float)sSettings.getLifterDistance();
            pos = min(max(pos, minSafePos), 1.0f);
        }

        long maxlen = sSettings.getLifterDistance();
        long target_ticks = pos * maxlen;
        bool success = false;
        float mpower = sSettings.fMinimumPower/100.0 + 0.05 + 0.1 * speed;
        DEBUG_PRINT(" MOTOR: "); DEBUG_PRINTLN(mpower);
        if (target_ticks > getLifterPosition())
        {
            // seek up
            bool topLimit = false;
            bool reachedTarget = false;
            // Use a position-change timeout instead of LifterStatus tick-counting,
            // which requires 20 ticks/200ms and falsely trips on slow/pulsed motors.
            long seekPrevPos = getLifterPosition();
            uint32_t seekPosChangeMs = millis();
            for (;;)
            {
                long encoder_ticks = getLifterPosition();
                topLimit = lifterTopLimit();
                reachedTarget = (encoder_ticks >= target_ticks);
                if (topLimit || reachedTarget || serialAbort())
                    break;
                if (encoder_ticks != seekPrevPos)
                {
                    seekPrevPos = encoder_ticks;
                    seekPosChangeMs = millis();
                }
                if (millis() - seekPosChangeMs > 2000)
                {
                    DEBUG_PRINTLN("ABORT");
                    break;
                }
                lifterMotorMove(mpower);
                delay(3);
                lifterMotorStop();
                delay(2);
            }
            success = topLimit || reachedTarget;
        }
        else
        {
            // seek down
            bool botLimit = false;
            bool reachedTarget = false;
            // Use a position-change timeout instead of LifterStatus tick-counting,
            // which requires 20 ticks/200ms and falsely trips on slow/pulsed motors.
            long seekPrevPos = getLifterPosition();
            uint32_t seekPosChangeMs = millis();
            for (;;)
            {
                long encoder_ticks = getLifterPosition();
                botLimit = lifterBottomLimit();
                reachedTarget = (encoder_ticks <= target_ticks);
                if (botLimit || reachedTarget || serialAbort())
                    break;
                if (encoder_ticks != seekPrevPos)
                {
                    seekPrevPos = encoder_ticks;
                    seekPosChangeMs = millis();
                }
                if (millis() - seekPosChangeMs > 2000)
                {
                    DEBUG_PRINTLN("ABORT");
                    break;
                }
                lifterMotorMove(-mpower);
                delay(3);
                lifterMotorStop();
                delay(1);
            }
            success = botLimit || reachedTarget;
        }
        lifterMotorStop();
        return success;
    }

    // Stall detector: isMoving() returns false if fewer than 20 encoder ticks in the last interval.
    class RotaryStatus
    {
    public:
        RotaryStatus()
        {
            resetRotaryChangedState();
            encoder_rotary_last_status = millis();
        }

        bool isMoving()
        {
            if (millis() - encoder_rotary_last_status >= ENCODER_STATUS_RATE)
            {
                encoder_rotary_last_status = millis();
                if (encoder_rotary_changed < 20)
                    return false;
                resetRotaryChangedState();
            }
            return !rotaryMotorFault();
        }
    };

    // Same stall detection logic as RotaryStatus, applied to the lifter.
    class LifterStatus
    {
    public:
        LifterStatus()
        {
            resetLifterChangedState();
            encoder_lifter_last_status = millis();
        }

        bool isMoving()
        {
            if (millis() - encoder_lifter_last_status >= ENCODER_STATUS_RATE)
            {
                encoder_lifter_last_status = millis();
                if (encoder_lifter_changed < 20)
                {
                    if (sVerboseDebug)
                    {
                        Serial.println("NO CHANGE");
                    }
                    return false;
                }
                resetLifterChangedState();
            }
            // printf("lifterMotorFault() : %d\n", lifterMotorFault());
            return !lifterMotorFault();
        }
    };

    // Encoder state — volatile because these are written from the interrupt service routine.
    static volatile long encoder_lifter_ticks;
    static volatile int encoder_lifter_pin_A_last;
    static volatile int encoder_lifter_val;
    static volatile uint16_t encoder_lifter_changed;  // tick count reset each stall-check interval
    static uint32_t encoder_lifter_last_status;

    static volatile long encoder_rotary_ticks;
    static volatile long encoder_rotary_stop_ticks;   // tick count when home was last detected
    static volatile int encoder_rotary_pin_A_last;
    static volatile int encoder_rotary_val;
    static volatile uint16_t encoder_rotary_changed;
    static volatile bool encoder_rotary_stop_limit;   // when true, ISR watches for the home switch
    static uint32_t encoder_rotary_last_status;

    // Motor and motion state
    static bool fMotorsEnabled;
    static uint32_t fMotorsEnabledTime;
    static float fLifterThrottle;
    static float fRotarySpeed;
    static float fRotaryThrottle;
    static uint32_t fRotaryThrottleUpdate;
    static uint32_t fRotaryEncoderLastStatus;
    static long fRotaryEncoderLastTick;
    static bool fRotaryMoving;
    static int8_t fLightShow;

    // Drift correction: last commanded position (as fraction 0.0–1.0), -1 = no target
    static float fLifterTargetPos;
    static bool fLifterHolding;  // true when lifter brake is active (holding position)

    // Move mode state
    static bool fMoveMode;
    static bool fMoveModeAuto;      // true = fully randomized (bare :PM), false = parameterized
    static uint32_t fMoveModeNextCmd;
    static uint8_t fMoveModeNextLifterSpeed;
    static uint8_t fMoveModeNextRotarySpeed;
    static uint8_t fMoveModeNextIntervalMin;
    static uint8_t fMoveModeNextIntervalMax;
};

// Encoder ISRs — called on every encoder pin change. IRAM_ATTR keeps them in fast RAM.
// On each rising edge of channel A, channel B indicates direction.

void IRAM_ATTR
PeriscopeLifter::measureLifterEncoder()
{
    encoder_lifter_val = digitalRead(PIN_LIFTER_ENCODER_A);
    if (encoder_lifter_pin_A_last == LOW && encoder_lifter_val == HIGH)
    {
        if (digitalRead(PIN_LIFTER_ENCODER_B) == LOW)
            encoder_lifter_ticks--;
        else
            encoder_lifter_ticks++;
        encoder_lifter_changed++;
    }
    encoder_lifter_pin_A_last = encoder_lifter_val;
}

void IRAM_ATTR
PeriscopeLifter::measureRotaryEncoder()
{
    encoder_rotary_val = digitalRead(PIN_ROTARY_ENCODER_A);
    if (encoder_rotary_pin_A_last == LOW && encoder_rotary_val == HIGH)
    {
        if (digitalRead(PIN_ROTARY_ENCODER_B) == LOW)
            encoder_rotary_ticks--;
        else
            encoder_rotary_ticks++;
        encoder_rotary_changed++;
        // UNSAFE cannot be called from PinInterrupt
        // if (encoder_rotary_stop_limit && rotaryHomeLimit())
        // {
        //     encoder_rotary_stop_ticks = encoder_rotary_ticks;
        //     // Stop rotary motor if limit switch was hit
        //     fRotarySpeed = 0;
        //     ::analogWrite(PIN_ROTARY_PWM1, 0);
        //     ::analogWrite(PIN_ROTARY_PWM2, 0);
        //     fRotaryThrottle = 0;
        //     fRotaryMoving = false;
        // }
    }
    encoder_rotary_pin_A_last = encoder_rotary_val;
}

// Static member variable definitions — required by C++ to allocate storage outside the class.

volatile long PeriscopeLifter::encoder_lifter_ticks;
volatile int PeriscopeLifter::encoder_lifter_pin_A_last;
volatile int PeriscopeLifter::encoder_lifter_val;
volatile uint16_t PeriscopeLifter::encoder_lifter_changed;
uint32_t PeriscopeLifter::encoder_lifter_last_status;

volatile long PeriscopeLifter::encoder_rotary_ticks;
volatile long PeriscopeLifter::encoder_rotary_stop_ticks;
volatile int PeriscopeLifter::encoder_rotary_pin_A_last;
volatile int PeriscopeLifter::encoder_rotary_val;
volatile uint16_t PeriscopeLifter::encoder_rotary_changed;
volatile bool PeriscopeLifter::encoder_rotary_stop_limit;
uint32_t PeriscopeLifter::encoder_rotary_last_status;

bool PeriscopeLifter::fMotorsEnabled;
uint32_t PeriscopeLifter::fMotorsEnabledTime;
float PeriscopeLifter::fLifterThrottle;
float PeriscopeLifter::fRotarySpeed;
float PeriscopeLifter::fRotaryThrottle;
uint32_t PeriscopeLifter::fRotaryThrottleUpdate;
uint32_t PeriscopeLifter::fRotaryEncoderLastStatus;
long PeriscopeLifter::fRotaryEncoderLastTick;
bool PeriscopeLifter::fRotaryMoving;
int8_t PeriscopeLifter::fLightShow;
float PeriscopeLifter::fLifterTargetPos = -1;
bool PeriscopeLifter::fLifterHolding;

bool PeriscopeLifter::fMoveMode;
bool PeriscopeLifter::fMoveModeAuto;
uint32_t PeriscopeLifter::fMoveModeNextCmd;
uint8_t PeriscopeLifter::fMoveModeNextLifterSpeed;
uint8_t PeriscopeLifter::fMoveModeNextRotarySpeed;
uint8_t PeriscopeLifter::fMoveModeNextIntervalMin;
uint8_t PeriscopeLifter::fMoveModeNextIntervalMax;

///////////////////////////////////////////////////////

// The single global lifter/rotary controller instance.
PeriscopeLifter lifter;

///////////////////////////////////////////////////////
// WIFI AND REMOTE STATE
//////////////////////////////////////////////////////

#ifdef USE_WIFI
WifiAccess wifiAccess;
bool wifiEnabled;
bool wifiActive;
#endif

#ifdef USE_DROID_REMOTE
bool remoteEnabled;
bool remoteActive;
#endif

#ifdef USE_WIFI_MARCDUINO
WifiMarcduinoReceiver wifiMarcduinoReceiver(wifiAccess);
#endif

#ifdef USE_WIFI
TaskHandle_t eventTask; // FreeRTOS task running WiFi/OTA/web on core 0 (separate from motor control)
#endif

#ifdef USE_OTA
bool otaInProgress;
#endif

///////////////////////////////////////////////////////
// COMMAND PARSING UTILITIES
// Custom number and keyword parsers used by the command handlers.
///////////////////////////////////////////////////////

// Read exactly numdigits ASCII digit characters and return the integer.
int atoi(const char* cmd, int numdigits)
{
    int result = 0;
    for (int i = 0; i < numdigits; i++)
        result = result*10 + (cmd[i]-'0');
    return result;
}

// Parse a signed integer from cmd, advancing *endptr past the digits.
int32_t strtol(const char* cmd, const char** endptr)
{
    bool sign = false;
    int32_t result = 0;
    if (*cmd == '-')
    {
        cmd++;
        sign = true;
    }
    while (isdigit(*cmd))
    {
        result = result*10L + (*cmd-'0');
        cmd++;
    }
    *endptr = cmd;
    return (sign) ? -result : result;
}

// Parse an unsigned integer from cmd, advancing *endptr past the digits.
uint32_t strtolu(const char* cmd, const char** endptr)
{
    uint32_t result = 0;
    while (isdigit(*cmd))
    {
        result = result*10L + (*cmd-'0');
        cmd++;
    }
    *endptr = cmd;
    return result;
}

// Returns true if cmd starts with str, and advances cmd past those characters.
bool startswith(const char* &cmd, const char* str)
{
    size_t len = strlen(str);
    if (strncmp(cmd, str, len) == 0)
    {
        cmd += len;
        return true;
    }
    return false;
}

// Parse and execute a single periscope command (the part after ":P").
// Returns false if the command is unrecognized.
bool processLifterCommand(const char* cmd)
{
    // move mode ends on the next serial command
    switch (*cmd++)
    {
        case 'S':
        {
            // stop move mode
            lifter.moveModeEnd();

            // play sequence
            uint32_t seq = strtolu(cmd, &cmd);
            if (*cmd == '\0')
            {
                seq = min(max(int(seq), 0), MAX_COMMANDS);
                if (!sSettings.readCommand(seq, sCmdBuffer, sizeof(sCmdBuffer), ":P"))
                {
                    sCmdBuffer[0] = '\0';
                }
            }
            break;
        }
        case 'P':
        {
            // stop move mode
            lifter.moveModeEnd();

            // seek lifter to position
            float speed = 1.0;
            uint32_t pos;
            if (*cmd == 'R')
            {
                pos = random(100);
                cmd++;
            }
            else
            {
                pos = strtolu(cmd, &cmd);
            }
            if (*cmd == ',')
            {
                uint32_t speedpercentage = strtolu(cmd+1, &cmd);
                if (*cmd == '\0')
                {
                    speedpercentage = min(max(int(speedpercentage), 0), 100);
                    speed = speedpercentage / 100.0;
                }
            }
            if (*cmd == '\0' || *cmd == ':')
            {
                pos = min(max(int(pos), 0), 100);
                Serial.print("POSITION: "); Serial.print(pos);
                Serial.print(" SPEED: "); Serial.println(int(speed*100));
                lifter.seekToPosition(pos/100.0, speed);
            }
            break;
        }
        case 'M':
        {
            // stop move mode
            lifter.moveModeEnd();

            if (*cmd == '\0' || *cmd == ':')
            {
                // Bare :PM — fully randomized auto mode.
                // Picks from a variety of dramatic, safe actions each cycle.
                Serial.println("MOVE MODE: auto random");
                lifter.moveModeAutoStart();
            }
            else
            {
                // Parameterized: :PM{liftSpeed},{rotSpeed},{minInterval},{maxInterval}
                int nextLifterSpeed = sSettings.fMinimumPower+5;
                int nextRotarySpeed = ROTARY_MINIMUM_POWER+5;
                int nextIntervalMin = 1 + random(MOVEMODE_MAX_INTERVAL);
                int nextIntervalMax = nextIntervalMin + random(MOVEMODE_MAX_INTERVAL);
                if (*cmd == ',')
                {
                    nextLifterSpeed = strtolu(cmd+1, &cmd);
                    nextLifterSpeed = max(nextLifterSpeed, int(sSettings.fMinimumPower));
                }
                if (*cmd == ',')
                {
                    nextRotarySpeed = strtolu(cmd+1, &cmd);
                    nextRotarySpeed = max(nextRotarySpeed, ROTARY_MINIMUM_POWER+5);
                }
                if (*cmd == ',')
                {
                    nextIntervalMin = strtolu(cmd+1, &cmd);
                    nextIntervalMin = max(nextIntervalMin, 1);
                }
                if (*cmd == ',')
                {
                    nextIntervalMax = strtolu(cmd+1, &cmd);
                    nextIntervalMax = max(nextIntervalMax, nextIntervalMin+1);
                }
                lifter.moveMode(nextLifterSpeed, nextRotarySpeed, nextIntervalMin, nextIntervalMax);
            }
            break;
        }
        case 'R':
        {
            // stop move mode
            lifter.moveModeEnd();

            // spin rotary speed
            int32_t speed = 0;
            if (*cmd == 'R')
            {
                speed = strtolu(cmd+1, &cmd);
                if (speed == 0)
                    speed = 80;
                speed = max(speed, (int32_t)ROTARY_MINIMUM_POWER);
                speed = -speed + random(speed*2);
                if (abs(speed) < ROTARY_MINIMUM_POWER)
                    speed = (speed < 0) ? -ROTARY_MINIMUM_POWER : ROTARY_MINIMUM_POWER;
            }
            else
            {
                speed = strtol(cmd, &cmd);
            }
            if (*cmd == '\0')
            {
                // Raise lifter to safe height if needed (same as random mode)
                if (!lifter.rotaryAllowed())
                    lifter.seekToPosition(1.0, sSettings.fMinimumPower/100.0);
                if (lifter.rotaryAllowed())
                {
                    speed = min(max(speed, (int32_t)-100), (int32_t)100);
                    Serial.print("ROTARY SPEED: "); Serial.println(speed);
                    if (speed == 0)
                    {
                        lifter.rotateHome();
                    }
                    else
                    {
                        lifter.rotaryMotorSpeed(speed / 100.0);
                    }
                }
            }
            break;
        }
        case 'A':
        case 'D':
        {
            bool relative = (cmd[-1] == 'D');
            // stop move mode
            lifter.moveModeEnd();

            // position absolute degree
            float speed = 0;
            float maxspeed = 0;
            int32_t degrees;
            if (*cmd == 'R' && (cmd[1] == ',' || cmd[1] == '\0'))
            {
                degrees = random(360);
                cmd++;
            }
            else
            {
                degrees = strtol(cmd, &cmd);
            }
            if (*cmd == ',')
            {
                uint32_t speedpercentage;
                if (cmd[1] == 'R' && (cmd[2] == ',' || cmd[2] == '\0'))
                {
                    speedpercentage = random(100);
                    cmd += 2;
                }
                else
                {
                    speedpercentage = strtolu(cmd+1, &cmd);
                }
                if (*cmd == ',' || *cmd == '\0')
                {
                    speedpercentage = max(min(max(int(speedpercentage), 0), 100), ROTARY_MINIMUM_POWER);
                    speed = speedpercentage / 100.0;
                }
            }
            if (*cmd == ',')
            {
                uint32_t speedpercentage;
                if (cmd[1] == 'R' && (cmd[2] == ',' || cmd[2] == '\0'))
                {
                    speedpercentage = speed * 100.0 + random(100 - speed * 100.0);
                    cmd += 2;
                }
                else
                {
                    speedpercentage = strtolu(cmd+1, &cmd);
                }
                if (*cmd == '\0')
                {
                    speedpercentage = max(min(max(int(speedpercentage), 0), 100), int(speed)*100);
                    maxspeed = speedpercentage / 100.0;
                }
            }
            if (*cmd == '\0')
            {
                // Raise lifter to safe height if needed (same as random mode)
                if (!lifter.rotaryAllowed())
                    lifter.seekToPosition(1.0, sSettings.fMinimumPower/100.0);
                if (lifter.rotaryAllowed())
                {
                    if (sVerboseDebug)
                    {
                        Serial.print("ROTARY DEGREE: "); Serial.println(degrees);
                    }
                    if (relative)
                    {
                        lifter.rotaryMotorRelativePosition(degrees);
                    }
                    else if (degrees == 0)
                    {
                        lifter.rotateHome();
                    }
                    else
                    {
                        lifter.rotaryMotorAbsolutePosition(degrees, speed, maxspeed);
                    }
                }
            }
            break;
        }
        case 'W':
        {
            // wait seconds
            bool rand = false;
            uint32_t seconds;
            if (sVerboseDebug)
            {
                Serial.print("WAIT: "); Serial.println(cmd);
            }
            if ((rand = (*cmd == 'R')))
                cmd++;
            seconds = strtolu(cmd, &cmd);
            if (rand)
            {
                if (seconds == 0)
                    seconds = 6;
                seconds = random(1, seconds);
            }
            if (sVerboseDebug)
            {
                Serial.print("WAIT SECONDS: "); Serial.println(seconds);
            }
            if (*cmd == '\0')
            {
                // Cap at 60 seconds — a longer wait almost certainly indicates a
                // runaway or malformed sequence, and would make the droid appear
                // frozen for minutes at an event.
                if (seconds > 60)
                {
                    Serial.print("WAIT clamped from ");
                    Serial.print(seconds);
                    Serial.println("s to 60s");
                    seconds = 60;
                }
                sWaitNextSerialCommand = millis() + uint32_t(max(int(seconds), 1)) * 1000L;
            }
            break;
        }
        case 'L':
        {
            // play light sequence
            uint32_t seq;
            if (*cmd == 'R' && cmd[1] == '\0')
            {
                do
                {
                    seq = random(8);
                }
                while (seq == lifter.kLightKit_Off);
                Serial.print("RAND: "); Serial.println(seq);
                cmd++;
            }
            else
            {
                seq = strtolu(cmd, &cmd);
            }
            if (*cmd == '\0')
            {
                seq = min(max(int(seq), 0), 7);
                if (seq == 0 && seq != lifter.kLightKit_Off)
                {
                    lifter.setLightShow(lifter.kLightKit_Off);
                    delay(10);
                }
                Serial.print("LIGHT: "); Serial.println(seq);
                lifter.setLightShow(seq);
            }
            break;
        }
        case 'H':
        {
            // stop move mode
            lifter.moveModeEnd();

            Serial.print(":PH cmd — pos=");
            Serial.print(lifter.getLifterPosition());
            Serial.print("/");
            Serial.print(sSettings.getLifterDistance());
            Serial.print(" bot=");
            Serial.print(lifter.lifterBottomLimit() ? "1" : "0");
            Serial.print(" top=");
            Serial.print(lifter.lifterTopLimit() ? "1" : "0");
            Serial.print(" home=");
            Serial.print(lifter.rotaryHomeLimit() ? "1" : "0");
            Serial.print(" rotAllowed=");
            Serial.print(lifter.rotaryAllowed() ? "1" : "0");
            Serial.print(" safety=");
            Serial.println(sSafetyManeuver ? "OK" : (sSafetyManeuverFailed ? "FAIL" : "PENDING"));

            // return home — skip if already at bottom (limit switch or near-zero position)
            if (lifter.ensureSafetyManeuver()
                && !lifter.lifterBottomLimit()
                && lifter.getLifterPositionClamped() > 5)
            {
                uint32_t speed = sSettings.fMinimumPower;
                // If rotary can't move (lifter too low), raise to safe height first.
                // But if rotary is already home, no need to raise — just skip to descent.
                if (!lifter.rotaryAllowed() && !lifter.rotaryHomeLimit())
                {
                    Serial.println(":PH raising to safe height for rotary");
                    lifter.seekToPosition(1.0, speed/100.0);
                }
                lifter.rotateHome();
                lifter.setLightShow(lifter.kLightKit_Off);
                lifter.rotateHome();
                if (isdigit(*cmd))
                {
                    speed = min(max(strtolu(cmd, &cmd), (uint32_t)sSettings.fMinimumPower), (uint32_t)100);
                }
                if (!lifter.seekToPosition(0, speed/100.0))
                {
                    if (lifter.getLifterPosition() > 5)
                    {
                        // If we failed and lifter not close to 0 position we try one more time
                        lifter.seekToPosition(1.0, speed/100.0);
                        lifter.rotateHome();
                        lifter.seekToPosition(0, speed/100.0);
                    }
                }
            }
            break;
        }
        default:
            // stop move mode
            lifter.moveModeEnd();

            Serial.println("Invalid");
            return false;
    }
    return true;
}

#define UPDATE_SETTING(a,b) { \
    if (a != b) { a = b; needsUpdate = true; } else { unchanged = true; } }

// Parse and execute a configuration command (the part after "#P").
// Handles calibration, WiFi, remote pairing, limit switch polarity, stored sequences, etc.
void processConfigureCommand(const char* cmd)
{
    bool needsUpdate = false;
    bool unchanged = false;
    if (strcmp(cmd, "SC") == 0)
    {
        // calibrate
        if (!lifter.calibrate())
            lifter.disableMotors();
    }
    else if (startswith(cmd, "RC"))
    {
        if (*cmd == '0')
        {
            sRCMode = false;
            Serial.println("RC Mode (PPM) disabled.");
        }
        else if (*cmd == '1')
        {
            sRCMode = true;
            Serial.println("RC Mode (PPM) enabled.");
        }
        else
        {
            Serial.println("Invalid");
        }
    }
    else if (startswith(cmd, "ID") && isdigit(*cmd))
    {
        uint32_t id = strtolu(cmd, &cmd);
        if (id != sSettings.fID)
        {
            sSettings.fID = id;
            Serial.print("ID Changed to: "); Serial.println(sSettings.fID);
            sSettings.write();
        }
    }
    else if (startswith(cmd, "ZERO"))
    {
        sSettings.clearCommands();
        Serial.println("Cleared");
    }
    else if (startswith(cmd, "FACTORY"))
    {
        sSettings.clearCommands();
        preferences.clear();
        sLifterParameters.load();
        Serial.println("Cleared");
        reboot();
    }
    else if (startswith(cmd, "RESTART"))
    {
        reboot();
    }
#ifdef USE_DROID_REMOTE
    else if (startswith(cmd, "RMASTERKEY0"))
    {
        if (preferences.remove(PREFERENCE_REMOTE_LMK))
        {
            printf("Master Key Deleted.\n");
        }
        else
        {
            printf("No master key.\n");
        }
    }
    else if (startswith(cmd, "RMASTERKEY"))
    {
        SMQLMK lmk;
        printf("New Master Key Generated. All devices must be paired again\n");
        SMQ::createLocalMasterKey(&lmk);
        preferences.putBytes(PREFERENCE_REMOTE_LMK, &lmk, sizeof(lmk));
        SMQ::setLocalMasterKey(&lmk);
    }
#endif
    else if (startswith(cmd, "DEBUG") && isdigit(*cmd))
    {
        bool debugSetting = (strtolu(cmd, &cmd) == 1);
        if (sVerboseDebug != debugSetting)
        {
            if (debugSetting)
            {
                Serial.println(F("Debug Enabled"));
            }
            else
            {
                Serial.println(F("Debug Disabled"));
            }
            sVerboseDebug = debugSetting;
        }
    }
#ifdef USE_WIFI
    else if (startswith(cmd, "WIFIRESET"))
    {
        lifter.lifterMotorStop();
        preferences.putString(PREFERENCE_WIFI_SSID, getHostName());
        preferences.putString(PREFERENCE_WIFI_PASS, WIFI_AP_PASSPHRASE);
        preferences.putBool(PREFERENCE_WIFI_AP, WIFI_ACCESS_POINT);
        preferences.putBool(PREFERENCE_WIFI_ENABLED, WIFI_ENABLED);
        Serial.println("\n\nWifi credenditals reset to default. Restarting ...\n\n\n"); Serial.flush();
        ESP.restart();
    }
#endif
#ifdef USE_WIFI
    else if (startswith(cmd, "WIFI"))
    {
        bool wifiSetting = wifiEnabled;
        switch (*cmd)
        {
            case '0':
                wifiSetting = false;
                break;
            case '1':
                wifiSetting = true;
                break;
            case '\0':
                // Toggle WiFi
                wifiSetting = !wifiSetting;
                break;
        }
        if (wifiEnabled != wifiSetting)
        {
            if (wifiSetting)
            {
                preferences.putBool(PREFERENCE_WIFI_ENABLED, true);
                Serial.println(F("WiFi Enabled"));
            }
            else
            {
                preferences.putBool(PREFERENCE_WIFI_ENABLED, false);
                Serial.println(F("WiFi Disabled"));
            }
            reboot();
        }
        else
        {
            Serial.println(F("Unchanged"));
        }
    }
#endif
#ifdef USE_DROID_REMOTE
    else if (startswith(cmd, "REMOTE") && isdigit(*cmd))
    {
        bool remoteSetting = (strtolu(cmd, &cmd) == 1);
        if (remoteEnabled != remoteSetting)
        {
            if (remoteSetting)
            {
                preferences.putBool(PREFERENCE_REMOTE_ENABLED, true);
                Serial.println(F("Remote Enabled"));
            }
            else
            {
                preferences.putBool(PREFERENCE_REMOTE_ENABLED, false);
                Serial.println(F("Remote Disabled"));
            }
            reboot();
        }
    }
    else if (startswith(cmd, "RNAME"))
    {
        String newName = String(cmd);
        if (preferences.getString(PREFERENCE_REMOTE_HOSTNAME, SMQ_HOSTNAME) != newName)
        {
            preferences.putString(PREFERENCE_REMOTE_HOSTNAME, cmd);
            printf("Changed.\n");
            reboot();
        }
    }
    else if (startswith(cmd, "RSECRET"))
    {
        String newSecret = String(cmd);
        if (preferences.getString(PREFERENCE_REMOTE_SECRET, SMQ_HOSTNAME) != newSecret)
        {
            preferences.putString(PREFERENCE_REMOTE_SECRET, newSecret);
            printf("Changed.\n");
            reboot();
        }
    }
    else if (startswith(cmd, "RPAIR"))
    {
        printf("Pairing Started ...\n");
        SMQ::startPairing();
    }
    else if (startswith(cmd, "RUNPAIR"))
    {
        if (preferences.remove(PREFERENCE_REMOTE_PAIRED))
        {
            printf("Unpairing Success...\n");
            reboot();
        }
        else
        {
            printf("Not Paired...\n");
        }
    }
#endif
    else if (startswith(cmd, "STATUS"))
    {
    #ifdef USE_WIFI
        if (wifiEnabled)
        {
            Serial.println(F("WiFi Enabled"));
        }
        else
        {
            Serial.println(F("WiFi Disabled"));
        }
    #endif
    #ifdef USE_DROID_REMOTE
        if (remoteEnabled)
        {
            Serial.println(F("Remote Enabled"));
        }
        else
        {
            Serial.println(F("Remote Disabled"));
        }
     #endif
    }
    else if (startswith(cmd, "CONFIG"))
    {
        Serial.print(F("Firmware:           ")); Serial.println(F(FIRMWARE_VERSION));
        Serial.print(F("ID#:                ")); Serial.println(sSettings.fID);
        Serial.print(F("Baud Rate:          ")); Serial.println(sSettings.fBaudRate);
        Serial.print(F("Rotary Disabled:    ")); Serial.println(sSettings.fDisableRotary);
        Serial.print(F("Min Power:          ")); Serial.println(sSettings.fMinimumPower);
        Serial.print(F("Distance:           ")); Serial.println(sSettings.getLifterDistance());
        Serial.print(F("Lifter Limit:       ")); Serial.println(sSettings.fLifterLimitSetting);
        Serial.print(F("Rotary Limit:       ")); Serial.println(sSettings.fRotaryLimitSetting);
        Serial.print(F("Up Calibrated:      ")); Serial.println(sSettings.fUpLimitsCalibrated);
        Serial.print(F("Lifter Min Power:   ")); Serial.print(sLifterParameters.fLifterMinPower);
        Serial.print(F(" [")); Serial.print(sLifterParameters.fLifterMinSeekBotPower); Serial.println("]");
        Serial.print(F("Rotary Min Power:   ")); Serial.println(sLifterParameters.fRotaryMinPower);
        Serial.print(F("Lifter Distance:    ")); Serial.println(sLifterParameters.fLifterDistance);
        Serial.print(F("Rotary Min Height:  ")); Serial.println(sLifterParameters.fRotaryMinHeight);
        Serial.print(F("Safety Maneuver:    ")); Serial.println(sSafetyManeuver);
        if (sSafetyManeuverFailed)
            Serial.print(F("Safety Maneuver FAILED"));
        Serial.println("Stored Sequences:");
        sSettings.listSortedCommands(Serial);
    }
    else if (startswith(cmd, "L"))
    {
        Serial.println("Stored Sequences:");
        sSettings.listSortedCommands(Serial);
    }
    else if (startswith(cmd, "D"))
    {
        if (isdigit(*cmd))
        {
            uint32_t seq = strtolu(cmd, &cmd);
            Serial.println(seq);
            if (sSettings.deleteCommand(seq))
                Serial.println("Deleted");
        }
    }
    else if (startswith(cmd, "BAUD"))
    {
        uint32_t baudrate = strtolu(cmd, &cmd);
        if (baudrate > 1200 && sSettings.fBaudRate != baudrate)
        {
            sSettings.fBaudRate = baudrate;
            Serial.print("Reboot baud rate: "); Serial.println(sSettings.fBaudRate);
            sSettings.write();
        }
    }
    else if (startswith(cmd, "R"))
    {
        lifter.lifterMotorStop();
        sSettings.fDisableRotary = !sSettings.fDisableRotary;
        sSettings.write();
        Serial.println(sSettings.fDisableRotary ? "Disabled" : "Enabled");
        Serial.flush(); delay(1000);
        ESP.restart();
    }
    else if (startswith(cmd, "N"))
    {
        if (cmd[1] == 'L')
        {
            Serial.println("LIFT");
            // PNCL - lifter limit normally closed
            // PNOL - lifter limit normally open
           if ((cmd[0] == 'C' && sSettings.fLifterLimitSetting) || 
                (cmd[0] == 'O' && !sSettings.fLifterLimitSetting))
            {
                Serial.println("Changed");
                sSettings.fLifterLimitSetting = !sSettings.fLifterLimitSetting;
                sSettings.write();
            }
        }
        else if (cmd[1] == 'R')
        {
            // PNCR - rotary limit normally closed
            // PNOR - rotary limit normally open
            if ((cmd[0] == 'C' && sSettings.fRotaryLimitSetting) || 
                (cmd[0] == 'O' && !sSettings.fRotaryLimitSetting))
            {
                Serial.println("Changed");
                sSettings.fRotaryLimitSetting = !sSettings.fRotaryLimitSetting;
                sSettings.write();
            }
        }
        else
        {
            Serial.println("Invalid");
        }
    }
    else if (startswith(cmd, "S"))
    {
        uint32_t storeseq = strtolu(cmd, &cmd);
        if (*cmd == ':')
        {
            storeseq = min(max(int(storeseq), 0), 100);
            const char* startcmd = ++cmd;
            while (*cmd != '\0')
            {
                switch (*cmd)
                {
                    case 'P':
                    {
                        // position
                        float speed = 1.0;
                        uint32_t pos = strtolu(cmd+1, &cmd);
                        // optional speed
                        if (*cmd == ',')
                        {
                            uint32_t speedpercentage = strtolu(cmd+1, &cmd);
                            speedpercentage = max(min(max(int(speedpercentage), 0), 100), int(sSettings.fMinimumPower));
                            speed = speedpercentage / 100.0;
                        }
                        pos = min(max(int(pos), 0), 100);
                        Serial.print("Lifter Position: ");
                        Serial.print(pos);
                        Serial.print(" Speed: "); Serial.println(int(speed*100));
                        break;
                    }
                    case 'R':
                    {
                        // speed
                        int32_t speed = strtol(cmd+1, &cmd);
                        speed = min(max(speed, (int32_t)-100), (int32_t)100);
                        Serial.print("Rotate Scope Speed: ");
                        Serial.println(speed);
                        break;
                    }
                    case 'A':
                    case 'D':
                    {
                        bool relative = (cmd[0] == 'D');
                        bool randdegrees = false;
                        bool randspeed = false;
                        bool randmax = false;
                        uint32_t speed = 0;
                        uint32_t maxspeed = 0;
                        int32_t degrees = 0;
                        cmd += 1;
                        if (*cmd == 'R')
                        {
                            randdegrees = true;
                            cmd++;
                        }
                        else
                        {
                            degrees = strtol(cmd, &cmd);
                        }
                        Serial.println(cmd);
                        // optional speed
                        if (*cmd == ',')
                        {
                            if (cmd[1] == 'R')
                            {
                                randspeed = true;
                                cmd += 2;
                            }
                            else
                            {
                                speed = strtolu(cmd+1, &cmd);
                                speed = max(min(max(int(speed), 0), 100), ROTARY_MINIMUM_POWER);
                            }
                        }
                        // optional maxspeed
                        if (*cmd == ',')
                        {
                            if (cmd[1] == 'R')
                            {
                                randmax = true;
                                cmd += 2;
                            }
                            else
                            {
                                maxspeed = strtolu(cmd+1, &cmd);
                                maxspeed = max(min(max(int(maxspeed), 0), 100), int(speed));
                            }
                        }
                        Serial.print("Rotate ");
                        Serial.print((relative) ? "Relative" : "Absolute");
                        Serial.print(" Degrees: ");
                        if (randdegrees)
                            Serial.print("Random");
                        else
                            Serial.print(degrees);
                        if (speed != 0 || randspeed)
                        {
                            Serial.print(" Speed: ");
                            if (randspeed)
                                Serial.print("Random");
                            else
                                Serial.print(speed);
                        }
                        if (maxspeed != 0 || randmax)
                        {
                            Serial.print(" Max Speed: ");
                            if (randmax)
                                Serial.print("Random");
                            else
                                Serial.print(maxspeed);
                        }
                        Serial.println();
                        break;
                    }
                    case 'M':
                    {
                        // move
                        int nextLifterSpeed = sSettings.fMinimumPower+5;
                        int nextRotarySpeed = ROTARY_MINIMUM_POWER+5;
                        int nextIntervalMin = MOVEMODE_MAX_INTERVAL;
                        int nextIntervalMax = MOVEMODE_MAX_INTERVAL;
                        cmd++;
                        if (*cmd == ',')
                        {
                            // command lifter speed
                            nextLifterSpeed = strtolu(cmd+1, &cmd);
                            nextLifterSpeed = max(nextLifterSpeed, int(sSettings.fMinimumPower));
                        }
                        if (*cmd == ',')
                        {
                            // command speed
                            nextRotarySpeed = strtolu(cmd+1, &cmd);
                            nextRotarySpeed = max(nextRotarySpeed, ROTARY_MINIMUM_POWER+5);
                        }
                        if (*cmd == ',')
                        {
                            // command interval
                            nextIntervalMin = strtolu(cmd+1, &cmd);
                            nextIntervalMin = max(nextIntervalMin, 1); // minimum duration 1 second
                        }
                        if (*cmd == ',')
                        {
                            // command interval
                            nextIntervalMax = strtolu(cmd+1, &cmd);
                            nextIntervalMax = max(nextIntervalMax, nextIntervalMin+1); // minimum duration + 1 second
                        }
                        Serial.print("Move Continous: ");
                        Serial.print(" Lifter: ");
                        Serial.print(nextLifterSpeed);
                        Serial.print(" Rotary: ");
                        Serial.print(nextRotarySpeed);
                        Serial.print(" Min: ");
                        Serial.print(nextIntervalMin);
                        Serial.print(" Max: ");
                        Serial.println(nextIntervalMax);
                        break;
                    }
                    case 'W':
                    {
                        // seconds
                        bool randseconds = false;
                        int seconds = 0;
                        cmd++;
                        if (*cmd == 'R')
                        {
                            randseconds = true;
                            cmd++;
                        }
                        seconds = strtolu(cmd, &cmd);
                        seconds = min(max(seconds, 0), 600);
                        Serial.print("Wait Seconds: ");
                        if (randseconds)
                            Serial.println("Random");
                        else
                            Serial.println(seconds);
                        break;
                    }
                    case 'L':
                    {
                        // light kit sequence
                        int seq = 0;
                        bool randseq = false;
                        cmd++;
                        if (*cmd == 'R')
                        {
                            seq = true;
                            cmd++;
                        }
                        else
                        {
                            seq = strtolu(cmd, &cmd);
                        }
                        seq = min(max(seq, 0), 7);
                        Serial.print("Light Kit Sequence: ");
                        if (randseq)
                            Serial.println("Random");
                        else
                            Serial.println(seq);
                        break;
                    }
                    case 'H':
                    {
                        // return home
                        unsigned speed = sSettings.fMinimumPower; cmd++;
                        if (isdigit(*cmd))
                        {
                            speed = min(unsigned(max(strtolu(cmd, &cmd), (uint32_t)sSettings.fMinimumPower)), 100u);
                        }
                        Serial.print("Return Home: Speed: "); Serial.println(speed);
                        break;
                    }
                    default:
                        cmd = nullptr;
                        break;
                }
                if (cmd != nullptr && *cmd == ':')
                {
                    cmd++;
                }
                else if (cmd == nullptr || *cmd != '\0')
                {
                    Serial.println("Invalid");
                    Serial.println(cmd);
                    cmd = nullptr;
                    break;
                }
            }
            if (cmd != nullptr && *cmd == '\0')
            {
                Serial.print("Sequence [");
                Serial.print(storeseq);
                Serial.print("]: ");
                Serial.println(startcmd);
                sCmdBuffer[0] = '\0';
                if (sSettings.writeCommand(storeseq, startcmd))
                    Serial.println("Stored");
                else
                    Serial.println("Failed");
            }
        }
        else
        {
            Serial.println("Invalid");
        }
    }
    else
    {
        Serial.println(F("Invalid"));
    }
    if (needsUpdate)
    {
        updateSettings();
    }
    if (unchanged)
    {
        Serial.println(F("Unchanged"));
    }
}

// Top-level command dispatcher: routes ":P..." to processLifterCommand and
// "#P..." to processConfigureCommand. firstCommand is false for chained sub-commands.
bool processCommand(const char* cmd, bool firstCommand)
{
    sWaitNextSerialCommand = 0;
    if (*cmd == '\0')
        return true;
    if (!firstCommand)
    {
        if (cmd[0] != ':')
        {
            Serial.println("Invalid");
            return false;
        }
        if (!sSafetyManeuver)
        {
            Serial.println("Safety maneuver failed");
            return false;
        }
        return processLifterCommand(cmd+1);
    }
    switch (cmd[0])
    {
        case ':':
            if (cmd[1] == 'P')
            {
                // Check if followed by ID
                cmd += 2;
                if (isdigit(*cmd))
                {
                    uint32_t id = 0;
                    while (isdigit(*cmd))
                    {
                        id = id*10L + (*cmd-'0');
                        cmd++;
                    }
                    if (id != sSettings.fID)
                    {
                        // Command not meant for this ID
                        resetSerialCommand();
                        return true;
                    }
                }
                return processLifterCommand(cmd);
            }
            break;
        case '#':
            if (cmd[1] == 'P')
            {
                // Check if followed by ID
                cmd += 2;
                if (isdigit(*cmd))
                {
                    uint32_t id = 0;
                    while (isdigit(*cmd))
                    {
                        id = id*10L + (*cmd-'0');
                        cmd++;
                    }
                    if (id != sSettings.fID)
                    {
                        // Command not meant for this ID
                        resetSerialCommand();
                        return true;
                    }
                }
                processConfigureCommand(cmd);
                return true;
            }
            Serial.println("Invalid");
            break;
        default:
            Serial.println("Invalid");
            break;
    }
    return false;
}

///////////////////////////////////////////////////////////////////////////////

#ifdef USE_WIFI_WEB
static bool sUpdateSettings;

// Rescue page jog watchdog.  The WAPI handler stamps an expiry time each time
// a "jog" keepalive arrives.  loop() checks these every iteration and stops
// the relevant motor if the browser stops sending keepalives (e.g. connection
// dropped while a button was held).
static volatile unsigned long sLifterJogExpiry = 0;
static volatile unsigned long sRotaryJogExpiry = 0;

#include "WebPages.h"
#endif

///////////////////////////////////////////////////////////////////////////////

#ifdef USE_DROID_REMOTE
static bool sRemoteConnected;
static bool sRemoteConnecting;
static SMQAddress sRemoteAddress;
#endif

///////////////////////////////////////////////////////////////////////////////

#ifdef USE_WIFI
String getHostName()
{
    String mac = wifiAccess.getMacAddress();
    String hostName = mac.substring(mac.length()-5, mac.length());
    hostName.remove(2, 1);
    hostName = WIFI_AP_NAME+String("-")+hostName;
    return hostName;
}
#endif

//////////////////////////////////////////////////////

#ifdef USE_MENUS

static unsigned sBooleanValues[] = {
    false,
    true,
};

static const char* sYesNoStrings[] = {
    "NO",
    "YES"
};

static const char* sOnOffStrings[] = {
    "OFF",
    "ON"
};

#include "Screens.h"
#include "menus/CommandScreen.h"

#include "menus/CommandScreenHandlerSMQ.h"
CommandScreenHandlerSMQ sDisplay;

#include "menus/utility/ChoiceIntArrayScreen.h"
#include "menus/utility/ChoiceStrArrayScreen.h"
#include "menus/utility/UnsignedValueScreen.h"
#include "menus/utility/MenuScreen.h"

#include "menus/EraseSettingsScreen.h"
#include "menus/MainScreen.h"
#include "menus/RotatePeriscopeScreen.h"
#include "menus/SelectScreen.h"
#include "menus/SettingsScreen.h"
#include "menus/SettingsUpdatedScreen.h"
#ifdef USE_WIFI
#include "menus/WiFiModeScreen.h"
#endif

#endif

//////////////////////////////////////////////////////

static int sCurrentInputValue = -1;
static int readInputHeaders()
{
#ifndef USE_SDCARD
    bool inputA = sPinManager.digitalRead(PIN_INPUT_A);
    bool inputB = sPinManager.digitalRead(PIN_INPUT_B);
    bool inputC = sPinManager.digitalRead(PIN_INPUT_C);

    return (uint8_t(!inputA)<<2) |
           (uint8_t(!inputB)<<1) |
           (uint8_t(!inputC)<<0);
#else
    return 0;
#endif
}

//////////////////////////////////////////////////////

// One-time startup: loads settings, initializes hardware, starts WiFi/remote tasks,
// runs the safety maneuver, and prints "READY" when done.
void setup()
{
    REELTWO_READY();

    if (!preferences.begin("uppityspinner", false))
    {
        DEBUG_PRINTLN("Failed to init prefs");
    }
#ifdef USE_SDCARD
    mountSDFileSystem();

    delay(200);

    if (getSDCardMounted())
    {
        File binImage = SD.open("/UPPITY.BIN");
        if (binImage)
        {
            Serial.println("Firmware image found on SD card");
            Serial.print("Reflashing");
            Update.begin(binImage.size());
            uint32_t readSize = 0;
            while (binImage.available())
            {
                uint8_t buf = binImage.read();
                Update.write(&buf, 1);
                readSize++;
                if ((readSize % 102400) == 0)
                    Serial.print(".");
            }
            Serial.println("");
            binImage.close();
            // Delete the image file so we don't constantly reflash the box
            SD.remove("/UPPITY.BIN");
            if (Update.end(true))
            {
                Serial.println("Update Success: "); Serial.println(readSize);
                reboot();
            }
        }
    }
#endif
    sLifterParameters.load();
    // Restore the encoder count measured during a previous safety maneuver.
    // If valid (>= 1000 ticks), the safety maneuver will skip the full-revolution
    // measurement and only need to find home once.
    if (sLifterParameters.fRotaryEncoderCount >= 1000)
        sRotaryCircleEncoderCount = sLifterParameters.fRotaryEncoderCount;

#ifdef USE_WIFI
    wifiEnabled = wifiActive = preferences.getBool(PREFERENCE_WIFI_ENABLED, WIFI_ENABLED);
#endif
#ifdef USE_DROID_REMOTE
    remoteEnabled = remoteActive = preferences.getBool(PREFERENCE_REMOTE_ENABLED, REMOTE_ENABLED);
#endif

    PrintReelTwoInfo(Serial, "Uppity Spinner");

#ifdef EEPROM_SIZE
    if (!EEPROM.begin(EEPROM_SIZE))
    {
        Serial.println(F("Failed to initialize EEPROM"));
    }
    else
#endif
    if (sSettings.read())
    {
        Serial.println(F("Settings Restored"));
    }
    else
    {
        Serial.println(F("First Time Settings"));
        sSettings.write();
        if (sSettings.read())
        {
            Serial.println(F("Readback Success"));
        }
    }

    Wire.begin();
    SetupEvent::ready();

#ifdef COMMAND_SERIAL
    COMMAND_SERIAL.begin(sSettings.fBaudRate, SERIAL_8N1, PIN_RXD2, PIN_TXD2);
#endif

    lifter.disableMotors();
    Serial.print("R2UppitySpinner v"); Serial.println(FIRMWARE_VERSION);
    Serial.println("READY");

#ifdef USE_DROID_REMOTE
    if (remoteEnabled)
    {
    #ifdef USE_SMQ
        WiFi.mode(WIFI_MODE_APSTA);
        if (SMQ::init(preferences.getString(PREFERENCE_REMOTE_HOSTNAME, SMQ_HOSTNAME),
                        preferences.getString(PREFERENCE_REMOTE_SECRET, SMQ_SECRET)))
        {
            SMQLMK key;
            if (preferences.getBytes(PREFERENCE_REMOTE_LMK, &key, sizeof(SMQLMK)) == sizeof(SMQLMK))
            {
                SMQ::setLocalMasterKey(&key);
            }

            SMQAddressKey pairedHosts[SMQ_MAX_PAIRED_HOSTS];
            size_t pairedHostsSize = preferences.getBytesLength(PREFERENCE_REMOTE_PAIRED);
            unsigned numHosts = pairedHostsSize / sizeof(pairedHosts[0]);
            printf("numHosts: %d\n", numHosts);
            Serial.print("WiFi.macAddress() : "); Serial.println(WiFi.macAddress());
            if (numHosts != 0)
            {
                if (preferences.getBytes(PREFERENCE_REMOTE_PAIRED, pairedHosts, pairedHostsSize) == pairedHostsSize)
                {
                    SMQ::addPairedHosts(numHosts, pairedHosts);
                }
            }
            printf("Droid Remote Enabled %s:%s\n",
                preferences.getString(PREFERENCE_REMOTE_HOSTNAME, SMQ_HOSTNAME).c_str(),
                    preferences.getString(PREFERENCE_REMOTE_SECRET, SMQ_SECRET).c_str());
            SMQ::setHostPairingCallback([](SMQHost* host) {
                if (host == nullptr)
                {
                    printf("Pairing timed out\n");
                }
                else //if (host->hasTopic("LCD"))
                {
                    switch (SMQ::masterKeyExchange(&host->fLMK))
                    {
                        case -1:
                            printf("Pairing Stopped\n");
                            SMQ::stopPairing();
                            return;
                        case 1:
                            // Save new master key
                            SMQLMK lmk;
                            SMQ::getLocalMasterKey(&lmk);
                            printf("Saved new master key\n");
                            preferences.putBytes(PREFERENCE_REMOTE_LMK, &lmk, sizeof(lmk));
                            break;
                        case 0:
                            // We had the master key
                            break;
                    }
                    printf("Pairing: %s [%s]\n", host->getHostName().c_str(), host->fLMK.toString().c_str());
                    if (SMQ::addPairedHost(&host->fAddr, &host->fLMK))
                    {
                        SMQAddressKey pairedHosts[SMQ_MAX_PAIRED_HOSTS];
                        unsigned numHosts = SMQ::getPairedHostCount();
                        if (SMQ::getPairedHosts(pairedHosts, numHosts) == numHosts)
                        {
                            preferences.putBytes(PREFERENCE_REMOTE_PAIRED,
                                pairedHosts, numHosts*sizeof(pairedHosts[0]));
                            printf("Pairing Success\n");
                        }
                    }
                    printf("Pairing Stopped\n");
                    SMQ::stopPairing();
                }
            });

            SMQ::setHostDiscoveryCallback([](SMQHost* host) {
                if (host->hasTopic("LCD"))
                {
                    printf("Remote Discovered: %s\n", host->getHostName().c_str());
                }
            });

            SMQ::setHostLostCallback([](SMQHost* host) {
                printf("Lost: %s [%s] [%s]\n", host->getHostName().c_str(), host->getHostAddress().c_str(),
                    sRemoteAddress.toString().c_str());
                if (sRemoteAddress.equals(host->fAddr.fData))
                {
                    printf("DISABLING REMOTE\n");
                    sDisplay.setEnabled(false);
                }
            });
        }
        else
        {
            printf("Failed to activate Droid Remote\n");
        }
    #endif
    }
#endif
#ifdef USE_WIFI
    if (wifiEnabled)
    {
    #ifdef USE_WIFI_WEB
        // In preparation for adding WiFi settings web page
        wifiAccess.setNetworkCredentials(
            preferences.getString(PREFERENCE_WIFI_SSID, getHostName()),
            preferences.getString(PREFERENCE_WIFI_PASS, WIFI_AP_PASSPHRASE),
            preferences.getBool(PREFERENCE_WIFI_AP, WIFI_ACCESS_POINT),
            preferences.getBool(PREFERENCE_WIFI_ENABLED, WIFI_ENABLED));
    #ifdef USE_WIFI_MARCDUINO
        wifiMarcduinoReceiver.setEnabled(preferences.getBool(PREFERENCE_MARCWIFI_ENABLED, MARC_WIFI_ENABLED));
        if (wifiMarcduinoReceiver.enabled())
        {
            wifiMarcduinoReceiver.setCommandHandler([](const char* cmd) {
                printf("[JAWALITE] %s\n", cmd);
                executeCommand(cmd);
            });
        }
    #endif
        wifiAccess.notifyWifiConnected([](WifiAccess &wifi) {
        #ifdef PIN_STATUSLED
            statusLED.setMode(sCurrentMode = kWifiModeHome);
        #endif
            if (webServer.enabled())
            {
                Serial.print("Connect to http://"); Serial.println(wifi.getIPAddress());
            #ifdef USE_MDNS
                // No point in setting up mDNS if R2 is the access point
                if (!wifi.isSoftAP())
                {
                    String hostName = getHostName();
                    Serial.print("Host name: "); Serial.println(hostName);
                    if (!MDNS.begin(hostName.c_str()))
                    {
                        DEBUG_PRINTLN("Error setting up MDNS responder!");
                    }
                }
            }
        #endif
        });
    #endif
    #ifdef USE_OTA
        ArduinoOTA.onStart([]()
        {
            String type;
            if (ArduinoOTA.getCommand() == U_FLASH)
            {
                type = "sketch";
            }
            else // U_SPIFFS
            {
                type = "filesystem";
            }
            DEBUG_PRINTLN("OTA START");
        })
        .onEnd([]()
        {
            DEBUG_PRINTLN("OTA END");
        })
        .onProgress([](unsigned int progress, unsigned int total)
        {
            // float range = (float)progress / (float)total;
        })
        .onError([](ota_error_t error)
        {
            String desc;
            if (error == OTA_AUTH_ERROR) desc = "Auth Failed";
            else if (error == OTA_BEGIN_ERROR) desc = "Begin Failed";
            else if (error == OTA_CONNECT_ERROR) desc = "Connect Failed";
            else if (error == OTA_RECEIVE_ERROR) desc = "Receive Failed";
            else if (error == OTA_END_ERROR) desc = "End Failed";
            else desc = "Error: "+String(error);
            DEBUG_PRINTLN(desc);
        });
    #endif
    }
#endif

#ifdef USE_WIFI_WEB
    // For safety we will stop the motors if the web client is connected
    webServer.setConnect([]() {
        // Callback for each connected web client
        // DEBUG_PRINTLN("Hello");
    });
#endif
#if defined(USE_WIFI) || defined(USE_DROID_REMOTE)
    xTaskCreatePinnedToCore(
          eventLoopTask,
          "Events",
          5000,    // shrink stack size?
          NULL,
          1,
          &eventTask,
          0);
#endif

    if (sRCMode)
        sPPM.begin();

    sCurrentInputValue = readInputHeaders();
}

// FreeRTOS task (core 0): services WiFi, OTA updates, web server, and the droid remote display.
// Runs independently so it never blocks the motor control loop on core 1.
#if defined(USE_WIFI) || defined(USE_DROID_REMOTE) || defined(USE_LVGL_DISPLAY)
void eventLoopTask(void* )
{
    for (;;)
    {
        if (wifiActive)
        {
        #ifdef USE_OTA
            ArduinoOTA.handle();
        #endif
        #ifdef USE_WIFI_WEB
            webServer.handle();
        #endif
        }
        if (remoteActive)
        {
        #ifdef USE_SMQ
            SMQ::process();
        #endif
        }
    #ifdef USE_MENUS
        if (sRemoteConnecting)
        {
            sMainScreen.init();
            sDisplay.remoteActive();
            sRemoteConnecting = false;
        }
        sDisplay.process();
    #endif
        vTaskDelay(1);
    }
}
#endif

///////////////////////////////////////////////////////////////////////////////

#ifdef USE_SMQ
SMQMESSAGE(DIAL, {
    long newValue = msg.get_int32("new");
    long oldValue = msg.get_int32("old");
    sDisplay.remoteDialEvent(newValue, oldValue);
})

///////////////////////////////////////////////////////////////////////////////

SMQMESSAGE(BUTTON, {
    uint8_t id = msg.get_uint8("id");
    bool pressed = msg.get_uint8("pressed");
    bool repeat = msg.get_uint8("repeat");
    sDisplay.remoteButtonEvent(id, pressed, repeat);
})

///////////////////////////////////////////////////////////////////////////////

SMQMESSAGE(SELECT, {
    printf("\nREMOTE ACTIVE\n");
    sRemoteConnected = true;
    sRemoteConnecting = true;
    sRemoteAddress = SMQ::messageSender();
})
#endif

#ifdef USE_DROID_REMOTE
static void DisconnectRemote()
{
#ifdef USE_SMQ
    printf("DisconnectRemote : %d\n", sRemoteConnected);
    if (sRemoteConnected)
    {
        if (SMQ::sendTopic("EXIT", "Remote"))
        {
            SMQ::sendString("addr", SMQ::getAddress());
            SMQ::sendEnd();
            printf("SENT EXIT\n");
            sRemoteConnected = false;
            sDisplay.setEnabled(false);
        #ifdef STATUSLED_PIN
            statusLED.setMode(sCurrentMode = kNormalMode);
        #endif
        }
    }
#endif
}
#endif

///////////////////////////////////////////////////////

static void updateSettings()
{
    Serial.println(F("Write Settings"));
    sSettings.write();
    Serial.println(F("Updated"));
#ifdef USE_WIFI_WEB
    sUpdateSettings = false;
#endif
}

///////////////////////////////////////////////////////

static int RememberSerialChar(int ch)
{
    if (ch == 0x0A || ch == 0x0D)
    {
        if (sCopyPos < SizeOfArray(sCopyBuffer)-4)
        {
            sCopyBuffer[sCopyPos] = '\\';
            sCopyBuffer[sCopyPos+1] = '\\';
            sCopyBuffer[sCopyPos+2] = (ch == 0x0A) ? 'n' : 'r';
            sCopyBuffer[sCopyPos+3] = '\0';
        }
        sCopyPos = 0;
    }
    else if (ch == '\\')
    {
        if (sCopyPos < SizeOfArray(sCopyBuffer)-2)
        {
            sCopyBuffer[sCopyPos++] = ch;
            sCopyBuffer[sCopyPos++] = ch;
            sCopyBuffer[sCopyPos] = '\0';
        }
    }
    else if (sCopyPos < SizeOfArray(sCopyBuffer)-1)
    {
        if (isprint(ch))
        {
            sCopyBuffer[sCopyPos++] = ch;
            sCopyBuffer[sCopyPos] = '\0';
        }
        else if (sCopyPos < SizeOfArray(sCopyBuffer)-6)
        {
            int firstNibble = ch / 16;
            int secondNibble = ch % 16;
            sCopyBuffer[sCopyPos++] = '\\';
            sCopyBuffer[sCopyPos++] = '\\';
            sCopyBuffer[sCopyPos++] = 'x';
            sCopyBuffer[sCopyPos++] = (firstNibble < 10) ? '0' + firstNibble : 'A' + (firstNibble - 10);
            sCopyBuffer[sCopyPos++] = (secondNibble < 10) ? '0' + secondNibble : 'A' + (secondNibble - 10);
            sCopyBuffer[sCopyPos] = '\0';
        }
    }
    return ch;
}

///////////////////////////////////////////////////////

// Main loop (core 1): reads serial input, dispatches commands, runs motor animations,
// handles RC input, and manages the status LED.
void loop()
{
    // Heartbeat: print uptime every 60 s so an operator with a serial monitor
    // can confirm the droid is alive without needing to trigger a command.
    static uint32_t sLastHeartbeat;
    if (millis() - sLastHeartbeat >= 60000)
    {
        sLastHeartbeat = millis();
        Serial.print("ALIVE v"); Serial.print(FIRMWARE_VERSION);
        Serial.print(" uptime=");
        Serial.print(millis() / 1000);
        Serial.print("s safety=");
        Serial.print(sSafetyManeuver ? "OK" : (sSafetyManeuverFailed ? "FAIL" : "PENDING"));
        Serial.print(" height=");
        Serial.print(lifter.getLifterHeightPercent());
        Serial.print("% (");
        Serial.print(lifter.getLifterPosition());
        Serial.print("/");
        Serial.print(sSettings.getLifterDistance());
        Serial.print(" ticks) lim=[top=");
        Serial.print(lifter.lifterTopLimit() ? "1" : "0");
        Serial.print(" bot=");
        Serial.print(lifter.lifterBottomLimit() ? "1" : "0");
        Serial.print(" home=");
        Serial.print(lifter.rotaryHomeLimit() ? "1" : "0");
        Serial.print("] rot=");
        Serial.print(lifter.rotaryMotorCurrentPosition());
        Serial.print("deg motors=");
        Serial.print(lifter.motorsEnabled() ? "ON" : "off");
        Serial.print(" liftFault=");
        Serial.print(lifter.lifterMotorFault() ? "YES" : "no");
        Serial.print(" rotFault=");
        Serial.print(lifter.rotaryMotorFault() ? "YES" : "no");
        if (lifter.fLifterHolding)
        {
            Serial.print(" HOLD=");
            Serial.print((int)(lifter.fLifterTargetPos * 100));
            Serial.print("%");
        }
    #ifdef USE_WIFI_WEB
        Serial.print(" wifi=");
        if (WiFi.status() == WL_CONNECTED)
        {
            Serial.print(WiFi.RSSI());
            Serial.print("dBm");
        }
        else
        {
            Serial.print("AP(");
            Serial.print(WiFi.softAPgetStationNum());
            Serial.print("clients)");
        }
    #endif
        Serial.println();
    }
#ifdef USE_WIFI_WEB
    if (sUpdateSettings)
    {
        updateSettings();
    }
    // Safety override auto-expiry (60 seconds)
    if (sRescueOverrideExpiry && millis() > sRescueOverrideExpiry)
    {
        sRescueOverrideExpiry = 0;
        Serial.println("SAFETY OVERRIDE EXPIRED");
    }
    // Rescue jog watchdog: auto-stop motors if the browser stops sending keepalives.
    {
        unsigned long now = millis();
        if (sLifterJogExpiry && now > sLifterJogExpiry) {
            lifter.lifterMotorStop();
            sLifterJogExpiry = 0;
        }
        if (sRotaryJogExpiry && now > sRotaryJogExpiry) {
            lifter.rotaryMotorStop();
            sRotaryJogExpiry = 0;
        }
    }
#endif
    AnimatedEvent::process();
    lifter.animate();

    // Drift correction: if the lifter is holding a position and has drifted
    // beyond the configured threshold, re-seek to the target.
    // Gives up after 3 consecutive failures to prevent infinite retry loops
    // (e.g. when encoder readings are corrupted by EMI).
    {
        static uint32_t sLastDriftCheck;
        static int sDriftFailCount;
        if (lifter.fLifterHolding && lifter.fLifterTargetPos > 0
            && !lifter.fMoveMode && !sCalibrating
            && millis() - sLastDriftCheck > 5000)
        {
            sLastDriftCheck = millis();
            int driftPct = sLifterParameters.fDriftCorrectionPct;
            if (driftPct > 0)
            {
                long maxlen = sSettings.getLifterDistance();
                if (maxlen > 0)
                {
                    float currentPos = (float)lifter.getLifterPosition() / (float)maxlen;
                    float drift = abs(currentPos - lifter.fLifterTargetPos);
                    float threshold = driftPct / 100.0f;
                    if (drift > threshold)
                    {
                        if (sDriftFailCount >= 3)
                        {
                            Serial.println("DRIFT CORRECTION: giving up after 3 failures — clearing hold");
                            lifter.fLifterHolding = false;
                            lifter.fLifterTargetPos = -1;
                            sDriftFailCount = 0;
                        }
                        else
                        {
                            Serial.print("DRIFT CORRECTION: cur=");
                            Serial.print((int)(currentPos * 100));
                            Serial.print("% target=");
                            Serial.print((int)(lifter.fLifterTargetPos * 100));
                            Serial.print("% drift=");
                            Serial.print((int)(drift * 100));
                            Serial.print("% threshold=");
                            Serial.print(driftPct);
                            Serial.print("% attempt=");
                            Serial.println(sDriftFailCount + 1);
                            float savedTarget = lifter.fLifterTargetPos;
                            float posBefore = currentPos;
                            lifter.seekToPosition(savedTarget, sSettings.fMinimumPower / 100.0f);
                            float posAfter = (float)lifter.getLifterPosition() / (float)maxlen;
                            float afterDrift = abs(posAfter - savedTarget);
                            if (afterDrift > threshold)
                                sDriftFailCount++;
                            else
                                sDriftFailCount = 0;
                        }
                    }
                    else
                    {
                        sDriftFailCount = 0;  // within threshold — reset counter
                    }
                }
            }
        }
    }

    // append commands to command buffer
    if (Serial.available())
    {
        int ch = RememberSerialChar(Serial.read());
        if (ch == 0x0A || ch == 0x0D)
        {
            runSerialCommand();
        }
        else if (sPos < SizeOfArray(sBuffer)-1)
        {
            sBuffer[sPos++] = ch;
            sBuffer[sPos] = '\0';
        }
    }
#ifdef COMMAND_SERIAL
    // Serial commands are processed in the same buffer as the console serial
    if (COMMAND_SERIAL.available())
    {
        int ch = RememberSerialChar(COMMAND_SERIAL.read());
        if (sPos != 0 || (ch == ':' || ch == '#'))
        {
            // Reduce serial noise by ignoring anything that doesn't start with : or #
            printf("ch: %c [%d]\n", ch, ch);
            if (ch == 0x0A || ch == 0x0D)
            {
                runSerialCommand();
            }
            else if (sPos < SizeOfArray(sBuffer)-1)
            {
                sBuffer[sPos++] = ch;
                sBuffer[sPos] = '\0';
            }
        }
    }
#endif
    if (sProcessing && millis() > sWaitNextSerialCommand)
    {
        if (sCmdBuffer[0] == ':')
        {
            char* end = strchr(sCmdBuffer+1, ':');
            if (end != nullptr)
                *end = '\0';
            if (!processCommand(sCmdBuffer, !sCmdNextCommand))
            {
                // command invalid abort buffer
                DEBUG_PRINT("Unrecognized: "); DEBUG_PRINTLN(sCmdBuffer);
                sWaitNextSerialCommand = 0;
                end = nullptr;
            }
            if (end != nullptr)
            {
                *end = ':';
                strcpy(sCmdBuffer, end);
                if (sVerboseDebug)
                {
                    DEBUG_PRINT("REMAINS: ");
                    DEBUG_PRINTLN(sCmdBuffer);
                }
                sCmdNextCommand = true;
            }
            else
            {
                sCmdBuffer[0] = '\0';
                sCmdNextCommand = false;
            }
        }
        else if (sBuffer[0] == ':')
        {
            char* end = strchr(sBuffer+1, ':');
            if (end != nullptr)
                *end = '\0';
            if (!processCommand(sBuffer, !sNextCommand))
            {
                // command invalid abort buffer
                DEBUG_PRINT("Unrecognized: "); DEBUG_PRINTLN(sBuffer);
                sWaitNextSerialCommand = 0;
                end = nullptr;
            }
            if (end != nullptr)
            {
                *end = ':';
                strcpy(sBuffer, end);
                sPos = strlen(sBuffer);
                if (sVerboseDebug)
                {
                    DEBUG_PRINT("REMAINS: ");
                    DEBUG_PRINTLN(sBuffer);
                }
                sNextCommand = true;
            }
            else
            {
                resetSerialCommand();
                sBuffer[0] = '\0';
            }
        }
        else
        {
            processCommand(sBuffer, true);
            resetSerialCommand();
        }
    }
    static char sRotaryHomeStatus = -1;
    if (lifter.isIdle() && lifter.motorsEnabled() && !lifter.fLifterHolding
#ifdef USE_WIFI_WEB
        && !sLifterJogExpiry && !sRotaryJogExpiry
#endif
    )
    {
        DEBUG_PRINTLN("DISABLE MOTORS");
        lifter.disableMotors();
    }
    char rotaryIsHome = lifter.rotaryHomeLimit();
    if (sRotaryHomeStatus != rotaryIsHome)
    {
        sRotaryHomeStatus = rotaryIsHome;
        if (rotaryIsHome)
        {
            static uint32_t sLastHomePrint;
            if (millis() - sLastHomePrint > 2000)
            {
                sLastHomePrint = millis();
                Serial.println("HOME");
            }
        #ifdef PIN_STATUSLED
            statusLED.setMode(sCurrentMode + kNormalModeHome);
        #endif
        }
        else
        {
        #ifdef PIN_STATUSLED
            statusLED.setMode(sCurrentMode + kNormalModeAway);
        #endif
        }
    }
    if (sDigitalReadAll)
    {
        sDigitalReadAll = false;
        static CustomPinManager::DigitalInput sLastFlags;
        CustomPinManager::DigitalInput sNowFlags;
        sNowFlags = sPinManager.digitalReadAll();
        if (memcmp(&sLastFlags, &sNowFlags, sizeof(sLastFlags)) != 0)
        {
            int inputValue = readInputHeaders();
            if (inputValue != sCurrentInputValue)
            {
                sCurrentInputValue = inputValue;
                if (inputValue != 0)
                {
                    char buffer[10];
                    snprintf(buffer, sizeof(buffer), ":PS%d", inputValue);
                    printf("[INPUT] :PS%d\n", inputValue);
                    executeCommand(buffer);
                }
            }
            sLastFlags = sNowFlags;
        }
    }
    if (sRCMode)
    {
        int16_t channels[6];
        for (uint16_t i = 0; i < SizeOfArray(channels); i++)
        {
            channels[i] = ((int)sPPM.latestValidChannelValue(i+1, 0) - 1000) / 10;
        }
        // if (channels[3])
        // {
        //     channels[3]
        // }
        // printf("1:%4u 2:%4u 3:%4u 4:%4u 5:%4u 6:%4u\n",
        //     channels[0], channels[1], channels[2], channels[3], channels[4], channels[5]);
        static uint32_t sLastReading;
        if (sLastReading + 1000 < millis())
        {
            int height = channels[2];
            static int sLastRCHeight;
            if (height >= 0 && height <= 100 && abs(height - sLastRCHeight) > 10)
            {
                executeCommand(":PP%d,%d", height, 50);
                printf("NEW HEIGHT : %d\n", height);
                sLastRCHeight = height;
            }
            sLastReading = millis();
        }
    }
    // if (sPPM.decode())
    // {
    //     uint16_t channels[6];
    //     for (uint16_t i = 0; i < SizeOfArray(channels); i++)
    //     {
    //         channels[i] = sPPM.channel(i, 0, 1024, 512);
    //     }
    //     // Throttle
    //     if (channels[3] > 0)
    //     {

    //     }
    //     // printf("1:%4u 2:%4u 3:%4u 4:%4u 5:%4u 6:%4u\n",
    //     //     channels[0], channels[1], channels[2], channels[3], channels[4], channels[5]);
    // }

#ifdef USE_DEBUG
    if (sVerboseDebug)
    {
        static bool sLastTop;
        static bool sLastBot;
        static bool sLastRot;
        static long sLastLifter;
        static long sLastRotary;
        if (sLastTop != lifter.lifterTopLimit() ||
            sLastBot != lifter.lifterBottomLimit() ||
            sLastRot != lifter.rotaryHomeLimit() ||
            sLastLifter != lifter.getLifterPosition() ||
            sLastRotary != lifter.getRotaryPosition())
        {
            sLastTop = lifter.lifterTopLimit();
            sLastBot = lifter.lifterBottomLimit();
            sLastRot = lifter.rotaryHomeLimit();
            sLastLifter = lifter.getLifterPosition();
            sLastRotary = lifter.getRotaryPosition();
            printf("T: %d B: %d R: %d H: %d D: %d P: %d F: %d\n",
                int(sLastTop),
                int(sLastBot),
                int(sLastRot),
                int(sLastLifter/float(sSettings.getLifterDistance())*100.0),
                int(lifter.rotaryMotorCurrentPosition()),
                int(lifter.getRotaryPosition()),
                int(lifter.lifterMotorFault()));
        }
    }
#endif
}

#pragma GCC diagnostic pop
