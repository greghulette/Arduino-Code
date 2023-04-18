#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wimplicit-fallthrough"

///////////////////////////////////

#if __has_include("build_version.h")
#include "build_version.h"
#endif

#if __has_include("reeltwo_build_version.h")
#include "reeltwo_build_version.h"
#endif

///////////////////////////////////

#undef  USE_DEBUG                     // Define to enable debug diagnostic
#define USE_WIFI                      // Define to enable Wifi support
#define USE_WIFI_JAWALITE

/////////////////////////////////////////

#define PREFERENCE_WIFI_ENABLED         "wifi"
#define PREFERENCE_WIFI_SSID            "ssid"
#define PREFERENCE_WIFI_PASS            "pass"
#define PREFERENCE_WIFI_AP              "ap"
#define PREFERENCE_JAWAWIFI_ENABLED     "mwifi"
#define PREFERENCE_JAWAWIFI_SERIAL_PASS "mwifipass"
#define PREFERENCE_LORA_REGION          "region"
#define PREFERENCE_LORA_TX_POWER        "txpwr"
#define PREFERENCE_LORA_PA_BOOST        "boost"
#define PREFERENCE_LORA_FREQUENCY       "freq"
#define PREFERENCE_LORA_SPREAD_FACTOR   "spread"
#define PREFERENCE_LORA_BANDWIDTH       "band"
#define PREFERENCE_LORA_CODING_RATE     "rate"
#define PREFERENCE_LORA_CRC             "crc"
#define PREFERENCE_LORA_SECRET          "secret"
#define PREFERENCE_LORA_ECHO            "echo"
#define PREFERENCE_SERIAL_BAUDRATE      "brate"

/////////////////////////////////////////

#include "ReelTwo.h"
#include "core/SetupEvent.h"
#include "core/StringUtils.h"
#include "wifi/WifiAccess.h"
#include "wifi/WifiMarcduinoReceiver.h"

#include "LoRa.h"
#include "SSD1306Wire.h"

/////////////////////////////////////////

#include "pin-map.h"

#ifdef USE_WIFI
#define WIFI_ENABLED         true     // default enabled
// Set these to your desired WiFi credentials.
#define WIFI_AP_NAME         "DroidLa"
#define WIFI_AP_PASSPHRASE   "Astromech"
#define WIFI_ACCESS_POINT    true  /* true if access point: false if joining existing wifi */
#endif
#ifdef USE_WIFI_JAWALITE
#include "wifi/WifiMarcduinoReceiver.h"
#endif
#include <Preferences.h>

#define JAWA_WIFI_ENABLED               true
#define JAWA_WIFI_SERIAL_PASS           true

#define SERIAL_BAUDRATE_DEFAULT         9600

#define LORA_DEFAULT_SECRET             "Astromech"

/////////////////////////////////////////

#define LORA_DEFAULT_TX_POWER           14    // valid values: 2 - 14
#define LORA_DEFAULT_PA_BOOST           true  // valid values: true, false
#define LORA_DEFAULT_SPREAD_FACTOR      11    // valid values: 6, 7, 8, 9, 10, 11, 12
#define LORA_DEFAULT_BANDWIDTH          125E3 // valid values: 7800, 10400, 15600, 20800, 31250, 41700, 62500, 125000, 250000
#define LORA_DEFAULT_CODING_RATE        5     // valid values: 5 and 8
#define LORA_DEFAULT_CRC                false
#define LORA_DEFAULT_ECHO               false

/////////////////////////////////////////

#define SCREEN_ADDRESS                  0x3c
#define SCREEN_SLEEP_TIMER              30 /* seconds */

/////////////////////////////////////////

#define COMMAND_SERIAL Serial2
#define CONSOLE_BUFFER_SIZE     300

////////////////////////////////

#ifdef USE_WIFI
WifiAccess wifiAccess;
bool wifiEnabled;
bool remoteEnabled;
bool remoteActive;
TaskHandle_t eventTask;
bool otaInProgress;
#endif

#ifdef USE_WIFI_JAWALITE
WifiMarcduinoReceiver wifiMarcduinoReceiver(wifiAccess);
#endif

Preferences preferences;

////////////////////////////////

String rssi = "RSSI --";
String packSize = "--";
String packet;
unsigned int counter = 0;
bool receiveflag = false;
int16_t RssiDetection = 0;
uint32_t receiveDisplayTimeOut = 0;
uint32_t sDisplayLastActivity = 0;
SSD1306Wire sDisplay(SCREEN_ADDRESS, SDA_OLED_PIN, SCL_OLED_PIN, RST_OLED_PIN);
bool sDisplaySleeping;

////////////////////////////////////////

// Valid baud rates
static unsigned sSerialBaudRates[] = {
    2400,
    9600,
    19200,
    38400
};

////////////////////////////////////////

struct RadioSettings
{
    uint8_t txPower;
    bool paBoost;
    uint8_t region;
    long frequency;
    uint8_t spreadFactor;
    long bandwidth;
    uint8_t codingRate;
    bool crcCheck;
    String secret;
    uint32_t secretHash;
    bool echo;
    unsigned serialBaudRate;

    String getRegion()
    {
        switch (region)
        {
            case 0:
                return "RADIO DISABLED";
            case 1:
                return "Europe";
            case 2:
                return "North America";
            case 3:
                return "China";
            case 4:
                return "Korea";
            case 5:
                return "Japan";
            case 6:
                return "India";
            default:
                return "Unknown";
        }
    }

    void load()
    {
        txPower = preferences.getUChar(PREFERENCE_LORA_TX_POWER, LORA_DEFAULT_TX_POWER);
        paBoost = preferences.getBool(PREFERENCE_LORA_PA_BOOST, LORA_DEFAULT_PA_BOOST);
        region = preferences.getUChar(PREFERENCE_LORA_REGION, 0);
        frequency = preferences.getInt(PREFERENCE_LORA_FREQUENCY, 0) * 1E6L;
        spreadFactor = preferences.getUChar(PREFERENCE_LORA_SPREAD_FACTOR, LORA_DEFAULT_SPREAD_FACTOR);
        bandwidth = preferences.getInt(PREFERENCE_LORA_BANDWIDTH, LORA_DEFAULT_BANDWIDTH/100) * 1E2;
        codingRate = preferences.getUChar(PREFERENCE_LORA_CODING_RATE, LORA_DEFAULT_CODING_RATE);
        crcCheck = preferences.getBool(PREFERENCE_LORA_CRC, LORA_DEFAULT_CRC);
        secret = preferences.getString(PREFERENCE_LORA_SECRET, LORA_DEFAULT_SECRET);
        echo = preferences.getBool(PREFERENCE_LORA_ECHO, LORA_DEFAULT_ECHO);
        serialBaudRate = preferences.getUInt(PREFERENCE_SERIAL_BAUDRATE, SERIAL_BAUDRATE_DEFAULT);
        secretHash = WSID32(secret.c_str());
    }
};

RadioSettings radioSettings;

////////////////////////////////////////

void onReceive(int packetSize)//LoRa receiver interrupt service
{
    uint32_t hash;
    if (LoRa.readBytes((char*)&hash, sizeof(hash)) == sizeof(hash) &&
        hash == radioSettings.secretHash)
    {
        packet = "";
        packSize = String(packetSize,DEC);

        while (LoRa.available())
        {
            packet += (char) LoRa.read();
        }

        rssi = "RSSI: " + String(LoRa.packetRssi(), DEC);
        RssiDetection= abs(LoRa.packetRssi());
        digitalWrite(LED_PIN, HIGH);
        receiveflag = true;
        sDisplayLastActivity = millis();
        receiveDisplayTimeOut = sDisplayLastActivity + 4000;
    }
}

////////////////////////////////////////

void initDisplay()
{
    sDisplay.init();
    sDisplay.setFont(ArialMT_Plain_10);
}

void updateDisplay()
{
    if (sDisplaySleeping)
    {
        sDisplaySleeping = false;
        sDisplay.wakeup();
    }
    sDisplay.clear();
    if (wifiEnabled)
    {
        sDisplay.drawString(0, 0, "Droid LoRa WiFi Bridge");
    }
    else
    {
        sDisplay.drawString(0, 0, "Droid LoRa Receiver");
    }
    if (radioSettings.region == 0)
    {
        sDisplay.drawString(0, 10, "RADIO DISABLED");
    }
    if (packet != "")
    {
        sDisplay.drawString(0, 20, "Received:");
        sDisplay.drawString(0, 30, packet);
    }
    if (RssiDetection != 0)
    {
        sDisplay.drawString(0, 50, rssi);
    }
    sDisplay.display();
}

////////////////////////////////////////

void setup()
{
    REELTWO_READY();
#ifndef USE_DEBUG
    Serial.begin(DEFAULT_BAUD_RATE);
#endif

    if (!preferences.begin("dora", false))
    {
        DEBUG_PRINTLN("Failed to init prefs");
    }
    PrintReelTwoInfo(Serial, "Droid LoRa Bridge");

#ifdef USE_WIFI
    wifiEnabled = preferences.getBool(PREFERENCE_WIFI_ENABLED, WIFI_ENABLED);
    if (wifiEnabled)
    {
        wifiAccess.setNetworkCredentials(
            preferences.getString(PREFERENCE_WIFI_SSID, WIFI_AP_NAME),
            preferences.getString(PREFERENCE_WIFI_PASS, WIFI_AP_PASSPHRASE),
            preferences.getBool(PREFERENCE_WIFI_AP, WIFI_ACCESS_POINT),
            preferences.getBool(PREFERENCE_WIFI_ENABLED, WIFI_ENABLED));
    #ifdef USE_WIFI_JAWALITE
        wifiMarcduinoReceiver.setEnabled(preferences.getBool(PREFERENCE_JAWAWIFI_ENABLED, JAWA_WIFI_ENABLED));
        if (wifiMarcduinoReceiver.enabled())
        {
            wifiMarcduinoReceiver.setCommandHandler([](const char* cmd) {
                if (preferences.getBool(PREFERENCE_JAWAWIFI_SERIAL_PASS, JAWA_WIFI_SERIAL_PASS))
                {
                    // Command received from port 2000 forward to LoRa
                    sendCommand(cmd);
                }
            });
        }
    #endif
    }
    wifiAccess.notifyWifiConnected([](WifiAccess &wifi) {
    #ifdef STATUSLED_PIN
        statusLED.setMode(sCurrentMode = kWifiMode);
    #endif
        Serial.println();
        Serial.print("Connect to http://"); Serial.println(wifi.getIPAddress());
    });
#endif

    SPI.begin(SCK_PIN, MISO_PIN, MOSI_PIN, SS_PIN);
    LoRa.setPins(SS_PIN, LORA_RST_PIN, DIO0_PIN);

    radioSettings.load();
    if (radioSettings.region == 0)
    {
        Serial.println("LoRa region not set. Radio not started.");
    }
    else if (LoRa.begin(radioSettings.frequency, radioSettings.paBoost))
    {
        LoRa.setSpreadingFactor(radioSettings.spreadFactor);
        LoRa.setTxPower(radioSettings.txPower,
            (radioSettings.paBoost == true) ?
                RF_PACONFIG_PASELECT_PABOOST :
                RF_PACONFIG_PASELECT_RFO);
        LoRa.setSignalBandwidth(radioSettings.bandwidth);
        if (radioSettings.crcCheck)
        {
            LoRa.enableCrc();
        }
        else
        {
            LoRa.disableCrc();
        }
        Serial.println();
        Serial.println("Started LoRa Radio");
        showConfig();
    }
    else
    {
        Serial.println("Starting LoRa failed!");
    }

    // scan_i2c();
    initDisplay();
    updateDisplay();

    SetupEvent::ready();

    pinMode(LED_PIN, OUTPUT);
    LoRa.onReceive(onReceive);
    LoRa.receive();

#ifdef COMMAND_SERIAL
    COMMAND_SERIAL.begin(radioSettings.serialBaudRate, SERIAL_8N1, RXD2_PIN, TXD2_PIN);
#endif
}

void sendCommand(const char* cmd)
{
    LoRa.beginPacket();
    LoRa.write((uint8_t*)&radioSettings.secretHash, sizeof(radioSettings.secretHash));
    LoRa.print(cmd);
    LoRa.endPacket();
    LoRa.receive();
}

void reboot()
{
    Serial.println(F("Restarting..."));
    preferences.end();
    ESP.restart();
}

void showConfig()
{
    Serial.print("Region:         ");
    Serial.println(radioSettings.getRegion());
    Serial.print("Frequency:      "); Serial.print(radioSettings.frequency/1E6); Serial.println("MHz");
    Serial.print("Bandwidth:      "); Serial.println(radioSettings.bandwidth/1E3);
    Serial.print("Spread Factor:  "); Serial.println(radioSettings.spreadFactor);
    Serial.print("Transmit Power: "); Serial.println(radioSettings.txPower);
    Serial.print("Secret:         "); Serial.println(radioSettings.secret);
    Serial.print("WiFi:           "); Serial.println(wifiEnabled ? "Enabled" : "Disabled");
    Serial.print("Serial Speed:   "); Serial.println(radioSettings.serialBaudRate);
}

void processConfigureCommand(const char* cmd)
{
    if (startswith_P(cmd, F("#DLRESTART")))
    {
        reboot();
    }
    else if (startswith_P(cmd, F("#DLCONFIG")))
    {
        showConfig();
    }
    else if (startswith_P(cmd, F("#DLREGION")) && isdigit(*cmd))
    {
        long region = strtolu(cmd, &cmd);
        if (region != radioSettings.region)
        {
            switch (region)
            {
                case 0:
                    // Disable radio
                    break;
                case 1:
                    // Europe
                    preferences.putInt(PREFERENCE_LORA_FREQUENCY, 868);
                    Serial.println("Region changed to Europe. Valid frequencies are 867-869MHz");
                    break;
                case 2:
                    // North America
                    preferences.putInt(PREFERENCE_LORA_FREQUENCY, 915);
                    Serial.println("Region changed to North America. Valid frequencies are 902-928MHz");
                    break;
                case 3:
                    // China
                    preferences.putInt(PREFERENCE_LORA_FREQUENCY, 490);
                    Serial.println("Region changed to China. Valid frequencies are 470-510MHz");
                    break;
                case 4:
                    // Korea
                    preferences.putInt(PREFERENCE_LORA_FREQUENCY, 922);
                    Serial.println("Region changed to Korea. Valid frequencies are 920-925MHz");
                    break;
                case 5:
                    // Japan
                    preferences.putInt(PREFERENCE_LORA_FREQUENCY, 922);
                    Serial.println("Region changed to Japan. Valid frequencies are 920-925MHz");
                    break;
                case 6:
                    // India
                    preferences.putInt(PREFERENCE_LORA_FREQUENCY, 866);
                    Serial.println("Region changed to India. Valid frequencies are 865-867MHz");
                    break;
                default:
                    Serial.println("Invalid region. Valid values are:");
                    Serial.println("0: Disabled");
                    Serial.println("1: Europe");
                    Serial.println("2: North America");
                    Serial.println("3: China");
                    Serial.println("4: Korea");
                    Serial.println("5: Japan");
                    Serial.println("6: India");
                    return;
            }
            preferences.putUChar(PREFERENCE_LORA_REGION, uint8_t(region));
            reboot();
        }
        else
        {
            Serial.println("Unchanged");
        }
    }
    else if (startswith_P(cmd, F("#DLFREQUENCY")) && isdigit(*cmd))
    {
        long frequency = strtolu(cmd, &cmd);
        if (frequency >= 400 && frequency <= 999)
        {
            if (radioSettings.frequency != frequency * 1E6)
            {
                preferences.putInt(PREFERENCE_LORA_FREQUENCY, frequency);
                Serial.println("Frequency changed.");
                reboot();
            }
            else
            {
                Serial.println("Unchanged");
            }
        }
        else
        {
            Serial.println("Invalid range. Must be in the range 400-999MHz");
        }
    }
    else if (startswith_P(cmd, F("#DLTXPOWER")) && isdigit(*cmd))
    {
        long txPower = strtolu(cmd, &cmd);
        if (txPower >= 2 && txPower <= 17)
        {
            if (radioSettings.txPower != txPower)
            {
                preferences.putUChar(PREFERENCE_LORA_TX_POWER, uint8_t(txPower));
                Serial.println("TX power changed");
                reboot();
            }
            else
            {
                Serial.println("Unchanged");
            }
        }
        else
        {
            Serial.println("Invalid range. Transmit power must be in the range 2-17");
        }
    }
    else if (startswith_P(cmd, F("#DLPABOOST")) && isdigit(*cmd))
    {
        bool paBoost = (strtolu(cmd, &cmd) == 1);
        if (radioSettings.paBoost != paBoost)
        {
            preferences.putBool(PREFERENCE_LORA_PA_BOOST, paBoost);
            Serial.println("TX power changed");
        }
        else
        {
                Serial.println("Unchanged");
        }
    }
    else if (startswith_P(cmd, F("#DLSPREAD")) && isdigit(*cmd))
    {
        long spreadFactor = strtolu(cmd, &cmd);
        if (spreadFactor >= 6 && spreadFactor <= 7)
        {
            if (radioSettings.spreadFactor != spreadFactor)
            {
                preferences.putUChar(PREFERENCE_LORA_SPREAD_FACTOR, uint8_t(spreadFactor));
                Serial.println("Spreading factor changed");
                reboot();
            }
            else
            {
                Serial.println("Unchanged");
            }
        }
        else
        {
            Serial.println("Invalid range. Spreading factor must be in the range 6-7");
        }
    }
    else if (startswith_P(cmd, F("#DLBANDWIDTH")) && isdigit(*cmd))
    {
        long bandwidth = strtolu(cmd, &cmd);
        switch (bandwidth)
        {
            case 7800:
            case 10400:
            case 15600:
            case 20800:
            case 31250:
            case 41700:
            case 62500:
            case 125000:
            case 250000:
                if (radioSettings.bandwidth != bandwidth)
                {
                    preferences.putInt(PREFERENCE_LORA_BANDWIDTH, uint8_t(bandwidth));
                    Serial.println("Bandwidth changed");
                    reboot();
                }
                else
                {
                    Serial.println("Unchanged");
                }
                break;
            default:
                Serial.println("Invalid value. Bandwidth must be one of:");
                Serial.println("7800, 10400, 15600, 20800, 31250, 41700, 62500, 125000, 250000");
                break;
        }
    }
    else if (startswith_P(cmd, F("#DLCODERATE")) && isdigit(*cmd))
    {
        long codingRate = strtolu(cmd, &cmd);
        if (codingRate == 5 || codingRate == 8)
        {
            if (radioSettings.codingRate != codingRate)
            {
                preferences.putUChar(PREFERENCE_LORA_CODING_RATE, uint8_t(codingRate));
                Serial.println("Coding rate changed");
                reboot();
            }
            else
            {
                Serial.println("Unchanged");
            }
        }
        else
        {
            Serial.println("Invalid range. Coding rate must be either 5 or 8 (representing 4/5 or 4/8)");
        }
    }
    else if (startswith_P(cmd, F("#DLCRC")) && isdigit(*cmd))
    {
        bool crcCheck = (strtolu(cmd, &cmd) == 1);
        if (radioSettings.crcCheck != crcCheck)
        {
            preferences.putBool(PREFERENCE_LORA_CRC, crcCheck);
            Serial.println("CRC changed");
            reboot();
        }
        else
        {
            Serial.println("Unchanged");
        }
    }
    else if (startswith_P(cmd, F("#DLECHO")) && isdigit(*cmd))
    {
        bool echo = (strtolu(cmd, &cmd) == 1);
        if (radioSettings.echo != echo)
        {
            preferences.putBool(PREFERENCE_LORA_ECHO, echo);
            Serial.println("Echo changed");
            reboot();
        }
        else
        {
            Serial.println("Unchanged");
        }
    }
    else if (startswith_P(cmd, F("#DLSECRET")))
    {
        String secret = String(cmd);
        if (radioSettings.secret != secret)
        {
            preferences.putString(PREFERENCE_LORA_SECRET, secret);
            Serial.println("Secret changed");
            reboot();
        }
        else
        {
            Serial.println("Unchanged");
        }
    }
    else if (startswith_P(cmd, F("#DLSERIALBAUD")) && isdigit(*cmd))
    {
        uint32_t baudrate = strtolu(cmd, &cmd);
        for (unsigned i = 0; i < SizeOfArray(sSerialBaudRates); i++)
        {
            if (baudrate == sSerialBaudRates[i])
            {
                preferences.putUInt(PREFERENCE_SERIAL_BAUDRATE, baudrate);
                Serial.println("Baud rate changed");
                reboot();
            }
            else
            {
                Serial.println("Not supported");
            }
        }
    }
#ifdef USE_WIFI
    else if (startswith_P(cmd, F("#DLWIFI")) && isdigit(*cmd))
    {
        bool wifiSetting = (strtolu(cmd, &cmd) == 1);
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
            Serial.println("Unchanged");
        }
    }
#endif
    else
    {
        Serial.println("Invalid. Valid commands are:");
        Serial.println("#DLCONFIG");
        Serial.println("#DLRESTART");
        Serial.println("#DLREGION");
        Serial.println("#DLFREQUENCY");
        Serial.println("#DLTXPOWER");
        Serial.println("#DLPABOOST");
        Serial.println("#DLSPREAD");
        Serial.println("#DLBANDWIDTH");
        Serial.println("#DLCODERATE");
        Serial.println("#DLCRC");
        Serial.println("#DLSERIALBAUD");
        Serial.println("#DLWIFI");
    }
}

static unsigned sPos;
static char sBuffer[CONSOLE_BUFFER_SIZE];

static void processSerial(Stream &stream)
{
    if (stream.available())
    {
        int ch = stream.read();
        if (ch == 0x0A || ch == 0x0D)
        {
            if (strncmp(sBuffer, "#DL", 3) == 0)
            {
                processConfigureCommand(sBuffer);
            }
            else
            {
                // Command received on Serial forward to LoRa
                sendCommand(sBuffer);
            }
            sPos = 0;
            sBuffer[sPos] = '\0';
        }
        else if (sPos < SizeOfArray(sBuffer)-1)
        {
            sBuffer[sPos++] = ch;
            sBuffer[sPos] = '\0';
        }
    }
}

void loop()
{
    AnimatedEvent::process();
    if (receiveflag)
    {
        updateDisplay();
        Serial.print("RECEIVED: ");
        Serial.println(packet);
        receiveflag = false;
        if (radioSettings.echo)
        {
            sendCommand(packet.c_str());
        }
        LoRa.receive();
    }
    if (receiveDisplayTimeOut != 0 && receiveDisplayTimeOut < millis())
    {
        packet = "";
        RssiDetection = 0;
        updateDisplay();
        digitalWrite(LED_PIN, LOW);
        receiveDisplayTimeOut = 0;
    }
    // Check if we should blank the screen
    if (!sDisplaySleeping && SCREEN_SLEEP_TIMER !=0 &&
        sDisplayLastActivity + SCREEN_SLEEP_TIMER * 1000L < millis())
    {
        sDisplay.sleep();
        sDisplaySleeping = true;
    }
    processSerial(Serial);
#ifdef COMMAND_SERIAL
    // Serial commands are processed in the same buffer as the console serial
    processSerial(COMMAND_SERIAL);
#endif
}

#pragma GCC diagnostic pop
