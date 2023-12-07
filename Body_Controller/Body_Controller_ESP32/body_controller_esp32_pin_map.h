#ifndef body_controller_esp32_pin_map.h
#define body_controller_esp32_pin_map.h

////////////////////////////////
// Body Controller Pinout
// ------------------
// This pinout is for a WROOM32
////////////////////////////////

#define UP_0  				0   // W // Unused Pin (Must be LOW to boot)
#define SERIAL_TX_PIN       1   // N // debug output
#define UP_2				2   // Y // Unused Pin(must be left floating or LOW to enter flashing mode)
#define SERIAL_RX_PIN       3   // N // high at boot
#define RESET_PIN_2560      4   // Y // Used to reset the ATMEGA
#define STAUS_LED_PIN		5   // Y // Status LED Data pin (must be HIGH during boot)
#define RESERVED_FLASH_6    6   // N // connected to SPI flash
#define RESERVED_FLASH_7    7   // N // connected to SPI flash
#define RESERVED_FLASH_8    8   // N // connected to SPI flash
#define RESERVED_FLASH_9    9   // N // connected to SPI flash
#define RESERVED_FLASH_10   10 // N // connected to SPI flash
#define RESERVED_FLASH_11   11 //  N // connected to SPI flash
// #define UP_12				12 // W // Unused Pin (must be LOW during boot)
#define SERIAL_TX_ST_PIN				12 // W // Unused Pin (must be LOW during boot)
#define SERIAL2_RX_PIN      13 // Y // AUX 2 Rx Pin
#define SERIAL2_TX_PIN      14 // Y // AUX 2 Tx Pin
#define SERIAL_TX_BL_PIN    15 // Y // ATMEGA Serial Tx pin(must be HIGH during boot)
#define SERIAL_RX_BL_PIN    16 // Y // ATMEGA Serial Tx pin
#define SERIAL_TX_RD_PIN    17 // Y // Roam a dome home Tx pin
#define SERIAL_TX_MP_PIN	18 // Y // MP3/HCR Serial Tx pin
#define SERIAL_TX_SB_PIN    19 // Y // Stealth Controller Serial Tx pin
// #define SERIAL_TX_ST_PIN    19 // Y // Stealth Controller Serial Tx pin
#define NONEXISTENT_PIN_20  20 // N // There is no GPIO20
#define I2C_SDA             21 // Y // I2C SDA
#define I2C_SCL             22 // Y // I2C SCL
#define UP_23	            23 // Y // Unused Pin 
#define NONEXISTENT_PIN_24  24 // N // There is no GPIO24
#define SERIAL1_RX_PIN      25 // Y //	AUX 1 Serial Rx pin
#define SERIAL1_TX_PIN	    26 // Y //  AUX 1 Serial Tx pin
#define UP_27	            27 // Y //  Unused Pin
#define NONEXISTENT_PIN_28  28 // N // There is no GPIO28
#define NONEXISTENT_PIN_29  29 // N // There is no GPIO29
#define NONEXISTENT_PIN_30  30 // N // There is no GPIO30
#define NONEXISTENT_PIN_31 	31 // N // There is no GPIO31
#define SERIAL_RX_MP_PIN	32 // Y // MP3/HCR Serial Rx pin
#define UP_33	 	 	  	33 // Y // Unused Pin
#define SERIAL_RX_RD_PIN    34 // Y // Roam a dome home Serial Rx pin- input only (Can be used for Rx) // Use pull-down resistor
// #define SERIAL_RX_ST_PIN    35 // Y // Stealth Controller Serial Rx pin - input only (Can be used for Rx) // Use pull-down resistor
#define SERIAL_RX_SB_PIN    35 // Y // Stealth Controller Serial Rx pin - input only (Can be used for Rx) // Use pull-down resistor
#define SERIAL_RX_ST_PIN               36 // Y // Unused Pin - input only (Can be used for Rx) // Use pull-down resistor
// #define UP_36               36 // Y // Unused Pin - input only (Can be used for Rx) // Use pull-down resistor
#define NONEXISTENT_PIN_37  37 // N // There is no GPIO37
#define NONEXISTENT_PIN_38  38 // N // There is no GPIO38
#define UP_39               39 // Y // Unused Pin - input only (Can be used for Rx) // Use pull-down resistor

#endif
