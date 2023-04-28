#ifndef kill_switch_droid_pin_map_h
#define kill_switch_droid_pin_map_h

////////////////////////////////
// Body Controller Pinout
// ------------------
// This pinout is for a WROOM32
////////////////////////////////

#define UP_0  				0  //  // Unused Pin (Must be LOW to boot)
#define SERIAL_TX_PIN       1  //  // debug output
#define SERIAL1_TX_PIN		2  //  // (must be left floating or LOW to enter flashing mode)
#define SERIAL_RX_PIN       3  //  // high at boot
#define EXT_SW_OUT          4  //  // Unused Pin
#define RELAY_LED_PIN	    5  //  // Unused Pin (must be HIGH during boot)
#define UP_6                6  //  // connected to SPI flash
#define SERIAL1_RX_PIN      7  //  // connected to SPI flash
#define RELAY_BUTTON        8  //  // connected to SPI flash
#define UP_9                9  //  // connected to SPI flash
#define UP_10               10 //  // connected to SPI flash
#define UP_11               11 //  // connected to SPI flash
#define MISO_LORA			12 //  // Unused Pin (must be LOW during boot)
#define MOSI_LORA           13 //  // Unused Pin
#define SCK_LORA            14 //  // Unused Pin
#define NSS_LORA       		15 //  // Unused Pin (must be HIGH during boot)
#define UP_16               16 //  // Unused Pin
#define UP_17               17 //  // Unused Pin
#define UP_18	            18 //  // Unused Pin
#define STATUS_LED_PIN      19 //  // ESP32 Status NeoPixel Pin
#define LORA_LED_PIN        20 //  // LoRa Status NeoPixel Pin
#define I2C_SDA             21 //  // I2C SDA
#define I2C_SCL             22 //  // I2C SCL
#define UP_23	            23 //  // Unused Pin 
#define UP_24               24 //  // nused Pin
#define RELAY_CONTROL       25 //  // Pin connected to the relay output
#define DIO_LORA	        26 //  // Unused Pin
#define RESET_LORA	        27 //  // Unused Pin
#define UP_28               28 //  // Unused Pin
#define UP_29               29 //  // Unused Pin
#define UP_30               30 //  // Unused Pin
#define UP_31 	            31 //  // TUnused Pin
#define UP_32			  	32 //  // Unused Pin
#define UP_33	 	 	  	33 //  // Unused Pin
#define UP_34 	            34 //  // Unused Pin - input only (Can be used for Rx) // Use pull-down resistor
#define UP_35               35 //  // Unused Pin - input only (Can be used for Rx) // Use pull-down resistor
#define UP_36               36 //  // Unused Pin - input only (Can be used for Rx) // Use pull-down resistor
#define UP_28               37 //  // Unused Pin - input only (Can be used for Rx) // Use pull-down resistor
#define UP_28               38 //  // Unused Pin - input only (Can be used for Rx) // Use pull-down resistor
#define UP_39               39 //  // Unused Pin - input only (Can be used for Rx) // Use pull-down resistor

#endif
