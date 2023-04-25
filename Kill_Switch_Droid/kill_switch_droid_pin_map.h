#ifndef kill_switch_droid_pin_map_h
#define kill_switch_droid_pin_map_h

////////////////////////////////
// Body Controller Pinout
// ------------------
// This pinout is for a WROOM32
////////////////////////////////

#define UP_0  				      0   // W // Unused Pin (Must be LOW to boot)
#define SERIAL_TX_PIN       1   // N // debug output
#define SERIAL_TX_HEADER		2   // Y // (must be left floating or LOW to enter flashing mode)
#define SERIAL_RX_PIN       3   // N // high at boot
#define EXT_SW_OUT          4   // Y // Unused Pin
#define RELAY_LED_PIN	5   // Y // Unused Pin (must be HIGH during boot)
#define UP_6                6   // N // connected to SPI flash
#define SERIAL_RX_HEADER    7   // N // connected to SPI flash
#define RELAY_BUTTON        8   // N // connected to SPI flash
#define UP_9                9   // N // connected to SPI flash
#define UP_10               10 // N // connected to SPI flash
#define UP_11               11 //  N // connected to SPI flash
#define MISO_LORA				    12 // W // Unused Pin (must be LOW during boot)
#define MOSI_LORA           13 // Y // Unused Pin
#define SCK_LORA            14 // Y // Unused Pin
#define NSS_LORA       		  15 // Y // Unused Pin (must be HIGH during boot)
#define UP_16               16 // Y // Unused Pin
#define UP_17               17 // Y // Unused Pin
#define UP_18	              18 // Y // Unused Pin
#define STATUS_LED_PIN          19 // Y // Unused Pin
#define LORA_LED_PIN            20 // N // There is no GPIO20
#define I2C_SDA             21 // Y // I2C SDA
#define I2C_SCL             22 // Y // I2C SCL
#define UP_23	              23 // Y // Unused Pin 
#define NONEXISTENT_PIN_24  24 // N // There is no GPIO20
#define RELAY_CONTROL       25 // Y //	Unused Pin
#define DIO_LORA	          26 // Y //  Unused Pin
#define RESET_LORA	        27 // Y //  Unused Pin
#define NONEXISTENT_PIN_28  28 // N // There is no GPIO28
#define NONEXISTENT_PIN_29  29 // N // There is no GPIO29
#define NONEXISTENT_PIN_30  30 // N // There is no GPIO30
#define NONEXISTENT_PIN_31 	31 // N // There is no GPIO31
#define UP_32			  	      32 // Y // Unused Pin
#define UP_33	 	 	  	      33 // Y // Unused Pin
#define UP_34 	            34 // Y // Unused Pin - input only (Can be used for Rx) // Use pull-down resistor
#define UP_35               35 // Y // Unused Pin - input only (Can be used for Rx) // Use pull-down resistor
#define UP_36               36 // Y // Unused Pin - input only (Can be used for Rx) // Use pull-down resistor
#define NONEXISTENT_PIN_37  37 // N // There is no GPIO37
#define NONEXISTENT_PIN_38  38 // N // There is no GPIO38
#define UP_39               39 // Y // Unused Pin - input only (Can be used for Rx) // Use pull-down resistor

#endif
