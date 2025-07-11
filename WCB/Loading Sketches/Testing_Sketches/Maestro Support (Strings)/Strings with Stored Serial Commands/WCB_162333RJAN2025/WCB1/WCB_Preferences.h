#include <sys/_stdint.h>
#pragma once
#ifndef WCB_Preferences.h

#define WCB_Preferences.h

#endif

//////////////////////////////////////////////////////////////////////
///*****          Preferences/Items to change                 *****///
//////////////////////////////////////////////////////////////////////

  // Board Revision
  // #define HWVERSION_1                               // Run 1 and 2, with LilyGo T7 v1.5 Board with version 1.0 on the label
  // #define HWVERSION_2_1                             // WCB  with version 2.1 on the label
  // #define HWVERSION_2_3                             // WCB with version 2.3  on the label
  #define HWVERSION_2_4                             // WCB with version  2.4 on the label


  #define MAESTRO_ENABLED                            // Comment this out if not using the Maestro Servo Controller

  #ifdef MAESTRO_ENABLED
    #define MAESTRO_CONTROLLER Serial1              // The serial port that the device sending the commands to the Maestro is connected to.
    #define MAESTRO_CONNECTION Serial2              // The serial port that the Maestro 6/12/18 is plugging into 
    // #define MESTRO_CONTROLLER_POLOLU_LIBRARY        // Uncomment this out for Kyber/Microcontroller using the Maestro Library
    #define MAESTRO_CONTROLLER_STRING               // Uncomment this out for devices sending serial strings (i.e.Stealth/Shadow/Padawan,...)
    #define MAESTRO_QTY 3                           // Number of Maestros being used (1-7)
    uint8_t localMaestroID = 0x01;                  // The Maestro ID that is connected to the local WCB
  #endif

    // Uncomment only the board that you are loading this sketch onto. 
  #define WCB1 
  // #define WCB2 
  // #define WCB3 
  // #define WCB4 
  // #define WCB5 
  // #define WCB6 
  // #define WCB7 
  // #define WCB8 
  // #define WCB9

  // Change to match the amount of WCB's that you are using.  This alleviates initialing ESP-NOW peers if they are not used.
  int Default_WCB_Quantity = 3;          

  // ESPNOW Password - This must be the same across all devices and unique to your droid/setup. (PLEASE CHANGE THIS)
  String DEFAULT_ESPNOWPASSWORD = "ESPNOW_xxxxxxxxx";

  // Default Serial Baud Rates   ******THESE ARE ONLY CORRECT UNTIL YOU CHANGE THEM VIA THE COMMAND LINE.  ONCE CHANGED, THEY MAY NOT MATCH THIS NUMBER.
  // The correct baud rates will be shown on the serial console on bootup.
  #ifdef MESTRO_CONTROLLER_POLOLU_LIBRARY           // Checks to see if you are recieving commands from the Pololu Library 
    #define SERIAL1_DEFAULT_BAUD_RATE 57692         // Set the baud rate to receive commands from the Pololu Library (Using 57692 per Kyber recommendations)
  #else
    #define SERIAL1_DEFAULT_BAUD_RATE 9600          // Default Baud Rate for Serial 1 if not using Maestro
  #endif
  #ifdef MAESTRO_ENABLED                            // Checks to see if the Maestro is enabled and connected to Serial 2
    #define SERIAL2_DEFAULT_BAUD_RATE 57692         // Set the baud rate to transmit commands to the Maestro (Using 57692 per Kyber recommendations)
  #else
    #define SERIAL2_DEFAULT_BAUD_RATE 9600          // Default Baud Rate for Serial 2 if not using Maestro
  #endif 
  #define SERIAL3_DEFAULT_BAUD_RATE 9600            //Should be lower than 57600, I'd recommend 9600 or lower for best reliability
  #define SERIAL4_DEFAULT_BAUD_RATE 9600            //Should be lower than 57600, I'd recommend 9600 or lower for best reliability
  #define SERIAL5_DEFAULT_BAUD_RATE 9600            //Should be lower than 57600, I'd recommend 9600 or lower for best reliability

  #define SERIAL1_BROADCAST_DEFAULT false
  #define SERIAL2_BROADCAST_DEFAULT true
  #define SERIAL3_BROADCAST_DEFAULT true
  #define SERIAL4_BROADCAST_DEFAULT true
  #define SERIAL5_BROADCAST_DEFAULT true
  
  // Mac Address Customization: MUST BE THE SAME ON ALL BOARDS - Allows you to easily change 2nd and 3rd octects of the mac addresses so that there are more unique addresses out there.  
  // Can be any 2-digit hexidecimal number. Each digit can be 0-9 or A-F.  Example is "0x1A", or "0x56" or "0xB2"

  const uint8_t umac_oct2 = 0x01;     // unique MAC address for the 2nd Octet
  const uint8_t umac_oct3 = 0x00;     // unique MAC address for the 3rd Octet

  const char* DELIMITER = "*";                // The character that separates the simultaneous commmands that were sent (Tested: & * ^ . - )
  const char* STORED_COMMAND_DELIMITER = "^";                // The character that separates the simultaneous commmands that were sent (Tested: & * ^ . - )

  char CommandCharacter = ';';                // The character that triggers unicast commands
  char LocalFunctionIdentifier = '?';

