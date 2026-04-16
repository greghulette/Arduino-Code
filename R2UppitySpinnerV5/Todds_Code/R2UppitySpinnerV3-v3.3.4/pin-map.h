////////////////////////////////////////////
// CONTROL BOARD PIN OUT
////////////////////////////////////////////
// This file tells the firmware which physical
// wire/pin on the ESP32 microcontroller connects
// to which part of the hardware.
//
// A "pin" is just a numbered electrical connection
// on the microcontroller chip. Think of it like
// the numbered terminals on a circuit board.
//
// PWM pins control motor speed by rapidly switching
// the voltage on and off (like a light dimmer switch).
// Two PWM pins per motor control forward vs. reverse.
//
// Encoder pins read the spinning disc that tracks
// how far each motor has moved. Two pins per encoder
// let the firmware figure out direction.
//
// Limit switch pins detect when a moving part has
// reached the physical end of its travel.
//
// DIAG pins report motor driver faults (overheating,
// short circuits, etc.).
//
// Only change these numbers if you are using a
// different PCB layout.
////////////////////////////////////////////

// --- Board variant: PCB with SD card reader ---
// This layout uses an I2C GPIO expander chip (PCF8574)
// for the limit switches and motor enable pins, freeing
// up direct ESP32 pins for the SD card interface.
#ifdef USE_SDCARD

// -----------------------------------------------
// LIFTER MOTOR — raises and lowers the periscope
// -----------------------------------------------

// Encoder tracks how far the lifter has traveled.
// Two pins together let the firmware determine
// both speed and direction of movement.
#define PIN_LIFTER_ENCODER_A   34   // Encoder channel A
#define PIN_LIFTER_ENCODER_B   35   // Encoder channel B

// Two PWM pins drive the lifter motor.
// PWM1 high + PWM2 low = motor goes up.
// PWM1 low + PWM2 high = motor goes down.
// Both low = motor coasts to a stop.
#define PIN_LIFTER_PWM1        32   // Motor direction / speed signal 1
#define PIN_LIFTER_PWM2        33   // Motor direction / speed signal 2

// The motor driver chip pulls this pin LOW if
// something is wrong (overheated, shorted, etc.).
#define PIN_LIFTER_DIAG        36   // Motor fault indicator (LOW = fault)

// -----------------------------------------------
// ROTARY MOTOR — spins the periscope head
// -----------------------------------------------

// Encoder tracks how far the rotary has turned.
#define PIN_ROTARY_ENCODER_A   27   // Encoder channel A
#define PIN_ROTARY_ENCODER_B   13   // Encoder channel B

// Two PWM pins drive the rotary motor (same logic as lifter).
#define PIN_ROTARY_PWM1        25   // Motor direction / speed signal 1
#define PIN_ROTARY_PWM2        26   // Motor direction / speed signal 2

// Fault indicator for the rotary motor driver.
#define PIN_ROTARY_DIAG        39   // Motor fault indicator (LOW = fault)

// -----------------------------------------------
// GENERAL I/O
// -----------------------------------------------

// RGB status LED that shows the current operating mode
// (home, moving, WiFi connected, error, etc.).
#define PIN_STATUSLED          5

// RC receiver input — reads a PPM signal from a
// hobby radio control receiver so R2 can be
// driven remotely with an RC transmitter.
#define PIN_PPMIN_RC           14

// Second serial port (UART2) used for Marcduino
// commands from other R2 electronics.
#define PIN_RXD2               16   // Receive data
#define PIN_TXD2               0    // Transmit data (not used)

// -----------------------------------------------
// SD CARD (SPI bus) — only present on this board variant
// -----------------------------------------------
// The SD card stores firmware update images.
// SPI is a standard 4-wire communication protocol.
#define PIN_SD_CS              4    // Chip Select — activates the SD card
#define PIN_SCK                18   // Clock signal
#define PIN_MISO               19   // Data from SD card to ESP32
#define PIN_MOSI               23   // Data from ESP32 to SD card

// -----------------------------------------------
// I2C BUS — used to talk to the GPIO expander chip
// -----------------------------------------------
// I2C is a 2-wire protocol that lets the ESP32
// control many extra pins through one small chip.
#define PIN_SDA                21   // I2C data line
#define PIN_SCL                22   // I2C clock line

// -----------------------------------------------
// I2C GPIO EXPANDER (PCF8574)
// -----------------------------------------------
// This chip adds 8 extra input/output pins via I2C.
// When any of those pins change state, it pulls
// PIN_GPIO_INTERRUPT low to alert the ESP32.
// Pins on this chip are numbered starting at 200
// so they can never be confused with real ESP32 pins.
#define PIN_GPIO_INTERRUPT     17   // Interrupt: fires when an expander pin changes
#define USE_I2C_GPIO_EXPANDER       // Tells the firmware to use the expander chip
#define GPIO_PIN_BASE           200 // Expander pins are numbered 200–207

// Expander pin assignments (SD card board variant):
#define PIN_ROTARY_LIMIT        GPIO_PIN_BASE+0  // Limit switch: rotary at home position
#define PIN_LIGHTKIT_A          GPIO_PIN_BASE+1  // Light kit mode select bit 2 (MSB)
#define PIN_LIGHTKIT_B          GPIO_PIN_BASE+2  // Light kit mode select bit 1
#define PIN_LIGHTKIT_C          GPIO_PIN_BASE+3  // Light kit mode select bit 0 (LSB)
#define PIN_MOTOR_EN            GPIO_PIN_BASE+4  // Motor enable (HIGH = motors powered on)
#define PIN_MOTOR_ENB           GPIO_PIN_BASE+5  // Motor enable inverted (LOW = motors powered on)
#define PIN_LIFTER_BOTLIMIT     GPIO_PIN_BASE+6  // Limit switch: lifter fully lowered
#define PIN_LIFTER_TOPLIMIT     GPIO_PIN_BASE+7  // Limit switch: lifter fully raised

#else

// --- Board variant: standard PCB (no SD card) ---
// Limit switches and motor enable pins connect
// directly to the ESP32 via the I2C GPIO expander,
// but the SD card SPI pins are freed up for
// three external trigger inputs instead.

// -----------------------------------------------
// LIFTER MOTOR
// -----------------------------------------------
#define PIN_LIFTER_ENCODER_A   34   // Encoder channel A
#define PIN_LIFTER_ENCODER_B   35   // Encoder channel B

#define PIN_LIFTER_PWM1        32   // Motor direction / speed signal 1
#define PIN_LIFTER_PWM2        33   // Motor direction / speed signal 2
#define PIN_LIFTER_DIAG        36   // Motor fault indicator (LOW = fault)

// These limit switches are wired directly to ESP32 pins
// (not through the expander) on this board variant.
#define PIN_LIFTER_TOPLIMIT    18   // Limit switch: lifter fully raised
#define PIN_LIFTER_BOTLIMIT    19   // Limit switch: lifter fully lowered

// -----------------------------------------------
// ROTARY MOTOR
// -----------------------------------------------
#define PIN_ROTARY_ENCODER_A   27   // Encoder channel A
#define PIN_ROTARY_ENCODER_B   13   // Encoder channel B

#define PIN_ROTARY_PWM1        25   // Motor direction / speed signal 1
#define PIN_ROTARY_PWM2        26   // Motor direction / speed signal 2
#define PIN_ROTARY_DIAG        39   // Motor fault indicator (LOW = fault)

// The rotary home limit switch is also direct on this variant.
#define PIN_ROTARY_LIMIT       23   // Limit switch: rotary at home position

// -----------------------------------------------
// GENERAL I/O
// -----------------------------------------------
#define PIN_STATUSLED          5    // RGB status LED
#define PIN_PPMIN_RC           14   // RC receiver PPM input
#define PIN_RXD2               16   // Serial receive (Marcduino commands)
#define PIN_TXD2               0    // Serial transmit (not used)

// -----------------------------------------------
// I2C BUS
// -----------------------------------------------
#define PIN_SDA                21   // I2C data line
#define PIN_SCL                22   // I2C clock line

// -----------------------------------------------
// I2C GPIO EXPANDER (PCF8574)
// -----------------------------------------------
#define PIN_GPIO_INTERRUPT     17   // Interrupt: fires when an expander pin changes
#define USE_I2C_GPIO_EXPANDER       // Tells the firmware to use the expander chip
#define GPIO_PIN_BASE           200 // Expander pins are numbered 200–207

// Expander pin assignments (standard board variant):
// On this variant the SD card pins are repurposed as
// three external trigger inputs (A, B, C). Grounding
// any combination of these three pins triggers a
// stored sequence (like pressing a button on R2).
#define PIN_INPUT_A             GPIO_PIN_BASE+0  // External trigger input A
#define PIN_LIGHTKIT_A          GPIO_PIN_BASE+1  // Light kit mode select bit 2 (MSB)
#define PIN_LIGHTKIT_B          GPIO_PIN_BASE+2  // Light kit mode select bit 1
#define PIN_LIGHTKIT_C          GPIO_PIN_BASE+3  // Light kit mode select bit 0 (LSB)
#define PIN_MOTOR_EN            GPIO_PIN_BASE+4  // Motor enable (HIGH = motors powered on)
#define PIN_MOTOR_ENB           GPIO_PIN_BASE+5  // Motor enable inverted (LOW = motors powered on)
#define PIN_INPUT_B             GPIO_PIN_BASE+6  // External trigger input B
#define PIN_INPUT_C             GPIO_PIN_BASE+7  // External trigger input C

#endif
