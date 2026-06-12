// ─────────────────────────────────────────────────────────────────────────────
// BL_TX_Test — minimal isolation test for the BC → Body-LED-Controller link.
//
// Flash this to the BC's ESP32 (board: "ESP32 Dev Module"). It does NOTHING but
// open hardware UART1 on the SAME pins the GUI uses (RX=GPIO16, TX=GPIO15) and
// spit "K99\r" out once a second at 9600 baud. No WiFi, no ESP-NOW, no SBUS, no
// SoftwareSerial — so it removes every variable the full GUI firmware adds.
//
// Watch the Mega's USB serial monitor (115200) while this runs:
//   • "Received Command over Serial2: K99" + magenta flash, or lights react
//       → the bare UART works → the GUI firmware is interfering; I fix the GUI.
//   • still nothing
//       → it's the ESP32 UART1-on-GPIO15 path or the board/Mega RX, not the GUI.
//
// It also echoes anything the Mega sends back (RX on GPIO16) to this sketch's
// own USB monitor, so you can confirm the RX half independently.
// ─────────────────────────────────────────────────────────────────────────────

HardwareSerial bl(1);   // ESP32 hardware UART1

#define BL_RX_PIN 16    // SERIAL_RX_BL_PIN  (Mega TX2 / pin 16 → here)
#define BL_TX_PIN 15    // SERIAL_TX_BL_PIN  (here → Mega RX2 / pin 17)

void setup() {
  Serial.begin(115200);
  delay(800);
  bl.begin(9600, SERIAL_8N1, BL_RX_PIN, BL_TX_PIN);   // RX=16, TX=15, like the GUI
  Serial.println("\n[BL_TX_Test] UART1 up: RX=GPIO16 TX=GPIO15 @9600");
  Serial.println("[BL_TX_Test] sending 'K99' every 1s — watch the Mega's monitor");
}

void loop() {
  bl.print("K99\r");
  Serial.println("[BL_TX_Test] -> sent K99 on GPIO15");

  // Echo whatever the Mega sends back (proves the RX half on GPIO16).
  while (bl.available()) Serial.write(bl.read());

  delay(1000);
}
