/*
  BasicUsage.ino — drive real devices with WcbCmd

  Shows the ONE call per module a sketch makes to turn a WCB ";"-command into the
  device's native serial bytes. No WCB required — this is the standalone story.

  Wire each device to its OWN UART at the baud in the comment. This demo uses
  Serial1 / Serial2 as stand-ins; on real hardware give each device its own port
  (on ESP32 you can remap pins: Serial1.begin(baud, SERIAL_8N1, rxPin, txPin)).
*/
#include <WcbCmd.h>

// Stateful modules — create once, globally:
Mp3Codec mp3;      // SparkFun MP3 Trigger   (;A)
HcrCodec hcr;      // HCR vocalizer          (;H)

void setup() {
  Serial.begin(115200);          // debug console

  Serial1.begin(9600);           // MP3 Trigger: 9600 or 38400
  Serial2.begin(9600);           // Maestro: the baud set in the Maestro Control Center
  // (WLED wants 115200; HCR wants 9600 — give each its own UART in real hardware.)

  mp3.begin(Serial1, &Serial);   // bind the MP3 port (+ optional diag to the console)
  mp3.onFinished = [](const char* key) {          // optional: fires when a track ends
    Serial.printf("MP3 finished, ONFIN key = %s\n", key);
  };

  // ── Maestro — trigger script subroutine 1 on Maestro id 1  (";M11") ──
  WcbMaestro::emit(Serial2, "M11");

  // ── MP3 — set volume then play track 5  (";A,PLAY,5"); ONFIN key optional ──
  mp3.handle("PLAY,5,ONFIN,done");

  // ── WLED — recall preset 2  (";L,PS,2")  (drive the WLED node's 115200 UART) ──
  WcbWled::emit(Serial2, "PS,2");

  // ── HCR — named constants read better than the raw fn/chan/track ints ──
  hcr.emit(Serial1, HcrCodec::Stimulate, HcrCodec::Happy, 50);        // ;H,FN,4,0,50
  hcr.emit(Serial1, HcrCodec::PlayWAV,   HcrCodec::ChA,   3);         // play file 0003 on ch A
  hcr.emit(Serial1, HcrCodec::SetVolume, HcrCodec::AllChannels, 40);  // all channels -> 40
}

void loop() {
  mp3.poll();          // pump MP3 Trigger RX (drives onFinished / onError)
}
