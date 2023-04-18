#include <SPI.h>
#include <LoRa.h>
#include "ds3231.h"


String outgoing;
  String LoRaOutgoing;

#define RELAY_STATE 15  
  byte msgCount = 0;            // count of outgoing messages
  byte localAddress = 0xBC;     // address of this device
  byte destination = 0xFF;      // destination to send to
  long lastSendTime = 0;        // last send time
  int interval = 2000;          // interval between sends
  const int csPin = 18;          // LoRa radio chip select
  const int resetPin = 14;       // LoRa radio reset
  const int irqPin = 26;         // change for your board; must be a hardware interrupt pin

  int LoRaRSSI;

void setup() {
  Serial.begin(115200);
  while (!Serial);

  pinMode(RELAY_STATE, OUTPUT);
  digitalWrite(RELAY_STATE, LOW);
  delay(500);
  digitalWrite(RELAY_STATE, HIGH);
  delay(500);
  digitalWrite(RELAY_STATE, LOW);


  Serial.println("LoRa Receiver");
  LoRa.setPins(18, 14, 26);// set CS, reset, IRQ pin

  if (!LoRa.begin(915E6)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }
}

void loop() {
    onReceive(LoRa.parsePacket());
  // delay(1000);



}
void sendMessage(String outgoing) {
  LoRa.beginPacket();                   // start packet
  LoRa.write(destination);              // add destination address
  LoRa.write(localAddress);             // add sender address
  LoRa.write(msgCount);                 // add message ID
  LoRa.write(outgoing.length());        // add payload length
  LoRa.print(outgoing);                 // add payload
  LoRa.endPacket();                     // finish packet and send it
  msgCount++;                           // increment message ID
  Serial.println("Message Sent");
}
void onReceive(int packetSize) {
  if (packetSize == 0) return;          // if there's no packet, return

  // read packet header bytes:
  int recipient = LoRa.read();          // recipient address
  byte sender = LoRa.read();            // sender address
  byte incomingMsgId = LoRa.read();     // incoming msg ID
  byte incomingLength = LoRa.read();    // incoming msg length

  String incoming = "";

  while (LoRa.available()) {
    incoming += (char)LoRa.read();
  }

  if (incomingLength != incoming.length()) {   // check length for error
    Serial.println("error: message length does not match length");
    return;                             // skip rest of function
  }

  // if the recipient isn't this device or broadcast,
  if (recipient != localAddress && recipient != 0xFF) {
    Serial.println("This message is not for me.");
    return;                             // skip rest of function
  }

  // if message is for this device, or broadcast, print details:
  Serial.println("Received from: 0x" + String(sender, HEX));
  Serial.println("Sent to: 0x" + String(recipient, HEX));
  Serial.println("Message ID: " + String(incomingMsgId));
  Serial.println("Message length: " + String(incomingLength));
  Serial.println("Message: " + incoming);
  Serial.println("RSSI: " + String(LoRa.packetRssi()));
  Serial.println("Snr: " + String(LoRa.packetSnr()));
  Serial.println();
  // sendMessage("Message Revieved");
}




