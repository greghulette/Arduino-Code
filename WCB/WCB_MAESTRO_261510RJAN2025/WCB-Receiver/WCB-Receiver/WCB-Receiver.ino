#include <esp_now.h>
#include <WiFi.h>

// Define Serial1 and Serial2 pins and baud rates
#define RXD1 21
#define TXD1 8
#define RXD2 7
#define TXD2 20
#define BAUD_RATE 57692

// Peer MAC address (replace with Transmitter ESP32 MAC)
uint8_t peerMAC[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// Circular buffer for Serial2 data
#define BUFFER_SIZE 512
uint8_t serialBuffer[BUFFER_SIZE];
volatile size_t writeIndex = 0;
volatile size_t readIndex = 0;

// Task handles
TaskHandle_t serialTaskHandle;
TaskHandle_t espNowTaskHandle;

// Circular buffer enqueue function
void IRAM_ATTR enqueueByte(uint8_t byte) {
  size_t nextIndex = (writeIndex + 1) % BUFFER_SIZE;
  if (nextIndex != readIndex) {
    serialBuffer[writeIndex] = byte;
    writeIndex = nextIndex;
  }
}

// Circular buffer dequeue function
bool dequeueByte(uint8_t &byte) {
  if (readIndex == writeIndex) {
    return false;  // Buffer empty
  }
  byte = serialBuffer[readIndex];
  readIndex = (readIndex + 1) % BUFFER_SIZE;
  return true;
}

// ESP-NOW send callback
void onSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Data sent!" : "Send failed!");
}

// ESP-NOW receive callback
void onDataReceive(const esp_now_recv_info *info, const uint8_t *data, int len) {
  Serial.print("Data received via ESP-NOW: ");
  for (int i = 0; i < len; i++) {
    Serial.printf("%02X ", data[i]);
    Serial2.write(data[i]);  // Forward to Serial2
  }
  Serial.println();
}

// Task to handle Serial2 data
void handleSerial(void *pvParameters) {
  for (;;) {
    if (Serial2.available()) {
      uint8_t byte = Serial2.read();
      enqueueByte(byte);
    }
    delay(1);  // Yield to other tasks
  }
}

// Task to handle ESP-NOW communication
void handleESPNow(void *pvParameters) {
  for (;;) {
    if (readIndex != writeIndex) {
      uint8_t data[BUFFER_SIZE];
      size_t count = 0;

      // Collect all available bytes from the buffer
      while (dequeueByte(data[count]) && count < sizeof(data)) {
        count++;
      }

      // Send data over ESP-NOW
      if (count > 0) {
        esp_now_send(peerMAC, data, count);
        Serial.println("Data sent via ESP-NOW.");
      }
    }
    delay(1);  // Yield to other tasks
  }
}

void setup() {
  Serial.begin(115200);
  Serial1.begin(BAUD_RATE, SERIAL_8N1, RXD1, TXD1);
  Serial2.begin(BAUD_RATE, SERIAL_8N1, RXD2, TXD2);

  // Initialize ESP-NOW
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed!");
    return;
  }
  esp_now_register_send_cb(onSent);
  esp_now_register_recv_cb(onDataReceive);  // Register the receive callback

  // Add peer
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, peerMAC, 6);
  peerInfo.channel = 0;  // Default channel
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer!");
    return;
  }

  // Create tasks
  xTaskCreatePinnedToCore(handleSerial, "SerialTask", 4096, NULL, 1, &serialTaskHandle, 0);
  xTaskCreatePinnedToCore(handleESPNow, "ESPNowTask", 4096, NULL, 1, &espNowTaskHandle, 1);

  Serial.println("Setup complete!");
}

void loop() {
  delay(100);
}
