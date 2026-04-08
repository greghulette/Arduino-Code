
#pragma once
/**
 * ETM_Droid.h — Ensured Transmission Mode for the Droid ESP-NOW network
 *
 * Provides reliable message delivery with sequence numbers, ACK tracking,
 * automatic retries, and heartbeat-based online/offline detection.
 *
 * USAGE — in each controller sketch:
 *
 *   // 1. Define your board index BEFORE including this header
 *   #define ETM_MY_BOARD_INDEX  ETM_BOARD_BS
 *
 *   // 2. Include
 *   #include <ETM_Droid.h>
 *
 *   // 3. In setup(), after esp_now_init() and peer registration:
 *   etmInit(ESPNOWPASSWORD.c_str(), ETM_MY_BOARD_INDEX);
 *
 *   // 4. In loop() (call every iteration):
 *   etmProcess();
 *
 * NOTE: All boards must be flashed with the same struct definition simultaneously.
 *       This is a custom version of the library — do not replace with stock ETM_Droid.
 */

#include <Arduino.h>
#include <esp_now.h>

// ---------------------------------------------------------------------------
// Board Index Definitions  (used as indices into MAC/status tables)
// ---------------------------------------------------------------------------
#define ETM_BOARD_DG   0   // Droid Gateway
#define ETM_BOARD_BC   1   // Body Controller
#define ETM_BOARD_BS   2   // Body Servo Controller
#define ETM_BOARD_DC   3   // Dome Controller
#define ETM_BOARD_DP   4   // Dome Plate Controller
#define ETM_BOARD_HP   5   // HP Controller
#define ETM_NUM_BOARDS 6

// ---------------------------------------------------------------------------
// Packet Type Constants
// ---------------------------------------------------------------------------
#define PACKET_TYPE_COMMAND   0
#define PACKET_TYPE_ACK       1
#define PACKET_TYPE_HEARTBEAT 2

// ---------------------------------------------------------------------------
// ETM Tuning Parameters  (override with #define before #include if needed)
// ---------------------------------------------------------------------------
#ifndef ETM_PENDING_MAX
  #define ETM_PENDING_MAX        8    // Max simultaneous in-flight messages
#endif
#ifndef ETM_RETRY_MAX
  #define ETM_RETRY_MAX          3    // Max retries per destination board
#endif
#ifndef ETM_TIMEOUT_MS
  #define ETM_TIMEOUT_MS         500  // ms to wait for ACK before retry
#endif
#ifndef ETM_HEARTBEAT_SEC
  #define ETM_HEARTBEAT_SEC      10   // Normal heartbeat interval (seconds)
#endif
#ifndef ETM_BOOT_HEARTBEAT_SEC
  #define ETM_BOOT_HEARTBEAT_SEC 2    // Boot-phase heartbeat interval (seconds)
#endif
#ifndef ETM_MISSED_HEARTBEATS
  #define ETM_MISSED_HEARTBEATS  3    // Missed heartbeats before marking offline
#endif
#ifndef ETM_BOOT_DURATION_MS
  #define ETM_BOOT_DURATION_MS   30000UL  // Boot phase duration (ms)
#endif

// ---------------------------------------------------------------------------
// MAC Address Table  (indexed by ETM_BOARD_* constants above)
// ---------------------------------------------------------------------------
static const uint8_t ETM_BOARD_MACS[ETM_NUM_BOARDS][6] = {
  {0x02, 0x00, 0x00, 0x00, 0x00, 0x01},  // DG — Droid Gateway
  {0x02, 0x00, 0x00, 0x00, 0x00, 0x02},  // BC — Body Controller
  {0x02, 0x00, 0x00, 0x00, 0x00, 0x03},  // BS — Body Servo Controller
  {0x02, 0x00, 0x00, 0x00, 0x00, 0x04},  // DC — Dome Controller
  {0x02, 0x00, 0x00, 0x00, 0x00, 0x05},  // DP — Dome Plate Controller
  {0x02, 0x00, 0x00, 0x00, 0x00, 0x06},  // HP — HP Controller
};

static const char* const ETM_BOARD_IDS[ETM_NUM_BOARDS] = {
  "DG", "BC", "BS", "DC", "DP", "HP"
};

static const uint8_t ETM_BROADCAST_MAC[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// ---------------------------------------------------------------------------
// Message Struct
// ALL boards must use this exact definition simultaneously.
// Fields structPacketType and structSequenceNumber are new vs. the legacy struct.
// ---------------------------------------------------------------------------
typedef struct __attribute__((packed)) {
  char     structPassword[20];
  char     structSenderID[4];
  char     structTargetID[4];
  bool     structCommandIncluded;
  uint32_t structSuccess;
  uint32_t structFailure;
  char     structCommand[100];
  uint8_t  structPacketType;      // PACKET_TYPE_COMMAND / ACK / HEARTBEAT
  uint16_t structSequenceNumber;  // ETM sequence number for ACK tracking
} espnow_struct_message;

// ---------------------------------------------------------------------------
// ETM State Structures
// ---------------------------------------------------------------------------
struct ETM_BoardStatus {
  bool          online;
  unsigned long lastSeenMs;
  unsigned long messagesSent;
  unsigned long messagesAckd;
  unsigned long totalRetries;
  unsigned long totalFailed;
};

struct ETM_PendingMessage {
  bool          active;
  uint16_t      sequenceNumber;
  char          command[100];
  char          targetID[4];
  unsigned long lastSentMs;
  int           targetBoard;                        // board index, or -1 for broadcast
  bool          expectAckFrom[ETM_NUM_BOARDS];
  bool          receivedAckFrom[ETM_NUM_BOARDS];
  uint8_t       retryCount[ETM_NUM_BOARDS];
};

// ---------------------------------------------------------------------------
// ETM Runtime State  (one copy per sketch — static is correct here)
// ---------------------------------------------------------------------------
static ETM_BoardStatus    etmBoardTable[ETM_NUM_BOARDS];
static ETM_PendingMessage etmPendingTable[ETM_PENDING_MAX];
static uint16_t           etmSequenceCounter = 0;
static unsigned long      etmNextHeartbeatMs  = 0;
static bool               etmBootPhase        = true;
static char               etmPassword[21]     = "";
static int                etmMyBoardIdx       = -1;
static bool               etmDebugEnabled     = false;  // set true to enable Serial ETM logs

// Set or toggle ETM debug output at runtime
inline void etmSetDebug(bool enable) { etmDebugEnabled = enable; }
inline void etmToggleDebug() {
  etmDebugEnabled = !etmDebugEnabled;
  Serial.printf("[ETM] Debug %s\n", etmDebugEnabled ? "ON" : "OFF");
}

#define ETM_LOG(...) do { if (etmDebugEnabled) Serial.printf(__VA_ARGS__); } while(0)

// ---------------------------------------------------------------------------
// Init — call once from setup() after esp_now_init() and peer registration
// ---------------------------------------------------------------------------
inline void etmInit(const char* password, int myBoardIdx) {
  etmMyBoardIdx = myBoardIdx;
  strncpy(etmPassword, password, sizeof(etmPassword) - 1);
  etmPassword[sizeof(etmPassword) - 1] = '\0';
  memset(etmBoardTable,   0, sizeof(etmBoardTable));
  memset(etmPendingTable, 0, sizeof(etmPendingTable));
  etmBootPhase       = true;
  etmNextHeartbeatMs = millis() + 1000;
  ETM_LOG("[ETM] Initialized as board %s (index %d)\n",
    (myBoardIdx >= 0 && myBoardIdx < ETM_NUM_BOARDS) ? ETM_BOARD_IDS[myBoardIdx] : "?",
    myBoardIdx);
}

// ---------------------------------------------------------------------------
// Utilities
// ---------------------------------------------------------------------------

// Returns board index for a given MAC address, or -1 if not recognized.
inline int etmBoardIndexFromMAC(const uint8_t* mac) {
  for (int i = 0; i < ETM_NUM_BOARDS; i++) {
    if (memcmp(mac, ETM_BOARD_MACS[i], 6) == 0) return i;
  }
  return -1;
}

// Returns board index for a two-char ID string (e.g. "BS"), or -1 if not found.
inline int etmBoardIndexFromID(const char* id) {
  for (int i = 0; i < ETM_NUM_BOARDS; i++) {
    if (strncasecmp(id, ETM_BOARD_IDS[i], 3) == 0) return i;
  }
  return -1;
}

// Returns true if the given board has been seen recently.
inline bool etmBoardOnline(int boardIdx) {
  if (boardIdx < 0 || boardIdx >= ETM_NUM_BOARDS) return false;
  return etmBoardTable[boardIdx].online;
}

// Returns the next sequence number (wraps at 65535).
inline uint16_t etmNextSequence() {
  return ++etmSequenceCounter;
}

// ---------------------------------------------------------------------------
// Heartbeat handling
// ---------------------------------------------------------------------------

// Call when a packet is received from a board (marks it online).
inline void etmHandleHeartbeat(int senderIdx) {
  if (senderIdx < 0 || senderIdx >= ETM_NUM_BOARDS) return;
  if (!etmBoardTable[senderIdx].online) {
    ETM_LOG("[ETM] Board %s came ONLINE\n", ETM_BOARD_IDS[senderIdx]);
  }
  etmBoardTable[senderIdx].online      = true;
  etmBoardTable[senderIdx].lastSeenMs  = millis();
}

// Broadcast a heartbeat packet to all peers.
inline void etmSendHeartbeat() {
  if (etmMyBoardIdx < 0) return;
  espnow_struct_message hb;
  memset(&hb, 0, sizeof(hb));
  strncpy(hb.structPassword, etmPassword,                    sizeof(hb.structPassword) - 1);
  strncpy(hb.structSenderID, ETM_BOARD_IDS[etmMyBoardIdx],  sizeof(hb.structSenderID) - 1);
  strncpy(hb.structTargetID, "BR",                           sizeof(hb.structTargetID) - 1);
  hb.structPacketType     = PACKET_TYPE_HEARTBEAT;
  hb.structSequenceNumber = 0;
  esp_now_send(ETM_BROADCAST_MAC, (uint8_t*)&hb, sizeof(hb));
}

// ---------------------------------------------------------------------------
// ACK sending
// ---------------------------------------------------------------------------

// Send an ACK back to the board at senderIdx for the given sequence number.
inline void etmSendAck(int senderIdx, uint16_t seqNum) {
  if (senderIdx < 0 || senderIdx >= ETM_NUM_BOARDS) return;
  if (etmMyBoardIdx < 0) return;
  espnow_struct_message ack;
  memset(&ack, 0, sizeof(ack));
  strncpy(ack.structPassword, etmPassword,                    sizeof(ack.structPassword) - 1);
  strncpy(ack.structSenderID, ETM_BOARD_IDS[etmMyBoardIdx],  sizeof(ack.structSenderID) - 1);
  strncpy(ack.structTargetID, ETM_BOARD_IDS[senderIdx],      sizeof(ack.structTargetID) - 1);
  ack.structPacketType     = PACKET_TYPE_ACK;
  ack.structSequenceNumber = seqNum;
  esp_now_send(ETM_BOARD_MACS[senderIdx], (uint8_t*)&ack, sizeof(ack));
}

// ---------------------------------------------------------------------------
// Pending table — tracks outbound messages waiting for ACK
// ---------------------------------------------------------------------------

// Add a sent message to the pending table for ACK tracking.
// targetBoard = -1 for broadcast; otherwise use ETM_BOARD_* index.
// Returns the slot index, or -1 if the table is full.
inline int etmAddToPending(const char* command, const char* targetID, int targetBoard, uint16_t seqNum) {
  for (int slot = 0; slot < ETM_PENDING_MAX; slot++) {
    if (etmPendingTable[slot].active) continue;

    ETM_PendingMessage& p = etmPendingTable[slot];
    p.active         = true;
    p.sequenceNumber = seqNum;
    p.lastSentMs     = millis();
    p.targetBoard    = targetBoard;
    strncpy(p.command,  command,  sizeof(p.command)  - 1);  p.command[sizeof(p.command) - 1]   = '\0';
    strncpy(p.targetID, targetID, sizeof(p.targetID) - 1);  p.targetID[sizeof(p.targetID) - 1] = '\0';

    for (int j = 0; j < ETM_NUM_BOARDS; j++) {
      p.expectAckFrom[j]   = false;
      p.receivedAckFrom[j] = false;
      p.retryCount[j]      = 0;
    }

    if (targetBoard == -1) {
      // Broadcast — expect ACK from every online board except ourselves
      for (int j = 0; j < ETM_NUM_BOARDS; j++) {
        if (j != etmMyBoardIdx && etmBoardTable[j].online) {
          p.expectAckFrom[j] = true;
          etmBoardTable[j].messagesSent++;   // count as sent to each online peer
        }
      }
    } else if (targetBoard >= 0 && targetBoard < ETM_NUM_BOARDS) {
      p.expectAckFrom[targetBoard] = true;
      etmBoardTable[targetBoard].messagesSent++;
    }

    return slot;
  }
  ETM_LOG("[ETM] WARNING: Pending table full — message dropped from ACK tracking\n");
  return -1;
}

// Process an incoming ACK packet.
inline void etmProcessAck(int senderIdx, uint16_t seqNum) {
  if (senderIdx < 0 || senderIdx >= ETM_NUM_BOARDS) return;

  for (int i = 0; i < ETM_PENDING_MAX; i++) {
    if (!etmPendingTable[i].active) continue;
    if (etmPendingTable[i].sequenceNumber != seqNum) continue;

    etmPendingTable[i].receivedAckFrom[senderIdx] = true;
    etmBoardTable[senderIdx].messagesAckd++;

    // Check whether all expected ACKs have been received
    bool allAckd = true;
    for (int j = 0; j < ETM_NUM_BOARDS; j++) {
      if (etmPendingTable[i].expectAckFrom[j] && !etmPendingTable[i].receivedAckFrom[j]) {
        allAckd = false;
        break;
      }
    }
    if (allAckd) {
      etmPendingTable[i].active = false;
      ETM_LOG("[ETM] Seq %u confirmed delivered\n", seqNum);
    }
    return;
  }
}

// ---------------------------------------------------------------------------
// Retry logic (internal helper)
// ---------------------------------------------------------------------------
inline void etmRetry(int slot) {
  ETM_PendingMessage& p = etmPendingTable[slot];
  espnow_struct_message msg;
  memset(&msg, 0, sizeof(msg));
  strncpy(msg.structPassword, etmPassword,                   sizeof(msg.structPassword) - 1);
  strncpy(msg.structSenderID, ETM_BOARD_IDS[etmMyBoardIdx], sizeof(msg.structSenderID) - 1);
  strncpy(msg.structTargetID, p.targetID,                    sizeof(msg.structTargetID) - 1);
  msg.structCommandIncluded = (p.command[0] != '\0');
  strncpy(msg.structCommand, p.command,                      sizeof(msg.structCommand) - 1);
  msg.structPacketType     = PACKET_TYPE_COMMAND;
  msg.structSequenceNumber = p.sequenceNumber;

  const uint8_t* mac = (p.targetBoard == -1) ? ETM_BROADCAST_MAC : ETM_BOARD_MACS[p.targetBoard];
  esp_now_send(mac, (uint8_t*)&msg, sizeof(msg));
  p.lastSentMs = millis();
}

// ---------------------------------------------------------------------------
// Process pending messages — check timeouts and retry  (called by etmProcess)
// ---------------------------------------------------------------------------
inline void etmProcessPending() {
  unsigned long now = millis();

  for (int i = 0; i < ETM_PENDING_MAX; i++) {
    if (!etmPendingTable[i].active) continue;
    if (now - etmPendingTable[i].lastSentMs < ETM_TIMEOUT_MS) continue;

    bool needsRetry  = false;
    bool allResolved = true;

    for (int j = 0; j < ETM_NUM_BOARDS; j++) {
      if (!etmPendingTable[i].expectAckFrom[j])   continue;
      if (etmPendingTable[i].receivedAckFrom[j])  continue;

      allResolved = false;

      // Board went offline — cancel wait for it
      if (!etmBoardTable[j].online) {
        etmPendingTable[i].expectAckFrom[j] = false;
        ETM_LOG("[ETM] Canceling ACK wait for offline board %s\n", ETM_BOARD_IDS[j]);
        continue;
      }

      // Max retries exceeded for this board
      if (etmPendingTable[i].retryCount[j] >= ETM_RETRY_MAX) {
        etmPendingTable[i].expectAckFrom[j] = false;
        etmBoardTable[j].totalFailed++;
        ETM_LOG("[ETM] Give up on %s seq %u after %d retries\n",
          ETM_BOARD_IDS[j], etmPendingTable[i].sequenceNumber, ETM_RETRY_MAX);
        continue;
      }

      // Retry
      etmPendingTable[i].retryCount[j]++;
      etmBoardTable[j].totalRetries++;
      needsRetry = true;
      ETM_LOG("[ETM] Retry %u/%u for %s seq %u\n",
        etmPendingTable[i].retryCount[j], ETM_RETRY_MAX,
        ETM_BOARD_IDS[j], etmPendingTable[i].sequenceNumber);
    }

    if (needsRetry) {
      etmRetry(i);
    } else {
      // Re-check if all outstanding ACKs are now resolved
      allResolved = true;
      for (int j = 0; j < ETM_NUM_BOARDS; j++) {
        if (etmPendingTable[i].expectAckFrom[j] && !etmPendingTable[i].receivedAckFrom[j]) {
          allResolved = false;
          break;
        }
      }
      if (allResolved) {
        etmPendingTable[i].active = false;
      }
    }
  }
}

// ---------------------------------------------------------------------------
// Process heartbeats — send periodic heartbeats, detect offline boards
// (called by etmProcess)
// ---------------------------------------------------------------------------
inline void etmProcessHeartbeats() {
  unsigned long now = millis();

  // Transition out of boot phase after ETM_BOOT_DURATION_MS
  if (etmBootPhase && now > ETM_BOOT_DURATION_MS) {
    etmBootPhase = false;
    ETM_LOG("[ETM] Boot phase complete\n");
  }

  // Send heartbeat if due
  if (now >= etmNextHeartbeatMs) {
    etmSendHeartbeat();
    unsigned long intervalMs = etmBootPhase
      ? (unsigned long)(ETM_BOOT_HEARTBEAT_SEC * 1000)
      : (unsigned long)(ETM_HEARTBEAT_SEC      * 1000);
    etmNextHeartbeatMs = now + intervalMs;
  }

  // Mark boards offline if heartbeats have been missed
  unsigned long timeout = (unsigned long)(ETM_HEARTBEAT_SEC + 1) * ETM_MISSED_HEARTBEATS * 1000UL;
  for (int i = 0; i < ETM_NUM_BOARDS; i++) {
    if (i == etmMyBoardIdx) continue;
    if (etmBoardTable[i].online && (now - etmBoardTable[i].lastSeenMs) > timeout) {
      etmBoardTable[i].online = false;
      ETM_LOG("[ETM] Board %s went OFFLINE\n", ETM_BOARD_IDS[i]);
    }
  }
}

// ---------------------------------------------------------------------------
// etmProcess — call once per loop() iteration
// ---------------------------------------------------------------------------
inline void etmProcess() {
  etmProcessPending();
  etmProcessHeartbeats();
}
