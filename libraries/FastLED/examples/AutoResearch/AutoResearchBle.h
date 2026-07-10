// examples/AutoResearch/AutoResearchBle.h
//
// BLE autoresearch for ESP32: GATT server for JSON-RPC over BLE.
// Used by `bash autoresearch --ble`.
//
// The ESP32 starts a BLE GATT server with a custom service.
// The host Python script connects via Bleak and tests
// bidirectional JSON-RPC communication (ping/pong).
//
// NOTE: BLE transport creation/destruction is handled by
// AutoResearchRemoteControl::startBleRemote()/stopBleRemote().
// This file provides state accessors only.

#pragma once

#include "fl/stl/stdint.h"

// BLE configuration constants
#define AUTORESEARCH_BLE_DEVICE_NAME "FastLED-C6"

/// @brief State for BLE autoresearch
struct AutoResearchBleState {
    bool ble_server_active = false;
};

/// @brief Get current BLE autoresearch state.
/// @return Reference to the global BLE state
AutoResearchBleState& getBleState();
