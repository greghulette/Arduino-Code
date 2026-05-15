#pragma once
// ─────────────────────────────────────────────────────────────────────────────
//  wcb_config.h — WCB network credentials and Maestro target definitions
//
//  Edit these values to match your WCB system configuration.
//  Find the network values by querying any WCB over its USB serial port:
//    ?WCBM   → MAC octets 2 and 3
//    ?WCBP   → network password
//    ?WCBQ   → total WCBs in the system
//
//  This file is intentionally separate from rc_config.h so network credentials
//  can be tracked differently in version control (e.g. gitignored).
// ─────────────────────────────────────────────────────────────────────────────

// ─── WCB network credentials ─────────────────────────────────────────────────
#define WCB_MAC_OCT2      0x00   // ?WCBM second octet
#define WCB_MAC_OCT3      0x00   // ?WCBM third octet
#define WCB_PASSWORD      "change_me"
#define WCB_QUANTITY      4      // ?WCBQ — total WCBs in the system
#define WCB_DEVICE_ID     3      // Unique ID for this RC Controller on the WCB network
                                  // Must be ≤ WCB_QUANTITY (or 20 for the special slot)

// ─── WiFi credentials ────────────────────────────────────────────────────────
//  Station networks to try on boot (first match wins).
//  Fall back to the built-in AP if none connect within WIFI_STA_TIMEOUT_MS.
//  Leave ssid = nullptr to skip a slot.
#define WIFI_STA_TIMEOUT_MS   6000
#define MAX_WIFI_NETS          4

// ─── Local Maestro serial baud ────────────────────────────────────────────────
//  Serial2 is the hardware UART wired to local Maestros (TX pin = GPIO6).
//  The baud rate is the only compile-time setting — Maestro device number,
//  remote routing, and slot enable/disable are all configured at RUNTIME via
//  the GUI's "Maestro Locations" panel and stored in NVS (rcConfig.maestros[]).
//
//  Each physical Maestro wired to Serial2 must have its UART configured for
//  this baud rate in Maestro Control Center (Serial Settings → Fixed Baud Rate)
//  OR be set to "Detect Baud Rate" mode (the auto-detect uses the 0xAA in the
//  Pololu protocol header to lock onto whatever rate the host is sending).
#define LOCAL_MAESTRO_BAUD_RATE   115200

// ─── Remote Maestros via WCBStream over ESP-NOW ───────────────────────────────
//  All remote Maestro commands BROADCAST over the Kyber path — every WCB on
//  the network receives the packet and any WCB configured with Kyber_Remote
//  forwards the raw bytes to its local Maestro serial port.  No per-slot
//  WCB/port routing is needed.
//
//  Up to 8 logical Maestros (IDs 1-8) can be defined; each one's location
//  (Local on Serial2 vs Remote via broadcast) and Pololu device # are set
//  per-slot in the GUI.  Multiple Maestros on the same physical bus are
//  distinguished by their Pololu device numbers — bytes sent to a slot
//  with a specific device # are ignored by Maestros with different IDs.
