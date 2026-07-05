#pragma once

#include <stdint.h>

// =============================================================================
// REFEREE WATCH <-> CONTROLLER ESP-NOW LINK (shared protocol contract)
// =============================================================================
// The watch uplink uses the ESP32's WiFi radio via ESP-NOW - the nRF24
// display broadcast is completely untouched. Unicast with MAC-layer ACK:
// the watch knows delivery from the send callback and gives haptic
// confirmation. Encryption: ESP-NOW PMK/LMK; the controller only accepts
// frames from allowlisted watch MACs.
//
// Pairing procedure (build-time, no runtime pairing):
//   1. Flash watch + controller; both log their STA MAC at boot.
//   2. Put the watch MACs in the controller's espnow_watches.h allowlist,
//      and the controller MAC in the watch's config.
//   3. Reflash. Keys below MUST be changed per deployment.

// WiFi channel for the ESP-NOW link. Channel 6 (2426-2448 MHz) sits between
// the nRF24 candidate channels 24 (2424) and 49 (2449), so the uplink never
// overlaps the display broadcast no matter which candidate is active
#define ESPNOW_WIFI_CHANNEL 6

// Primary master key (16 bytes) + per-peer local master key. CHANGE THESE
// for a real deployment - anyone with the defaults can inject commands
#define ESPNOW_PMK "sb_clock_pmk_v1!"
#define ESPNOW_LMK "sb_clock_lmk_v1!"

#define ESPNOW_CMD_MAGIC 0x52 // 'R'

typedef enum {
  ESPNOW_CMD_START_STOP = 1,
  ESPNOW_CMD_RESET = 2,
} espnow_cmd_t;

// Command frame, watch -> controller. sequence increments per button press
// (per watch); the controller drops repeats of the same (watch_id, sequence)
// so a retry burst is never a double-toggle
typedef struct __attribute__((packed)) {
  uint8_t magic;    // ESPNOW_CMD_MAGIC
  uint8_t watch_id; // 1..ESPNOW_MAX_WATCHES, unique per watch
  uint8_t command;  // espnow_cmd_t
  uint8_t sequence; // per-watch press counter (wraps)
} EspNowCommand;

#define ESPNOW_MAX_WATCHES 4
