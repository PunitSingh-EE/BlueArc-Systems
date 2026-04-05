#pragma once
// ════════════════════════════════════════════════════════════════
//  BlueArc Systems — Slave Node  |  config.h
//  All configuration in one place — edit here before upload
// ════════════════════════════════════════════════════════════════

// ── NODE SELECTION ───────────────────────────────────────────────
//  Uncomment ONE only before uploading
// ────────────────────────────────────────────────────────────────
//#define NODE_A1          // A1: has INA260, receives A2 packet, forwards to Master
 #define NODE_A2           // A2: endpoint node, transmits only

#ifdef NODE_A1
  #define SLAVE_ID        "A1"
  #define ENABLE_INA260   1     // INA260 current sensor fitted on A1
  #define IS_RELAY_NODE   1     // receives from A2 → forwards combined packet to Master
#else
  #define SLAVE_ID        "A2"
  #define ENABLE_INA260   0     // no INA260 on A2
  #define IS_RELAY_NODE   0     // A2 is endpoint — transmit only
#endif

// ── DEFAULT LOCATION (overridable via WiFi dashboard) ────────────
#define DEFAULT_LAT       23.0225
#define DEFAULT_LON       72.5714

// ── PIN DEFINITIONS — ESP32-C3 Super Mini ────────────────────────
#define PIN_LORA_DIO0     0     // LoRa interrupt
#define PIN_LORA_NSS      1     // LoRa CS
#define PIN_PIR           5     // HC-SR501 OUT
#define PIN_LDR           2     // LDR analog out (ADC1)
#define PIN_WS2812_STREET 4     // WS2812 #1 — streetlight
#define PIN_WS2812_STATUS 3     // WS2812 #2 — status indicator
#define PIN_LORA_MISO     6
#define PIN_LORA_MOSI     7
#define PIN_I2C_SDA       8
#define PIN_I2C_SCL       9
#define PIN_LORA_SCK      10
#define PIN_RELAY         20    // LOW = relay energised = line live
#define PIN_BYPASS_SW     21    // INPUT_PULLUP — LOW = bypass active

// ── WS2812 ───────────────────────────────────────────────────────
#define NUM_LEDS_STREET   8     // Number of LEDs on streetlight strip
#define NUM_LEDS_STATUS   8     // Status LED strip
#define BRIGHTNESS_FULL   255
#define BRIGHTNESS_SAVER  38    // ~15%
#define BRIGHTNESS_OFF    0

// ── TIMING ───────────────────────────────────────────────────────
// Fast heartbeat: send every 500ms, declare dead after 2 misses (1s)
// This gives ~1s wire-break detection vs the old 3s
#define HEARTBEAT_TX_MS   500   // transmit packet every 500ms
#define HB_TIMEOUT_MS     1100  // 2.2 missed packets → wire break declared
#define PIR_HOLD_MS       5000
#define FADE_STEP_MS      20

// ── SENSOR THRESHOLDS ────────────────────────────────────────────
#define LDR_NIGHT_THRESHOLD  600
#define LDR_DAY_THRESHOLD    400
#define TILT_THRESHOLD    30.0f
#define MPU_ACCEL_SCALE   16384.0f

// ── LORA ─────────────────────────────────────────────────────────
#define LORA_FREQUENCY    433E6
#define LORA_BANDWIDTH    125E3
#define LORA_SF           7
#define LORA_TX_POWER     17
#define LORA_SYNC_WORD    0xB4

// ── I2C ADDRESSES ────────────────────────────────────────────────
#define MPU6050_ADDR      0x68
#define INA260_ADDR       0x40

// ── ENERGY BASELINE ──────────────────────────────────────────────
#define BASELINE_W        1.41f   // watts at 100% — update if LED count changes

// ── NVS KEYS ─────────────────────────────────────────────────────
#define NVS_NAMESPACE     "bluearc"
#define NVS_KEY_BYPASS_F  "bypass_intent"
#define NVS_KEY_ID        "slave_id"
#define NVS_KEY_LAT       "lat"
#define NVS_KEY_LON       "lon"

// ── BLE ──────────────────────────────────────────────────────────
#define BLE_DEVICE_NAME   "BlueArc-Slave-" SLAVE_ID
#define NUS_SVC_UUID      "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define NUS_RX_UUID       "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define NUS_TX_UUID       "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

// ── FAULT FLAGS (bitmask) ─────────────────────────────────────────
#define FAULT_OVERCURRENT  (1 << 0)
#define FAULT_OVERVOLTAGE  (1 << 1)
#define FAULT_VOLTAGE_SAG  (1 << 2)
#define FAULT_OVERTEMP     (1 << 3)
#define FAULT_HB_LOST      (1 << 4)
#define FAULT_TILT         (1 << 5)
#define FAULT_RELAY_TRIP   (1 << 6)
