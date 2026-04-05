// ════════════════════════════════════════════════════════════════
//  BlueArc Systems — MASTER NODE
//  File: master.ino  |  ESP32-38 pin WROOM
//
//  Hardware (as per pinout doc):
//    • ACS71020 power monitor (I2C 0x60) — V, I, P, Q, PF, freq
//    • OLED SSD1306 0.96" (I2C 0x3C)
//    • LoRa SX1278 433MHz (VSPI)
//    • Relay module (GPIO2)
//    • 4 pushbuttons (GPIO32, 33, 25, 35)
//    • 3 status LEDs (GPIO26 green, GPIO27 orange, GPIO12 blue)
//
//  Libraries required:
//    - LoRa             by Sandeep Mistry
//    - ArduinoJson      by Benoit Blanchon (v6.x)
//    - Adafruit SSD1306 by Adafruit
//    - Adafruit GFX     by Adafruit
//    - WiFi, WebServer  (built-in ESP32 core)
// ════════════════════════════════════════════════════════════════

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <LoRa.h>
#include <ArduinoJson.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ── OLED ─────────────────────────────────────────────────────────
#define OLED_ADDR   0x3C
#define OLED_W      128
#define OLED_H      64
Adafruit_SSD1306 oled(OLED_W, OLED_H, &Wire, -1);

// ── LoRa pins ────────────────────────────────────────────────────
#define LORA_SCK    18
#define LORA_MISO   19
#define LORA_MOSI   23
#define LORA_NSS     5
#define LORA_DIO0   34
#define LORA_RST    15
#define LORA_FREQ   433E6
#define LORA_SF     7
#define LORA_BW     125E3
#define LORA_SYNC   0xB4
#define LORA_PWR    17

// ── ACS71020 I2C ─────────────────────────────────────────────────
#define ACS_ADDR    0x60

// ACS71020 register map
#define ACS_REG_VRMS      0x10
#define ACS_REG_IRMS      0x12
#define ACS_REG_PACTIVE   0x14
#define ACS_REG_PREACTIVE 0x15
#define ACS_REG_PAPPARENT 0x16
#define ACS_REG_PF        0x17
#define ACS_REG_NFAULT    0x1A
#define ACS_REG_VCODES    0x1D
#define ACS_REG_ICODES    0x1E
#define ACS_REG_PINSTANT  0x1F
#define ACS_REG_VZERO     0x18  // for zero-crossing / freq
// Scale factors (for ACS71020 15A variant — adjust for your variant)
#define ACS_VREF       605.0f   // internal Vref mV — see datasheet §7.4
#define ACS_VRANGE     250.0f   // AC RMS Vrms max (250V variant)
#define ACS_IRANGE     15.0f    // current range in A
#define ACS_PMAX       (ACS_VRANGE * ACS_IRANGE)

// ── GPIOs ────────────────────────────────────────────────────────
#define PIN_RELAY    2
#define PIN_PB1      32   // Overcurrent sim
#define PIN_PB2      33   // Overvoltage sim
#define PIN_PB3      25   // Voltage sag sim
#define PIN_PB4      35   // Fault reset (input-only)
#define PIN_LED_GRN  26   // Green
#define PIN_LED_ORG  27   // Orange
#define PIN_LED_BLU  12   // Blue (LoRa activity)

// ── Nominal values ────────────────────────────────────────────────
#define NOM_VOLTAGE     230.0f
#define OC_THRESHOLD    10.0f   // A — overcurrent trip
#define OV_THRESHOLD    253.0f  // V — 110% of 230V
#define UV_THRESHOLD    200.0f  // V — voltage sag
#define OT_THRESHOLD    65.0f   // °C — box overtemp (no sensor, demo only)
#define PF_CRIT         0.70f   // critical power factor
#define HB_TIMEOUT_MS   1100    // wire-break timeout (matches slaves)

// ── WiFi AP ───────────────────────────────────────────────────────
#define WIFI_AP_SSID  "BlueArc-Master"
#define WIFI_AP_PASS  "bluearc123"
WebServer httpServer(80);

// ── Fault flags ───────────────────────────────────────────────────
#define FAULT_OVERCURRENT  (1 << 0)
#define FAULT_OVERVOLTAGE  (1 << 1)
#define FAULT_VOLTAGE_SAG  (1 << 2)
#define FAULT_OVERTEMP     (1 << 3)
#define FAULT_HB_LOST_A1   (1 << 4)
#define FAULT_HB_LOST_A2   (1 << 5)
#define FAULT_TILT_A1      (1 << 6)
#define FAULT_TILT_A2      (1 << 7)
#define FAULT_RELAY_TRIP   (1 << 8)
#define FAULT_PF_CRIT      (1 << 9)

// ═════════════════════════════════════════════════════════════════
//  DATA STRUCTURES
// ═════════════════════════════════════════════════════════════════

struct MasterData {
  float vrms      = 0;
  float irms      = 0;
  float pActive   = 0;   // W
  float pReactive = 0;   // VAR
  float pApparent = 0;   // VA
  float pf        = 0;
  float freqHz    = 50.0f;
  float vInst     = 0;
  float iInst     = 0;
  float pInst     = 0;
  bool  acsOk     = false;
  uint16_t nfault = 0;
  // Simulation overrides
  bool  simOC     = false;
  bool  simOV     = false;
  bool  simSag    = false;
};

struct SlaveCache {
  bool    valid       = false;
  unsigned long lastRxMs = 0;
  uint32_t lastSeq    = 0;
  bool    pir         = false;
  bool    isNight     = false;
  bool    tilt        = false;
  bool    relayOK     = true;
  bool    bypassActive= false;
  bool    bypassIntent= false;
  uint8_t faultFlags  = 0;
  int     rssi        = 0;
  float   inaCurrentA = 0;
  float   inaPowerW   = 0;
  float   savingW     = 0;
  float   costSaved   = 0;
};

MasterData g_m;
SlaveCache g_a1;
SlaveCache g_a2;

bool     g_relayTripped = false;
uint16_t g_faultFlags   = 0;
uint32_t g_txSeq        = 0;

// OLED page cycling
uint8_t  g_oledPage    = 0;
unsigned long g_lastOledSwitch = 0;
#define OLED_PAGE_MS  3000   // rotate pages every 3s

// Serial status
unsigned long g_lastSerialMs = 0;

// ─────────────────────────────────────────────────────────────────
//  ACS71020 I2C read (returns 16-bit raw)
// ─────────────────────────────────────────────────────────────────
uint16_t acsRead(uint8_t reg) {
  Wire.beginTransmission(ACS_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return 0;
  Wire.requestFrom(ACS_ADDR, 2, true);
  if (Wire.available() < 2) return 0;
  uint8_t hi = Wire.read();
  uint8_t lo = Wire.read();
  return ((uint16_t)hi << 8) | lo;
}

// Signed 16-bit read (for reactive power, instantaneous values)
int16_t acsReadSigned(uint8_t reg) {
  return (int16_t)acsRead(reg);
}

void readACS71020() {
  // VRMS: raw * (VREF / 32768) * scale — check datasheet for your variant
  uint16_t vRaw = acsRead(ACS_REG_VRMS);
  uint16_t iRaw = acsRead(ACS_REG_IRMS);
  int16_t  pRaw = acsReadSigned(ACS_REG_PACTIVE);
  int16_t  qRaw = acsReadSigned(ACS_REG_PREACTIVE);
  uint16_t sRaw = acsRead(ACS_REG_PAPPARENT);
  int16_t  pfRw = acsReadSigned(ACS_REG_PF);
  uint16_t nf   = acsRead(ACS_REG_NFAULT);
  int16_t  vInst= acsReadSigned(ACS_REG_VCODES);
  int16_t  iInst= acsReadSigned(ACS_REG_ICODES);
  int16_t  pInst= acsReadSigned(ACS_REG_PINSTANT);

  // Apply scale factors
  g_m.vrms      = vRaw  * (ACS_VRANGE / 32768.0f);
  g_m.irms      = iRaw  * (ACS_IRANGE / 32768.0f);
  g_m.pActive   = pRaw  * (ACS_PMAX   / 32768.0f);
  g_m.pReactive = qRaw  * (ACS_PMAX   / 32768.0f);
  g_m.pApparent = sRaw  * (ACS_PMAX   / 32768.0f);
  g_m.pf        = pfRw  / 32768.0f;
  g_m.vInst     = vInst * (ACS_VRANGE / 32768.0f);
  g_m.iInst     = iInst * (ACS_IRANGE / 32768.0f);
  g_m.pInst     = pInst * (ACS_PMAX   / 32768.0f);
  g_m.nfault    = nf;
  g_m.acsOk     = true;

  // Apply simulation overrides
  if (g_m.simOC)  g_m.irms  = OC_THRESHOLD * 1.5f;
  if (g_m.simOV)  g_m.vrms  = NOM_VOLTAGE  * 1.15f;
  if (g_m.simSag) g_m.vrms  = NOM_VOLTAGE  * 0.75f;
}

// Zero-crossing frequency estimation (count ZC edges over 200ms)
void updateFrequency() {
  static unsigned long zcTime[4] = {0};
  static uint8_t zcIdx = 0;
  static int16_t lastVInst = 0;

  int16_t v = acsReadSigned(ACS_REG_VCODES);
  // Rising zero crossing
  if (lastVInst < 0 && v >= 0) {
    zcTime[zcIdx % 4] = millis();
    zcIdx++;
    if (zcIdx >= 4) {
      // Period = average of last 2 half-cycles * 2
      unsigned long period = (zcTime[(zcIdx-1)%4] - zcTime[(zcIdx-3)%4]);
      if (period > 0) g_m.freqHz = 2000.0f / (float)period;
    }
  }
  lastVInst = v;
}

// ─────────────────────────────────────────────────────────────────
//  RELAY CONTROL
// ─────────────────────────────────────────────────────────────────
void tripRelay(const char* reason) {
  if (g_relayTripped) return;
  digitalWrite(PIN_RELAY, LOW);   // LOW = trip (feeder cut) per pinout
  g_relayTripped = true;
  g_faultFlags |= FAULT_RELAY_TRIP;
  digitalWrite(PIN_LED_ORG, HIGH);
  Serial.printf("[RELAY] TRIPPED — %s\n", reason);
}

void clearRelay() {
  digitalWrite(PIN_RELAY, HIGH);  // HIGH = normal per pinout
  g_relayTripped = false;
  g_faultFlags &= ~FAULT_RELAY_TRIP;
  g_m.simOC  = false;
  g_m.simOV  = false;
  g_m.simSag = false;
  // Clear sim-only faults
  g_faultFlags &= ~(FAULT_OVERCURRENT | FAULT_OVERVOLTAGE | FAULT_VOLTAGE_SAG);
  Serial.println("[RELAY] Cleared — feeder restored");
}

// ─────────────────────────────────────────────────────────────────
//  FAULT PROCESSING
// ─────────────────────────────────────────────────────────────────
void processFaults() {
  // ── Overcurrent ─────────────────────────────────────────────────
  if (g_m.irms > OC_THRESHOLD) {
    g_faultFlags |= FAULT_OVERCURRENT;
    tripRelay("Overcurrent");
    Serial.printf("[FAULT] Overcurrent: %.2fA (threshold=%.0fA)\n", g_m.irms, OC_THRESHOLD);
  }

  // ── Overvoltage ──────────────────────────────────────────────────
  if (g_m.vrms > OV_THRESHOLD) {
    g_faultFlags |= FAULT_OVERVOLTAGE;
    tripRelay("Overvoltage");
    Serial.printf("[FAULT] Overvoltage: %.1fV\n", g_m.vrms);
  }

  // ── Voltage sag ──────────────────────────────────────────────────
  if (g_m.vrms > 10 && g_m.vrms < UV_THRESHOLD) {
    g_faultFlags |= FAULT_VOLTAGE_SAG;
    // No relay trip for sag — just alert
    Serial.printf("[FAULT] Voltage sag: %.1fV\n", g_m.vrms);
  } else if (!(g_m.simSag)) {
    g_faultFlags &= ~FAULT_VOLTAGE_SAG;
  }

  // ── Low power factor ─────────────────────────────────────────────
  if (g_m.pf < PF_CRIT && g_m.pf > 0.01f) {
    g_faultFlags |= FAULT_PF_CRIT;
  } else {
    g_faultFlags &= ~FAULT_PF_CRIT;
  }

  // ── A1 heartbeat ─────────────────────────────────────────────────
  if (g_a1.valid && (millis() - g_a1.lastRxMs > HB_TIMEOUT_MS)) {
    if (!g_a1.bypassIntent) {
      if (!(g_faultFlags & FAULT_HB_LOST_A1)) {
        g_faultFlags |= FAULT_HB_LOST_A1;
        Serial.println("[FAULT] A1 heartbeat lost — Master→A1 wire break suspected");
      }
    }
  } else if (g_a1.valid) {
    g_faultFlags &= ~FAULT_HB_LOST_A1;
  }

  // ── A2 heartbeat (via A1 forwarded data) ─────────────────────────
  if (g_a2.valid && (millis() - g_a2.lastRxMs > HB_TIMEOUT_MS * 2)) {
    // A2 age comes via A1 forward, so add one hop tolerance
    if (!g_a2.bypassIntent) {
      if (!(g_faultFlags & FAULT_HB_LOST_A2)) {
        g_faultFlags |= FAULT_HB_LOST_A2;
        Serial.println("[FAULT] A2 data stale — A1→A2 wire break suspected");
      }
    }
  } else if (g_a2.valid) {
    g_faultFlags &= ~FAULT_HB_LOST_A2;
  }

  // ── Tilt from slaves ─────────────────────────────────────────────
  if (g_a1.tilt) g_faultFlags |= FAULT_TILT_A1; else g_faultFlags &= ~FAULT_TILT_A1;
  if (g_a2.tilt) g_faultFlags |= FAULT_TILT_A2; else g_faultFlags &= ~FAULT_TILT_A2;

  // ── LED update ───────────────────────────────────────────────────
  bool hasFault = (g_faultFlags != 0);
  bool critical = g_relayTripped;

  if (critical) {
    // Fast blink orange
    digitalWrite(PIN_LED_ORG, (millis() / 100) % 2);
    digitalWrite(PIN_LED_GRN, LOW);
  } else if (hasFault) {
    digitalWrite(PIN_LED_ORG, HIGH);
    // Slow blink green = partial
    digitalWrite(PIN_LED_GRN, (millis() / 500) % 2);
  } else {
    digitalWrite(PIN_LED_ORG, LOW);
    digitalWrite(PIN_LED_GRN, HIGH);
  }
}

// ─────────────────────────────────────────────────────────────────
//  PUSHBUTTON HANDLING
// ─────────────────────────────────────────────────────────────────
void checkButtons() {
  static unsigned long lastPB1 = 0, lastPB2 = 0, lastPB3 = 0, lastPB4 = 0;
  unsigned long now = millis();

  // PB1 — Overcurrent sim (debounced 200ms)
  if (digitalRead(PIN_PB1) == LOW && now - lastPB1 > 200) {
    lastPB1 = now;
    g_m.simOC = true;
    g_m.simOV = false;
    g_m.simSag = false;
    g_faultFlags |= FAULT_OVERCURRENT;
    Serial.println("[SIM] PB1 — Overcurrent simulation active");
  }

  // PB2 — Overvoltage sim
  if (digitalRead(PIN_PB2) == LOW && now - lastPB2 > 200) {
    lastPB2 = now;
    g_m.simOV = true;
    g_m.simOC = false;
    g_m.simSag = false;
    g_faultFlags |= FAULT_OVERVOLTAGE;
    Serial.println("[SIM] PB2 — Overvoltage simulation active");
  }

  // PB3 — Voltage sag sim
  if (digitalRead(PIN_PB3) == LOW && now - lastPB3 > 200) {
    lastPB3 = now;
    g_m.simSag = true;
    g_m.simOC = false;
    g_m.simOV = false;
    g_faultFlags |= FAULT_VOLTAGE_SAG;
    Serial.println("[SIM] PB3 — Voltage sag simulation active");
  }

  // PB4 — Reset all faults
  if (digitalRead(PIN_PB4) == LOW && now - lastPB4 > 500) {
    lastPB4 = now;
    clearRelay();
    g_faultFlags = 0;
    Serial.println("[SIM] PB4 — All faults cleared, relay reset");
  }
}

// ─────────────────────────────────────────────────────────────────
//  LORA — Parse incoming A1 packet
// ─────────────────────────────────────────────────────────────────
void parseLoRaPacket(const String& raw, int rssi) {
  StaticJsonDocument<512> doc;
  DeserializationError err = deserializeJson(doc, raw);
  if (err) {
    Serial.printf("[LoRa RX] Parse error: %s\n", err.c_str());
    return;
  }

  const char* src = doc["src"] | "";
  if (strcmp(src, "A1") != 0) return;  // Master only accepts A1 upstream

  // ── Update A1 cache ──────────────────────────────────────────────
  g_a1.valid        = true;
  g_a1.lastRxMs     = millis();
  g_a1.lastSeq      = doc["sq"]   | 0;
  g_a1.pir          = doc["pir"]  | 0;
  g_a1.isNight      = (doc["ldr"] | 1) == 0;
  g_a1.tilt         = doc["tilt"] | 0;
  g_a1.relayOK      = doc["rel"]  | 1;
  g_a1.bypassActive = doc["byp"]  | 0;
  g_a1.bypassIntent = doc["bypI"] | 0;
  g_a1.faultFlags   = doc["F"]    | 0;
  g_a1.rssi         = rssi;
  g_a1.inaCurrentA  = doc["inaI"] | 0.0f;
  g_a1.inaPowerW    = doc["inaP"] | 0.0f;
  g_a1.savingW      = doc["savW"] | 0.0f;
  g_a1.costSaved    = doc["cost"] | 0.0f;

  // ── Update A2 cache (forwarded by A1) ────────────────────────────
  if (doc.containsKey("A2")) {
    JsonObject a2doc = doc["A2"];
    uint8_t a2F = a2doc["F"] | 0;
    unsigned long a2age = a2doc["age"] | 255;

    if (a2F & 0x10 || a2age == 255) {
      // A1 is reporting A2 as HB-lost
      if (g_a2.valid) {
        // Keep last known data but mark stale
        g_a2.faultFlags |= 0x10;
      }
    } else {
      g_a2.valid        = true;
      g_a2.lastRxMs     = millis() - (a2age * 1000);  // estimated time
      g_a2.lastSeq      = a2doc["sq"]   | 0;
      g_a2.pir          = a2doc["pir"]  | 0;
      g_a2.isNight      = (a2doc["ldr"] | 1) == 0;
      g_a2.tilt         = a2doc["tilt"] | 0;
      g_a2.relayOK      = a2doc["rel"]  | 1;
      g_a2.bypassActive = a2doc["byp"]  | 0;
      g_a2.bypassIntent = a2doc["bypI"] | 0;
      g_a2.faultFlags   = a2F;
      g_a2.rssi         = a2doc["rssi"] | 0;
    }
  }

  Serial.printf("[LoRa RX A1] sq=%u  RSSI=%d  V=%.1f  I=%.3f\n",
                g_a1.lastSeq, rssi, g_m.vrms, g_m.irms);
}

// ─────────────────────────────────────────────────────────────────
//  LORA — Transmit master packet to receiving station
// ─────────────────────────────────────────────────────────────────
void transmitMasterPacket() {
  StaticJsonDocument<640> doc;
  doc["src"] = "M";
  doc["sq"]  = ++g_txSeq;
  doc["V"]   = serialized(String(g_m.vrms, 1));
  doc["I"]   = serialized(String(g_m.irms, 3));
  doc["P"]   = serialized(String(g_m.pActive, 1));
  doc["Q"]   = serialized(String(g_m.pReactive, 1));
  doc["S"]   = serialized(String(g_m.pApparent, 1));
  doc["PF"]  = serialized(String(g_m.pf, 3));
  doc["Hz"]  = serialized(String(g_m.freqHz, 2));
  doc["R"]   = g_relayTripped ? 0 : 1;
  doc["F"]   = g_faultFlags;

  if (g_a1.valid) {
    JsonObject a1 = doc.createNestedObject("A1");
    a1["pir"]  = g_a1.pir  ? 1 : 0;
    a1["ldr"]  = g_a1.isNight ? 0 : 1;
    a1["tilt"] = g_a1.tilt ? 1 : 0;
    a1["rel"]  = g_a1.relayOK ? 1 : 0;
    a1["byp"]  = g_a1.bypassActive ? 1 : 0;
    a1["bypI"] = g_a1.bypassIntent ? 1 : 0;
    a1["F"]    = g_a1.faultFlags;
    a1["rssi"] = g_a1.rssi;
    a1["inaI"] = serialized(String(g_a1.inaCurrentA, 3));
    a1["inaP"] = serialized(String(g_a1.inaPowerW,   2));
    a1["savW"] = serialized(String(g_a1.savingW,      2));
    a1["cost"] = serialized(String(g_a1.costSaved,    2));
    a1["age"]  = (millis() - g_a1.lastRxMs) / 1000;
  } else {
    JsonObject a1 = doc.createNestedObject("A1");
    a1["F"] = 0x10; a1["age"] = 255;
  }

  if (g_a2.valid) {
    JsonObject a2 = doc.createNestedObject("A2");
    a2["pir"]  = g_a2.pir  ? 1 : 0;
    a2["ldr"]  = g_a2.isNight ? 0 : 1;
    a2["tilt"] = g_a2.tilt ? 1 : 0;
    a2["rel"]  = g_a2.relayOK ? 1 : 0;
    a2["byp"]  = g_a2.bypassActive ? 1 : 0;
    a2["bypI"] = g_a2.bypassIntent ? 1 : 0;
    a2["F"]    = g_a2.faultFlags;
    a2["rssi"] = g_a2.rssi;
    a2["age"]  = (millis() - g_a2.lastRxMs) / 1000;
  } else {
    JsonObject a2 = doc.createNestedObject("A2");
    a2["F"] = 0x10; a2["age"] = 255;
  }

  String payload;
  serializeJson(doc, payload);

  digitalWrite(PIN_LED_BLU, HIGH);
  LoRa.beginPacket();
  LoRa.print(payload);
  LoRa.endPacket();
  delay(5);
  digitalWrite(PIN_LED_BLU, LOW);

  Serial.printf("[LoRa TX] sq=%u  %dB\n", g_txSeq, payload.length());
}

// ─────────────────────────────────────────────────────────────────
//  OLED DISPLAY — multi-page rolling display
// ─────────────────────────────────────────────────────────────────
void drawOLED() {
  oled.clearDisplay();
  oled.setTextColor(SSD1306_WHITE);

  if (g_oledPage == 0) {
    // Page 0: Voltage + Current + Freq
    oled.setTextSize(1);
    oled.setCursor(0,0); oled.print("BlueArc  MASTER");
    oled.drawFastHLine(0, 9, 128, SSD1306_WHITE);
    oled.setCursor(0,12);
    oled.printf("V  : %6.1f V", g_m.vrms);
    oled.setCursor(0,22);
    oled.printf("I  : %6.3f A", g_m.irms);
    oled.setCursor(0,32);
    oled.printf("Hz : %6.2f", g_m.freqHz);
    oled.setCursor(0,42);
    oled.printf("P  : %6.1f W", g_m.pActive);
    oled.setCursor(0,52);
    bool fault = g_faultFlags != 0;
    oled.printf("SYS: %s", fault ? "** FAULT **" : "All OK");

  } else if (g_oledPage == 1) {
    // Page 1: Power triangle
    oled.setTextSize(1);
    oled.setCursor(0,0); oled.print("Power Quality");
    oled.drawFastHLine(0, 9, 128, SSD1306_WHITE);
    oled.setCursor(0,12);
    oled.printf("P (act) : %6.1f W", g_m.pActive);
    oled.setCursor(0,22);
    oled.printf("Q (rct) : %6.1f VAR", g_m.pReactive);
    oled.setCursor(0,32);
    oled.printf("S (app) : %6.1f VA", g_m.pApparent);
    oled.setCursor(0,42);
    oled.printf("PF      : %6.3f", g_m.pf);
    oled.setCursor(0,52);
    oled.printf("PF%s", g_m.pf < PF_CRIT ? " !! LOW" : "   OK");

  } else if (g_oledPage == 2) {
    // Page 2: A1 + A2 status
    oled.setTextSize(1);
    oled.setCursor(0,0); oled.print("Slave Status");
    oled.drawFastHLine(0, 9, 128, SSD1306_WHITE);
    oled.setCursor(0,12);
    if (g_a1.valid) {
      unsigned long age1 = (millis() - g_a1.lastRxMs) / 1000;
      oled.printf("A1:%s PIR:%d Tlt:%d %lus",
                  g_a1.relayOK ? "OK" : "TRIP",
                  g_a1.pir, g_a1.tilt, age1);
    } else {
      oled.print("A1: -- WAITING --");
    }
    oled.setCursor(0,22);
    if (g_a1.valid && g_a1.inaPowerW > 0) {
      oled.printf("A1 INA: %.2fW sv%.2fW", g_a1.inaPowerW, g_a1.savingW);
    }
    oled.setCursor(0,32);
    if (g_a2.valid) {
      unsigned long age2 = (millis() - g_a2.lastRxMs) / 1000;
      oled.printf("A2:%s PIR:%d Tlt:%d %lus",
                  g_a2.relayOK ? "OK" : "TRIP",
                  g_a2.pir, g_a2.tilt, age2);
    } else {
      oled.print("A2: -- WAITING --");
    }
    oled.setCursor(0,42);
    oled.printf("Relay: %s", g_relayTripped ? "TRIPPED!" : "OK (line live)");
    oled.setCursor(0,52);
    oled.printf("Faults: 0x%04X", g_faultFlags);

  } else {
    // Page 3: Fault summary
    oled.setTextSize(1);
    oled.setCursor(0,0); oled.print("Fault Summary");
    oled.drawFastHLine(0, 9, 128, SSD1306_WHITE);
    int y = 12;
    if (g_faultFlags == 0) {
      oled.setCursor(0,y); oled.print("No faults :)");
    } else {
      if (g_faultFlags & FAULT_OVERCURRENT) { oled.setCursor(0,y); oled.print("[OC] Overcurrent"); y+=10; }
      if (g_faultFlags & FAULT_OVERVOLTAGE) { oled.setCursor(0,y); oled.print("[OV] Overvoltage"); y+=10; }
      if (g_faultFlags & FAULT_VOLTAGE_SAG) { oled.setCursor(0,y); oled.print("[SAG] Voltage Sag"); y+=10; }
      if (g_faultFlags & FAULT_HB_LOST_A1) { oled.setCursor(0,y); oled.print("[HB] A1 offline"); y+=10; }
      if (g_faultFlags & FAULT_HB_LOST_A2) { oled.setCursor(0,y); oled.print("[HB] A2 offline"); y+=10; }
      if (g_faultFlags & FAULT_TILT_A1)    { oled.setCursor(0,y); oled.print("[TLT] A1 tilt"); y+=10; }
      if (g_faultFlags & FAULT_TILT_A2)    { oled.setCursor(0,y); oled.print("[TLT] A2 tilt"); y+=10; }
      if (g_faultFlags & FAULT_PF_CRIT)    { oled.setCursor(0,y); oled.print("[PF] Low PF"); y+=10; }
    }
  }

  oled.display();
}

// ─────────────────────────────────────────────────────────────────
//  SERIAL STATUS
// ─────────────────────────────────────────────────────────────────
void printSerialStatus() {
  Serial.println("\n┌─────────────── BlueArc MASTER Status ───────────────┐");
  Serial.printf("│ ACS71020 : V=%.1fV  I=%.3fA  P=%.1fW  Q=%.1fVAR  S=%.1fVA\n",
                g_m.vrms, g_m.irms, g_m.pActive, g_m.pReactive, g_m.pApparent);
  Serial.printf("│           PF=%.3f  Hz=%.2f\n", g_m.pf, g_m.freqHz);
  Serial.printf("│ Relay    : %s\n", g_relayTripped ? "⛔ TRIPPED" : "✅ OK (feeder live)");
  Serial.printf("│ A1       : %s  sq=%u  age=%lus  RSSI=%d\n",
                g_a1.valid ? "✅" : "⚠ WAIT",
                g_a1.lastSeq,
                g_a1.valid ? (millis()-g_a1.lastRxMs)/1000 : 0,
                g_a1.rssi);
  Serial.printf("│ A2       : %s  sq=%u  age=%lus\n",
                g_a2.valid ? "✅" : "⚠ WAIT",
                g_a2.lastSeq,
                g_a2.valid ? (millis()-g_a2.lastRxMs)/1000 : 0);
  if (g_faultFlags) {
    Serial.printf("│ FAULTS   : 0x%04X\n", g_faultFlags);
  } else {
    Serial.println("│ Faults   : None ✅");
  }
  Serial.println("└──────────────────────────────────────────────────────┘");
}

// ═════════════════════════════════════════════════════════════════
//  WEB DASHBOARD — HTML
// ═════════════════════════════════════════════════════════════════
static const char MASTER_HTML[] PROGMEM = R"rawhtml(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>BlueArc Master</title>
<style>
  :root{--bg:#0d1117;--card:#161b22;--border:#30363d;--accent:#58a6ff;
        --green:#3fb950;--orange:#d29922;--red:#f85149;--txt:#e6edf3;--muted:#8b949e;
        --blue:#79c0ff}
  *{box-sizing:border-box;margin:0;padding:0}
  body{background:var(--bg);color:var(--txt);font-family:'Segoe UI',system-ui,sans-serif;
       padding:16px;max-width:720px;margin:auto}
  h1{color:var(--accent);font-size:1.4rem;margin-bottom:4px}
  .sub{color:var(--muted);font-size:.8rem;margin-bottom:16px}
  .card{background:var(--card);border:1px solid var(--border);border-radius:8px;
        padding:14px 16px;margin-bottom:12px}
  .card h2{font-size:.75rem;text-transform:uppercase;letter-spacing:.1em;
           color:var(--muted);margin-bottom:10px}
  .grid2{display:grid;grid-template-columns:1fr 1fr;gap:8px}
  .metric{background:#0d1117;border:1px solid var(--border);border-radius:6px;
          padding:10px 12px}
  .metric .lbl{font-size:.72rem;color:var(--muted);margin-bottom:2px}
  .metric .val{font-size:1.2rem;font-weight:700;color:var(--accent)}
  .metric .unit{font-size:.78rem;color:var(--muted);margin-left:3px}
  .row{display:flex;justify-content:space-between;align-items:center;
       padding:5px 0;border-bottom:1px solid var(--border);font-size:.88rem}
  .row:last-child{border-bottom:none}
  .lbl{color:var(--muted)} .ok{color:var(--green)} .warn{color:var(--orange)} .err{color:var(--red)}
  .badge{padding:2px 9px;border-radius:4px;font-size:.78rem;font-weight:600}
  .badge.ok{background:#1a3a22} .badge.warn{background:#3a2e10} .badge.err{background:#3a1010}
  button{background:var(--accent);color:#000;border:none;border-radius:5px;
    padding:7px 18px;font-size:.88rem;font-weight:700;cursor:pointer;margin:4px 4px 4px 0}
  button.danger{background:var(--red);color:#fff}
  button.sim{background:#21262d;color:var(--orange);border:1px solid var(--orange)}
  .resp{margin-top:8px;font-size:.82rem;color:var(--green);min-height:18px}
  #ts{font-size:.75rem;color:var(--muted);text-align:right;margin-bottom:8px}
  .fault-box{background:#1a0808;border:1px solid var(--red);border-radius:6px;
             padding:10px;margin-top:6px;font-size:.82rem;color:var(--red)}
  .node-card{border-left:3px solid var(--accent);padding-left:12px;margin-bottom:8px}
  .node-card.a2{border-left-color:var(--blue)}
  .node-card.fault{border-left-color:var(--red)}
</style>
</head>
<body>
<h1>⚡ BlueArc Systems — Master Node</h1>
<p class="sub">Distribution Transformer Monitor | Smart Line Protection</p>
<div id="ts">Loading…</div>

<div class="card">
  <h2>⚡ Live Power Measurements (ACS71020)</h2>
  <div class="grid2">
    <div class="metric"><div class="lbl">Voltage</div>
      <span class="val" id="m-v">—</span><span class="unit">V</span></div>
    <div class="metric"><div class="lbl">Current</div>
      <span class="val" id="m-i">—</span><span class="unit">A</span></div>
    <div class="metric"><div class="lbl">Active Power</div>
      <span class="val" id="m-p">—</span><span class="unit">W</span></div>
    <div class="metric"><div class="lbl">Reactive Power</div>
      <span class="val" id="m-q">—</span><span class="unit">VAR</span></div>
    <div class="metric"><div class="lbl">Apparent Power</div>
      <span class="val" id="m-s">—</span><span class="unit">VA</span></div>
    <div class="metric"><div class="lbl">Power Factor</div>
      <span class="val" id="m-pf">—</span></div>
    <div class="metric"><div class="lbl">Frequency</div>
      <span class="val" id="m-hz">—</span><span class="unit">Hz</span></div>
    <div class="metric"><div class="lbl">Relay</div>
      <span class="val" id="m-rel">—</span></div>
  </div>
</div>

<div class="card" id="fault-card" style="display:none">
  <h2>⚠ Active Faults</h2>
  <div class="fault-box" id="fault-list">None</div>
</div>

<div class="card">
  <h2>Slave Nodes</h2>
  <div class="node-card" id="nc-a1">
    <div class="row"><span class="lbl">A1</span><span id="a1-status">—</span></div>
    <div class="row"><span class="lbl">PIR / LDR / Tilt</span><span id="a1-sensors">—</span></div>
    <div class="row"><span class="lbl">INA260 Power</span><span id="a1-power">—</span></div>
    <div class="row"><span class="lbl">Saving / Cost</span><span id="a1-saving">—</span></div>
    <div class="row"><span class="lbl">Last packet age</span><span id="a1-age">—</span></div>
  </div>
  <div class="node-card a2" id="nc-a2">
    <div class="row"><span class="lbl">A2</span><span id="a2-status">—</span></div>
    <div class="row"><span class="lbl">PIR / LDR / Tilt</span><span id="a2-sensors">—</span></div>
    <div class="row"><span class="lbl">Last packet age</span><span id="a2-age">—</span></div>
  </div>
</div>

<div class="card">
  <h2>Fault Simulation (Pushbuttons / Web)</h2>
  <p style="font-size:.8rem;color:var(--muted);margin-bottom:8px">
    Simulate faults via web — same as pressing physical PB1–PB3.
  </p>
  <button class="sim" onclick="cmd('SIM_OC','sim-r')">PB1 Overcurrent</button>
  <button class="sim" onclick="cmd('SIM_OV','sim-r')">PB2 Overvoltage</button>
  <button class="sim" onclick="cmd('SIM_SAG','sim-r')">PB3 Voltage Sag</button>
  <button class="danger" onclick="if(confirm('Clear all faults?'))cmd('RESET','sim-r')">
    PB4 Reset All
  </button>
  <div class="resp" id="sim-r"></div>
</div>

<script>
const FL={
  0:'OVERCURRENT', 1:'OVERVOLTAGE', 2:'VOLTAGE SAG', 3:'OVERTEMP',
  4:'A1 HB LOST', 5:'A2 HB LOST', 6:'A1 TILT', 7:'A2 TILT',
  8:'RELAY TRIP', 9:'PF CRITICAL'
};
const ge=id=>document.getElementById(id);
const badge=(ok,a,b)=>`<span class="badge ${ok?'ok':'err'}">${ok?a:b}</span>`;
const fix=(v,d)=>v!=null?v.toFixed(d):'—';

async function refresh(){
  try{
    const d=await fetch('/api/status').then(r=>r.json());
    ge('ts').textContent='Updated: '+new Date().toLocaleTimeString();

    ge('m-v').textContent=fix(d.vrms,1);
    ge('m-i').textContent=fix(d.irms,3);
    ge('m-p').textContent=fix(d.pActive,1);
    ge('m-q').textContent=fix(d.pReactive,1);
    ge('m-s').textContent=fix(d.pApparent,1);
    ge('m-pf').innerHTML=`<span class="${d.pf<0.7?'warn':'ok'}">${fix(d.pf,3)}</span>`;
    ge('m-hz').textContent=fix(d.freqHz,2);
    ge('m-rel').innerHTML=badge(!d.relayTripped,'✅ OK','⛔ TRIPPED');

    // Faults
    let flist=[]; let f=d.faultFlags;
    for(let b=0;b<16;b++) if(f&(1<<b)) flist.push('⚠ '+FL[b]);
    if(flist.length){
      ge('fault-card').style.display='block';
      ge('fault-list').innerHTML=flist.join('<br>');
    } else {
      ge('fault-card').style.display='none';
    }

    // A1
    const a1=d.a1;
    if(a1.valid){
      ge('a1-status').innerHTML=badge(!a1.faultFlags,'Online OK','Online ⚠')
        +` RSSI: ${a1.rssi}dBm`;
      ge('a1-sensors').innerHTML=
        badge(!a1.pir,'No motion','🚶 Motion')+' '+
        badge(!a1.isNight,'☀ Day','🌙 Night')+' '+
        badge(!a1.tilt,'OK','⚠ Tilt');
      ge('a1-power').textContent=`${fix(a1.inaCurrentA*1000,1)}mA  ${fix(a1.inaPowerW,2)}W`;
      ge('a1-saving').textContent=`${fix(a1.savingW,2)}W saved  ₹${fix(a1.costSaved,4)}`;
      ge('a1-age').textContent=a1.age_s+'s ago';
    } else {
      ge('a1-status').innerHTML='<span class="err">⚠ Not received yet</span>';
    }

    // A2
    const a2=d.a2;
    if(a2.valid){
      ge('a2-status').innerHTML=badge(!a2.faultFlags,'Online OK','Online ⚠')
        +` RSSI: ${a2.rssi}dBm`;
      ge('a2-sensors').innerHTML=
        badge(!a2.pir,'No motion','🚶 Motion')+' '+
        badge(!a2.isNight,'☀ Day','🌙 Night')+' '+
        badge(!a2.tilt,'OK','⚠ Tilt');
      ge('a2-age').textContent=a2.age_s+'s ago';
    } else {
      ge('a2-status').innerHTML='<span class="err">⚠ Not received yet</span>';
    }
  } catch(e){ge('ts').textContent='Connection error…';}
}

async function cmd(c,rid){
  ge(rid).textContent='Sending…';
  try{
    const r=await fetch('/api/cmd?c='+encodeURIComponent(c));
    ge(rid).textContent=await r.text();
    setTimeout(refresh,400);
  }catch(e){ge(rid).textContent='Error: '+e;}
}
refresh();setInterval(refresh,2000);
</script>
</body></html>
)rawhtml";

// ─────────────────────────────────────────────────────────────────
void handleApiStatus() {
  String j = "{";
  j += "\"vrms\":"     + String(g_m.vrms, 1)      + ",";
  j += "\"irms\":"     + String(g_m.irms, 3)      + ",";
  j += "\"pActive\":"  + String(g_m.pActive, 1)   + ",";
  j += "\"pReactive\":"+ String(g_m.pReactive, 1) + ",";
  j += "\"pApparent\":"+ String(g_m.pApparent, 1) + ",";
  j += "\"pf\":"       + String(g_m.pf, 3)        + ",";
  j += "\"freqHz\":"   + String(g_m.freqHz, 2)    + ",";
  j += "\"relayTripped\":" + String(g_relayTripped ? "true" : "false") + ",";
  j += "\"faultFlags\":" + String(g_faultFlags) + ",";

  // A1
  j += "\"a1\":{";
  j += "\"valid\":"   + String(g_a1.valid ? "true" : "false") + ",";
  j += "\"age_s\":"   + String(g_a1.valid ? (millis()-g_a1.lastRxMs)/1000 : 255) + ",";
  j += "\"rssi\":"    + String(g_a1.rssi) + ",";
  j += "\"pir\":"     + String(g_a1.pir ? "true" : "false") + ",";
  j += "\"isNight\":" + String(g_a1.isNight ? "true" : "false") + ",";
  j += "\"tilt\":"    + String(g_a1.tilt ? "true" : "false") + ",";
  j += "\"relayOK\":" + String(g_a1.relayOK ? "true" : "false") + ",";
  j += "\"faultFlags\":" + String(g_a1.faultFlags) + ",";
  j += "\"inaCurrentA\":" + String(g_a1.inaCurrentA, 3) + ",";
  j += "\"inaPowerW\":"   + String(g_a1.inaPowerW,   2) + ",";
  j += "\"savingW\":"     + String(g_a1.savingW,      2) + ",";
  j += "\"costSaved\":"   + String(g_a1.costSaved,    4) + "},";

  // A2
  j += "\"a2\":{";
  j += "\"valid\":"   + String(g_a2.valid ? "true" : "false") + ",";
  j += "\"age_s\":"   + String(g_a2.valid ? (millis()-g_a2.lastRxMs)/1000 : 255) + ",";
  j += "\"rssi\":"    + String(g_a2.rssi) + ",";
  j += "\"pir\":"     + String(g_a2.pir ? "true" : "false") + ",";
  j += "\"isNight\":" + String(g_a2.isNight ? "true" : "false") + ",";
  j += "\"tilt\":"    + String(g_a2.tilt ? "true" : "false") + ",";
  j += "\"relayOK\":" + String(g_a2.relayOK ? "true" : "false") + ",";
  j += "\"faultFlags\":" + String(g_a2.faultFlags) + "}";
  j += "}";
  httpServer.send(200, "application/json", j);
}

void handleApiCmd() {
  if (!httpServer.hasArg("c")) { httpServer.send(400,"text/plain","ERR"); return; }
  String cmd = httpServer.arg("c"); cmd.trim();
  String resp;

  if (cmd == "SIM_OC")  { g_m.simOC  = true; g_faultFlags |= FAULT_OVERCURRENT; resp = "OK Overcurrent simulated"; }
  else if (cmd == "SIM_OV")  { g_m.simOV  = true; g_faultFlags |= FAULT_OVERVOLTAGE; resp = "OK Overvoltage simulated"; }
  else if (cmd == "SIM_SAG") { g_m.simSag = true; g_faultFlags |= FAULT_VOLTAGE_SAG; resp = "OK Voltage sag simulated"; }
  else if (cmd == "RESET") { clearRelay(); g_faultFlags = 0; resp = "OK All faults cleared"; }
  else { resp = "ERR. Valid: SIM_OC SIM_OV SIM_SAG RESET"; }

  Serial.println("[Web CMD] " + cmd + " -> " + resp);
  httpServer.send(200, "text/plain", resp);
}

// ═════════════════════════════════════════════════════════════════
//  SETUP
// ═════════════════════════════════════════════════════════════════
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n══════════════════════════════════════════════");
  Serial.println("  BlueArc Systems — MASTER NODE — Booting");
  Serial.println("══════════════════════════════════════════════");

  // ── GPIOs ─────────────────────────────────────────────────────
  pinMode(PIN_RELAY,   OUTPUT); digitalWrite(PIN_RELAY, HIGH);  // HIGH = normal
  pinMode(PIN_PB1,     INPUT_PULLUP);
  pinMode(PIN_PB2,     INPUT_PULLUP);
  pinMode(PIN_PB3,     INPUT_PULLUP);
  pinMode(PIN_PB4,     INPUT_PULLUP);  // GPIO35 input-only — OK
  pinMode(PIN_LED_GRN, OUTPUT); digitalWrite(PIN_LED_GRN, LOW);
  pinMode(PIN_LED_ORG, OUTPUT); digitalWrite(PIN_LED_ORG, HIGH);  // Orange on = booting
  pinMode(PIN_LED_BLU, OUTPUT); digitalWrite(PIN_LED_BLU, LOW);

  // ── I2C ───────────────────────────────────────────────────────
  Wire.begin(21, 22);
  Wire.setClock(400000);

  // ── OLED ──────────────────────────────────────────────────────
  if (!oled.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.println("[OLED] INIT FAILED");
  } else {
    oled.clearDisplay();
    oled.setTextSize(1); oled.setTextColor(SSD1306_WHITE);
    oled.setCursor(0,0);  oled.println("BlueArc Systems");
    oled.setCursor(0,12); oled.println("  MASTER NODE");
    oled.setCursor(0,26); oled.println("  Initializing...");
    oled.display();
    Serial.println("[OLED] Init OK");
  }

  // ── ACS71020 ping ─────────────────────────────────────────────
  Wire.beginTransmission(ACS_ADDR);
  if (Wire.endTransmission() == 0) {
    g_m.acsOk = true;
    Serial.println("[ACS71020] Found at 0x60");
  } else {
    Serial.println("[ACS71020] NOT FOUND — check SS/ADDR tied to 3.3V");
  }

  // ── LoRa ──────────────────────────────────────────────────────
  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_NSS);
  LoRa.setSPI(SPI);
  LoRa.setPins(LORA_NSS, LORA_RST, LORA_DIO0);
  if (!LoRa.begin(LORA_FREQ)) {
    Serial.println("[LoRa] INIT FAILED");
    oled.clearDisplay(); oled.setCursor(0,0); oled.print("LoRa FAIL"); oled.display();
  } else {
    LoRa.setSpreadingFactor(LORA_SF);
    LoRa.setSignalBandwidth(LORA_BW);
    LoRa.setSyncWord(LORA_SYNC);
    LoRa.setTxPower(LORA_PWR);
    LoRa.enableCrc();
    Serial.printf("[LoRa] Ready — 433MHz SF%d\n", LORA_SF);
  }

  // ── WiFi AP ───────────────────────────────────────────────────
  WiFi.mode(WIFI_AP);
  WiFi.softAP(WIFI_AP_SSID, WIFI_AP_PASS);
  IPAddress ip = WiFi.softAPIP();
  Serial.printf("[WiFi] AP: %s  http://%s\n", WIFI_AP_SSID, ip.toString().c_str());

  httpServer.on("/",           HTTP_GET, []{ httpServer.send_P(200,"text/html",MASTER_HTML); });
  httpServer.on("/api/status", HTTP_GET, handleApiStatus);
  httpServer.on("/api/cmd",    HTTP_GET, handleApiCmd);
  httpServer.onNotFound([]{ httpServer.send(404,"text/plain","Not found"); });
  httpServer.begin();

  digitalWrite(PIN_LED_ORG, LOW);
  digitalWrite(PIN_LED_GRN, HIGH);
  Serial.println("[Boot] Master ready.\n");
}

// ═════════════════════════════════════════════════════════════════
//  LOOP
// ═════════════════════════════════════════════════════════════════

unsigned long g_lastTxMs    = 0;
unsigned long g_lastOledMs  = 0;
unsigned long g_lastAcsMs   = 0;

void loop() {
  unsigned long now = millis();

  // ── 1. READ ACS71020 (every 200ms) ───────────────────────────────
  if (now - g_lastAcsMs >= 200) {
    g_lastAcsMs = now;
    if (g_m.acsOk) {
      readACS71020();
      updateFrequency();
    }
  }

  // ── 2. CHECK PUSHBUTTONS ──────────────────────────────────────────
  checkButtons();

  // ── 3. PROCESS FAULTS ────────────────────────────────────────────
  processFaults();

  // ── 4. RECEIVE FROM A1 ───────────────────────────────────────────
  int pkt = LoRa.parsePacket();
  if (pkt > 0) {
    String raw = "";
    while (LoRa.available()) raw += (char)LoRa.read();
    int rssi = LoRa.packetRssi();
    // Blue LED flash on RX
    digitalWrite(PIN_LED_BLU, HIGH); delay(2); digitalWrite(PIN_LED_BLU, LOW);
    parseLoRaPacket(raw, rssi);
  }

  // ── 5. TRANSMIT TO RECEIVING STATION (every 500ms) ───────────────
  if (now - g_lastTxMs >= 500) {
    g_lastTxMs = now;
    transmitMasterPacket();
  }

  // ── 6. OLED UPDATE (page rotation every 3s) ──────────────────────
  if (now - g_lastOledMs >= 100) {    // refresh display every 100ms
    g_lastOledMs = now;
    if (now - g_lastOledSwitch >= OLED_PAGE_MS) {
      g_lastOledSwitch = now;
      g_oledPage = (g_oledPage + 1) % 4;
    }
    drawOLED();
  }

  // ── 7. SERIAL STATUS (every 3s) ──────────────────────────────────
  if (now - g_lastSerialMs >= 3000) {
    g_lastSerialMs = now;
    printSerialStatus();
  }

  // ── 8. WEB SERVER ────────────────────────────────────────────────
  httpServer.handleClient();

  delay(5);
}
