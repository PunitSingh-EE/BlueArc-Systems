// ════════════════════════════════════════════════════════════════
//  BlueArc Systems — Slave Node
//  File: slave.ino  |  ESP32-C3 Super Mini
//
//  Libraries required:
//    - LoRa             by Sandeep Mistry
//    - ArduinoJson      by Benoit Blanchon   (v6.x)
//    - Adafruit NeoPixel by Adafruit
//    - Adafruit INA260  by Adafruit           (A1 only)
//
//  Board: "ESP32C3 Dev Module"
// ════════════════════════════════════════════════════════════════

#include <Arduino.h>
#include <Wire.h>
#include <Preferences.h>
#include <Adafruit_NeoPixel.h>

#include "config.h"
#include "lora_comms.h"
#include "wifi_config.h"

#if ENABLE_INA260
  #include <Adafruit_INA260.h>
#endif

// ═════════════════════════════════════════════════════════════════
//  GLOBALS
// ═════════════════════════════════════════════════════════════════

Preferences prefs;

// Config — loaded from flash on boot
char  g_slaveId[4]       = SLAVE_ID;
float g_lat              = DEFAULT_LAT;
float g_lon              = DEFAULT_LON;
bool  g_bypassIntentFlag = false;
bool  g_relayTripped     = false;

NodeData g_node;

Adafruit_NeoPixel stripStreet(NUM_LEDS_STREET, PIN_WS2812_STREET, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel stripStatus(NUM_LEDS_STATUS,  PIN_WS2812_STATUS, NEO_GRB + NEO_KHZ800);

unsigned long g_lastTxMs       = 0;
unsigned long g_pirTriggeredMs = 0;
unsigned long g_lastSerialMs   = 0;
unsigned long g_pirLastHighMs  = 0;
bool          g_pirActive      = false;
bool          g_pirDebounced   = false;
#define PIR_DEBOUNCE_MS  500

uint8_t g_currentBrightness = 0;
uint8_t g_targetBrightness  = 0;

float g_inaVoltageV  = 0.0f;
float g_inaCurrentMA = 0.0f;
float g_totalSavingWh  = 0.0f;
unsigned long g_lastEnergyMs = 0;
const float   ENERGY_RATE_RS = 7.0f;  // ₹/kWh

float g_tiltOffset    = 0.0f;
bool  g_tiltCalibrated = false;

#if ENABLE_INA260
  Adafruit_INA260 ina260;
#endif

#define MPU_ADDR       0x68
#define MPU_PWR_MGMT_1 0x6B
#define MPU_ACCEL_XOUT 0x3B

// ═════════════════════════════════════════════════════════════════
//  RELAY CONTROL
// ═════════════════════════════════════════════════════════════════

void tripRelay(const char* reason) {
  if (g_relayTripped) return;
  digitalWrite(PIN_RELAY, HIGH);
  g_relayTripped = true;
  g_node.faultFlags |= FAULT_RELAY_TRIP;
  Serial.printf("[RELAY] TRIPPED — %s\n", reason);
}

void clearRelay() {
  digitalWrite(PIN_RELAY, LOW);
  g_relayTripped = false;
  g_node.faultFlags &= ~FAULT_RELAY_TRIP;
  g_bypassIntentFlag = false;
  prefs.begin(NVS_NAMESPACE, false);
  prefs.putBool(NVS_KEY_BYPASS_F, false);
  prefs.end();
  Serial.println("[RELAY] Cleared — line restored");
}

// ═════════════════════════════════════════════════════════════════
//  SENSOR INIT & READ
// ═════════════════════════════════════════════════════════════════

void initMPU6050() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(MPU_PWR_MGMT_1);
  Wire.write(0x00);
  Wire.endTransmission();
  delay(100);

  Serial.println("[MPU6050] Calibrating — keep device still...");
  float sum = 0;
  for (int i = 0; i < 20; i++) {
    // inline raw read for calibration
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(MPU_ACCEL_XOUT);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, 6, true);
    int16_t ax = (Wire.read() << 8) | Wire.read();
    int16_t ay = (Wire.read() << 8) | Wire.read();
    int16_t az = (Wire.read() << 8) | Wire.read();
    float axG = ax / MPU_ACCEL_SCALE;
    float ayG = ay / MPU_ACCEL_SCALE;
    float azG = az / MPU_ACCEL_SCALE;
    sum += degrees(acos(constrain(azG / sqrt(axG*axG + ayG*ayG + azG*azG), -1.0f, 1.0f)));
    delay(20);
  }
  g_tiltOffset     = sum / 20.0f;
  g_tiltCalibrated = true;
  Serial.printf("[MPU6050] OK — offset=%.1f°\n", g_tiltOffset);
}

float readMPU6050() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(MPU_ACCEL_XOUT);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 6, true);
  int16_t ax = (Wire.read() << 8) | Wire.read();
  int16_t ay = (Wire.read() << 8) | Wire.read();
  int16_t az = (Wire.read() << 8) | Wire.read();
  float axG = ax / MPU_ACCEL_SCALE;
  float ayG = ay / MPU_ACCEL_SCALE;
  float azG = az / MPU_ACCEL_SCALE;
  float raw = degrees(acos(constrain(azG / sqrt(axG*axG + ayG*ayG + azG*azG), -1.0f, 1.0f)));
  float cal = raw - g_tiltOffset;
  if (cal < 0) cal = -cal;
  return cal;
}

#if ENABLE_INA260
void initINA260() {
  if (!ina260.begin(INA260_ADDR, &Wire)) {
    Serial.println("[INA260] INIT FAILED — check wiring!");
    return;
  }
  ina260.setAveragingCount(INA260_COUNT_16);
  ina260.setVoltageConversionTime(INA260_TIME_1_1_ms);
  ina260.setCurrentConversionTime(INA260_TIME_1_1_ms);
  Serial.println("[INA260] Init OK");
}

void readINA260() {
  g_inaCurrentMA     = ina260.readCurrent();
  g_inaVoltageV      = ina260.readBusVoltage() / 1000.0f;
  g_node.inaCurrentA = g_inaCurrentMA / 1000.0f;
  g_node.inaPowerW   = ina260.readPower() / 1000.0f;

  g_node.savingW = BASELINE_W - g_node.inaPowerW;
  if (g_node.savingW < 0) g_node.savingW = 0;

  unsigned long now = millis();
  float dtHours = (now - g_lastEnergyMs) / 3600000.0f;
  g_totalSavingWh += g_node.savingW * dtHours;
  g_lastEnergyMs = now;
  g_node.costSaved = (g_totalSavingWh / 1000.0f) * ENERGY_RATE_RS;
}
#endif

// ═════════════════════════════════════════════════════════════════
//  WS2812 CONTROL
// ═════════════════════════════════════════════════════════════════

void updateStreetlight() {
  static unsigned long lastFadeMs = 0;
  static uint8_t lastShownBrightness = 255;

  bool needFade = (g_currentBrightness != g_targetBrightness);

  if (needFade) {
    if (millis() - lastFadeMs < FADE_STEP_MS) return;
    lastFadeMs = millis();
    if (g_currentBrightness < g_targetBrightness) {
      uint8_t stepUp = 5;
      g_currentBrightness = (g_currentBrightness + stepUp < g_targetBrightness)
                            ? g_currentBrightness + stepUp : g_targetBrightness;
    } else {
      uint8_t step = 5;
      g_currentBrightness = (g_currentBrightness > g_targetBrightness + step)
                            ? g_currentBrightness - step : g_targetBrightness;
    }
  }

  if (g_currentBrightness == 0) {
    for (int i = 0; i < NUM_LEDS_STREET; i++)
      stripStreet.setPixelColor(i, 0);
    stripStreet.setBrightness(255);
  } else {
    stripStreet.setBrightness(g_currentBrightness);
    for (int i = 0; i < NUM_LEDS_STREET; i++)
      stripStreet.setPixelColor(i, stripStreet.Color(255, 255, 255));
  }

  if (needFade || g_currentBrightness != lastShownBrightness) {
    stripStreet.show();
    lastShownBrightness = g_currentBrightness;
  }
}

void setStreetlightTarget(uint8_t brightness) {
  g_targetBrightness = brightness;
}

void updateLightingLogic() {
  bool day    = !g_node.isNight;
  bool motion = g_node.pir;

  if (day) {
    setStreetlightTarget(BRIGHTNESS_OFF);
    return;
  }
  if (motion) {
    if (!g_pirActive) {
      g_pirTriggeredMs = millis();
      g_pirActive = true;
    }
    setStreetlightTarget(BRIGHTNESS_FULL);
  } else if (g_pirActive) {
    if (millis() - g_pirLastHighMs > PIR_HOLD_MS) {
      g_pirActive = false;
      setStreetlightTarget(BRIGHTNESS_SAVER);
    }
  } else {
    setStreetlightTarget(BRIGHTNESS_SAVER);
  }
}

// ── Status LED — only LED[0] flashes, rest stay as status colour ──
// LED[0]  = TX blink (blue pulse on send)
// LED[1]  = status colour (green/orange/red)
// LED[2..7] = off (reserved)
void setStatusLED(uint8_t r, uint8_t g, uint8_t b) {
  stripStatus.setBrightness(25);
  // LED 0: dim version of status colour (will get overridden on TX pulse)
  stripStatus.setPixelColor(0, stripStatus.Color(r/4, g/4, b/4));
  // LED 1: full status colour
  stripStatus.setPixelColor(1, stripStatus.Color(r, g, b));
  // LED 2-7: off
  for (int i = 2; i < NUM_LEDS_STATUS; i++)
    stripStatus.setPixelColor(i, 0);
  stripStatus.show();
}

// ── Quick blue flash on LED[0] only — called during TX ────────────
void txBlinkLED0() {
  stripStatus.setBrightness(25);
  stripStatus.setPixelColor(0, stripStatus.Color(0, 0, 255));  // LED 0 = blue
  stripStatus.show();
}

// ═════════════════════════════════════════════════════════════════
//  FAULT PROCESSING
// ═════════════════════════════════════════════════════════════════

void processFaults() {
  if (g_node.tilt) {
    g_node.faultFlags |= FAULT_TILT;
    if (g_node.tiltAngle > 60.0f) tripRelay("Severe tilt — pole may have fallen");
  } else {
    g_node.faultFlags &= ~FAULT_TILT;
  }

#if IS_RELAY_NODE
  if (checkA2Timeout()) {
    if (!g_a2Cache.bypassIntent) {
      tripRelay("A2 heartbeat timeout — possible wire break A1→A2");
      Serial.println("[FAULT] A1→A2 wire break suspected");
    } else {
      Serial.println("[INFO] A2 silent — bypass intent set, treating as DEVICE FAULT");
    }
  }
#endif

  if (g_node.faultFlags != 0 || g_relayTripped) {
    setStatusLED(255, 80, 0);
  } else if (!g_node.isNight) {
    setStatusLED(0, 255, 0);
  } else {
    setStatusLED(0, 80, 0);
  }
}

// ═════════════════════════════════════════════════════════════════
//  VERBOSE SERIAL STATUS
// ═════════════════════════════════════════════════════════════════

void printSerialStatus() {
  // NOTE: uses g_slaveId (runtime value) NOT compile-time SLAVE_ID
  //       so changing ID via web dashboard shows correctly here
  Serial.printf("\n┌─────────────── BlueArc %s Status ───────────────┐\n", g_slaveId);
  Serial.printf("│ LDR      : %s  (ADC=%d)\n",
                g_node.isNight ? "🌙 NIGHT" : "☀  DAY  ", analogRead(PIN_LDR));
  Serial.printf("│ PIR      : %s\n",
                g_node.pir ? "🚶 MOTION" : "   No motion");
  Serial.printf("│ Tilt     : %.1f°  %s\n",
                g_node.tiltAngle, g_node.tilt ? "⚠ ALARM" : "OK");
  Serial.printf("│ Light    : %d%%\n", (g_currentBrightness * 100) / 255);

#if ENABLE_INA260
  Serial.printf("│ INA260   : V=%.3fV  I=%.1fmA  P=%.3fW\n",
                g_inaVoltageV, g_inaCurrentMA, g_node.inaPowerW);
  Serial.printf("│ Saving   : %.3fW  ₹%.4f saved so far\n",
                g_node.savingW, g_node.costSaved);
#endif

  Serial.printf("│ Relay    : %s\n", g_relayTripped ? "⛔ TRIPPED" : "✅ OK");
  Serial.printf("│ Bypass   : %s\n",
                g_node.bypassActive ? "🔧 ACTIVE" : "Normal");

#if IS_RELAY_NODE
  if (g_a2Cache.valid) {
    Serial.printf("│ A2 last  : %lus ago  RSSI=%d  sq=%u  F=0x%02X  %s\n",
                  (millis() - g_a2Cache.lastRxMs) / 1000,
                  g_a2Cache.rssi, g_a2Cache.lastSeq, g_a2Cache.faultFlags,
                  (millis() - g_a2Cache.lastRxMs > HB_TIMEOUT_MS) ? "⚠ TIMEOUT" : "✅ Alive");
  } else {
    Serial.println("│ A2       : ⏳ Waiting...");
  }
#endif

  if (g_node.faultFlags) {
    Serial.printf("│ FAULTS   : 0x%02X → ", g_node.faultFlags);
    if (g_node.faultFlags & FAULT_OVERCURRENT)  Serial.print("[OC] ");
    if (g_node.faultFlags & FAULT_OVERVOLTAGE)  Serial.print("[OV] ");
    if (g_node.faultFlags & FAULT_VOLTAGE_SAG)  Serial.print("[SAG] ");
    if (g_node.faultFlags & FAULT_OVERTEMP)     Serial.print("[TEMP] ");
    if (g_node.faultFlags & FAULT_HB_LOST)      Serial.print("[HB_LOST] ");
    if (g_node.faultFlags & FAULT_TILT)         Serial.print("[TILT] ");
    if (g_node.faultFlags & FAULT_RELAY_TRIP)   Serial.print("[TRIP] ");
    Serial.println();
  } else {
    Serial.println("│ Faults   : None ✅");
  }

  Serial.println("└──────────────────────────────────────────────────────┘");
}

// ═════════════════════════════════════════════════════════════════
//  SETUP
// ═════════════════════════════════════════════════════════════════

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n══════════════════════════════════");
  Serial.printf("  BlueArc Slave %s  — Booting\n", SLAVE_ID);
  Serial.println("══════════════════════════════════");

  pinMode(PIN_PIR,       INPUT);
  pinMode(PIN_LDR,       INPUT);
  pinMode(PIN_RELAY,     OUTPUT);
  pinMode(PIN_BYPASS_SW, INPUT_PULLUP);
  digitalWrite(PIN_RELAY, LOW);

  // Load config from flash
  prefs.begin(NVS_NAMESPACE, true);
  String savedId  = prefs.getString(NVS_KEY_ID,  SLAVE_ID);
  float  savedLat = prefs.getFloat(NVS_KEY_LAT,  DEFAULT_LAT);
  float  savedLon = prefs.getFloat(NVS_KEY_LON,  DEFAULT_LON);
  g_bypassIntentFlag = prefs.getBool(NVS_KEY_BYPASS_F, false);
  prefs.end();

  savedId.toCharArray(g_slaveId, 4);
  g_lat = savedLat;
  g_lon = savedLon;

  // Print using runtime g_slaveId — reflects any stored change
  Serial.printf("[Config] ID=%s  LAT=%.4f  LON=%.4f\n", g_slaveId, g_lat, g_lon);
  Serial.printf("[Config] TX every %dms, wire-break timeout %dms\n",
                HEARTBEAT_TX_MS, HB_TIMEOUT_MS);

  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  Wire.setClock(400000);

  initMPU6050();

  // WS2812 init
  stripStreet.begin();
  stripStatus.begin();
  stripStreet.setBrightness(255);
  for (int i = 0; i < NUM_LEDS_STREET; i++)
    stripStreet.setPixelColor(i, 0);
  stripStreet.show();
  stripStatus.setBrightness(128);
  for (int i = 0; i < NUM_LEDS_STATUS; i++)
    stripStatus.setPixelColor(i, 0);
  stripStatus.show();

  g_currentBrightness = 0;
  g_targetBrightness  = 0;

  // Boot indicator: only LED[0] blue
  txBlinkLED0();

#if ENABLE_INA260
  initINA260();
  g_lastEnergyMs = millis();
#endif
  delay(500);

  if (!initLoRa()) {
    setStatusLED(255, 0, 0);
  }

  initWiFiDashboard();

  setStatusLED(0, 255, 0);
  Serial.printf("[Boot] %s ready. All systems go.\n", g_slaveId);
  Serial.println("══════════════════════════════════\n");

  g_lastTxMs     = millis();
  g_lastEnergyMs = millis();
  g_lastSerialMs = millis();

  int ldrBoot = analogRead(PIN_LDR);
  Serial.printf("[LDR] Boot reading: %d  (%s)\n",
                ldrBoot, ldrBoot > LDR_DAY_THRESHOLD ? "NIGHT" : "DAY");
}

// ═════════════════════════════════════════════════════════════════
//  LOOP
// ═════════════════════════════════════════════════════════════════

void loop() {

  // ── 1. READ SENSORS ──────────────────────────────────────────────
  if (digitalRead(PIN_PIR) == HIGH) {
    g_pirLastHighMs = millis();
    g_pirDebounced  = true;
  } else if (millis() - g_pirLastHighMs > PIR_DEBOUNCE_MS) {
    g_pirDebounced  = false;
  }
  g_node.pir = g_pirDebounced;

  int ldrVal = analogRead(PIN_LDR);
  if (g_node.isNight) {
    if (ldrVal < LDR_DAY_THRESHOLD)   g_node.isNight = false;
  } else {
    if (ldrVal > LDR_NIGHT_THRESHOLD) g_node.isNight = true;
  }

  g_node.tiltAngle = readMPU6050();
  g_node.tilt      = (g_node.tiltAngle > TILT_THRESHOLD);
  g_node.bypassActive = (digitalRead(PIN_BYPASS_SW) == LOW);
  g_node.bypassIntent = g_bypassIntentFlag;
  g_node.relayOK   = !g_relayTripped;

#if ENABLE_INA260
  readINA260();
#endif

  // ── 2. LIGHTING ───────────────────────────────────────────────────
  updateLightingLogic();
  updateStreetlight();

  // ── 3. FAULTS ────────────────────────────────────────────────────
  processFaults();

  // ── 4. RECEIVE A2 (A1 only) ──────────────────────────────────────
#if IS_RELAY_NODE
  receiveA2Packet();
#endif

  // ── 5. TRANSMIT (every HEARTBEAT_TX_MS = 500ms) ──────────────────
  if (millis() - g_lastTxMs >= HEARTBEAT_TX_MS) {
    g_lastTxMs = millis();

    // Only LED[0] blinks blue during TX — all other LEDs keep their colour
    txBlinkLED0();
    sendPacket(g_node, g_slaveId);
    delay(5);

    // Restore LED[0] to dim status colour
    if (g_node.faultFlags != 0 || g_relayTripped) setStatusLED(255, 80, 0);
    else if (g_node.isNight)                       setStatusLED(0, 80, 0);
    else                                           setStatusLED(0, 255, 0);
  }

  // ── 6. SERIAL STATUS (every 2s) ───────────────────────────────────
  if (millis() - g_lastSerialMs >= 2000) {
    g_lastSerialMs = millis();
    printSerialStatus();
  }

  // ── 7. INCOMING COMMANDS FROM MASTER (A2 only) ───────────────────
#if !IS_RELAY_NODE
  int pktSize = LoRa.parsePacket();
  if (pktSize > 0) {
    String raw = "";
    while (LoRa.available()) raw += (char)LoRa.read();
    StaticJsonDocument<128> cmd;
    if (!deserializeJson(cmd, raw)) {
      const char* dst = cmd["dst"] | "";
      if (strcmp(dst, g_slaveId) == 0) {
        const char* action = cmd["action"] | "";
        if (strcmp(action, "TRIP") == 0)  tripRelay("Master command");
        if (strcmp(action, "CLEAR") == 0) clearRelay();
      }
    }
  }
#endif

  wifiHandleClient();
  delay(5);
}
