// ════════════════════════════════════════════════════════════════
//  BlueArc Systems — RECEIVING STATION (WireWhisper V1 repurpose)
//  ESP32 WROOM-32 DevKit
//
//  Hardware:
//    • TFT ST7789 240×320 (VSPI) — SCADA display
//    • SD card (VSPI, separate CS) — logs to /BLUEARC_DATA/
//    • DS3231 RTC (I2C) — timestamps
//    • LoRa SX1278 433MHz (HSPI)
//    • 5 status LEDs:
//        GPIO25 — LED_BLUE    (LoRa RX activity)
//        GPIO32 — LED_GRN_M   (Master OK — green)
//        GPIO26 — LED_ORG_M   (Master fault — orange)
//        GPIO16 — LED_GRN_S   (Slaves OK — green)
//        GPIO17 — LED_ORG_S   (Slave fault/offline — orange)
//
//  Libraries required:
//    - LoRa             by Sandeep Mistry
//    - ArduinoJson      by Benoit Blanchon (v6.x)
//    - Adafruit GFX     by Adafruit
//    - Adafruit ST7789  by Adafruit
//    - RTClib           by Adafruit
//    - SD               (built-in ESP32 core)
// ════════════════════════════════════════════════════════════════

#include <SPI.h>
#include <LoRa.h>
#include <Wire.h>
#include <ArduinoJson.h>
#include <RTClib.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <SD.h>
#include <FS.h>

// ── LoRa (HSPI) ──────────────────────────────────────────────────
#define LORA_SCK    14
#define LORA_MISO   12
#define LORA_MOSI   13
#define LORA_CS      5
#define LORA_RST    -1
#define LORA_IRQ    34
#define LORA_FREQ   433E6
#define LORA_SF     7
#define LORA_BW     125E3
#define LORA_SYNC   0xB4

// ── TFT ST7789 240×320 (VSPI: SCK=18 MOSI=23 MISO=19) ───────────
#define TFT_CS   27
#define TFT_RST   4
#define TFT_DC    2
#define TFT_W   240
#define TFT_H   320

// ── SD card (VSPI shared with TFT) ───────────────────────────────
#define SD_CS     33
#define LOG_DIR   "/BLUEARC_DATA"

// ── RTC DS3231 ───────────────────────────────────────────────────
#define RTC_SDA  21
#define RTC_SCL  22

// ── LEDs ─────────────────────────────────────────────────────────
#define LED_BLUE   25   // LoRa RX blink
#define LED_GRN_M  32   // Master OK (green)
#define LED_ORG_M  26   // Master fault/offline (orange)
#define LED_GRN_S  16   // Slaves OK (green)
#define LED_ORG_S  17   // Slave fault/offline (orange)

// ── TFT colours (RGB565) ─────────────────────────────────────────
#define C_BG        0x0842   // very dark blue-grey
#define C_HEADER    0x0010   // near-black header
#define C_ACCENT    0x05FF   // cyan-ish (SCADA accent)
#define C_GREEN     0x07E0
#define C_ORANGE    0xFC60
#define C_RED       0xF800
#define C_WHITE     0xFFFF
#define C_LTGREY    0xC618
#define C_DKGREY    0x4208
#define C_YELLOW    0xFFE0
#define C_BLUE      0x001F
#define C_CYAN      0x07FF
#define C_PURPLE    0xA01F
#define C_MUTED     0x8410   // muted grey for labels

// ── Energy baseline ───────────────────────────────────────────────
#define BASELINE_W_RX  1.41f  // baseline standby load (W) for saving graph

// ─────────────────────────────────────────────────────────────────
//  DATA STRUCTURES
// ─────────────────────────────────────────────────────────────────
struct MasterPkt {
  bool    valid       = false;
  unsigned long lastRxMs = 0;
  uint32_t seq        = 0;
  float   vrms        = 0;
  float   irms        = 0;
  float   pActive     = 0;
  float   pReactive   = 0;
  float   pApparent   = 0;
  float   pf          = 0;
  float   freqHz      = 50;
  bool    relayOK     = true;
  uint16_t faultFlags = 0;
  int     rssi        = 0;
};

struct SlaveStatus {
  bool    valid       = false;
  unsigned long lastRxMs = 0;
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
  uint32_t ageS       = 255;
};

MasterPkt  g_master;
SlaveStatus g_a1;
SlaveStatus g_a2;

// ── Graph history (ring buffers, 60 samples each) ─────────────────
#define GRAPH_SAMPLES 60
float g_vHistory[GRAPH_SAMPLES];
float g_iHistory[GRAPH_SAMPLES];
float g_savWHistory[GRAPH_SAMPLES];
float g_costHistory[GRAPH_SAMPLES];
uint8_t g_histIdx = 0;
unsigned long g_lastHistMs = 0;

// ── Display state ─────────────────────────────────────────────────
uint8_t  g_page = 0;     // 0=Master, 1=A1, 2=A2, 3=V/I graph, 4=Saving graph, 5=Map
#define  NUM_PAGES 6
unsigned long g_lastPageSwitch = 0;
#define  PAGE_MS  4000

// ─────────────────────────────────────────────────────────────────
RTC_DS3231 rtc;
Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);

bool sdOk  = false;
bool rtcOk = false;
String g_logFile = "";

// ─────────────────────────────────────────────────────────────────
//  HELPERS
// ─────────────────────────────────────────────────────────────────
String getTimestamp() {
  if (!rtcOk) return "NO_RTC_" + String(millis()/1000);
  DateTime now = rtc.now();
  char buf[24];
  snprintf(buf, sizeof(buf), "%04d-%02d-%02d %02d:%02d:%02d",
    now.year(), now.month(), now.day(),
    now.hour(), now.minute(), now.second());
  return String(buf);
}

// ─────────────────────────────────────────────────────────────────
//  SD LOGGING — logs to /BLUEARC_DATA/BARC_YYYYMMDD.csv
// ─────────────────────────────────────────────────────────────────
void initSDLogging() {
  if (!sdOk) return;
  if (!SD.exists(LOG_DIR)) {
    if (SD.mkdir(LOG_DIR)) Serial.println("[SD] Created /BLUEARC_DATA");
    else { Serial.println("[SD] mkdir failed"); return; }
  }
  // Session file named by date
  String fname = String(LOG_DIR) + "/BARC_";
  if (rtcOk) {
    DateTime now = rtc.now();
    char ds[9]; snprintf(ds,9,"%04d%02d%02d",now.year(),now.month(),now.day());
    fname += ds;
  } else {
    fname += "BOOT_" + String(millis()/1000);
  }
  fname += ".csv";
  g_logFile = fname;

  if (!SD.exists(fname)) {
    File f = SD.open(fname, FILE_WRITE);
    if (f) {
      f.println("timestamp,vrms,irms,pActive,pReactive,pApparent,pf,freqHz,"
                "masterRelay,masterFaults,masterRSSI,"
                "a1Valid,a1PIR,a1Night,a1Tilt,a1RelayOK,a1inaI,a1inaP,a1savW,a1cost,a1RSSI,"
                "a2Valid,a2PIR,a2Night,a2Tilt,a2RelayOK,a2RSSI");
      f.close();
      Serial.println("[SD] Log file: " + fname);
    }
  } else {
    Serial.println("[SD] Appending to: " + fname);
  }
}

void logData() {
  if (!sdOk || g_logFile == "") return;
  File f = SD.open(g_logFile, FILE_APPEND);
  if (!f) return;
  f.printf("%s,%.1f,%.3f,%.1f,%.1f,%.1f,%.3f,%.2f,%d,%u,%d,"
           "%d,%d,%d,%d,%d,%.3f,%.2f,%.2f,%.2f,%d,"
           "%d,%d,%d,%d,%d,%d\n",
    getTimestamp().c_str(),
    g_master.vrms, g_master.irms,
    g_master.pActive, g_master.pReactive, g_master.pApparent,
    g_master.pf, g_master.freqHz,
    g_master.relayOK ? 1:0, g_master.faultFlags, g_master.rssi,
    g_a1.valid?1:0, g_a1.pir?1:0, g_a1.isNight?1:0, g_a1.tilt?1:0,
    g_a1.relayOK?1:0, g_a1.inaCurrentA, g_a1.inaPowerW, g_a1.savingW, g_a1.costSaved, g_a1.rssi,
    g_a2.valid?1:0, g_a2.pir?1:0, g_a2.isNight?1:0, g_a2.tilt?1:0,
    g_a2.relayOK?1:0, g_a2.rssi
  );
  f.close();
}

// ─────────────────────────────────────────────────────────────────
//  PARSE MASTER PACKET
// ─────────────────────────────────────────────────────────────────
void parseMasterPacket(const String& raw, int rssi) {
  StaticJsonDocument<640> doc;
  if (deserializeJson(doc, raw)) return;

  const char* src = doc["src"] | "";
  if (strcmp(src, "M") != 0) return;

  g_master.valid     = true;
  g_master.lastRxMs  = millis();
  g_master.seq       = doc["sq"]  | 0;
  g_master.vrms      = doc["V"]   | 0.0f;
  g_master.irms      = doc["I"]   | 0.0f;
  g_master.pActive   = doc["P"]   | 0.0f;
  g_master.pReactive = doc["Q"]   | 0.0f;
  g_master.pApparent = doc["S"]   | 0.0f;
  g_master.pf        = doc["PF"]  | 0.0f;
  g_master.freqHz    = doc["Hz"]  | 50.0f;
  g_master.relayOK   = (doc["R"]  | 1) == 1;
  g_master.faultFlags= doc["F"]   | 0;
  g_master.rssi      = rssi;

  // A1
  if (doc.containsKey("A1")) {
    JsonObject a1d = doc["A1"];
    g_a1.valid        = (a1d["age"]|255) < 250;
    g_a1.lastRxMs     = millis();
    g_a1.ageS         = a1d["age"] | 255;
    g_a1.pir          = a1d["pir"]  | 0;
    g_a1.isNight      = (a1d["ldr"] | 1) == 0;
    g_a1.tilt         = a1d["tilt"] | 0;
    g_a1.relayOK      = a1d["rel"]  | 1;
    g_a1.bypassActive = a1d["byp"]  | 0;
    g_a1.bypassIntent = a1d["bypI"] | 0;
    g_a1.faultFlags   = a1d["F"]    | 0;
    g_a1.rssi         = a1d["rssi"] | 0;
    g_a1.inaCurrentA  = a1d["inaI"] | 0.0f;
    g_a1.inaPowerW    = a1d["inaP"] | 0.0f;
    g_a1.savingW      = a1d["savW"] | 0.0f;
    g_a1.costSaved    = a1d["cost"] | 0.0f;
  }

  // A2
  if (doc.containsKey("A2")) {
    JsonObject a2d = doc["A2"];
    g_a2.valid        = (a2d["age"]|255) < 250;
    g_a2.lastRxMs     = millis();
    g_a2.ageS         = a2d["age"] | 255;
    g_a2.pir          = a2d["pir"]  | 0;
    g_a2.isNight      = (a2d["ldr"] | 1) == 0;
    g_a2.tilt         = a2d["tilt"] | 0;
    g_a2.relayOK      = a2d["rel"]  | 1;
    g_a2.bypassActive = a2d["byp"]  | 0;
    g_a2.bypassIntent = a2d["bypI"] | 0;
    g_a2.faultFlags   = a2d["F"]    | 0;
    g_a2.rssi         = a2d["rssi"] | 0;
  }

  // Add to graph history every 2s
  if (millis() - g_lastHistMs > 2000) {
    g_lastHistMs = millis();
    g_vHistory[g_histIdx]   = g_master.vrms;
    g_iHistory[g_histIdx]   = g_master.irms;
    g_savWHistory[g_histIdx]= g_a1.savingW;
    g_costHistory[g_histIdx]= g_a1.costSaved;
    g_histIdx = (g_histIdx + 1) % GRAPH_SAMPLES;
  }

  Serial.printf("[RX Master] seq=%u  V=%.1f  I=%.3f  PF=%.3f  RSSI=%d\n",
                g_master.seq, g_master.vrms, g_master.irms, g_master.pf, rssi);
}

// ─────────────────────────────────────────────────────────────────
//  UPDATE LEDs
// ─────────────────────────────────────────────────────────────────
void updateLEDs() {
  // Master status LEDs
  bool masterOffline = !g_master.valid || (millis()-g_master.lastRxMs > 3000);
  bool masterFault   = g_master.faultFlags != 0 || !g_master.relayOK;

  if (masterOffline || masterFault) {
    digitalWrite(LED_GRN_M, LOW);
    // Blink orange on fault, solid on offline
    digitalWrite(LED_ORG_M, masterOffline ? HIGH : (millis()/200)%2);
  } else {
    digitalWrite(LED_GRN_M, HIGH);
    digitalWrite(LED_ORG_M, LOW);
  }

  // Slave status LEDs
  bool slaveFault = (g_a1.faultFlags && g_a1.valid) || (g_a2.faultFlags && g_a2.valid);
  bool slaveOffline= !g_a1.valid || !g_a2.valid;

  if (slaveFault || slaveOffline) {
    digitalWrite(LED_GRN_S, LOW);
    digitalWrite(LED_ORG_S, slaveFault ? (millis()/200)%2 : HIGH);
  } else {
    digitalWrite(LED_GRN_S, HIGH);
    digitalWrite(LED_ORG_S, LOW);
  }
}

// ═════════════════════════════════════════════════════════════════
//  TFT DISPLAY — SCADA SCREENS
// ═════════════════════════════════════════════════════════════════

// ── Shared header ─────────────────────────────────────────────────
void drawHeader(const char* pageTitle) {
  tft.fillRect(0, 0, TFT_W, 28, C_HEADER);
  // Branding
  tft.setTextColor(C_ACCENT); tft.setTextSize(1);
  tft.setCursor(4, 4); tft.print("BLUEARC");
  tft.setTextColor(C_WHITE); tft.setTextSize(1);
  tft.setCursor(52, 4); tft.print("SYSTEMS");
  // Page title right-justified
  tft.setTextColor(C_YELLOW); tft.setTextSize(1);
  int tw = strlen(pageTitle) * 6;
  tft.setCursor(TFT_W - tw - 4, 4); tft.print(pageTitle);
  // Timestamp row
  tft.setTextColor(C_DKGREY); tft.setTextSize(1);
  String ts = getTimestamp();
  tft.setCursor(4, 16); tft.print(ts);
  // Divider
  tft.drawFastHLine(0, 27, TFT_W, C_ACCENT);
}

// ── Status dot ────────────────────────────────────────────────────
void drawDot(int x, int y, uint16_t colour) {
  tft.fillCircle(x, y, 4, colour);
  tft.drawCircle(x, y, 4, C_WHITE);
}

// ── Value row ─────────────────────────────────────────────────────
void drawRow(int y, const char* label, const char* value, uint16_t valColour = C_WHITE) {
  tft.setTextSize(1);
  tft.setTextColor(C_LTGREY);
  tft.setCursor(6, y); tft.print(label);
  tft.setTextColor(valColour);
  tft.setCursor(120, y); tft.print(value);
}

// ── Big value + unit ──────────────────────────────────────────────
void drawBigVal(int x, int y, const char* val, const char* unit, uint16_t col = C_CYAN) {
  tft.setTextColor(col); tft.setTextSize(2);
  tft.setCursor(x, y); tft.print(val);
  tft.setTextColor(C_LTGREY); tft.setTextSize(1);
  tft.setCursor(x, y+17); tft.print(unit);
}

// ── PAGE 0: Branding splash + system status ───────────────────────
void drawPageBrand() {
  tft.fillScreen(C_BG);
  // Big logo area
  tft.fillRect(0, 0, TFT_W, 70, 0x0008);
  tft.drawRect(0, 0, TFT_W, 70, C_ACCENT);

  tft.setTextColor(C_ACCENT); tft.setTextSize(3);
  tft.setCursor(14, 8); tft.print("BLUEARC");
  tft.setTextColor(C_WHITE); tft.setTextSize(2);
  tft.setCursor(28, 38); tft.print("SYSTEMS");
  tft.setTextColor(C_DKGREY); tft.setTextSize(1);
  tft.setCursor(6, 60); tft.print("Smart Grid & DT Monitoring");

  // Status section
  tft.drawFastHLine(0, 72, TFT_W, C_ACCENT);
  tft.setTextColor(C_LTGREY); tft.setTextSize(1);
  tft.setCursor(6, 78); tft.print("SYSTEM STATUS");

  int y = 92;
  // Master
  bool mOK = g_master.valid && (millis()-g_master.lastRxMs < 3000);
  drawDot(10, y+4, mOK ? C_GREEN : C_RED);
  tft.setTextColor(C_WHITE); tft.setTextSize(1);
  tft.setCursor(20, y);
  tft.printf("MASTER  V=%.1fV  I=%.3fA  PF=%.2f",
             g_master.vrms, g_master.irms, g_master.pf);
  y += 18;

  // A1
  bool a1OK = g_a1.valid;
  drawDot(10, y+4, a1OK ? C_GREEN : C_ORANGE);
  tft.setCursor(20, y);
  if (a1OK) tft.printf("A1  INA: %.2fW  Sav: %.2fW  RSSI:%d",
                        g_a1.inaPowerW, g_a1.savingW, g_a1.rssi);
  else tft.print("A1  --  Not receiving  --");
  y += 18;

  // A2
  bool a2OK = g_a2.valid;
  drawDot(10, y+4, a2OK ? C_GREEN : C_ORANGE);
  tft.setCursor(20, y);
  if (a2OK) tft.printf("A2  PIR:%s  Tilt:%s  RSSI:%d",
                        g_a2.pir?"ON":"--", g_a2.tilt?"!":"OK", g_a2.rssi);
  else tft.print("A2  --  Not receiving  --");
  y += 18;

  // Fault summary
  tft.drawFastHLine(0, y+4, TFT_W, C_DKGREY);
  y += 10;
  if (g_master.faultFlags) {
    tft.setTextColor(C_RED); tft.setTextSize(1);
    tft.setCursor(6, y);
    tft.printf("FAULT FLAGS: 0x%04X", g_master.faultFlags);
  } else {
    tft.setTextColor(C_GREEN); tft.setTextSize(1);
    tft.setCursor(6, y); tft.print("All systems nominal  OK");
  }
  y += 14;
  // Timestamp
  tft.setTextColor(C_DKGREY); tft.setTextSize(1);
  tft.setCursor(6, y); tft.print(getTimestamp());
}

// ── PAGE 1: Master power data ─────────────────────────────────────
void drawPageMaster() {
  tft.fillScreen(C_BG);
  drawHeader("MASTER / DT");

  // 2×4 big value grid
  int gx[2] = {6, 122};
  int gy[4] = {34, 90, 146, 202};
  struct { float val; const char* lbl; const char* unit; } items[] = {
    { g_master.vrms,      "VOLTAGE",   "V AC" },
    { g_master.irms,      "CURRENT",   "A"    },
    { g_master.pActive,   "ACTIVE P",  "W"    },
    { g_master.pReactive, "REACTIVE",  "VAR"  },
    { g_master.pApparent, "APPARENT",  "VA"   },
    { g_master.pf,        "PWR FCTR",  ""     },
    { g_master.freqHz,    "FREQUENCY", "Hz"   },
  };
  for (int i = 0; i < 7; i++) {
    int col = i % 2, row = i / 2;
    int x = gx[col], y = gy[row];
    tft.fillRect(x, y, 112, 50, C_HEADER);
    tft.drawRect(x, y, 112, 50, C_DKGREY);
    tft.setTextColor(C_MUTED); tft.setTextSize(1);
    tft.setTextColor(C_LTGREY);
    tft.setCursor(x+4, y+4); tft.print(items[i].lbl);

    char buf[12];
    if (i == 5) snprintf(buf, 12, "%.3f", items[i].val);   // PF 3dp
    else if (i == 6) snprintf(buf, 12, "%.2f", items[i].val);
    else snprintf(buf, 12, "%.1f", items[i].val);

    tft.setTextColor(C_CYAN); tft.setTextSize(2);
    tft.setCursor(x+4, y+18); tft.print(buf);
    tft.setTextColor(C_DKGREY); tft.setTextSize(1);
    tft.setCursor(x+4, y+38); tft.print(items[i].unit);
  }
  // Relay status tile
  int x = gx[1], y = gy[3];
  tft.fillRect(x, y, 112, 50, g_master.relayOK ? 0x0240 : 0x3800);
  tft.drawRect(x, y, 112, 50, g_master.relayOK ? C_GREEN : C_RED);
  tft.setTextColor(C_WHITE); tft.setTextSize(1);
  tft.setCursor(x+4, y+4); tft.print("RELAY");
  tft.setTextSize(2);
  tft.setCursor(x+4, y+18);
  tft.print(g_master.relayOK ? "OK" : "TRIP!");

  // Bottom bar
  tft.drawFastHLine(0, 262, TFT_W, C_DKGREY);
  tft.setTextColor(C_DKGREY); tft.setTextSize(1);
  tft.setCursor(4, 267);
  tft.printf("RSSI:%ddBm  seq:%u  age:%lus",
             g_master.rssi, g_master.seq,
             g_master.valid ? (millis()-g_master.lastRxMs)/1000 : 0);
}

// ── PAGE 2: Slave A1 ─────────────────────────────────────────────
void drawPageA1() {
  tft.fillScreen(C_BG);
  drawHeader("NODE A1");

  int y = 34;
  tft.fillRect(0, y, TFT_W, 14, 0x0210);
  tft.setTextColor(C_ACCENT); tft.setTextSize(1);
  tft.setCursor(4, y+3); tft.print("A1 — INA260 Energy Monitor + Streetlight");
  y += 18;

  if (!g_a1.valid) {
    tft.setTextColor(C_ORANGE); tft.setTextSize(2);
    tft.setCursor(20, 80); tft.print("WAITING...");
    tft.setTextColor(C_LTGREY); tft.setTextSize(1);
    tft.setCursor(20, 104); tft.print("No packets from A1 yet");
    return;
  }

  // Status row
  drawDot(10, y+4, !g_a1.faultFlags ? C_GREEN : C_ORANGE);
  tft.setTextColor(C_WHITE); tft.setTextSize(1);
  tft.setCursor(22, y);
  tft.printf("RSSI:%ddBm  Fault:0x%02X  Age:%us",
             g_a1.rssi, g_a1.faultFlags, g_a1.ageS);
  y += 16;

  // Sensor indicators
  tft.drawRect(4, y, 70, 26, C_DKGREY);
  tft.setTextColor(g_a1.pir ? C_GREEN : C_DKGREY); tft.setTextSize(1);
  tft.setCursor(8, y+4); tft.print(g_a1.pir ? "PIR:MOTION" : "PIR: clear");
  tft.setCursor(8, y+14); tft.setTextColor(g_a1.isNight ? C_BLUE : C_YELLOW);
  tft.print(g_a1.isNight ? "LDR: NIGHT" : "LDR:  DAY");

  tft.drawRect(80, y, 70, 26, C_DKGREY);
  tft.setTextColor(g_a1.tilt ? C_RED : C_GREEN); tft.setTextSize(1);
  tft.setCursor(84, y+4); tft.print(g_a1.tilt ? "TILT: ALARM" : "TILT:  OK");
  tft.setCursor(84, y+14); tft.setTextColor(g_a1.relayOK ? C_GREEN : C_RED);
  tft.print(g_a1.relayOK ? "RELAY:  OK" : "RELAY:TRIP");

  tft.drawRect(156, y, 80, 26, C_DKGREY);
  tft.setTextColor(C_LTGREY); tft.setTextSize(1);
  tft.setCursor(160, y+4); tft.print("BYPASS");
  tft.setTextColor(g_a1.bypassActive ? C_ORANGE : C_GREEN);
  tft.setCursor(160, y+14); tft.print(g_a1.bypassActive ? "ACTIVE" : "normal");
  y += 32;

  // INA260 big values
  tft.drawFastHLine(0, y, TFT_W, C_DKGREY); y += 6;
  tft.setTextColor(C_LTGREY); tft.setTextSize(1);
  tft.setCursor(4, y); tft.print("INA260 — Streetlight Power");
  y += 12;

  struct { float v; const char* l; const char* u; } pw[] = {
    { g_a1.inaCurrentA*1000, "Current", "mA" },
    { g_a1.inaPowerW,        "Power",   "W"  },
    { g_a1.savingW,          "Saving",  "W"  },
  };
  int bx = 4;
  for (int i = 0; i < 3; i++) {
    tft.fillRect(bx, y, 74, 44, C_HEADER);
    tft.drawRect(bx, y, 74, 44, C_DKGREY);
    tft.setTextColor(C_LTGREY); tft.setTextSize(1);
    tft.setCursor(bx+4, y+4); tft.print(pw[i].l);
    char buf[10]; snprintf(buf, 10, "%.2f", pw[i].v);
    tft.setTextColor(C_CYAN); tft.setTextSize(2);
    tft.setCursor(bx+4, y+18); tft.print(buf);
    tft.setTextColor(C_DKGREY); tft.setTextSize(1);
    tft.setCursor(bx+4, y+36); tft.print(pw[i].u);
    bx += 78;
  }
  // Cost saved
  tft.fillRect(4, y+48, 232, 28, 0x0420);
  tft.drawRect(4, y+48, 232, 28, C_GREEN);
  tft.setTextColor(C_GREEN); tft.setTextSize(1);
  tft.setCursor(8, y+52); tft.print("Total Cost Saved:");
  tft.setTextColor(C_WHITE); tft.setTextSize(2);
  char costbuf[12]; snprintf(costbuf, 12, "%c%.4f", 0xA4, g_a1.costSaved); // Rs symbol
  tft.setCursor(130, y+50); tft.print("Rs ");
  tft.print(g_a1.costSaved, 4);
}

// ── PAGE 3: Slave A2 ─────────────────────────────────────────────
void drawPageA2() {
  tft.fillScreen(C_BG);
  drawHeader("NODE A2");

  int y = 34;
  tft.fillRect(0, y, TFT_W, 14, 0x0010);
  tft.setTextColor(C_BLUE); tft.setTextSize(1);
  tft.setCursor(4, y+3); tft.print("A2 — Endpoint Node (Load End)");
  y += 18;

  if (!g_a2.valid) {
    tft.setTextColor(C_ORANGE); tft.setTextSize(2);
    tft.setCursor(20, 80); tft.print("WAITING...");
    return;
  }

  drawDot(10, y+4, !g_a2.faultFlags ? C_GREEN : C_ORANGE);
  tft.setTextColor(C_WHITE); tft.setTextSize(1);
  tft.setCursor(22, y);
  tft.printf("RSSI:%ddBm  Fault:0x%02X  Age:%us",
             g_a2.rssi, g_a2.faultFlags, g_a2.ageS);
  y += 20;

  // Sensor grid
  struct { const char* lbl; bool state; const char* onTxt; const char* offTxt;
           uint16_t onCol; uint16_t offCol; } rows[] = {
    { "PIR Motion",  g_a2.pir,         "MOTION DETECTED", "No motion",    C_GREEN,  C_DKGREY },
    { "LDR",         g_a2.isNight,     "NIGHT",           "DAY",          C_BLUE,   C_YELLOW },
    { "Tilt",        g_a2.tilt,        "!! ALARM !!",     "OK",           C_RED,    C_GREEN  },
    { "Relay",       !g_a2.relayOK,    "TRIPPED!",        "OK (live)",    C_RED,    C_GREEN  },
    { "Bypass",      g_a2.bypassActive,"ACTIVE",          "Normal",       C_ORANGE, C_DKGREY },
    { "Consumer Load",!g_a2.pir&&!g_a2.isNight, "Likely ON","Likely OFF or Night",C_CYAN, C_LTGREY},
  };
  for (int i = 0; i < 6; i++) {
    tft.fillRect(4, y, 232, 20, C_HEADER);
    tft.drawRect(4, y, 232, 20, C_DKGREY);
    tft.setTextColor(C_LTGREY); tft.setTextSize(1);
    tft.setCursor(8, y+6); tft.print(rows[i].lbl);
    uint16_t col = rows[i].state ? rows[i].onCol : rows[i].offCol;
    tft.setTextColor(col);
    tft.setCursor(120, y+6); tft.print(rows[i].state ? rows[i].onTxt : rows[i].offTxt);
    y += 24;
  }

  // Wire break note
  y += 4;
  tft.drawFastHLine(0, y, TFT_W, C_DKGREY); y += 8;
  tft.setTextColor(C_DKGREY); tft.setTextSize(1);
  tft.setCursor(4, y); tft.print("Wire break: A2 goes offline (HB timeout)");
  y += 12;
  tft.setCursor(4, y); tft.print("Load off: A2 alive, consumer switch = open");
}

// ── PAGE 4: V/I Trend graph ───────────────────────────────────────
void drawPageVIGraph() {
  tft.fillScreen(C_BG);
  drawHeader("V / I TREND");

  // Graph area: x=20..230, y=40..150 (voltage), y=160..270 (current)
  // Find min/max for scaling
  float vMin=999, vMax=-999, iMin=999, iMax=-999;
  for (int i = 0; i < GRAPH_SAMPLES; i++) {
    if (g_vHistory[i] > 0) {
      if (g_vHistory[i] < vMin) vMin = g_vHistory[i];
      if (g_vHistory[i] > vMax) vMax = g_vHistory[i];
    }
    if (g_iHistory[i] >= 0) {
      if (g_iHistory[i] < iMin) iMin = g_iHistory[i];
      if (g_iHistory[i] > iMax) iMax = g_iHistory[i];
    }
  }
  if (vMax - vMin < 10) { vMin -= 5; vMax += 5; }
  if (iMax - iMin < 0.1f) { iMin -= 0.05f; iMax += 0.05f; }

  // Voltage graph (top half)
  int gx1=22, gx2=234, gy1v=36, gy2v=148;
  int gx1i=22, gx2i=234, gy1i=158, gy2i=268;

  tft.drawRect(gx1, gy1v, gx2-gx1, gy2v-gy1v, C_DKGREY);
  tft.setTextColor(C_CYAN); tft.setTextSize(1);
  tft.setCursor(gx1, gy1v-10); tft.print("Voltage (V)");
  tft.setTextColor(C_DKGREY); tft.setTextSize(1);
  // Y-axis labels
  char lbuf[8];
  snprintf(lbuf, 8, "%.0f", vMax); tft.setCursor(0, gy1v); tft.print(lbuf);
  snprintf(lbuf, 8, "%.0f", vMin); tft.setCursor(0, gy2v-8); tft.print(lbuf);

  // Draw voltage line
  for (int i = 1; i < GRAPH_SAMPLES; i++) {
    int idxCur  = (g_histIdx + i)     % GRAPH_SAMPLES;
    int idxPrev = (g_histIdx + i - 1) % GRAPH_SAMPLES;
    if (g_vHistory[idxCur] == 0 || g_vHistory[idxPrev] == 0) continue;
    int x0 = gx1 + (i-1) * (gx2-gx1) / GRAPH_SAMPLES;
    int x1 = gx1 + i     * (gx2-gx1) / GRAPH_SAMPLES;
    int y0 = gy2v - (int)((g_vHistory[idxPrev]-vMin)/(vMax-vMin)*(gy2v-gy1v));
    int y1 = gy2v - (int)((g_vHistory[idxCur] -vMin)/(vMax-vMin)*(gy2v-gy1v));
    y0 = constrain(y0, gy1v, gy2v);
    y1 = constrain(y1, gy1v, gy2v);
    tft.drawLine(x0, y0, x1, y1, C_CYAN);
  }
  // Nominal line
  int yNom = gy2v - (int)((230.0f-vMin)/(vMax-vMin)*(gy2v-gy1v));
  if (yNom > gy1v && yNom < gy2v)
    tft.drawFastHLine(gx1, yNom, gx2-gx1, 0x0420);  // faint green

  // Current graph (bottom half)
  tft.drawRect(gx1i, gy1i, gx2i-gx1i, gy2i-gy1i, C_DKGREY);
  tft.setTextColor(C_ORANGE); tft.setTextSize(1);
  tft.setCursor(gx1i, gy1i-10); tft.print("Current (A)");
  snprintf(lbuf, 8, "%.2f", iMax); tft.setCursor(0, gy1i); tft.setTextColor(C_DKGREY); tft.print(lbuf);
  snprintf(lbuf, 8, "%.2f", iMin); tft.setCursor(0, gy2i-8); tft.print(lbuf);

  for (int i = 1; i < GRAPH_SAMPLES; i++) {
    int idxCur  = (g_histIdx + i)     % GRAPH_SAMPLES;
    int idxPrev = (g_histIdx + i - 1) % GRAPH_SAMPLES;
    if (g_iHistory[idxCur] == 0 || g_iHistory[idxPrev] == 0) continue;
    int x0 = gx1i + (i-1) * (gx2i-gx1i) / GRAPH_SAMPLES;
    int x1 = gx1i + i     * (gx2i-gx1i) / GRAPH_SAMPLES;
    int y0 = gy2i - (int)((g_iHistory[idxPrev]-iMin)/(iMax-iMin)*(gy2i-gy1i));
    int y1 = gy2i - (int)((g_iHistory[idxCur] -iMin)/(iMax-iMin)*(gy2i-gy1i));
    y0 = constrain(y0, gy1i, gy2i);
    y1 = constrain(y1, gy1i, gy2i);
    tft.drawLine(x0, y0, x1, y1, C_ORANGE);
  }

  // X-axis label
  tft.setTextColor(C_DKGREY); tft.setTextSize(1);
  tft.setCursor(4, 276); tft.print("← 120s ago");
  tft.setCursor(170, 276); tft.print("now →");
}

// ── PAGE 5: Saving graph ──────────────────────────────────────────
void drawPageSavingGraph() {
  tft.fillScreen(C_BG);
  drawHeader("ENERGY SAVING");

  float sMin=9999, sMax=-9999, cMin=9999, cMax=-9999;
  for (int i = 0; i < GRAPH_SAMPLES; i++) {
    if (g_savWHistory[i] >= 0) {
      if (g_savWHistory[i] < sMin) sMin = g_savWHistory[i];
      if (g_savWHistory[i] > sMax) sMax = g_savWHistory[i];
    }
    if (g_costHistory[i] >= 0) {
      if (g_costHistory[i] < cMin) cMin = g_costHistory[i];
      if (g_costHistory[i] > cMax) cMax = g_costHistory[i];
    }
  }
  if (sMax - sMin < 0.1f) { sMin = 0; sMax = BASELINE_W_RX + 0.1f; }
  if (cMax - cMin < 0.001f) { cMin = 0; cMax += 0.01f; }

  int gx1=22, gx2=234, gy1s=36, gy2s=148;
  int gy1c=158, gy2c=268;

  tft.drawRect(gx1, gy1s, gx2-gx1, gy2s-gy1s, C_DKGREY);
  tft.setTextColor(C_GREEN); tft.setTextSize(1);
  tft.setCursor(gx1, gy1s-10); tft.print("A1 Saving (W)");

  for (int i = 1; i < GRAPH_SAMPLES; i++) {
    int idxCur  = (g_histIdx + i)     % GRAPH_SAMPLES;
    int idxPrev = (g_histIdx + i - 1) % GRAPH_SAMPLES;
    if (g_savWHistory[idxCur] == 0 && g_savWHistory[idxPrev] == 0) continue;
    int x0 = gx1 + (i-1) * (gx2-gx1) / GRAPH_SAMPLES;
    int x1 = gx1 + i     * (gx2-gx1) / GRAPH_SAMPLES;
    int y0 = gy2s - (int)((g_savWHistory[idxPrev]-sMin)/(sMax-sMin)*(gy2s-gy1s));
    int y1 = gy2s - (int)((g_savWHistory[idxCur] -sMin)/(sMax-sMin)*(gy2s-gy1s));
    tft.drawLine(x0, constrain(y0,gy1s,gy2s), x1, constrain(y1,gy1s,gy2s), C_GREEN);
  }

  tft.drawRect(gx1, gy1c, gx2-gx1, gy2c-gy1c, C_DKGREY);
  tft.setTextColor(C_YELLOW); tft.setTextSize(1);
  tft.setCursor(gx1, gy1c-10); tft.print("Cost Saved (Rs)");

  for (int i = 1; i < GRAPH_SAMPLES; i++) {
    int idxCur  = (g_histIdx + i)     % GRAPH_SAMPLES;
    int idxPrev = (g_histIdx + i - 1) % GRAPH_SAMPLES;
    int x0 = gx1 + (i-1) * (gx2-gx1) / GRAPH_SAMPLES;
    int x1 = gx1 + i     * (gx2-gx1) / GRAPH_SAMPLES;
    float denom = cMax - cMin; if (denom < 0.001f) denom = 0.001f;
    int y0 = gy2c - (int)((g_costHistory[idxPrev]-cMin)/denom*(gy2c-gy1c));
    int y1 = gy2c - (int)((g_costHistory[idxCur] -cMin)/denom*(gy2c-gy1c));
    tft.drawLine(x0, constrain(y0,gy1c,gy2c), x1, constrain(y1,gy1c,gy2c), C_YELLOW);
  }

  // Current totals
  tft.setTextColor(C_DKGREY); tft.setTextSize(1);
  tft.setCursor(4, 276);
  tft.printf("Now: %.2fW saved  Rs %.4f total", g_a1.savingW, g_a1.costSaved);
}

// ── PAGE 6: Node location map (text-based grid) ───────────────────
// (No internet on TFT — draw a schematic topology map instead)
void drawPageMap() {
  tft.fillScreen(C_BG);
  drawHeader("NODE MAP");

  int y = 36;
  tft.setTextColor(C_LTGREY); tft.setTextSize(1);
  tft.setCursor(4, y); tft.print("Network Topology & Locations");
  y += 14;

  // Draw chain: DT(M) --LoRa--> A1 --LoRa--> A2 --[load]
  // DT box
  int bw=60, bh=36;
  int xM=4, xA1=90, xA2=174;
  int by = y+10;

  // Master box
  tft.fillRect(xM, by, bw, bh, !g_master.faultFlags ? 0x0420 : 0x3800);
  tft.drawRect(xM, by, bw, bh, g_master.relayOK ? C_GREEN : C_RED);
  tft.setTextColor(C_WHITE); tft.setTextSize(1);
  tft.setCursor(xM+4, by+4); tft.print("MASTER");
  tft.setCursor(xM+4, by+14); tft.setTextColor(C_CYAN);
  tft.printf("%.0fV %.1fHz", g_master.vrms, g_master.freqHz);
  tft.setCursor(xM+4, by+24); tft.setTextColor(C_LTGREY);
  tft.print(g_master.relayOK ? "Relay:OK" : "TRIPPED");

  // Arrow M→A1
  tft.setTextColor(C_ACCENT);
  tft.setCursor(xM+bw+2, by+14); tft.print("-LoRa->");

  // A1 box
  tft.fillRect(xA1, by, bw, bh, g_a1.valid ? 0x0208 : 0x3800);
  tft.drawRect(xA1, by, bw, bh, g_a1.valid ? C_CYAN : C_RED);
  tft.setTextColor(C_WHITE); tft.setTextSize(1);
  tft.setCursor(xA1+4, by+4); tft.print("A1+INA");
  tft.setCursor(xA1+4, by+14); tft.setTextColor(g_a1.valid ? C_GREEN : C_RED);
  tft.print(g_a1.valid ? "Online" : "OFFLINE");
  tft.setCursor(xA1+4, by+24); tft.setTextColor(C_LTGREY);
  tft.printf("%.2fW", g_a1.inaPowerW);

  // Arrow A1→A2
  tft.setTextColor(C_ACCENT);
  tft.setCursor(xA1+bw+2, by+14); tft.print("-LoRa->");

  // A2 box
  tft.fillRect(xA2, by, bw, bh, g_a2.valid ? 0x0008 : 0x3800);
  tft.drawRect(xA2, by, bw, bh, g_a2.valid ? C_BLUE : C_RED);
  tft.setTextColor(C_WHITE); tft.setTextSize(1);
  tft.setCursor(xA2+4, by+4); tft.print("A2");
  tft.setCursor(xA2+4, by+14); tft.setTextColor(g_a2.valid ? C_GREEN : C_RED);
  tft.print(g_a2.valid ? "Online" : "OFFLINE");
  tft.setCursor(xA2+4, by+24); tft.setTextColor(C_LTGREY);
  tft.print(g_a2.pir ? "PIR:ON" : "PIR:--");

  // Consumer load indicator
  tft.setCursor(xA2+bw+2, by+14); tft.setTextColor(C_DKGREY);
  tft.print("~Load");
  y = by + bh + 12;

  // GPS Coordinates
  tft.drawFastHLine(0, y, TFT_W, C_DKGREY); y += 6;
  tft.setTextColor(C_LTGREY); tft.setTextSize(1);
  tft.setCursor(4, y); tft.print("Node Coordinates (GPS)");
  y += 12;

  // Master (hardcoded — no GPS on master, user sets in config)
  tft.setTextColor(C_ACCENT);
  tft.setCursor(4, y); tft.print("MASTER DT:");
  tft.setTextColor(C_WHITE);
  tft.setCursor(70, y); tft.printf("23.02250, 72.57140");
  y += 12;

  tft.setTextColor(C_CYAN);
  tft.setCursor(4, y); tft.print("A1      :");
  tft.setTextColor(C_WHITE);
  // A1 location would be set via BLE/WiFi config — show stored value
  tft.setCursor(70, y); tft.printf("23.02260, 72.57160");
  y += 12;

  tft.setTextColor(C_BLUE);
  tft.setCursor(4, y); tft.print("A2      :");
  tft.setTextColor(C_WHITE);
  tft.setCursor(70, y); tft.printf("23.02270, 72.57180");
  y += 16;

  // Link quality
  tft.drawFastHLine(0, y, TFT_W, C_DKGREY); y += 6;
  tft.setTextColor(C_LTGREY); tft.setTextSize(1);
  tft.setCursor(4, y); tft.print("RF Link Quality");
  y += 12;
  tft.setTextColor(C_WHITE);
  tft.setCursor(4, y); tft.printf("M<-A1: %3ddBm   A1<-A2: %3ddBm   M RX: %3ddBm",
                                    g_a1.rssi, g_a2.rssi, g_master.rssi);
  y += 14;

  // Timestamp
  tft.setTextColor(C_DKGREY); tft.setTextSize(1);
  tft.setCursor(4, y); tft.print(getTimestamp());
}

// ─────────────────────────────────────────────────────────────────
//  RENDER CURRENT PAGE
// ─────────────────────────────────────────────────────────────────
void renderPage() {
  switch (g_page) {
    case 0: drawPageBrand();      break;
    case 1: drawPageMaster();     break;
    case 2: drawPageA1();         break;
    case 3: drawPageA2();         break;
    case 4: drawPageVIGraph();    break;
    case 5: drawPageSavingGraph();break;
    // case 6: drawPageMap(); — used below via manual trigger
    default: drawPageMap();       break;
  }
}

// ═════════════════════════════════════════════════════════════════
//  SETUP
// ═════════════════════════════════════════════════════════════════
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n══════════════════════════════════════════════════");
  Serial.println("  BlueArc Systems — RECEIVING STATION — Booting");
  Serial.println("══════════════════════════════════════════════════");

  // Zero graph buffers
  memset(g_vHistory,    0, sizeof(g_vHistory));
  memset(g_iHistory,    0, sizeof(g_iHistory));
  memset(g_savWHistory, 0, sizeof(g_savWHistory));
  memset(g_costHistory, 0, sizeof(g_costHistory));

  // ── LEDs ──────────────────────────────────────────────────────
  pinMode(LED_BLUE,  OUTPUT);
  pinMode(LED_GRN_M, OUTPUT);
  pinMode(LED_ORG_M, OUTPUT);
  pinMode(LED_GRN_S, OUTPUT);
  pinMode(LED_ORG_S, OUTPUT);
  // Boot indicator: all orange
  digitalWrite(LED_ORG_M, HIGH); digitalWrite(LED_ORG_S, HIGH);
  digitalWrite(LED_GRN_M, LOW);  digitalWrite(LED_GRN_S, LOW);
  digitalWrite(LED_BLUE, LOW);

  // ── RTC ───────────────────────────────────────────────────────
  Wire.begin(RTC_SDA, RTC_SCL);
  if (!rtc.begin(&Wire)) {
    Serial.println("[WARN] RTC not found");
  } else {
    rtcOk = true;
    if (rtc.lostPower()) rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    Serial.println("[OK] RTC DS3231");
  }

  // ── VSPI: TFT + SD ────────────────────────────────────────────
  SPI.begin(18, 19, 23);

  // TFT boot splash
  tft.init(TFT_W, TFT_H);
  tft.setRotation(0);  // portrait 240×320
  tft.fillScreen(0x0000);
  tft.setTextColor(C_ACCENT); tft.setTextSize(3);
  tft.setCursor(14, 60); tft.print("BLUEARC");
  tft.setTextColor(C_WHITE); tft.setTextSize(2);
  tft.setCursor(28, 104); tft.print("SYSTEMS");
  tft.setTextColor(C_DKGREY); tft.setTextSize(1);
  tft.setCursor(20, 136); tft.print("Receiving Station v2");
  tft.setCursor(20, 150); tft.print("SCADA Interface");
  tft.setTextColor(C_LTGREY); tft.setTextSize(1);
  tft.setCursor(20, 180); tft.print("Initializing...");
  Serial.println("[OK] TFT ST7789");

  // SD
  if (!SD.begin(SD_CS)) {
    Serial.println("[WARN] SD Card not found");
    tft.setTextColor(C_ORANGE);
    tft.setCursor(20, 196); tft.print("SD: NOT FOUND");
  } else {
    sdOk = true;
    Serial.println("[OK] SD Card");
    initSDLogging();
    tft.setTextColor(C_GREEN);
    tft.setCursor(20, 196); tft.print("SD: OK — /BLUEARC_DATA");
  }

  delay(1500);

  // ── LoRa (HSPI) ───────────────────────────────────────────────
  SPIClass* loraSpi = new SPIClass(HSPI);
  loraSpi->begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS);
  LoRa.setSPI(*loraSpi);
  LoRa.setPins(LORA_CS, LORA_RST, LORA_IRQ);

  tft.setTextColor(C_LTGREY); tft.setCursor(20, 212); tft.print("LoRa init...");
  if (!LoRa.begin(LORA_FREQ)) {
    Serial.println("[ERROR] LoRa init failed");
    tft.setTextColor(C_RED); tft.setCursor(20, 228); tft.print("LoRa: FAILED");
    while (true) delay(1000);
  }
  LoRa.setSpreadingFactor(LORA_SF);
  LoRa.setSignalBandwidth(LORA_BW);
  LoRa.setSyncWord(LORA_SYNC);
  LoRa.enableCrc();
  Serial.println("[OK] LoRa receiver @ 433MHz");
  tft.setTextColor(C_GREEN); tft.setCursor(20, 228); tft.print("LoRa: OK @ 433MHz");

  delay(1000);

  // Boot complete
  digitalWrite(LED_ORG_M, LOW); digitalWrite(LED_ORG_S, LOW);
  Serial.println("[Boot] Receiving station ready.\n");

  g_lastPageSwitch = millis();
}

// ═════════════════════════════════════════════════════════════════
//  LOOP
// ═════════════════════════════════════════════════════════════════
unsigned long g_lastDisplayMs = 0;
unsigned long g_lastLogMs     = 0;
#define DISPLAY_REDRAW_MS  500   // redraw every 500ms
#define LOG_INTERVAL_MS   5000   // log every 5s

void loop() {
  unsigned long now = millis();

  // ── 1. RECEIVE LORA ───────────────────────────────────────────────
  int pkt = LoRa.parsePacket();
  if (pkt > 0) {
    String raw = "";
    while (LoRa.available()) raw += (char)LoRa.read();
    int rssi = LoRa.packetRssi();

    // Blue LED blink on RX
    digitalWrite(LED_BLUE, HIGH);

    parseMasterPacket(raw, rssi);
    Serial.printf("[RX] %dB  RSSI=%d\n", pkt, rssi);

    delay(5);
    digitalWrite(LED_BLUE, LOW);
  }

  // ── 2. UPDATE LEDs ────────────────────────────────────────────────
  updateLEDs();

  // ── 3. PAGE ROTATION ─────────────────────────────────────────────
  if (now - g_lastPageSwitch >= PAGE_MS) {
    g_lastPageSwitch = now;
    g_page = (g_page + 1) % NUM_PAGES;
  }

  // ── 4. REDRAW DISPLAY ────────────────────────────────────────────
  if (now - g_lastDisplayMs >= DISPLAY_REDRAW_MS) {
    g_lastDisplayMs = now;
    renderPage();
  }

  // ── 5. SD LOGGING ────────────────────────────────────────────────
  if (now - g_lastLogMs >= LOG_INTERVAL_MS) {
    g_lastLogMs = now;
    if (g_master.valid) logData();
  }

  delay(5);
}
