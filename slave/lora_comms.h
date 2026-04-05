#pragma once
// ════════════════════════════════════════════════════════════════
//  BlueArc Systems — Slave Node  |  lora_comms.h
//  LoRa SX1278 — packet build, send, receive
//  Chain:  A2 ──LoRa──► A1 ──LoRa──► Master ──LoRa──► Rx Station
//
//  FAST WIRE-BREAK DETECTION:
//    • Packets sent every 500ms (HEARTBEAT_TX_MS)
//    • Each packet carries a sequence number ("sq")
//    • Receiver checks elapsed time AND sequence gaps
//    • Wire break declared after HB_TIMEOUT_MS (1100ms) of silence
//    • This gives ~1s detection vs the old 3s
// ════════════════════════════════════════════════════════════════

#include <SPI.h>
#include <LoRa.h>
#include <ArduinoJson.h>
#include "config.h"

// ── Globals shared with slave.ino ────────────────────────────────
struct NodeData {
  bool  pir;
  bool  isNight;
  bool  tilt;
  float tiltAngle;
  float inaCurrentA;
  float inaPowerW;
  float savingW;
  float costSaved;
  bool  relayOK;
  bool  bypassActive;
  bool  bypassIntent;
  uint8_t faultFlags;
};

struct A2Cache {
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
};

static A2Cache   g_a2Cache;
static uint32_t  g_txSeq = 0;   // monotonic TX sequence counter

// ── LoRa Init ────────────────────────────────────────────────────
bool initLoRa() {
  SPI.begin(PIN_LORA_SCK, PIN_LORA_MISO, PIN_LORA_MOSI, PIN_LORA_NSS);
  LoRa.setSPI(SPI);
  LoRa.setPins(PIN_LORA_NSS, /*rst=*/-1, PIN_LORA_DIO0);

  if (!LoRa.begin(LORA_FREQUENCY)) {
    Serial.println("[LoRa] INIT FAILED");
    return false;
  }

  LoRa.setSpreadingFactor(LORA_SF);
  LoRa.setSignalBandwidth(LORA_BANDWIDTH);
  LoRa.setSyncWord(LORA_SYNC_WORD);
  LoRa.setTxPower(LORA_TX_POWER);
  LoRa.enableCrc();

  Serial.printf("[LoRa] Ready — %.0f MHz SF%d BW%.0fkHz\n",
                LORA_FREQUENCY / 1E6, LORA_SF, LORA_BANDWIDTH / 1E3);
  return true;
}

// ── Build + Send packet ──────────────────────────────────────────
void sendPacket(const NodeData& nd, const char* slaveId) {
  StaticJsonDocument<400> doc;

  doc["src"]  = slaveId;
  doc["sq"]   = ++g_txSeq;           // sequence number — receiver detects gaps
  doc["pir"]  = nd.pir  ? 1 : 0;
  doc["ldr"]  = nd.isNight ? 0 : 1;
  doc["tilt"] = nd.tilt ? 1 : 0;
  doc["tiltA"]= (int)nd.tiltAngle;
  doc["rel"]  = nd.relayOK ? 1 : 0;
  doc["byp"]  = nd.bypassActive ? 1 : 0;
  doc["bypI"] = nd.bypassIntent ? 1 : 0;
  doc["F"]    = nd.faultFlags;

#if ENABLE_INA260
  doc["inaI"] = serialized(String(nd.inaCurrentA, 3));
  doc["inaP"] = serialized(String(nd.inaPowerW,   2));
  doc["savW"] = serialized(String(nd.savingW,      2));
  doc["cost"] = serialized(String(nd.costSaved,    2));
#endif

#if IS_RELAY_NODE
  if (g_a2Cache.valid) {
    JsonObject a2 = doc.createNestedObject("A2");
    a2["pir"]  = g_a2Cache.pir  ? 1 : 0;
    a2["ldr"]  = g_a2Cache.isNight ? 0 : 1;
    a2["tilt"] = g_a2Cache.tilt ? 1 : 0;
    a2["rel"]  = g_a2Cache.relayOK ? 1 : 0;
    a2["byp"]  = g_a2Cache.bypassActive ? 1 : 0;
    a2["bypI"] = g_a2Cache.bypassIntent ? 1 : 0;
    a2["F"]    = g_a2Cache.faultFlags;
    a2["rssi"] = g_a2Cache.rssi;
    a2["age"]  = (millis() - g_a2Cache.lastRxMs) / 1000;
    a2["sq"]   = g_a2Cache.lastSeq;   // forward last known A2 sequence
  } else {
    JsonObject a2 = doc.createNestedObject("A2");
    a2["F"]  = FAULT_HB_LOST;
    a2["age"]= 255;
    a2["sq"] = 0;
  }
#endif

  String payload;
  serializeJson(doc, payload);

  LoRa.beginPacket();
  LoRa.print(payload);
  LoRa.endPacket();

  Serial.printf("[LoRa TX] sq=%u  %s\n", g_txSeq, payload.c_str());
}

// ── Receive A2 packet (A1 listens, non-blocking) ─────────────────
bool receiveA2Packet() {
#if !IS_RELAY_NODE
  return false;
#endif

  int packetSize = LoRa.parsePacket();
  if (packetSize == 0) return false;

  String raw = "";
  while (LoRa.available()) raw += (char)LoRa.read();
  int rssi = LoRa.packetRssi();

  StaticJsonDocument<256> doc;
  DeserializationError err = deserializeJson(doc, raw);
  if (err) {
    Serial.printf("[LoRa RX] JSON error: %s\n", err.c_str());
    return false;
  }

  const char* src = doc["src"] | "";
  if (strcmp(src, "A2") != 0) return false;

  g_a2Cache.valid        = true;
  g_a2Cache.lastRxMs     = millis();
  g_a2Cache.lastSeq      = doc["sq"]   | 0;
  g_a2Cache.pir          = doc["pir"]  | 0;
  g_a2Cache.isNight      = (doc["ldr"] | 1) == 0;
  g_a2Cache.tilt         = doc["tilt"] | 0;
  g_a2Cache.relayOK      = doc["rel"]  | 1;
  g_a2Cache.bypassActive = doc["byp"]  | 0;
  g_a2Cache.bypassIntent = doc["bypI"] | 0;
  g_a2Cache.faultFlags   = doc["F"]    | 0;
  g_a2Cache.rssi         = rssi;

  Serial.printf("[LoRa RX A2] sq=%u  RSSI=%d\n", g_a2Cache.lastSeq, rssi);
  return true;
}

// ── Check A2 heartbeat timeout ───────────────────────────────────
bool checkA2Timeout() {
#if !IS_RELAY_NODE
  return false;
#endif
  if (!g_a2Cache.valid) return false;
  if (millis() - g_a2Cache.lastRxMs > HB_TIMEOUT_MS) {
    g_a2Cache.faultFlags |= FAULT_HB_LOST;
    return true;
  }
  return false;
}
