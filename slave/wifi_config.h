#pragma once
// ════════════════════════════════════════════════════════════════
//  BlueArc Systems — Slave Node  |  wifi_config.h
//  WiFi AP + Web Dashboard  (no extra libraries — WebServer.h only)
//  Added: live OpenStreetMap/Leaflet map showing node location
// ════════════════════════════════════════════════════════════════

#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include "config.h"

#define WIFI_AP_SSID   "BlueArc-" SLAVE_ID
#define WIFI_AP_PASS   "bluearc123"

extern Preferences prefs;
extern char  g_slaveId[4];
extern float g_lat;
extern float g_lon;
extern bool  g_bypassIntentFlag;
extern bool  g_relayTripped;
extern void  tripRelay(const char* reason);
extern void  clearRelay();
extern NodeData  g_node;
extern float     g_inaVoltageV;
extern float     g_inaCurrentMA;
extern float     g_totalSavingWh;
extern uint8_t   g_currentBrightness;
extern A2Cache   g_a2Cache;

static WebServer g_httpServer(80);

// ─────────────────────────────────────────────────────────────────
static const char DASHBOARD_HTML[] PROGMEM = R"rawhtml(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>BlueArc Node</title>
<link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css"/>
<script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script>
<style>
  :root{--bg:#0d1117;--card:#161b22;--border:#30363d;--accent:#58a6ff;
        --green:#3fb950;--orange:#d29922;--red:#f85149;--txt:#e6edf3;--muted:#8b949e}
  *{box-sizing:border-box;margin:0;padding:0}
  body{background:var(--bg);color:var(--txt);font-family:'Segoe UI',system-ui,sans-serif;
       padding:16px;max-width:680px;margin:auto}
  h1{color:var(--accent);font-size:1.3rem;margin-bottom:16px;letter-spacing:.05em}
  .card{background:var(--card);border:1px solid var(--border);border-radius:8px;
        padding:14px 16px;margin-bottom:12px}
  .card h2{font-size:.75rem;text-transform:uppercase;letter-spacing:.1em;
           color:var(--muted);margin-bottom:10px}
  .row{display:flex;justify-content:space-between;align-items:center;
       padding:4px 0;border-bottom:1px solid var(--border);font-size:.9rem}
  .row:last-child{border-bottom:none}
  .lbl{color:var(--muted)} .val{font-weight:600}
  .ok{color:var(--green)} .warn{color:var(--orange)} .err{color:var(--red)}
  .badge{padding:2px 8px;border-radius:4px;font-size:.78rem;font-weight:600}
  .badge.ok{background:#1a3a22} .badge.warn{background:#3a2e10} .badge.err{background:#3a1010}
  input[type=text]{background:#0d1117;border:1px solid var(--border);border-radius:5px;
    color:var(--txt);padding:6px 10px;width:100%;font-size:.88rem;margin:6px 0}
  button{background:var(--accent);color:#000;border:none;border-radius:5px;
    padding:7px 18px;font-size:.88rem;font-weight:700;cursor:pointer;margin-top:6px;margin-right:4px}
  button.danger{background:var(--red);color:#fff}
  button.secondary{background:var(--card);color:var(--accent);border:1px solid var(--accent)}
  .resp{margin-top:8px;font-size:.82rem;color:var(--green);min-height:18px;word-break:break-word}
  #ts{font-size:.75rem;color:var(--muted);text-align:right;margin-bottom:8px}
  .fault-list{font-size:.82rem;color:var(--red);margin-top:4px}
  #map{height:260px;border-radius:8px;border:1px solid var(--border)}
</style>
</head>
<body>
<h1>⚡ BlueArc Node Dashboard</h1>
<div id="ts">Loading…</div>

<div class="card">
  <h2>Node Info</h2>
  <div class="row"><span class="lbl">Node ID</span><span class="val" id="s-id">…</span></div>
  <div class="row"><span class="lbl">Location</span><span class="val" id="s-loc">…</span></div>
  <div class="row"><span class="lbl">Uptime</span><span class="val" id="s-up">…</span></div>
</div>

<div class="card">
  <h2>📍 Node Location Map</h2>
  <div id="map"></div>
</div>

<div class="card">
  <h2>Sensors</h2>
  <div class="row"><span class="lbl">LDR</span><span class="val" id="s-ldr">…</span></div>
  <div class="row"><span class="lbl">PIR</span><span class="val" id="s-pir">…</span></div>
  <div class="row"><span class="lbl">Tilt angle</span><span class="val" id="s-tilt">…</span></div>
  <div class="row"><span class="lbl">Brightness</span><span class="val" id="s-bri">…</span></div>
</div>

<div class="card">
  <h2>Power (INA260)</h2>
  <div class="row"><span class="lbl">Voltage</span><span class="val" id="s-v">…</span></div>
  <div class="row"><span class="lbl">Current</span><span class="val" id="s-i">…</span></div>
  <div class="row"><span class="lbl">Power</span><span class="val" id="s-p">…</span></div>
  <div class="row"><span class="lbl">Saving</span><span class="val" id="s-sw">…</span></div>
  <div class="row"><span class="lbl">Cost saved</span><span class="val" id="s-cost">…</span></div>
</div>

<div class="card">
  <h2>Relay / Bypass</h2>
  <div class="row"><span class="lbl">Relay</span><span class="val" id="s-rel">…</span></div>
  <div class="row"><span class="lbl">Bypass switch</span><span class="val" id="s-byp">…</span></div>
  <div class="row"><span class="lbl">Bypass intent</span><span class="val" id="s-bypi">…</span></div>
  <div class="row"><span class="lbl">Fault flags</span><span class="val" id="s-flt">…</span></div>
  <div class="fault-list" id="s-flt-list"></div>
</div>

<div class="card" id="card-a2" style="display:none">
  <h2>A2 Node (relay cache)</h2>
  <div class="row"><span class="lbl">Last seen</span><span class="val" id="a2-age">…</span></div>
  <div class="row"><span class="lbl">RSSI</span><span class="val" id="a2-rssi">…</span></div>
  <div class="row"><span class="lbl">PIR</span><span class="val" id="a2-pir">…</span></div>
  <div class="row"><span class="lbl">LDR</span><span class="val" id="a2-ldr">…</span></div>
  <div class="row"><span class="lbl">Tilt</span><span class="val" id="a2-tilt">…</span></div>
  <div class="row"><span class="lbl">Relay</span><span class="val" id="a2-rel">…</span></div>
  <div class="row"><span class="lbl">Faults</span><span class="val" id="a2-flt">…</span></div>
</div>

<div class="card">
  <h2>Configuration</h2>
  <label>Node ID<input type="text" id="f-id" placeholder="e.g. A1" maxlength="3"></label>
  <label>Latitude<input type="text" id="f-lat" placeholder="e.g. 23.0225"></label>
  <label>Longitude<input type="text" id="f-lon" placeholder="e.g. 72.5714"></label>
  <button onclick="cmd('ID:'+ge('f-id').value,'cfg-resp')">Set ID</button>
  <button onclick="cmd('LAT:'+ge('f-lat').value,'cfg-resp')">Set LAT</button>
  <button onclick="cmd('LON:'+ge('f-lon').value,'cfg-resp')">Set LON</button>
  <button onclick="cmd('SAVE','cfg-resp')">💾 Save to Flash</button>
  <div class="resp" id="cfg-resp"></div>
</div>

<div class="card">
  <h2>Lineman Bypass Intent</h2>
  <p style="font-size:.82rem;color:var(--muted);margin-bottom:8px">
    Send BEFORE flipping the SPDT switch so master logs DEVICE FAULT, not wire break.
  </p>
  <input type="text" id="f-byp" placeholder="Node ID (e.g. A1)">
  <button class="secondary" onclick="cmd('BYPASS_INTENT:'+ge('f-byp').value,'byp-resp')">
    🔧 Register Bypass Intent
  </button>
  <div class="resp" id="byp-resp"></div>
</div>

<div class="card">
  <h2>Relay Control</h2>
  <button class="danger" onclick="if(confirm('Reset relay?'))cmd('RESET','relay-resp')">
    ⚡ Reset Relay (restore line)
  </button>
  <div class="resp" id="relay-resp"></div>
</div>

<script>
const FL=[[0,'OVERCURRENT'],[1,'OVERVOLTAGE'],[2,'VOLTAGE SAG'],
          [3,'OVERTEMP'],[4,'HB LOST'],[5,'TILT'],[6,'RELAY TRIP']];
const ge=id=>document.getElementById(id);
const badge=(ok,a,b)=>
  `<span class="badge ${ok?'ok':'err'}">${ok?a:b}</span>`;

// ── Map init ────────────────────────────────────────────────────
let map = L.map('map',{zoomControl:true});
L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png',
  {attribution:'© OpenStreetMap'}).addTo(map);
let nodeMarker = null;
let mapInitialized = false;

function updateMap(lat, lon, nodeId) {
  if (!mapInitialized) {
    map.setView([lat, lon], 16);
    mapInitialized = true;
  }
  const popupText = `<b>BlueArc Node ${nodeId}</b><br>
    Lat: ${lat.toFixed(5)}<br>Lon: ${lon.toFixed(5)}`;
  if (!nodeMarker) {
    nodeMarker = L.marker([lat, lon]).addTo(map).bindPopup(popupText).openPopup();
  } else {
    nodeMarker.setLatLng([lat, lon]).setPopupContent(popupText);
  }
}

async function refresh() {
  try {
    const r = await fetch('/api/status');
    const d = await r.json();
    const upH = Math.floor(d.uptime_s/3600);
    const upM = Math.floor((d.uptime_s%3600)/60);
    const upS = d.uptime_s%60;
    ge('ts').textContent = 'Updated: '+new Date().toLocaleTimeString();
    ge('s-id').textContent = d.id;
    ge('s-loc').textContent = d.lat.toFixed(5)+', '+d.lon.toFixed(5);
    ge('s-up').textContent = `${upH}h ${upM}m ${upS}s`;

    // Update map
    updateMap(d.lat, d.lon, d.id);

    ge('s-ldr').innerHTML = badge(!d.isNight,'☀ DAY','🌙 NIGHT');
    ge('s-pir').innerHTML = badge(!d.pir,'No motion','🚶 MOTION');
    const tc = d.tiltAlarm ? 'err' : 'ok';
    ge('s-tilt').innerHTML = `<span class="${tc}">${d.tiltAngle.toFixed(1)}°${d.tiltAlarm?' ⚠':'  OK'}</span>`;
    ge('s-bri').textContent = d.brightness_pct+'%';
    ge('s-v').textContent  = d.ina_v!=null  ? d.ina_v.toFixed(3)+' V':'N/A';
    ge('s-i').textContent  = d.ina_mA!=null ? d.ina_mA.toFixed(1)+' mA':'N/A';
    ge('s-p').textContent  = d.ina_W!=null  ? d.ina_W.toFixed(2)+' W':'N/A';
    ge('s-sw').textContent = d.saving_W!=null ? d.saving_W.toFixed(2)+' W':'N/A';
    ge('s-cost').textContent = d.cost_rs!=null ? '₹'+d.cost_rs.toFixed(4):'N/A';
    ge('s-rel').innerHTML  = badge(!d.relayTripped,'✅ OK (line live)','⛔ TRIPPED');
    ge('s-byp').innerHTML  = badge(!d.bypassActive,'Normal','🔧 ACTIVE');
    ge('s-bypi').innerHTML = badge(!d.bypassIntent,'Clear','⚠ SET');
    const fh='0x'+d.faultFlags.toString(16).padStart(2,'0').toUpperCase();
    ge('s-flt').innerHTML  = d.faultFlags
      ? `<span class="err">${fh}</span>` : `<span class="ok">None</span>`;
    let fl=''; FL.forEach(([b,l])=>{if(d.faultFlags&(1<<b))fl+='⚠ '+l+'  ';});
    ge('s-flt-list').textContent = fl;

    if (d.isRelayNode) {
      ge('card-a2').style.display = 'block';
      const a = d.a2;
      ge('a2-age').textContent  = a.valid ? a.age_s+'s ago':'Never received';
      ge('a2-rssi').textContent = a.valid ? a.rssi+' dBm':'—';
      ge('a2-pir').innerHTML    = a.valid ? badge(!a.pir,'No motion','🚶 MOTION'):'—';
      ge('a2-ldr').innerHTML    = a.valid ? badge(!a.isNight,'☀ DAY','🌙 NIGHT'):'—';
      ge('a2-tilt').innerHTML   = a.valid ? badge(!a.tilt,'OK','⚠ TILT'):'—';
      ge('a2-rel').innerHTML    = a.valid ? badge(a.relayOK,'✅ OK','⛔ TRIPPED'):'—';
      const af='0x'+a.faultFlags.toString(16).padStart(2,'0').toUpperCase();
      ge('a2-flt').innerHTML    = a.valid
        ? (a.faultFlags ? `<span class="err">${af}</span>`:'<span class="ok">None</span>'):'—';
    }
  } catch(e) {
    ge('ts').textContent = 'Connection error — retrying…';
  }
}

async function cmd(c, rid) {
  ge(rid).textContent='Sending…';
  try {
    const r = await fetch('/api/cmd?c='+encodeURIComponent(c));
    ge(rid).textContent = await r.text();
    setTimeout(refresh, 400);
  } catch(e) { ge(rid).textContent='Error: '+e; }
}

refresh();
setInterval(refresh, 2000);
</script>
</body>
</html>
)rawhtml";

// ── Route handlers ────────────────────────────────────────────────
static void handleApiStatus() {
  String j = "{";
  j += "\"id\":\"";          j += g_slaveId;                        j += "\",";
  j += "\"lat\":";           j += String(g_lat, 5);                 j += ",";
  j += "\"lon\":";           j += String(g_lon, 5);                 j += ",";
  j += "\"uptime_s\":";      j += millis() / 1000;                  j += ",";
  j += "\"isNight\":";       j += g_node.isNight  ? "true":"false"; j += ",";
  j += "\"pir\":";           j += g_node.pir      ? "true":"false"; j += ",";
  j += "\"tiltAngle\":";     j += String(g_node.tiltAngle, 1);      j += ",";
  j += "\"tiltAlarm\":";     j += g_node.tilt     ? "true":"false"; j += ",";
  j += "\"brightness_pct\":";j += (g_currentBrightness * 100) / 255; j += ",";
#if ENABLE_INA260
  j += "\"ina_v\":"    + String(g_inaVoltageV,  3) + ",";
  j += "\"ina_mA\":"   + String(g_inaCurrentMA, 1) + ",";
  j += "\"ina_W\":"    + String(g_node.inaPowerW,2) + ",";
  j += "\"saving_W\":" + String(g_node.savingW,  2) + ",";
  j += "\"cost_rs\":"  + String(g_node.costSaved,4) + ",";
#else
  j += "\"ina_v\":null,\"ina_mA\":null,\"ina_W\":null,\"saving_W\":null,\"cost_rs\":null,";
#endif
  j += "\"relayTripped\":"; j += g_relayTripped      ? "true":"false"; j += ",";
  j += "\"bypassActive\":"; j += g_node.bypassActive ? "true":"false"; j += ",";
  j += "\"bypassIntent\":"; j += g_bypassIntentFlag  ? "true":"false"; j += ",";
  j += "\"faultFlags\":";   j += g_node.faultFlags;                    j += ",";
#if IS_RELAY_NODE
  j += "\"isRelayNode\":true,\"a2\":{";
  j += "\"valid\":";    j += g_a2Cache.valid ? "true":"false"; j += ",";
  j += "\"age_s\":";    j += g_a2Cache.valid ? (millis()-g_a2Cache.lastRxMs)/1000 : 255; j += ",";
  j += "\"rssi\":";     j += g_a2Cache.rssi;                   j += ",";
  j += "\"pir\":";      j += g_a2Cache.pir      ? "true":"false"; j += ",";
  j += "\"isNight\":";  j += g_a2Cache.isNight  ? "true":"false"; j += ",";
  j += "\"tilt\":";     j += g_a2Cache.tilt     ? "true":"false"; j += ",";
  j += "\"relayOK\":";  j += g_a2Cache.relayOK  ? "true":"false"; j += ",";
  j += "\"faultFlags\":"; j += g_a2Cache.faultFlags; j += "}";
#else
  j += "\"isRelayNode\":false,\"a2\":{\"valid\":false,\"age_s\":255,\"rssi\":0,"
       "\"pir\":false,\"isNight\":false,\"tilt\":false,\"relayOK\":true,\"faultFlags\":0}";
#endif
  j += "}";
  g_httpServer.send(200, "application/json", j);
}

static void handleApiCmd() {
  if (!g_httpServer.hasArg("c")) {
    g_httpServer.send(400, "text/plain", "ERR missing ?c=");
    return;
  }
  String cmd = g_httpServer.arg("c");
  cmd.trim();
  String resp;

  if (cmd.startsWith("ID:")) {
    String v = cmd.substring(3); v.trim(); v.toCharArray(g_slaveId, 4);
    resp = "OK ID=" + String(g_slaveId) + " (SAVE to persist)";
  } else if (cmd.startsWith("LAT:")) {
    g_lat = cmd.substring(4).toFloat();
    resp = "OK LAT=" + String(g_lat,5) + " (SAVE to persist)";
  } else if (cmd.startsWith("LON:")) {
    g_lon = cmd.substring(4).toFloat();
    resp = "OK LON=" + String(g_lon,5) + " (SAVE to persist)";
  } else if (cmd == "SAVE") {
    prefs.begin(NVS_NAMESPACE, false);
    prefs.putString(NVS_KEY_ID,  g_slaveId);
    prefs.putFloat(NVS_KEY_LAT,  g_lat);
    prefs.putFloat(NVS_KEY_LON,  g_lon);
    prefs.end();
    resp = "OK Saved: ID=" + String(g_slaveId)
         + " LAT=" + String(g_lat,5) + " LON=" + String(g_lon,5);
  } else if (cmd == "RESET") {
    clearRelay();
    resp = "OK Relay reset — line restored";
  } else if (cmd.startsWith("BYPASS_INTENT:")) {
    String nid = cmd.substring(14); nid.trim();
    if (nid == String(g_slaveId)) {
      g_bypassIntentFlag = true;
      prefs.begin(NVS_NAMESPACE, false);
      prefs.putBool(NVS_KEY_BYPASS_F, true);
      prefs.end();
      resp = "OK BYPASS INTENT for " + String(g_slaveId);
    } else {
      resp = "IGNORED — this node is " + String(g_slaveId) + ", intent was for " + nid;
    }
  } else if (cmd == "STATUS") {
    char fh[5]; sprintf(fh,"0x%02X",g_node.faultFlags);
    resp = "ID=" + String(g_slaveId)
         + " LAT=" + String(g_lat,5) + " LON=" + String(g_lon,5)
         + " | PIR=" + String(g_node.pir?"MOTION":"clear")
         + " Relay=" + String(g_relayTripped?"TRIPPED":"OK")
         + " F=" + fh;
  } else {
    resp = "ERR Unknown. Valid: ID: LAT: LON: SAVE STATUS RESET BYPASS_INTENT:";
  }

  Serial.println("[WiFi CMD] " + cmd + " -> " + resp);
  g_httpServer.send(200, "text/plain", resp);
}

void initWiFiDashboard() {
  WiFi.mode(WIFI_AP);
  WiFi.softAP(WIFI_AP_SSID, WIFI_AP_PASS);
  IPAddress ip = WiFi.softAPIP();
  Serial.printf("[WiFi] AP: %s  IP: %s\n", WIFI_AP_SSID, ip.toString().c_str());
  Serial.printf("[WiFi] Dashboard -> http://%s\n", ip.toString().c_str());

  g_httpServer.on("/",           HTTP_GET, []{ g_httpServer.send_P(200,"text/html",DASHBOARD_HTML); });
  g_httpServer.on("/api/status", HTTP_GET, handleApiStatus);
  g_httpServer.on("/api/cmd",    HTTP_GET, handleApiCmd);
  g_httpServer.onNotFound([]{ g_httpServer.send(404,"text/plain","Not found"); });
  g_httpServer.begin();
  Serial.println("[WiFi] HTTP server on port 80");
}

inline void wifiHandleClient() {
  g_httpServer.handleClient();
}
