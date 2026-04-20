#include <WiFi.h>
#include <WebServer.h>
#include <TinyGPS++.h>
#include <HardwareSerial.h>

// --- AP Mode & Network ---
const char* ssid = "ARES-Command-Net"; 
const char* password = "rescueadmin"; 
WebServer server(80);

// --- GPS Setup ---
TinyGPSPlus gps;
HardwareSerial GPS_Serial(2); // Use UART2 (Pins 16 RX, 17 TX)

// --- Motor Pins ---
#define IN1 25
#define IN2 26
#define IN3 27
#define IN4 14

// --- Autonomous Variables ---
const float BATTERY_CAPACITY_MAH = 3000.0;
const float ENERGY_COST_PER_METER_MAH = 0.6; 
const float SAFETY_RESERVE_PERCENT = 15.0; 

double origin_lat = 0.0;
double origin_lon = 0.0;
bool origin_saved = false;

double current_lat = 0.0;
double current_lon = 0.0;
double dist_to_home = 0.0;
double bearing_to_home = 0.0;

int sys_state = 0; // 0=Safe, 1=Warning, 2=Auto-Return
bool manual_rtb_override = false; 

// --- The Dashboard HTML ---
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0, maximum-scale=1.0, user-scalable=no">
    <title>A.R.E.S. Command</title>
    <link href="https://fonts.googleapis.com/css2?family=Orbitron:wght@400;700&display=swap" rel="stylesheet">
    
    <link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css" />
    <script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script>

    <style>
        :root { --bg: #050505; --panel: #111; --safe: #00ffcc; --warn: #ffcc00; --crit: #ff0033; --text: #fff; --dim: #666; }
        body { margin: 0; padding: 15px; background: var(--bg); color: var(--text); font-family: 'Orbitron', sans-serif; height: 100vh; display: flex; flex-direction: column; overflow: hidden; user-select: none; }
        .header { display: flex; justify-content: space-between; padding-bottom: 10px; border-bottom: 2px solid var(--dim); margin-bottom: 15px; }
        #sys-state { font-weight: bold; padding: 5px 15px; border-radius: 4px; border: 1px solid var(--safe); color: var(--safe); }
        .sys-state-warn { border-color: var(--warn) !important; color: var(--warn) !important; animation: pulse 1s infinite; }
        .sys-state-crit { border-color: var(--crit) !important; background: var(--crit) !important; color: #000 !important; animation: blink 0.5s infinite; }
        @keyframes pulse { 50% { opacity: 0.7; } }
        @keyframes blink { 0%, 100% { opacity: 1; } 50% { opacity: 0; } }
        
        /* 4-Column Layout */
        .dashboard { display: grid; grid-template-columns: repeat(4, 1fr); gap: 15px; flex: 1; min-height: 0; }
        .panel { background: var(--panel); border: 1px solid #333; border-radius: 8px; padding: 15px; display: flex; flex-direction: column; overflow: hidden;}
        .panel h2 { margin: 0 0 15px 0; font-size: 0.9rem; color: var(--dim); text-transform: uppercase;}
        
        .big-data { font-size: 2rem; font-weight: bold; }
        .unit { font-size: 0.9rem; color: var(--dim); }
        .data-row { display: flex; justify-content: space-between; border-bottom: 1px solid #222; padding: 8px 0; font-size: 0.9rem; }
        .data-label { color: var(--dim); }
        .prog-bg { width: 100%; height: 8px; background: #222; border-radius: 4px; overflow: hidden; margin-top: 5px; }
        .prog-fill { height: 100%; background: var(--safe); transition: width 0.5s; }

        /* Tactical Controls */
        .d-pad-container { display: flex; justify-content: center; align-items: center; flex: 1; margin-top: 10px;}
        .d-pad { display: grid; grid-template-columns: repeat(3, 60px); grid-template-rows: repeat(3, 60px); gap: 5px; }
        .btn { background: #222; border: 1px solid #444; border-radius: 8px; color: var(--safe); font-size: 1.5rem; display: flex; justify-content: center; align-items: center; cursor: pointer; transition: 0.1s;}
        .btn:active { background: var(--safe); color: #000; transform: scale(0.95); }
        .btn-up { grid-column: 2; grid-row: 1; }
        .btn-left { grid-column: 1; grid-row: 2; }
        .btn-right { grid-column: 3; grid-row: 2; }
        .btn-down { grid-column: 2; grid-row: 3; }

        .rtb-btn { background: transparent; border: 2px solid var(--crit); color: var(--crit); padding: 10px; font-family: 'Orbitron'; font-weight: bold; border-radius: 5px; cursor: pointer; text-transform: uppercase; margin-top: 15px; transition: 0.3s; }
        .rtb-active { background: var(--crit); color: #000; box-shadow: 0 0 15px var(--crit); }

        /* Leaflet Map Integration */
        #map { flex: 1; width: 100%; border-radius: 5px; border: 1px solid #444; margin-top: 10px; background: #222; }
        .leaflet-container { font-family: 'Orbitron', sans-serif; }
    </style>
</head>
<body>
    <div class="header">
        <div style="font-size: 1.5rem; font-weight: bold;">A.R.E.S. OS v2.3 <span style="font-size:0.8rem; color:var(--dim);">GNSS TRACKING</span></div>
        <div id="sys-state">MANUAL SAFE</div>
    </div>
    <div class="dashboard">
        
        <div class="panel">
            <h2>Tactical Command</h2>
            <div class="d-pad-container">
                <div class="d-pad">
                    <div class="btn btn-up" onmousedown="drive('F')" onmouseup="drive('S')" ontouchstart="drive('F')" ontouchend="drive('S')">&#9650;</div>
                    <div class="btn btn-left" onmousedown="drive('L')" onmouseup="drive('S')" ontouchstart="drive('L')" ontouchend="drive('S')">&#9664;</div>
                    <div class="btn btn-right" onmousedown="drive('R')" onmouseup="drive('S')" ontouchstart="drive('R')" ontouchend="drive('S')">&#9654;</div>
                    <div class="btn btn-down" onmousedown="drive('B')" onmouseup="drive('S')" ontouchstart="drive('B')" ontouchend="drive('S')">&#9660;</div>
                </div>
            </div>
            <button id="rtb-toggle" class="rtb-btn" onclick="toggleRTB()">FORCE RTB</button>
        </div>

        <div class="panel">
            <h2>Energy Intelligence</h2>
            <div style="text-align: center; margin-bottom: 15px;">
                <div class="big-data" id="batt-pct">--<span class="unit">%</span></div>
                <div class="prog-bg"><div class="prog-fill" id="batt-bar"></div></div>
            </div>
            <div class="data-row"><span class="data-label">Live Draw</span><span id="current">-- A</span></div>
            <div class="data-row"><span class="data-label">Est. Runtime</span><span id="runtime" style="color: var(--safe);">-- min</span></div>
            <div class="data-row"><span class="data-label">Mode</span><span id="nav-mode">MANUAL</span></div>
        </div>

        <div class="panel">
            <h2>Return Logistics</h2>
            <div style="text-align: center; margin-bottom: 15px;">
                <div class="big-data" id="dist-home">--<span class="unit">m</span></div>
                <div style="color: var(--dim);">from origin</div>
            </div>
            <div class="data-label">Safe Radius Limit</div>
            <div class="prog-bg" style="height: 12px; margin-bottom:15px;"><div class="prog-fill" id="radius-bar" style="background: var(--warn); width: 0%;"></div></div>
            <div class="data-row"><span class="data-label">Return Cost</span><span id="cost">-- mAh</span></div>
            <div class="data-row"><span class="data-label">Safety Margin</span><span id="margin">-- %</span></div>
        </div>

        <div class="panel">
            <div style="display: flex; justify-content: space-between; align-items: baseline;">
                <h2>Tactical Map</h2>
                <div style="font-size: 0.8rem; color: var(--dim);" id="gps-sats">-- Sats</div>
            </div>
            
            <div id="map"></div>
            
            <div style="margin-top: 10px; text-align:center; padding: 5px; background: #222; border-radius: 5px;">
                <div style="font-size: 0.7rem; color: var(--dim);">CURRENT COORDINATES</div>
                <div id="origin-coords" style="font-family: monospace; font-size: 0.9rem; margin-top: 2px;">AWAITING LOCK</div>
            </div>
        </div>
    </div>

    <script>
        // --- MAP INITIALIZATION ---
        // Initialize map centered at 0,0 temporarily
        var map = L.map('map', { zoomControl: false }).setView([0, 0], 2);
        
        // Use CartoDB Dark Matter tiles for the futuristic aesthetic
        L.tileLayer('https://{s}.basemaps.cartocdn.com/dark_all/{z}/{x}/{y}{r}.png', {
            attribution: '&copy; OpenStreetMap &copy; CARTO',
            subdomains: 'abcd',
            maxZoom: 20
        }).addTo(map);

        // Path Drawing line (Cyan)
        var pathLine = L.polyline([], {color: '#00ffcc', weight: 3}).addTo(map);
        
        // Live Rover Marker (Red Dot)
        var roverMarker = L.circleMarker([0, 0], {
            color: '#ff0033',
            fillColor: '#ff0033',
            fillOpacity: 1,
            radius: 5
        }).addTo(map);

        // Origin Marker (Home Base)
        var originMarker = null;
        var mapLockedToRover = false;

        // --- COMMAND FUNCTIONS ---
        function drive(cmd) { fetch('/control?cmd=' + cmd); }

        function toggleRTB() {
            fetch('/toggle_rtb').then(r => r.text()).then(state => {
                let btn = document.getElementById('rtb-toggle');
                if(state === "1") {
                    btn.className = "rtb-btn rtb-active"; btn.innerText = "CANCEL RTB";
                } else {
                    btn.className = "rtb-btn"; btn.innerText = "FORCE RTB";
                }
            });
        }

        // --- DATA POLLING ---
        setInterval(() => {
            fetch('/data').then(r => r.json()).then(d => {
                document.getElementById('batt-pct').innerText = d.batt_pct;
                document.getElementById('batt-bar').style.width = d.batt_pct + '%';
                document.getElementById('current').innerText = d.amps.toFixed(2) + ' A';
                document.getElementById('runtime').innerText = d.runtime_min + ' min';
                
                document.getElementById('dist-home').innerText = d.dist_home;
                document.getElementById('cost').innerText = d.return_cost + ' mAh';
                document.getElementById('margin').innerText = d.safety_margin.toFixed(1) + ' %';
                document.getElementById('radius-bar').style.width = Math.min((d.dist_home / d.max_radius) * 100, 100) + '%';
                
                document.getElementById('gps-sats').innerText = d.sats + ' Sats';

                // --- GNSS MAP UPDATES ---
                if(d.cur_lat !== 0.0 && d.cur_lon !== 0.0) {
                    let latlng = [d.cur_lat, d.cur_lon];
                    
                    document.getElementById('origin-coords').innerText = d.cur_lat.toFixed(5) + ", " + d.cur_lon.toFixed(5);
                    
                    // Update Marker & Path
                    roverMarker.setLatLng(latlng);
                    pathLine.addLatLng(latlng);

                    // Center map on first valid lock, and set zoom to street level
                    if(!mapLockedToRover) {
                        map.setView(latlng, 18);
                        mapLockedToRover = true;
                    }

                    // Draw Home Base if saved
                    if(d.origin_saved && originMarker === null) {
                        originMarker = L.circleMarker([d.origin_lat, d.origin_lon], {
                            color: '#00ffcc', fillColor: '#00ffcc', fillOpacity: 0.5, radius: 8
                        }).addTo(map);
                    }
                }

                // --- SYSTEM STATUS LOGIC ---
                let stateEl = document.getElementById('sys-state');
                let modeEl = document.getElementById('nav-mode');
                
                if(d.sys_state === 2) {
                    stateEl.className = "sys-state-crit"; stateEl.innerText = "AUTO-RETURN ACTIVE";
                    modeEl.innerText = "AUTONOMOUS (RTB)"; modeEl.style.color = "var(--crit)";
                } else if(d.sys_state === 1) {
                    stateEl.className = "sys-state-warn"; stateEl.innerText = "WARNING: BINGO FUEL";
                    modeEl.innerText = "RESTRICTED"; modeEl.style.color = "var(--warn)";
                } else {
                    stateEl.className = ""; stateEl.innerText = "MANUAL SAFE";
                    modeEl.innerText = "MANUAL"; modeEl.style.color = "var(--safe)";
                }
                
                let btn = document.getElementById('rtb-toggle');
                if(d.manual_rtb === 1) {
                    btn.className = "rtb-btn rtb-active"; btn.innerText = "CANCEL RTB";
                } else {
                    btn.className = "rtb-btn"; btn.innerText = "FORCE RTB";
                }
            });
        }, 1000);
    </script>
</body>
</html>
)rawliteral";

void setup() {
  Serial.begin(115200);
  GPS_Serial.begin(9600, SERIAL_8N1, 16, 17);

  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  stopMotors();

  Serial.println("Starting A.R.E.S. AP...");
  WiFi.softAP(ssid, password);
  
  server.on("/", HTTP_GET, []() { server.send(200, "text/html", index_html); });
  
  server.on("/control", HTTP_GET, []() {
    String cmd = server.arg("cmd");
    if (sys_state != 2) { 
        if (cmd == "F") driveForward();
        else if (cmd == "B") driveBackward();
        else if (cmd == "L") turnLeft();
        else if (cmd == "R") turnRight();
        else if (cmd == "S") stopMotors();
    }
    server.send(200, "text/plain", "OK");
  });

  server.on("/toggle_rtb", HTTP_GET, []() {
    manual_rtb_override = !manual_rtb_override;
    if(!manual_rtb_override) stopMotors();
    server.send(200, "text/plain", manual_rtb_override ? "1" : "0");
  });

  server.on("/data", HTTP_GET, handleDataUpdate);
  server.begin();
}

void loop() {
  server.handleClient();
  
  while (GPS_Serial.available() > 0) {
    gps.encode(GPS_Serial.read());
  }

  // Set Origin
  if (gps.location.isValid() && !origin_saved && gps.satellites.value() > 4) {
    origin_lat = gps.location.lat();
    origin_lon = gps.location.lng();
    origin_saved = true;
  }

  // Constantly update current coordinates
  if (gps.location.isValid() && gps.location.isUpdated()) {
    current_lat = gps.location.lat();
    current_lon = gps.location.lng();
    if(origin_saved) {
      dist_to_home = gps.distanceBetween(current_lat, current_lon, origin_lat, origin_lon);
      bearing_to_home = gps.courseTo(current_lat, current_lon, origin_lat, origin_lon);
    }
  }

  if (sys_state == 2 && origin_saved) {
    executeAutoReturn();
  }
}

void executeAutoReturn() {
  if (dist_to_home < 3.0) {
    stopMotors();
    manual_rtb_override = false; 
    return;
  }
  
  float current_heading = 0.0; 
  float heading_error = bearing_to_home - current_heading;
  
  if (heading_error > 180) heading_error -= 360;
  if (heading_error < -180) heading_error += 360;

  if (heading_error > 15) turnRight();
  else if (heading_error < -15) turnLeft(); 
  else driveForward(); 
}

// --- Motor Controls ---
void driveForward() { digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW); digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW); }
void driveBackward() { digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH); digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH); }
void turnRight() { digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW); digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH); }
void turnLeft() { digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH); digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW); }
void stopMotors() { digitalWrite(IN1, LOW); digitalWrite(IN2, LOW); digitalWrite(IN3, LOW); digitalWrite(IN4, LOW); }

void handleDataUpdate() {
  float volts = 7.4; 
  float amps = 1.5; 
  int batt_pct = 85; 
  
  float remaining_mah = (batt_pct / 100.0) * BATTERY_CAPACITY_MAH;
  int runtime_min = (amps > 0) ? (int)((remaining_mah / (amps * 1000.0)) * 60) : 0;
  int return_cost_mah = dist_to_home * ENERGY_COST_PER_METER_MAH;
  float safety_margin = 100.0 - ((return_cost_mah / remaining_mah) * 100.0);
  int max_safe_radius = (int)((remaining_mah - (BATTERY_CAPACITY_MAH * (SAFETY_RESERVE_PERCENT/100.0))) / ENERGY_COST_PER_METER_MAH);

  if (manual_rtb_override || safety_margin < 10.0 || batt_pct <= SAFETY_RESERVE_PERCENT) {
      sys_state = 2; 
  } else if (safety_margin < 25.0) {
      sys_state = 1; 
  } else {
      sys_state = 0; 
  }

  // Build JSON Package
  String json = "{";
  json += "\"batt_pct\":" + String(batt_pct) + ",";
  json += "\"amps\":" + String(amps, 2) + ",";
  json += "\"runtime_min\":" + String(runtime_min) + ",";
  json += "\"dist_home\":" + String((int)dist_to_home) + ",";
  json += "\"return_cost\":" + String(return_cost_mah) + ",";
  json += "\"safety_margin\":" + String(safety_margin, 1) + ",";
  json += "\"max_radius\":" + String(max_safe_radius) + ",";
  json += "\"sys_state\":" + String(sys_state) + ",";
  json += "\"manual_rtb\":" + String(manual_rtb_override ? 1 : 0) + ",";
  
  // GNSS Data
  json += "\"sats\":" + String(gps.satellites.value()) + ",";
  json += "\"cur_lat\":" + String(current_lat, 6) + ",";
  json += "\"cur_lon\":" + String(current_lon, 6) + ",";
  json += "\"origin_saved\":" + String(origin_saved ? "true" : "false") + ",";
  json += "\"origin_lat\":" + String(origin_lat, 6) + ",";
  json += "\"origin_lon\":" + String(origin_lon, 6);
  json += "}";

  server.send(200, "application/json", json);
}
