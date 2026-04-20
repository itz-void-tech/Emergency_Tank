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
HardwareSerial GPS_Serial(2);

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

int sys_state = 0; 
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
        :root { --bg: #050505; --panel: rgba(17, 17, 17, 0.85); --safe: #00ffcc; --warn: #ffcc00; --crit: #ff0033; --text: #fff; --dim: #666; }
        
        /* Tactical Grid Background */
        body { 
            margin: 0; padding: 15px; background-color: var(--bg); 
            background-image: linear-gradient(rgba(0, 255, 204, 0.03) 1px, transparent 1px), linear-gradient(90deg, rgba(0, 255, 204, 0.03) 1px, transparent 1px);
            background-size: 30px 30px;
            color: var(--text); font-family: 'Orbitron', sans-serif; height: 100vh; display: flex; flex-direction: column; overflow: hidden; user-select: none; 
        }
        
        .header { display: flex; justify-content: space-between; padding-bottom: 10px; border-bottom: 2px solid var(--dim); margin-bottom: 15px; }
        #sys-state { font-weight: bold; padding: 5px 15px; border-radius: 4px; border: 1px solid var(--safe); color: var(--safe); backdrop-filter: blur(5px); }
        .sys-state-warn { border-color: var(--warn) !important; color: var(--warn) !important; animation: pulse 1s infinite; }
        .sys-state-crit { border-color: var(--crit) !important; background: var(--crit) !important; color: #000 !important; animation: blink 0.5s infinite; }
        @keyframes pulse { 50% { opacity: 0.7; } }
        @keyframes blink { 0%, 100% { opacity: 1; } 50% { opacity: 0; } }
        
        /* 4-Column Layout */
        .dashboard { display: grid; grid-template-columns: repeat(4, 1fr); gap: 15px; flex: 1; min-height: 0; }
        .panel { background: var(--panel); border: 1px solid #333; border-radius: 8px; padding: 15px; display: flex; flex-direction: column; overflow: hidden; backdrop-filter: blur(10px); box-shadow: 0 4px 15px rgba(0,0,0,0.5); }
        .panel h2 { margin: 0 0 15px 0; font-size: 0.85rem; color: var(--safe); text-transform: uppercase; letter-spacing: 1px; text-shadow: 0 0 8px rgba(0,255,204,0.3); }
        
        .big-data { font-size: 1.8rem; font-weight: bold; }
        .unit { font-size: 0.8rem; color: var(--dim); }
        .data-row { display: flex; justify-content: space-between; border-bottom: 1px solid rgba(255,255,255,0.05); padding: 6px 0; font-size: 0.85rem; }
        .data-label { color: var(--dim); }
        .prog-bg { width: 100%; height: 6px; background: #222; border-radius: 3px; overflow: hidden; margin-top: 5px; }
        .prog-fill { height: 100%; background: var(--safe); transition: width 0.5s; }

        /* Tactical Controls */
        .d-pad-container { display: flex; justify-content: center; align-items: center; flex: 1; margin-top: 5px;}
        .d-pad { display: grid; grid-template-columns: repeat(3, 50px); grid-template-rows: repeat(3, 50px); gap: 5px; }
        .btn { background: #222; border: 1px solid #444; border-radius: 8px; color: var(--safe); font-size: 1.2rem; display: flex; justify-content: center; align-items: center; cursor: pointer; transition: 0.1s; box-shadow: inset 0 0 10px rgba(0,0,0,0.5); }
        .btn:active { background: var(--safe); color: #000; transform: scale(0.95); }
        .btn-up { grid-column: 2; grid-row: 1; }
        .btn-left { grid-column: 1; grid-row: 2; }
        .btn-right { grid-column: 3; grid-row: 2; }
        .btn-down { grid-column: 2; grid-row: 3; }
        .rtb-btn { background: transparent; border: 2px solid var(--crit); color: var(--crit); padding: 8px; font-family: 'Orbitron'; font-weight: bold; border-radius: 5px; cursor: pointer; text-transform: uppercase; margin-top: auto; transition: 0.3s; }
        .rtb-active { background: var(--crit); color: #000; box-shadow: 0 0 15px var(--crit); }

        /* --- NEW: COMPASS WIDGET --- */
        .compass-wrapper { position: relative; width: 100px; height: 100px; margin: 0 auto 15px; }
        .compass-dial { width: 100%; height: 100%; border-radius: 50%; border: 2px solid rgba(255,255,255,0.1); background: rgba(0,0,0,0.6); position: relative; transition: transform 0.15s linear; box-shadow: inset 0 0 20px rgba(0,255,204,0.1); }
        .compass-mark { position: absolute; font-size: 0.7rem; font-weight: bold; color: var(--dim); }
        .mark-n { top: 2px; left: 50%; transform: translateX(-50%); color: var(--safe); text-shadow: 0 0 5px var(--safe); }
        .mark-e { top: 50%; right: 4px; transform: translateY(-50%); }
        .mark-s { bottom: 2px; left: 50%; transform: translateX(-50%); }
        .mark-w { top: 50%; left: 4px; transform: translateY(-50%); }
        .compass-arrow { position: absolute; top: -5px; left: 50%; transform: translateX(-50%); width: 0; height: 0; border-left: 6px solid transparent; border-right: 6px solid transparent; border-bottom: 12px solid var(--crit); z-index: 10; filter: drop-shadow(0 0 4px var(--crit)); }
        
        /* --- NEW: 3D CUBE MPU6050 --- */
        .scene { width: 80px; height: 80px; perspective: 400px; margin: 15px auto 25px; }
        .cube { width: 100%; height: 100%; position: relative; transform-style: preserve-3d; transition: transform 0.1s linear; }
        .cube-face { position: absolute; width: 80px; height: 80px; border: 1px solid rgba(0,255,204,0.6); background: rgba(0,255,204,0.05); display: flex; align-items: center; justify-content: center; font-size: 0.6rem; font-weight: bold; color: var(--safe); box-shadow: inset 0 0 15px rgba(0,255,204,0.15); }
        .face-front  { transform: rotateY(  0deg) translateZ(40px); }
        .face-right  { transform: rotateY( 90deg) translateZ(40px); }
        .face-back   { transform: rotateY(180deg) translateZ(40px); }
        .face-left   { transform: rotateY(-90deg) translateZ(40px); }
        .face-top    { transform: rotateX( 90deg) translateZ(40px); border-color: rgba(255,0,51,0.6); color: var(--crit); } /* Top distinguished */
        .face-bottom { transform: rotateX(-90deg) translateZ(40px); }

        #map { flex: 1; width: 100%; border-radius: 5px; border: 1px solid #444; background: #222; }
    </style>
</head>
<body>
    <div class="header">
        <div style="font-size: 1.5rem; font-weight: bold;">A.R.E.S. OS <span style="font-size:0.8rem; color:var(--safe);">v2.4 ADVANCED</span></div>
        <div id="sys-state">MANUAL SAFE</div>
    </div>
    <div class="dashboard">
        
        <div class="panel">
            <h2>Command</h2>
            <div class="data-row"><span class="data-label">Nav Mode</span><span id="nav-mode" style="font-weight:bold;">MANUAL</span></div>
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
            <h2>Sys & Logistics</h2>
            <div style="display:flex; justify-content: space-between; align-items: flex-end; margin-bottom: 5px;">
                <div class="big-data" id="batt-pct">--<span class="unit">%</span></div>
                <div style="color:var(--safe); font-size: 0.9rem;" id="runtime">-- min</div>
            </div>
            <div class="prog-bg" style="margin-bottom: 15px;"><div class="prog-fill" id="batt-bar"></div></div>
            
            <div class="data-row"><span class="data-label">Load</span><span id="current">-- A</span></div>
            
            <div style="margin-top: 15px; margin-bottom: 5px; color: var(--dim); font-size:0.8rem;">SAFE RADIUS LIMIT</div>
            <div class="prog-bg" style="height: 10px; margin-bottom:10px;"><div class="prog-fill" id="radius-bar" style="background: var(--warn); width: 0%;"></div></div>
            <div class="data-row"><span class="data-label">Distance</span><span id="dist-home" style="color:white; font-weight:bold;">-- m</span></div>
            <div class="data-row"><span class="data-label">RTB Cost</span><span id="cost">-- mAh</span></div>
            <div class="data-row"><span class="data-label">Reserve</span><span id="margin">-- %</span></div>
        </div>

        <div class="panel">
            <h2>Spatial Dynamics</h2>
            
            <div class="compass-wrapper">
                <div class="compass-arrow"></div>
                <div class="compass-dial" id="compass-dial">
                    <div class="compass-mark mark-n">N</div>
                    <div class="compass-mark mark-e">E</div>
                    <div class="compass-mark mark-s">S</div>
                    <div class="compass-mark mark-w">W</div>
                </div>
            </div>
            <div style="text-align:center; font-size:0.85rem; color:var(--safe); margin-bottom: 10px; font-weight:bold; letter-spacing:1px;">
                HDG: <span id="val-heading">--</span>&deg;
            </div>

            <div class="scene">
                <div class="cube" id="mpu-cube">
                    <div class="cube-face face-front">FRONT</div>
                    <div class="cube-face face-back">BACK</div>
                    <div class="cube-face face-right">R</div>
                    <div class="cube-face face-left">L</div>
                    <div class="cube-face face-top">TOP</div>
                    <div class="cube-face face-bottom">BOT</div>
                </div>
            </div>
            <div style="display:flex; justify-content:space-between; font-size:0.75rem; text-align:center;">
                <div><span style="color:var(--dim)">R(X)</span><br><span id="val-roll">--</span>&deg;</div>
                <div><span style="color:var(--dim)">P(Y)</span><br><span id="val-pitch">--</span>&deg;</div>
                <div><span style="color:var(--dim)">Y(Z)</span><br><span id="val-yaw">--</span>&deg;</div>
            </div>
        </div>

        <div class="panel" style="padding-bottom: 5px;">
            <div style="display: flex; justify-content: space-between; align-items: baseline;">
                <h2>Tactical Map</h2>
                <div style="font-size: 0.75rem; color: var(--dim);" id="gps-sats">-- Sats</div>
            </div>
            <div id="map"></div>
            <div style="margin-top: 8px; text-align:center; padding: 5px; background: rgba(0,0,0,0.5); border-radius: 4px; border: 1px solid #333;">
                <div style="font-size: 0.65rem; color: var(--dim);">ORIGIN (HOME)</div>
                <div id="origin-coords" style="font-family: monospace; font-size: 0.8rem; margin-top: 2px;">AWAITING LOCK</div>
            </div>
        </div>
    </div>

    <script>
        var map = L.map('map', { zoomControl: false, attributionControl: false }).setView([0, 0], 2);
        L.tileLayer('https://{s}.basemaps.cartocdn.com/dark_all/{z}/{x}/{y}{r}.png', { maxZoom: 20 }).addTo(map);
        var pathLine = L.polyline([], {color: '#00ffcc', weight: 3}).addTo(map);
        var roverMarker = L.circleMarker([0, 0], { color: '#ff0033', fillColor: '#ff0033', fillOpacity: 1, radius: 4 }).addTo(map);
        var originMarker = null;
        var mapLockedToRover = false;

        function drive(cmd) { fetch('/control?cmd=' + cmd); }
        function toggleRTB() {
            fetch('/toggle_rtb').then(r => r.text()).then(state => {
                let btn = document.getElementById('rtb-toggle');
                if(state === "1") { btn.className = "rtb-btn rtb-active"; btn.innerText = "CANCEL RTB"; } 
                else { btn.className = "rtb-btn"; btn.innerText = "FORCE RTB"; }
            });
        }

        setInterval(() => {
            fetch('/data').then(r => r.json()).then(d => {
                // Systems & Logistics
                document.getElementById('batt-pct').innerText = d.batt_pct;
                document.getElementById('batt-bar').style.width = d.batt_pct + '%';
                document.getElementById('current').innerText = d.amps.toFixed(2) + ' A';
                document.getElementById('runtime').innerText = d.runtime_min + ' min';
                document.getElementById('dist-home').innerText = d.dist_home;
                document.getElementById('cost').innerText = d.return_cost + ' mAh';
                document.getElementById('margin').innerText = d.safety_margin.toFixed(1) + ' %';
                document.getElementById('radius-bar').style.width = Math.min((d.dist_home / d.max_radius) * 100, 100) + '%';
                document.getElementById('gps-sats').innerText = d.sats + ' Sats';

                // --- SPATIAL DYNAMICS ANIMATION ---
                // Compass Rotation (Counter-rotate the dial against the heading)
                document.getElementById('compass-dial').style.transform = `rotate(${-d.heading}deg)`;
                document.getElementById('val-heading').innerText = d.heading;

                // 3D Cube Rotation (CSS Euler Angles)
                document.getElementById('mpu-cube').style.transform = `rotateX(${d.pitch}deg) rotateY(${d.yaw}deg) rotateZ(${d.roll}deg)`;
                document.getElementById('val-roll').innerText = d.roll;
                document.getElementById('val-pitch').innerText = d.pitch;
                document.getElementById('val-yaw').innerText = d.yaw;

                // GNSS Map Updates
                if(d.cur_lat !== 0.0 && d.cur_lon !== 0.0) {
                    let latlng = [d.cur_lat, d.cur_lon];
                    roverMarker.setLatLng(latlng);
                    pathLine.addLatLng(latlng);
                    if(!mapLockedToRover) { map.setView(latlng, 18); mapLockedToRover = true; }
                    if(d.origin_saved) {
                        document.getElementById('origin-coords').innerText = d.origin_lat.toFixed(5) + ", " + d.origin_lon.toFixed(5);
                        if(originMarker === null) {
                            originMarker = L.circleMarker([d.origin_lat, d.origin_lon], { color: '#00ffcc', fillColor: '#00ffcc', fillOpacity: 0.3, radius: 8 }).addTo(map);
                        }
                    }
                }

                // Status State
                let stateEl = document.getElementById('sys-state');
                let modeEl = document.getElementById('nav-mode');
                if(d.sys_state === 2) {
                    stateEl.className = "sys-state-crit"; stateEl.innerText = "AUTO-RETURN ACTIVE";
                    modeEl.innerText = "AUTOPILOT (RTB)"; modeEl.style.color = "var(--crit)";
                } else if(d.sys_state === 1) {
                    stateEl.className = "sys-state-warn"; stateEl.innerText = "WARNING: BINGO FUEL";
                    modeEl.innerText = "RESTRICTED"; modeEl.style.color = "var(--warn)";
                } else {
                    stateEl.className = ""; stateEl.innerText = "MANUAL SAFE";
                    modeEl.innerText = "MANUAL"; modeEl.style.color = "var(--safe)";
                }
            });
        }, 100); // Increased polling to 10Hz (100ms) for smooth IMU/Compass animation
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

  WiFi.softAP(ssid, password);
  server.on("/", HTTP_GET, []() { server.send(200, "text/html", index_html); });
  
  server.on("/control", HTTP_GET, []() {
    String cmd = server.arg("cmd");
    if (sys_state != 2) { 
        if (cmd == "F") driveForward(); else if (cmd == "B") driveBackward();
        else if (cmd == "L") turnLeft(); else if (cmd == "R") turnRight();
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
  while (GPS_Serial.available() > 0) gps.encode(GPS_Serial.read());

  if (gps.location.isValid() && !origin_saved && gps.satellites.value() > 4) {
    origin_lat = gps.location.lat(); origin_lon = gps.location.lng(); origin_saved = true;
  }
  if (gps.location.isValid() && gps.location.isUpdated()) {
    current_lat = gps.location.lat(); current_lon = gps.location.lng();
    if(origin_saved) {
      dist_to_home = gps.distanceBetween(current_lat, current_lon, origin_lat, origin_lon);
      bearing_to_home = gps.courseTo(current_lat, current_lon, origin_lat, origin_lon);
    }
  }

  if (sys_state == 2 && origin_saved) executeAutoReturn();
}

void executeAutoReturn() {
  if (dist_to_home < 3.0) { stopMotors(); manual_rtb_override = false; return; }
  float current_heading = 0.0; // Needs Magnetometer logic
  float heading_error = bearing_to_home - current_heading;
  if (heading_error > 180) heading_error -= 360; if (heading_error < -180) heading_error += 360;
  if (heading_error > 15) turnRight(); else if (heading_error < -15) turnLeft(); else driveForward(); 
}

void driveForward() { digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW); digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW); }
void driveBackward() { digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH); digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH); }
void turnRight() { digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW); digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH); }
void turnLeft() { digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH); digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW); }
void stopMotors() { digitalWrite(IN1, LOW); digitalWrite(IN2, LOW); digitalWrite(IN3, LOW); digitalWrite(IN4, LOW); }

void handleDataUpdate() {
  // --- HARDWARE PLACEHOLDERS ---
  float volts = 7.4; float amps = 1.5; int batt_pct = 85; 
  
  // Simulated IMU & Mag Data (Replace with actual Wire.h sensor reads)
  int sim_heading = millis() / 50 % 360; 
  int sim_roll = (millis() / 30 % 40) - 20; 
  int sim_pitch = (millis() / 40 % 30) - 15;
  int sim_yaw = millis() / 60 % 360;

  float remaining_mah = (batt_pct / 100.0) * BATTERY_CAPACITY_MAH;
  int runtime_min = (amps > 0) ? (int)((remaining_mah / (amps * 1000.0)) * 60) : 0;
  int return_cost_mah = dist_to_home * ENERGY_COST_PER_METER_MAH;
  float safety_margin = 100.0 - ((return_cost_mah / remaining_mah) * 100.0);
  int max_safe_radius = (int)((remaining_mah - (BATTERY_CAPACITY_MAH * (SAFETY_RESERVE_PERCENT/100.0))) / ENERGY_COST_PER_METER_MAH);

  if (manual_rtb_override || safety_margin < 10.0 || batt_pct <= SAFETY_RESERVE_PERCENT) sys_state = 2; 
  else if (safety_margin < 25.0) sys_state = 1; 
  else sys_state = 0; 

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
  
  // Spatial Data
  json += "\"heading\":" + String(sim_heading) + ",";
  json += "\"roll\":" + String(sim_roll) + ",";
  json += "\"pitch\":" + String(sim_pitch) + ",";
  json += "\"yaw\":" + String(sim_yaw) + ",";

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
