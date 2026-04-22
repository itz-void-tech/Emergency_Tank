#include <WiFi.h>
#include <WebServer.h>
#include <TinyGPS++.h>
#include <HardwareSerial.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_HMC5883_U.h>
#include <Adafruit_INA219.h>
#include <Adafruit_Sensor.h>

// --- Sensors & Hardware Objects ---
Adafruit_MPU6050 mpu;
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);
Adafruit_INA219 ina219;
TinyGPSPlus gps;
HardwareSerial GPS_Serial(2);
WebServer server(80);

// --- AP Mode & Network ---
const char* ssid = "ARES-Command-Net"; 
const char* password = "rescueadmin"; 

// --- Motor Pins ---
#define ENA 32  
#define IN1 25
#define IN2 26
#define IN3 27
#define IN4 14
#define ENB 33  

// --- Navigation & Power Variables ---
double origin_lat = 0, origin_lon = 0, current_lat = 0, current_lon = 0;
bool origin_saved = false;
double dist_to_home = 0, bearing_to_home = 0;
int sys_state = 0; 
bool manual_rtb_override = false; 
int current_speed = 255;

// --- Sensor Fusion Variables ---
float roll = 0, pitch = 0, yaw = 0;
float mag_heading = 0;
float filter_alpha = 0.96; 
unsigned long last_micros;

// --- Battery Logistics ---
const float BATTERY_CAPACITY_MAH = 3000.0;
const float ENERGY_COST_PER_METER_MAH = 0.6; 
const float SAFETY_RESERVE_PERCENT = 15.0;
float bus_voltage = 0, current_amps = 0;

// ==========================================
//   MOTOR & UTILITY FUNCTIONS (Defined First)
// ==========================================

void stopMotors() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
}

void driveForward() {
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
}

void driveBackward() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
}

void turnRight() {
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
}

void turnLeft() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
}

void updateSensors() {
  sensors_event_t a, g, temp, m;
  mpu.getEvent(&a, &g, &temp);
  mag.getEvent(&m);

  unsigned long now = micros();
  float dt = (now - last_micros) / 1000000.0;
  last_micros = now;

  float roll_acc = atan2(a.acceleration.y, a.acceleration.z) * 180 / PI;
  float pitch_acc = atan2(-a.acceleration.x, sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z)) * 180 / PI;

  roll = filter_alpha * (roll + g.gyro.x * dt * 180 / PI) + (1 - filter_alpha) * roll_acc;
  pitch = filter_alpha * (pitch + g.gyro.y * dt * 180 / PI) + (1 - filter_alpha) * pitch_acc;

  float phi = roll * PI / 180;
  float theta = pitch * PI / 180;
  float Xh = m.magnetic.x * cos(theta) + m.magnetic.z * sin(theta);
  float Yh = m.magnetic.x * sin(phi) * sin(theta) + m.magnetic.y * cos(phi) - m.magnetic.z * sin(phi) * cos(theta);
  
  mag_heading = atan2(Yh, Xh) * 180 / PI;
  if (mag_heading < 0) mag_heading += 360;

  bus_voltage = ina219.getBusVoltage_V();
  current_amps = ina219.getCurrent_mA() / 1000.0;
}

void updateGPS() {
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
}

void executeAutoReturn() {
  if (dist_to_home < 3.0) { stopMotors(); manual_rtb_override = false; return; }
  float heading_error = bearing_to_home - mag_heading;
  if (heading_error > 180) heading_error -= 360; if (heading_error < -180) heading_error += 360;
  if (heading_error > 15) turnRight(); else if (heading_error < -15) turnLeft(); else driveForward(); 
}

// ==========================================
//   HTML & WEB HANDLERS
// ==========================================

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
        body { margin: 0; padding: 15px; background-color: var(--bg); background-image: linear-gradient(rgba(0, 255, 204, 0.03) 1px, transparent 1px), linear-gradient(90deg, rgba(0, 255, 204, 0.03) 1px, transparent 1px); background-size: 30px 30px; color: var(--text); font-family: 'Orbitron', sans-serif; height: 100vh; display: flex; flex-direction: column; overflow: hidden; }
        .header { display: flex; justify-content: space-between; padding-bottom: 10px; border-bottom: 2px solid var(--dim); margin-bottom: 15px; }
         #sys-state { font-weight: bold; padding: 5px 15px; border-radius: 4px; border: 1px solid var(--safe); color: var(--safe); backdrop-filter: blur(5px); }
        .sys-state-warn { border-color: var(--warn) !important; color: var(--warn) !important; animation: pulse 1s infinite; }
        .sys-state-crit { border-color: var(--crit) !important; background: var(--crit) !important; color: #000 !important; animation: blink 0.5s infinite; }
        @keyframes pulse { 50% { opacity: 0.7; } }
        @keyframes blink { 0%, 100% { opacity: 1; } 50% { opacity: 0; } }
        .dashboard { display: grid; grid-template-columns: repeat(4, 1fr); gap: 15px; flex: 1; min-height: 0; }
        .panel { background: var(--panel); border: 1px solid #333; border-radius: 8px; padding: 15px; display: flex; flex-direction: column; overflow: hidden; backdrop-filter: blur(10px); }
        .panel h2 { margin: 0 0 15px 0; font-size: 0.85rem; color: var(--safe); text-transform: uppercase; }
        .big-data { font-size: 1.8rem; font-weight: bold; }
        .unit { font-size: 0.8rem; color: var(--dim); }
        .data-row { display: flex; justify-content: space-between; border-bottom: 1px solid rgba(255,255,255,0.05); padding: 6px 0; font-size: 0.85rem; }
        .data-label { color: var(--dim); }
        .prog-bg { width: 100%; height: 6px; background: #222; border-radius: 3px; overflow: hidden; }
        .prog-fill { height: 100%; background: var(--safe); transition: width 0.5s; }
        .d-pad-container { display: flex; justify-content: center; align-items: center; flex: 1; }
        .d-pad { display: grid; grid-template-columns: repeat(3, 50px); grid-template-rows: repeat(3, 50px); gap: 5px; }
        .btn { background: #222; border: 1px solid #444; border-radius: 8px; color: var(--safe); font-size: 1.2rem; display: flex; justify-content: center; align-items: center; cursor: pointer; }
        .btn:active { background: var(--safe); color: #000; }
        .btn-up { grid-column: 2; grid-row: 1; }
        .btn-left { grid-column: 1; grid-row: 2; }
        .btn-right { grid-column: 3; grid-row: 2; }
        .btn-down { grid-column: 2; grid-row: 3; }
        .slider { -webkit-appearance: none; width: 100%; height: 6px; background: #222; border-radius: 3px; outline: none; margin-top: 8px; }
        .slider::-webkit-slider-thumb { -webkit-appearance: none; width: 16px; height: 16px; border-radius: 50%; background: var(--safe); cursor: pointer; box-shadow: 0 0 10px var(--safe); }
        .rtb-btn { background: transparent; border: 2px solid var(--crit); color: var(--crit); padding: 8px; font-family: 'Orbitron'; font-weight: bold; border-radius: 5px; cursor: pointer; margin-top: auto; }
        .rtb-active { background: var(--crit); color: #000; }
        .compass-wrapper { position: relative; width: 100px; height: 100px; margin: 0 auto 15px; }
        .compass-dial { width: 100%; height: 100%; border-radius: 50%; border: 2px solid rgba(255,255,255,0.1); position: relative; transition: transform 0.15s linear; }
        .compass-mark { position: absolute; font-size: 0.7rem; font-weight: bold; color: var(--dim); }
        .mark-n { top: 2px; left: 50%; transform: translateX(-50%); color: var(--safe); }
        .compass-arrow { position: absolute; top: -5px; left: 50%; transform: translateX(-50%); width: 0; height: 0; border-left: 6px solid transparent; border-right: 6px solid transparent; border-bottom: 12px solid var(--crit); z-index: 10; }
        .scene { width: 80px; height: 80px; perspective: 400px; margin: 15px auto 25px; }
        .cube { width: 100%; height: 100%; position: relative; transform-style: preserve-3d; transition: transform 0.1s linear; }
        .cube-face { position: absolute; width: 80px; height: 80px; border: 1px solid rgba(0,255,204,0.6); background: rgba(0,255,204,0.05); display: flex; align-items: center; justify-content: center; font-size: 0.6rem; color: var(--safe); }
        .face-front { transform: rotateY(0deg) translateZ(40px); }
        .face-back { transform: rotateY(180deg) translateZ(40px); }
        .face-right { transform: rotateY(90deg) translateZ(40px); }
        .face-left { transform: rotateY(-90deg) translateZ(40px); }
        .face-top { transform: rotateX(90deg) translateZ(40px); border-color: var(--crit); color: var(--crit); }
        .face-bottom { transform: rotateX(-90deg) translateZ(40px); }
         #map { flex: 1; width: 100%; border-radius: 5px; border: 1px solid #444; background: #222; }
    </style>
</head>
<body>
    <div class="header">
        <div style="font-size: 1.5rem; font-weight: bold;">A.R.E.S. OS <span style="font-size:0.8rem; color:var(--safe);">v2.5</span></div>
        <div id="sys-state">MANUAL SAFE</div>
    </div>
    <div class="dashboard">
        <div class="panel">
            <h2>Command</h2>
            <div class="data-row"><span class="data-label">Nav Mode</span><span id="nav-mode">MANUAL</span></div>
            <div class="d-pad-container">
                <div class="d-pad">
                    <div class="btn btn-up" onmousedown="drive('F')" onmouseup="drive('S')" ontouchstart="drive('F')" ontouchend="drive('S')">&#9650;</div>
                    <div class="btn btn-left" onmousedown="drive('L')" onmouseup="drive('S')" ontouchstart="drive('L')" ontouchend="drive('S')">&#9664;</div>
                    <div class="btn btn-right" onmousedown="drive('R')" onmouseup="drive('S')" ontouchstart="drive('R')" ontouchend="drive('S')">&#9654;</div>
                    <div class="btn btn-down" onmousedown="drive('B')" onmouseup="drive('S')" ontouchstart="drive('B')" ontouchend="drive('S')">&#9660;</div>
                </div>
            </div>
            <input type="range" min="0" max="255" value="255" class="slider" id="speed-slider" onchange="setSpeed(this.value)">
            <button id="rtb-toggle" class="rtb-btn" onclick="toggleRTB()">FORCE RTB</button>
        </div>
        <div class="panel">
            <h2>Logistics</h2>
            <div class="big-data" id="batt-pct">--%</div>
            <div class="prog-bg"><div class="prog-fill" id="batt-bar"></div></div>
            <div class="data-row"><span class="data-label">Current</span><span id="current">-- A</span></div>
            <div class="data-row"><span class="data-label">Dist</span><span id="dist-home">-- m</span></div>
        </div>
        <div class="panel">
            <h2>Spatial</h2>
            <div class="compass-wrapper">
                <div class="compass-arrow"></div>
                <div class="compass-dial" id="compass-dial">
                    <div class="compass-mark mark-n">N</div>
                </div>
            </div>
            <div class="scene"><div class="cube" id="mpu-cube"><div class="cube-face face-front">FRONT</div><div class="cube-face face-top">TOP</div></div></div>
        </div>
        <div class="panel">
            <h2>Tactical Map</h2>
            <div id="map"></div>
        </div>
    </div>
    <script>
        var map = L.map('map', { zoomControl: false }).setView([0, 0], 2);
        L.tileLayer('https://{s}.basemaps.cartocdn.com/dark_all/{z}/{x}/{y}{r}.png').addTo(map);
        var pathLine = L.polyline([], {color: '#00ffcc'}).addTo(map);
        var roverMarker = L.circleMarker([0, 0], { color: '#ff0033', radius: 5 }).addTo(map);

        function drive(cmd) { fetch('/control?cmd=' + cmd); }
        function setSpeed(v) { fetch('/speed?val=' + v); }
        function toggleRTB() { fetch('/toggle_rtb'); }

        setInterval(() => {
            fetch('/data').then(r => r.json()).then(d => {
                document.getElementById('batt-pct').innerText = d.batt_pct + '%';
                document.getElementById('batt-bar').style.width = d.batt_pct + '%';
                document.getElementById('current').innerText = d.amps.toFixed(2) + ' A';
                document.getElementById('dist-home').innerText = d.dist_home + ' m';
                document.getElementById('compass-dial').style.transform = `rotate(${-d.heading}deg)`;
                document.getElementById('mpu-cube').style.transform = `rotateX(${d.pitch}deg) rotateY(${d.yaw}deg) rotateZ(${d.roll}deg)`;
                if(d.cur_lat != 0) {
                    let ll = [d.cur_lat, d.cur_lon];
                    roverMarker.setLatLng(ll);
                    pathLine.addLatLng(ll);
                    map.panTo(ll);
                }
            });
        }, 200);
    </script>
</body>
</html>
)rawliteral";

void handleDataUpdate() {
  int batt_pct = map(constrain(bus_voltage * 100, 680, 840), 680, 840, 0, 100);
  int return_cost_mah = dist_to_home * ENERGY_COST_PER_METER_MAH;
  float remaining_mah = (batt_pct / 100.0) * BATTERY_CAPACITY_MAH;
  float safety_margin = 100.0 - ((float)return_cost_mah / remaining_mah * 100.0);

  if (manual_rtb_override || safety_margin < 10.0 || batt_pct <= SAFETY_RESERVE_PERCENT) sys_state = 2; 
  else if (safety_margin < 25.0) sys_state = 1; 
  else sys_state = 0; 

  String json = "{";
  json += "\"batt_pct\":" + String(batt_pct) + ",";
  json += "\"amps\":" + String(current_amps, 2) + ",";
  json += "\"dist_home\":" + String((int)dist_to_home) + ",";
  json += "\"sys_state\":" + String(sys_state) + ",";
  json += "\"heading\":" + String((int)mag_heading) + ",";
  json += "\"roll\":" + String((int)roll) + ",";
  json += "\"pitch\":" + String((int)pitch) + ",";
  json += "\"yaw\":" + String((int)mag_heading) + ",";
  json += "\"cur_lat\":" + String(current_lat, 6) + ",";
  json += "\"cur_lon\":" + String(current_lon, 6);
  json += "}";
  server.send(200, "application/json", json);
}

void setupWebEndpoints() {
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

  server.on("/speed", HTTP_GET, []() {
    if(server.hasArg("val")) {
        current_speed = server.arg("val").toInt();
        analogWrite(ENA, current_speed); analogWrite(ENB, current_speed);
    }
    server.send(200, "text/plain", "OK");
  });

  server.on("/toggle_rtb", HTTP_GET, []() {
    manual_rtb_override = !manual_rtb_override;
    if(!manual_rtb_override) stopMotors();
    server.send(200, "text/plain", "OK");
  });

  server.on("/data", HTTP_GET, handleDataUpdate);
}

// ==========================================
//   MAIN SETUP & LOOP
// ==========================================

void setup() {
  Serial.begin(115200);
  GPS_Serial.begin(9600, SERIAL_8N1, 16, 17);
  Wire.begin(21, 22);

  if (!mpu.begin()) Serial.println("MPU6050 Fail");
  if (!mag.begin()) Serial.println("HMC5883L Fail");
  if (!ina219.begin()) Serial.println("INA219 Fail");

  pinMode(ENA, OUTPUT); pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  
  analogWrite(ENA, current_speed); analogWrite(ENB, current_speed);
  stopMotors();

  WiFi.softAP(ssid, password);
  setupWebEndpoints();
  server.begin();
  last_micros = micros();
}

void loop() {
  server.handleClient();
  updateSensors();
  updateGPS();
  if (sys_state == 2 && origin_saved) executeAutoReturn();
}
