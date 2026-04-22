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
int current_speed = 200; // Default safer speed

// --- Sensor Fusion Variables ---
float roll = 0, pitch = 0, mag_heading = 0;
float filter_alpha = 0.96; 
unsigned long last_micros;
unsigned long sensor_timer = 0;

// --- Battery Logistics ---
const float BATTERY_CAPACITY_MAH = 3000.0;
const float ENERGY_COST_PER_METER_MAH = 0.6; 
const float SAFETY_RESERVE_PERCENT = 15.0;
float bus_voltage = 0, current_amps = 0;

// ==========================================
//   MOTOR & UTILITY FUNCTIONS
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
  // Only update I2C sensors every 50ms to prevent bus collisions
  if (millis() - sensor_timer < 50) return;
  sensor_timer = millis();

  sensors_event_t a, g, temp, m;
  mpu.getEvent(&a, &g, &temp);
  mag.getEvent(&m);

  unsigned long now = micros();
  float dt = (now - last_micros) / 1000000.0;
  if (dt <= 0 || dt > 0.5) dt = 0.01;
  last_micros = now;

  // Pitch and Roll from Accelerometer
  float roll_acc = atan2(a.acceleration.y, a.acceleration.z) * 180 / PI;
  float pitch_acc = atan2(-a.acceleration.x, sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z)) * 180 / PI;

  // Complementary Filter
  roll = filter_alpha * (roll + g.gyro.x * dt * 180 / PI) + (1 - filter_alpha) * roll_acc;
  pitch = filter_alpha * (pitch + g.gyro.y * dt * 180 / PI) + (1 - filter_alpha) * pitch_acc;

  // Tilt Compensated Magnetometer Heading
  float phi = roll * PI / 180.0;
  float theta = pitch * PI / 180.0;
  
  float Xh = m.magnetic.x * cos(theta) + m.magnetic.y * sin(phi) * sin(theta) + m.magnetic.z * cos(phi) * sin(theta);
  float Yh = m.magnetic.y * cos(phi) - m.magnetic.z * sin(phi);
  
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
    <title>A.R.E.S. UNIT 01 HUD</title>
    <link href="https://fonts.googleapis.com/css2?family=Syncopate:wght@700&family=Share+Tech+Mono&display=swap" rel="stylesheet">
    <link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css" />
    <style>
        :root { --neon-blue: #00f2ff; --neon-red: #ff3131; --bg-dark: #020406; --panel-bg: rgba(0, 20, 35, 0.8); --font-main: 'Share Tech Mono', monospace; }
        * { box-sizing: border-box; -webkit-tap-highlight-color: transparent; }
        body { margin: 0; background-color: var(--bg-dark); color: var(--neon-blue); font-family: var(--font-main); height: 100vh; overflow: hidden; display: flex; flex-direction: column; }
        body::before { content: " "; display: block; position: absolute; top: 0; left: 0; bottom: 0; right: 0; background: linear-gradient(rgba(18, 16, 16, 0) 50%, rgba(0, 0, 0, 0.2) 50%), linear-gradient(90deg, rgba(255, 0, 0, 0.03), rgba(0, 255, 0, 0.01), rgba(0, 0, 255, 0.03)); z-index: 50; background-size: 100% 3px, 3px 100%; pointer-events: none; opacity: 0.7; }
        .header { padding: 8px 25px; display: flex; justify-content: space-between; align-items: center; border-bottom: 2px solid var(--neon-blue); background: rgba(0, 242, 255, 0.05); }
        .dashboard { display: grid; grid-template-columns: 320px 1fr 340px; grid-template-rows: 1fr 140px; gap: 12px; padding: 12px; flex: 1; min-height: 0; }
        .panel { background: var(--panel-bg); border: 1px solid rgba(0, 242, 255, 0.3); display: flex; flex-direction: column; position: relative; border-radius: 4px; overflow: hidden; }
        .panel-h { background: rgba(0, 242, 255, 0.15); padding: 4px 10px; font-size: 0.75rem; letter-spacing: 1px; display: flex; justify-content: space-between; font-weight: bold; }
        .attitude-container { width: 160px; height: 160px; border-radius: 50%; border: 2px solid var(--neon-blue); margin: 15px auto; position: relative; overflow: hidden; background: #000; }
        .horizon-inner { position: absolute; width: 400px; height: 400px; left: -120px; top: -120px; background: linear-gradient(to bottom, #1a4d70 49.5%, #fff 50%, #4a2f1a 50.5%); transition: transform 0.1s linear; }
        .crosshair { position: absolute; width: 60px; height: 2px; background: var(--neon-red); top: 50%; left: 50%; transform: translate(-50%, -50%); z-index: 10; }
        .compass-box { position: relative; height: 160px; display: flex; justify-content: center; align-items: center; }
        .compass-dial { width: 140px; height: 140px; border-radius: 50%; border: 1px dashed var(--neon-blue); position: relative; }
        .cardinal { position: absolute; font-weight: bold; font-size: 0.8rem; }
        .N { top: 5px; left: 50%; transform: translateX(-50%); color: var(--neon-red); }
        .S { bottom: 5px; left: 50%; transform: translateX(-50%); }
        .E { right: 8px; top: 50%; transform: translateY(-50%); }
        .W { left: 8px; top: 50%; transform: translateY(-50%); }
        .controls { display: grid; grid-template-columns: repeat(3, 1fr); gap: 8px; padding: 15px; margin: auto; }
        .joy-btn { width: 55px; height: 55px; border: 1px solid var(--neon-blue); background: rgba(0, 242, 255, 0.05); color: var(--neon-blue); display: flex; align-items: center; justify-content: center; border-radius: 6px; font-size: 1.2rem; cursor: pointer; }
        .joy-btn:active { background: var(--neon-blue); color: #000; }
        .stat-line { padding: 10px 15px; border-bottom: 1px solid rgba(0, 242, 255, 0.1); }
        .stat-val { font-size: 1.4rem; color: #fff; text-shadow: 0 0 8px var(--neon-blue); }
        #map { flex: 1; filter: invert(90%) hue-rotate(180deg) brightness(0.9); }
        .rtb-box { margin: 15px; padding: 10px; border: 1px solid var(--neon-red); color: var(--neon-red); text-align: center; cursor: pointer; animation: blink-slow 2s infinite; font-weight: bold;}
        .rtb-active { background: var(--neon-red); color: #fff !important; }
        @keyframes blink-slow { 50% { opacity: 0.4; } }
        .sys-crit { --neon-blue: #ff3131 !important; color: #ff3131 !important; }
    </style>
</head>
<body>
    <div class="header">
        <div style="font-family: 'Syncopate';">A.R.E.S. GCS UNIT_01</div>
        <div id="connection">SAT_LINK: ONLINE</div>
        <div id="clock">00:00:00</div>
    </div>
    <div class="dashboard" id="main-ui">
        <div class="panel">
            <div class="panel-h"><span>LOGISTICS TELEMETRY</span></div>
            <div class="stat-line"><small>BATT VOLTAGE</small><br><span class="stat-val" id="batt-pct">--%</span> <small id="v-val">(0.0 V)</small></div>
            <div class="stat-line"><small>DIST TO ORIGIN</small><br><span class="stat-val" id="dist-val">-- M</span></div>
            <div style="flex:1; text-align:center; padding-top:20px"><div id="sys-mode" style="font-size:1.2rem">NOMINAL</div></div>
            <div id="rtb-btn" class="rtb-box" onclick="toggleRTB()">INIT RTB AUTO-RETURN</div>
        </div>
        <div class="panel"><div id="map"></div></div>
        <div class="panel">
            <div class="panel-h"><span>SPATIAL HUD</span></div>
            <div class="attitude-container"><div class="horizon-inner" id="horizon"></div><div class="crosshair"></div></div>
            <div class="compass-box">
                <div class="compass-dial" id="compass"><span class="cardinal N">N</span><span class="cardinal E">E</span><span class="cardinal S">S</span><span class="cardinal W">W</span></div>
                <div id="head-num" style="position:absolute; font-size:1.2rem; font-weight:bold;">000&deg;</div>
            </div>
            <div class="controls">
                <div></div><div class="joy-btn" onmousedown="drive('F')" onmouseup="drive('S')">▲</div><div></div>
                <div class="joy-btn" onmousedown="drive('L')" onmouseup="drive('S')">◀</div>
                <div class="joy-btn" onclick="drive('S')">■</div>
                <div class="joy-btn" onmousedown="drive('R')" onmouseup="drive('S')">▶</div>
                <div></div><div class="joy-btn" onmousedown="drive('B')" onmouseup="drive('S')">▼</div><div></div>
            </div>
        </div>
        <div class="panel" style="grid-column: span 3; flex-direction:row; padding:10px; align-items:center;">
            <div style="flex:1"><small>LAT:</small> <span id="lat-val">0.000</span></div>
            <div style="flex:1"><small>LON:</small> <span id="lon-val">0.000</span></div>
            <div style="flex:1; text-align:right"><small>HEARTBEAT:</small> <span id="heart">OFFLINE</span></div>
        </div>
    </div>
    <script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script>
    <script>
        var map = L.map('map', { zoomControl: false, attributionControl: false }).setView([0, 0], 19);
        L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png').addTo(map);
        var marker = L.circleMarker([0, 0], { color: '#ff3131', radius: 8 }).addTo(map);

        function drive(c) { fetch('/control?cmd=' + c); }
        function toggleRTB() { fetch('/toggle_rtb'); }

        function updateHUD() {
            fetch('/data').then(r => r.json()).then(d => {
                document.getElementById('batt-pct').innerText = d.batt_pct + "%";
                document.getElementById('v-val').innerText = "(" + d.volts.toFixed(1) + " V)";
                document.getElementById('dist-val').innerText = d.dist_home + " M";
                document.getElementById('lat-val').innerText = d.cur_lat.toFixed(6);
                document.getElementById('lon-val').innerText = d.cur_lon.toFixed(6);
                document.getElementById('head-num').innerHTML = Math.round(d.heading) + "&deg;";
                document.getElementById('compass').style.transform = `rotate(${-d.heading}deg)`;
                document.getElementById('horizon').style.transform = `rotate(${d.roll}deg) translateY(${d.pitch * 2}px)`;
                document.getElementById('heart').innerText = "LIVE_" + Math.floor(Date.now()/1000)%10;
                if(d.sys_state == 2) { document.getElementById('main-ui').classList.add('sys-crit'); document.getElementById('rtb-btn').classList.add('rtb-active'); }
                else { document.getElementById('main-ui').classList.remove('sys-crit'); document.getElementById('rtb-btn').classList.remove('rtb-active'); }
                if(d.cur_lat != 0) { marker.setLatLng([d.cur_lat, d.cur_lon]); map.panTo([d.cur_lat, d.cur_lon]); }
            }).catch(e => { document.getElementById('heart').innerText = "LOST_LINK"; });
        }
        setInterval(updateHUD, 250);
        setInterval(() => { document.getElementById('clock').innerText = new Date().toLocaleTimeString(); }, 1000);
    </script>
</body>
</html>
)rawliteral";

// ==========================================
//   SERVER DATA HANDLER
// ==========================================

void handleDataUpdate() {
  int batt_pct = map(constrain(bus_voltage * 100, 680, 840), 680, 840, 0, 100);
  
  if (manual_rtb_override || batt_pct <= SAFETY_RESERVE_PERCENT) sys_state = 2; 
  else if (batt_pct < 30) sys_state = 1; 
  else sys_state = 0; 

  String json = "{";
  json += "\"batt_pct\":" + String(batt_pct) + ",";
  json += "\"volts\":" + String(bus_voltage, 2) + ",";
  json += "\"dist_home\":" + String((int)dist_to_home) + ",";
  json += "\"sys_state\":" + String(sys_state) + ",";
  json += "\"heading\":" + String(mag_heading, 1) + ",";
  json += "\"roll\":" + String(roll, 1) + ",";
  json += "\"pitch\":" + String(pitch, 1) + ",";
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

  server.on("/toggle_rtb", HTTP_GET, []() {
    manual_rtb_override = !manual_rtb_override;
    if(!manual_rtb_override) stopMotors();
    server.send(200, "text/plain", "OK");
  });

  server.on("/data", HTTP_GET, handleDataUpdate);
}

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
  
  // Set speed using LEDC/AnalogWrite for ESP32
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
