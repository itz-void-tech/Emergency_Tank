#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_HMC5883_U.h>
#include <Adafruit_INA219.h>
#include <Adafruit_Sensor.h>

// --- Objects ---
Adafruit_MPU6050 mpu;
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);
Adafruit_INA219 ina219;
WebServer server(80);

// --- Network Config ---
const char* ssid = "ARES-SENSOR-HUD"; 
const char* password = "rescueadmin"; 

// --- Calibration & Fusion ---
float gyroX_off = 0, gyroY_off = 0, gyroZ_off = 0;
float roll = 0, pitch = 0, yaw = 0;
float filter_alpha = 0.96; 
unsigned long last_micros;
float bus_voltage = 0, current_amps = 0;

// ==========================================
//   CALIBRATION & SENSORS
// ==========================================

void calibrateSensors() {
  Serial.println(">>> CALIBRATING GYRO - DO NOT MOVE <<<");
  float gx = 0, gy = 0, gz = 0;
  for (int i = 0; i < 200; i++) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    gx += g.gyro.x; gy += g.gyro.y; gz += g.gyro.z;
    delay(5);
  }
  gyroX_off = gx / 200.0;
  gyroY_off = gy / 200.0;
  gyroZ_off = gz / 200.0;
  Serial.println(">>> CALIBRATION COMPLETE <<<");
}

void updateSensors() {
  sensors_event_t a, g, temp, m;
  mpu.getEvent(&a, &g, &temp);
  mag.getEvent(&m);

  unsigned long now = micros();
  float dt = (now - last_micros) / 1000000.0;
  last_micros = now;

  // Accel calculations
  float roll_acc = atan2(a.acceleration.y, a.acceleration.z) * 180 / PI;
  float pitch_acc = atan2(-a.acceleration.x, sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z)) * 180 / PI;

  // Gyro integration with offsets
  roll = filter_alpha * (roll + (g.gyro.x - gyroX_off) * dt * 180 / PI) + (1 - filter_alpha) * roll_acc;
  pitch = filter_alpha * (pitch + (g.gyro.y - gyroY_off) * dt * 180 / PI) + (1 - filter_alpha) * pitch_acc;

  // Tilt-compensated Magnetometer
  float phi = roll * PI / 180;
  float theta = pitch * PI / 180;
  float Xh = m.magnetic.x * cos(theta) + m.magnetic.z * sin(theta);
  float Yh = m.magnetic.x * sin(phi) * sin(theta) + m.magnetic.y * cos(phi) - m.magnetic.z * sin(phi) * cos(theta);
  
  yaw = atan2(Yh, Xh) * 180 / PI;
  if (yaw < 0) yaw += 360;

  bus_voltage = ina219.getBusVoltage_V();
  current_amps = ina219.getCurrent_mA() / 1000.0;
}

// ==========================================
//   HUD HTML
// ==========================================

const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
    <title>ARES SENSOR HUB</title>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <link href="https://fonts.googleapis.com/css2?family=Share+Tech+Mono&display=swap" rel="stylesheet">
    <style>
        body { background: #020406; color: #00f2ff; font-family: 'Share Tech Mono', monospace; text-align: center; }
        .grid { display: grid; grid-template-columns: 1fr 1fr; gap: 20px; padding: 20px; }
        .card { background: rgba(0, 30, 50, 0.5); border: 1px solid #00f2ff; padding: 15px; border-radius: 5px; }
        .val { font-size: 2rem; color: #fff; text-shadow: 0 0 10px #00f2ff; }
        .gauge-box { height: 150px; display: flex; align-items: center; justify-content: center; position: relative; overflow: hidden; border: 1px solid #333; }
        #horizon { width: 300px; height: 300px; background: linear-gradient(to bottom, #1a4d70 50%, #4a2f1a 50%); position: absolute; transition: 0.1s; }
        #compass { width: 100px; height: 100px; border: 2px dashed #00f2ff; border-radius: 50%; transition: 0.2s; }
    </style>
</head>
<body>
    <h1>A.R.E.S. SENSOR CORE</h1>
    <div class="grid">
        <div class="card">
            <small>ATTITUDE (MPU6050)</small>
            <div class="gauge-box"><div id="horizon"></div></div>
            <div>PITCH: <span id="p-val">0</span>° | ROLL: <span id="r-val">0</span>°</div>
        </div>
        <div class="card">
            <small>HEADING (MAG)</small>
            <div class="gauge-box"><div id="compass">N</div></div>
            <div class="val" id="y-val">000°</div>
        </div>
        <div class="card">
            <small>VOLTAGE</small>
            <div class="val" id="v-val">0.0V</div>
            <div id="b-pct">--%</div>
        </div>
        <div class="card">
            <small>CURRENT DRAW</small>
            <div class="val" id="a-val">0.00A</div>
        </div>
    </div>
    <script>
        setInterval(() => {
            fetch('/data').then(r => r.json()).then(d => {
                document.getElementById('p-val').innerText = d.pitch;
                document.getElementById('r-val').innerText = d.roll;
                document.getElementById('y-val').innerText = d.yaw + "°";
                document.getElementById('v-val').innerText = d.volts.toFixed(2) + "V";
                document.getElementById('a-val').innerText = d.amps.toFixed(2) + "A";
                document.getElementById('b-pct').innerText = d.batt_pct + "%";
                
                // Update UI Visuals
                document.getElementById('horizon').style.transform = `rotate(${d.roll}deg) translateY(${d.pitch * 2}px)`;
                document.getElementById('compass').style.transform = `rotate(${-d.yaw}deg)`;
            });
        }, 200);
    </script>
</body>
</html>
)rawliteral";

// ==========================================
//   SERVER & MAIN
// ==========================================

void handleData() {
  int batt_pct = map(constrain(bus_voltage * 100, 680, 840), 680, 840, 0, 100);
  String json = "{";
  json += "\"roll\":" + String((int)roll) + ",";
  json += "\"pitch\":" + String((int)pitch) + ",";
  json += "\"yaw\":" + String((int)yaw) + ",";
  json += "\"volts\":" + String(bus_voltage, 2) + ",";
  json += "\"amps\":" + String(current_amps, 2) + ",";
  json += "\"batt_pct\":" + String(batt_pct);
  json += "}";
  server.send(200, "application/json", json);
}

void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);
  
  if(!mpu.begin()) Serial.println("MPU Fail");
  if(!mag.begin()) Serial.println("Mag Fail");
  if(!ina219.begin()) Serial.println("INA Fail");

  calibrateSensors();

  WiFi.softAP(ssid, password);
  server.on("/", []() { server.send(200, "text/html", index_html); });
  server.on("/data", handleData);
  server.begin();
  last_micros = micros();
}

void loop() {
  server.handleClient();
  updateSensors();
}
