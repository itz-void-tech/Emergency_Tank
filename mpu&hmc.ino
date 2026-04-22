#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_HMC5883_U.h>
#include <Adafruit_Sensor.h>

// --- Hardware Objects ---
Adafruit_MPU6050 mpu;
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);
WebServer server(80);

// --- Network Settings ---
const char* ssid = "ESP32-SPATIAL-HUD"; 
const char* password = "password123"; 

// --- Calibration & Fusion Variables ---
float gyroX_off = 0, gyroY_off = 0, gyroZ_off = 0;
float roll = 0, pitch = 0, yaw = 0;
bool is_calibrating = false;
int calibration_progress = 0;
unsigned long last_micros = 0;

// ==========================================
//   CALIBRATION & SENSOR LOGIC
// ==========================================

void calibrateSensors() {
  is_calibrating = true;
  float gx = 0, gy = 0, gz = 0;
  const int samples = 150;

  for (int i = 0; i < samples; i++) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    gx += g.gyro.x; 
    gy += g.gyro.y; 
    gz += g.gyro.z;
    calibration_progress = map(i, 0, samples - 1, 0, 100);
    server.handleClient(); 
    delay(20); 
  }

  gyroX_off = gx / samples;
  gyroY_off = gy / samples;
  gyroZ_off = gz / samples;
  is_calibrating = false;
}

void updateSensors() {
  if (is_calibrating) return;

  sensors_event_t a, g, temp, m;
  mpu.getEvent(&a, &g, &temp);
  mag.getEvent(&m);

  unsigned long now = micros();
  float dt = (now - last_micros) / 1000000.0;
  if (dt <= 0 || dt > 0.5) dt = 0.01;
  last_micros = now;

  // 1. Roll & Pitch (Complementary Filter)
  float roll_acc = atan2(a.acceleration.y, a.acceleration.z) * 180 / PI;
  float pitch_acc = atan2(-a.acceleration.x, sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z)) * 180 / PI;

  roll = 0.96 * (roll + (g.gyro.x - gyroX_off) * dt * 180 / PI) + 0.04 * roll_acc;
  pitch = 0.96 * (pitch + (g.gyro.y - gyroY_off) * dt * 180 / PI) + 0.04 * pitch_acc;

  // 2. Tilt-Compensated Heading
  float phi = roll * PI / 180;
  float theta = pitch * PI / 180;
  float Xh = m.magnetic.x * cos(theta) + m.magnetic.z * sin(theta);
  float Yh = m.magnetic.x * sin(phi) * sin(theta) + m.magnetic.y * cos(phi) - m.magnetic.z * sin(phi) * cos(theta);
  
  yaw = atan2(Yh, Xh) * 180 / PI;
  if (yaw < 0) yaw += 360;
}

// ==========================================
//   WEB INTERFACE (HTML/CSS/JS)
// ==========================================

const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html><html><head>
<title>SPATIAL HUD</title>
<meta name="viewport" content="width=device-width, initial-scale=1">
<style>
    :root { --neon: #00f2ff; --bg: #05080a; }
    body { background: var(--bg); color: var(--neon); font-family: monospace; display: flex; flex-direction: column; align-items: center; text-transform: uppercase; margin: 0; padding: 20px; }
    .hud-box { background: #0d141a; border: 1px solid #1a2c38; border-top: 4px solid var(--neon); padding: 30px; border-radius: 5px; box-shadow: 0 0 20px rgba(0,242,255,0.2); text-align: center; position: relative; }
    
    /* Calibration Overlay */
    #overlay { display: none; position: absolute; top: 0; left: 0; width: 100%; height: 100%; background: rgba(0,0,0,0.9); z-index: 100; flex-direction: column; align-items: center; justify-content: center; }
    .progress-box { width: 80%; height: 10px; background: #111; border: 1px solid var(--neon); margin: 15px 0; }
    #bar { width: 0%; height: 100%; background: var(--neon); }

    /* Spatial Axis Oval */
    .axis-container { width: 220px; height: 110px; border: 2px solid var(--neon); border-radius: 50%; position: relative; overflow: hidden; background: #000; margin: 0 auto 30px auto; }
    #horizon-line { width: 400px; height: 2px; background: var(--neon); position: absolute; top: 50%; left: -90px; transition: 0.1s linear; }
    .fixed-ref { width: 60px; height: 3px; background: #ff3131; position: absolute; top: 50%; left: 50%; transform: translate(-50%, -50%); z-index: 5; }

    /* Compass Ring */
    .compass-wrap { position: relative; width: 180px; height: 180px; margin: auto; }
    #markers { transition: 0.1s linear; transform-origin: center; }
    .heading-display { position: absolute; top: 50%; left: 50%; transform: translate(-50%, -50%); font-size: 1.8em; font-weight: bold; text-shadow: 0 0 10px var(--neon); }
    
    button { background: transparent; border: 1px solid var(--neon); color: var(--neon); padding: 10px; cursor: pointer; margin-top: 20px; font-family: inherit; width: 100%; }
</style>
</head><body>

<div class="hud-box">
    <div id="overlay">
        <span>CALIBRATING...</span>
        <div class="progress-box"><div id="bar"></div></div>
    </div>

    <h2>SPATIAL HUD</h2>

    <div class="axis-container">
        <div class="fixed-ref"></div>
        <div id="horizon-line"></div>
    </div>

    <div class="compass-wrap">
        <div class="heading-display" id="y-val">000°</div>
        <svg viewBox="0 0 160 160">
            <circle cx="80" cy="80" r="78" fill="none" stroke="#1a2c38" stroke-width="1" stroke-dasharray="4,4" />
            <g id="markers">
                <text x="80" y="25" fill="#ff3131" text-anchor="middle" font-weight="bold">N</text>
                <text x="140" y="85" fill="white" text-anchor="middle">E</text>
                <text x="80" y="145" fill="white" text-anchor="middle">S</text>
                <text x="20" y="85" fill="white" text-anchor="middle">W</text>
            </g>
        </svg>
    </div>

    <button onclick="startCal()">RECALIBRATE IMU</button>
</div>

<script>
    function startCal() { fetch('/start_cal'); }

    setInterval(() => {
        fetch('/data').then(r => r.json()).then(d => {
            if(d.is_cal) {
                document.getElementById('overlay').style.display = 'flex';
                document.getElementById('bar').style.width = d.prog + "%";
            } else {
                document.getElementById('overlay').style.display = 'none';
                // Update Compass
                document.getElementById('markers').style.transform = `rotate(${-d.yaw}deg)`;
                document.getElementById('y-val').innerText = Math.round(d.yaw).toString().padStart(3, '0') + '°';
                // Update Horizon (Roll and Pitch)
                document.getElementById('horizon-line').style.transform = `rotate(${d.roll}deg) translateY(${d.pitch}px)`;
            }
        });
    }, 100);
</script>
</body></html>
)rawliteral";

// ==========================================
//   SERVER HANDLERS & SETUP
// ==========================================

void handleData() {
  String json = "{";
  json += "\"is_cal\":" + String(is_calibrating ? "true" : "false") + ",";
  json += "\"prog\":" + String(calibration_progress) + ",";
  json += "\"roll\":" + String((int)roll) + ",";
  json += "\"pitch\":" + String((int)pitch) + ",";
  json += "\"yaw\":" + String((int)yaw);
  json += "}";
  server.send(200, "application/json", json);
}

void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22); // Standard SDA/SCL for ESP32
  
  if(!mpu.begin()) Serial.println("MPU FAIL");
  if(!mag.begin()) Serial.println("HMC FAIL");

  calibrateSensors();
  WiFi.softAP(ssid, password);
  
  server.on("/", []() { server.send(200, "text/html", index_html); });
  server.on("/data", handleData);
  server.on("/start_cal", []() {
    server.send(200, "text/plain", "OK");
    calibrateSensors();
  });
  
  server.begin();
  last_micros = micros();
}

void loop() {
  server.handleClient();
  updateSensors();
}
