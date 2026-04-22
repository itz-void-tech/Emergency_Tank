#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_HMC5883_U.h>
#include <Adafruit_INA219.h>
#include <Adafruit_Sensor.h>

// --- Hardware Objects ---
Adafruit_MPU6050 mpu;
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);
Adafruit_INA219 ina219;
WebServer server(80);

// --- Network Settings ---
const char* ssid = "ARES-SENSOR-COMMAND"; 
const char* password = "rescueadmin"; 

// --- Calibration & Fusion Variables ---
float gyroX_off = 0, gyroY_off = 0, gyroZ_off = 0;
float roll = 0, pitch = 0, yaw = 0;
float bus_voltage = 0, current_amps = 0;
bool is_calibrating = false;
int calibration_progress = 0;
unsigned long last_micros = 0;

// ==========================================
//   CALIBRATION LOGIC
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
    
    // Update web client while in loop to prevent timeout
    server.handleClient(); 
    delay(20); 
  }

  gyroX_off = gx / samples;
  gyroY_off = gy / samples;
  gyroZ_off = gz / samples;
  
  is_calibrating = false;
  calibration_progress = 0;
  Serial.println("IMU Zeroed Successfully.");
}

void updateSensors() {
  if (is_calibrating) return;

  sensors_event_t a, g, temp, m;
  mpu.getEvent(&a, &g, &temp);
  mag.getEvent(&m);

  unsigned long now = micros();
  float dt = (now - last_micros) / 1000000.0;
  if (dt <= 0 || dt > 0.5) dt = 0.01; // Safety cap
  last_micros = now;

  // 1. Roll & Pitch (Accelerometer + Gyro Complementary Filter)
  float roll_acc = atan2(a.acceleration.y, a.acceleration.z) * 180 / PI;
  float pitch_acc = atan2(-a.acceleration.x, sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z)) * 180 / PI;

  roll = 0.96 * (roll + (g.gyro.x - gyroX_off) * dt * 180 / PI) + 0.04 * roll_acc;
  pitch = 0.96 * (pitch + (g.gyro.y - gyroY_off) * dt * 180 / PI) + 0.04 * pitch_acc;

  // 2. Tilt-Compensated Heading (Yaw)
  float phi = roll * PI / 180;
  float theta = pitch * PI / 180;
  float Xh = m.magnetic.x * cos(theta) + m.magnetic.z * sin(theta);
  float Yh = m.magnetic.x * sin(phi) * sin(theta) + m.magnetic.y * cos(phi) - m.magnetic.z * sin(phi) * cos(theta);
  
  yaw = atan2(Yh, Xh) * 180 / PI;
  if (yaw < 0) yaw += 360;

  // 3. Power Metrics
  bus_voltage = ina219.getBusVoltage_V();
  current_amps = ina219.getCurrent_mA() / 1000.0;
}

// ==========================================
//   WEB INTERFACE (HTML/CSS/JS)
// ==========================================

const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
    <title>ARES SENSOR SUITE</title>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <link href="https://fonts.googleapis.com/css2?family=Share+Tech+Mono&display=swap" rel="stylesheet">
    <style>
        :root { --neon: #00f2ff; --bg: #020406; --warn: #ff3131; }
        body { background: var(--bg); color: var(--neon); font-family: 'Share Tech Mono', monospace; margin: 0; padding: 15px; overflow-x: hidden; }
        .hud-container { max-width: 500px; margin: auto; }
        .card { background: rgba(0, 35, 55, 0.4); border: 1px solid var(--neon); padding: 20px; margin-bottom: 15px; border-radius: 8px; position: relative; box-shadow: 0 0 15px rgba(0, 242, 255, 0.1); }
        .val-big { font-size: 2.2rem; color: #fff; text-shadow: 0 0 10px var(--neon); }
        
        /* Calibration Overlay */
        #overlay { display: none; position: absolute; top: 0; left: 0; width: 100%; height: 100%; background: rgba(0,0,0,0.95); z-index: 100; flex-direction: column; align-items: center; justify-content: center; border-radius: 8px; }
        .progress-box { width: 80%; height: 12px; background: #111; border: 1px solid var(--neon); margin: 15px 0; border-radius: 10px; overflow: hidden; }
        #bar { width: 0%; height: 100%; background: var(--neon); box-shadow: 0 0 15px var(--neon); transition: width 0.1s; }
        
        button { background: transparent; border: 1px solid var(--neon); color: var(--neon); padding: 12px; width: 100%; font-family: inherit; font-size: 1rem; cursor: pointer; margin-top: 10px; border-radius: 4px; }
        button:active { background: var(--neon); color: #000; }
        
        .status-tag { font-size: 0.7rem; letter-spacing: 2px; color: var(--neon); opacity: 0.7; margin-bottom: 5px; display: block; }
        .grid-2 { display: grid; grid-template-columns: 1fr 1fr; gap: 10px; }
        .rotate-hint { color: var(--warn); font-size: 0.8rem; margin-top: 10px; display: none; text-align: center; }
    </style>
</head>
<body>
    <div class="hud-container">
        <h2 style="text-align:center; letter-spacing:4px;">A.R.E.S. SENSOR CORE</h2>
        
        <div class="card">
            <div id="overlay">
                <span style="color:var(--warn)">INITIALIZING CALIBRATION</span>
                <div class="progress-box"><div id="bar"></div></div>
                <small id="step-text">STEP: ZEROING GYRO...</small>
            </div>
            <span class="status-tag">ORIENTATION (IMU)</span>
            <div class="grid-2">
                <div><small>PITCH</small><br><span id="p-val" class="val-big">0</span>°</div>
                <div><small>ROLL</small><br><span id="r-val" class="val-big">0</span>°</div>
            </div>
            <button onclick="startCal()">INITIATE CALIBRATION SEQUENCE</button>
            <div id="rot-msg" class="rotate-hint">CAL COMPLETE: ROTATE UNIT 360&deg; NOW</div>
        </div>

        <div class="card">
            <span class="status-tag">MAGNETIC HEADING</span>
            <small>YAW ANGLE</small><br>
            <span id="y-val" class="val-big">000</span><span class="val-big" style="font-size:1.2rem">&deg; N</span>
        </div>

        <div class="card">
            <span class="status-tag">POWER DIAGNOSTICS</span>
            <div class="grid-2">
                <div><small>BUS VOLTAGE</small><br><span id="v-val" class="val-big">0.00</span>V</div>
                <div><small>CURRENT</small><br><span id="a-val" class="val-big">0.00</span>A</div>
            </div>
            <div id="b-pct" style="margin-top:10px; text-align:right; font-size:0.9rem;">BATT: --%</div>
        </div>
    </div>

    <script>
        function startCal() {
            document.getElementById('overlay').style.display = 'flex';
            document.getElementById('rot-msg').style.display = 'none';
            fetch('/start_cal');
        }

        setInterval(() => {
            fetch('/data').then(r => r.json()).then(d => {
                if(d.is_cal) {
                    document.getElementById('overlay').style.display = 'flex';
                    document.getElementById('bar').style.width = d.prog + "%";
                } else {
                    if(document.getElementById('overlay').style.display == 'flex') {
                        document.getElementById('overlay').style.display = 'none';
                        document.getElementById('rot-msg').style.display = 'block';
                    }
                    document.getElementById('p-val').innerText = d.pitch;
                    document.getElementById('r-val').innerText = d.roll;
                    document.getElementById('y-val').innerText = d.yaw.toString().padStart(3, '0');
                    document.getElementById('v-val').innerText = d.volts.toFixed(2);
                    document.getElementById('a-val').innerText = d.amps.toFixed(2);
                    document.getElementById('b-pct').innerText = "BATT CAPACITY: " + d.batt_pct + "%";
                }
            });
        }, 150);
    </script>
</body>
</html>
)rawliteral";

// ==========================================
//   SERVER HANDLERS & SETUP
// ==========================================

void handleData() {
  int batt_pct = map(constrain(bus_voltage * 100, 680, 840), 680, 840, 0, 100);
  String json = "{";
  json += "\"is_cal\":" + String(is_calibrating ? "true" : "false") + ",";
  json += "\"prog\":" + String(calibration_progress) + ",";
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

  // Initial fast calibration
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
