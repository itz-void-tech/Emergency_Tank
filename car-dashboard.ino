#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_MPU6050.h>
#include <QMC5883LCompass.h>
#include <TinyGPSPlus.h>
#include <HardwareSerial.h>

const char* ssid = "Swarnendu";
const char* password = "12345678";

WebServer server(80);

// Sensors
Adafruit_MPU6050 mpu;
QMC5883LCompass mag;
TinyGPSPlus gps;
HardwareSerial GPS_Serial(2); // RX2: 16, TX2: 17

// --- Motor & Buzzer Pin Definitions ---
#define IN1 13
#define IN2 12
#define ENA 14

#define IN3 27
#define IN4 26
#define ENB 25

#define BUZZER_PIN 33

int motorSpeed = 200; // 0 to 255

// --- Sensor Variables ---
float pitch = 0, roll = 0, heading = 0, temperature = 0, yaw = 0;
float gyroX_off = 0, gyroY_off = 0, gyroZ_off = 0;
bool is_calibrating = false;
unsigned long last_micros = 0;

// --- Siren Variables ---
bool sirenActive = false;
unsigned long sirenTimer = 0;
bool sirenHighFreq = true;

// Motor Control Functions (Inverted for specific wiring setup)
void moveForward() { digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH); digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH); }
void moveBackward() { digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW); digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW); }
void turnLeft() { digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW); digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH); }
void turnRight() { digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH); digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW); }
void stopMotors() { digitalWrite(IN1, LOW); digitalWrite(IN2, LOW); digitalWrite(IN3, LOW); digitalWrite(IN4, LOW); }

TaskHandle_t sirenTask;
void sirenTaskCode(void * parameter) {
  for(;;) {
    if (sirenActive) {
      // Emergency Yelp Siren (Continuous sweep up and down)
      // Sweep UP to higher pitch
      for (int freq = 700; freq <= 1500; freq += 40) {
        if (!sirenActive) break;
        tone(BUZZER_PIN, freq);
        vTaskDelay(10 / portTICK_PERIOD_MS);
      }
      // Sweep DOWN
      for (int freq = 1500; freq >= 700; freq -= 40) {
        if (!sirenActive) break;
        tone(BUZZER_PIN, freq);
        vTaskDelay(10 / portTICK_PERIOD_MS);
      }
    } else {
      noTone(BUZZER_PIN);
      vTaskDelay(50 / portTICK_PERIOD_MS);
    }
  }
}

void calibrateSensors() {
  is_calibrating = true;
  float gx = 0, gy = 0, gz = 0;
  for (int i = 0; i < 150; i++) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    gx += g.gyro.x; gy += g.gyro.y; gz += g.gyro.z;
    server.handleClient(); 
    delay(15); 
  }
  gyroX_off = gx / 150.0;
  gyroY_off = gy / 150.0;
  gyroZ_off = gz / 150.0;
  is_calibrating = false;
}

void updateSensors() {
  if (is_calibrating) return;
  
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  temperature = temp.temperature;
  
  mag.read();
  float mx = mag.getX();
  float my = mag.getY();
  float mz = mag.getZ();

  unsigned long now = micros();
  float dt = (now - last_micros) / 1000000.0;
  if (dt <= 0 || dt > 0.5) dt = 0.01;
  last_micros = now;

  float roll_acc = atan2(a.acceleration.y, a.acceleration.z) * 180 / PI;
  float pitch_acc = atan2(-a.acceleration.x, sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z)) * 180 / PI;

  roll = 0.96 * (roll + (g.gyro.x - gyroX_off) * dt * 180 / PI) + 0.04 * roll_acc;
  pitch = 0.96 * (pitch + (g.gyro.y - gyroY_off) * dt * 180 / PI) + 0.04 * pitch_acc;

  float phi = roll * PI / 180;
  float theta = pitch * PI / 180;
  
  // Tilt Compensation Math
  float Xh = mx * cos(theta) + mz * sin(theta);
  float Yh = mx * sin(phi) * sin(theta) + my * cos(phi) - mz * sin(phi) * cos(theta);
  
  yaw = atan2(Yh, Xh) * 180 / PI;
  if (yaw < 0) yaw += 360;
  heading = yaw; // Sync for dashboard UI
}

// Embedded HTML UI
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0, maximum-scale=1.0, user-scalable=no">
    <title>ESP32 Rover Dashboard</title>
    <link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css" />
    <script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script>
    <style>
        @import url('https://fonts.googleapis.com/css2?family=Inter:wght@300;500;700&display=swap');
        body { margin: 0; font-family: 'Inter', sans-serif; background: linear-gradient(135deg, #0f172a, #1e293b); color: #f8fafc; min-height: 100vh; overflow-x: hidden; touch-action: manipulation; }
        .header { padding: 20px; text-align: center; background: rgba(255,255,255,0.05); backdrop-filter: blur(10px); border-bottom: 1px solid rgba(255,255,255,0.1); }
        h1 { margin: 0; font-weight: 700; font-size: 28px; letter-spacing: 1px; background: -webkit-linear-gradient(#38bdf8, #818cf8); -webkit-background-clip: text; -webkit-text-fill-color: transparent; }
        .container { display: grid; grid-template-columns: repeat(auto-fit, minmax(320px, 1fr)); gap: 25px; padding: 25px; max-width: 1400px; margin: 0 auto; }
        .card { background: rgba(30, 41, 59, 0.6); border: 1px solid rgba(255,255,255,0.05); border-radius: 20px; padding: 25px; box-shadow: 0 10px 30px rgba(0,0,0,0.3); backdrop-filter: blur(10px); transition: transform 0.3s ease; }
        .card h2 { margin-top: 0; font-size: 16px; color: #94a3b8; text-transform: uppercase; letter-spacing: 2px; border-bottom: 1px solid rgba(255,255,255,0.1); padding-bottom: 10px; margin-bottom: 20px; }
        #map { height: 300px; border-radius: 12px; border: 1px solid rgba(255,255,255,0.1); z-index: 1; }
        
        .data-grid { display: grid; grid-template-columns: 1fr 1fr; gap: 15px; }
        .data-item { background: rgba(0,0,0,0.3); padding: 15px; border-radius: 12px; text-align: center; border: 1px solid rgba(255,255,255,0.02); }
        .data-label { font-size: 11px; color: #94a3b8; letter-spacing: 1px; text-transform: uppercase; }
        .data-value { font-size: 24px; font-weight: 700; color: #e2e8f0; margin-top: 8px; font-variant-numeric: tabular-nums; }
        
        .compass-container { position: relative; width: 220px; height: 220px; margin: 30px auto; border: 6px solid rgba(56, 189, 248, 0.3); border-radius: 50%; display: flex; justify-content: center; align-items: center; box-shadow: inset 0 0 20px rgba(0,0,0,0.5), 0 0 20px rgba(56, 189, 248, 0.2); background: radial-gradient(circle, rgba(30,41,59,1) 0%, rgba(15,23,42,1) 100%); }
        .compass-needle { position: absolute; width: 8px; height: 190px; background: linear-gradient(to top, #e2e8f0 50%, #ef4444 50%); border-radius: 4px; transition: transform 0.2s cubic-bezier(0.4, 0.0, 0.2, 1); box-shadow: 0 0 10px rgba(0,0,0,0.5); z-index: 2; }
        .compass-center { width: 16px; height: 16px; background: #38bdf8; border-radius: 50%; z-index: 3; box-shadow: 0 0 10px rgba(0,0,0,0.5); }
        .compass-markers { position: absolute; width: 100%; height: 100%; font-weight: 700; font-size: 18px; }
        .marker { position: absolute; color: #64748b; }
        .marker-n { top: 12px; left: 50%; transform: translateX(-50%); color: #ef4444; }
        .marker-s { bottom: 12px; left: 50%; transform: translateX(-50%); }
        .marker-e { right: 12px; top: 50%; transform: translateY(-50%); }
        .marker-w { left: 12px; top: 50%; transform: translateY(-50%); }
        .heading-display { text-align: center; font-size: 32px; font-weight: 700; color: #38bdf8; margin-top: 15px; font-variant-numeric: tabular-nums;}

        .car-container { perspective: 1000px; width: 160px; height: 80px; margin: 60px auto; }
        .car-3d { width: 100%; height: 100%; position: relative; transform-style: preserve-3d; transition: transform 0.1s linear; }
        .car-panel { position: absolute; display: flex; justify-content: center; align-items: center; font-weight: 900; color: white; border: 2px solid #38bdf8; background: rgba(56, 189, 248, 0.2); backdrop-filter: blur(2px); }
        .car-left   { width: 160px; height: 40px; transform: rotateY(0deg) translateZ(40px); }
        .car-right  { width: 160px; height: 40px; transform: rotateY(180deg) translateZ(40px); }
        .car-front  { width: 80px; height: 40px; transform: rotateY(90deg) translateZ(120px) translateX(-40px); border-color: #fcd34d; background: rgba(252, 211, 77, 0.2); color: #fcd34d; }
        .car-back   { width: 80px; height: 40px; transform: rotateY(-90deg) translateZ(40px) translateX(40px); border-color: #ef4444; background: rgba(239, 68, 68, 0.2); color: #ef4444; }
        .car-top    { width: 160px; height: 80px; transform: rotateX(90deg) translateZ(20px); border-radius: 10px; }
        .car-bottom { width: 160px; height: 80px; transform: rotateX(-90deg) translateZ(20px); background: rgba(15, 23, 42, 0.9); border: none; box-shadow: 0 10px 30px rgba(56,189,248,0.5); }

        /* Controller Styles */
        .d-pad { display: grid; grid-template-columns: repeat(3, 80px); grid-template-rows: repeat(3, 80px); gap: 10px; justify-content: center; margin: 20px 0; }
        .d-pad button { background: rgba(56, 189, 248, 0.2); border: 2px solid #38bdf8; border-radius: 12px; color: #fff; font-size: 32px; cursor: pointer; user-select: none; transition: transform 0.1s, background 0.1s; -webkit-tap-highlight-color: transparent; }
        .d-pad button:active { transform: scale(0.9); background: rgba(56, 189, 248, 0.5); }
    </style>
</head>
<body>
    <div class="header">
        <h1>ESP32 Rover Dashboard</h1>
    </div>
    <div class="container">
        <!-- Rover Controls -->
        <div class="card">
            <h2>Drive Controls</h2>
            <div class="d-pad">
                <div></div>
                <button onmousedown="cmd('F')" onmouseup="cmd('S')" ontouchstart="cmd('F'); event.preventDefault();" ontouchend="cmd('S'); event.preventDefault();">▲</button>
                <div></div>
                <button onmousedown="cmd('L')" onmouseup="cmd('S')" ontouchstart="cmd('L'); event.preventDefault();" ontouchend="cmd('S'); event.preventDefault();">◀</button>
                <button onmousedown="cmd('S')" ontouchstart="cmd('S'); event.preventDefault();" style="background: rgba(239, 68, 68, 0.3); border-color: #ef4444;">■</button>
                <button onmousedown="cmd('R')" onmouseup="cmd('S')" ontouchstart="cmd('R'); event.preventDefault();" ontouchend="cmd('S'); event.preventDefault();">▶</button>
                <div></div>
                <button onmousedown="cmd('B')" onmouseup="cmd('S')" ontouchstart="cmd('B'); event.preventDefault();" ontouchend="cmd('S'); event.preventDefault();">▼</button>
                <div></div>
            </div>
            
            <div style="margin-top: 15px;">
                <label style="font-size: 14px; font-weight: bold; color: #94a3b8;">MOTOR SPEED: <span id="speed-val">200</span></label>
                <input type="range" id="speedSlider" min="0" max="255" value="200" oninput="document.getElementById('speed-val').innerText = this.value" onchange="updateSpeed(this.value)" style="width: 100%; margin-top: 10px;">
            </div>

            <button id="siren-btn" onclick="toggleSiren()" style="margin-top: 15px; width: 100%; background: #fbbf24; border: none; color: #000; padding: 15px; border-radius: 8px; font-weight: bold; font-size: 16px; cursor: pointer; transition: background 0.2s;">🚨 TOGGLE SIREN</button>
        </div>

        <!-- Compass Card -->
        <div class="card">
            <h2>QMC5883L Compass</h2>
            <div class="compass-container">
                <div class="compass-markers">
                    <div class="marker marker-n">N</div>
                    <div class="marker marker-s">S</div>
                    <div class="marker marker-e">E</div>
                    <div class="marker marker-w">W</div>
                </div>
                <div class="compass-needle" id="needle"></div>
                <div class="compass-center"></div>
            </div>
            <div class="heading-display" id="heading-val">0.0&deg;</div>
            <button onclick="fetch('/start_cal')" style="margin-top: 15px; width: 100%; background: #38bdf8; border: none; color: #0f172a; padding: 10px; border-radius: 8px; font-weight: bold; cursor: pointer; transition: transform 0.1s; box-shadow: 0 4px 10px rgba(56,189,248,0.3);">CALIBRATE GYRO</button>
        </div>

        <!-- Map Card -->
        <div class="card">
            <h2 id="gps-status-header">Live Location (GNSS)</h2>
            <div id="map"></div>
        </div>

        <!-- Orientation Card -->
        <div class="card">
            <h2>Rover Orientation</h2>
            <div class="car-container">
                <div class="car-3d" id="cube">
                    <div class="car-panel car-top">ROOF</div>
                    <div class="car-panel car-bottom"></div>
                    <div class="car-panel car-front">FRONT</div>
                    <div class="car-panel car-back">REAR</div>
                    <div class="car-panel car-left">LEFT</div>
                    <div class="car-panel car-right">RIGHT</div>
                </div>
            </div>
            <div class="data-grid" style="margin-top: 30px;">
                <div class="data-item"><div class="data-label">PITCH</div><div class="data-value" id="pitch-val">0.0&deg;</div></div>
                <div class="data-item"><div class="data-label">ROLL</div><div class="data-value" id="roll-val">0.0&deg;</div></div>
            </div>
        </div>

        <!-- Telemetry Card -->
        <div class="card">
            <h2>System Telemetry</h2>
            <div class="data-grid">
                <div class="data-item"><div class="data-label">LATITUDE</div><div class="data-value" id="lat-val">0.000000</div></div>
                <div class="data-item"><div class="data-label">LONGITUDE</div><div class="data-value" id="lng-val">0.000000</div></div>
                <div class="data-item"><div class="data-label">SATELLITES</div><div class="data-value" id="sat-val">0</div></div>
                <div class="data-item"><div class="data-label">TEMPERATURE</div><div class="data-value" id="temp-val">0.0 &deg;C</div></div>
            </div>
        </div>
    </div>

    <script>
        // Rover Controls
        function cmd(direction) {
            fetch(`/cmd?dir=${direction}`).catch(console.error);
        }

        function updateSpeed(val) {
            fetch(`/speed?val=${val}`).catch(console.error);
        }

        let sirenState = false;
        function toggleSiren() {
            sirenState = !sirenState;
            fetch(`/siren?state=${sirenState ? 'on' : 'off'}`).catch(console.error);
            document.getElementById('siren-btn').style.background = sirenState ? '#ef4444' : '#fbbf24';
            document.getElementById('siren-btn').style.color = sirenState ? '#fff' : '#000';
        }

        // Map Initialization
        const map = L.map('map').setView([0, 0], 2);
        L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
            attribution: '&copy; OpenStreetMap',
            maxZoom: 19
        }).addTo(map);
        
        const markerIcon = L.divIcon({
            className: 'custom-marker',
            html: `<div style="background:#ef4444;width:16px;height:16px;border-radius:50%;border:3px solid white;box-shadow:0 0 10px rgba(0,0,0,0.5);"></div>`,
            iconSize: [22, 22],
            iconAnchor: [11, 11]
        });
        const marker = L.marker([0, 0], {icon: markerIcon}).addTo(map);

        let mapInitialized = false;

        async function fetchData() {
            try {
                const response = await fetch('/data');
                const data = await response.json();
                
                document.getElementById('lat-val').innerText = data.lat.toFixed(6);
                document.getElementById('lng-val').innerText = data.lng.toFixed(6);
                document.getElementById('sat-val').innerText = data.sat;
                document.getElementById('temp-val').innerText = data.temp.toFixed(1) + ' °C';

                const gpsHeader = document.getElementById('gps-status-header');
                if (data.valid) {
                    gpsHeader.innerText = 'LIVE LOCATION (GNSS)';
                    gpsHeader.style.color = '#38bdf8';
                } else if (data.lat !== 0) {
                    gpsHeader.innerText = 'LAST KNOWN LOCATION (GNSS)';
                    gpsHeader.style.color = '#fbbf24';
                } else {
                    gpsHeader.innerText = 'WAITING FOR FIX (GNSS)';
                    gpsHeader.style.color = '#94a3b8';
                }
                
                document.getElementById('heading-val').innerHTML = data.heading.toFixed(1) + '&deg;';
                document.getElementById('pitch-val').innerHTML = data.pitch.toFixed(1) + '&deg;';
                document.getElementById('roll-val').innerHTML = data.roll.toFixed(1) + '&deg;';

                if (data.lat !== 0 || data.lng !== 0) {
                    const latlng = [data.lat, data.lng];
                    marker.setLatLng(latlng);
                    if (!mapInitialized && data.sat > 0) {
                        map.setView(latlng, 17);
                        mapInitialized = true;
                    }
                }

                document.getElementById('needle').style.transform = `rotate(${data.heading}deg)`;
                document.getElementById('cube').style.transform = `rotateX(${-data.pitch}deg) rotateY(${data.heading}deg) rotateZ(${-data.roll}deg)`;

            } catch (err) {
                console.error("Error fetching telemetry:", err);
            }
        }

        setInterval(fetchData, 200);
    </script>
</body>
</html>
)rawliteral";

void setup() {
  Serial.begin(115200);
  
  // Setup Motor Pins
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  stopMotors();

  // AnalogWrite provides PWM on ESP32 in recent Arduino cores
  analogWrite(ENA, motorSpeed);
  analogWrite(ENB, motorSpeed);

  // Setup Buzzer
  pinMode(BUZZER_PIN, OUTPUT);
  noTone(BUZZER_PIN);

  // Init Serial & I2C
  GPS_Serial.begin(9600, SERIAL_8N1, 16, 17);
  Wire.begin(); 
  
  Serial.println("Initializing MPU6050...");
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip.");
  } else {
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    mpu.setI2CBypass(true); 
    Serial.println("MPU6050 ready.");
  }

  Serial.println("Initializing QMC5883L...");
  mag.init();
  Serial.println("QMC5883L ready.");

  // Connect to Wi-Fi
  Serial.print("Connecting to Wi-Fi: ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWi-Fi connected!");
  Serial.print("Access Dashboard at: http://");
  Serial.println(WiFi.localIP());

  calibrateSensors();
  last_micros = micros();

  xTaskCreate(
    sirenTaskCode,   // Task function
    "SirenTask",     // Name of task
    2048,            // Stack size (words)
    NULL,            // Parameter
    1,               // Priority
    &sirenTask       // Task handle
  );

  // Define HTTP routes
  server.on("/", HTTP_GET, []() {
    server.send(200, "text/html", index_html);
  });

  server.on("/start_cal", HTTP_GET, []() {
    server.send(200, "text/plain", "OK");
    calibrateSensors();
  });

  // Motor Commands
  server.on("/cmd", HTTP_GET, []() {
    if (server.hasArg("dir")) {
      String dir = server.arg("dir");
      if (dir == "F") moveForward();
      else if (dir == "B") moveBackward();
      else if (dir == "L") turnLeft();
      else if (dir == "R") turnRight();
      else if (dir == "S") stopMotors();
    }
    server.send(200, "text/plain", "OK");
  });

  // Speed Command
  server.on("/speed", HTTP_GET, []() {
    if (server.hasArg("val")) {
      motorSpeed = server.arg("val").toInt();
      analogWrite(ENA, motorSpeed);
      analogWrite(ENB, motorSpeed);
    }
    server.send(200, "text/plain", "OK");
  });

  // Siren Command
  server.on("/siren", HTTP_GET, []() {
    if (server.hasArg("state")) {
      String state = server.arg("state");
      sirenActive = (state == "on");
    }
    server.send(200, "text/plain", "OK");
  });

  server.on("/data", HTTP_GET, []() {
    String json = "{";
    json += "\"lat\":" + String(gps.location.lat(), 6) + ",";
    json += "\"lng\":" + String(gps.location.lng(), 6) + ",";
    json += "\"sat\":" + String(gps.satellites.value()) + ",";
    json += "\"valid\":" + String(gps.location.isValid() && gps.location.age() < 5000 ? "true" : "false") + ",";
    json += "\"pitch\":" + String(pitch, 2) + ",";
    json += "\"roll\":" + String(roll, 2) + ",";
    json += "\"yaw\":" + String(yaw, 2) + ",";
    json += "\"heading\":" + String(heading, 2) + ",";
    json += "\"temp\":" + String(temperature, 2);
    json += "}";
    server.send(200, "application/json", json);
  });

  server.begin();
}

unsigned long lastSensorPoll = 0;
void loop() {
  server.handleClient();
  
  // Read GPS Data
  while (GPS_Serial.available() > 0) {
    gps.encode(GPS_Serial.read());
  }

  // Update Sensors and Fusion Logic at 50Hz to prevent web server lagging
  if (millis() - lastSensorPoll >= 20) {
    lastSensorPoll = millis();
    updateSensors();
  }
}
