#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_HMC5883_U.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_INA219.h>
#include <TinyGPS++.h>
#include <Adafruit_NeoPixel.h>

// --- ESP32-S3 Hardware Pins ---
// I2C
#define I2C_SDA 8
#define I2C_SCL 9

// GPS
#define RXD2 18
#define TXD2 17

// L298N Motor Driver
#define ENA 4
#define IN1 5
#define IN2 6
#define ENB 7
#define IN3 15
#define IN4 16

// Buzzer / Horn
#define BUZZER_PIN 10

// Built-in WS2812 RGB LED (Commonly GPIO 48 on ESP32-S3)
#define RGB_LED_PIN 48
#define NUM_LEDS 1

// --- Hardware Objects ---
Adafruit_NeoPixel strip(NUM_LEDS, RGB_LED_PIN, NEO_GRB + NEO_KHZ800);
Adafruit_MPU6050 mpu;
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);
Adafruit_INA219 ina219;
TinyGPSPlus gps;
WebServer server(80);

// --- Network Settings ---
const char* ssid = "Swarnendu";
const char* password = "12345678";

// --- Sensor Variables ---
float gyroX_off = 0, gyroY_off = 0, gyroZ_off = 0;
float roll = 0, pitch = 0, yaw = 0;
float busvoltage = 0, current_mA = 0, power_mW = 0;
double latitude = 0.0, longitude = 0.0;
int satellites = 0;
bool is_calibrating = false;
unsigned long last_micros = 0;

// Mutex for safe data access between FreeRTOS tasks
SemaphoreHandle_t dataMutex;

// --- Tasks ---
TaskHandle_t GpsTask;
TaskHandle_t SirenTask;

// --- Drive Variables ---
int currentSpeed = 150;
bool hornActive = false;
bool lightOn = true;

// --- Motor Control Functions ---
void stopMotors() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
  analogWrite(ENA, 0); analogWrite(ENB, 0);
}

void moveForward() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
  analogWrite(ENA, currentSpeed); analogWrite(ENB, currentSpeed);
}

void moveBackward() {
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  analogWrite(ENA, currentSpeed); analogWrite(ENB, currentSpeed);
}

void turnLeft() {
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
  analogWrite(ENA, currentSpeed); analogWrite(ENB, currentSpeed);
}

void turnRight() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  analogWrite(ENA, currentSpeed); analogWrite(ENB, currentSpeed);
}

// --- Siren Task ---
void sirenTaskCode(void * parameter) {
  for(;;) {
    if (hornActive) {
      // Ascending pitch + RED Light
      strip.setPixelColor(0, strip.Color(255, 0, 0));
      strip.show();
      for (int freq = 500; freq <= 1200; freq += 20) {
        if (!hornActive) break;
        tone(BUZZER_PIN, freq);
        vTaskDelay(20 / portTICK_PERIOD_MS);
      }
      
      // Descending pitch + BLUE Light
      strip.setPixelColor(0, strip.Color(0, 0, 255));
      strip.show();
      for (int freq = 1200; freq >= 500; freq -= 20) {
        if (!hornActive) break;
        tone(BUZZER_PIN, freq);
        vTaskDelay(20 / portTICK_PERIOD_MS);
      }
    } else {
      noTone(BUZZER_PIN);
      if (lightOn) {
        static bool isRed = true;
        strip.setPixelColor(0, isRed ? strip.Color(255, 0, 0) : strip.Color(0, 0, 255));
        strip.show();
        isRed = !isRed;
        vTaskDelay(100 / portTICK_PERIOD_MS); // Fast strobe
      } else {
        strip.setPixelColor(0, strip.Color(0, 0, 0)); // Turn off LED
        strip.show();
        vTaskDelay(50 / portTICK_PERIOD_MS);
      }
    }
  }
}

// --- Sensor Functions ---
void calibrateSensors() {
  is_calibrating = true;
  float gx = 0, gy = 0, gz = 0;
  for (int i = 0; i < 150; i++) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    gx += g.gyro.x; gy += g.gyro.y; gz += g.gyro.z;
    delay(15); 
  }
  gyroX_off = gx / 150.0;
  gyroY_off = gy / 150.0;
  gyroZ_off = gz / 150.0;
  is_calibrating = false;
}

void updateSensors() {
  if (is_calibrating) return;

  unsigned long now_millis = millis();
  static unsigned long last_sensor_poll = 0;
  if (now_millis - last_sensor_poll < 20) return; // 50 Hz rate limit
  last_sensor_poll = now_millis;

  sensors_event_t a, g, temp, m;
  mpu.getEvent(&a, &g, &temp);
  mag.getEvent(&m);

  unsigned long now = micros();
  float dt = (now - last_micros) / 1000000.0;
  if (dt <= 0 || dt > 0.5) dt = 0.02;
  last_micros = now;

  float roll_acc = atan2(a.acceleration.y, a.acceleration.z) * 180 / PI;
  float pitch_acc = atan2(-a.acceleration.x, sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z)) * 180 / PI;

  if(xSemaphoreTake(dataMutex, portMAX_DELAY)) {
      roll = 0.96 * (roll + (g.gyro.x - gyroX_off) * dt * 180 / PI) + 0.04 * roll_acc;
      pitch = 0.96 * (pitch + (g.gyro.y - gyroY_off) * dt * 180 / PI) + 0.04 * pitch_acc;

      float phi = roll * PI / 180;
      float theta = pitch * PI / 180;
      float Xh = m.magnetic.x * cos(theta) + m.magnetic.z * sin(theta);
      float Yh = m.magnetic.x * sin(phi) * sin(theta) + m.magnetic.y * cos(phi) - m.magnetic.z * sin(phi) * cos(theta);
      
      yaw = atan2(Yh, Xh) * 180 / PI;
      if (yaw < 0) yaw += 360;
      
      if (isnan(roll)) roll = 0;
      if (isnan(pitch)) pitch = 0;
      if (isnan(yaw)) yaw = 0;

      busvoltage = ina219.getBusVoltage_V();
      current_mA = ina219.getCurrent_mA();
      power_mW = ina219.getPower_mW();
      
      xSemaphoreGive(dataMutex);
  }
}

// FreeRTOS Task to handle GPS constantly on Core 0
void gpsTaskCode(void * parameter) {
  for(;;) {
    while (Serial2.available() > 0) {
      if (gps.encode(Serial2.read())) {
        if (xSemaphoreTake(dataMutex, portMAX_DELAY)) {
          if (gps.location.isValid()) {
            latitude = gps.location.lat();
            longitude = gps.location.lng();
          }
          if (gps.satellites.isValid()) {
            satellites = gps.satellites.value();
          }
          xSemaphoreGive(dataMutex);
        }
      }
    }
    vTaskDelay(10 / portTICK_PERIOD_MS); // Yield to watchdog
  }
}

const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>ROVER COMMAND HQ</title>
    <link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css" />
    <script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script>
    <style>
        :root {
            --bg: #050505; --accent: #00ff41; --warning: #ffcf00; --danger: #ff3e3e;
            --panel: #111; --text: #00ff41; --mag: #00ccff; --mpu: #ff00ff;
        }

        body {
            background-color: var(--bg); color: var(--text);
            font-family: 'Courier New', Courier, monospace;
            margin: 0; padding: 10px; overflow: hidden;
            display: flex; flex-direction: column; height: 100vh;
        }

        header { display: flex; justify-content: space-between; align-items: center; border-bottom: 2px solid var(--accent); padding-bottom: 5px; margin-bottom: 10px; }
        .grid { display: grid; grid-template-columns: 320px 1fr 320px; grid-template-rows: 1fr 260px; gap: 10px; flex-grow: 1; }
        .panel { background: var(--panel); border: 1px solid #333; position: relative; padding: 10px; display: flex; flex-direction: column; }
        .panel-label { position: absolute; top: -8px; left: 10px; background: var(--bg); padding: 0 5px; font-size: 0.8em; color: #888; }
        
        canvas { width: 100%; height: auto; }
        #map { width: 100%; height: 100%; min-height: 200px; border: 1px solid #333; }

        .stat-row { display: flex; justify-content: space-between; font-size: 0.9em; margin: 2px 0; }
        .val-mag { color: var(--mag); }
        .val-mpu { color: var(--mpu); }

        .bar-container { width: 100%; height: 8px; background: #222; border: 1px solid #444; margin: 4px 0; }
        .bar-fill { height: 100%; width: 0%; transition: width 0.2s; }
        
        /* Controls Styles */
        .controls-panel { align-items: center; justify-content: center; padding-top: 15px; }
        .dpad { display: grid; grid-template-columns: 60px 60px 60px; grid-template-rows: 60px 60px 60px; gap: 5px; margin-bottom: 15px; }
        .btn { background: #222; border: 2px solid var(--accent); color: var(--accent); font-weight: bold; cursor: pointer; border-radius: 8px; font-size: 1.5em; display: flex; align-items: center; justify-content: center; user-select: none; }
        .btn:active { background: var(--accent); color: #000; }
        .btn-up { grid-column: 2; grid-row: 1; }
        .btn-left { grid-column: 1; grid-row: 2; }
        .btn-right { grid-column: 3; grid-row: 2; }
        .btn-down { grid-column: 2; grid-row: 3; }
        .btn-stop { grid-column: 2; grid-row: 2; border-color: var(--warning); color: var(--warning); }
        .btn-stop:active { background: var(--warning); color: #000; }
        
        .slider-container { width: 100%; text-align: center; margin-bottom: 15px; }
        input[type="range"] { width: 90%; accent-color: var(--accent); cursor: pointer; }

        .horn-btn { width: 100%; padding: 12px; background: #222; border: 2px solid var(--danger); color: var(--danger); font-size: 1.2em; font-weight: bold; border-radius: 8px; cursor: pointer; user-select: none; text-align: center; box-sizing: border-box; }
        .horn-btn:active { background: var(--danger); color: #fff; }

        .online { color: var(--accent); } .offline { color: var(--danger); }
        @media (max-width: 900px) { .grid { grid-template-columns: 1fr; overflow-y: auto; } body { overflow: auto; } }
    </style>
</head>
<body>

<header>
    <div><strong>DISASTER ROVER HQ</strong> <span id="mode-status">[ MANUAL ]</span></div>
    <div id="conn-status" class="offline">LINK: DISCONNECTED</div>
    <div style="font-size: 0.8em;">RSSI: <span id="rssi">-00</span> dBm</div>
</header>

<div class="grid">
    <!-- Left Column: Primary HUD -->
    <div class="panel">
        <span class="panel-label">ATTITUDE & HORIZON</span>
        <canvas id="horizonCanvas" width="300" height="200"></canvas>
        <div class="stat-row">Pitch: <span id="pitch-txt">0.0</span>°</div>
        <div class="stat-row">Roll: <span id="roll-txt">0.0</span>°</div>
        
        <div style="margin-top: 15px; border-top: 1px solid #333; padding-top: 10px;">
            <span style="font-size: 0.7em; color: #888;">MAG RAW (µT)</span>
            <div class="stat-row">X: <span id="mx" class="val-mag">0</span> Y: <span id="my" class="val-mag">0</span> Z: <span id="mz" class="val-mag">0</span></div>
            <span style="font-size: 0.7em; color: #888; margin-top:5px; display:block;">MPU RAW (m/s²)</span>
            <div class="stat-row">X: <span id="ax" class="val-mpu">0</span> Y: <span id="ay" class="val-mpu">0</span> Z: <span id="az" class="val-mpu">0</span></div>
        </div>
    </div>

    <!-- Center: Map -->
    <div class="panel">
        <span class="panel-label">LIVE MAP</span>
        <div id="map"></div>
    </div>

    <!-- Right: Sensor Fusion (New Compass + Axis) -->
    <div class="panel">
        <span class="panel-label">NAVIGATION & VECTORS</span>
        <canvas id="compassCanvas" width="300" height="160"></canvas>
        <div style="text-align: center; font-size: 1.2em; font-weight: bold; margin-bottom: 10px;" id="heading-txt">0° NORTH</div>
        
        <span style="font-size: 0.7em; color: #888;">3D AXIS VISUALIZER</span>
        <canvas id="axisCanvas" width="300" height="150"></canvas>
    </div>

    <!-- Bottom Center: Controls (Centered) -->
    <div class="panel controls-panel" style="grid-column: 2;">
        <span class="panel-label">ROVER DRIVE & ACTUATORS</span>
        
        <div class="slider-container">
            <label>THROTTLE: <span id="speed-val">150</span></label><br>
            <input type="range" id="speed-slider" min="50" max="255" value="150" oninput="updateSpeed(this.value)">
        </div>

        <div class="dpad">
            <div class="btn btn-up" id="btn-f">▲</div>
            <div class="btn btn-left" id="btn-l">◄</div>
            <div class="btn btn-stop" id="btn-s">◼</div>
            <div class="btn btn-right" id="btn-r">►</div>
            <div class="btn btn-down" id="btn-b">▼</div>
        </div>

        <div class="horn-btn" id="light-btn" style="border-color: var(--text); color: var(--text); margin-bottom: 8px;">💡 POLICE LIGHT: ON</div>
        <div class="horn-btn" id="horn-btn">🚨 SOUND SIREN</div>
    </div>

    <!-- Bottom Right: Power Stats -->
    <div class="panel">
        <span class="panel-label">SYSTEM PWR</span>
        <div style="margin-top: auto;">
            <div class="stat-row">Battery: <span id="bat-val">0%</span></div>
            <div class="bar-container"><div id="bat-bar" class="bar-fill" style="background:var(--accent)"></div></div>
        </div>
        <div class="stat-row" style="margin-top: 10px;">Volts: <span id="v-val">0.0V</span></div>
        <div class="stat-row">Amps: <span id="c-val">0mA</span></div>
        <div class="stat-row">Watts: <span id="p-val">0.00W</span></div>
        <button onclick="toggleMode()" style="margin-top:10px; width:100%; border:1px solid var(--accent); background:transparent; color:var(--accent); cursor:pointer;">MODE SELECT</button>
    </div>
</div>

<script>
    let isHornOn = false;
    let isLightOn = true;

    // --- Commands API ---
    const sendCmd = (dir) => {
        fetch(`/cmd?dir=${dir}`);
    }

    const updateSpeed = (val) => {
        document.getElementById('speed-val').innerText = val;
        fetch(`/speed?val=${val}`);
    }

    const toggleHorn = () => {
        isHornOn = !isHornOn;
        fetch(`/horn?state=${isHornOn ? 1 : 0}`);
        const el = document.getElementById('horn-btn');
        if (el) {
            el.style.background = isHornOn ? 'var(--danger)' : '#222';
            el.style.color = isHornOn ? '#fff' : 'var(--danger)';
        }
    }

    const toggleLight = () => {
        isLightOn = !isLightOn;
        document.getElementById('light-btn').innerText = isLightOn ? '💡 POLICE LIGHT: ON' : '💡 POLICE LIGHT: OFF';
        fetch(`/light?state=${isLightOn ? 1 : 0}`);
    }

    const attachHoldEvent = (id, startFunc, endFunc) => {
        const el = document.getElementById(id);
        if(!el) return;
        const press = (e) => { e.preventDefault(); startFunc(); };
        const release = (e) => { e.preventDefault(); endFunc(); };
        el.addEventListener('mousedown', press);
        el.addEventListener('touchstart', press, {passive: false});
        el.addEventListener('mouseup', release);
        el.addEventListener('mouseleave', release);
        el.addEventListener('touchend', release);
        el.addEventListener('touchcancel', release);
    }

    // --- Polling Logic ---
    const startPolling = () => {
        document.getElementById('conn-status').className = 'online';
        document.getElementById('conn-status').innerText = 'LINK: CONNECTED';
        
        setInterval(() => {
            fetch('/data').then(r => r.json()).then(d => {
                document.getElementById('conn-status').className = 'online';
                document.getElementById('conn-status').innerText = 'LINK: CONNECTED';
                updateUI(d);
            }).catch(e => {
                document.getElementById('conn-status').className = 'offline';
                document.getElementById('conn-status').innerText = 'LINK: OFFLINE';
            });
        }, 200);
    };

    // --- Drawing Contexts ---
    const hCanvas = document.getElementById('horizonCanvas');
    const cCanvas = document.getElementById('compassCanvas');
    const aCanvas = document.getElementById('axisCanvas');
    const hCtx = hCanvas.getContext('2d');
    const cCtx = cCanvas.getContext('2d');
    const aCtx = aCanvas.getContext('2d');

    // --- Leaflet Setup ---
    const map = L.map('map', {zoomControl:true, attributionControl:false}).setView([0,0], 2);
    L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png').addTo(map);
    const marker = L.marker([0,0]).addTo(map);
    let mapCentered = false;

    const updateUI = (d) => {
        document.getElementById('pitch-txt').innerText = d.pitch.toFixed(1);
        document.getElementById('roll-txt').innerText = d.roll.toFixed(1);
        document.getElementById('heading-txt').innerText = `${d.heading}° ${getCardinal(d.heading)}`;
        document.getElementById('bat-val').innerText = d.battery + "%";
        document.getElementById('bat-bar').style.width = d.battery + "%";
        document.getElementById('v-val').innerText = d.voltage + "V";
        document.getElementById('c-val').innerText = d.current + "mA";
        document.getElementById('p-val').innerText = (d.voltage * (d.current/1000)).toFixed(2) + "W";

        document.getElementById('mx').innerText = d.magX || 0;
        document.getElementById('my').innerText = d.magY || 0;
        document.getElementById('mz').innerText = d.magZ || 0;
        document.getElementById('ax').innerText = d.ax || 0;
        document.getElementById('ay').innerText = d.ay || 0;
        document.getElementById('az').innerText = d.az || 0;

        drawHorizon(d.pitch, d.roll);
        drawCompass(d.heading);
        drawAxes(d.magX || 0, d.magY || 0, d.magZ || 0, d.ax || 0, d.ay || 0, d.az || 0);

        if(d.lat != 0) {
            const p = [d.lat, d.lng];
            marker.setLatLng(p);
            if (!mapCentered) {
                map.setView(p, 16);
                mapCentered = true;
            } else {
                map.setView(p, map.getZoom());
            }
        }
    }

    const drawHorizon = (pitch, roll) => {
        const w = hCanvas.width, h = hCanvas.height, cx = w/2, cy = h/2;
        hCtx.clearRect(0,0,w,h);
        hCtx.save();
        hCtx.translate(cx, cy);
        hCtx.rotate(-roll * Math.PI / 180);
        
        hCtx.fillStyle = "#3e2723"; hCtx.fillRect(-w*2, pitch*2, w*4, h*2);
        hCtx.fillStyle = "#1a237e"; hCtx.fillRect(-w*2, -h*2 + pitch*2, w*4, h*2);
        
        hCtx.strokeStyle = "#fff";
        for(let i=-60; i<=60; i+=20) {
            let py = -i*2 + pitch*2;
            hCtx.beginPath(); hCtx.moveTo(-20, py); hCtx.lineTo(20, py); hCtx.stroke();
        }
        hCtx.restore();
        hCtx.strokeStyle = "red"; hCtx.strokeRect(cx-10, cy-10, 20, 20);
    }

    const drawCompass = (hdg) => {
        const w = cCanvas.width, h = cCanvas.height, cx = w/2, cy = h/2;
        cCtx.clearRect(0,0,w,h);
        cCtx.save();
        cCtx.translate(cx, cy);
        cCtx.rotate(-hdg * Math.PI/180);
        
        cCtx.strokeStyle = "var(--accent)"; cCtx.lineWidth = 2;
        cCtx.beginPath(); cCtx.arc(0,0,50,0,Math.PI*2); cCtx.stroke();
        
        const labels = ['N','E','S','W'];
        for(let i=0; i<4; i++) {
            cCtx.fillStyle = i===0 ? "red" : "white";
            cCtx.fillText(labels[i], -5, -55);
            cCtx.rotate(Math.PI/2);
        }
        cCtx.restore();
        cCtx.fillStyle = "white"; cCtx.beginPath(); cCtx.moveTo(cx, cy-45); cCtx.lineTo(cx-5, cy-30); cCtx.lineTo(cx+5, cy-30); cCtx.fill();
    }

    const drawAxes = (mx, my, mz, ax, ay, az) => {
        const w = aCanvas.width, h = aCanvas.height, cx = w/2, cy = h/2;
        aCtx.clearRect(0,0,w,h);
        
        aCtx.strokeStyle = "#222";
        aCtx.beginPath(); aCtx.moveTo(0,cy); aCtx.lineTo(w,cy); aCtx.moveTo(cx,0); aCtx.lineTo(cx,h); aCtx.stroke();

        const drawVector = (x, y, z, color, label) => {
            aCtx.strokeStyle = color;
            aCtx.beginPath();
            aCtx.moveTo(cx, cy);
            let tx = cx + (x * 0.5) + (z * 0.3);
            let ty = cy - (y * 0.5) + (z * 0.3);
            aCtx.lineTo(tx, ty);
            aCtx.stroke();
            aCtx.fillStyle = color;
            aCtx.fillText(label, tx, ty);
        }

        drawVector(mx/2, my/2, mz/2, "var(--mag)", "MAG");
        drawVector(ax*5, ay*5, az*5, "var(--mpu)", "ACC");
    }



    const getCardinal = (angle) => {
        const d = ["N", "NE", "E", "SE", "S", "SW", "W", "NW"];
        return d[Math.round(angle / 45) % 8];
    }

    const toggleMode = () => {
        const mode = document.getElementById('mode-status');
        mode.innerText = mode.innerText === "[ MANUAL ]" ? "[ AUTO ]" : "[ MANUAL ]";
    }

    window.onload = () => {
        startPolling();
        attachHoldEvent('btn-f', () => sendCmd('F'), () => sendCmd('S'));
        attachHoldEvent('btn-b', () => sendCmd('B'), () => sendCmd('S'));
        attachHoldEvent('btn-l', () => sendCmd('L'), () => sendCmd('S'));
        attachHoldEvent('btn-r', () => sendCmd('R'), () => sendCmd('S'));
        
        const stopBtn = document.getElementById('btn-s');
        if(stopBtn) {
            stopBtn.addEventListener('mousedown', (e) => { e.preventDefault(); sendCmd('S'); });
            stopBtn.addEventListener('touchstart', (e) => { e.preventDefault(); sendCmd('S'); }, {passive: false});
        }

        const hornBtn = document.getElementById('horn-btn');
        if(hornBtn) {
            hornBtn.addEventListener('click', toggleHorn);
            hornBtn.addEventListener('touchstart', (e) => { e.preventDefault(); toggleHorn(); }, {passive: false});
        }
        
        const lightBtn = document.getElementById('light-btn');
        if(lightBtn) {
            lightBtn.addEventListener('click', toggleLight);
            lightBtn.addEventListener('touchstart', (e) => { e.preventDefault(); toggleLight(); }, {passive: false});
        }
    };
</script>
</body>
</html>
)rawliteral";

void setup() {
  Serial.begin(115200);
  
  // Init I2C for ESP32-S3
  Wire.begin(I2C_SDA, I2C_SCL);
  
  // Init RGB LED
  strip.begin();
  strip.show(); // Initialize all pixels to 'off'
  strip.setBrightness(150); // Set brightness to ~60%
  
  // Init Motor Pins
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  stopMotors();

  // Init Sensors
  if (!mpu.begin()) Serial.println("Failed to find MPU6050");
  if (!mag.begin()) Serial.println("Failed to find HMC5883L");
  if (!ina219.begin()) Serial.println("Failed to find INA219");
  
  calibrateSensors();
  
  // Init GPS Serial for ESP32-S3
  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);
  
  // Create Mutex
  dataMutex = xSemaphoreCreateMutex();
  
  // Start GPS Task on Core 0
  xTaskCreatePinnedToCore(
    gpsTaskCode,   // Task function
    "GPS_Task",    // Name of task
    10000,         // Stack size
    NULL,          // Parameter
    1,             // Priority
    &GpsTask,      // Task handle
    0              // Core 0
  );

  // Start Siren Task on Core 1 (non-blocking)
  xTaskCreatePinnedToCore(
    sirenTaskCode,
    "Siren_Task",
    4000,
    NULL,
    1,
    &SirenTask,
    1
  );

  // Setup WiFi Station
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nIP Address: " + WiFi.localIP().toString());
  
  // Web Server Routes
  server.on("/", []() { server.send(200, "text/html", index_html); });
  
  server.on("/data", [](){
    float c_roll = 0, c_pitch = 0, c_yaw = 0, c_volt = 0, c_curr = 0, c_pwr = 0;
    double c_lat = 0, c_lng = 0;
    int c_sats = 0;
    
    if(xSemaphoreTake(dataMutex, portMAX_DELAY)) {
      c_roll = roll; c_pitch = pitch; c_yaw = yaw;
      c_volt = busvoltage; c_curr = current_mA; c_pwr = power_mW;
      c_lat = latitude; c_lng = longitude; c_sats = satellites;
      xSemaphoreGive(dataMutex);
    }
    
    int bat_pct = (c_volt / 8.4) * 100;
    if (bat_pct > 100) bat_pct = 100;
    if (bat_pct < 0) bat_pct = 0;

    String json = "{";
    json += "\"roll\":" + String(c_roll) + ",";
    json += "\"pitch\":" + String(c_pitch) + ",";
    json += "\"heading\":" + String(c_yaw) + ",";
    json += "\"voltage\":" + String(c_volt) + ",";
    json += "\"current\":" + String(c_curr) + ",";
    json += "\"battery\":" + String(bat_pct) + ",";
    json += "\"lat\":" + String(c_lat, 6) + ",";
    json += "\"lng\":" + String(c_lng, 6) + ",";
    json += "\"sats\":" + String(c_sats);
    json += "}";
    
    server.send(200, "application/json", json);
  });
  
  // Command API
  server.on("/cmd", [](){
    if(server.hasArg("dir")) {
      String dir = server.arg("dir");
      if(dir == "F") moveForward();
      else if(dir == "B") moveBackward();
      else if(dir == "L") turnLeft();
      else if(dir == "R") turnRight();
      else if(dir == "S") stopMotors();
    }
    server.send(200, "text/plain", "OK");
  });

  server.on("/speed", [](){
    if(server.hasArg("val")) {
      currentSpeed = server.arg("val").toInt();
      // Optionally update motor speed instantly if already moving
      // by relying on loop continuous update or immediate write
      // We will let the next movement command apply the speed for safety,
      // or we can apply to ENA/ENB directly if they are already not zero.
    }
    server.send(200, "text/plain", "OK");
  });

  server.on("/horn", [](){
    if(server.hasArg("state")) {
      hornActive = (server.arg("state") == "1");
    }
    server.send(200, "text/plain", "OK");
  });

  server.on("/light", [](){
    if(server.hasArg("state")) {
      lightOn = (server.arg("state") == "1");
    }
    server.send(200, "text/plain", "OK");
  });

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
