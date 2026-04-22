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
const char* ssid = "ARES-PRECISION-COMPASS"; 
const char* password = "password123"; 

// --- Sensor Variables ---
float gyroX_off = 0, gyroY_off = 0, gyroZ_off = 0;
float roll = 0, pitch = 0, yaw = 0;
bool is_calibrating = false;
unsigned long last_micros = 0;

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
  sensors_event_t a, g, temp, m;
  mpu.getEvent(&a, &g, &temp);
  mag.getEvent(&m);

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
  float Xh = m.magnetic.x * cos(theta) + m.magnetic.z * sin(theta);
  float Yh = m.magnetic.x * sin(phi) * sin(theta) + m.magnetic.y * cos(phi) - m.magnetic.z * sin(phi) * cos(theta);
  
  yaw = atan2(Yh, Xh) * 180 / PI;
  if (yaw < 0) yaw += 360;
}

const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html><html><head>
<meta name="viewport" content="width=device-width, initial-scale=1">
<style>
    body { background: #000; color: #fff; font-family: sans-serif; display: flex; flex-direction: column; align-items: center; margin: 0; padding-top: 20px; }
    
    .hud-header { font-size: 2em; font-weight: bold; margin-bottom: 5px; color: #fff; }
    .hud-sub { font-size: 1.2em; color: #ff3131; margin-bottom: 20px; }

    /* Main Compass Container */
    .compass-container {
        position: relative; width: 320px; height: 320px;
        display: flex; align-items: center; justify-content: center;
    }

    /* Fixed Red Pointer at Top */
    .top-pointer {
        position: absolute; top: -10px; width: 20px; height: 30px;
        background: #ff0000; clip-path: polygon(50% 100%, 0 0, 100% 0);
        z-index: 10;
    }

    /* The Rotating Dial */
    #compass-dial {
        width: 300px; height: 300px;
        transition: transform 0.1s linear;
    }

    /* Horizon Oval */
    .horizon-box {
        width: 200px; height: 80px; border: 1px solid #333;
        border-radius: 50%; margin-top: 30px; position: relative; overflow: hidden;
    }
    #horizon-line {
        width: 400px; height: 2px; background: #00f2ff;
        position: absolute; top: 50%; left: -100px;
    }

    button { 
        margin-top: 30px; background: #222; border: 1px solid #444; 
        color: #888; padding: 10px 20px; cursor: pointer; border-radius: 5px;
    }
</style>
</head><body>

    <div class="hud-header" id="deg-text">0°</div>
    <div class="hud-sub">NORTH</div>

    <div class="compass-container">
        <div class="top-pointer"></div>
        <svg id="compass-dial" viewBox="0 0 200 200">
            <circle cx="100" cy="100" r="95" fill="none" stroke="#333" stroke-width="1"/>
            
            <g id="ticks" stroke="#fff" stroke-width="1">
                <line x1="100" y1="10" x2="100" y2="20" transform="rotate(0, 100, 100)" />
                <line x1="100" y1="10" x2="100" y2="15" transform="rotate(30, 100, 100)" />
                <line x1="100" y1="10" x2="100" y2="15" transform="rotate(60, 100, 100)" />
                <line x1="100" y1="10" x2="100" y2="20" transform="rotate(90, 100, 100)" />
                <line x1="100" y1="10" x2="100" y2="15" transform="rotate(120, 100, 100)" />
                <line x1="100" y1="10" x2="100" y2="15" transform="rotate(150, 100, 100)" />
                <line x1="100" y1="10" x2="100" y2="20" transform="rotate(180, 100, 100)" />
                <line x1="100" y1="10" x2="100" y2="15" transform="rotate(210, 100, 100)" />
                <line x1="100" y1="10" x2="100" y2="15" transform="rotate(240, 100, 100)" />
                <line x1="100" y1="10" x2="100" y2="20" transform="rotate(270, 100, 100)" />
                <line x1="100" y1="10" x2="100" y2="15" transform="rotate(300, 100, 100)" />
                <line x1="100" y1="10" x2="100" y2="15" transform="rotate(330, 100, 100)" />
            </g>

            <g fill="#fff" font-family="Arial" font-weight="bold" text-anchor="middle">
                <text x="100" y="40" fill="#ff3131" font-size="24">N</text>
                <text x="165" y="108" font-size="20">E</text>
                <text x="100" y="175" font-size="20">S</text>
                <text x="35" y="108" font-size="20">W</text>
                
                <text x="145" y="60" fill="#888" font-size="10">NE</text>
                <text x="145" y="150" fill="#888" font-size="10">SE</text>
                <text x="55" y="150" fill="#888" font-size="10">SW</text>
                <text x="55" y="60" fill="#888" font-size="10">NW</text>
            </g>

            <polygon points="100,70 105,95 130,100 105,105 100,130 95,105 70,100 95,95" fill="#111" stroke="#333" stroke-width="1"/>
        </svg>
    </div>

    <div class="horizon-box">
        <div id="horizon-line"></div>
    </div>

    <button onclick="fetch('/start_cal')">CALIBRATE SENSORS</button>

<script>
    function getCardinal(angle) {
        const directions = ["NORTH", "NE", "EAST", "SE", "SOUTH", "SW", "WEST", "NW"];
        return directions[Math.round(angle / 45) % 8];
    }

    setInterval(() => {
        fetch('/data').then(r => r.json()).then(d => {
            // Rotate the entire dial based on yaw
            document.getElementById('compass-dial').style.transform = `rotate(${-d.yaw}deg)`;
            document.getElementById('deg-text').innerText = Math.round(d.yaw) + "°";
            document.querySelector('.hud-sub').innerText = getCardinal(d.yaw);
            
            // Update Horizon (Pitch/Roll)
            document.getElementById('horizon-line').style.transform = `rotate(${d.roll}deg) translateY(${d.pitch}px)`;
        });
    }, 100);
</script>
</body></html>
)rawliteral";

void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);
  mpu.begin();
  mag.begin();
  calibrateSensors();
  WiFi.softAP(ssid, password);
  server.on("/", []() { server.send(200, "text/html", index_html); });
  server.on("/data", [](){
    String json = "{\"roll\":" + String((int)roll) + ",\"pitch\":" + String((int)pitch) + ",\"yaw\":" + String((int)yaw) + "}";
    server.send(200, "application/json", json);
  });
  server.on("/start_cal", []() { server.send(200, "text/plain", "OK"); calibrateSensors(); });
  server.begin();
  last_micros = micros();
}

void loop() {
  server.handleClient();
  updateSensors();
}
