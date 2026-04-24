#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_MPU6050.h>
#include <TinyGPSPlus.h>
#include <HardwareSerial.h>
#include <WiFiClientSecure.h>
#include <UniversalTelegramBot.h>
#include <ArduinoJson.h>

#define BOT_TOKEN "8710079110:AAFsugljOiBE46NT-lMHPMG_zY0PUwGAWag"
#define CHAT_ID "7901238515"
WiFiClientSecure secured_client;
UniversalTelegramBot bot(BOT_TOKEN, secured_client);

unsigned long bot_lasttime = 0;
const unsigned long BOT_MTBS = 1500;
bool isTracking = false;
unsigned long track_lasttime = 0;
const unsigned long TRACK_INTERVAL = 10000;

const char* ssid = "Swarnendu";
const char* password = "12345678";

WebServer server(80);

// Sensors
Adafruit_MPU6050 mpu;
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
float pitch = 0, roll = 0, temperature = 0;
float gyroX_off = 0, gyroY_off = 0, gyroZ_off = 0;
bool is_calibrating = false;
unsigned long last_micros = 0;

// --- Siren Variables ---
bool sirenActive = false;
unsigned long sirenTimer = 0;
bool sirenHighFreq = true;

// ===== AUTO RETURN START =====
// --- Auto Return Variables ---
bool autoMode = false;
double homeLat = 0.0, homeLng = 0.0;
bool homeSet = false;
double heading = 0.0;
double gyroZ_offset = 0.0;
unsigned long lastHeadingMicros = 0;
double prevLat = 0.0, prevLng = 0.0;
bool prevGPSSet = false;
unsigned long lastAutoMillis = 0;
const unsigned long AUTO_INTERVAL = 200;
unsigned long lastGPSCorrection = 0;
const unsigned long GPS_CORRECT_INTERVAL = 2000;
double distanceToHome = 0.0;
// ===== AUTO RETURN END =====

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
  gyroZ_offset = gyroZ_off; // Also set auto-return heading offset
  is_calibrating = false;
}

void updateSensors() {
  if (is_calibrating) return;
  
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  temperature = temp.temperature;
  
  unsigned long now = micros();
  float dt = (now - last_micros) / 1000000.0;
  if (dt <= 0 || dt > 0.5) dt = 0.01;
  last_micros = now;

  float roll_acc = atan2(a.acceleration.y, a.acceleration.z) * 180 / PI;
  float pitch_acc = atan2(-a.acceleration.x, sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z)) * 180 / PI;

  roll = 0.96 * (roll + (g.gyro.x - gyroX_off) * dt * 180 / PI) + 0.04 * roll_acc;
  pitch = 0.96 * (pitch + (g.gyro.y - gyroY_off) * dt * 180 / PI) + 0.04 * pitch_acc;
}

// ===== AUTO RETURN FUNCTIONS START =====
double calculateDistance(double lat1, double lng1, double lat2, double lng2) {
  double dLat = radians(lat2 - lat1);
  double dLng = radians(lng2 - lng1);
  double a = sin(dLat / 2.0) * sin(dLat / 2.0) +
             cos(radians(lat1)) * cos(radians(lat2)) *
             sin(dLng / 2.0) * sin(dLng / 2.0);
  double c = 2.0 * atan2(sqrt(a), sqrt(1.0 - a));
  return 6371000.0 * c;
}

double calculateBearing(double lat1, double lng1, double lat2, double lng2) {
  double dLng = radians(lng2 - lng1);
  double y = sin(dLng) * cos(radians(lat2));
  double x = cos(radians(lat1)) * sin(radians(lat2)) -
             sin(radians(lat1)) * cos(radians(lat2)) * cos(dLng);
  double brng = atan2(y, x) * 180.0 / PI;
  if (brng < 0) brng += 360.0;
  return brng;
}

void updateHeading() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  unsigned long now = micros();
  float dt = (now - lastHeadingMicros) / 1000000.0;
  if (dt <= 0 || dt > 0.5) dt = 0.01;
  lastHeadingMicros = now;
  heading += (g.gyro.z - gyroZ_offset) * dt * 180.0 / PI;
  if (heading > 360.0) heading -= 360.0;
  if (heading < 0.0) heading += 360.0;
}

void correctHeadingWithGPS() {
  if (!gps.location.isValid() || gps.location.age() > 5000) return;
  double curLat = gps.location.lat();
  double curLng = gps.location.lng();
  if (prevGPSSet) {
    double dist = calculateDistance(prevLat, prevLng, curLat, curLng);
    if (dist > 1.5) {
      double gpsBearing = calculateBearing(prevLat, prevLng, curLat, curLng);
      heading = heading * 0.3 + gpsBearing * 0.7;
      if (heading > 360.0) heading -= 360.0;
      if (heading < 0.0) heading += 360.0;
    }
  }
  prevLat = curLat;
  prevLng = curLng;
  prevGPSSet = true;
}

void runAutoReturn() {
  if (!homeSet) {
    stopMotors();
    autoMode = false;
    Serial.println("[AUTO] No home set. Stopping.");
    return;
  }
  if (!gps.location.isValid() || gps.location.age() > 5000) {
    stopMotors();
    Serial.println("[AUTO] GPS invalid. Failsafe stop.");
    return;
  }

  updateHeading();

  if (millis() - lastGPSCorrection >= GPS_CORRECT_INTERVAL) {
    correctHeadingWithGPS();
    lastGPSCorrection = millis();
  }

  double curLat = gps.location.lat();
  double curLng = gps.location.lng();
  distanceToHome = calculateDistance(curLat, curLng, homeLat, homeLng);
  double targetBearing = calculateBearing(curLat, curLng, homeLat, homeLng);

  double error = targetBearing - heading;
  if (error > 180.0) error -= 360.0;
  if (error < -180.0) error += 360.0;

  Serial.println("--- AUTO RETURN DEBUG ---");
  Serial.print("CUR: "); Serial.print(curLat, 6); Serial.print(", "); Serial.println(curLng, 6);
  Serial.print("HOME: "); Serial.print(homeLat, 6); Serial.print(", "); Serial.println(homeLng, 6);
  Serial.print("DIST: "); Serial.print(distanceToHome); Serial.println(" m");
  Serial.print("BEARING: "); Serial.println(targetBearing);
  Serial.print("HEADING: "); Serial.println(heading);
  Serial.print("ERROR: "); Serial.println(error);

  if (distanceToHome < 3.0) {
    stopMotors();
    autoMode = false;
    Serial.println("[AUTO] HOME REACHED!");
    return;
  }

  if (error > 10.0) {
    turnRight();
  } else if (error < -10.0) {
    turnLeft();
  } else {
    moveForward();
  }
}
// ===== AUTO RETURN FUNCTIONS END =====

void handleTelegramMessages(int numNewMessages) {
  for (int i = 0; i < numNewMessages; i++) {
    String chat_id = String(bot.messages[i].chat_id);
    if (chat_id != CHAT_ID) {
      bot.sendMessage(chat_id, "Unauthorized user", "");
      continue;
    }
    
    String text = bot.messages[i].text;

    if (text == "/start") {
      String welcome = "🚨 ROVER CONTROL PANEL\n";
      String keyboardJson = "[[\"⬆ Forward\"], [\"⬅ Left\", \"⏹ Stop\", \"➡ Right\"], [\"⬇ Back\"], [\"🚨 Siren\", \"📍 Location\"], [\"📡 Track ON\", \"📡 Track OFF\"], [\"🏠 Set Home\", \"🤖 Auto Return\", \"🛑 Cancel Auto\"]]";
      bot.sendMessageWithReplyKeyboard(chat_id, welcome, "", keyboardJson, true);
    }
    else if (text == "⬆ Forward") {
      autoMode = false;
      moveForward();
      bot.sendMessage(chat_id, "Moving Forward", "");
    }
    else if (text == "⬇ Back") {
      autoMode = false;
      moveBackward();
      bot.sendMessage(chat_id, "Moving Backward", "");
    }
    else if (text == "⬅ Left") {
      autoMode = false;
      turnLeft();
      bot.sendMessage(chat_id, "Turning Left", "");
    }
    else if (text == "➡ Right") {
      autoMode = false;
      turnRight();
      bot.sendMessage(chat_id, "Turning Right", "");
    }
    else if (text == "⏹ Stop") {
      autoMode = false;
      stopMotors();
      bot.sendMessage(chat_id, "Motors Stopped", "");
    }
    else if (text == "🚨 Siren") {
      sirenActive = !sirenActive;
      bot.sendMessage(chat_id, sirenActive ? "Siren is ON" : "Siren is OFF", "");
    }
    else if (text == "📍 Location") {
      if (gps.location.isValid() && gps.location.age() < 5000) {
        String msg = "https://maps.google.com/?q=" + String(gps.location.lat(), 6) + "," + String(gps.location.lng(), 6);
        bot.sendMessage(chat_id, msg, "");
      } else {
        bot.sendMessage(chat_id, "❌ GPS not connected to satellites", "");
      }
    }
    else if (text == "📡 Track ON") {
      isTracking = true;
      bot.sendMessage(chat_id, "Live tracking started (10s interval)", "");
    }
    else if (text == "📡 Track OFF") {
      isTracking = false;
      bot.sendMessage(chat_id, "Live tracking stopped", "");
    }
    // ===== AUTO RETURN TELEGRAM START =====
    else if (text == "🏠 Set Home") {
      if (gps.location.isValid() && gps.location.age() < 5000) {
        homeLat = gps.location.lat();
        homeLng = gps.location.lng();
        homeSet = true;
        String msg = "🏠 Home set: " + String(homeLat, 6) + ", " + String(homeLng, 6);
        bot.sendMessage(chat_id, msg, "");
      } else {
        bot.sendMessage(chat_id, "❌ GPS invalid. Cannot set home.", "");
      }
    }
    else if (text == "🤖 Auto Return") {
      if (homeSet) {
        autoMode = true;
        lastHeadingMicros = micros();
        prevGPSSet = false;
        lastGPSCorrection = millis();
        bot.sendMessage(chat_id, "🤖 Auto Return ACTIVATED", "");
      } else {
        bot.sendMessage(chat_id, "❌ Home not set. Send 🏠 Set Home first.", "");
      }
    }
    else if (text == "🛑 Cancel Auto") {
      autoMode = false;
      stopMotors();
      bot.sendMessage(chat_id, "🛑 Auto Return CANCELLED", "");
    }
    // ===== AUTO RETURN TELEGRAM END =====
  }
}

// Embedded HTML UI
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0, maximum-scale=1.0, user-scalable=no">
    <title>ESP32 V-CORE ROVER HUD</title>
    <link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css" />
    <script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script>
    <style>
        @import url('https://fonts.googleapis.com/css2?family=Orbitron:wght@400;700&family=Rajdhani:wght@300;500;700&display=swap');

        :root {
            --neon-blue: #06b6d4;
            --neon-red: #ef4444;
            --neon-green: #22c55e;
            --glass-bg: rgba(15, 23, 42, 0.7);
            --panel-border: 1px solid rgba(6, 182, 212, 0.2);
            --glow: 0 0 15px rgba(6, 182, 212, 0.3);
        }

        * { box-sizing: border-box; -webkit-tap-highlight-color: transparent; }
        
        body { 
            margin: 0; 
            font-family: 'Rajdhani', sans-serif; 
            background: #020617; 
            background-image: 
                radial-gradient(circle at 50% 50%, rgba(15, 23, 42, 0) 0%, rgba(2, 6, 23, 1) 100%),
                linear-gradient(rgba(6, 182, 212, 0.05) 1px, transparent 1px),
                linear-gradient(90deg, rgba(6, 182, 212, 0.05) 1px, transparent 1px);
            background-size: 100% 100%, 30px 30px, 30px 30px;
            color: #e2e8f0; 
            min-height: 100vh; 
            overflow-x: hidden; 
        }

        /* Scanline effect */
        body::before {
            content: " ";
            display: block;
            position: fixed;
            top: 0; left: 0; bottom: 0; right: 0;
            background: linear-gradient(rgba(18, 16, 16, 0) 50%, rgba(0, 0, 0, 0.1) 50%), 
                        linear-gradient(90deg, rgba(255, 0, 0, 0.02), rgba(0, 255, 0, 0.01), rgba(0, 0, 255, 0.02));
            z-index: 9999;
            background-size: 100% 3px, 3px 100%;
            pointer-events: none;
        }

        .header { 
            padding: 15px; 
            text-align: center; 
            border-bottom: 2px solid var(--neon-blue);
            box-shadow: var(--glow);
            background: rgba(15, 23, 42, 0.9);
        }
        
        h1 { 
            margin: 0; 
            font-family: 'Orbitron', sans-serif;
            font-size: clamp(18px, 5vw, 24px); 
            letter-spacing: 4px; 
            color: var(--neon-blue);
            text-transform: uppercase;
        }

        .container { 
            display: grid; 
            grid-template-columns: repeat(auto-fit, minmax(340px, 1fr)); 
            gap: 20px; 
            padding: 20px; 
            max-width: 1400px; 
            margin: 0 auto; 
        }

        .card { 
            background: var(--glass-bg); 
            border: var(--panel-border); 
            border-radius: 10px; 
            padding: 20px; 
            backdrop-filter: blur(10px); 
            position: relative;
            transition: all 0.3s ease;
        }

        .card::before {
            content: '';
            position: absolute;
            top: -1px; left: -1px; width: 20px; height: 20px;
            border-top: 2px solid var(--neon-blue);
            border-left: 2px solid var(--neon-blue);
        }

        .card h2 { 
            margin-top: 0; 
            font-family: 'Orbitron', sans-serif;
            font-size: 14px; 
            color: var(--neon-blue); 
            letter-spacing: 2px; 
            border-bottom: 1px solid rgba(6, 182, 212, 0.2);
            padding-bottom: 10px; 
        }

        /* Artificial Horizon Styling */
        .attitude-container {
            width: 240px; height: 240px;
            margin: 0 auto;
            border-radius: 50%;
            border: 4px solid #334155;
            position: relative;
            overflow: hidden;
            background: #22c55e; /* Ground color backup */
            box-shadow: 0 0 20px rgba(0,0,0,0.5);
        }

        #horizon-ball {
            position: absolute;
            width: 100%; height: 200%;
            top: -50%;
            transition: transform 0.1s linear;
        }

        .sky { height: 50%; background: linear-gradient(to top, #38bdf8, #0369a1); display: flex; align-items: flex-end; justify-content: center; }
        .ground { height: 50%; background: linear-gradient(to bottom, #78350f, #451a03); }
        
        .pitch-lines {
            width: 100%;
            pointer-events: none;
        }
        .pitch-line {
            border-top: 1px solid rgba(255,255,255,0.4);
            margin: 20px auto;
            position: relative;
        }
        .pitch-line::after {
            content: attr(data-deg);
            position: absolute;
            right: 10%; top: -10px;
            font-size: 10px; color: white;
        }

        .fixed-aircraft {
            position: absolute;
            top: 50%; left: 50%;
            transform: translate(-50%, -50%);
            width: 80%; height: 2px;
            background: #facc15;
            z-index: 5;
            box-shadow: 0 0 10px rgba(250, 204, 21, 0.8);
        }
        .fixed-aircraft::before {
            content: '';
            position: absolute;
            left: 50%; top: -5px;
            width: 10px; height: 10px;
            border: 2px solid #facc15;
            transform: translateX(-50%);
        }

        /* Controls (D-Pad) */
        .d-pad { display: grid; grid-template-columns: repeat(3, 1fr); gap: 15px; max-width: 300px; margin: 20px auto; }
        .d-pad button {
            aspect-ratio: 1;
            background: rgba(6, 182, 212, 0.1);
            border: 1px solid var(--neon-blue);
            color: var(--neon-blue);
            border-radius: 8px;
            font-size: 24px;
            cursor: pointer;
            box-shadow: inset 0 0 10px rgba(6,182,212,0.2);
            transition: all 0.1s;
        }
        .d-pad button:active { background: var(--neon-blue); color: #000; transform: scale(0.95); box-shadow: 0 0 20px var(--neon-blue); }

        /* Speed Slider HUD style */
        input[type=range] {
            -webkit-appearance: none;
            width: 100%; background: transparent;
        }
        input[type=range]::-webkit-slider-runnable-track {
            height: 10px; background: rgba(255,255,255,0.1); border-radius: 5px; border: 1px solid var(--neon-blue);
        }
        input[type=range]::-webkit-slider-thumb {
            -webkit-appearance: none;
            height: 25px; width: 15px; background: var(--neon-blue); border-radius: 3px; cursor: pointer; margin-top: -8px; box-shadow: 0 0 10px var(--neon-blue);
        }

        /* Alert Overlay */
        #alert-overlay {
            position: fixed; top: 0; left: 0; width: 100%; height: 100%;
            pointer-events: none; z-index: 10000;
            border: 0px solid transparent;
            transition: all 0.3s;
        }
        .hazard-active #alert-overlay {
            animation: red-pulse 1.5s infinite;
        }
        .hazard-banner {
            position: fixed; top: 80px; left: 50%; transform: translateX(-50%);
            background: #ef4444; color: white; padding: 10px 40px; font-weight: 800;
            letter-spacing: 5px; z-index: 10001; border-radius: 5px;
            display: none; box-shadow: 0 0 20px #ef4444;
            animation: shake 0.5s infinite;
        }

        #map { height: 280px; border-radius: 8px; border: var(--panel-border); filter: grayscale(1) invert(0.9) brightness(0.8) contrast(1.2); }

        .data-grid { display: grid; grid-template-columns: 1fr 1fr; gap: 10px; }
        .data-item { background: rgba(0,0,0,0.4); padding: 15px; border-radius: 8px; border-left: 3px solid var(--neon-blue); }
        .data-label { font-size: 10px; letter-spacing: 1px; color: var(--neon-blue); text-transform: uppercase; }
        .data-value { font-family: 'Orbitron'; font-size: 18px; color: #fff; margin-top: 5px; }

        @keyframes red-pulse { 0% { box-shadow: inset 0 0 0px #ef4444; } 50% { box-shadow: inset 0 0 100px #ef4444; } 100% { box-shadow: inset 0 0 0px #ef4444; } }
        @keyframes shake { 0% { margin-left: 0; } 25% { margin-left: 5px; } 50% { margin-left: 0; } 75% { margin-left: -5px; } 100% { margin-left: 0; } }

        /* Tablet/Desktop Tweaks */
        @media (min-width: 900px) {
            .container { grid-template-columns: 1fr 1.5fr 1fr; }
        }
    </style>
</head>
<body>
    <div id="alert-overlay"></div>
    <div class="hazard-banner" id="danger-ui">⚠ HAZARD DETECTED</div>

    <div class="header">
        <h1>V-CORE ROVER HUD</h1>
    </div>

    <div class="container">
        <!-- AI: Artificial Horizon -->
        <div class="card">
            <h2>Attitude Indicator</h2>
            <div class="attitude-container">
                <div id="horizon-ball">
                    <div class="sky">
                        <div class="pitch-lines">
                            <div class="pitch-line" style="width: 40%;" data-deg="20"></div>
                            <div class="pitch-line" style="width: 60%;" data-deg="10"></div>
                        </div>
                    </div>
                    <div class="ground">
                        <div class="pitch-lines">
                            <div class="pitch-line" style="width: 60%;" data-deg="-10"></div>
                            <div class="pitch-line" style="width: 40%;" data-deg="-20"></div>
                        </div>
                    </div>
                </div>
                <div class="fixed-aircraft"></div>
            </div>
            <div class="data-grid" style="margin-top: 20px;">
                <div class="data-item"><div class="data-label">PITCH</div><div class="data-value" id="pitch-val">0.0&deg;</div></div>
                <div class="data-item"><div class="data-label">ROLL</div><div class="data-value" id="roll-val">0.0&deg;</div></div>
            </div>
            <button onclick="fetch('/start_cal')" style="margin-top: 15px; width: 100%; background: none; border: 1px solid var(--neon-blue); color: var(--neon-blue); padding: 8px; font-family: Orbitron; cursor: pointer;">GYRO RESET</button>
        </div>

        <!-- Drive controls Center -->
        <div class="card">
            <h2 id="gps-status-header">NAV-GRID READY</h2>
            <div id="map"></div>
            
            <div class="d-pad" style="margin-top: 20px;">
                <div></div>
                <button onmousedown="cmd('F')" onmouseup="cmd('S')" ontouchstart="cmd('F');">▲</button>
                <div></div>
                <button onmousedown="cmd('L')" onmouseup="cmd('S')" ontouchstart="cmd('L');">◀</button>
                <button onmousedown="cmd('S')" ontouchstart="cmd('S');" style="color:var(--neon-red); border-color: var(--neon-red);">STOP</button>
                <button onmousedown="cmd('R')" onmouseup="cmd('S')" ontouchstart="cmd('R');">▶</button>
                <div></div>
                <button onmousedown="cmd('B')" onmouseup="cmd('S')" ontouchstart="cmd('B');">▼</button>
                <div></div>
            </div>

            <div style="margin-top: 10px;">
                <div class="data-label">THRUST LEVEL: <span id="speed-val">200</span></div>
                <input type="range" id="speedSlider" min="0" max="255" value="200" onchange="updateSpeed(this.value)" oninput="document.getElementById('speed-val').innerText = this.value">
            </div>
        </div>

        <!-- Systems -->
        <div class="card">
            <div class="data-grid">
                <div class="data-item"><div class="data-label">SATCOM</div><div class="data-value" id="sat-val">0</div></div>
                <div class="data-item"><div class="data-label">CORE TEMP</div><div class="data-value" id="temp-val">0.0&deg;C</div></div>
            </div>

            <div style="display:grid; grid-template-columns: 1fr 1fr; gap: 10px; margin-top: 10px;">
                <div class="data-item"><div class="data-label">LAT</div><div class="data-value" style="font-size:12px" id="lat-val">0.000</div></div>
                <div class="data-item"><div class="data-label">LNG</div><div class="data-value" style="font-size:12px" id="lng-val">0.000</div></div>
            </div>

            <button id="siren-btn" onclick="toggleSiren()" style="margin-top: 15px; width: 100%; background: #000; border: 1px solid var(--neon-red); color: var(--neon-red); padding: 15px; border-radius: 4px; font-weight: bold; cursor: pointer;">ALARM SYSTEM</button>
        </div>
    </div>

    <script>
        // Existing Function Logic preserved
        function cmd(direction) { fetch(`/cmd?dir=${direction}`).catch(console.error); }
        function updateSpeed(val) { fetch(`/speed?val=${val}`).catch(console.error); }
        
        let sirenState = false;
        function toggleSiren() {
            sirenState = !sirenState;
            fetch(`/siren?state=${sirenState ? 'on' : 'off'}`).catch(console.error);
            const btn = document.getElementById('siren-btn');
            btn.style.background = sirenState ? '#ef4444' : '#000';
            btn.style.color = sirenState ? '#fff' : '#ef4444';
        }

        // Map setup
        const map = L.map('map', {zoomControl: false}).setView([0, 0], 2);
        L.tileLayer('https://{s}.basemaps.cartocdn.com/dark_all/{z}/{x}/{y}{r}.png').addTo(map);
        
        const markerIcon = L.divIcon({
            className: 'custom-marker',
            html: `<div style="background:var(--neon-blue);width:12px;height:12px;border-radius:50%;box-shadow:0 0 10px var(--neon-blue);"></div>`,
            iconSize: [12, 12]
        });
        const marker = L.marker([0, 0], {icon: markerIcon}).addTo(map);

        let mapInitialized = false;

        async function fetchData() {
            try {
                const response = await fetch('/data');
                const data = await response.json();
                
                // Numeric Telemetry
                document.getElementById('lat-val').innerText = data.lat.toFixed(5);
                document.getElementById('lng-val').innerText = data.lng.toFixed(5);
                document.getElementById('sat-val').innerText = data.sat;
                document.getElementById('temp-val').innerText = data.temp.toFixed(1) + ' °C';
                document.getElementById('pitch-val').innerHTML = data.pitch.toFixed(1) + '&deg;';
                document.getElementById('roll-val').innerHTML = data.roll.toFixed(1) + '&deg;';

                // Hazard Alert Logic (Threshold 50C - Adjust as needed)
                if(data.temp > 50) {
                    document.body.classList.add('hazard-active');
                    document.getElementById('danger-ui').style.display = 'block';
                } else {
                    document.body.classList.remove('hazard-active');
                    document.getElementById('danger-ui').style.display = 'none';
                }

                // Map/GPS Logic
                const gpsHeader = document.getElementById('gps-status-header');
                if (data.valid) {
                    gpsHeader.innerText = '🛰️ NAV-LINK ESTABLISHED';
                    gpsHeader.style.color = 'var(--neon-green)';
                } else {
                    gpsHeader.innerText = '📡 ACQUIRING SATELLITES...';
                    gpsHeader.style.color = '#94a3b8';
                }
                
                if (data.lat !== 0 || data.lng !== 0) {
                    const latlng = [data.lat, data.lng];
                    marker.setLatLng(latlng);
                    if (!mapInitialized && data.sat > 0) {
                        map.setView(latlng, 17);
                        mapInitialized = true;
                    }
                }

                // ARTIFICIAL HORIZON Logic
                // pitch affects vertical translate, roll affects rotation
                const horizonBall = document.getElementById('horizon-ball');
                const pitchShift = data.pitch * 2; // Adjust multiplier for sensitivity
                horizonBall.style.transform = `rotate(${-data.roll}deg) translateY(${pitchShift}px)`;

            } catch (err) {
                console.error("Telemetry Link Failure", err);
            }
        }

        // Real-time polling
        setInterval(fetchData, 150);
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

  secured_client.setCACert(TELEGRAM_CERTIFICATE_ROOT); // Required for ESP32 Telegram

  calibrateSensors();
  last_micros = micros();
  lastHeadingMicros = micros();

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
    autoMode = false; // Manual override cancels auto return
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


  // ===== AUTO RETURN WEB ROUTES START =====
  server.on("/sethome", HTTP_GET, []() {
    if (gps.location.isValid() && gps.location.age() < 5000) {
      homeLat = gps.location.lat();
      homeLng = gps.location.lng();
      homeSet = true;
      server.send(200, "text/plain", "Home set");
    } else {
      server.send(200, "text/plain", "GPS invalid");
    }
  });

  server.on("/autoon", HTTP_GET, []() {
    if (homeSet) {
      autoMode = true;
      lastHeadingMicros = micros();
      prevGPSSet = false;
      lastGPSCorrection = millis();
      server.send(200, "text/plain", "Auto ON");
    } else {
      server.send(200, "text/plain", "Home not set");
    }
  });

  server.on("/autooff", HTTP_GET, []() {
    autoMode = false;
    stopMotors();
    server.send(200, "text/plain", "Auto OFF");
  });
  // ===== AUTO RETURN WEB ROUTES END =====

  server.on("/data", HTTP_GET, []() {
    String json = "{";
    json += "\"lat\":" + String(gps.location.lat(), 6) + ",";
    json += "\"lng\":" + String(gps.location.lng(), 6) + ",";
    json += "\"sat\":" + String(gps.satellites.value()) + ",";
    json += "\"valid\":" + String(gps.location.isValid() && gps.location.age() < 5000 ? "true" : "false") + ",";
    json += "\"pitch\":" + String(pitch, 2) + ",";
    json += "\"roll\":" + String(roll, 2) + ",";
    json += "\"temp\":" + String(temperature, 2) + ",";
    json += "\"autoMode\":" + String(autoMode ? "true" : "false") + ",";
    json += "\"distanceToHome\":" + String(distanceToHome, 2);
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

  // Telegram Polling
  if (millis() - bot_lasttime > BOT_MTBS) {
    int numNewMessages = bot.getUpdates(bot.last_message_received + 1);
    while (numNewMessages) {
      handleTelegramMessages(numNewMessages);
      numNewMessages = bot.getUpdates(bot.last_message_received + 1);
    }
    bot_lasttime = millis();
  }

  // Live GPS Tracking
  if (isTracking && (millis() - track_lasttime > TRACK_INTERVAL)) {
    if (gps.location.isValid() && gps.location.age() < 5000) {
      String msg = "Live Track: https://maps.google.com/?q=" + String(gps.location.lat(), 6) + "," + String(gps.location.lng(), 6);
      bot.sendMessage(CHAT_ID, msg, "");
    } else {
      bot.sendMessage(CHAT_ID, "❌ Tracking Error: GPS not connected to satellites", "");
    }
    track_lasttime = millis();
  }

  // ===== AUTO RETURN LOOP START =====
  if (autoMode && (millis() - lastAutoMillis >= AUTO_INTERVAL)) {
    lastAutoMillis = millis();
    runAutoReturn();
  }
  // ===== AUTO RETURN LOOP END =====

}

/*
=========================================
      HARDWARE CONNECTION GUIDE
=========================================

1. ESP32 TO MPU6050 (GYRO/ACCEL)
   - VCC  -> 3.3V
   - GND  -> GND
   - SCL  -> D22
   - SDA  -> D21

2. ESP32 TO GPS MODULE (NEO-6M / M8N)
   - VCC  -> 3.3V / 5V (Check module spec)
   - GND  -> GND
   - TX   -> D16 (RX2 on ESP32)
   - RX   -> D17 (TX2 on ESP32)

3. ESP32 TO L298N MOTOR DRIVER
   - ENA  -> D14 (PWM Speed Control Left)
   - IN1  -> D13 (Direction Left)
   - IN2  -> D12 (Direction Left)
   - ENB  -> D25 (PWM Speed Control Right)
   - IN3  -> D27 (Direction Right)
   - IN4  -> D26 (Direction Right)
   * Remember to share GND between ESP32 and L298N!

4. ESP32 TO SIREN/BUZZER
   - VCC/SIG -> D33
   - GND     -> GND
=========================================
*/
