#include <WiFi.h>
#include <WebServer.h>
#include <DHT.h>
#include <ESP32Servo.h>
#include <WiFiClientSecure.h>
#include <UniversalTelegramBot.h>
#include <ArduinoJson.h>

// --- Telegram Configuration ---
#define BOT_TOKEN "8710079110:AAFsugljOiBE46NT-lMHPMG_zY0PUwGAWag"
#define CHAT_ID "7901238515"
WiFiClientSecure secured_client;
UniversalTelegramBot bot(BOT_TOKEN, secured_client);

unsigned long bot_lasttime = 0;
const unsigned long BOT_MTBS = 1500; // Check Telegram every 1.5s
unsigned long alert_lasttime = 0;
const unsigned long ALERT_INTERVAL = 1000; // Fast 1-second auto-alert check

// --- Alert System Configuration ---
enum AlertState { STATE_SAFE, STATE_WARNING, STATE_DANGER };

AlertState lastGasState = STATE_SAFE;
unsigned long lastGasAlertTime = 0;

AlertState lastTempState = STATE_SAFE;
unsigned long lastTempAlertTime = 0;

const unsigned long ALERT_COOLDOWN = 30000; // 30 seconds cooldown

// --- Function Prototypes ---
void handleNewMessages(int numNewMessages);
void checkAlerts();
void handleDataUpdate();
float readDustDensity(int &rawVal);
void telegramTask(void *pvParameters);
void updateSensors();

// --- Global Sensor State ---
float current_temp = 0.0;
float current_hum = 0.0;
float current_gas = 0.0;
float current_dust = 0.0;
int current_rawADC = 0;
unsigned long sensor_lasttime = 0;
const unsigned long SENSOR_INTERVAL = 2000; // DHT11 max poll rate is 1Hz (every 2s is safe)

// --- WiFi Credentials (Must be Station mode for Internet/Telegram) ---
const char* ssid = "Swarnendu"; 
const char* password = "12345678"; 

// --- Pin Definitions ---
#define DHTPIN 4
#define DHTTYPE DHT11 
#define MQ2_PIN 34
#define DUST_LED_PIN 5
#define DUST_OUT_PIN 35
#define SERVO_PIN 18

DHT dht(DHTPIN, DHTTYPE);
WebServer server(80);
Servo myServo;

// --- High-End Futuristic Dashboard ---
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Hazard Monitoring System</title>
    <link href="https://fonts.googleapis.com/css2?family=Share+Tech+Mono&display=swap" rel="stylesheet">
    <style>
        :root { --bg: #0a0b0d; --card: #15191d; --cyan: #00f2ff; --green: #39ff14; --red: #ff003c; --font: #e0e6ed; }
        body { font-family: 'Share Tech Mono', monospace; background: var(--bg); color: var(--font); margin: 0; padding: 20px; display: flex; justify-content: center; }
        .container { width: 100%; max-width: 900px; background: var(--card); border: 1px solid #333; box-shadow: 0 0 20px rgba(0,0,0,0.5); }
        .header { padding: 20px; border-bottom: 2px solid #333; display: flex; justify-content: space-between; color: var(--cyan); }
        .grid { padding: 20px; display: grid; grid-template-columns: repeat(auto-fit, minmax(300px, 1fr)); gap: 20px; }
        .card { background: #1a1f24; border: 1px solid #2a3138; padding: 20px; position: relative; transition: all 0.3s; }
        .value { font-size: 2.5rem; font-weight: bold; margin: 10px 0; }
        .unit { font-size: 1rem; color: #808d9a; }
        .gauge { width: 100%; height: 6px; background: #222; margin-top: 10px; }
        .bar { height: 100%; width: 0%; background: var(--cyan); transition: width 1s ease, background 0.5s; }
        .status { font-size: 0.8rem; text-transform: uppercase; color: var(--green); transition: color 0.5s; display: flex; justify-content: space-between; }
        
        .trend { font-weight: bold; transition: color 0.3s; }
        .trend.rising { color: #ffeb3b; animation: pulseRising 1s infinite; }
        .trend.falling { color: var(--green); }
        .trend.stable { color: #808d9a; }
        .trend.danger { color: var(--red); animation: pulseDanger 0.5s infinite; }
        
        @keyframes pulseRising {
            0%, 100% { opacity: 1; text-shadow: 0 0 5px #ffeb3b; }
            50% { opacity: 0.6; text-shadow: 0 0 15px #ffeb3b; }
        }
        @keyframes pulseDanger {
            0%, 100% { opacity: 1; text-shadow: 0 0 10px var(--red); }
            50% { opacity: 0.5; text-shadow: 0 0 25px var(--red); }
        }
        @keyframes shake {
            0%, 100% { transform: translateX(0); }
            25% { transform: translateX(-3px) rotate(-1deg); }
            50% { transform: translateX(3px) rotate(1deg); }
            75% { transform: translateX(-3px) rotate(-1deg); }
        }
        @keyframes dangerGlow {
            0%, 100% { box-shadow: 0 0 15px var(--red), inset 0 0 10px var(--red); border-color: var(--red); background: rgba(255, 0, 60, 0.1); }
            50% { box-shadow: 0 0 35px var(--red), inset 0 0 25px var(--red); border-color: #ff4d4d; background: rgba(255, 0, 60, 0.2); }
        }
        .card.danger { animation: dangerGlow 1s infinite, shake 0.4s infinite; z-index: 10; transform-origin: center; }

        @keyframes flashRed {
            0% { box-shadow: inset 0 0 0px var(--red); }
            50% { box-shadow: inset 0 0 80px var(--red); }
            100% { box-shadow: inset 0 0 0px var(--red); }
        }
        .alert-active { animation: flashRed 1s infinite; }
        .control-panel { background: #1a1f24; border: 1px solid #2a3138; padding: 20px; margin-top: 20px; border-radius: 5px; }
        .btn { background: var(--red); color: #fff; border: none; padding: 10px 20px; font-family: 'Share Tech Mono', monospace; cursor: pointer; font-weight: bold; transition: 0.3s; font-size: 1rem; border-radius: 3px; }
        .btn:hover { background: #ff4d4d; box-shadow: 0 0 15px var(--red); }
        input[type=range] { accent-color: var(--cyan); cursor: pointer; }
    </style>
</head>
<body>
    <div class="container">
        <div class="header">
            <div>SYSTEM STATUS: <span id="main-status">NOMINAL</span></div>
            <div style="font-size: 0.8rem;">DISASTER RESPONSE HUB V3.0</div>
        </div>
        <div class="grid">
            <div class="card" id="card-temp">
                <div class="status">Temperature <span class="trend" id="t-temp">STABLE</span></div>
                <div class="value" id="v-temp">--<span class="unit">°C</span></div>
                <div class="gauge"><div class="bar" id="b-temp"></div></div>
            </div>
            <div class="card" id="card-hum">
                <div class="status">Humidity <span class="trend" id="t-hum">STABLE</span></div>
                <div class="value" id="v-hum">--<span class="unit">%</span></div>
                <div class="gauge"><div class="bar" id="b-hum"></div></div>
            </div>
            <div class="card" id="card-gas">
                <div class="status">Toxic Gas Load <span class="trend" id="t-gas">STABLE</span></div>
                <div class="value" id="v-gas">--<span class="unit">raw</span></div>
                <div class="gauge"><div class="bar" id="b-gas"></div></div>
            </div>
            <div class="card" id="card-dust">
                <div class="status">Dust Density <span class="trend" id="t-dust">STABLE</span></div>
                <div class="value" id="v-dust">--<span class="unit">mg/m³</span></div>
                <div class="gauge"><div class="bar" id="b-dust"></div></div>
                <div style="font-size: 0.7rem; color: #555; margin-top: 5px;">ADC Signal: <span id="v-raw">0</span></div>
            </div>
        </div>
        <div class="control-panel">
            <div style="margin-bottom: 15px; color: var(--cyan); font-weight: bold;">SERVO DEPLOYMENT SYSTEM</div>
            <input type="range" id="servoSlider" min="0" max="180" value="0" style="width: 100%;">
            <div style="display: flex; justify-content: space-between; margin-top: 10px; align-items: center;">
                <span>Current Angle: <span id="servoAngle" style="color: var(--cyan); font-size: 1.2rem; font-weight: bold;">0</span>&deg;</span>
                <button id="dropBtn" class="btn">DROP DOWN</button>
            </div>
        </div>
    </div>
    <script>
        let prevData = null;
        setInterval(() => {
            fetch('/data').then(res => res.json()).then(data => {
                if (!prevData) prevData = data;
                
                const updateCard = (id, val, max, unit, isDanger, prevVal, customStatus = null) => {
                    document.getElementById('v-' + id).innerHTML = val + '<span class="unit">' + unit + '</span>';
                    document.getElementById('b-' + id).style.width = Math.min(100, (val / max * 100)) + '%';
                    
                    const cardEl = document.getElementById('card-' + id);
                    const trendEl = document.getElementById('t-' + id);
                    const barEl = document.getElementById('b-' + id);
                    
                    let trendStr = 'STABLE';
                    let trendClass = 'stable';
                    
                    if (isDanger) {
                        trendStr = customStatus ? customStatus : 'DANGER CRITICAL!';
                        trendClass = 'danger';
                        cardEl.classList.add('danger');
                        barEl.style.background = 'var(--red)';
                    } else {
                        cardEl.classList.remove('danger');
                        barEl.style.background = 'var(--cyan)';
                        if (customStatus) {
                            trendStr = customStatus;
                            trendClass = (val > prevVal) ? 'rising' : ((val < prevVal) ? 'falling' : 'stable');
                        } else if (val > prevVal) {
                            trendStr = 'RISING ▲';
                            trendClass = 'rising';
                        } else if (val < prevVal) {
                            trendStr = 'FALLING ▼';
                            trendClass = 'falling';
                        } else {
                            trendStr = 'STABLE ⬌';
                            trendClass = 'stable';
                        }
                    }
                    
                    trendEl.innerText = trendStr;
                    trendEl.className = 'trend ' + trendClass;
                    return isDanger;
                };

                let tempRisingFast = (data.temp - prevData.temp) > 5;
                let isAlertTemp = data.temp > 45 || tempRisingFast || (data.temp > 40 && tempRisingFast);
                let isAlertHum = data.hum < 25 || data.hum > 80;
                let isAlertGas = data.gas > 1000;
                let isAlertDust = data.dust >= 0.4;

                let tempStatus = null;
                if (data.temp > 40 && tempRisingFast) tempStatus = "STRONG FIRE DETECTION";
                else if (data.temp > 45) tempStatus = "FIRE ALERT";
                else if (tempRisingFast) tempStatus = "RAPID HEAT ALERT 🔥";

                let humStatus = null;
                if (isAlertHum) humStatus = "ENVIRONMENT ALERT";

                let gasStatus = "clean air (varies!)";
                if (data.gas > 1200) gasStatus = "very high concentration";
                else if (data.gas > 1000) gasStatus = "significant gas/smoke";
                else if (data.gas > 900) gasStatus = "slight gas presence";

                updateCard('temp', data.temp, 50, '°C', isAlertTemp, prevData.temp, tempStatus);
                updateCard('hum', data.hum, 100, '%', isAlertHum, prevData.hum, humStatus);
                updateCard('gas', data.gas, 4095, 'raw', isAlertGas, prevData.gas, gasStatus);
                updateCard('dust', data.dust, 0.5, 'mg/m³', isAlertDust, prevData.dust);
                
                document.getElementById('v-raw').innerText = data.raw;
                
                let alertMsg = [];
                if (tempStatus) alertMsg.push(tempStatus);
                if (humStatus) alertMsg.push(humStatus);
                if (data.gas > 1200) alertMsg.push("GAS: LEAK DETECTED");
                else if (data.gas > 1000) alertMsg.push("GAS: SIGNIFICANT");
                if (isAlertDust) alertMsg.push("HIGH DUST");
                
                let isAlert = alertMsg.length > 0;
                
                const statusEl = document.getElementById('main-status');
                if (isAlert) {
                    statusEl.innerText = 'CRITICAL ALERT (' + alertMsg.join(', ') + ')';
                    statusEl.style.color = 'var(--red)';
                    document.body.classList.add('alert-active');
                } else {
                    statusEl.innerText = 'NOMINAL';
                    statusEl.style.color = 'var(--cyan)';
                    document.body.classList.remove('alert-active');
                }
                
                prevData = data;
            });
        }, 1500);

        const servoSlider = document.getElementById('servoSlider');
        const servoAngleText = document.getElementById('servoAngle');
        const dropBtn = document.getElementById('dropBtn');

        // Throttle function to prevent overwhelming the ESP32
        let servoTimeout = null;
        servoSlider.addEventListener('input', (e) => {
            let angle = e.target.value;
            servoAngleText.innerText = angle;
            
            if (servoTimeout) clearTimeout(servoTimeout);
            servoTimeout = setTimeout(() => {
                fetch('/servo?angle=' + angle);
            }, 50); // Send update at most every 50ms for smooth movement
        });

        dropBtn.addEventListener('click', () => {
            let angle = 180;
            servoSlider.value = angle;
            servoAngleText.innerText = angle;
            fetch('/servo?angle=' + angle);
        });
    </script>
</body>
</html>
)rawliteral";

void setup() {
  Serial.begin(115200);
  
  // ADC Calibration for Dust and Gas
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db); 

  dht.begin();
  pinMode(DUST_LED_PIN, OUTPUT);
  digitalWrite(DUST_LED_PIN, HIGH); // Default OFF for Sharp Sensor
  pinMode(MQ2_PIN, INPUT);
  
  // Servo Initialization
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  myServo.setPeriodHertz(50); // Standard 50Hz servo
  myServo.attach(SERVO_PIN, 500, 2400); // Standard min/max pulse width for MG90S
  myServo.write(0); // Start at 0 degrees
  
  // Connect to WiFi (Station Mode required for Telegram API)
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected. IP: ");
  Serial.println(WiFi.localIP());

  // Telegram secure client setup
  // By using setInsecure(), we BYPASS the need for NTP time sync completely!
  // This makes Telegram connect instantly (0 seconds delay).
  secured_client.setInsecure();

  server.on("/", HTTP_GET, []() {
    server.send(200, "text/html", index_html);
  });

  server.on("/data", HTTP_GET, handleDataUpdate);

  server.on("/servo", HTTP_GET, []() {
    server.sendHeader("Access-Control-Allow-Origin", "*");
    if (server.hasArg("angle")) {
      int angle = server.arg("angle").toInt();
      if (angle >= 0 && angle <= 180) {
        myServo.write(angle);
        Serial.print("Servo Drop System moved to: ");
        Serial.println(angle);
      }
    }
    server.send(200, "text/plain", "OK");
  });

  server.begin();

  // Run Telegram in a separate task so it doesn't block WebServer
  xTaskCreatePinnedToCore(
    telegramTask,
    "TelegramTask",
    8192,
    NULL,
    1,
    NULL,
    0 // Run on Core 0 (Loop runs on Core 1)
  );
}

void loop() {
  server.handleClient();

  // Read sensors globally every 2 seconds to prevent locking the HTTP server
  if (millis() - sensor_lasttime > SENSOR_INTERVAL) {
    updateSensors();
    sensor_lasttime = millis();
  }
}

// --- FreeRTOS Task for Telegram ---
void telegramTask(void *pvParameters) {
  Serial.println("\nTelegram Task Started. Bot is online instantly.");

  for (;;) {
    if (millis() - bot_lasttime > BOT_MTBS) {
      int numNewMessages = bot.getUpdates(bot.last_message_received + 1);
      while (numNewMessages) {
        handleNewMessages(numNewMessages);
        numNewMessages = bot.getUpdates(bot.last_message_received + 1);
      }
      bot_lasttime = millis();
    }

    // Auto Alerts check moved here to prevent concurrent WiFiClientSecure access
    if (millis() - alert_lasttime > ALERT_INTERVAL) {
      checkAlerts();
      alert_lasttime = millis();
    }

    vTaskDelay(pdMS_TO_TICKS(100)); // Yield to other tasks
  }
}

// --- Telegram Logic ---
// Message handling: Processes incoming Telegram messages and button commands
void handleNewMessages(int numNewMessages) {
  for (int i = 0; i < numNewMessages; i++) {
    String chat_id = String(bot.messages[i].chat_id);
    if (chat_id != CHAT_ID) {
      bot.sendMessage(chat_id, "Unauthorized user", "");
      continue;
    }
    String text = bot.messages[i].text;
    String from_name = bot.messages[i].from_name;

    if (text == "/start") {
      String welcome = "🚨 Disaster Monitoring System Ready\n";
      
      // Button creation: Setup Keyboard Layout using ReplyKeyboardMarkup (Not Inline)
      String keyboardJson = "[[\"🔥 Check Gas\", \"🌡 Check Temp\"],[\"💧 Check Humidity\"]]";
      bot.sendMessageWithReplyKeyboard(chat_id, welcome, "", keyboardJson, true);
    } 
    else if (text == "🔥 Check Gas") {
      String status = "SAFE";
      if (current_gas > 1200) status = "DANGER";
      else if (current_gas > 1000) status = "WARNING";
      else if (current_gas > 900) status = "ELEVATED";
      
      // Send response with status
      String msg = "Gas Level: " + String((int)current_gas) + "\nStatus: " + status;
      bot.sendMessage(chat_id, msg, "");
    }
    else if (text == "🌡 Check Temp") {
      bot.sendMessage(chat_id, "Temperature: " + String(current_temp, 1) + " °C", "");
    }
    else if (text == "💧 Check Humidity") {
      bot.sendMessage(chat_id, "Humidity: " + String(current_hum, 1) + " %", "");
    }
  }
}

// Alert system: State-based logic with cooldown to prevent spam
void checkAlerts() {
  unsigned long now = millis();

  // --- GAS ALERT LOGIC ---
  AlertState currentGasState = lastGasState; // Hysteresis to prevent bouncing
  if (current_gas > 1200) {
    currentGasState = STATE_DANGER;
  } else if (current_gas > 1000) {
    currentGasState = STATE_WARNING;
  } else if (current_gas < 900) {
    currentGasState = STATE_SAFE;
  }

  // Only trigger on state changes and enforce cooldown
  if (currentGasState != lastGasState) {
    if (now - lastGasAlertTime >= ALERT_COOLDOWN || lastGasAlertTime == 0) {
      String msg = "";
      if (currentGasState == STATE_DANGER) {
        msg = "🚨 GAS LEAK DETECTED! Level: " + String((int)current_gas);
      } else if (currentGasState == STATE_WARNING) {
        msg = "⚠️ Gas level rising: " + String((int)current_gas);
      } else if (currentGasState == STATE_SAFE) {
        msg = "✅ Gas level back to normal";
      }
      
      bot.sendMessage(CHAT_ID, msg, "");
      lastGasState = currentGasState;
      lastGasAlertTime = now;
    }
  }

  // --- TEMP ALERT LOGIC ---
  AlertState currentTempState = STATE_SAFE;
  if (current_temp > 35.0) {
    currentTempState = STATE_DANGER;
  }

  if (currentTempState != lastTempState) {
    if (now - lastTempAlertTime >= ALERT_COOLDOWN || lastTempAlertTime == 0) {
      String msg = "";
      if (currentTempState == STATE_DANGER) {
        msg = "🔥 HIGH TEMPERATURE ALERT!";
      } else if (currentTempState == STATE_SAFE) {
        msg = "✅ Temperature back to normal";
      }
      
      bot.sendMessage(CHAT_ID, msg, "");
      lastTempState = currentTempState;
      lastTempAlertTime = now;
    }
  }
}

// Separate function for the tricky Dust Sensor timing
float readDustDensity(int &rawVal) {
  float totalVo = 0;
  int iterations = 10;

  for (int i = 0; i < iterations; i++) {
    digitalWrite(DUST_LED_PIN, LOW); // LED ON
    delayMicroseconds(280);
    int vo = analogRead(DUST_OUT_PIN);
    delayMicroseconds(40);
    digitalWrite(DUST_LED_PIN, HIGH); // LED OFF
    delayMicroseconds(9680);
    totalVo += vo;
  }

  rawVal = totalVo / iterations;
  float voltage = rawVal * (3.3 / 4095.0);
  
  // Linear Equation: Change 0.1 to 0.0 if reading is always 0.00
  float density = 0.17 * voltage - 0.1;
  if (density < 0) density = 0;
  return density;
}

void updateSensors() {
  float t = dht.readTemperature();
  float h = dht.readHumidity();
  if (!isnan(t)) current_temp = t;
  if (!isnan(h)) current_hum = h;

  // Rapid burst averaging to eliminate ADC noise INSTANTLY without delaying alerts
  long gas_sum = 0;
  for(int i = 0; i < 5; i++) {
    gas_sum += analogRead(MQ2_PIN);
    delay(5);
  }
  current_gas = gas_sum / 5.0;

  current_dust = readDustDensity(current_rawADC);
}

void handleDataUpdate() {
  String json = "{";
  json += "\"temp\":" + String(current_temp, 1) + ",";
  json += "\"hum\":" + String(current_hum, 1) + ",";
  json += "\"gas\":" + String((int)current_gas) + ",";
  json += "\"dust\":" + String(current_dust, 3) + ",";
  json += "\"raw\":" + String(current_rawADC);
  json += "}";

  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send(200, "application/json", json);
}
