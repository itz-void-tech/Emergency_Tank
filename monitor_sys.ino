#include <WiFi.h>
#include <WebServer.h>
#include <DHT.h>

// --- Function Prototypes ---
void handleDataUpdate();
float readDustDensity();

// --- AP Mode Credentials ---
const char* ssid = "Hazard-Tank-AP"; 
const char* password = "rescueadmin"; 

// --- Pin Definitions ---
#define DHTPIN 4
#define DHTTYPE DHT11 
#define MQ2_PIN 34
#define DUST_LED_PIN 5
#define DUST_OUT_PIN 35

DHT dht(DHTPIN, DHTTYPE);
WebServer server(80);

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
        .card { background: #1a1f24; border: 1px solid #2a3138; padding: 20px; position: relative; }
        .value { font-size: 2.5rem; font-weight: bold; margin: 10px 0; }
        .unit { font-size: 1rem; color: #808d9a; }
        .gauge { width: 100%; height: 6px; background: #222; margin-top: 10px; }
        .bar { height: 100%; width: 0%; background: var(--cyan); transition: width 1s ease; }
        .status { font-size: 0.8rem; text-transform: uppercase; color: var(--green); }
    </style>
</head>
<body>
    <div class="container">
        <div class="header">
            <div>SYSTEM STATUS: <span id="main-status">NOMINAL</span></div>
            <div style="font-size: 0.8rem;">DISASTER RESPONSE HUB V3.0</div>
        </div>
        <div class="grid">
            <div class="card">
                <div class="status">Temperature</div>
                <div class="value" id="v-temp">--<span class="unit">°C</span></div>
                <div class="gauge"><div class="bar" id="b-temp"></div></div>
            </div>
            <div class="card">
                <div class="status">Humidity</div>
                <div class="value" id="v-hum">--<span class="unit">%</span></div>
                <div class="gauge"><div class="bar" id="b-hum"></div></div>
            </div>
            <div class="card">
                <div class="status">Toxic Gas Load</div>
                <div class="value" id="v-gas">--<span class="unit">raw</span></div>
                <div class="gauge"><div class="bar" id="b-gas"></div></div>
            </div>
            <div class="card">
                <div class="status">Dust Density</div>
                <div class="value" id="v-dust">--<span class="unit">mg/m³</span></div>
                <div class="gauge"><div class="bar" id="b-dust"></div></div>
                <div style="font-size: 0.7rem; color: #555; margin-top: 5px;">ADC Signal: <span id="v-raw">0</span></div>
            </div>
        </div>
    </div>
    <script>
        setInterval(() => {
            fetch('/data').then(res => res.json()).then(data => {
                document.getElementById('v-temp').innerHTML = data.temp + '<span class="unit">°C</span>';
                document.getElementById('b-temp').style.width = (data.temp / 50 * 100) + '%';
                
                document.getElementById('v-hum').innerHTML = data.hum + '<span class="unit">%</span>';
                document.getElementById('b-hum').style.width = data.hum + '%';
                
                document.getElementById('v-gas').innerHTML = data.gas + '<span class="unit">raw</span>';
                document.getElementById('b-gas').style.width = (data.gas / 4095 * 100) + '%';
                
                document.getElementById('v-dust').innerHTML = data.dust + '<span class="unit">mg/m³</span>';
                document.getElementById('b-dust').style.width = Math.min(100, (data.dust / 0.5 * 100)) + '%';
                document.getElementById('v-raw').innerText = data.raw;
            });
        }, 1500);
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
  
  WiFi.softAP(ssid, password);
  Serial.println("Access Point Started");
  Serial.println(WiFi.softAPIP());

  server.on("/", HTTP_GET, []() {
    server.send(200, "text/html", index_html);
  });

  server.on("/data", HTTP_GET, handleDataUpdate);
  server.begin();
}

void loop() {
  server.handleClient();
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

void handleDataUpdate() {
  float t = dht.readTemperature();
  float h = dht.readHumidity();
  if (isnan(t)) t = 0.0;
  if (isnan(h)) h = 0.0;

  int gas_raw = analogRead(MQ2_PIN);
  
  int rawADC = 0;
  float d = readDustDensity(rawADC);

  String json = "{";
  json += "\"temp\":" + String(t, 1) + ",";
  json += "\"hum\":" + String(h, 1) + ",";
  json += "\"gas\":" + String(gas_raw) + ",";
  json += "\"dust\":" + String(d, 3) + ",";
  json += "\"raw\":" + String(rawADC);
  json += "}";

  server.send(200, "application/json", json);
}
