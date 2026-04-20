#include <WiFi.h>
#include <WebServer.h>
#include <DHT.h>

// --- AP Mode Credentials ---
// This is the Wi-Fi network your tank will broadcast. 
const char* ssid = "Hazard-Tank-AP"; 
const char* password = "rescueadmin"; // Must be at least 8 characters

// --- Pin Definitions ---
#define DHTPIN 4
#define DHTTYPE DHT11 // Change to DHT22 if you are using the white sensor
#define MQ2_PIN 34
#define DUST_LED_PIN 5
#define DUST_OUT_PIN 35

DHT dht(DHTPIN, DHTTYPE);
WebServer server(80);

// --- High-End, Futuristic HTML Dashboard (Stored in Program Memory) ---
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Environmental Hazard Dashboard</title>
    <link href="https://fonts.googleapis.com/css2?family=Share+Tech+Mono&display=swap" rel="stylesheet">
    <style>
        :root {
            --bg-dark: #0a0b0d;
            --bg-card: #15191d;
            --neon-cyan: #00f2ff;
            --neon-green: #39ff14;
            --neon-yellow: #ffde00;
            --neon-red: #ff003c;
            --border-faint: rgba(255, 255, 255, 0.1);
            --font-main: #e0e6ed;
            --font-faint: #808d9a;
            --shadow-glow: 0 0 15px rgba(0, 242, 255, 0.2);
        }

        * { box-sizing: border-box; }

        body {
            font-family: 'Share Tech Mono', monospace;
            background-color: var(--bg-dark);
            color: var(--font-main);
            margin: 0;
            padding: 20px;
            display: flex;
            justify-content: center;
            align-items: center;
            min-height: 100vh;
            overflow-x: hidden;
            position: relative;
        }

        /* Cyber Background Effects */
        body::before {
            content: "";
            position: fixed;
            top: 0; left: 0; width: 100%; height: 100%;
            background: 
                linear-gradient(rgba(18, 16, 16, 0) 50%, rgba(0, 0, 0, 0.1) 50%),
                linear-gradient(90deg, rgba(255, 0, 0, 0.03), rgba(0, 255, 0, 0.01), rgba(0, 0, 255, 0.03));
            background-size: 100% 4px, 3px 100%;
            z-index: -1;
            pointer-events: none;
        }

        .grid-bg {
            position: fixed;
            top: 0; left: 0; width: 100%; height: 100%;
            background-image: 
                linear-gradient(var(--border-faint) 1px, transparent 1px),
                linear-gradient(90deg, var(--border-faint) 1px, transparent 1px);
            background-size: 50px 50px;
            z-index: -2;
            opacity: 0.3;
        }

        .scanline {
            width: 100%; height: 100px;
            z-index: -1;
            background: linear-gradient(0deg, rgba(0, 242, 255, 0) 0%, rgba(0, 242, 255, 0.05) 50%, rgba(0, 242, 255, 0) 100%);
            position: fixed;
            opacity: 0.5;
            animation: scan 8s linear infinite;
        }

        @keyframes scan {
            0% { top: -100px; }
            100% { top: 100%; }
        }

        /* Layout Container */
        .dashboard-container {
            width: 100%;
            max-width: 900px;
            background-color: var(--bg-card);
            border: 1px solid #333;
            border-radius: 4px;
            box-shadow: 0 0 40px rgba(0, 0, 0, 0.8);
            overflow: hidden;
            position: relative;
            z-index: 10;
            backdrop-filter: blur(5px);
        }

        .header {
            padding: 20px 25px;
            background: linear-gradient(to bottom, rgba(255,255,255,0.05), transparent);
            border-bottom: 2px solid #333;
            display: flex;
            justify-content: space-between;
            align-items: center;
        }

        .header h1 {
            margin: 0;
            font-size: 1.3rem;
            text-transform: uppercase;
            letter-spacing: 3px;
            color: var(--neon-cyan);
            text-shadow: 0 0 10px rgba(0, 242, 255, 0.5);
        }

        .main-status-banner {
            padding: 12px 25px;
            border-bottom: 1px solid #333;
            font-weight: bold;
            text-transform: uppercase;
            letter-spacing: 2px;
            color: var(--neon-cyan);
            background: rgba(0, 242, 255, 0.05);
            transition: all 0.5s ease;
        }

        /* Grid System */
        .grid-container {
            padding: 25px;
            display: grid;
            grid-template-columns: repeat(auto-fit, minmax(350px, 1fr));
            gap: 20px;
        }

        /* Enhanced Card Design */
        .card {
            background: linear-gradient(145deg, #1a1f24, #121519);
            border: 1px solid #2a3138;
            padding: 25px;
            position: relative;
            transition: transform 0.2s cubic-bezier(0.175, 0.885, 0.32, 1.275), box-shadow 0.3s ease, border-color 0.3s ease;
        }

        .card::before { /* Decorative corner */
            content: "";
            position: absolute;
            top: 0; left: 0; width: 10px; height: 10px;
            border-top: 2px solid var(--neon-cyan);
            border-left: 2px solid var(--neon-cyan);
            opacity: 0.5;
        }

        .card:hover {
            transform: scale(1.02);
            border-color: #444;
            box-shadow: 0 10px 20px rgba(0,0,0,0.4);
        }

        .card-header {
            display: flex;
            justify-content: space-between;
            align-items: center;
            margin-bottom: 20px;
        }

        .icon-container {
            display: flex;
            align-items: center;
            gap: 12px;
        }

        .icon {
            width: 22px; height: 22px;
            fill: var(--neon-cyan);
            filter: drop-shadow(0 0 5px var(--neon-cyan));
        }

        .label {
            text-transform: uppercase;
            font-size: 0.85rem;
            color: var(--font-faint);
            letter-spacing: 1px;
        }

        /* Dynamic Status Pills */
        .status-pill {
            padding: 4px 12px;
            border-radius: 2px;
            font-size: 0.7rem;
            font-weight: bold;
            text-transform: uppercase;
            letter-spacing: 1px;
            border: 1px solid transparent;
            transition: all 0.3s ease;
        }

        .status-pill.safe { 
            color: var(--neon-green); 
            border-color: var(--neon-green);
            box-shadow: inset 0 0 5px rgba(57, 255, 20, 0.2);
        }
        .status-pill.warning { 
            color: var(--neon-yellow); 
            border-color: var(--neon-yellow); 
            animation: flicker 1.5s infinite;
        }
        .status-pill.danger { 
            color: white; 
            background: var(--neon-red); 
            animation: pulse-danger-pill 0.8s infinite alternate;
        }

        /* Readout Value Enhancement */
        .value-readout {
            font-size: 2.6rem;
            font-weight: 900;
            color: var(--font-main);
            margin-bottom: 15px;
            transition: all 0.3s ease;
            text-shadow: 2px 2px 0px rgba(0,0,0,0.5);
        }
        
        .value-pulse {
            color: var(--neon-cyan);
            text-shadow: 0 0 15px var(--neon-cyan);
            transform: translateY(-2px);
        }

        .unit {
            font-size: 0.9rem;
            color: var(--font-faint);
            font-weight: normal;
        }

        /* Dynamic Gauge Enhancement */
        .gauge-container {
            width: 100%;
            height: 4px;
            background-color: rgba(255,255,255,0.05);
            overflow: visible;
            position: relative;
        }

        .gauge-bar {
            height: 100%;
            width: 0%; 
            transition: width 1.2s cubic-bezier(0.23, 1, 0.32, 1); 
            position: relative;
            box-shadow: 0 0 10px rgba(255,255,255,0.1);
        }

        .gauge-bar.safe { background: var(--neon-green); }
        .gauge-bar.warning { background: var(--neon-yellow); box-shadow: 0 0 10px var(--neon-yellow); }
        .gauge-bar.danger { background: var(--neon-red); box-shadow: 0 0 15px var(--neon-red); }

        /* Status Visual Modes */
        .hazard-detected .card { border-color: rgba(255, 222, 0, 0.3); }
        .critical-alert { animation: shake 0.5s infinite; }
        .critical-alert .dashboard-container { border-color: var(--neon-red); box-shadow: 0 0 50px rgba(255, 0, 60, 0.4); }
        .critical-alert .main-status-banner { 
            background: var(--neon-red); 
            color: white;
            animation: strobing 0.5s infinite alternate;
        }

        /* Keyframes */
        @keyframes flicker {
            0%, 100% { opacity: 1; }
            50% { opacity: 0.6; }
        }

        @keyframes pulse-danger-pill {
            from { box-shadow: 0 0 5px var(--neon-red); }
            to { box-shadow: 0 0 15px var(--neon-red); }
        }

        @keyframes shake {
            0% { transform: translate(1px, 1px) rotate(0deg); }
            10% { transform: translate(-1px, -2px) rotate(-1deg); }
            30% { transform: translate(3px, 2px) rotate(0deg); }
            50% { transform: translate(-1px, 2px) rotate(1deg); }
            80% { transform: translate(1px, -1px) rotate(-1deg); }
            100% { transform: translate(1px, -2px) rotate(0deg); }
        }

        @keyframes strobing {
            from { background: var(--neon-red); opacity: 1; }
            to { background: #80001e; opacity: 0.8; }
        }

        @keyframes loadIn {
            from { opacity: 0; transform: translateY(10px); }
            to { opacity: 1; transform: translateY(0); }
        }

        .dashboard-container { animation: loadIn 0.8s ease-out; }
    </style>
</head>
<body class="system-stable">
    <div class="grid-bg"></div>
    <div class="scanline"></div>

    <div class="dashboard-container">
        <div class="header">
            <h1>Monitoring Panel</h1>
            <div class="header-status">
                <span id="overall-status-pill" class="status-pill safe">System Online</span>
            </div>
        </div>
        
        <div id="main-status-banner" class="main-status-banner">Monitoring environmental biometrics...</div>

        <div class="grid-container">
            <!-- TEMP -->
            <div class="card" id="card-temp">
                <div class="card-header">
                    <div class="icon-container">
                        <svg class="icon" viewBox="0 0 24 24"><path d="M15 13V5c0-1.66-1.34-3-3-3S9 3.34 9 5v8c-1.21.91-2 2.37-2 4 0 2.76 2.24 5 5 5s5-2.24 5-5c0-1.63-.79-3.09-2-4zm-4-2V5c0-.55.45-1 1-1s1 .45 1 1v6h-2z"/></svg>
                        <span class="label">Atmospheric Temp</span>
                    </div>
                    <span id="status-temp-pill" class="status-pill safe">Safe</span>
                </div>
                <div class="value-readout" id="value-temp">-- <span class="unit">&deg;C</span></div>
                <div class="gauge-container"><div class="gauge-bar safe" id="gauge-temp"></div></div>
            </div>

            <!-- HUM -->
            <div class="card" id="card-hum">
                <div class="card-header">
                    <div class="icon-container">
                        <svg class="icon" viewBox="0 0 24 24"><path d="M12 21.5c-4.14 0-7.5-3.36-7.5-7.5 0-3.9 4.14-9.33 6.64-12.18.45-.52 1.27-.52 1.72 0 2.5 2.85 6.64 8.28 6.64 12.18 0 4.14-3.36 7.5-7.5 7.5z"/></svg>
                        <span class="label">Relative Humidity</span>
                    </div>
                    <span id="status-hum-pill" class="status-pill safe">Safe</span>
                </div>
                <div class="value-readout" id="value-hum">-- <span class="unit">%</span></div>
                <div class="gauge-container"><div class="gauge-bar safe" id="gauge-hum"></div></div>
            </div>

            <!-- GAS -->
            <div class="card" id="card-gas">
                <div class="card-header">
                    <div class="icon-container">
                        <svg class="icon" viewBox="0 0 24 24"><path d="M12 2L4.5 20.29l.71.71L12 18l6.79 3 .71-.71L12 2z"/></svg>
                        <span class="label">Toxic Gas Load</span>
                    </div>
                    <span id="status-gas-pill" class="status-pill safe">Safe</span>
                </div>
                <div class="value-readout" id="value-gas">-- <span class="unit">ppm</span></div>
                <div class="gauge-container"><div class="gauge-bar safe" id="gauge-gas"></div></div>
            </div>

            <!-- DUST -->
            <div class="card" id="card-dust">
                <div class="card-header">
                    <div class="icon-container">
                        <svg class="icon" viewBox="0 0 24 24"><path d="M17.66 8L12 2.35 6.34 8C4.78 9.56 4 11.64 4 13.64s.78 4.11 2.34 5.67 3.61 2.34 5.66 2.34c2.05 0 4.1-.78 5.66-2.34S20 15.64 20 13.64s-.78-4.08-2.34-5.64zM12 19c-2.97 0-5.4-2.42-5.4-5.4 0-1.49.6-2.84 1.58-3.83L12 6.1l3.82 3.67c.98.99 1.58 2.34 1.58 3.83 0 2.98-2.43 5.4-5.4 5.4z"/></svg>
                        <span class="label">Particle Density</span>
                    </div>
                    <span id="status-dust-pill" class="status-pill safe">Safe</span>
                </div>
                <div class="value-readout" id="value-dust">-- <span class="unit">&micro;g/m&sup3;</span></div>
                <div class="gauge-container"><div class="gauge-bar safe" id="gauge-dust"></div></div>
            </div>

        </div>
    </div>

    <script>
        let prevValues = { temp: 0, hum: 0, gas: 0, dust: 0 };

        setInterval(function ( ) {
            fetch('/data').then(response => response.json()).then(data => {
                
                let highestStatus = 0; 

                // Process specific levels
                let sTemp = getStatusLevel(data.temp, 45, 60);
                let sHum = getStatusLevel(data.hum, 80, 95);
                let sGas = getStatusLevel(data.gas, 1000, 2500); 
                let sDust = getStatusLevel(data.dust * 1000, 50, 150); 
                
                highestStatus = Math.max(sTemp, sHum, sGas, sDust);

                // UI Updates
                updateMetric("temp", data.temp, prevValues.temp, 45, 60, sTemp);
                updateMetric("hum", data.hum, prevValues.hum, 80, 95, sHum);
                updateMetric("gas", data.gas, prevValues.gas, 1000, 2500, sGas);
                updateMetric("dust", data.dust * 1000, prevValues.dust * 1000, 50, 150, sDust); 
                
                prevValues.temp = data.temp;
                prevValues.hum = data.hum;
                prevValues.gas = data.gas;
                prevValues.dust = data.dust;

                updateGlobalStatus(highestStatus);
            }).catch(e => console.log("ESP disconnected. Static UI Mode."));
        }, 1500);

        function getStatusLevel(val, min, max) {
            if (val >= max) return 2;
            if (val >= min) return 1;
            return 0;
        }

        function updateMetric(id, newVal, prevVal, min, max, valStatus) {
            const readoutEl = document.getElementById("value-" + id);
            const pillEl = document.getElementById("status-" + id + "-pill");
            const gaugeBarEl = document.getElementById("gauge-" + id);

            readoutEl.innerHTML = newVal.toFixed(1) + ' <span class="unit">' + getUnit(id) + '</span>';

            // High-impact glow update logic
            if(Math.abs(newVal - prevVal) > 0.1) { 
                readoutEl.classList.add("value-pulse");
                setTimeout(() => readoutEl.classList.remove("value-pulse"), 500);
            }

            let newStatusClass = "safe";
            let newStatusText = "Safe";

            if (valStatus === 2) {
                newStatusClass = "danger";
                newStatusText = getDangerText(id);
            } else if (valStatus === 1) {
                newStatusClass = "warning";
                newStatusText = "Warning";
            }
            
            pillEl.className = "status-pill " + newStatusClass;
            pillEl.innerText = newStatusText;
            gaugeBarEl.className = "gauge-bar " + newStatusClass;

            // Normalized Gauge Width (40% start to simulate industrial headroom)
            let displayMin = min * 0.3; 
            let displayMax = max * 1.3; 
            let mappedPct = ((newVal - displayMin) / (displayMax - displayMin)) * 100;
            mappedPct = Math.max(2, Math.min(100, mappedPct));
            gaugeBarEl.style.width = mappedPct + "%";
        }

        function updateGlobalStatus(highestStatus) {
            let bodyEl = document.body;
            let bannerEl = document.getElementById("main-status-banner");
            let pillEl = document.getElementById("overall-status-pill");

            if (highestStatus === 2) {
                bodyEl.className = "critical-alert";
                bannerEl.innerText = "CRITICAL FAILURE - EVACUATE ZONE";
                pillEl.className = "status-pill danger";
                pillEl.innerText = "IMMEDIATE EVAC";
            } else if (highestStatus === 1) {
                bodyEl.className = "hazard-detected";
                bannerEl.innerText = "CAUTION: SUB-OPTIMAL BIOMETRICS";
                pillEl.className = "status-pill warning";
                pillEl.innerText = "HAZARD DETECTED";
            } else {
                bodyEl.className = "system-stable";
                bannerEl.innerText = "Monitoring: Nominal Operations";
                pillEl.className = "status-pill safe";
                pillEl.innerText = "System Online";
            }
        }

        function getUnit(id) {
            const units = { temp: "&deg;C", hum: "%", gas: "ppm", dust: "&micro;g/m&sup3;" };
            return units[id] || "";
        }

        function getDangerText(id) {
            const dangerTexts = { temp: "EXCESS HEAT", hum: "OVER-SATURATION", gas: "LETHAL LEVEL", dust: "BIOHAZARD" };
            return dangerTexts[id] || "DANGER";
        }
    </script>
</body>
</html>
)rawliteral";

// --- Functions ---

void setup() {
  Serial.begin(115200);
  dht.begin();
  pinMode(DUST_LED_PIN, OUTPUT);
  pinMode(MQ2_PIN, INPUT);
  
  // --- START ACCESS POINT MODE ---
  Serial.println("\n--- Initializing Disaster Response Hub ---");
  Serial.print("Starting Access Point: ");
  Serial.println(ssid);
  
  // Create the Wi-Fi network
  WiFi.softAP(ssid, password);

  // Get the IP address of the ESP32 AP (Usually 192.168.4.1)
  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP Address: ");
  Serial.println(IP);

  // Web Server Routing
  server.on("/", HTTP_GET, []() {
    server.send(200, "text/html", index_html);
  });

  server.on("/data", HTTP_GET, handleDataUpdate);
  server.begin();
  Serial.println("HTTP server started and broadcasting.");
}

void loop() {
  server.handleClient();
}

void handleDataUpdate() {
  // Read DHT for both Temp and Humidity
  float t = dht.readTemperature();
  float h = dht.readHumidity();
  
  if (isnan(t)) t = 0.0;
  if (isnan(h)) h = 0.0;

  // Read MQ-2 (Raw ADC value 0-4095)
  int mq2_raw = analogRead(MQ2_PIN);

  // Read Optical Dust Sensor
  digitalWrite(DUST_LED_PIN, LOW);
  delayMicroseconds(280);
  int dust_vo = analogRead(DUST_OUT_PIN); 
  delayMicroseconds(40);
  digitalWrite(DUST_LED_PIN, HIGH);
  delayMicroseconds(9680);

  // Calculate Dust Density
  float calcVoltage = dust_vo * (3.3 / 4095.0); 
  float dustDensity = 0.17 * calcVoltage - 0.1;
  if (dustDensity < 0) dustDensity = 0.00;

  // Build JSON String
  String json = "{";
  json += "\"temp\":" + String(t, 1) + ",";
  json += "\"hum\":" + String(h, 1) + ",";
  json += "\"gas\":" + String(mq2_raw) + ",";
  json += "\"dust\":" + String(dustDensity, 3);
  json += "}";

  server.send(200, "application/json", json);
}
