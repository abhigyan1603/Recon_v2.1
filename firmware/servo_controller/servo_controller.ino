// ServoControl - ESP32-S3 + 2x ST3125 (daisy-chained, ID 1 and ID 2)
// Wi-Fi SoftAP + WebSocket (ws://192.168.4.1/ws)
// Libraries: ESPAsyncWebServer (+AsyncTCP), ArduinoJson (6.21+ or 7.x), SCServo (SMS_STS)
//
// ---- Protocol (JSON text frames) ----
// App -> ESP32 :  {"t":"servo","p1":2048,"p2":2048,"spd":1000,"acc":50}   move both servos
//                 {"t":"torque","on":true}                                  torque enable / release
//                 {"t":"ping"}                                              re-scan servos
// ESP32 -> App :  {"t":"tel","s":[{"id":1,"ok":true,"pos":2048,"temp":31,"volt":74,"load":0}, {...}],"tq":true}
//
// ---- How to add a feature ----
// 1) Add a handler function  void cmdXxx(JsonDocument& d)
// 2) Register it in the COMMANDS table below
// 3) Send {"t":"xxx", ...} from the app (App.send in www/index.html)

#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>
#include <SCServo.h>

// ---------------- Wi-Fi SoftAP ----------------
const char* AP_SSID = "ESP32_Robot_AP";
const char* AP_PASS = "Password123";
IPAddress local_ip(192, 168, 4, 1), gateway(192, 168, 4, 1), subnet(255, 255, 255, 0);

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

// ---------------- Pins (unchanged) ----------------
#define SERVO_RX_PIN 18
#define SERVO_TX_PIN 17
#define SERVO_BAUD   1000000

// ---------------- Servos ----------------
const int NUM_SERVOS = 2;
const uint8_t SERVO_IDS[NUM_SERVOS] = {1, 2};   // change if your servos use other IDs
const int POS_MIN = 0, POS_MAX = 4095;          // ST3125: 4096 steps / 360 deg

SMS_STS st;
bool torqueOn = true;
bool servoOk[NUM_SERVOS] = {false, false};

// ---------------- Command handlers ----------------
void cmdServo(JsonDocument& d) {
  if (!torqueOn) return;
  int spd = constrain((int)(d["spd"] | 1000), 50, 3400);
  int acc = constrain((int)(d["acc"] | 50), 0, 254);
  int p[NUM_SERVOS] = { d["p1"] | -1, d["p2"] | -1 };
  for (int i = 0; i < NUM_SERVOS; i++) {
    if (p[i] < 0) continue;                      // field not sent -> leave servo alone
    st.WritePosEx(SERVO_IDS[i], constrain(p[i], POS_MIN, POS_MAX), spd, acc);
  }
}

void cmdTorque(JsonDocument& d) {
  bool on = d["on"] | true;
  if (on && !torqueOn) {
    // hold current position when re-enabling so nothing jumps
    for (int i = 0; i < NUM_SERVOS; i++) {
      int pos = st.ReadPos(SERVO_IDS[i]);
      if (pos >= 0) st.WritePosEx(SERVO_IDS[i], pos, 500, 30);
    }
  }
  for (int i = 0; i < NUM_SERVOS; i++) st.EnableTorque(SERVO_IDS[i], on ? 1 : 0);
  torqueOn = on;
}

void cmdPing(JsonDocument&) {
  for (int i = 0; i < NUM_SERVOS; i++) servoOk[i] = (st.Ping(SERVO_IDS[i]) != -1);
}

// >>> ADD NEW COMMANDS HERE <<<
struct Command { const char* name; void (*fn)(JsonDocument&); };
const Command COMMANDS[] = {
  {"servo",  cmdServo},
  {"torque", cmdTorque},
  {"ping",   cmdPing},
};

void handleMessage(const char* json) {
  StaticJsonDocument<384> doc;
  if (deserializeJson(doc, json)) return;
  const char* t = doc["t"] | "";
  for (const Command& c : COMMANDS) {
    if (strcmp(c.name, t) == 0) { c.fn(doc); return; }
  }
}

void onEvent(AsyncWebSocket*, AsyncWebSocketClient*, AwsEventType type, void* arg, uint8_t* data, size_t len) {
  if (type != WS_EVT_DATA) return;
  AwsFrameInfo* info = (AwsFrameInfo*)arg;
  if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT && len < 384) {
    char buf[384];
    memcpy(buf, data, len);
    buf[len] = 0;
    handleMessage(buf);
  }
}

// ---------------- Telemetry (10 Hz) ----------------
unsigned long lastTel = 0;
void sendTelemetry() {
  StaticJsonDocument<512> doc;
  doc["t"] = "tel";
  doc["tq"] = torqueOn;
  JsonArray arr = doc["s"].to<JsonArray>();
  for (int i = 0; i < NUM_SERVOS; i++) {
    uint8_t id = SERVO_IDS[i];
    int pos = st.ReadPos(id);
    servoOk[i] = (pos >= 0);
    JsonObject o = arr.add<JsonObject>();
    o["id"] = id;
    o["ok"] = servoOk[i];
    if (servoOk[i]) {
      o["pos"]  = pos;
      o["temp"] = st.ReadTemper(id);
      o["volt"] = st.ReadVoltage(id);   // 0.1 V units
      o["load"] = st.ReadLoad(id);
    }
  }
  String out;
  serializeJson(doc, out);
  ws.textAll(out);
}

void setup() {
  Serial.begin(115200);
  delay(500);

  Serial1.begin(SERVO_BAUD, SERIAL_8N1, SERVO_RX_PIN, SERVO_TX_PIN);
  st.pSerial = &Serial1;

  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(local_ip, gateway, subnet);
  if (WiFi.softAP(AP_SSID, AP_PASS, 1, 0, 4)) {
    Serial.print("Wi-Fi AP ready: ");
    Serial.println(WiFi.softAPIP());
  }
  ws.onEvent(onEvent);
  server.addHandler(&ws);
  server.begin();

  for (int i = 0; i < NUM_SERVOS; i++) {
    servoOk[i] = (st.Ping(SERVO_IDS[i]) != -1);
    st.EnableTorque(SERVO_IDS[i], 1);
  }
}

void loop() {
  ws.cleanupClients();
  if (millis() - lastTel >= 100) {
    lastTel = millis();
    if (ws.count() > 0) sendTelemetry();
  }
  // >>> ADD NEW PERIODIC TASKS HERE (sensors, safety checks, ...) <<<
}
