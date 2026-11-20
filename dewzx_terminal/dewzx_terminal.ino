/*
 * ═══════════════════════════════════════════════════════════════
 *  DEWZX AI TERMINAL  (ESP32 DevKit / tanpa kamera)
 *  ESP32 + ST7789 240x240 + BLE Keyboard + Gemini AI
 * ═══════════════════════════════════════════════════════════════
 *  WIRING (pin TFT di-set lewat build flag di build.yml)
 *  TFT GND → GND       TFT VCC → 3V3
 *  TFT SCL/SCK → GPIO 18   TFT SDA/MOSI → GPIO 23
 *  TFT RES → GPIO 17       TFT DC  → GPIO 16
 *  TFT CS  → GPIO 5        TFT BLK → GPIO 21
 * ═══════════════════════════════════════════════════════════════
 */

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <Preferences.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <ArduinoJson.h>
#include <TFT_eSPI.h>
#include <NimBLEDevice.h>

// ─── Backlight Pin ───────────────────────────────────────────
#define TFT_BLK_PIN     21

// ─── Display Constants ────────────────────────────────────────
#define SCREEN_W        240
#define SCREEN_H        240
#define HEADER_H         18
#define FOOTER_H         22
#define CONTENT_H       (SCREEN_H - HEADER_H - FOOTER_H)  // 200
#define LINE_H           10
#define DISPLAY_LINES   (CONTENT_H / LINE_H)              // 20
#define CHARS_PER_LINE   38
#define MAX_BUF_LINES   120

// Colors (RGB565)
#define C_BG      TFT_BLACK
#define C_HEADER  0x1030
#define C_USER    0x07FF
#define C_AI      0x87E0
#define C_STATUS  0xFDA0
#define C_INPUT   TFT_WHITE
#define C_DIM     0x8410
#define C_FOOTER  0x1082

// ─── BLE Constants ────────────────────────────────────────────
#define KB_TARGET_NAME  "Bluetooth Keyboard"
#define KEY_BUF_SIZE    256

// ─── Globals ─────────────────────────────────────────────────
TFT_eSPI    tft;
Preferences prefs;
WebServer   httpServer(80);
DNSServer   dnsServer;

String wifiSSID, wifiPass, geminiKey, geminiModel;

enum AppMode { MODE_CONFIG, MODE_TERMINAL, MODE_BUSY };
volatile AppMode appMode = MODE_TERMINAL;

// Scroll buffer
struct BufLine { String text; uint16_t color; };
BufLine  lineBuf[MAX_BUF_LINES];
int      lineHead  = 0;
int      lineCount = 0;

// Input
String inputLine = "";

// BLE ring buffer
volatile char kbBuf[KEY_BUF_SIZE];
volatile int  kbHead = 0, kbTail = 0;
volatile bool kbConnected      = false;
volatile bool rescanRequested  = false;
volatile bool bleShowPin       = false;
volatile uint32_t bleDisplayPin = 0;

static volatile NimBLEAdvertisedDevice* devToConnect = nullptr;

// ─── HID Keycode Tables ──────────────────────────────────────
static const char HID_NORMAL[57] = {
  0, 0, 0, 0,
  'a','b','c','d','e','f','g','h','i','j',
  'k','l','m','n','o','p','q','r','s','t',
  'u','v','w','x','y','z',
  '1','2','3','4','5','6','7','8','9','0',
  '\n', 0x1b, '\b', '\t', ' ', '-', '=', '[',
  ']', '\\', '#', ';', '\'', '`', ',', '.', '/'
};

static const char HID_SHIFT[57] = {
  0, 0, 0, 0,
  'A','B','C','D','E','F','G','H','I','J',
  'K','L','M','N','O','P','Q','R','S','T',
  'U','V','W','X','Y','Z',
  '!','@','#','$','%','^','&','*','(',')',
  '\n', 0x1b, '\b', '\t', ' ', '_', '+', '{',
  '}', '|', '~', ':', '"', '~', '<', '>', '?'
};

char hidToChar(uint8_t code, bool shift) {
  if (code < 4 || code > 56) return 0;
  return shift ? HID_SHIFT[code] : HID_NORMAL[code];
}

// ─── Key Ring Buffer ─────────────────────────────────────────
void kbPush(char c) {
  int next = (kbHead + 1) % KEY_BUF_SIZE;
  if (next != kbTail) { kbBuf[kbHead] = c; kbHead = next; }
}
bool kbHasKey() { return kbHead != kbTail; }
char kbPop() {
  if (!kbHasKey()) return 0;
  char c = kbBuf[kbTail];
  kbTail = (kbTail + 1) % KEY_BUF_SIZE;
  return c;
}

// ─── HID Report Parser ───────────────────────────────────────
static uint8_t prevKeys[6] = {0};

void processHIDReport(uint8_t* data, size_t len) {
  if (len < 2) return;
  uint8_t modifier = data[0];
  bool    shift    = (modifier & 0x22) != 0;
  uint8_t* keys    = (len > 2) ? data + 2 : nullptr;
  int     keyCount = keys ? (int)min((size_t)6, len - 2) : 0;

  for (int i = 0; i < keyCount; i++) {
    uint8_t k = keys[i];
    if (k == 0) continue;
    bool held = false;
    for (int j = 0; j < 6; j++) {
      if (prevKeys[j] == k) { held = true; break; }
    }
    if (!held) {
      char c = hidToChar(k, shift);
      if (c) kbPush(c);
    }
  }
  if (keys) {
    memset(prevKeys, 0, sizeof(prevKeys));
    memcpy(prevKeys, keys, keyCount);
  }
}

void hidNotifyCB(NimBLERemoteCharacteristic*, uint8_t* data,
                 size_t len, bool) {
  processHIDReport(data, len);
}

// ─── BLE Callbacks ───────────────────────────────────────────
class BLEClientCB : public NimBLEClientCallbacks {
  void onConnect(NimBLEClient*)    override { kbConnected = true; }
  void onDisconnect(NimBLEClient*) override {
    kbConnected     = false;
    rescanRequested = true;
  }
};
static BLEClientCB bleClientCB;

class BLEAdvCB : public NimBLEAdvertisedDeviceCallbacks {
  void onResult(NimBLEAdvertisedDevice* dev) override {
    if (devToConnect) return;
    bool byName = (dev->getName() == KB_TARGET_NAME);
    bool byHID  = dev->isAdvertisingService(NimBLEUUID((uint16_t)0x1812));
    if (byName || byHID) {
      devToConnect = dev;
      NimBLEDevice::getScan()->stop();
    }
  }
};
static BLEAdvCB bleAdvCB;

class BLESecCB : public NimBLESecurityCallbacks {
  bool onConfirmPIN(uint32_t pin) override { return true; }
  void onPassKeyNotify(uint32_t pin) override {
    bleDisplayPin = pin;
    bleShowPin    = true;
  }
  bool onSecurityRequest() override { return true; }
  void onAuthenticationComplete(ble_gap_conn_desc*) override {}
  uint32_t onPassKeyRequest() override { return 0; }
};
static BLESecCB bleSecCB;

static NimBLEClient* activeClient = nullptr;

void connectToKeyboard(NimBLEAdvertisedDevice* dev) {
  if (activeClient) {
    if (activeClient->isConnected()) activeClient->disconnect();
    NimBLEDevice::deleteClient(activeClient);
    activeClient = nullptr;
  }

  activeClient = NimBLEDevice::createClient();
  activeClient->setClientCallbacks(&bleClientCB, false);

  if (!activeClient->connect(dev, false)) {
    NimBLEDevice::deleteClient(activeClient);
    activeClient = nullptr;
    rescanRequested = true;
    return;
  }

  activeClient->secureConnection();

  NimBLERemoteService* hidSvc =
    activeClient->getService(NimBLEUUID((uint16_t)0x1812));
  if (!hidSvc) { activeClient->disconnect(); return; }

  auto* chars = hidSvc->getCharacteristics(true);
  if (!chars) { activeClient->disconnect(); return; }

  for (auto* ch : *chars) {
    if (ch->getUUID() == NimBLEUUID((uint16_t)0x2A4D) && ch->canNotify()) {
      ch->subscribe(true, hidNotifyCB, false);
    }
  }
  kbConnected = true;
}

void startBLEScan() {
  NimBLEScan* scan = NimBLEDevice::getScan();
  scan->setAdvertisedDeviceCallbacks(&bleAdvCB, false);
  scan->setActiveScan(true);
  scan->setInterval(100);
  scan->setWindow(99);
  scan->start(0, nullptr, false);
}

void initBLE() {
  NimBLEDevice::init("DEWZX-Terminal");
  NimBLEDevice::setPower(ESP_PWR_LVL_P9);
  NimBLEDevice::setSecurityAuth(true, false, true);
  NimBLEDevice::setSecurityPasskey(0);
  NimBLEDevice::setSecurityIOCap(BLE_HS_IO_NO_INPUT_OUTPUT);
  NimBLEDevice::setSecurityCallbacks(&bleSecCB);
  startBLEScan();
}

// ─── Display Functions ───────────────────────────────────────
void initDisplay() {
  pinMode(TFT_BLK_PIN, OUTPUT);
  digitalWrite(TFT_BLK_PIN, HIGH);
  tft.init();
  tft.setRotation(0);  // ganti ke 2 jika terbalik
  tft.fillScreen(C_BG);
  tft.setTextWrap(false);
}

void drawHeader(const char* right) {
  tft.fillRect(0, 0, SCREEN_W, HEADER_H, C_HEADER);
  tft.setTextColor(C_STATUS);
  tft.drawString("DEWZX AI", 3, 3, 2);
  tft.setTextDatum(TR_DATUM);
  tft.setTextColor(C_DIM);
  tft.drawString(right, SCREEN_W - 3, 5, 1);
  tft.setTextDatum(TL_DATUM);
}

void addLine(const String& text, uint16_t color) {
  int idx = lineHead % MAX_BUF_LINES;
  lineBuf[idx].text  = text;
  lineBuf[idx].color = color;
  lineHead++;
  if (lineCount < MAX_BUF_LINES) lineCount++;
}

void renderContent() {
  tft.fillRect(0, HEADER_H, SCREEN_W, CONTENT_H, C_BG);
  int first = max(0, lineCount - DISPLAY_LINES);
  int y     = HEADER_H + 1;
  for (int i = first; i < lineCount && y < (SCREEN_H - FOOTER_H); i++) {
    int idx = ((lineHead - lineCount + i) % MAX_BUF_LINES
               + MAX_BUF_LINES) % MAX_BUF_LINES;
    tft.setTextColor(lineBuf[idx].color, C_BG);
    String s = lineBuf[idx].text;
    if ((int)s.length() > CHARS_PER_LINE) s = s.substring(0, CHARS_PER_LINE);
    tft.drawString(s, 1, y, 1);
    y += LINE_H;
  }
}

void renderFooter() {
  static bool     cursorOn  = false;
  static uint32_t lastBlink = 0;
  if (millis() - lastBlink > 500) {
    cursorOn  = !cursorOn;
    lastBlink = millis();
  }
  tft.fillRect(0, SCREEN_H - FOOTER_H, SCREEN_W, FOOTER_H, C_FOOTER);
  tft.setTextColor(C_INPUT);

  String disp = "> " + inputLine;
  if ((int)disp.length() >= CHARS_PER_LINE - 1) {
    int keep = CHARS_PER_LINE - 6;
    disp = ">>" + inputLine.substring(inputLine.length() - keep);
  }
  if (cursorOn) disp += "|";
  tft.drawString(disp, 2, SCREEN_H - FOOTER_H + 6, 1);
}

void fullRedraw(const char* status) {
  drawHeader(status);
  renderContent();
  renderFooter();
}

void addWrapped(const String& src, uint16_t color) {
  int pos = 0;
  while (pos <= (int)src.length()) {
    int    nl  = src.indexOf('\n', pos);
    String seg = (nl < 0) ? src.substring(pos) : src.substring(pos, nl);

    while ((int)seg.length() > CHARS_PER_LINE) {
      int cut = CHARS_PER_LINE;
      while (cut > 0 && seg[cut] != ' ') cut--;
      if (cut == 0) cut = CHARS_PER_LINE;
      addLine(seg.substring(0, cut), color);
      seg = seg.substring(cut + (seg[cut] == ' ' ? 1 : 0));
    }
    if (seg.length() > 0) addLine(seg, color);
    if (nl < 0) break;
    pos = nl + 1;
  }
}

// ─── Config Portal (Captive Portal) ──────────────────────────
static const char CONFIG_HTML[] PROGMEM = R"html(
<!DOCTYPE html><html><head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>DEWZX Setup</title>
<style>
*{box-sizing:border-box}
body{margin:0;padding:24px 16px;background:#070a0f;color:#0ff;font-family:monospace}
h1{color:#0f0;font-size:1.3rem;margin-bottom:1rem}
label{display:block;color:#888;margin:14px 0 4px;font-size:.9rem}
input,select{width:100%;padding:10px;background:#0d1117;color:#0ff;
  border:1px solid #0f0;border-radius:4px;font-family:monospace;font-size:14px}
button{margin-top:24px;width:100%;padding:14px;background:#0f0;color:#000;
  border:none;border-radius:4px;font-size:1rem;font-family:monospace;
  font-weight:bold;cursor:pointer}
button:hover{background:#0ff}
</style></head><body>
<h1>&#11041; DEWZX AI Setup</h1>
<form method="POST" action="/save">
<label>WiFi SSID</label>
<input name="ssid" type="text" placeholder="NamaWiFi" required>
<label>WiFi Password</label>
<input name="pass" type="password" placeholder="********">
<label>Gemini API Key</label>
<input name="key" type="text" placeholder="AIza..." required>
<label>Model Gemini</label>
<select name="model">
  <option value="gemini-2.5-flash">gemini-2.5-flash (cepat)</option>
  <option value="gemini-2.5-flash-lite">gemini-2.5-flash-lite (ringan)</option>
  <option value="gemini-2.5-pro">gemini-2.5-pro (akurat)</option>
</select>
<button type="submit">&#9654; Simpan &amp; Restart</button>
</form></body></html>)html";

static const IPAddress AP_IP(192, 168, 4, 1);

void portalHandleRoot() {
  httpServer.sendHeader("Cache-Control", "no-cache, no-store, must-revalidate");
  httpServer.send_P(200, "text/html", CONFIG_HTML);
}

void portalHandleSave() {
  if (httpServer.hasArg("ssid"))  wifiSSID    = httpServer.arg("ssid");
  if (httpServer.hasArg("pass"))  wifiPass    = httpServer.arg("pass");
  if (httpServer.hasArg("key"))   geminiKey   = httpServer.arg("key");
  if (httpServer.hasArg("model")) geminiModel = httpServer.arg("model");

  prefs.begin("dewzx", false);
  prefs.putString("ssid",  wifiSSID);
  prefs.putString("pass",  wifiPass);
  prefs.putString("key",   geminiKey);
  prefs.putString("model", geminiModel);
  prefs.end();

  httpServer.send(200, "text/html",
    "<html><body style='background:#070a0f;color:#0f0;font-family:monospace;"
    "padding:30px'><h1>Tersimpan!</h1><p>Restart dalam 2 detik...</p></body></html>");
  delay(2000);
  ESP.restart();
}

void portalRedirect() {
  httpServer.sendHeader("Location", "http://192.168.4.1/", true);
  httpServer.sendHeader("Cache-Control", "no-cache, no-store, must-revalidate");
  httpServer.send(302, "text/plain", "");
}

void startConfigPortal() {
  appMode = MODE_CONFIG;

  WiFi.disconnect(true);
  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(AP_IP, AP_IP, IPAddress(255, 255, 255, 0));
  WiFi.softAP("DEWZX-Setup");          // tanpa password -> portal langsung muncul
  delay(300);

  dnsServer.setErrorReplyCode(DNSReplyCode::NoError);
  dnsServer.start(53, "*", AP_IP);

  httpServer.on("/",                          portalHandleRoot);
  httpServer.on("/save", HTTP_POST,           portalHandleSave);
  httpServer.on("/generate_204",              portalRedirect);
  httpServer.on("/gen_204",                   portalRedirect);
  httpServer.on("/hotspot-detect.html",       portalHandleRoot);
  httpServer.on("/library/test/success.html", portalHandleRoot);
  httpServer.on("/connecttest.txt",           portalRedirect);
  httpServer.on("/ncsi.txt",                  portalRedirect);
  httpServer.on("/redirect",                  portalRedirect);
  httpServer.on("/fwlink",                    portalRedirect);
  httpServer.on("/canonical.html",            portalRedirect);
  httpServer.on("/success.txt",               portalRedirect);
  httpServer.onNotFound(portalRedirect);
  httpServer.begin();

  tft.fillScreen(C_BG);
  drawHeader("SETUP");
  tft.setTextColor(C_STATUS);
  tft.drawString("Belum ada WiFi/Key!", 4, 25, 1);
  tft.setTextColor(C_DIM);
  tft.drawString("Hubungkan HP ke WiFi:", 4, 45, 1);
  tft.setTextColor(C_USER);
  tft.drawString("DEWZX-Setup", 4, 60, 2);
  tft.setTextColor(C_DIM);
  tft.drawString("(tanpa password)", 4, 85, 1);
  tft.drawString("Halaman setup terbuka", 4, 100, 1);
  tft.drawString("otomatis di HP!", 4, 112, 1);
  tft.setTextColor(C_AI);
  tft.drawString("192.168.4.1", 4, 130, 2);
}

void loopConfigPortal() {
  dnsServer.processNextRequest();
  httpServer.handleClient();
}

// ─── Gemini API (teks saja) ──────────────────────────────────
String geminiPOST(const uint8_t* body, size_t len) {
  if (WiFi.status() != WL_CONNECTED) return "ERROR: No WiFi";

  WiFiClientSecure sc;
  sc.setInsecure();

  HTTPClient http;
  String url = "https://generativelanguage.googleapis.com/v1beta/models/";
  url += geminiModel;
  url += ":generateContent?key=";
  url += geminiKey;

  http.begin(sc, url);
  http.addHeader("Content-Type", "application/json");
  http.setTimeout(50000);

  int code = http.POST(const_cast<uint8_t*>(body), len);
  if (code != 200) {
    String e = "HTTP " + String(code) + ": " +
               http.getString().substring(0, 80);
    http.end();
    return "ERROR: " + e;
  }

  JsonDocument doc;
  deserializeJson(doc, http.getStream());
  http.end();

  const char* text = doc["candidates"][0]["content"]["parts"][0]["text"];
  if (!text) {
    const char* apiErr = doc["error"]["message"];
    return apiErr ? String("API ERR: ") + apiErr : "ERROR: empty resp";
  }
  return String(text);
}

String jsonEsc(const String& s) {
  String r = s;
  r.replace("\\", "\\\\");
  r.replace("\"", "\\\"");
  r.replace("\n", "\\n");
  r.replace("\r", "");
  return r;
}

String askText(const String& prompt) {
  String body = "{\"contents\":[{\"parts\":[{\"text\":\"";
  body += jsonEsc(prompt);
  body += "\"}]}],\"generationConfig\":{"
          "\"maxOutputTokens\":1024,\"temperature\":0.7}}";
  return geminiPOST((const uint8_t*)body.c_str(), body.length());
}

// ─── Terminal Logic ──────────────────────────────────────────
const char* headerStatus() {
  return kbConnected ? "KB:OK" : "KB:scan";
}

void sendQuery() {
  String query = inputLine;
  inputLine = "";
  appMode   = MODE_BUSY;

  addLine("> " + query, C_USER);
  addLine("... menunggu AI...", C_DIM);
  int thinkIdx = (lineHead - 1 + MAX_BUF_LINES) % MAX_BUF_LINES;
  fullRedraw("BUSY");

  String resp = askText(query);

  lineBuf[thinkIdx].text  = "";
  lineBuf[thinkIdx].color = C_DIM;

  addLine("AI:", C_AI);
  addWrapped(resp, C_AI);
  addLine("--------------------", C_DIM);

  appMode = MODE_TERMINAL;
  fullRedraw(headerStatus());
}

void handleChar(char c) {
  if (appMode == MODE_BUSY) return;

  if (c == '\b') {
    if (inputLine.length() > 0) {
      inputLine.remove(inputLine.length() - 1);
      renderFooter();
    }
    return;
  }

  if (c == '\n' || c == '\r') {
    if (inputLine.length() > 0) sendQuery();
    return;
  }

  if (c >= 32 && c < 127 && (int)inputLine.length() < 200) {
    inputLine += c;
    renderFooter();
  }
}

// ─── Setup ───────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);

  initDisplay();
  tft.setTextColor(C_USER);
  tft.drawString("DEWZX AI", 50, 90, 4);
  tft.setTextColor(C_DIM);
  tft.drawString("booting...", 65, 135, 2);
  delay(400);

  prefs.begin("dewzx", true);
  wifiSSID    = prefs.getString("ssid",  "");
  wifiPass    = prefs.getString("pass",  "");
  geminiKey   = prefs.getString("key",   "");
  geminiModel = prefs.getString("model", "gemini-2.5-flash");
  prefs.end();

  if (wifiSSID.isEmpty() || geminiKey.isEmpty()) {
    startConfigPortal();
    return;
  }

  tft.fillRect(0, 150, SCREEN_W, 90, C_BG);
  tft.setTextColor(C_DIM);
  tft.drawString("WiFi: " + wifiSSID.substring(0, 14), 4, 155, 1);

  WiFi.mode(WIFI_STA);
  WiFi.begin(wifiSSID.c_str(), wifiPass.c_str());
  uint32_t t0 = millis();
  int dotX = 4;
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 15000) {
    delay(400);
    tft.setTextColor(C_STATUS);
    tft.drawString(".", dotX, 168, 1);
    dotX += 7;
    if (dotX > 220) { tft.fillRect(4, 168, 236, 10, C_BG); dotX = 4; }
  }

  if (WiFi.status() != WL_CONNECTED) {
    tft.fillRect(0, 150, SCREEN_W, 90, C_BG);
    tft.setTextColor(C_STATUS);
    tft.drawString("WiFi GAGAL!", 4, 155, 2);
    delay(2500);
    startConfigPortal();
    return;
  }

  tft.fillRect(0, 150, SCREEN_W, 90, C_BG);
  tft.setTextColor(C_AI);
  tft.drawString("WiFi: " + WiFi.localIP().toString(), 4, 155, 1);

  tft.setTextColor(C_DIM);
  tft.drawString("BLE scan...", 4, 170, 1);
  initBLE();

  delay(500);

  appMode   = MODE_TERMINAL;
  lineHead  = 0;
  lineCount = 0;

  addLine("DEWZX AI Terminal v1.0", C_STATUS);
  addLine("IP: " + WiFi.localIP().toString(), C_DIM);
  addLine("Model: " + geminiModel, C_DIM);
  addLine("KB: scan '" KB_TARGET_NAME "'", C_DIM);
  addLine("", C_DIM);
  addLine("Ketik lalu Enter = kirim", C_DIM);
  addLine("", C_DIM);

  fullRedraw(headerStatus());
}

// ─── Loop ────────────────────────────────────────────────────
static bool     lastKBState    = false;
static uint32_t lastCursorTick = 0;

void loop() {
  if (appMode == MODE_CONFIG) {
    loopConfigPortal();
    return;
  }

  if (devToConnect && !kbConnected) {
    NimBLEAdvertisedDevice* dev =
      const_cast<NimBLEAdvertisedDevice*>(devToConnect);
    devToConnect = nullptr;
    connectToKeyboard(dev);
  }

  if (rescanRequested) {
    rescanRequested = false;
    startBLEScan();
    addLine("[KB terputus, scan ulang...]", C_DIM);
    renderContent();
  }

  if (bleShowPin) {
    bleShowPin = false;
    addLine("PIN BLE: " + String(bleDisplayPin), C_STATUS);
    fullRedraw(headerStatus());
  }

  if (kbConnected != lastKBState) {
    lastKBState = kbConnected;
    if (kbConnected) {
      addLine("[Keyboard terhubung!]", C_AI);
      fullRedraw(headerStatus());
    } else {
      drawHeader(headerStatus());
    }
  }

  if (millis() - lastCursorTick > 500 && appMode != MODE_BUSY) {
    lastCursorTick = millis();
    renderFooter();
  }

  while (kbHasKey()) {
    handleChar(kbPop());
  }
}
