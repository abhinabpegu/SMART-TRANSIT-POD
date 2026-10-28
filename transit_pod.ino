/*
 * Smart Cargo Transit Pod  |  Team Ghee refilled  |  PS 20
 * Remote asset geofencing + shock-aware offline logger (ESP32)
 *
 * WHAT IT DOES
 *   - Reads the MPU6050 about 200 times a second and computes the total g-force vector.
 *   - Reads the NEO-6M GPS and computes the Haversine distance from a stored "fence centre".
 *   - On a SHOCK (> 3.0 g), FREE-FALL (< 0.25 g) or FENCE BREACH (> 100 m) it:
 *       1) saves an event to ESP32 flash (Preferences / NVS)  -> no SD card needed
 *       2) turns on the buzzer + red LED until acknowledged
 *   - Timestamps come from GPS time (the ESP32 has no real-time clock).
 *   - Shows live status on the 0.96" OLED and sends telemetry over BLE.
 *
 * BUTTONS (both wired to GND, INPUT_PULLUP)
 *   BTN1 (GPIO32): set fence centre to the current GPS position
 *   BTN2 (GPIO33): short press = acknowledge alarm, hold 2 s = clear logs and max shock
 *
 * LIBRARIES (Arduino Library Manager)
 *   TinyGPSPlus (Mikal Hart)       - NMEA parsing
 *   Adafruit MPU6050               - accelerometer (also installs Adafruit Unified Sensor + BusIO)
 *   Adafruit SSD1306 + Adafruit GFX - OLED
 *   Built in with the ESP32 core: Wire, Preferences, HardwareSerial, BLE (BLEDevice etc.)
 *   Board package: "esp32" by Espressif, board = "ESP32 Dev Module"
 *
 * KNOWN LIMITS
 *   - A very short spike that lands while the OLED is redrawing (~25 ms) can be missed.
 *   - GPS needs open sky; the first fix can take minutes indoors.
 *   - No compass: the MPU6050 gives acceleration only.
 */

#include <Wire.h>
#include <Preferences.h>
#include <TinyGPSPlus.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// Set to 0 if you want to build without BLE (for example while debugging).
#define USE_BLE 1
#if USE_BLE
  #include <BLEDevice.h>
  #include <BLEServer.h>
  #include <BLEUtils.h>
  #include <BLE2902.h>
#endif

// ======================== PIN MAP (matches the wiring diagram) ========================
#define PIN_SDA        21   // I2C data  (MPU6050 + OLED share the bus)
#define PIN_SCL        22   // I2C clock
#define PIN_GPS_RX     16   // ESP32 RX2  <- GPS TX
#define PIN_GPS_TX     17   // ESP32 TX2  -> GPS RX
#define PIN_BUZZER     25   // active buzzer (+)
#define PIN_LED_ALERT  26   // red LED   (through 220 ohm)
#define PIN_LED_STATUS 27   // green LED (through 220 ohm)
#define PIN_BTN_SET    32   // BTN1: set fence centre
#define PIN_BTN_RESET  33   // BTN2: acknowledge / clear

// ======================== THRESHOLDS (same values as the slides) ========================
const float    SHOCK_G           = 3.0f;    // total g above this = shock
const float    FREEFALL_G        = 0.25f;   // total g below this = free-fall
const double   FENCE_M           = 100.0;   // geofence radius in metres
const uint8_t  BREACH_CONFIRM    = 3;       // GPS updates in a row outside the fence before alarming (stops GPS jitter)
const uint8_t  FREEFALL_SAMPLES  = 6;       // consecutive low samples (~30 ms) before calling it free-fall
const float    MAX_HDOP          = 5.0f;    // ignore GPS fixes worse than this

// ======================== TIMING ========================
const unsigned long IMU_PERIOD_MS   = 5;     // 200 Hz IMU sampling
const unsigned long OLED_PERIOD_MS  = 250;   // 4 Hz screen refresh
const unsigned long SHOCK_HOLDOFF   = 1000;  // ms between logged shocks (one impact = one log)
const unsigned long MSG_SHOW_MS     = 2000;  // how long short messages (ACK, FENCE SET) stay on screen

// ======================== FLASH LOG ========================
const uint8_t MAX_LOGS = 20;                // ring buffer: newest entry overwrites the oldest
enum EventType : uint8_t { EVT_SHOCK = 1, EVT_FREEFALL = 2, EVT_FENCE = 3 };

struct LogEntry {
  uint8_t  type;      // EventType
  float    value;     // g-force for shock/free-fall, distance in m for fence breach
  uint32_t date;      // GPS date as ddmmyy
  uint32_t time;      // GPS time as hhmmss (UTC)
  double   lat;       // position when the event happened
  double   lon;
};

// ======================== OBJECTS ========================
Adafruit_SSD1306 display(128, 64, &Wire, -1);
Adafruit_MPU6050 mpu;
TinyGPSPlus      gps;
HardwareSerial   gpsSerial(2);   // UART2
Preferences      prefs;          // flash storage (NVS)

// ======================== STATE ========================
bool     originSet   = false;    // has a fence centre been stored?
double   originLat   = 0, originLon = 0;
float    maxG        = 0;        // highest g seen since last clear
float    liveG       = 1.0f;     // latest g reading (shown on OLED)
double   lastDist    = -1;       // latest distance from centre, metres (-1 = unknown)
uint32_t totalLogs   = 0;        // events ever saved (ring buffer index = totalLogs % MAX_LOGS)
bool     alarmActive = false;    // buzzer + red LED on until acknowledged
bool     breachActive = false;   // currently outside the fence (alarm already raised)
uint8_t  outsideCount = 0;       // consecutive GPS updates outside the fence
uint32_t lastFlashUs = 0;        // how long the last flash write took (microseconds)
char     msgText[16] = "";       // short status message for the OLED
unsigned long msgUntil = 0;

// ======================== SMALL HELPERS ========================
void showMsg(const char* m) {
  strncpy(msgText, m, sizeof(msgText) - 1);
  msgText[sizeof(msgText) - 1] = 0;
  msgUntil = millis() + MSG_SHOW_MS;
}

bool gpsHasFix() {
  // valid position, recent, and good enough accuracy
  if (!gps.location.isValid() || gps.location.age() > 3000) return false;
  if (gps.hdop.isValid() && gps.hdop.hdop() > MAX_HDOP) return false;
  return true;
}

// Haversine: great-circle distance between two lat/lon points, in metres.
double haversineM(double lat1, double lon1, double lat2, double lon2) {
  const double R = 6371000.0;                       // mean Earth radius in metres
  double p1 = radians(lat1), p2 = radians(lat2);
  double dp = radians(lat2 - lat1);
  double dl = radians(lon2 - lon1);
  double a = sin(dp / 2) * sin(dp / 2) + cos(p1) * cos(p2) * sin(dl / 2) * sin(dl / 2);
  return 2.0 * R * atan2(sqrt(a), sqrt(1.0 - a));
}

// ======================== FLASH LOGGING ========================
// Writes one event to flash and raises the alarm. Called only when something happens,
// so flash is not written continuously (saves flash wear).
void saveEvent(uint8_t type, float value) {
  LogEntry e = {};
  e.type  = type;
  e.value = value;
  if (gps.date.isValid()) e.date = gps.date.value();          // ddmmyy
  if (gps.time.isValid()) e.time = gps.time.value() / 100;    // hhmmsscc -> hhmmss
  if (gps.location.isValid()) { e.lat = gps.location.lat(); e.lon = gps.location.lng(); }

  uint32_t t0 = micros();
  char key[8];
  snprintf(key, sizeof(key), "e%u", (unsigned)(totalLogs % MAX_LOGS));
  prefs.putBytes(key, &e, sizeof(e));          // the entry itself
  totalLogs++;
  prefs.putUInt("total", totalLogs);           // new count
  lastFlashUs = micros() - t0;                 // measured flash write time (use this for the test table)

  alarmActive = true;
  Serial.printf("EVENT type=%u value=%.2f time=%06lu flash_write=%lu us\n",
                type, value, (unsigned long)e.time, (unsigned long)lastFlashUs);
}

void clearLogs() {
  totalLogs = 0;
  maxG = 0;
  prefs.putUInt("total", 0);      // old entries are ignored because the count is zero
  prefs.putFloat("maxg", 0);
  alarmActive = false;
  breachActive = false;
  outsideCount = 0;
}

// ======================== IMU: SHOCK + FREE-FALL ========================
void readImu() {
  static unsigned long lastShockMs = 0;
  static uint8_t lowCount = 0;
  static bool inFreeFall = false;

  sensors_event_t a, g, t;
  mpu.getEvent(&a, &g, &t);

  // Total g-force vector magnitude: sqrt(ax^2 + ay^2 + az^2), converted from m/s^2 to g.
  float gForce = sqrtf(a.acceleration.x * a.acceleration.x +
                       a.acceleration.y * a.acceleration.y +
                       a.acceleration.z * a.acceleration.z) / 9.80665f;
  liveG = gForce;
  if (gForce > maxG) maxG = gForce;

  // SHOCK: one log per impact (hold-off stops the same impact logging many times)
  if (gForce >= SHOCK_G && millis() - lastShockMs > SHOCK_HOLDOFF) {
    lastShockMs = millis();
    prefs.putFloat("maxg", maxG);
    saveEvent(EVT_SHOCK, gForce);
  }

  // FREE-FALL: g stays near zero for a few samples, so one noisy reading cannot trigger it
  if (gForce < FREEFALL_G) {
    if (lowCount < 255) lowCount++;
    if (lowCount >= FREEFALL_SAMPLES && !inFreeFall) {
      inFreeFall = true;
      saveEvent(EVT_FREEFALL, gForce);
    }
  } else {
    lowCount = 0;
    if (gForce > 0.5f) inFreeFall = false;    // re-arm once the pod is no longer falling
  }
}

// ======================== GPS: GEOFENCE ========================
void updateFence() {
  if (!gps.location.isUpdated()) return;      // only run when a new GPS fix arrived
  if (!originSet || !gpsHasFix()) return;

  lastDist = haversineM(originLat, originLon, gps.location.lat(), gps.location.lng());

  if (lastDist > FENCE_M) {
    if (outsideCount < 255) outsideCount++;
    if (outsideCount >= BREACH_CONFIRM && !breachActive) {   // confirmed breach, log it once
      breachActive = true;
      saveEvent(EVT_FENCE, (float)lastDist);
    }
  } else {
    outsideCount = 0;
    breachActive = false;                      // back inside, re-arm
  }
}

// ======================== BUTTONS ========================
void onSetFence() {
  if (gpsHasFix()) {
    originLat = gps.location.lat();
    originLon = gps.location.lng();
    originSet = true;
    breachActive = false;
    outsideCount = 0;
    lastDist = 0;
    prefs.putDouble("oLat", originLat);        // survives power cycles
    prefs.putDouble("oLon", originLon);
    prefs.putBool("hasO", true);
    showMsg("FENCE SET");
  } else {
    showMsg("NO GPS FIX");
  }
}

void handleButtons() {
  static bool b1 = false, b2 = false;
  static unsigned long t1 = 0, t2 = 0, b2Start = 0;
  unsigned long now = millis();
  bool n1 = digitalRead(PIN_BTN_SET)   == LOW;   // pressed = LOW (INPUT_PULLUP)
  bool n2 = digitalRead(PIN_BTN_RESET) == LOW;

  // simple 40 ms debounce
  if (n1 != b1 && now - t1 > 40) {
    t1 = now; b1 = n1;
    if (b1) onSetFence();
  }
  if (n2 != b2 && now - t2 > 40) {
    t2 = now; b2 = n2;
    if (b2) {
      b2Start = now;                            // pressed: start timing
    } else if (now - b2Start >= 2000) {         // released after a long press: clear everything
      clearLogs();
      showMsg("LOGS CLEARED");
    } else {                                    // released after a short press: acknowledge
      alarmActive = false;
      showMsg("ACK");
    }
  }
}

// ======================== OUTPUTS: BUZZER + LEDS ========================
void updateAlarmOutputs() {
  static unsigned long tBeep = 0;
  static bool beepOn = false;

  if (!alarmActive) {                           // no alarm: everything off
    digitalWrite(PIN_BUZZER, LOW);
    digitalWrite(PIN_LED_ALERT, LOW);
    beepOn = false;
  } else {                                      // alarm: red LED steady, buzzer beeps (non-blocking)
    digitalWrite(PIN_LED_ALERT, HIGH);
    if (millis() - tBeep >= 150) {
      tBeep = millis();
      beepOn = !beepOn;
      digitalWrite(PIN_BUZZER, beepOn);
    }
  }

  // green status LED: steady = GPS fix, blinking = still searching
  if (gpsHasFix()) digitalWrite(PIN_LED_STATUS, HIGH);
  else             digitalWrite(PIN_LED_STATUS, (millis() / 500) % 2);
}

// ======================== OLED ========================
void drawOLED() {
  char buf[24];
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  // line 1: GPS status
  display.setCursor(0, 0);
  display.print(gpsHasFix() ? "GPS: LOCKED" : "GPS: SEARCHING");

  // line 2: fence status (inverted bar when breached)
  const char* fenceTxt = !originSet ? "FENCE: NOT SET" : (breachActive ? "FENCE: BREACH" : "FENCE: OK");
  if (breachActive) {
    display.fillRect(0, 10, 128, 10, SSD1306_WHITE);
    display.setTextColor(SSD1306_BLACK);
  }
  display.setCursor(2, 11);
  display.print(fenceTxt);
  display.setTextColor(SSD1306_WHITE);

  // line 3: distance from fence centre
  if (originSet && lastDist >= 0) snprintf(buf, sizeof(buf), "DIST: %.0fm / %.0fm", lastDist, FENCE_M);
  else                            snprintf(buf, sizeof(buf), "DIST: -- / %.0fm", FENCE_M);
  display.setCursor(0, 23);
  display.print(buf);

  // line 4: highest shock and pass/fail against the 3 g limit
  snprintf(buf, sizeof(buf), "MAX SHOCK: %.1fg %s", maxG, maxG >= SHOCK_G ? "FAIL" : "OK");
  display.setCursor(0, 34);
  display.print(buf);

  // line 5: events stored in flash
  uint32_t shown = totalLogs > MAX_LOGS ? MAX_LOGS : totalLogs;
  snprintf(buf, sizeof(buf), "LOGS: %02lu IN FLASH", (unsigned long)shown);
  display.setCursor(0, 45);
  display.print(buf);

  // line 6: short message, otherwise the live g reading
  display.setCursor(0, 56);
  if (millis() < msgUntil) display.print(msgText);
  else { snprintf(buf, sizeof(buf), "NOW: %.2fg", liveG); display.print(buf); }

  display.display();
}

// ======================== BLE TELEMETRY (optional) ========================
#if USE_BLE
#define SERVICE_UUID "7a1c0001-5e2b-4c3a-9d1e-0f6a8b2c4d10"
#define CHAR_UUID    "7a1c0002-5e2b-4c3a-9d1e-0f6a8b2c4d10"

BLECharacteristic* pChar = nullptr;
bool bleConnected = false;
volatile bool dumpRequested = false;

class ServerCb : public BLEServerCallbacks {
  void onConnect(BLEServer*) override { bleConnected = true; }
  void onDisconnect(BLEServer* s) override {
    bleConnected = false;
    s->getAdvertising()->start();               // keep advertising so the phone can reconnect
  }
};

// A phone writes the text "DUMP" to ask for the stored log.
class CharCb : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* c) override {
    auto v = c->getValue();
    if (v == "DUMP") dumpRequested = true;
  }
};

void bleSend(const char* s) {                   // default BLE payload is 20 bytes, keep lines short
  pChar->setValue((uint8_t*)s, strlen(s));
  pChar->notify();
}

void bleInit() {
  BLEDevice::init("GheeRefilled-Pod");
  BLEServer* server = BLEDevice::createServer();
  server->setCallbacks(new ServerCb());
  BLEService* svc = server->createService(SERVICE_UUID);
  pChar = svc->createCharacteristic(CHAR_UUID,
            BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_WRITE);
  pChar->addDescriptor(new BLE2902());          // lets the phone subscribe to notifications
  pChar->setCallbacks(new CharCb());
  svc->start();
  server->getAdvertising()->addServiceUUID(SERVICE_UUID);
  server->getAdvertising()->start();
}

void bleLoop() {
  static unsigned long tStatus = 0;
  if (!bleConnected) return;

  // once a second: short status line, e.g. "D142 G3.2 L3 F1"
  if (millis() - tStatus >= 1000) {
    tStatus = millis();
    char buf[24];
    snprintf(buf, sizeof(buf), "D%.0f G%.1f L%lu F%d", lastDist < 0 ? 0 : lastDist, maxG,
             (unsigned long)(totalLogs > MAX_LOGS ? MAX_LOGS : totalLogs), breachActive ? 1 : 0);
    bleSend(buf);
  }

  // on request: send every stored event, oldest first, two short lines each
  if (dumpRequested) {
    dumpRequested = false;
    uint32_t count = totalLogs > MAX_LOGS ? MAX_LOGS : totalLogs;
    uint32_t start = totalLogs > MAX_LOGS ? totalLogs % MAX_LOGS : 0;
    for (uint32_t i = 0; i < count; i++) {
      char key[8];
      snprintf(key, sizeof(key), "e%u", (unsigned)((start + i) % MAX_LOGS));
      LogEntry e;
      if (prefs.getBytes(key, &e, sizeof(e)) != sizeof(e)) continue;
      char buf[24];
      const char* tn = e.type == EVT_SHOCK ? "S" : (e.type == EVT_FREEFALL ? "F" : "B");   // Shock / Fall / Breach
      snprintf(buf, sizeof(buf), "%lu,%s,%.1f,%06lu", (unsigned long)(i + 1), tn, e.value, (unsigned long)e.time);
      bleSend(buf); delay(40);
      snprintf(buf, sizeof(buf), "%.4f,%.4f", e.lat, e.lon);
      bleSend(buf); delay(40);
    }
    bleSend("END");
  }
}
#endif

// ======================== SETUP ========================
void setup() {
  Serial.begin(115200);

  pinMode(PIN_BUZZER, OUTPUT);
  pinMode(PIN_LED_ALERT, OUTPUT);
  pinMode(PIN_LED_STATUS, OUTPUT);
  pinMode(PIN_BTN_SET, INPUT_PULLUP);
  pinMode(PIN_BTN_RESET, INPUT_PULLUP);

  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(400000);                        // fast I2C keeps the OLED redraw short

  // OLED (I2C 0x3C)
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("OLED not found");
    while (true) { digitalWrite(PIN_LED_ALERT, !digitalRead(PIN_LED_ALERT)); delay(200); }
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("Transit Pod booting");
  display.display();

  // MPU6050 (I2C 0x68): +/-16 g range so a 3 g shock is well inside the range
  if (!mpu.begin(0x68)) {
    display.println("MPU6050 NOT FOUND");
    display.display();
    while (true) { digitalWrite(PIN_LED_ALERT, !digitalRead(PIN_LED_ALERT)); delay(200); }
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
  mpu.setFilterBandwidth(MPU6050_BAND_260_HZ);  // widest filter so short spikes are not smoothed away

  // GPS on UART2, 9600 baud
  gpsSerial.setRxBufferSize(1024);              // big buffer so no NMEA bytes are lost while the OLED redraws
  gpsSerial.begin(9600, SERIAL_8N1, PIN_GPS_RX, PIN_GPS_TX);

  // Load saved state from flash (survives power cycles)
  prefs.begin("pod", false);
  totalLogs = prefs.getUInt("total", 0);
  maxG      = prefs.getFloat("maxg", 0);
  originSet = prefs.getBool("hasO", false);
  if (originSet) {
    originLat = prefs.getDouble("oLat", 0);
    originLon = prefs.getDouble("oLon", 0);
  }

#if USE_BLE
  bleInit();
#endif

  // short beep = ready
  digitalWrite(PIN_BUZZER, HIGH); delay(100); digitalWrite(PIN_BUZZER, LOW);
  Serial.println("Pod ready");
}

// ======================== MAIN LOOP ========================
void loop() {
  // 1) feed every GPS byte to the NMEA parser
  while (gpsSerial.available()) gps.encode(gpsSerial.read());

  // 2) buttons
  handleButtons();

  // 3) IMU at 200 Hz
  static unsigned long tImu = 0;
  if (millis() - tImu >= IMU_PERIOD_MS) { tImu = millis(); readImu(); }

  // 4) geofence check whenever a new GPS fix arrives
  updateFence();

  // 5) buzzer + LEDs
  updateAlarmOutputs();

  // 6) screen at 4 Hz
  static unsigned long tOled = 0;
  if (millis() - tOled >= OLED_PERIOD_MS) { tOled = millis(); drawOLED(); }

#if USE_BLE
  bleLoop();
#endif
}
