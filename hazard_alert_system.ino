/*
  ESP32 HAZARD ALERT SYSTEM  (v2)
  --------------------------------------------------------------
  - ALL tunable values live in section 1 (USER CONFIG). Edit only there.
  - Every sensor is judged on its own (OK / WARNING / DANGER) with
    debounce + hysteresis, so single noisy readings never trigger alarms.
  - Smart fusion rules escalate to DANGER when sensors agree
    (smoke+CO, smoke+heat, vibration+gas).
  - Dashboard + OLED show WHICH sensor raised the alert and WHY.
  - MQ gas sensors auto-calibrate their clean-air baseline after warm-up.
*/

#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_BMP280.h>
#include <DHT.h>
#include "driver/gpio.h"
#include <math.h>

// =====================================================================
// 1. USER CONFIG  (change values here only)
// =====================================================================

// ---------- Wi-Fi ----------
const char* WIFI_SSID     = "SHASANE";
const char* WIFI_PASSWORD = "Shasane30891";
const char* AP_SSID       = "ESP32-Hazard-System";   // backup hotspot
const char* AP_PASSWORD   = "12345678";
const char* MDNS_NAME     = "hazard";                // http://hazard.local

// ---------- Pins ----------
#define I2C_SDA         21
#define I2C_SCL         22
#define PIN_MQ2         34
#define PIN_MQ4         35
#define PIN_MQ7         36
#define PIN_LDR         39
#define PIN_SW420       32
#define PIN_FLAME       33
#define PIN_DHT         4
#define PIN_LED_RED     13
#define PIN_LED_YELLOW  23
#define PIN_LED_GREEN   15
#define PIN_BUZZER_1    16
#define PIN_BUZZER_2    17

#define LED_ACTIVE_HIGH      true   // false if LEDs are wired active-LOW
#define FLAME_DETECTED_STATE LOW    // most flame modules go LOW on fire
#define VIB_DETECTED_STATE   HIGH   // SW-420 goes HIGH on vibration

// ---------- MQ gas sensors ----------
// Alert = (raw ADC) >= (clean-air baseline + DELTA).
// Delta thresholds work on every module, because each one has a different
// clean-air reading. Set DELTA = old absolute threshold and BASELINE = 0
// with MQ_AUTO_BASELINE false to get the old "fixed number" behaviour.
#define MQ_AUTO_BASELINE   true   // measure clean-air baseline after warm-up
#define MQ_WARMUP_S        60     // gas alerts are blocked during warm-up (use 180+ for real use)
#define MQ_CALIB_S         10     // last N seconds of warm-up are averaged as baseline
#define MQ_BASELINE_MAX    2500   // if measured baseline is higher (gas at boot), use fixed value
#define MQ_ADC_CEILING     4050   // thresholds are clamped below ADC saturation

// Fixed baselines (used if auto-baseline is off or invalid)
#define MQ2_FIXED_BASELINE 1000
#define MQ4_FIXED_BASELINE 1000
#define MQ7_FIXED_BASELINE 1000

// Rise above baseline (ADC counts) -> WARNING / DANGER
#define MQ2_WARN_DELTA     600    // smoke / LPG
#define MQ2_DANGER_DELTA   1400
#define MQ4_WARN_DELTA     600    // methane / CNG
#define MQ4_DANGER_DELTA   1400
#define MQ7_WARN_DELTA     500    // carbon monoxide
#define MQ7_DANGER_DELTA   1200

#define GAS_HYST_COUNTS    100    // must fall this far below a threshold to step down
#define GAS_RAISE_N        3      // consecutive polls above threshold to raise (3 x 250ms)
#define GAS_CLEAR_N        12     // consecutive polls below to clear (3 s)
#define ADC_SAMPLES        8      // averaged ADC samples per reading

// ---------- Temperature ----------
#define TEMP_WARN_C        42.0   // fixed-temperature warning
#define TEMP_DANGER_C      55.0   // fixed-temperature danger (heat detectors use ~57C)
#define TEMP_HYST_C        1.5
#define RISE_WINDOW_S      30     // window for rate-of-rise
#define RISE_WARN_C_MIN    4.0    // C per minute -> WARNING
#define RISE_DANGER_C_MIN  8.0    // C per minute -> DANGER
#define RISE_HYST          1.0
#define TEMP_RAISE_N       2      // counted per DHT reading (2 x 2 s)
#define TEMP_CLEAR_N       3

// ---------- Flame ----------
#define FLAME_RAISE_N      2      // 2 x 250ms
#define FLAME_CLEAR_N      8      // 2 s

// ---------- Vibration (SW-420) ----------
// Counted as pulses inside a sliding window, so one bump never alarms.
#define VIB_WINDOW_MS      2000
#define VIB_DEBOUNCE_MS    20
#define VIB_WARN_COUNT     3
#define VIB_DANGER_COUNT   15
#define VIB_CLEAR_N        6

// ---------- Fusion (cross-sensor) rules ----------
#define FUSION_SMOKE_CO    true   // MQ2 + MQ7 both alarming  -> DANGER (typical fire signature)
#define FUSION_SMOKE_HEAT  true   // MQ2 + temp/rise alarming -> DANGER
#define FUSION_VIB_GAS     true   // vibration + MQ4/MQ2      -> DANGER (pipe damage / quake)
#define RISK_WARN_PCT      60.0   // combined risk index raises WARNING (never DANGER on its own)

// ---------- Timing ----------
#define SENSOR_POLL_MS     250
#define ENV_POLL_MS        2000   // DHT22 minimum is 2 s
#define OLED_POLL_MS       500
#define OLED_AUTO_ROTATE   true
#define OLED_ROTATE_MS     4000
#define OLED_HOLD_MS       15000  // after manual page change
#define BMP_RETRY_MS       10000
#define WIFI_RETRY_MS      30000

// ---------- Buzzer patterns ----------
#define DANGER_BUZZ_ON_MS  400    // fast beeping until cleared or silenced
#define DANGER_BUZZ_OFF_MS 400
#define WARN_BUZZ_ON_MS    200    // two short beeps every WARN_BUZZ_PERIOD_MS
#define WARN_BUZZ_GAP_MS   200
#define WARN_BUZZ_PERIOD_MS 2000
#define WARN_BUZZ_TOTAL_MS 20000  // warning buzzer stops after this long (LED stays on)
#define MUTE_MS            60000  // "Silence buzzer" button duration (auto-unmutes on escalation)
#define LED_DANGER_BLINK   true   // blink red LED in danger

// =====================================================================
// 2. OBJECTS & STATE
// =====================================================================
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
Adafruit_BMP280 bmp;
DHT dht(PIN_DHT, DHT22);
WebServer server(80);

bool oledReady = false, bmpReady = false;
String i2cScanList = "Scanning...";
unsigned long bootMs = 0;

struct SensorData {
  int mq2 = 0, mq4 = 0, mq7 = 0, ldr = 0;
  bool flame = false, vibNow = false;
  int vibCount = 0;
  float dhtTemp = 0, dhtHum = 0, bmpTemp = 0, bmpPres = 0;
  float temp = 0, tempRate = 0;
  bool tempValid = false, dhtOk = false;
  float risk = 0;
} sensors;

// ----- alert channels -----
enum Level : uint8_t { L_OK = 0, L_WARN = 1, L_DANGER = 2 };
enum ChanId { S_MQ2, S_MQ4, S_MQ7, S_FLAME, S_TEMP, S_RISE, S_VIB, S_FUSE, S_COUNT };

struct AlertChannel {
  const char* id; const char* label; const char* abbr;
  uint8_t level, raiseNeed, clearNeed, raiseCnt, clearCnt;
  char why[64];
};
AlertChannel ch[S_COUNT] = {
  {"mq2",  "MQ2 Smoke/LPG",   "MQ2",   0, GAS_RAISE_N,   GAS_CLEAR_N,   0, 0},
  {"mq4",  "MQ4 Methane",     "MQ4",   0, GAS_RAISE_N,   GAS_CLEAR_N,   0, 0},
  {"mq7",  "MQ7 CO",          "MQ7",   0, GAS_RAISE_N,   GAS_CLEAR_N,   0, 0},
  {"flame","Flame Sensor",    "FLAME", 0, FLAME_RAISE_N, FLAME_CLEAR_N, 0, 0},
  {"temp", "Temperature",     "TEMP",  0, TEMP_RAISE_N,  TEMP_CLEAR_N,  0, 0},
  {"rise", "Temp Rise Rate",  "RISE",  0, TEMP_RAISE_N,  TEMP_CLEAR_N,  0, 0},
  {"vib",  "Vibration",       "VIB",   0, 1,             VIB_CLEAR_N,   0, 0},
  {"fuse", "Combined Risk",   "FUSION",0, 1,             1,             0, 0},
};

uint8_t currentDanger = L_OK;
unsigned long alertStartTime = 0, muteUntil = 0;

// ----- alert log -----
struct LogEntry { uint32_t t; uint8_t lv; uint8_t chan; char why[64]; };
LogEntry alertLog[8];
int logHead = 0, logCount = 0;

// ----- MQ calibration -----
enum MqPhase { MQ_WARMING, MQ_CALIB, MQ_READY };
MqPhase mqPhase = MQ_WARMING;
unsigned long calibStart = 0;
double calibAcc[3] = {0, 0, 0};
int calibN = 0;
float gasBase[3] = {MQ2_FIXED_BASELINE, MQ4_FIXED_BASELINE, MQ7_FIXED_BASELINE};
const float gasWarnD[3]   = {MQ2_WARN_DELTA,   MQ4_WARN_DELTA,   MQ7_WARN_DELTA};
const float gasDangerD[3] = {MQ2_DANGER_DELTA, MQ4_DANGER_DELTA, MQ7_DANGER_DELTA};
float gasWarnT[3], gasDangerT[3];

// ----- temperature history -----
#define RISE_SAMPLES ((RISE_WINDOW_S * 1000 / ENV_POLL_MS) + 1)
float tHist[RISE_SAMPLES];
int tIdx = 0, tCount = 0, dhtFails = 0;

// ----- vibration -----
uint32_t vibTs[40];
int vibHead = 0;
bool lastVibState = false;
unsigned long lastVibEdge = 0;

// ----- OLED -----
int oledPage = 0;
const int TOTAL_OLED_PAGES = 4;
unsigned long oledHoldUntil = 0, lastOledRotate = 0;

// ----- manual test mode -----
bool testMode = false, testRed = false, testYellow = false, testGreen = false, testBuz1 = false, testBuz2 = false;

unsigned long lastSensorPoll = 0, lastEnvPoll = 0, lastOledPoll = 0, lastBmpRetry = 0, lastWifiRetry = 0;

// =====================================================================
// 3. HELPERS
// =====================================================================
void setLED(uint8_t pin, bool on) {
  digitalWrite(pin, (LED_ACTIVE_HIGH ? on : !on) ? HIGH : LOW);
}

const char* lvlName(uint8_t l) { return l == L_DANGER ? "DANGER" : (l == L_WARN ? "WARNING" : "OK"); }

void scanI2CBus() {
  i2cScanList = "";
  int count = 0;
  for (uint8_t a = 1; a < 127; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) {
      if (count++) i2cScanList += ", ";
      i2cScanList += "0x" + String(a, HEX);
    }
  }
  if (!count) i2cScanList = "None (check SDA21/SCL22)";
  Serial.println("I2C: " + i2cScanList);
}

bool initBMP280() {
  const uint8_t addrs[2] = {0x76, 0x77};
  const uint8_t ids[4]   = {0x58, 0x60, 0x56, 0x57};
  for (int a = 0; a < 2; a++)
    for (int c = 0; c < 4; c++)
      if (bmp.begin(addrs[a], ids[c])) {
        bmp.setSampling(Adafruit_BMP280::MODE_NORMAL, Adafruit_BMP280::SAMPLING_X2,
                        Adafruit_BMP280::SAMPLING_X16, Adafruit_BMP280::FILTER_X16,
                        Adafruit_BMP280::STANDBY_MS_500);
        Serial.printf("BMP280 at 0x%02X\n", addrs[a]);
        return true;
      }
  return false;
}

void runHardwareSelfTest() {
  setLED(PIN_LED_RED, false); setLED(PIN_LED_YELLOW, false); setLED(PIN_LED_GREEN, false);
  digitalWrite(PIN_BUZZER_1, LOW); digitalWrite(PIN_BUZZER_2, LOW);
  setLED(PIN_LED_RED, true);    delay(300); setLED(PIN_LED_RED, false);
  setLED(PIN_LED_YELLOW, true); delay(300); setLED(PIN_LED_YELLOW, false);
  setLED(PIN_LED_GREEN, true);  delay(300);
  digitalWrite(PIN_BUZZER_1, HIGH); digitalWrite(PIN_BUZZER_2, HIGH); delay(120);
  digitalWrite(PIN_BUZZER_1, LOW);  digitalWrite(PIN_BUZZER_2, LOW);
}

int readADC(int pin) {
  long s = 0;
  for (int i = 0; i < ADC_SAMPLES; i++) s += analogRead(pin);
  return (int)(s / ADC_SAMPLES);
}

// 0 at lo, 0.5 at warn, 1.0 at danger (used for the risk index)
float norm(float v, float lo, float warn, float danger) {
  if (v <= lo) return 0;
  if (v < warn) return 0.5f * (v - lo) / (warn - lo);
  if (v < danger) return 0.5f + 0.5f * (v - warn) / (danger - warn);
  return 1.0f;
}

void logEvent(int i) {
  LogEntry &e = alertLog[logHead];
  e.t = millis() / 1000; e.lv = ch[i].level; e.chan = i;
  strncpy(e.why, ch[i].why, sizeof(e.why) - 1); e.why[sizeof(e.why) - 1] = 0;
  logHead = (logHead + 1) % 8;
  if (logCount < 8) logCount++;
  Serial.printf("[ALERT] %s -> %s : %s\n", ch[i].label, lvlName(ch[i].level), ch[i].why);
}

// Core judge: threshold + hysteresis + debounce (raise slowly, clear slowly)
void updateChannel(int i, float v, float warnT, float dangerT, float hyst, const char* why) {
  AlertChannel &c = ch[i];
  strncpy(c.why, why, sizeof(c.why) - 1); c.why[sizeof(c.why) - 1] = 0;

  uint8_t inst = L_OK;
  if (v >= dangerT || (c.level == L_DANGER && v >= dangerT - hyst)) inst = L_DANGER;
  else if (v >= warnT || (c.level >= L_WARN && v >= warnT - hyst)) inst = L_WARN;

  if (inst > c.level) {
    c.clearCnt = 0;
    if (++c.raiseCnt >= c.raiseNeed) { c.level = inst; c.raiseCnt = 0; logEvent(i); }
  } else if (inst < c.level) {
    c.raiseCnt = 0;
    if (++c.clearCnt >= c.clearNeed) { c.level = inst; c.clearCnt = 0; }
  } else {
    c.raiseCnt = c.clearCnt = 0;
  }
}

// =====================================================================
// 4. SENSOR READING & EVALUATION
// =====================================================================
void pollVibration() {
  bool v = (digitalRead(PIN_SW420) == VIB_DETECTED_STATE);
  unsigned long now = millis();
  if (v && !lastVibState && now - lastVibEdge > VIB_DEBOUNCE_MS) {
    lastVibEdge = now;
    vibTs[vibHead] = now ? now : 1;
    vibHead = (vibHead + 1) % 40;
  }
  lastVibState = v;
  sensors.vibNow = v;
}

void updateGasPhase(const int raw[3]) {
  unsigned long now = millis();
  if (mqPhase == MQ_WARMING && now - bootMs >= (unsigned long)(MQ_WARMUP_S - MQ_CALIB_S) * 1000UL) {
    mqPhase = MQ_CALIB; calibStart = now; calibN = 0;
    for (int i = 0; i < 3; i++) calibAcc[i] = 0;
  }
  if (mqPhase == MQ_CALIB) {
    for (int i = 0; i < 3; i++) calibAcc[i] += raw[i];
    calibN++;
    if (now - calibStart >= (unsigned long)MQ_CALIB_S * 1000UL) {
      const float fixedB[3] = {MQ2_FIXED_BASELINE, MQ4_FIXED_BASELINE, MQ7_FIXED_BASELINE};
      for (int i = 0; i < 3; i++) {
        float b = calibN ? (float)(calibAcc[i] / calibN) : fixedB[i];
        gasBase[i] = (MQ_AUTO_BASELINE && b <= MQ_BASELINE_MAX) ? b : fixedB[i];
      }
      mqPhase = MQ_READY;
      Serial.printf("Gas baseline: MQ2=%.0f MQ4=%.0f MQ7=%.0f\n", gasBase[0], gasBase[1], gasBase[2]);
    }
  }
  for (int i = 0; i < 3; i++) {
    gasWarnT[i]   = min(gasBase[i] + gasWarnD[i],   (float)MQ_ADC_CEILING);
    gasDangerT[i] = min(gasBase[i] + gasDangerD[i], (float)MQ_ADC_CEILING + 30.0f);
  }
}

int mqWarmupLeft() {
  unsigned long e = millis() - bootMs;
  if (mqPhase == MQ_WARMING) return (int)(((unsigned long)MQ_WARMUP_S * 1000UL > e ? (unsigned long)MQ_WARMUP_S * 1000UL - e : 0) / 1000) + 1;
  if (mqPhase == MQ_CALIB) {
    unsigned long c = millis() - calibStart;
    return (int)(((unsigned long)MQ_CALIB_S * 1000UL > c ? (unsigned long)MQ_CALIB_S * 1000UL - c : 0) / 1000) + 1;
  }
  return 0;
}

void evaluateFastSensors() {
  int raw[3] = {readADC(PIN_MQ2), readADC(PIN_MQ4), readADC(PIN_MQ7)};
  sensors.mq2 = raw[0]; sensors.mq4 = raw[1]; sensors.mq7 = raw[2];
  sensors.ldr = readADC(PIN_LDR);
  sensors.flame = (digitalRead(PIN_FLAME) == FLAME_DETECTED_STATE);

  updateGasPhase(raw);

  char buf[64];
  // ---- gas ----
  for (int i = 0; i < 3; i++) {
    if (mqPhase != MQ_READY) {
      ch[i].level = L_OK; ch[i].raiseCnt = ch[i].clearCnt = 0;
      snprintf(buf, sizeof(buf), "Warming up / calibrating");
      strncpy(ch[i].why, buf, sizeof(ch[i].why) - 1);
      continue;
    }
    snprintf(buf, sizeof(buf), "ADC %d (warn %.0f, danger %.0f)", raw[i], gasWarnT[i], gasDangerT[i]);
    updateChannel(i, raw[i], gasWarnT[i], gasDangerT[i], GAS_HYST_COUNTS, buf);
    // slow downward baseline tracking (handles sensor drift, never raises baseline)
    if (MQ_AUTO_BASELINE && ch[i].level == L_OK && raw[i] < gasBase[i])
      gasBase[i] = 0.9995f * gasBase[i] + 0.0005f * raw[i];
  }

  // ---- flame ----
  updateChannel(S_FLAME, sensors.flame ? 1 : 0, 1, 1, 0, sensors.flame ? "Flame detected by IR sensor" : "No flame");

  // ---- vibration ----
  unsigned long now = millis();
  int cnt = 0;
  for (int k = 0; k < 40; k++) if (vibTs[k] && now - vibTs[k] <= VIB_WINDOW_MS) cnt++;
  sensors.vibCount = cnt;
  snprintf(buf, sizeof(buf), "%d pulses in %ds (warn %d, danger %d)", cnt, VIB_WINDOW_MS / 1000, VIB_WARN_COUNT, VIB_DANGER_COUNT);
  updateChannel(S_VIB, cnt, VIB_WARN_COUNT, VIB_DANGER_COUNT, 0, buf);
}

void evaluateEnvSensors() {
  // DHT22
  float t = dht.readTemperature(), h = dht.readHumidity();
  if (!isnan(t) && t > -40 && t < 120) { sensors.dhtTemp = t; dhtFails = 0; }
  else dhtFails++;
  if (!isnan(h) && h >= 0 && h <= 100) sensors.dhtHum = h;
  sensors.dhtOk = (dhtFails < 5);

  // BMP280 (auto reconnect)
  if (!bmpReady && millis() - lastBmpRetry > BMP_RETRY_MS) {
    lastBmpRetry = millis(); scanI2CBus(); bmpReady = initBMP280();
  }
  if (bmpReady) {
    float bt = bmp.readTemperature(), bp = bmp.readPressure() / 100.0F;
    if (!isnan(bt) && bp > 300 && bp < 1300) { sensors.bmpTemp = bt; sensors.bmpPres = bp; }
    else bmpReady = false;
  }

  // Pick temperature source
  if (sensors.dhtOk && dhtFails == 0) { sensors.temp = sensors.dhtTemp; sensors.tempValid = true; }
  else if (bmpReady)                  { sensors.temp = sensors.bmpTemp; sensors.tempValid = true; }
  else if (sensors.dhtOk)             { sensors.temp = sensors.dhtTemp; sensors.tempValid = true; }
  else                                  sensors.tempValid = false;
  if (!sensors.tempValid) return;

  // Rate of rise over RISE_WINDOW_S
  tHist[tIdx] = sensors.temp;
  tIdx = (tIdx + 1) % RISE_SAMPLES;
  if (tCount < RISE_SAMPLES) tCount++;
  if (tCount >= RISE_SAMPLES) {
    float oldest = tHist[tIdx];  // next slot to be overwritten = oldest sample
    sensors.tempRate = (sensors.temp - oldest) * (60.0f / RISE_WINDOW_S);
  } else sensors.tempRate = 0;

  char buf[64];
  snprintf(buf, sizeof(buf), "%.1fC (warn %.0f, danger %.0f)", sensors.temp, TEMP_WARN_C, TEMP_DANGER_C);
  updateChannel(S_TEMP, sensors.temp, TEMP_WARN_C, TEMP_DANGER_C, TEMP_HYST_C, buf);
  snprintf(buf, sizeof(buf), "+%.1fC/min over %ds (warn %.0f, danger %.0f)", sensors.tempRate, RISE_WINDOW_S, RISE_WARN_C_MIN, RISE_DANGER_C_MIN);
  updateChannel(S_RISE, sensors.tempRate, RISE_WARN_C_MIN, RISE_DANGER_C_MIN, RISE_HYST, buf);
}

void evaluateOverall() {
  // ---- risk index (informational + can raise WARNING) ----
  float f[7], mx = 0, sum = 0;
  for (int i = 0; i < 3; i++) f[i] = (mqPhase == MQ_READY) ? norm((i == 0 ? sensors.mq2 : i == 1 ? sensors.mq4 : sensors.mq7), gasBase[i], gasWarnT[i], gasDangerT[i]) : 0;
  f[3] = sensors.flame ? 1.0f : 0.0f;
  f[4] = sensors.tempValid ? norm(sensors.temp, 25.0f, TEMP_WARN_C, TEMP_DANGER_C) : 0;
  f[5] = norm(sensors.tempRate, 0, RISE_WARN_C_MIN, RISE_DANGER_C_MIN);
  f[6] = norm(sensors.vibCount, 0, VIB_WARN_COUNT, VIB_DANGER_COUNT);
  for (int i = 0; i < 7; i++) { sum += f[i]; if (f[i] > mx) mx = f[i]; }
  sensors.risk = 100.0f * min(1.0f, mx + 0.3f * (sum - mx));

  // ---- fusion rules ----
  bool smoke = ch[S_MQ2].level >= L_WARN, co = ch[S_MQ7].level >= L_WARN, ch4 = ch[S_MQ4].level >= L_WARN;
  bool heat = ch[S_TEMP].level >= L_WARN || ch[S_RISE].level >= L_WARN;
  bool vib = ch[S_VIB].level >= L_WARN;
  uint8_t fl = L_OK; char buf[64] = "No cross-sensor correlation";
  if (FUSION_SMOKE_CO && smoke && co)               { fl = L_DANGER; strcpy(buf, "Smoke + CO together (fire signature)"); }
  else if (FUSION_SMOKE_HEAT && smoke && heat)      { fl = L_DANGER; strcpy(buf, "Smoke + rising heat (fire signature)"); }
  else if (FUSION_VIB_GAS && vib && (ch4 || smoke)) { fl = L_DANGER; strcpy(buf, "Vibration + gas (leak / damage)"); }
  else if (sensors.risk >= RISK_WARN_PCT)           { fl = L_WARN;   snprintf(buf, sizeof(buf), "Combined risk index %.0f%%", sensors.risk); }
  if (fl > ch[S_FUSE].level) { ch[S_FUSE].level = fl; strncpy(ch[S_FUSE].why, buf, 63); logEvent(S_FUSE); }
  else { ch[S_FUSE].level = fl; strncpy(ch[S_FUSE].why, buf, 63); }

  // ---- overall level ----
  uint8_t lvl = L_OK;
  for (int i = 0; i < S_COUNT; i++) if (ch[i].level > lvl) lvl = ch[i].level;
  if (lvl != currentDanger) {
    alertStartTime = millis();
    if (lvl > currentDanger) muteUntil = 0;   // escalation un-silences the buzzer
    currentDanger = lvl;
  }
}

String alertSummary(bool shortForm) {
  String s = "";
  for (int i = 0; i < S_COUNT; i++) {
    if (ch[i].level == L_OK) continue;
    if (s.length()) s += shortForm ? " " : ", ";
    s += shortForm ? ch[i].abbr : ch[i].label;
    if (!shortForm) s += ch[i].level == L_DANGER ? " (DANGER)" : " (WARN)";
  }
  return s;
}

// =====================================================================
// 5. OUTPUTS (LED + BUZZER)
// =====================================================================
void updateOutputs() {
  if (testMode) {
    setLED(PIN_LED_RED, testRed); setLED(PIN_LED_YELLOW, testYellow); setLED(PIN_LED_GREEN, testGreen);
    digitalWrite(PIN_BUZZER_1, testBuz1); digitalWrite(PIN_BUZZER_2, testBuz2);
    return;
  }
  unsigned long now = millis(), el = now - alertStartTime;
  bool buz = false;

  if (currentDanger == L_DANGER) {
    setLED(PIN_LED_RED, LED_DANGER_BLINK ? ((now / 300) % 2 == 0) : true);
    setLED(PIN_LED_YELLOW, false); setLED(PIN_LED_GREEN, false);
    buz = (el % (DANGER_BUZZ_ON_MS + DANGER_BUZZ_OFF_MS)) < DANGER_BUZZ_ON_MS;
  } else if (currentDanger == L_WARN) {
    setLED(PIN_LED_RED, false); setLED(PIN_LED_YELLOW, true); setLED(PIN_LED_GREEN, false);
    if (el < WARN_BUZZ_TOTAL_MS) {
      unsigned long p = el % WARN_BUZZ_PERIOD_MS;
      buz = (p < WARN_BUZZ_ON_MS) || (p >= WARN_BUZZ_ON_MS + WARN_BUZZ_GAP_MS && p < 2 * WARN_BUZZ_ON_MS + WARN_BUZZ_GAP_MS);
    }
  } else {
    setLED(PIN_LED_RED, false); setLED(PIN_LED_YELLOW, false); setLED(PIN_LED_GREEN, true);
  }
  if (now < muteUntil) buz = false;
  digitalWrite(PIN_BUZZER_1, buz); digitalWrite(PIN_BUZZER_2, buz);
}

// =====================================================================
// 6. OLED
// =====================================================================
void updateOLED() {
  if (!oledReady) return;
  unsigned long now = millis();
  if (currentDanger > L_OK && now > oledHoldUntil) oledPage = 0;
  else if (OLED_AUTO_ROTATE && currentDanger == L_OK && now > oledHoldUntil && now - lastOledRotate > OLED_ROTATE_MS) {
    oledPage = (oledPage + 1) % TOTAL_OLED_PAGES; lastOledRotate = now;
  }

  display.clearDisplay(); display.setTextSize(1); display.setTextColor(SSD1306_WHITE); display.setCursor(0, 0);
  const int gv[3] = {sensors.mq2, sensors.mq4, sensors.mq7};
  const char* gn[3] = {"MQ2", "MQ4", "MQ7"};

  if (oledPage == 0) {
    display.println(F("== HAZARD SYSTEM =="));
    display.print(F("State: ")); display.println(currentDanger == L_DANGER ? F("!! DANGER !!") : currentDanger == L_WARN ? F("WARNING") : F("SAFE"));
    display.printf("Risk:%.0f%% T:%.1fC\n", sensors.risk, sensors.temp);
    if (currentDanger > L_OK) {
      display.println(F("ALERT FROM:"));
      display.println(alertSummary(true).substring(0, 42));
    } else if (mqPhase != MQ_READY) {
      display.printf("Gas warm-up: %ds\n", mqWarmupLeft());
    } else display.println(F("All sensors normal"));
  } else if (oledPage == 1) {
    display.println(F("=== GAS SENSORS ==="));
    for (int i = 0; i < 3; i++)
      display.printf("%s %4d %+5d %s\n", gn[i], gv[i], gv[i] - (int)gasBase[i], lvlName(ch[i].level));
    display.printf("Flame: %s\n", sensors.flame ? "FIRE!" : "clear");
    display.printf("Vib: %d pulses\n", sensors.vibCount);
  } else if (oledPage == 2) {
    display.println(F("=== ENVIRONMENT ==="));
    display.printf("DHT : %.1fC %.0f%%\n", sensors.dhtTemp, sensors.dhtHum);
    if (bmpReady) display.printf("BMP : %.1fC\nPres: %.1f hPa\n", sensors.bmpTemp, sensors.bmpPres);
    else display.println(F("BMP : not found\n"));
    display.printf("Rise: %+.1f C/min\n", sensors.tempRate);
    display.printf("Light: %d\n", sensors.ldr);
  } else {
    display.println(F("=== NETWORK/STATUS =="));
    display.print(F("IP: ")); display.println(WiFi.status() == WL_CONNECTED ? WiFi.localIP().toString() : String("AP ") + WiFi.softAPIP().toString());
    display.print(F("I2C: ")); display.println(i2cScanList.substring(0, 16));
    display.printf("Mode: %s\n", testMode ? "MANUAL TEST" : "AUTO");
    display.printf("Muted: %s\n", millis() < muteUntil ? "YES" : "no");
  }
  display.display();
}

// =====================================================================
// 7. WEB DASHBOARD
// =====================================================================
const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en"><head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1, viewport-fit=cover">
<title>Hazard Monitor</title>
<style>
:root{--bg:#0b1220;--card:#162032;--line:#2a3a55;--txt:#f1f5f9;--sub:#94a3b8;--ok:#10b981;--warn:#f59e0b;--bad:#ef4444;--acc:#6366f1}
*{box-sizing:border-box}
body{margin:0;background:var(--bg);color:var(--txt);font-family:system-ui,-apple-system,Segoe UI,Roboto,sans-serif;padding:0 12px 24px;padding-left:max(12px,env(safe-area-inset-left));padding-right:max(12px,env(safe-area-inset-right))}
.wrap{max-width:1000px;margin:0 auto}
header{display:flex;justify-content:space-between;align-items:center;gap:8px;padding:12px 0 8px}
h1{font-size:1.1rem;margin:0}
.meta{font-size:.72rem;color:var(--sub);text-align:right}
#banner{position:sticky;top:0;z-index:5;padding:14px;border-radius:12px;font-weight:700;text-align:center;margin:6px 0 12px;border:2px solid;font-size:clamp(.95rem,3.8vw,1.2rem)}
#banner small{display:block;font-weight:500;font-size:.78rem;margin-top:4px;opacity:.9}
.b0{background:#064e3b;border-color:var(--ok)}.b1{background:#78350f;border-color:var(--warn)}.b2{background:#7f1d1d;border-color:var(--bad);animation:pulse 1s infinite}.bx{background:#334155;border-color:#64748b}
@keyframes pulse{50%{opacity:.75}}
.panel{background:var(--card);border:1px solid var(--line);border-radius:12px;padding:14px;margin-bottom:12px}
.panel h3{margin:0 0 8px;font-size:.78rem;color:var(--sub);text-transform:uppercase;letter-spacing:.04em}
.chips{display:flex;flex-wrap:wrap;gap:8px}
.chip{padding:8px 10px;border-radius:10px;font-size:.82rem;border:1px solid;flex:1 1 220px}
.chip b{display:block}.chip span{font-size:.74rem;opacity:.85}
.c1{background:#451a03;border-color:var(--warn)}.c2{background:#450a0a;border-color:var(--bad)}
.none{color:var(--ok);font-size:.9rem}
.risk{display:flex;align-items:center;gap:12px}
.risk .n{font-size:1.6rem;font-weight:800;min-width:4.2ch;color:#a5b4fc}
.bar{flex:1;height:12px;background:#0b1220;border-radius:8px;overflow:hidden}
.bar i{display:block;height:100%;width:0;background:linear-gradient(90deg,var(--ok),var(--warn),var(--bad));transition:width .4s}
.grid{display:grid;grid-template-columns:repeat(auto-fill,minmax(180px,1fr));gap:10px;margin-bottom:12px}
.card{background:var(--card);border:2px solid var(--line);border-radius:12px;padding:12px;position:relative;transition:border-color .3s,background .3s}
.card .t{font-size:.72rem;color:var(--sub);text-transform:uppercase;display:flex;justify-content:space-between;gap:6px;align-items:center}
.badge{font-size:.62rem;font-weight:800;padding:2px 7px;border-radius:20px;background:#064e3b;color:#a7f3d0;white-space:nowrap}
.val{font-size:clamp(1.15rem,5vw,1.55rem);font-weight:700;margin:6px 0 2px;word-break:break-word}
.sub{font-size:.72rem;color:var(--sub)}
.why{font-size:.74rem;margin-top:6px;display:none;font-weight:600}
.gauge{position:relative;height:8px;background:#0b1220;border-radius:6px;margin-top:8px}
.gauge i{position:absolute;left:0;top:0;bottom:0;border-radius:6px;background:var(--ok)}
.gauge u{position:absolute;top:-3px;bottom:-3px;width:2px}
.lv1{border-color:var(--warn);background:#2a1f0b}.lv1 .badge{background:var(--warn);color:#000}.lv1 .why{display:block;color:#fcd34d}.lv1 .gauge i{background:var(--warn)}
.lv2{border-color:var(--bad);background:#3b0d0d;animation:glow 1s infinite}.lv2 .badge{background:var(--bad);color:#fff}.lv2 .why{display:block;color:#fca5a5}.lv2 .gauge i{background:var(--bad)}
@keyframes glow{50%{box-shadow:0 0 0 3px rgba(239,68,68,.55)}}
.btn{padding:11px 14px;border:2px solid transparent;border-radius:10px;font-weight:600;color:#fff;background:#3b82f6;margin:3px;font-size:.85rem;cursor:pointer;touch-action:manipulation}
.btn.active{border-color:#fff;box-shadow:0 0 8px rgba(255,255,255,.6)}
.r{background:#dc2626}.y{background:#d97706}.g{background:#059669}.p{background:#7c3aed}.s{background:#475569}
details summary{cursor:pointer;font-weight:600;font-size:.9rem}
table{width:100%;border-collapse:collapse;font-size:.78rem}td{padding:5px 4px;border-bottom:1px solid var(--line);vertical-align:top}
@media(max-width:520px){.grid{grid-template-columns:repeat(2,1fr);gap:8px}.card{padding:10px}.btn{flex:1 1 45%}header{flex-direction:column;align-items:flex-start}.meta{text-align:left}}
</style></head><body><div class="wrap">
<header><h1>🔥 ESP32 Hazard Monitor</h1><div class="meta" id="meta">connecting…</div></header>
<div id="banner" class="bx">Connecting…</div>

<div class="panel"><h3>Active alerts &mdash; who raised it</h3>
 <div id="alerts" class="chips"><span class="none">No active alerts</span></div>
 <div style="margin-top:10px"><button class="btn r" id="muteBtn" onclick="act('/api/mute')">🔕 Silence buzzer</button></div>
</div>

<div class="panel"><h3>Combined risk index</h3>
 <div class="risk"><div class="n" id="riskN">0%</div><div class="bar"><i id="riskBar"></i></div></div>
 <div class="sub" id="fuseWhy" style="margin-top:6px"></div>
</div>

<div class="grid" id="grid"></div>

<div class="panel"><h3>Recent alert log</h3><table id="log"><tr><td class="sub">Empty</td></tr></table></div>

<div class="panel"><details><summary>Hardware tester &amp; tools</summary>
 <p class="sub">OLED: <b id="oledTxt">1/4</b> · Mode: <b id="modeTxt">AUTO</b> · I2C: <b id="i2cTxt">-</b></p>
 <button class="btn p" onclick="act('/api/screen')">Next OLED page</button>
 <button class="btn" onclick="act('/api/test?target=all_on')">All ON</button>
 <button class="btn s" onclick="act('/api/test?target=auto')">Back to AUTO</button>
 <button class="btn s" onclick="if(confirm('Recalibrate gas baseline? Keep sensors in CLEAN AIR for 10 s.'))act('/api/recal')">Recalibrate gas</button>
 <hr style="border-color:var(--line)">
 <button id="tRed" class="btn r" onclick="act('/api/test?target=red')">Red</button>
 <button id="tYel" class="btn y" onclick="act('/api/test?target=yellow')">Yellow</button>
 <button id="tGrn" class="btn g" onclick="act('/api/test?target=green')">Green</button>
 <button id="tB1" class="btn" onclick="act('/api/test?target=buz1')">Buzzer 1</button>
 <button id="tB2" class="btn" onclick="act('/api/test?target=buz2')">Buzzer 2</button>
</details></div>
</div>
<script>
const $=id=>document.getElementById(id);
const CARDS=[
 {id:'mq2',t:'MQ2 Smoke / LPG',p:'GPIO34',g:1},{id:'mq4',t:'MQ4 Methane',p:'GPIO35',g:1},{id:'mq7',t:'MQ7 Carbon Monoxide',p:'GPIO36',g:1},
 {id:'flame',t:'Flame',p:'GPIO33'},{id:'temp',t:'Temperature',p:'DHT22 / BMP280'},{id:'rise',t:'Temp Rise Rate',p:'30 s window'},
 {id:'vib',t:'Vibration',p:'SW-420 GPIO32'},{id:'hum',t:'Humidity',p:'DHT22 GPIO4'},{id:'bmp',t:'Pressure',p:'BMP280 I2C'},{id:'ldr',t:'Light',p:'LDR GPIO39'}];
$('grid').innerHTML=CARDS.map(c=>`<div class="card" id="c_${c.id}"><div class="t"><span>${c.t}</span><span class="badge" id="b_${c.id}">OK</span></div><div class="val" id="v_${c.id}">--</div><div class="sub" id="s_${c.id}">${c.p}</div>${c.g?`<div class="gauge" id="g_${c.id}"><i></i><u style="background:#f59e0b"></u><u style="background:#ef4444"></u></div>`:''}<div class="why" id="w_${c.id}"></div></div>`).join('');
const LN=['OK','WARNING','DANGER'];
function lv(id,l,why){const c=$('c_'+id);if(!c)return;c.className='card'+(l?' lv'+l:'');$('b_'+id).innerText=LN[l];$('w_'+id).innerText=why||''}
function V(id,v,s){$('v_'+id).innerText=v;if(s!==undefined)$('s_'+id).innerText=s}
function gauge(id,raw,thr){const g=$('g_'+id);if(!g)return;const m=4095;g.children[0].style.width=Math.min(100,raw/m*100)+'%';g.children[1].style.left=thr[0]/m*100+'%';g.children[2].style.left=thr[1]/m*100+'%'}
async function act(u){try{await fetch(u)}catch(e){}load()}
let fails=0;
async function load(){
 const ctl=new AbortController();const to=setTimeout(()=>ctl.abort(),2500);
 try{
  const d=await (await fetch('/api/data',{signal:ctl.signal})).json();clearTimeout(to);fails=0;
  const L={};d.ch.forEach(c=>L[c.id]=c);
  // gas
  ['mq2','mq4','mq7'].forEach((k,i)=>{V(k,d[k]+' / 4095',d.warm?('Warming up '+d.warm+'s'):('Δ '+(d[k]-d.base[i]>=0?'+':'')+Math.round(d[k]-d.base[i])+' vs clean-air '+Math.round(d.base[i])));lv(k,L[k].lv,L[k].why);gauge(k,d[k],d.thr[k])});
  V('flame',d.flame?'FIRE DETECTED':'Clear','GPIO33');lv('flame',L.flame.lv,L.flame.why);
  V('temp',d.tv?d.temp.toFixed(1)+' °C':'No sensor','warn '+d.thr.temp[0]+' / danger '+d.thr.temp[1]+' °C');lv('temp',L.temp.lv,L.temp.why);
  V('rise',(d.rate>=0?'+':'')+d.rate.toFixed(1)+' °C/min','warn '+d.thr.rise[0]+' / danger '+d.thr.rise[1]);lv('rise',L.rise.lv,L.rise.why);
  V('vib',d.vibCount+' pulses',d.vib?'vibrating now':'steady');lv('vib',L.vib.lv,L.vib.why);
  V('hum',d.dhtOk?d.dhtHum.toFixed(0)+' %':'DHT fault',d.dhtOk?'DHT22 temp '+d.dhtTemp.toFixed(1)+' °C':'check wiring');
  V('bmp',d.bmpReady?d.bmpPres.toFixed(1)+' hPa':'Disconnected',d.bmpReady?'BMP temp '+d.bmpTemp.toFixed(1)+' °C':'check CSB→3.3V, SDA/SCL');
  V('ldr',d.ldr+' / 4095','light level');
  // risk
  $('riskN').innerText=d.risk.toFixed(0)+'%';$('riskBar').style.width=d.risk+'%';$('fuseWhy').innerText=L.fuse.why;
  // banner
  const b=$('banner');
  if(d.danger==2){b.className='b2';b.innerHTML='🚨 DANGER<small>'+d.alerts+'</small>'}
  else if(d.danger==1){b.className='b1';b.innerHTML='⚠️ WARNING<small>'+d.alerts+'</small>'}
  else{b.className='b0';b.innerHTML='✅ System Safe'+(d.warm?'<small>Gas sensors warming up ('+d.warm+'s)</small>':'')}
  document.title=(d.danger==2?'🚨 ':d.danger==1?'⚠️ ':'✅ ')+'Hazard Monitor';
  // alert chips
  const act_=d.ch.filter(c=>c.lv>0);
  $('alerts').innerHTML=act_.length?act_.map(c=>`<div class="chip c${c.lv}"><b>${c.lv==2?'🔴':'🟠'} ${c.n} &mdash; ${LN[c.lv]}</b><span>${c.why}</span></div>`).join(''):'<span class="none">No active alerts</span>';
  $('muteBtn').innerText=d.muted?('🔔 Silenced ('+d.muted+'s) — tap to unmute'):'🔕 Silence buzzer';
  $('muteBtn').style.display=d.danger?'inline-block':'none';
  // log
  $('log').innerHTML=d.log.length?d.log.map(e=>`<tr><td>${e.t}s</td><td style="color:${e.lv==2?'#fca5a5':'#fcd34d'}"><b>${e.s}</b> ${LN[e.lv]}</td><td class="sub">${e.w}</td></tr>`).join(''):'<tr><td class="sub">Empty</td></tr>';
  // tools
  $('oledTxt').innerText=(d.oled+1)+'/4';$('modeTxt').innerText=d.testMode?'MANUAL TEST':'AUTO';$('i2cTxt').innerText=d.i2c;
  [['tRed','tr'],['tYel','ty'],['tGrn','tg'],['tB1','b1'],['tB2','b2']].forEach(a=>$(a[0]).classList.toggle('active',d[a[1]]));
  $('meta').innerText=d.ip+' · up '+Math.floor(d.up/60)+'m';
 }catch(e){clearTimeout(to);if(++fails>=2){$('banner').className='bx';$('banner').innerHTML='⛔ Connection lost<small>Retrying…</small>'}}
}
setInterval(load,1000);load();
</script></body></html>
)rawliteral";

// =====================================================================
// 8. WEB HANDLERS
// =====================================================================
String b(bool v) { return v ? "true" : "false"; }

void handleApiData() {
  String j; j.reserve(3600);
  j += "{\"mq2\":" + String(sensors.mq2) + ",\"mq4\":" + String(sensors.mq4) + ",\"mq7\":" + String(sensors.mq7);
  j += ",\"ldr\":" + String(sensors.ldr) + ",\"flame\":" + b(sensors.flame) + ",\"vib\":" + b(sensors.vibNow);
  j += ",\"vibCount\":" + String(sensors.vibCount);
  j += ",\"dhtTemp\":" + String(sensors.dhtTemp, 1) + ",\"dhtHum\":" + String(sensors.dhtHum, 1) + ",\"dhtOk\":" + b(sensors.dhtOk);
  j += ",\"bmpReady\":" + b(bmpReady) + ",\"bmpTemp\":" + String(sensors.bmpTemp, 1) + ",\"bmpPres\":" + String(sensors.bmpPres, 1);
  j += ",\"temp\":" + String(sensors.temp, 1) + ",\"tv\":" + b(sensors.tempValid) + ",\"rate\":" + String(sensors.tempRate, 1);
  j += ",\"risk\":" + String(sensors.risk, 1) + ",\"danger\":" + String(currentDanger);
  j += ",\"alerts\":\"" + alertSummary(false) + "\"";
  j += ",\"muted\":" + String(millis() < muteUntil ? (muteUntil - millis()) / 1000 : 0);
  j += ",\"warm\":" + String(mqWarmupLeft());
  j += ",\"base\":[" + String(gasBase[0], 0) + "," + String(gasBase[1], 0) + "," + String(gasBase[2], 0) + "]";
  j += ",\"thr\":{";
  j += "\"mq2\":[" + String(gasWarnT[0], 0) + "," + String(gasDangerT[0], 0) + "],";
  j += "\"mq4\":[" + String(gasWarnT[1], 0) + "," + String(gasDangerT[1], 0) + "],";
  j += "\"mq7\":[" + String(gasWarnT[2], 0) + "," + String(gasDangerT[2], 0) + "],";
  j += "\"temp\":[" + String(TEMP_WARN_C, 0) + "," + String(TEMP_DANGER_C, 0) + "],";
  j += "\"rise\":[" + String(RISE_WARN_C_MIN, 0) + "," + String(RISE_DANGER_C_MIN, 0) + "]}";
  j += ",\"ch\":[";
  for (int i = 0; i < S_COUNT; i++) {
    if (i) j += ",";
    j += "{\"id\":\"" + String(ch[i].id) + "\",\"n\":\"" + String(ch[i].label) + "\",\"lv\":" + String(ch[i].level) + ",\"why\":\"" + String(ch[i].why) + "\"}";
  }
  j += "],\"log\":[";
  for (int k = 0; k < logCount; k++) {
    int idx = (logHead - 1 - k + 16) % 8;
    if (k) j += ",";
    j += "{\"t\":" + String(alertLog[idx].t) + ",\"s\":\"" + String(ch[alertLog[idx].chan].label) + "\",\"lv\":" + String(alertLog[idx].lv) + ",\"w\":\"" + String(alertLog[idx].why) + "\"}";
  }
  j += "],\"oled\":" + String(oledPage) + ",\"i2c\":\"" + i2cScanList + "\"";
  j += ",\"testMode\":" + b(testMode) + ",\"tr\":" + b(testRed) + ",\"ty\":" + b(testYellow) + ",\"tg\":" + b(testGreen);
  j += ",\"b1\":" + b(testBuz1) + ",\"b2\":" + b(testBuz2);
  j += ",\"ip\":\"" + (WiFi.status() == WL_CONNECTED ? WiFi.localIP().toString() : WiFi.softAPIP().toString()) + "\"";
  j += ",\"up\":" + String(millis() / 1000) + "}";
  server.send(200, "application/json", j);
}

void handleSwitchScreen() {
  oledPage = (oledPage + 1) % TOTAL_OLED_PAGES;
  oledHoldUntil = millis() + OLED_HOLD_MS;
  updateOLED();
  server.send(200, "text/plain", "OK");
}

void handleMute() {
  if (millis() < muteUntil) muteUntil = 0;
  else if (currentDanger > L_OK) muteUntil = millis() + MUTE_MS;
  server.send(200, "text/plain", "OK");
}

void handleRecal() {
  mqPhase = MQ_CALIB; calibStart = millis(); calibN = 0;
  for (int i = 0; i < 3; i++) calibAcc[i] = 0;
  server.send(200, "text/plain", "OK");
}

void handleTestHardware() {
  String t = server.arg("target");
  if (t == "auto") { testMode = false; testRed = testYellow = testGreen = testBuz1 = testBuz2 = false; }
  else if (t == "all_on") { testMode = true; testRed = testYellow = testGreen = testBuz1 = testBuz2 = true; }
  else {
    testMode = true;
    if (t == "red") testRed = !testRed;
    if (t == "yellow") testYellow = !testYellow;
    if (t == "green") testGreen = !testGreen;
    if (t == "buz1") testBuz1 = !testBuz1;
    if (t == "buz2") testBuz2 = !testBuz2;
  }
  updateOutputs();
  server.send(200, "text/plain", "OK");
}

// =====================================================================
// 9. SETUP & LOOP
// =====================================================================
void setup() {
  Serial.begin(115200);
  delay(200);
  bootMs = millis();

  pinMode(PIN_SW420, INPUT);
  pinMode(PIN_FLAME, INPUT);
  pinMode(PIN_LED_RED, OUTPUT); pinMode(PIN_LED_YELLOW, OUTPUT); pinMode(PIN_LED_GREEN, OUTPUT);
  pinMode(PIN_BUZZER_1, OUTPUT); pinMode(PIN_BUZZER_2, OUTPUT);
  gpio_set_drive_capability((gpio_num_t)PIN_LED_RED, GPIO_DRIVE_CAP_3);
  gpio_set_drive_capability((gpio_num_t)PIN_LED_YELLOW, GPIO_DRIVE_CAP_3);
  gpio_set_drive_capability((gpio_num_t)PIN_LED_GREEN, GPIO_DRIVE_CAP_3);

  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);

  runHardwareSelfTest();

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(100000);
  Wire.setTimeOut(50);
  scanI2CBus();

  if (display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    oledReady = true;
    display.clearDisplay(); display.setTextSize(1); display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0); display.println(F("Hazard Sys Booting..."));
    display.println(F("Gas warm-up started")); display.display();
  } else Serial.println(F("WARN: OLED not found at 0x3C"));

  bmpReady = initBMP280();
  if (!bmpReady) Serial.println(F("WARN: BMP280 not found"));
  dht.begin();

  for (int i = 0; i < 3; i++) { gasWarnT[i] = gasBase[i] + gasWarnD[i]; gasDangerT[i] = gasBase[i] + gasDangerD[i]; }

  WiFi.mode(WIFI_AP_STA);
  WiFi.softAP(AP_SSID, AP_PASSWORD);
  Serial.printf("Backup hotspot: %s -> http://%s\n", AP_SSID, WiFi.softAPIP().toString().c_str());
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  for (int tries = 0; WiFi.status() != WL_CONNECTED && tries < 20; tries++) {
    setLED(PIN_LED_YELLOW, tries % 2 == 0); delay(400); Serial.print('.');
  }
  setLED(PIN_LED_YELLOW, false); setLED(PIN_LED_GREEN, true);
  Serial.println();
  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("Dashboard: http://%s  (or http://%s.local)\n", WiFi.localIP().toString().c_str(), MDNS_NAME);
    MDNS.begin(MDNS_NAME);
  } else Serial.println(F("Router not connected. Join the hotspot and open http://192.168.4.1"));

  server.on("/", []() { server.send_P(200, "text/html", INDEX_HTML); });
  server.on("/api/data", handleApiData);
  server.on("/api/screen", handleSwitchScreen);
  server.on("/api/test", handleTestHardware);
  server.on("/api/mute", handleMute);
  server.on("/api/recal", handleRecal);
  server.begin();
}

void loop() {
  server.handleClient();
  pollVibration();                       // fast, edge-counted
  unsigned long now = millis();

  if (now - lastSensorPoll >= SENSOR_POLL_MS) {
    lastSensorPoll = now;
    evaluateFastSensors();
    evaluateOverall();
  }
  if (now - lastEnvPoll >= ENV_POLL_MS) {
    lastEnvPoll = now;
    evaluateEnvSensors();
    evaluateOverall();
  }
  if (now - lastOledPoll >= OLED_POLL_MS) {
    lastOledPoll = now;
    updateOLED();
  }
  if (WiFi.status() != WL_CONNECTED && now - lastWifiRetry > WIFI_RETRY_MS) {
    lastWifiRetry = now;
    WiFi.reconnect();
  }
  updateOutputs();
}
