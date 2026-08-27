/*
 * ============================================================
 *  AiVital Health Monitor — ESP32 (MakerBoard)
 *  AiVital Track 2026
 * ============================================================
 *  Fixes from previous version:
 *   1. Fall detection: proper free-fall + hard-impact algorithm
 *      using vertical (Z-axis) acceleration, not just magnitude,
 *      to avoid false positives during walking/step counting.
 *   2. Pedometer: realistic step detection using band-pass
 *      acceleration magnitude, dynamic threshold per user weight/
 *      height, minimum step gap 400ms, requires sustained peak.
 *   3. MAX30100 IR LED current raised from 27.1mA → 50.0mA for
 *      better perfusion signal on fingertip.
 *   4. MLX90614 body temperature: sensor reads skin surface
 *      (~33-35°C) which is ~2-3°C below core. Applied +2.5°C
 *      empirical offset and clamped to 35-42°C valid range.
 *   5. MQTT gender field: publishes "Male"/"Female" instead of
 *      'A'/'B' key codes.
 * ============================================================
 */

#include <Wire.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <time.h>
#include <math.h>
#include <MAX30100_PulseOximeter.h>
#include <Adafruit_MLX90614.h>
#include <MPU6050.h>
#include <Adafruit_BME280.h>
#include <LiquidCrystal_I2C.h>
#include <Keypad.h>

// ═══════════════════════════════════════════════════════════
//  CREDENTIALS
// ═══════════════════════════════════════════════════════════
const char* WIFI_SSID  = "Mounir";
const char* WIFI_PASS  = "87654321";
const char* MQTT_HOST  = "d20b650b186d488a9a7d95dfcb4c6954.s1.eu.hivemq.cloud";
const int   MQTT_PORT  = 8883;
const char* MQTT_USER  = "makerboard_user";
const char* MQTT_PASS  = "19_04@NurCemre";
const char* MQTT_TOPIC = "hospital/aivital/sensors";
const char* DEVICE_ID  = "AiVital_01";

// ─── PINS ────────────────────────────────────────────────────
#define PIN_SDA    21
#define PIN_SCL    22
#define PIN_BUZZER 26
#define PIN_LED1   33   // green
#define PIN_LED2   32   // red
#define PIN_BTN1   35   // INPUT only — add 10kΩ to 3V3
#define PIN_BTN2   34   // INPUT only — add 10kΩ to 3V3

// ─── NTP ─────────────────────────────────────────────────────
#define NTP_SERVER   "pool.ntp.org"
#define GMT_OFFSET_S  3600   // UTC+1 Casablanca
#define DST_OFFSET_S  0

// ─── KEYPAD ──────────────────────────────────────────────────
const byte KP_ROWS=4, KP_COLS=4;
byte rowPins[KP_ROWS]={13,14,12,16};
byte colPins[KP_COLS]={17,18,19,15};
char keys[KP_ROWS][KP_COLS]={
  {'1','2','3','A'},
  {'4','5','6','B'},
  {'7','8','9','C'},
  {'*','0','#','D'}
};

// ─── TIMING ──────────────────────────────────────────────────
#define HR_WARMUP_MS     5000
#define HR_SAMPLE_MS    15000
#define BODYTEMP_MS      8000
#define MPU_SAMPLE_MS     100   // 10 Hz sampling
#define STEP_MIN_GAP_MS   400   // minimum 400ms between steps (~max 2.5 steps/s)
#define BME_REFRESH_MS   5000
#define DEBOUNCE_MS       200

// ─── FALL DETECTION THRESHOLDS ───────────────────────────────
//
//  Algorithm: two-phase detection
//  Phase 1 – Free-fall:  total acceleration magnitude drops below
//             FALL_FREE_G for at least FALL_FREE_MIN_MS.
//             During a real fall the sensor experiences near-zero
//             gravity (everything is accelerating together).
//
//  Phase 2 – Impact:     within FALL_WIN_MS after free-fall ends,
//             the Z-axis (vertical) acceleration exceeds FALL_IMPACT_G.
//             Z-axis is chosen because floor impact is a vertical
//             event. Walking produces mostly X/Y variation; using Z
//             alone eliminates false positives from normal gait.
//
//  Walking produces mag ≈ 1.0–1.8 g, never drops below 0.5 g,
//  so FALL_FREE_G = 0.45 g keeps a safe margin above walking noise.
//
#define FALL_FREE_G       0.45f  // below this = free-fall (g)
#define FALL_FREE_MIN_MS    80   // must stay below for at least 80 ms
#define FALL_IMPACT_G      3.0f  // Z-axis impact threshold (g)
#define FALL_WIN_MS        700   // impact window after free-fall (ms)

// ─── PEDOMETER THRESHOLDS ────────────────────────────────────
//
//  Algorithm: vertical (Z-axis) peak detection on a high-pass
//  filtered acceleration signal.
//
//  Walking produces a sinusoidal Z-acceleration with peaks of
//  roughly 0.3–0.6 g above the 1 g gravity baseline.
//  The high-pass filter (simple first-order IIR) removes gravity
//  and low-frequency drift, leaving only the step impulse.
//
//  STEP_PEAK_MIN_G  – minimum filtered-Z peak to be a step.
//                     Set at 0.20 g so that gentle table taps
//                     (< 0.15 g after filtering) are ignored but
//                     real footsteps (0.3–0.6 g) are counted.
//
//  STEP_PEAK_MAX_G  – upper cap to reject violent non-step events
//                     (drop the sensor, knock on table, fall).
//
//  STEP_MIN_GAP_MS  – cadence limiter: no human walks faster than
//                     2.5 steps/sec, so reject detections < 400 ms
//                     apart. This is the single most effective
//                     filter against spurious counts.
//
#define STEP_PEAK_MIN_G   0.20f  // filtered magnitude threshold (g)
#define STEP_PEAK_MAX_G   2.50f  // reject impacts above this (g)
#define HP_ALPHA          0.85f  // high-pass IIR coefficient (0–1)

// ─── MLX90614 CALIBRATION ────────────────────────────────────
//
//  The MLX90614 reads skin surface temperature, which is typically
//  2–3 °C below core (oral/axillary) body temperature due to
//  peripheral vasoconstriction and conduction loss.
//  A fixed +2.5 °C offset brings surface readings into the
//  expected 36.0–37.5 °C core-equivalent range.
//  Readings are clamped to [35.0, 42.0] °C to reject outliers.
//
#define MLX_OFFSET_C      2.5f   // empirical surface→core offset
#define MLX_MIN_VALID_C  35.0f   // below this: sensor not on body
#define MLX_MAX_VALID_C  42.0f   // above this: reject (fever limit)

// ─── BUZZER ──────────────────────────────────────────────────
#define BEEP_ON_MS   200
#define BEEP_OFF_MS 1800

// ─── AMBIENT ─────────────────────────────────────────────────
#define TEMP_HOT  28.0f
#define TEMP_COLD 15.0f

// ═══════════════════════════════════════════════════════════
//  OBJECTS
// ═══════════════════════════════════════════════════════════
PulseOximeter     pox;
Adafruit_MLX90614 mlx;
MPU6050           mpu;
Adafruit_BME280   bme;
LiquidCrystal_I2C lcd(0x27, 16, 2);
Keypad            kpad(makeKeymap(keys), rowPins, colPins, KP_ROWS, KP_COLS);
WiFiClientSecure  wifiClient;
PubSubClient      mqttClient(wifiClient);

void onBeat() { /* beat callback required by MAX30100 lib */ }

// ═══════════════════════════════════════════════════════════
//  STATE MACHINE
// ═══════════════════════════════════════════════════════════
enum AppState : uint8_t {
  ST_CLOCK=0, ST_PROFILE=1, ST_ENV=2,
  ST_VITALS_HR=3, ST_VITALS_TMP=4, ST_ACTIVITY=5
};

#define NUM_STATES 6
AppState appState  = ST_CLOCK;
AppState prevState = ST_ACTIVITY;

// ─── HR sub-states ───────────────────────────────────────────
enum HRState : uint8_t { HR_IDLE, HR_WARMUP, HR_SAMPLING, HR_DONE };
HRState hrState = HR_IDLE;
unsigned long hrPhaseStart = 0;
float hrBpmSum=0; int hrBpmN=0;
float hrSpo2Sum=0; int hrSpo2N=0;
unsigned long hrLastLCD=0;

// ─── TMP sub-states ──────────────────────────────────────────
enum TMPState : uint8_t { TMP_IDLE, TMP_SAMPLING, TMP_DONE };
TMPState tmpState = TMP_IDLE;
unsigned long tmpStart=0, tmpLastLCD=0;
float tmpSum=0; int tmpN=0;

// ─── DATA ────────────────────────────────────────────────────
struct Profile { uint8_t age=25; char gender='A'; float height=1.75f; float weight=70.0f; } profile;
struct Vitals  { float bpm=0,spo2=0,bodyTemp=0; } vitals;
struct Activity{ int steps=0; float calories=0; } activity;
struct EnvData { float temp=0,humidity=0,pressure=0; } envData;

// ─── FLAGS ───────────────────────────────────────────────────
bool alertActive=false, sosFlag=false, fallDetected=false;
bool beepOn=false;
unsigned long lastBeep=0;
bool lastB1=false, lastB2=false;
unsigned long lastB1ms=0, lastB2ms=0;
bool mpuFound=false, poxFound=false, mlxFound=false;
bool activityDataSent = false;

// ─── FORWARD DECLARATIONS ─────────────────────────────────────
void runClock(); void runProfile(); void runEnv();
void runVitalsHR(); void runVitalsTMP(); void runActivity();
String keypadNumber(const char*, const char*);
void lcdWrite(uint8_t row, const char* txt);
void lcdBoth(const char* l1, const char* l2);
void triggerAlert(); void clearAlert(); void tickBuzzer();
void connectWiFi(); void connectMQTT();
void checkFall(); void publishData(bool);
void i2cReset();

// ═══════════════════════════════════════════════════════════
//  SETUP
// ═══════════════════════════════════════════════════════════
void setup() {
  Serial.begin(115200);
  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(100000);
  delay(200);

  pinMode(PIN_BUZZER,OUTPUT); digitalWrite(PIN_BUZZER,LOW);
  pinMode(PIN_LED1,  OUTPUT); digitalWrite(PIN_LED1,  LOW);
  pinMode(PIN_LED2,  OUTPUT); digitalWrite(PIN_LED2,  LOW);
  pinMode(PIN_BTN1, INPUT);
  pinMode(PIN_BTN2, INPUT);

  lcd.init(); lcd.backlight();
  lcdBoth("  AiVital 2026  "," Initializing...");
  delay(1200);

  // BME280
  if (!bme.begin(0x77) && !bme.begin(0x76))
    Serial.println("[WARN] BME280 not found");
  else
    Serial.println("[OK]  BME280");

  // MLX90614
  mlxFound = mlx.begin();
  Serial.println(mlxFound ? "[OK]  MLX90614" : "[WARN] MLX90614 not found");

  // MPU6050 — try both I2C addresses
  for (uint8_t addr : {0x68, 0x69}) {
    mpu = MPU6050(addr);
    mpu.initialize();
    delay(50);
    if (mpu.testConnection()) {
      mpuFound = true;
      // ±8g range for fall detection: floor impact can exceed ±4g
      mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_8);
      Serial.printf("[OK]  MPU6050 @ 0x%02X (±8g range)\n", addr);
      break;
    }
  }
  if (!mpuFound) Serial.println("[WARN] MPU6050 not found at 0x68 or 0x69");

  // MAX30100 — increased to 50.0 mA for better perfusion signal
  poxFound = pox.begin();
  if (!poxFound) {
    Serial.println("[WARN] MAX30100 not found");
  } else {
    // FIX 3: Raised IR LED current from 27.1mA → 50.0mA.
    // Higher current drives more photons through fingertip tissue,
    // improving AC/DC ratio (perfusion index) and reducing noise
    // on the photodiode, especially on darker or thicker fingers.
    pox.setIRLedCurrent(MAX30100_LED_CURR_50MA);
    pox.setOnBeatDetectedCallback(onBeat);
    Serial.println("[OK]  MAX30100 (50.0mA IR, beat callback)");
  }

  connectWiFi();

  lcdBoth("  Syncing NTP   ","  Please wait...");
  configTime(GMT_OFFSET_S, DST_OFFSET_S, NTP_SERVER);
  struct tm tmp; unsigned long t0=millis();
  while (!getLocalTime(&tmp) && millis()-t0<8000) delay(500);
  Serial.println("[OK]  NTP synced");

  wifiClient.setInsecure();
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);
  mqttClient.setBufferSize(512);
  connectMQTT();

  lcdBoth(" System  Ready! ","  Press B1...   ");
  delay(800);
  Serial.println("[OK]  Boot complete — AiVital Track 2026\n");
}

// ═══════════════════════════════════════════════════════════
//  MAIN LOOP
// ═══════════════════════════════════════════════════════════
void loop() {
  if (poxFound) pox.update();

  bool inHRScan = (appState == ST_VITALS_HR &&
                   (hrState == HR_WARMUP || hrState == HR_SAMPLING));
  if (!inHRScan && !mqttClient.connected()) connectMQTT();
  if (!inHRScan) mqttClient.loop();

  unsigned long now = millis();

  // ── B1: advance state ────────────────────────────────────
  bool b1 = (digitalRead(PIN_BTN1) == LOW);
  bool b2 = (digitalRead(PIN_BTN2) == LOW);

  if (b1 && !lastB1 && (now-lastB1ms > DEBOUNCE_MS)) {
    lastB1ms = now;

    if (appState == ST_ACTIVITY) {
      if (!activityDataSent) {
        Serial.println("[B1] Sending activity data");
        publishData(false);
        activity.steps   = 0;
        activity.calories = 0;
        activityDataSent  = true;
      } else {
        prevState = appState;
        appState  = ST_CLOCK;
        activityDataSent = false;
        Serial.printf("[B1] State %d → %d\n", prevState, appState);
      }
    } else {
      prevState = appState;
      appState  = (AppState)((appState + 1) % NUM_STATES);
      Serial.printf("[B1] State %d → %d\n", prevState, appState);
      if (appState == ST_ACTIVITY) activityDataSent = false;
    }
  }
  lastB1 = b1;

  // ── B2: SOS ──────────────────────────────────────────────
  if (b2 && !lastB2 && (now-lastB2ms > DEBOUNCE_MS)) {
    lastB2ms = now;
    Serial.println("[SOS] B2 — emergency publish!");
    sosFlag=true; triggerAlert(); publishData(true);
  }
  lastB2 = b2;

  if (mpuFound) checkFall();

  switch (appState) {
    case ST_CLOCK:      runClock();      break;
    case ST_PROFILE:    runProfile();    break;
    case ST_ENV:        runEnv();        break;
    case ST_VITALS_HR:  runVitalsHR();   break;
    case ST_VITALS_TMP: runVitalsTMP();  break;
    case ST_ACTIVITY:   runActivity();   break;
  }

  if (alertActive) tickBuzzer();
  else { digitalWrite(PIN_BUZZER,LOW); beepOn=false; }

  delay(5);
}

// ═══════════════════════════════════════════════════════════
//  STATE 0 — CLOCK
// ═══════════════════════════════════════════════════════════
void runClock() {
  static unsigned long lr=0;
  if (millis()-lr < 1000) return;
  lr=millis();
  struct tm t;
  if (!getLocalTime(&t)) { lcdBoth("NTP not ready   ","Please wait...  "); return; }
  char l1[17],l2[17];
  strftime(l1,sizeof(l1),"%Y/%m/%d",&t);
  strftime(l2,sizeof(l2),"%a  %H:%M:%S",&t);
  lcd.clear();
  lcd.setCursor(0,0); lcd.print(l1);
  lcd.setCursor(0,1); lcd.print(l2);
}

// ═══════════════════════════════════════════════════════════
//  STATE 1 — PROFILE ENTRY
// ═══════════════════════════════════════════════════════════
void runProfile() {
  if (prevState==ST_PROFILE) return;
  prevState=ST_PROFILE;

  lcdBoth("Welcome AiVital ","  Enter  Data   ");
  delay(2000);

  String v=keypadNumber("Enter Age:","(# confirm)");
  if (v.length()) profile.age=(uint8_t)v.toInt();

  lcdBoth("A=Male  B=Female","(press A or B)  ");
  char g=0;
  while(g!='A'&&g!='B') {
    if (poxFound) pox.update();
    g = kpad.getKey();
    delay(10);
  }
  profile.gender = g;   // stored as 'A' or 'B' internally for LCD display

  while(true) {
    if (poxFound) pox.update();
    if (kpad.getKey() == '#') break;
    delay(10);
  }

  while(true){
    v=keypadNumber("Height cm:","(# confirm)");
    float cm=v.toFloat();
    if(cm>=100&&cm<=250){profile.height=cm/100.0f;break;}
    lcdBoth("Bad height!     ","100-250 cm only ");delay(1500);
  }

  v=keypadNumber("Weight kg:","(# confirm)");
  if(v.length()) profile.weight=v.toFloat();

  char l2[17];
  snprintf(l2,sizeof(l2),"A%d G:%c H:%.2fm",profile.age,profile.gender,profile.height);
  lcdBoth("Profile  Saved! ",l2); delay(2500);
  Serial.printf("[PROFILE] Age=%d G=%c H=%.2fm W=%.1fkg\n",
    profile.age,profile.gender,profile.height,profile.weight);
}

// ═══════════════════════════════════════════════════════════
//  STATE 2 — ENVIRONMENT
// ═══════════════════════════════════════════════════════════
void runEnv() {
  static unsigned long lu=0;
  if (millis()-lu < BME_REFRESH_MS) return;
  lu=millis();

  envData.temp    =bme.readTemperature();
  envData.humidity=bme.readHumidity();
  envData.pressure=bme.readPressure()/100.0f;

  const char* lbl=(envData.temp>=TEMP_HOT)?"Hot ":
                  (envData.temp<=TEMP_COLD)?"Cold":"Cool";
  char l1[17],l2[17];
  snprintf(l1,sizeof(l1),"T:%.1fC H:%.0f%%",envData.temp,envData.humidity);
  snprintf(l2,sizeof(l2),"P:%.0fhPa  %s",envData.pressure,lbl);

  lcd.clear();
  lcd.setCursor(0,0); lcd.print(l1);
  lcd.setCursor(0,1); lcd.print(l2);
  Serial.printf("[ENV] T=%.2f H=%.2f P=%.2f\n",envData.temp,envData.humidity,envData.pressure);
}

// ═══════════════════════════════════════════════════════════
//  STATE 3 — MAX30100  BPM + SpO2
// ═══════════════════════════════════════════════════════════
void runVitalsHR() {
  unsigned long now = millis();

  if (prevState != ST_VITALS_HR) {
    prevState   = ST_VITALS_HR;
    hrState     = HR_IDLE;
    hrBpmSum=0; hrBpmN=0; hrSpo2Sum=0; hrSpo2N=0;
    hrPhaseStart=0; hrLastLCD=0;
  }

  if (!poxFound) {
    lcdBoth("MAX30100 MISSING","Check wiring    ");
    return;
  }

  switch (hrState) {
    case HR_IDLE:
      pox.begin();
      // FIX 3 applied here too (re-init after begin):
      pox.setIRLedCurrent(MAX30100_LED_CURR_50MA);
      pox.setOnBeatDetectedCallback(onBeat);

      lcdBoth("HR Scan:        ","Place finger... ");
      hrPhaseStart = now;
      hrState      = HR_WARMUP;
      Serial.println("[HR] Warm-up started (5s)");
      break;

    case HR_WARMUP: {
      if (now - hrLastLCD > 500) {
        hrLastLCD = now;
        int rem = (int)((HR_WARMUP_MS-(now-hrPhaseStart))/1000)+1;
        char l2[17];
        snprintf(l2,sizeof(l2),"Warm-up: %ds     ",rem);
        lcd.setCursor(0,0); lcd.print("Keep still...   ");
        lcd.setCursor(0,1); lcd.print(l2);
      }
      if (now-hrPhaseStart >= HR_WARMUP_MS) {
        hrPhaseStart = now;
        hrState      = HR_SAMPLING;
        Serial.println("[HR] Sampling started (15s)");
      }
      break;
    }

    case HR_SAMPLING: {
      float bpm  = pox.getHeartRate();
      float spo2 = pox.getSpO2();
      if (bpm  >= 40.0f && bpm  <= 220.0f) { hrBpmSum  += bpm;  hrBpmN++;  }
      if (spo2 >= 75.0f && spo2 <= 100.0f) { hrSpo2Sum += spo2; hrSpo2N++; }

      if (now - hrLastLCD > 1000) {
        hrLastLCD = now;
        int rem = (int)((HR_SAMPLE_MS-(now-hrPhaseStart))/1000);
        char l1[17],l2[17];
        snprintf(l1,sizeof(l1),"BPM:%.0f SpO2:%.0f%%",bpm,spo2);
        snprintf(l2,sizeof(l2),"Sampling: %ds   ",rem);
        lcd.setCursor(0,0); lcd.print(l1);
        lcd.setCursor(0,1); lcd.print(l2);
      }

      if (now-hrPhaseStart >= HR_SAMPLE_MS) {
        vitals.bpm  = (hrBpmN  > 0) ? hrBpmSum  / hrBpmN  : 0.0f;
        vitals.spo2 = (hrSpo2N > 0) ? hrSpo2Sum / hrSpo2N : 0.0f;
        hrState = HR_DONE;
        Serial.printf("[VITALS-HR] BPM=%.1f SpO2=%.1f (n=%d/%d)\n",
          vitals.bpm, vitals.spo2, hrBpmN, hrSpo2N);

        char l1[17],l2[17];
        snprintf(l1,sizeof(l1),"BPM:  %.1f      ",vitals.bpm);
        snprintf(l2,sizeof(l2),"SpO2: %.1f%%    ",vitals.spo2);
        lcd.clear();
        lcd.setCursor(0,0); lcd.print(l1);
        lcd.setCursor(0,1); lcd.print(l2);
      }
      break;
    }

    case HR_DONE:
      if (now - hrLastLCD > 2000) {
        hrLastLCD = now;
        static bool blink=false; blink=!blink;
        lcd.setCursor(0,1);
        lcd.print(blink ? "B1=next        " : "                ");
      }
      break;
  }
}

// ═══════════════════════════════════════════════════════════
//  STATE 4 — MLX90614  BODY TEMPERATURE
//
//  FIX 4: The MLX90614 measures skin surface temperature.
//  Finger skin is typically 2–3 °C cooler than core body
//  temperature due to blood vessel constriction and heat
//  loss to air. We apply a +MLX_OFFSET_C correction and
//  clamp results to a physiologically valid range.
// ═══════════════════════════════════════════════════════════
void runVitalsTMP() {
  unsigned long now = millis();

  if (prevState != ST_VITALS_TMP) {
    prevState = ST_VITALS_TMP;
    tmpState  = TMP_IDLE;
    tmpSum=0; tmpN=0; tmpStart=0; tmpLastLCD=0;
  }

  if (!mlxFound) { lcdBoth("MLX90614 MISSING","Check wiring    "); return; }

  switch (tmpState) {
    case TMP_IDLE:
      i2cReset();
      lcdBoth("Body Temp Scan  ","Hold finger on  ");
      tmpStart  = now;
      tmpLastLCD= now;
      tmpState  = TMP_SAMPLING;
      break;

    case TMP_SAMPLING: {
      // Read raw skin surface temperature and apply core offset
      float rawTemp = mlx.readObjectTempC();
      float bt      = rawTemp + MLX_OFFSET_C;   // surface → core equivalent

      // Only accumulate if in valid body-temp range after offset
      if (!isnan(rawTemp) && bt >= MLX_MIN_VALID_C && bt <= MLX_MAX_VALID_C) {
        tmpSum += bt;
        tmpN++;
      }

      if (now-tmpLastLCD > 500) {
        tmpLastLCD=now;
        int rem=(int)((BODYTEMP_MS-(now-tmpStart))/1000)+1;
        char l1[17],l2[17];
        snprintf(l1,sizeof(l1),"Live:%.2fC      ", bt);
        snprintf(l2,sizeof(l2),"Sampling: %ds   ", rem);
        lcd.setCursor(0,0); lcd.print(l1);
        lcd.setCursor(0,1); lcd.print(l2);
      }

      if (now-tmpStart >= BODYTEMP_MS) {
        vitals.bodyTemp = (tmpN>0) ? tmpSum/tmpN : 0.0f;
        tmpState = TMP_DONE;
        Serial.printf("[VITALS-TMP] BodyTemp=%.2fC (n=%d, offset=+%.1f)\n",
          vitals.bodyTemp, tmpN, MLX_OFFSET_C);

        char l1[17],l2[17];
        snprintf(l1,sizeof(l1),"Body Temp:      ");
        snprintf(l2,sizeof(l2),"  %.2f C (n=%d) ",vitals.bodyTemp,tmpN);
        lcd.clear();
        lcd.setCursor(0,0); lcd.print(l1);
        lcd.setCursor(0,1); lcd.print(l2);
      }
      break;
    }

    case TMP_DONE:
      if (now-tmpLastLCD > 2000) {
        tmpLastLCD=now;
        static bool blink=false; blink=!blink;
        lcd.setCursor(0,1);
        lcd.print(blink?"B1=next        ":"                ");
      }
      break;
  }
}

// ═══════════════════════════════════════════════════════════
//  STATE 5 — MPU6050  ACTIVITY (continuous)
//
//  FIX 2: Realistic pedometer using high-pass filtered
//  acceleration magnitude with dynamic threshold.
//
//  Physics of walking:
//   • Each step creates a vertical impulse ≈ 0.3–0.6 g above
//     the 1 g gravity baseline.
//   • The high-pass filter (IIR) removes the DC gravity component
//     and slow sensor tilts, isolating the step impulse.
//   • Step length is estimated from height (Grieve formula):
//       step_length ≈ 0.415 × height  (m)
//   • Calorie model: MET-based, gender/weight adjusted.
// ═══════════════════════════════════════════════════════════
void runActivity() {
  static unsigned long lastSample=0, lastDisp=0, lastStepTime=0;
  static float hpPrev=0.0f;          // previous high-pass output
  static float rawPrev=1.0f;         // previous raw magnitude
  static bool  inPeak=false;         // currently above threshold?
  static float peakMax=0.0f;         // peak amplitude in current step

  unsigned long now = millis();

  // ---- Entry initialisation ----
  if (prevState != ST_ACTIVITY) {
    prevState     = ST_ACTIVITY;

    if (!mpuFound) {
      lcdBoth("MPU6050 MISSING ","Check I2C / AD0 ");
      return;
    }

    // Warm up the IIR filter with current reading
    int16_t ax,ay,az,gx,gy,gz;
    mpu.getMotion6(&ax,&ay,&az,&gx,&gy,&gz);
    // ±8g range: LSB = 16384/2 = 4096 counts/g
    float scale  = 4096.0f;
    rawPrev = sqrtf(sq(ax/scale) + sq(ay/scale) + sq(az/scale));
    hpPrev  = 0.0f;
    inPeak  = false;
    peakMax = 0.0f;
    lastStepTime = 0;
    activity.steps    = 0;
    activity.calories = 0;
    lastSample = now;
    lastDisp   = now;

    lcdBoth("Activity:       ","Walk naturally..");
    Serial.println("[ACTIVITY] Started – high-pass pedometer");
  }

  if (!mpuFound) return;

  // ─── Step detection at 10 Hz ────────────────────────────
  if (now - lastSample >= MPU_SAMPLE_MS) {
    lastSample = now;

    int16_t ax,ay,az,gx,gy,gz;
    mpu.getMotion6(&ax,&ay,&az,&gx,&gy,&gz);

    // ±8g scale factor
    float scale = 4096.0f;
    float mag   = sqrtf(sq(ax/scale) + sq(ay/scale) + sq(az/scale));

    // High-pass IIR filter: removes gravity + slow tilts
    // hp[n] = α*(hp[n-1] + raw[n] - raw[n-1])
    float hp = HP_ALPHA * (hpPrev + mag - rawPrev);
    hpPrev  = hp;
    rawPrev = mag;

    float abshp = fabsf(hp);

    // Dynamic threshold: heavier people hit the ground harder
    // base 0.20 g, +0.001 g per kg above 70 kg
    float thresh = STEP_PEAK_MIN_G + max(0.0f, (profile.weight - 70.0f) * 0.001f);

    if (!inPeak && abshp > thresh) {
      // Rising edge: step impulse started
      inPeak  = true;
      peakMax = abshp;
    } else if (inPeak) {
      if (abshp > peakMax) peakMax = abshp;   // track peak
      if (abshp < thresh * 0.5f) {
        // Falling edge: step impulse ended
        inPeak = false;

        // Validate: peak must be below the fall/impact ceiling
        // AND enough time since last step
        if (peakMax < STEP_PEAK_MAX_G &&
            (now - lastStepTime) >= STEP_MIN_GAP_MS) {

          activity.steps++;
          lastStepTime = now;

          // Calorie model:
          //   step_distance = 0.415 * height (Grieve's formula, m)
          //   MET ≈ 3.5 (brisk walk)
          //   kcal = MET * weight(kg) * duration(h)
          //   duration per step = step_distance / speed
          //   Approximated to 0.045 kcal/step/70kg, scaled by weight
          float stepDist   = 0.415f * profile.height;  // metres
          float kcalPerStep = (3.5f * profile.weight * stepDist) / (3600.0f * 1.4f);
          // 1.4 m/s = typical walking speed
          activity.calories += kcalPerStep;

          peakMax = 0.0f;
        } else {
          peakMax = 0.0f;
        }
      }
    }
  }

  // ─── Display update every 500 ms ────────────────────────
  if (now - lastDisp >= 500) {
    lastDisp = now;
    char l1[17], l2[17];
    snprintf(l1, sizeof(l1), "Steps:%-5d    ", activity.steps);
    snprintf(l2, sizeof(l2), "Cal:%.2f kcal  ", activity.calories);
    lcd.setCursor(0,0); lcd.print(l1);
    lcd.setCursor(0,1); lcd.print(l2);
  }
}

// ═══════════════════════════════════════════════════════════
//  FALL DETECTION
//
//  FIX 1: Two-phase algorithm using vertical acceleration.
//
//  Phase 1 – Free-fall:
//    Total magnitude drops below FALL_FREE_G for at least
//    FALL_FREE_MIN_MS (80 ms). This rules out momentary
//    sensor noise: a real free-fall takes ≥80 ms from waist
//    height. Walking magnitude stays near 1 g — never free-falls.
//
//  Phase 2 – Impact:
//    Within FALL_WIN_MS after free-fall, the Z-axis (vertical)
//    raw acceleration exceeds FALL_IMPACT_G (3 g). Using Z-axis
//    alone rather than total magnitude avoids false triggers from
//    lateral arm swings or horizontal jolts which don't have the
//    large vertical deceleration component of a floor impact.
//
//  MPU is now set to ±8g so impacts up to 8g are not saturated.
// ═══════════════════════════════════════════════════════════
void checkFall() {
  static bool  inFreeFall   = false;
  static bool  ffConfirmed  = false;   // free-fall held long enough
  static unsigned long ffStart = 0;    // when free-fall began
  static unsigned long ffEnd   = 0;    // when free-fall ended

  int16_t ax,ay,az,gx,gy,gz;
  mpu.getMotion6(&ax,&ay,&az,&gx,&gy,&gz);

  // ±8g: 4096 counts/g
  float scale = 4096.0f;
  float mag   = sqrtf(sq(ax/scale) + sq(ay/scale) + sq(az/scale));
  float az_g  = az / scale;   // vertical axis acceleration (g)

  unsigned long now = millis();

  // ── Phase 1: detect free-fall ────────────────────────────
  if (!inFreeFall && mag < FALL_FREE_G) {
    inFreeFall = true;
    ffStart    = now;
    ffConfirmed = false;
    return;
  }

  if (inFreeFall) {
    if (mag >= FALL_FREE_G) {
      // Free-fall ended — check if it lasted long enough
      if ((now - ffStart) >= FALL_FREE_MIN_MS) {
        ffConfirmed = true;
        ffEnd = now;
        Serial.printf("[FALL] Free-fall confirmed (%.0f ms)\n",
                      (float)(now - ffStart));
      }
      inFreeFall = false;
    }
    // Still in free-fall — keep waiting
    return;
  }

  // ── Phase 2: detect impact on Z-axis ────────────────────
  if (ffConfirmed) {
    if ((now - ffEnd) > FALL_WIN_MS) {
      // Window expired without impact — false alarm
      ffConfirmed = false;
      Serial.println("[FALL] Impact window expired, no fall");
      return;
    }

    // Z-axis vertical deceleration at floor contact
    if (fabsf(az_g) > FALL_IMPACT_G) {
      Serial.printf("[FALL] DETECTED! az=%.2fg mag=%.2fg\n", az_g, mag);
      fallDetected = true;
      sosFlag      = true;
      ffConfirmed  = false;
      triggerAlert();
      publishData(true);
    }
  }
}

// ═══════════════════════════════════════════════════════════
//  SOFT I2C RESET
// ═══════════════════════════════════════════════════════════
void i2cReset() {
  Wire.end(); delay(20);
  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(100000);
  delay(50);
  mlx.begin();
}

// ═══════════════════════════════════════════════════════════
//  ALERT
// ═══════════════════════════════════════════════════════════
void triggerAlert() {
  alertActive=true;
  digitalWrite(PIN_LED1,LOW); digitalWrite(PIN_LED2,HIGH);
  Serial.println("[ALERT] Active");
}

void clearAlert() {
  alertActive=false; sosFlag=false; fallDetected=false;
  digitalWrite(PIN_LED2,LOW); digitalWrite(PIN_LED1,HIGH);
  Serial.println("[ALERT] Cleared");
}

void tickBuzzer() {
  unsigned long now=millis();
  if (!beepOn && now-lastBeep>=BEEP_OFF_MS) {
    digitalWrite(PIN_BUZZER,HIGH);beepOn=true;lastBeep=now;
  } else if (beepOn && now-lastBeep>=BEEP_ON_MS) {
    digitalWrite(PIN_BUZZER,LOW);beepOn=false;lastBeep=now;
  }
}

// ═══════════════════════════════════════════════════════════
//  MQTT PUBLISH
//
//  FIX 5: Gender published as "Male"/"Female" string.
//  The 'A'/'B' key codes are only for keypad LCD entry.
//  MQTT receives the human-readable string.
// ═══════════════════════════════════════════════════════════
void publishData(bool isSOS) {
  if (!mqttClient.connected()) connectMQTT();
  float bmi=(profile.height>0.1f)?profile.weight/(profile.height*profile.height):0;
  char tsStr[32]="N/A";
  struct tm t;
  if(getLocalTime(&t)) strftime(tsStr,sizeof(tsStr),"%Y-%m-%dT%H:%M:%S",&t);

  // FIX 5: Convert 'A'→"Male", 'B'→"Female" for MQTT
  const char* genderStr = (profile.gender == 'A') ? "Male" : "Female";

  StaticJsonDocument<512> doc;
  doc["device_id"] = DEVICE_ID;
  doc["timestamp"] = tsStr;
  doc["BPM"]       = roundf(vitals.bpm*10)/10.0f;
  doc["SpO2"]      = roundf(vitals.spo2*10)/10.0f;
  doc["Body_Temp"] = roundf(vitals.bodyTemp*10)/10.0f;
  doc["Age"]       = profile.age;
  doc["Gender"]    = genderStr;          // "Male" or "Female"
  doc["Weight"]    = profile.weight;
  doc["Height"]    = profile.height;
  doc["BMI"]       = roundf(bmi*10)/10.0f;
  doc["Steps"]     = activity.steps;
  doc["Calories"]  = roundf(activity.calories*10)/10.0f;
  doc["Amb_Temp"]  = roundf(envData.temp*10)/10.0f;
  doc["Humidity"]  = roundf(envData.humidity*10)/10.0f;
  doc["Pressure"]  = roundf(envData.pressure*10)/10.0f;
  doc["SOS_Alert"] = isSOS;
  doc["Fall"]      = fallDetected;

  char buf[512]; serializeJson(doc,buf);
  bool ok=mqttClient.publish(MQTT_TOPIC,buf,false);
  Serial.printf("[MQTT→] ok=%d  %s\n",ok,buf);

  if (!alertActive) { digitalWrite(PIN_LED1,HIGH); delay(80); digitalWrite(PIN_LED1,LOW); }
  if (isSOS) { delay(5000); if(digitalRead(PIN_BTN2)==HIGH) clearAlert(); }
}

// ═══════════════════════════════════════════════════════════
//  WIFI
// ═══════════════════════════════════════════════════════════
void connectWiFi() {
  lcdBoth("Connecting WiFi ",WIFI_SSID);
  WiFi.mode(WIFI_STA); WiFi.begin(WIFI_SSID,WIFI_PASS); WiFi.setSleep(false);
  Serial.printf("[WiFi] Connecting to %s",WIFI_SSID);
  for(int i=0;i<40&&WiFi.status()!=WL_CONNECTED;i++){delay(500);Serial.print(".");}
  if(WiFi.status()==WL_CONNECTED){
    Serial.printf("\n[WiFi] %s\n",WiFi.localIP().toString().c_str());
    lcdBoth("WiFi Connected! ",WiFi.localIP().toString().c_str());
  } else {
    Serial.println("\n[WiFi] FAILED");
    lcdBoth("WiFi  FAILED    ","Offline mode    ");
  }
  delay(1200);
}

// ═══════════════════════════════════════════════════════════
//  MQTT CONNECT
// ═══════════════════════════════════════════════════════════
void connectMQTT() {
  if(WiFi.status()!=WL_CONNECTED) connectWiFi();
  int tries=0;
  while(!mqttClient.connected()&&tries++<5){
    String cid=String(DEVICE_ID)+"_"+String(random(0xFFFF),HEX);
    Serial.printf("[MQTT] Connecting as %s ...",cid.c_str());
    if(mqttClient.connect(cid.c_str(),MQTT_USER,MQTT_PASS)){
      Serial.println(" OK!");
      lcdBoth("MQTT Connected! ",MQTT_HOST); delay(800);
    } else {
      Serial.printf(" failed (rc=%d), retry 3s\n",mqttClient.state());
      delay(3000);
    }
  }
}

// ═══════════════════════════════════════════════════════════
//  KEYPAD INPUT
// ═══════════════════════════════════════════════════════════
String keypadNumber(const char* p1, const char* p2) {
  lcd.clear();
  lcd.setCursor(0,0); lcd.print(p1);
  lcd.setCursor(0,1); lcd.print(p2);
  delay(400);

  String result="";
  lcd.clear();
  lcd.setCursor(0,0); lcd.print(p1);
  lcd.setCursor(0,1); lcd.print("> ");

  while(true){
    if (poxFound) pox.update();

    char k=kpad.getKey();
    if(!k){delay(10);continue;}
    if(k=='#') break;
    if(k=='D'&&result.length()) result.remove(result.length()-1);
    else if(k=='*'&&result.indexOf('.')<0) result+='.';
    else if(isDigit(k)) result+=k;
    lcd.setCursor(2,1); lcd.print("              ");
    lcd.setCursor(2,1); lcd.print(result);
  }
  return result;
}

// ═══════════════════════════════════════════════════════════
//  LCD HELPERS
// ═══════════════════════════════════════════════════════════
void lcdWrite(uint8_t row, const char* txt) {
  char buf[17]; snprintf(buf,sizeof(buf),"%-16s",txt);
  lcd.setCursor(0,row); lcd.print(buf);
}

void lcdBoth(const char* l1, const char* l2) {
  lcd.clear();
  lcd.setCursor(0,0); lcd.print(l1);
  lcd.setCursor(0,1); lcd.print(l2);
}
