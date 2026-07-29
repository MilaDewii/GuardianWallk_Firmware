#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <Preferences.h>   // NVS -- cache nomor caregiver biar tetap ada walau Firebase gak bisa diakses

// ── GSM (SIM808) — SEKARANG HANYA UNTUK SMS & GPS, BUKAN DATA ──────
// PENTING: TINY_GSM_MODEM_xxx HARUS didefinisikan SEBELUM include TinyGsmClient.h
// TinyGsm dipakai buat modem.init(), modem.sendSMS(), modem.getSignalQuality() --
// SEMUA jalur data GPRS (ensureGPRS, TinyGsmClient sebagai HTTP transport,
// ESP_SSLClient) sudah DIHAPUS. Kalau WiFi mati, device TIDAK lagi coba
// nyambung data lewat GPRS -- cadangannya sekarang murni SMS (4 trigger:
// jatuh, keluar geofence, offline lama, anomali ekstrem/berulang).
#define TINY_GSM_MODEM_SIM800
#include <TinyGsmClient.h>

#include <time.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <MPU6050.h>
#include <TinyGPS++.h>
#include <HardwareSerial.h>
#include <FirebaseJson.h>   // standalone JSON builder — TIDAK butuh WiFi/Firebase library

// ============================================================
// KONFIGURASI
// ============================================================
#define WIFI_SSID     "ESP32"
#define WIFI_PASSWORD "kiaaaaaa"

#define FIREBASE_HOST "smartwalker-3480e-default-rtdb.firebaseio.com"
#define FIREBASE_AUTH "aQrMeNP3fBeZsjXqn3E6fD0LNmllIPEAgXHoEHCF"   // legacy token / database secret
#define DB_PATH       "/Walkers/walker_001"

// ── SMS darurat (last resort saat WiFi gak nyambung) ──
// Dipakai HANYA kalau nomor caregiver belum pernah berhasil disinkron dari
// Firebase (NVS kosong) -- begitu ada sinkron pertama, nomor caregiver asli
// dari aplikasi (disimpan di Firebase "/config/caregiver_phone") yang dipakai.
#define SMS_RECIPIENT_DEFAULT "+6281229129479"

#define IV_HB         3000
#define IV_SEN        500
#define IV_GPS        5000
#define IV_GEO        5000
#define IV_GEO_RELOAD 30000
#define IV_TIME_RETRY 60000   // kalau waktu masih belum sinkron, coba lagi tiap 60s

#define TRIG_FRONT  33
#define ECHO_FRONT  32
#define TRIG_BACK   2
#define ECHO_BACK   15
#define BACK_SMOOTH_SIZE 5
#define BUZZER_PIN  23
#define SIM808_RX   16
#define SIM808_TX   17
#define SIM808_BAUD 9600

#define OBSTACLE_TH       50
#define USER_ENTER_MIN_CM  10
#define USER_ENTER_MAX_CM  100
#define USER_EXIT_MIN_CM   15
#define USER_EXIT_MAX_CM   110
#define NO_USER_ALARM_DELAY 12000
// ── FIX: jangkauan fisik maksimum HC-SR04. Bacaan di atas
// ini PASTI noise/echo palsu, bukan pantulan objek asli — dipakai buat
// menyaring bacaan mustahil di readSensors() (lihat komentar di sana).
#define BACK_MAX_VALID_CM 400.0f
#define BACK_STUCK_TIMEOUT_MS 3000

#define BUZZ_FREQ_OBSTACLE 2000
#define BUZZ_FREQ_NOUSER   1000
#define NOTIF_COOLDOWN 100000

#define OBSTACLE_DEBOUNCE_COUNT 3
#define NOUSER_DEBOUNCE_COUNT   6

// ============================================================
// KONFIGURASI STEP COUNTER
// ============================================================
#define STEP_SAMPLE_INTERVAL_MS 10
#define SMOOTH_SIZE              4
#define DEBOUNCE_MS             400
#define BASELINE_ALPHA          0.005f
#define CALIBRATION_MS          2000
#define STEP_REARM_MIN_MS 120 

// ── FIX STEP COUNTER (bagian yang diubah) ──────────────────
#define STEP_MIN_DELTA   0.035f   // lantai minimum ambang (g) -- di atas noise floor umum MPU6050
#define STEP_NOISE_MULT  6.0f     // dynamicPeakDelta = max(STEP_MIN_DELTA, noiseStdDev * ini)
// ── ADAPTIVE LOCKOUT setelah step keras ─────────────────────
#define STEP_HARD_RATIO      1.6f   // kalau puncak > dynamicPeakDelta * ini -> "step keras"
#define STEP_HARD_REARM_MS   350    // rearm delay diperpanjang utk step keras
#define STEP_HARD_EXIT_RATIO 0.25f  // exit level diturunkan (lebih dekat baseline) utk step keras
#define STEP_HARD_DEBOUNCE_MS 700  // jarak minimum antar-step diperpanjang utk step keras


// ── FIX STEP COUNTER: HYSTERESIS ────────────────────────────
#define STEP_EXIT_RATIO  0.45f

// ============================================================
// KONFIGURASI FUZZY FALL DETECTION
// ============================================================
#define FALL_PRE_FILTER_TH    0.07f
#define FALL_FUZZY_THRESHOLD  0.45f
#define FALL_HOLD_DURATION_MS 8000
#define DIAM_MPU_MARGIN        0.05f

// ============================================================
// KONFIGURASI ANOMALY-BASED TRIGGER (DIPERKUAT)
// ============================================================
// Jalur notifikasi TAMBAHAN yang independen dari fuzzy logic. Memakai
// anomalyScore (Z-score dari totF, window 20 sampel ~10 detik) sebagai
// pemicu WASPADA/DARURAT terpisah untuk menangkap pola gerakan yang
// secara statistik tidak biasa (mis. tremor/limbung berulang) namun
// TIDAK cukup ekstrem untuk memicu satu pun rule pada fuzzy logic.
//
// DIPERKUAT: sebelumnya sistem cuma mendeteksi SATU window anomali
// tunggal (Z-score tinggi selama 1,5 detik beruntun -> notifikasi
// waspada, selesai). Sekarang ada 2 lapisan tambahan:
//  1) Level EKSTREM -- Z-score sangat tinggi (>5.0) langsung dianggap
//     event tanpa nunggu debounce, karena lonjakan setajam itu biasanya
//     berarti gerakan liar/tak terkendali yang perlu direspons cepat.
//  2) Deteksi POLA BERULANG -- sistem mencatat waktu tiap kejadian
//     anomali dan menghitung berapa kali terjadi dalam 90 detik
//     terakhir. Kalau >=3 kali, dieskalasi jadi status DARURAT dan
//     ikut mengirim SMS -- ini implementasi konkret dari "getaran/
//     limbung berulang yang tidak stabil" sebagai peringatan dini,
//     bukan cuma satu lonjakan tunggal.
#define ANOMALY_ALERT_THRESHOLD   1.5f   // Z-score dasar (3-sigma) -> anomali TUNGGAL
#define ANOMALY_SEVERE_THRESHOLD  3.0f   // Z-score sangat tinggi -> anomali EKSTREM, langsung darurat
#define ANOMALY_DEBOUNCE_COUNT    3      // 3 siklus (1,5s) konsisten tinggi -> event tunggal terkonfirmasi
#define ANOMALY_NOTIF_COOLDOWN    (2UL*60UL*1000UL) // 2 menit, cegah spam kalau anomali terus-menerus

#define ANOMALY_REPEAT_WINDOW_MS  (90UL*1000UL)  // amati 90 detik terakhir
#define ANOMALY_REPEAT_LOG_SIZE   6              // simpan maks 6 event terakhir
#define ANOMALY_REPEAT_ESCALATE   3              // >=3 event dlm window -> eskalasi DARURAT

// ============================================================
// KONFIGURASI SMS & CAREGIVER PHONE
// ============================================================
// FIX: cooldown sekarang 1 menit PER JENIS EVENT (jatuh/geofence/offline/
// anomali masing-masing punya timer sendiri), bukan 1 timer global. Dulu
// kalau SMS jatuh baru terkirim, SMS geofence yang menyusul segera
// sesudahnya bisa ke-skip gara-gara masih dalam cooldown event LAIN --
// padahal itu info yang beda dan sama-sama penting. Lihat
// smsCooldownSlot() di bawah.
#define SMS_COOLDOWN_MS        (1UL*60UL*1000UL)          // 1 menit per jenis event
#define IV_CAREGIVER_SYNC      (5UL*60UL*1000UL) // resync nomor dari Firebase tiap 5 menit saat online
#define OFFLINE_ALERT_THRESHOLD_MS (5UL*60UL*1000UL)  // device offline (WiFi mati) >10 menit -> SMS sekali

// ============================================================
// GLOBALS
// ============================================================
Adafruit_SSD1306 display(128, 64, &Wire, -1);
bool oledReady = false, firebaseReady = false, mpuReady = false, gpsReady = false;
bool gsmModemReady = false;   // modem.init() berhasil -> SMS bisa dipakai

MPU6050 mpu;
HardwareSerial sim808Serial(2);
TinyGPSPlus gps;
Preferences prefs;

// ── WiFi -- SEKARANG SATU-SATUNYA jalur ke Firebase ─────────
WiFiClientSecure wifiSecure;
// FIX LATENCY: httpWifi jadi objek GLOBAL + setReuse(true) di firebaseRequest()
// -> koneksi TCP/TLS ke Firebase gak perlu handshake ulang tiap request kalau
// host sama (dulu `HTTPClient http;` dibuat baru tiap panggilan).
HTTPClient httpWifi;
bool       httpWifiInited = false;

// modem TETAP ADA -- dipakai buat SMS (modem.sendSMS()) dan info sinyal.
// TIDAK lagi dipakai sebagai transport data (tidak ada TinyGsmClient/
// ESP_SSLClient yang dipasang ke HTTPClient/HttpClient manapun).
TinyGsm modem(sim808Serial);

// ── Status sinkronisasi waktu ───────────────────────────────
bool          timeSynced = false;   // true kalau RTC ESP32 sudah pernah berhasil di-set (NTP/GSM/HTTP-Date)
unsigned long tTimeRetry  = 0;

// ── Retry WiFi non-blocking (FIX: dulu connectWiFi() blocking dipanggil
// tiap loop() saat WiFi mati -> sensor/buzzer/SMS jatuh macet total
// selama proses reconnect) ──────────────────────────────────
#define IV_WIFI_RETRY 15000   // percobaan reconnect paling cepat tiap 15 detik
unsigned long tWifiRetry = 0;

// ── Nomor caregiver (cache NVS + sinkron Firebase) ──────────
String        caregiverPhone   = "";     // hasil sinkron dari Firebase, di-cache ke NVS
unsigned long tCaregiverSync   = 0;

// ── Cooldown SMS PER JENIS EVENT (bukan 1 timer global lagi) ────────
// Tiap jenis event punya "jam" sendiri, jadi SMS jatuh gak nge-block SMS
// geofence yang menyusul, dst. Lihat smsCooldownSlot().
unsigned long tLastSMS_Jatuh    = 0;
unsigned long tLastSMS_Geofence = 0;
unsigned long tLastSMS_Offline  = 0;
unsigned long tLastSMS_Anomali  = 0;   // BARU -- slot khusus SMS anomali
unsigned long tLastSMS_Lainnya  = 0;   // fallback buat label event lain (mis. "TEST")

// ── Deteksi "device offline lama" (WiFi mati) ───────────────
unsigned long tLastOnlineOk    = 0;
bool          offlineAlertSent = false;

uint32_t      notifCounter  = 0;
// FIX LATENCY: historyCounter di-cache lokal (kayak notifCounter), disinkron
// SEKALI di registerWalker() (boot) lalu di-increment lokal terus -> hemat 1
// request GET yang dulu selalu dijalankan tiap pushHistory() (termasuk tiap
// fall event).
uint32_t historyCounter = 0;
unsigned long tUserLastSeen = 0;
unsigned long tHB = 0, tSen = 0, tGPS = 0, tGeo = 0, tGeoReload = 0;
unsigned long lastNotifNoUser = 0;
unsigned long lastNotifObstacle = 0;

bool  prevFall = false, prevOutside = false;
float ax = 0, ay = 0, az = 0;
float gx = 0, gy = 0, gz = 0;
float distFront = 999, distBack = 999;
float backBuf[BACK_SMOOTH_SIZE] = {999,999,999,999,999};
uint8_t backBufIdx = 0;
float lastGoodBack = 999.0f;
uint8_t backOutlierStreak = 0;
unsigned long lastGoodBackTime = 0;
float latitude = 0.0f, longitude = 0.0f, jarakDariPusat = 0.0f;
bool  geofenceOutside = false;
bool  obstacleDetected = false;
bool  userDetected     = false;
bool          prevWaspada     = false;
unsigned long lastNotifWaspada = 0;

bool    obstacleConfirmed = false;
uint8_t obstacleTrueCnt   = 0;
uint8_t obstacleFalseCnt  = 0;

bool    userConfirmed = false;
uint8_t userTrueCnt   = 0;
uint8_t userFalseCnt  = 0;

float geoLat = -7.051f, geoLng = 110.442f, geoRadius = 100.0f;

static const char DB[] = DB_PATH;

// ── BUZZER STATE (dipakai step counter buat skip getaran palsu) ────
bool buzzerActive = false;
unsigned long buzzerOffTime = 0;
#define BUZZ_SETTLE_MS 300   // jeda setelah buzzer mati sebelum step counter dipercaya lagi

// ---------- STEP COUNTER ----------
unsigned long tStepSample    = 0;
bool          isCalibrating  = false;
unsigned long calibStartTime = 0;
float         calibSum       = 0;
float         calibSumSq     = 0;   // FIX: buat hitung stddev noise saat kalibrasi
int           calibCount     = 0;
float         dynamicPeakDelta = STEP_MIN_DELTA;
float         smoothBuf[SMOOTH_SIZE];
int           smoothIdx    = 0;
bool          smoothFull   = false;
float         baseline     = 1.0f;
int           stepCount    = 0;
unsigned long lastStepTime = 0;
unsigned long belowExitSince = 0;
bool          belowExitPending = false;
bool   lastStepWasHard = false;   // NEW
bool          wasAboveThreshold = false;

// ---------- FUZZY FALL DETECTION ----------
float   axBuf[4] = {0,0,0,0};
float   ayBuf[4] = {0,0,0,0};
float   azBuf[4] = {0,0,0,1.0f};
uint8_t accelIdx = 0;
float   axF = 0, ayF = 0, azF = 1.0f;

float   peakImpactBuf[4] = {0};
float   peakGyroBuf[4]   = {0};
uint8_t peakImpactIdx    = 0;
uint8_t peakGyroIdx      = 0;

float         diamSecs     = 0.0f;
bool          diamMpuActive = false;
unsigned long diamMpuStart  = 0;

bool          fallHoldActive = false;
unsigned long fallHoldStart  = 0;
float         lastFallRisk   = 0.0f;
float         lastImpact     = 0.0f;
float         lastGyro       = 0.0f;
float         lastConf       = 0.0f;
// ============================================================
// FAST FALL SAMPLING -- menangkap PUNCAK benturan yang terjadi
// DI ANTARA dua panggilan checkFall() (yang cuma tiap 500ms).
// Tanpa ini, benturan fisik yang cuma berlangsung 10-50ms gampang
// "kelewat" karena checkFall() dulu cuma ambil 1 titik data mentah
// tiap 500ms.
// ============================================================
#define FAST_FALL_SAMPLE_INTERVAL_MS 10
unsigned long tFastFallSample = 0;
float fastPeakImpact = 0.0f;   // max |tot-1g| sejak checkFall() terakhir
float fastPeakGyro   = 0.0f;   // max gyroMag sejak checkFall() terakhir

void updateFastFallSampling() {
  if (!mpuReady) return;
  unsigned long now = millis();
  if (now - tFastFallSample < FAST_FALL_SAMPLE_INTERVAL_MS) return;
  tFastFallSample = now;

  int16_t rax, ray, raz, rgx, rgy, rgz;
  mpu.getMotion6(&rax, &ray, &raz, &rgx, &rgy, &rgz);
  float fx = rax/16384.0f, fy = ray/16384.0f, fz = raz/16384.0f;
  float fgx = rgx/131.0f,  fgy = rgy/131.0f,  fgz = rgz/131.0f;

  float tot = sqrtf(fx*fx + fy*fy + fz*fz);
  float impactNow = fabsf(tot - 1.0f);
  if (impactNow > fastPeakImpact) fastPeakImpact = impactNow;

  float gyroNow = sqrtf(fgx*fgx + fgy*fgy + fgz*fgz);
  if (gyroNow > fastPeakGyro) fastPeakGyro = gyroNow;
}

// ── Snapshot terbaru untuk dikirim bareng sendCombinedUpdate() ─
float   gPeakImpact   = 0.0f;
float   gPeakGyro     = 0.0f;
float   gTotAccel     = 0.0f;
float   gFuzzyRisk    = 0.0f;
float   gReportConf   = 0.0f;
bool    gJatuhFirebase = false;

#define ANOMALY_WINDOW 20
float   accelWindow[ANOMALY_WINDOW];
uint8_t winIdx  = 0;
bool    winFull = false;
float   anomalyScore = 0.0f;

// ── State mesin ANOMALY-BASED TRIGGER (DIPERKUAT) ────────────
bool          anomalyConfirmed = false;
uint8_t       anomalyTrueCnt   = 0;
uint8_t       anomalyFalseCnt  = 0;
bool          prevAnomalyEvent = false;
unsigned long lastNotifAnomaly = 0;

// ── State BARU: log timestamp kejadian anomali, buat deteksi pola
// berulang (mis. limbung berulang) dalam window waktu tertentu ──────
unsigned long anomalyEventLog[ANOMALY_REPEAT_LOG_SIZE] = {0};
uint8_t       anomalyEventLogIdx   = 0;
uint8_t       anomalyEventLogCount = 0;
uint8_t       anomalyRepeatCount   = 0;  // hasil hitung terakhir, ikut dikirim ke Firebase

// ============================================================
// FORWARD DECLARATIONS
// ============================================================
bool pushNotification(const char* title, const char* description,
                      const char* level, float scoreFall, int durasiDetik);
void pushHistory(const char* title, const char* subtitle,
                 const char* jenisKejadian, const char* status,
                 const char* statusBahaya,  const char* category,
                 const char* type,          const char* sensor,
                 float skorAnomali,
                 const char* tindakan1,     const char* tindakan2);
bool syncTimeFromGSM();   // fallback sinkronisasi waktu lewat AT+CCLK? (network time SIM808)
bool isOnline();
bool sendEmergencySMS(const char* eventLabel, const String& detail);
void loadCaregiverPhoneFromNVS();
void saveCaregiverPhoneToNVS(const String& num);
void syncCaregiverPhoneFromFirebase();
String parseFirebaseString(const String& raw);
void checkOfflineDuration();
void pauseGNSS();
void resumeGNSS();
bool firebaseRequest(const char* method, const char* path,
                      const String& body, String* outResponse = nullptr);
void checkAnomalyTrigger(bool statusAmanFuzzy);
void logAnomalyEvent(unsigned long now);
uint8_t countRecentAnomalyEvents(unsigned long now);

// ============================================================
// HELPERS UMUM
// ============================================================
void makePath(char* buf, uint8_t blen, const char* sub) {
  snprintf(buf, blen, "%s%s", DB, sub);
}

void getTS(char* buf, uint8_t len) {
  struct tm t;
  if (!getLocalTime(&t, 50)) {
    unsigned long s = millis() / 1000;
    snprintf(buf, len, "%02lu:%02lu:%02lu", (s/3600)%24, (s/60)%60, s%60);
    return;
  }
  strftime(buf, len, "%H:%M:%S", &t);
}

void getDTS(char* buf, uint8_t len) {
  struct tm t;
  if (!getLocalTime(&t, 50)) { strncpy(buf, "1970-01-01 00:00:00", len); return; }
  strftime(buf, len, "%Y-%m-%d %H:%M:%S", &t);
}

// ============================================================
// PATCH #1: calcDist() diganti versi retry + median-of-3
// (khusus buat kompensasi noise/glitch di TRIG_BACK/ECHO_BACK yang
// menempati strapping pin GPIO2/15 -- tidak bisa diganti pin fisiknya,
// jadi dikompensasi lewat software)
// ============================================================
float calcDistRaw(uint8_t trig, uint8_t echo) {
  for (uint8_t attempt = 0; attempt < 2; attempt++) {
    digitalWrite(trig, LOW);  delayMicroseconds(4);
    digitalWrite(trig, HIGH); delayMicroseconds(20);   // dari 10us -> 20us
    digitalWrite(trig, LOW);
    long d = pulseIn(echo, HIGH, 38000);
    if (d > 0) return d * 0.034f / 2.0f;
    delayMicroseconds(2000);
  }
  return 999.0f;
}

float calcDist(uint8_t trig, uint8_t echo) {
  float s[3];
  float minValid = 999.0f;
  uint8_t validCount = 0;

  for (uint8_t i = 0; i < 3; i++) {
    s[i] = calcDistRaw(trig, echo);
    if (i < 2) delayMicroseconds(3000);
    if (s[i] < 999.0f) {
      validCount++;
      if (s[i] < minValid) minValid = s[i];   // ambil yang PALING DEKAT
    }
  }

  Serial.printf("[CALCDIST-DBG] trig=%d echo=%d | s0=%.1f s1=%.1f s2=%.1f | valid=%d min=%.1f\n",
                trig, echo, s[0], s[1], s[2], validCount, minValid);

  if (validCount == 0) {
    return 999.0f;
  }

  return minValid;
}

float haversine(float la1, float lo1, float la2, float lo2) {
  const float R = 6371000.0f;
  float dLa = radians(la2-la1), dLo = radians(lo2-lo1);
  float a = sinf(dLa/2)*sinf(dLa/2)
            + cosf(radians(la1))*cosf(radians(la2))*sinf(dLo/2)*sinf(dLo/2);
  return R * 2 * atan2f(sqrtf(a), sqrtf(1-a));
}

void showOLED(const char* l1, const char* l2="",
              const char* l3="", const char* l4="") {
  if (!oledReady) return;
  display.clearDisplay();
  display.setTextSize(1); display.setTextColor(WHITE);
  display.setCursor(0,  0); display.println(l1);
  display.setCursor(0, 16); display.println(l2);
  display.setCursor(0, 32); display.println(l3);
  display.setCursor(0, 48); display.println(l4);
  display.display();
}

// ============================================================
// STATUS JARINGAN "LIVE" — SEKARANG MURNI WIFI (GPRS dihapus)
// ============================================================
bool isOnline() {
  return WiFi.status() == WL_CONNECTED;
}

// ============================================================
// BUZZER
// ============================================================
void buzzerOn(uint32_t freq) { ledcWriteTone(BUZZER_PIN, freq); }
void buzzerOff()              { ledcWriteTone(BUZZER_PIN, 0);   }
void updateBuzzer() {
  bool shouldBuzz = fallHoldActive || obstacleConfirmed;

  if (shouldBuzz) {
    buzzerOn(BUZZ_FREQ_OBSTACLE);
    buzzerActive = true;
  } else {
    if (buzzerActive) buzzerOffTime = millis();
    buzzerOff();
    buzzerActive = false;
  }
}

// ============================================================
// WIFI
// ============================================================
void connectWiFi() {
  Serial.print(F("[WiFi] Konek..."));
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  uint8_t att = 0;
  while (WiFi.status() != WL_CONNECTED && att < 30) { delay(500); Serial.print('.'); att++; }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print(F("\n[WiFi] OK: ")); Serial.println(WiFi.localIP());
  } else {
    Serial.println(F("\n[WiFi] GAGAL!"));
  }
}

// versi non-blocking, dipakai di loop() supaya reconnect WiFi TIDAK
// menahan readSensors()/checkFall()/updateBuzzer() selama belasan detik.
// Fungsi ini langsung return -- status koneksi dicek secara alami di
// iterasi loop() berikutnya lewat WiFi.status()
void connectWiFiNonBlocking() {
  if (WiFi.status() == WL_CONNECTED) return;
  Serial.println(F("[WiFi] Reconnect dimulai (non-blocking)..."));
  WiFi.disconnect();
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
}

// ============================================================
// STATUS JARINGAN GABUNGAN — dipanggil tiap loop(), murah (cuma cek
// WiFi.status(), tidak ada request jaringan apa pun di sini).
// Dulu ada updateNetworkMode() yang urus fallback ke GPRS tiap
// IV_NET_CHECK -- sudah dihapus total bareng semua kode GPRS.
// ============================================================
void updateNetworkMode() {
  bool wifiOk = (WiFi.status() == WL_CONNECTED);
  if (wifiOk != firebaseReady) {
    Serial.println(wifiOk
      ? F("[NET] WiFi tersambung -> Firebase aktif")
      : F("[NET] WiFi terputus -> jalur data mati, SMS jadi cadangan"));
  }
  firebaseReady = wifiOk;
}

// ============================================================
// SINKRONISASI WAKTU DARI HEADER HTTP "Date:" (fallback ke-3)
// ============================================================
bool applyHttpDateHeader(const String& dateHeader) {
  char dow[4], monStr[4];
  int day, year, hh, mm, ss;
  int parsed = sscanf(dateHeader.c_str(), "%3s, %d %3s %d %d:%d:%d",
                       dow, &day, monStr, &year, &hh, &mm, &ss);
  if (parsed < 7) return false;

  const char* months = "JanFebMarAprMayJunJulAugSepOctNovDec";
  const char* pos = strstr(months, monStr);
  if (!pos) return false;
  int mon = (pos - months) / 3;   // 0-11

  struct tm tmS = {0};
  tmS.tm_year = year - 1900;
  tmS.tm_mon  = mon;
  tmS.tm_mday = day;
  tmS.tm_hour = hh;
  tmS.tm_min  = mm;
  tmS.tm_sec  = ss;

  time_t epoch = mktime(&tmS);
  struct timeval tv = { epoch, 0 };
  configTime(7*3600, 0, "pool.ntp.org");   // set offset lokal, tanpa nunggu respons NTP
  settimeofday(&tv, NULL);

  char buf[24];
  struct tm* localT = localtime(&epoch);
  strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", localT);
  Serial.print(F("[HTTP-DATE] Waktu disinkron dari header Firebase: "));
  Serial.println(buf);
  return true;
}

// ============================================================
// NTP + FALLBACK GSM (AT+CCLK?)
// ============================================================
void syncTime() {
  bool ntpOk = false;

  if (WiFi.status() == WL_CONNECTED) {
    configTime(7*3600, 0, "pool.ntp.org", "time.nist.gov");
    struct tm t;
    ntpOk = getLocalTime(&t, 8000);   // tunggu maks 8 detik, non-blocking permanen
  }

  if (ntpOk) {
    timeSynced = true;
    char buf[20]; getTS(buf, sizeof(buf));
    Serial.print(F("[NTP] OK: ")); Serial.println(buf);
    return;
  }

  Serial.println(F("[NTP] Gagal / WiFi tidak konek -> coba waktu jaringan GSM"));
  if (syncTimeFromGSM()) {
    timeSynced = true;
  } else {
    timeSynced = false;
    Serial.println(F("[TIME] GAGAL total (NTP & GSM) — akan dicoba ulang otomatis di loop() / lewat header HTTP-Date"));
  }
}

bool syncTimeFromGSM() {
  while (sim808Serial.available()) sim808Serial.read();   // flush buffer lama
  sim808Serial.println(F("AT+CCLK?"));

  String resp = "";
  unsigned long start = millis();
  while (millis() - start < 3000) {
    while (sim808Serial.available()) resp += (char)sim808Serial.read();
    if (resp.indexOf("OK") != -1 || resp.indexOf("ERROR") != -1) break;
    delay(10);
  }

  int idx = resp.indexOf("+CCLK:");
  if (idx == -1) {
    Serial.println(F("[GSM-TIME] Tidak ada respons +CCLK (modem belum siap/sinyal kosong)"));
    return false;
  }

  int q1 = resp.indexOf('"', idx);
  int q2 = (q1 != -1) ? resp.indexOf('"', q1 + 1) : -1;
  if (q1 == -1 || q2 == -1) {
    Serial.println(F("[GSM-TIME] Format respons tidak dikenali"));
    return false;
  }
  String ts = resp.substring(q1 + 1, q2);   // "yy/MM/dd,hh:mm:ss+zz"

  int yy, MM, dd, hh, mm, ss, tzQuarter;
  char sign;
  int parsed = sscanf(ts.c_str(), "%d/%d/%d,%d:%d:%d%c%d",
                       &yy, &MM, &dd, &hh, &mm, &ss, &sign, &tzQuarter);
  if (parsed < 6) {
    Serial.println(F("[GSM-TIME] Gagal parse waktu"));
    return false;
  }
  if (yy < 20) {
    Serial.println(F("[GSM-TIME] Modem belum dapat waktu jaringan valid (SIM/sinyal?)"));
    return false;
  }

  struct tm tmS = {0};
  tmS.tm_year = yy + 2000 - 1900;
  tmS.tm_mon  = MM - 1;
  tmS.tm_mday = dd;
  tmS.tm_hour = hh;
  tmS.tm_min  = mm;
  tmS.tm_sec  = ss;

  configTime(0, 0, "pool.ntp.org");   // reset offset ke 0, tidak menunggu respons NTP
  time_t epoch = mktime(&tmS);
  struct timeval tv = { (time_t)epoch, 0 };
  settimeofday(&tv, NULL);

  char buf[24];
  strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", &tmS);
  Serial.print(F("[GSM-TIME] OK (dari jaringan seluler): ")); Serial.println(buf);
  return true;
}

// ============================================================
// PAUSE/RESUME GNSS — TETAP DIPERTAHANKAN
// ============================================================
// GPS (NMEA) & AT command (SMS, AT+CCLK?) masih berbagi UART yang sama
// dengan SIM808, jadi tetap perlu di-pause biar gak nyelip pas kirim
// SMS atau baca waktu jaringan. Ini SATU-SATUNYA alasan pauseGNSS masih
// ada -- sudah tidak ada lagi kaitannya dengan GPRS/data.
uint8_t gnssPauseDepth = 0;

void pauseGNSS() {
  gnssPauseDepth++;
  if (gnssPauseDepth > 1) return;

  sim808Serial.println(F("AT+CGNSTST=0"));

  unsigned long lastByte = millis();
  unsigned long start    = millis();
  while (millis() - start < 800) {
    if (sim808Serial.available()) {
      sim808Serial.read();
      lastByte = millis();
    } else if (millis() - lastByte >= 150) {
      break;
    }
  }
}

void resumeGNSS() {
  if (gnssPauseDepth == 0) return;
  gnssPauseDepth--;
  if (gnssPauseDepth > 0) return;

  sim808Serial.println(F("AT+CGNSTST=1"));
  delay(200);
  while (sim808Serial.available()) sim808Serial.read();
}

// ============================================================
// SMS DARURAT (last resort) + NOMOR CAREGIVER (NVS + Firebase)
// ============================================================
void loadCaregiverPhoneFromNVS() {
  prefs.begin("guardianwalk", true);   // read-only
  caregiverPhone = prefs.getString("cg_phone", "");
  prefs.end();
  if (caregiverPhone.length() > 0) {
    Serial.print(F("[CAREGIVER] Nomor dari cache NVS: "));
    Serial.println(caregiverPhone);
  } else {
    Serial.println(F("[CAREGIVER] Cache NVS kosong -> pakai default sampai sinkron pertama berhasil"));
  }
}

void saveCaregiverPhoneToNVS(const String& num) {
  prefs.begin("guardianwalk", false);  // read-write
  prefs.putString("cg_phone", num);
  prefs.end();
}

String parseFirebaseString(const String& raw) {
  String s = raw;
  s.trim();
  if (s.length() >= 2 && s.startsWith("\"") && s.endsWith("\"")) {
    s = s.substring(1, s.length() - 1);
  }
  if (s == "null") s = "";
  return s;
}

void syncCaregiverPhoneFromFirebase() {
  if (!isOnline()) return;
  String resp;
  char path[48]; makePath(path, sizeof(path), "/config/caregiver_phone");
  if (!firebaseRequest("GET", path, "", &resp)) {
    Serial.println(F("[CAREGIVER] Gagal ambil nomor dari Firebase"));
    return;
  }
  String num = parseFirebaseString(resp);
  if (num.length() == 0) {
    Serial.println(F("[CAREGIVER] Firebase belum punya nomor caregiver (config/caregiver_phone kosong)"));
    return;
  }
  if (num != caregiverPhone) {
    caregiverPhone = num;
    saveCaregiverPhoneToNVS(num);
    Serial.print(F("[CAREGIVER] Nomor disinkron & disimpan ke NVS: "));
    Serial.println(num);
  }
}

String activeSmsNumber() {
  if (caregiverPhone.length() > 0) return caregiverPhone;
  return String(SMS_RECIPIENT_DEFAULT);
}

// Pilih variabel cooldown sesuai jenis event -- INI yang bikin cooldown
// per-event, bukan global. Label tak dikenal (mis. "TEST" dari serial
// monitor) jatuh ke slot "Lainnya" biar tetap ada batasannya juga.
unsigned long* smsCooldownSlot(const char* eventLabel) {
  if (strcmp(eventLabel, "JATUH")    == 0) return &tLastSMS_Jatuh;
  if (strcmp(eventLabel, "GEOFENCE") == 0) return &tLastSMS_Geofence;
  if (strcmp(eventLabel, "OFFLINE")  == 0) return &tLastSMS_Offline;
  if (strcmp(eventLabel, "ANOMALI")  == 0) return &tLastSMS_Anomali;   // BARU
  return &tLastSMS_Lainnya;
}

bool sendEmergencySMS(const char* eventLabel, const String& detail) {
  unsigned long now = millis();
  unsigned long* cooldownSlot = smsCooldownSlot(eventLabel);

  bool sudahPernahKirim = (*cooldownSlot != 0);       
  if (sudahPernahKirim && (now - *cooldownSlot < SMS_COOLDOWN_MS)) {   
    Serial.printf("[SMS] Skip -- [%s] masih dalam cooldown (sisa %lus)\n",
                  eventLabel, (SMS_COOLDOWN_MS - (now - *cooldownSlot)) / 1000);
    return false;
  }

  String number = activeSmsNumber();
  if (number.length() == 0) {
    Serial.println(F("[SMS] Skip -- nomor caregiver belum tersedia"));
    return false;
  }

  char ts[12]; getTS(ts, sizeof(ts));
  String msg = "GuardianWalk [" + String(eventLabel) + "] " + String(ts) + " - " + detail;
  if (gpsReady) {
    char loc[64];
    snprintf(loc, sizeof(loc), " Lok: https://maps.google.com/?q=%.6f,%.6f", latitude, longitude);
    msg += loc;
  }
  if (msg.length() > 150) msg = msg.substring(0, 150);   // batas aman 1 segmen SMS

  pauseGNSS();
  bool ok = modem.sendSMS(number, msg);
  resumeGNSS();

  if (ok) {
    *cooldownSlot = now;
    Serial.print(F("[SMS] Terkirim ke ")); Serial.println(number);
  } else {
    Serial.println(F("[SMS] GAGAL terkirim (cek sinyal GSM/pulsa SMS/nomor)"));
  }
  return ok;
}

void checkOfflineDuration() {
  unsigned long now = millis();
  if (isOnline()) {
    tLastOnlineOk    = now;
    offlineAlertSent = false;
    return;
  }
  if (!offlineAlertSent && (now - tLastOnlineOk >= OFFLINE_ALERT_THRESHOLD_MS)) {
    char detail[48];
    snprintf(detail, sizeof(detail), "Device offline > %lu menit",
             (now - tLastOnlineOk) / 60000UL);
    if (sendEmergencySMS("OFFLINE", String(detail))) {
      offlineAlertSent = true;
    }
  }
}

// ============================================================
// FIREBASE — REST API, SEKARANG MURNI LEWAT WIFI
// ============================================================
// Cabang GPRS (TinyGsmClient/ESP_SSLClient sebagai transport HTTPS) sudah
// dihapus total. Kalau WiFi mati, fungsi ini langsung gagal cepat --
// device TIDAK lagi coba jalur data lain, cadangannya murni SMS.
bool firebaseRequest(const char* method, const char* path,
                      const String& body, String* outResponse) {
  if (WiFi.status() != WL_CONNECTED) return false;

  String uri = String(path) + ".json?auth=" + FIREBASE_AUTH;

  // FIX LATENCY: dulu `HTTPClient http;` dibuat baru tiap panggilan ->
  // TCP+TLS handshake dari nol tiap request (ratusan ms - detik sendiri).
  // Sekarang pakai httpWifi GLOBAL + setReuse(true) -> request berikutnya
  // ke host yang sama pakai socket yang udah ada, gak handshake ulang.
  if (!httpWifiInited) {
    wifiSecure.setInsecure();
    httpWifi.setReuse(true);
    httpWifi.setTimeout(5000);
    httpWifiInited = true;
  }

  if (!httpWifi.begin(wifiSecure, FIREBASE_HOST, 443, uri, true)) return false;
  httpWifi.addHeader("Content-Type", "application/json");

  int code;
  if      (strcmp(method, "GET")   == 0) code = httpWifi.GET();
  else if (strcmp(method, "PUT")   == 0) code = httpWifi.PUT(body);
  else                                    code = httpWifi.PATCH(body);

  // Sinkron waktu dari header Date kalau RTC belum pernah ke-set.
  if (!timeSynced) {
    String d = httpWifi.header("Date");
    if (d.length() > 0 && applyHttpDateHeader(d)) timeSynced = true;
  }

  bool ok = (code > 0 && code < 300);
  if (!ok) Serial.printf("[FB-WIFI] %s %s -> HTTP %d\n", method, path, code);
  if (outResponse) *outResponse = httpWifi.getString();
  // PENTING: httpWifi.end() dengan setReuse(true) TIDAK menutup socket --
  // cuma reset state HTTPClient buat siap dipakai request berikutnya.
  httpWifi.end();
  return ok;
}

bool fbSet(const char* path, FirebaseJson &j) {
  String body; j.toString(body, false);
  return firebaseRequest("PUT", path, body);
}
bool fbUpdate(const char* path, FirebaseJson &j) {
  String body; j.toString(body, false);
  return firebaseRequest("PATCH", path, body);
}
bool fbSetInt(const char* path, int val) {
  return firebaseRequest("PUT", path, String(val));
}
bool fbGetFloat(const char* path, float &out) {
  String resp;
  if (!firebaseRequest("GET", path, "", &resp)) return false;
  if (resp.length() == 0 || resp == "null") return false;
  out = resp.toFloat();
  return true;
}
bool fbGetInt(const char* path, int &out) {
  String resp;
  if (!firebaseRequest("GET", path, "", &resp)) return false;
  if (resp.length() == 0 || resp == "null") return false;
  out = resp.toInt();
  return true;
}

void connectFirebase() {
  String resp;
  bool ok = firebaseRequest("GET", "/meta/notifCount", "", &resp);
  firebaseReady = ok;
  Serial.println(ok ? F("[Firebase] OK") : F("[Firebase] GAGAL (cek host/secret/jaringan)"));
}

void loadGeofenceFromFirebase() {
  if (!isOnline()) return;
  char path[64]; float v;
  makePath(path, sizeof(path), "/geofence/center_latitude");
  if (fbGetFloat(path, v) && v != 0.0f) geoLat = v;
  makePath(path, sizeof(path), "/geofence/center_longitude");
  if (fbGetFloat(path, v) && v != 0.0f) geoLng = v;
  makePath(path, sizeof(path), "/geofence/radius");
  if (fbGetFloat(path, v) && v > 0.0f)  geoRadius = v;
  Serial.printf("[GEO] lat=%.6f lng=%.6f radius=%.0f\n", geoLat, geoLng, geoRadius);
}

void registerWalker() {
  if (!isOnline()) return;

  char metaPath[60];
  snprintf(metaPath, sizeof(metaPath), "%s/meta/notifCount", DB);
  int nc;
  if (fbGetInt(metaPath, nc) && nc > 0) {
    notifCounter = (uint32_t)nc;
    Serial.printf("[META] notifCounter sync: %u\n", notifCounter);
  }

  char histMetaPath[64];
  snprintf(histMetaPath, sizeof(histMetaPath), "%s/meta/historyCount", DB);
  int hc0;
  if (fbGetInt(histMetaPath, hc0) && hc0 > 0) {
    historyCounter = (uint32_t)hc0;
    Serial.printf("[META] historyCounter sync: %u\n", historyCounter);
  }

  char dts[20]; getDTS(dts, sizeof(dts));
  char ts[12];  getTS(ts, sizeof(ts));
  FirebaseJson j;
  j.set("status",        "normal");
  j.set("walker_active", true);
  j.set("last_update",   ts);
  j.set("activity/langkah", 0);
  j.set("fall_detetction/jatuh",         false);
  j.set("fall_detetction/fall_detected", false);
  j.set("fall_detetction/confidence",    0.0);
  j.set("fall_detetction/impact",        0.0);
  j.set("fall_detetction/impact_value",  0.0);
  j.set("fall_detetction/fuzzy_risk",    0.0);
  j.set("fall_detetction/gyro_peak",     0.0);
  j.set("fall_detetction/az_filtered",   0.0);
  j.set("fall_detetction/diam_detik",    0.0);
  j.set("fall_detetction/anomaly_score", 0.0);
  j.set("fall_detetction/anomaly_repeat_count", 0);
  j.set("fall_detetction/timestamp",     dts);
  j.set("location/latitude",    0.0);
  j.set("location/longitude",   0.0);
  j.set("location/last_update", dts);
  j.set("sensors/mpu_active",      false);
  j.set("sensors/mpu6050/accel_x", 0.0);
  j.set("sensors/mpu6050/accel_y", 0.0);
  j.set("sensors/mpu6050/accel_z", 0.0);
  j.set("sensors/mpu6050/gyro_x",  0.0);
  j.set("sensors/mpu6050/gyro_y",  0.0);
  j.set("sensors/mpu6050/gyro_z",  0.0);
  j.set("sensors/hcsr04_front/distance",          0.0);
  j.set("sensors/hcsr04_front/obstacle_detected", false);
  j.set("sensors/hcsr04_back/distance",           0.0);
  j.set("sensors/hcsr04_back/user_detected",      false);
  j.set("sensors/ringkasanSensor/hcsr04_front",       false);
  j.set("sensors/ringkasanSensor/user_detected_back", false);
  j.set("sensors/gps_active",             false);
  j.set("sensors/sim808/gps_status",      false);
  j.set("sensors/sim808/gsm_signal",      0);
  j.set("sensors/sim808/internet_status", false);
  j.set("sensors/sim808/satelite_count",  0);
  j.set("sensors/statusSistem/gpsConnected", false);
  j.set("sensors/statusSistem/gsmConnected", false);
  j.set("sensors/statusSistem/imuNormal",    false);
  j.set("adaptive/calib_done",     false);
  j.set("adaptive/baseline_accel", 0.0);
  j.set("adaptive/step_low",       0.0);
  j.set("adaptive/step_high",      0.0);
  j.set("meta/notifCount",   (int)notifCounter);

  fbUpdate(DB, j);
  loadGeofenceFromFirebase();
  Serial.println(F("[Firebase] Walker terdaftar!"));
}

// ============================================================
// OLED
// ============================================================
void setupOLED() {
  if (display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    oledReady = true;
    display.clearDisplay(); display.setTextSize(1);
    display.setTextColor(WHITE); display.display();
    Serial.println(F("[OLED] OK"));
  } else {
    Serial.println(F("[OLED] Tidak ditemukan."));
  }
}

// ============================================================
// SENSOR SETUP
// ============================================================
void setupMPU() {
  Wire.begin();
  mpu.initialize();
  mpuReady = mpu.testConnection();
  if (mpuReady) {
    mpu.setDLPFMode(MPU6050_DLPF_BW_42);
    mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
    mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
  }
  Serial.println(mpuReady ? F("[MPU6050] OK") : F("[MPU6050] Tidak ditemukan!"));
}

// GPRS_APN/GPRS_USER/GPRS_PASS/GSM_PIN DIHAPUS -- SIM808 gak lagi
// dipakai buat data, jadi APN gak relevan lagi. setup di bawah cuma
// nyalain GNSS (buat GPS) & modem.init() (buat SMS).
void setupSIM808() {
  sim808Serial.begin(SIM808_BAUD, SERIAL_8N1, SIM808_RX, SIM808_TX);
  delay(1000);

  sim808Serial.println(F("AT")); delay(500);
  if (sim808Serial.available()) {
    gpsReady = true;
    sim808Serial.println(F("AT+CGNSPWR=1")); delay(500);
    while (sim808Serial.available()) sim808Serial.read();   // buang respons AT+CGNSPWR
    Serial.println(F("[SIM808] AT OK"));
  } else {
    Serial.println(F("[SIM808] Tidak merespons."));
  }

  Serial.println(F("[GSM] Inisialisasi modem..."));
  gsmModemReady = modem.init();
  if (gsmModemReady) {
    Serial.print(F("[GSM] Modem info: "));
    Serial.println(modem.getModemInfo());
    sim808Serial.println(F("AT+CLTS=1"));
    delay(200);
    while (sim808Serial.available()) sim808Serial.read();
  } else {
    Serial.println(F("[GSM] modem.init() gagal — cek wiring/VCC SIM808 (SMS tidak akan bisa terkirim)"));
  }

  sim808Serial.println(F("AT+CGNSTST=1"));
  delay(200);
  while (sim808Serial.available()) sim808Serial.read();

  // TIDAK ADA LAGI setup gsmSecure/gsmPlain di sini -- modem sudah tidak
  // dipakai sebagai transport HTTP/TLS untuk data.
}

// ============================================================
// STEP COUNTER
// ============================================================
void resetStepDetection() {
  stepCount    = 0;
  lastStepTime = 0;
  wasAboveThreshold = false;
  baseline = 1.0f;
  smoothIdx    = 0; smoothFull = false;
  for (int i = 0; i < SMOOTH_SIZE; i++) smoothBuf[i] = 1.0f;
  isCalibrating  = true;
  calibStartTime = millis();
  calibSum = 0; calibSumSq = 0; calibCount = 0;   // FIX: reset akumulator stddev juga
}

float getSmoothed() {
  int n = smoothFull ? SMOOTH_SIZE : smoothIdx;
  if (n == 0) return 1.0f;
  float s = 0;
  for (int i = 0; i < n; i++) s += smoothBuf[i];
  return s / n;
}

void detectStep(float mag, unsigned long now) {
  if (isCalibrating) {
    calibSum   += mag;
    calibSumSq += mag * mag;      // FIX: akumulasi buat variance
    calibCount++;
    if (now - calibStartTime >= CALIBRATION_MS) {
      baseline = calibSum / calibCount;

      float variance = (calibSumSq / calibCount) - (baseline * baseline);
      if (variance < 0) variance = 0;   // jaga2 dari floating point error
      float noiseStd = sqrtf(variance);

      isCalibrating = false;
      dynamicPeakDelta = fmaxf(STEP_MIN_DELTA, noiseStd * STEP_NOISE_MULT);

      for (int i = 0; i < SMOOTH_SIZE; i++) smoothBuf[i] = baseline;
      if (isOnline()) {
        char path[60]; FirebaseJson jc;
        jc.set("baseline_accel", (double)baseline);
        jc.set("step_low",       (double)(baseline + dynamicPeakDelta * STEP_EXIT_RATIO));
        jc.set("step_high",      (double)(baseline + dynamicPeakDelta));
        jc.set("calib_done",     true);
        snprintf(path, sizeof(path), "%s/adaptive", DB);
        fbUpdate(path, jc);
      }
      Serial.printf("[KALIBRASI] baseline=%.4f noiseStd=%.4f delta=%.4f\n",
                    baseline, noiseStd, dynamicPeakDelta);
    }
    return;
  }

  smoothBuf[smoothIdx] = mag;
  smoothIdx = (smoothIdx + 1) % SMOOTH_SIZE;
  if (smoothIdx == 0) smoothFull = true;
  float sm = getSmoothed();

  static unsigned long tDbg = 0;
  if (now - tDbg >= 200) {
    tDbg = now;
    Serial.printf("SM=%.3f BASE=%.3f DIFF=%.3f\n", sm, baseline, sm - baseline);
  }

  if (fabsf(sm - baseline) < dynamicPeakDelta * 0.5f)
    baseline = baseline*(1.0f-BASELINE_ALPHA) + sm*BASELINE_ALPHA;

  float triggerLevel = baseline + dynamicPeakDelta;

  float exitRatioNow   = lastStepWasHard ? STEP_HARD_EXIT_RATIO : STEP_EXIT_RATIO;
  float exitLevel      = baseline + dynamicPeakDelta * exitRatioNow;
  unsigned long rearmMinMs   = lastStepWasHard ? STEP_HARD_REARM_MS   : STEP_REARM_MIN_MS;
  unsigned long debounceNow  = lastStepWasHard ? STEP_HARD_DEBOUNCE_MS : DEBOUNCE_MS;

  bool isAboveNow;
  if (wasAboveThreshold) {
    isAboveNow = (sm > exitLevel);

    if (!isAboveNow) {
      if (!belowExitPending) {
        belowExitPending = true;
        belowExitSince   = now;
      }
      if (now - belowExitSince < rearmMinMs) {
        isAboveNow = true;   // paksa belum siap
      }
    } else {
      belowExitPending = false;
    }
  } else {
    isAboveNow = (sm > triggerLevel);
  }

  bool crossedTrigger = (sm > triggerLevel);
  if (crossedTrigger && !wasAboveThreshold && (now - lastStepTime) > debounceNow) {
    stepCount++;

    float excess = sm - triggerLevel;
    lastStepWasHard = (excess > dynamicPeakDelta * (STEP_HARD_RATIO - 1.0f));

    Serial.printf("[STEP %d] mag=%.4f base=%.4f delta=%.4f gap=%lums hard=%d\n",
                  stepCount, sm, baseline, dynamicPeakDelta, now - lastStepTime, lastStepWasHard);
    lastStepTime = now;
    belowExitPending = false;
  }

  wasAboveThreshold = isAboveNow;
}

void updateStepCounter() {
  if (!mpuReady) return;
  if (buzzerActive) return;                                   // buzzer nyala -> jangan hitung
  if (millis() - buzzerOffTime < BUZZ_SETTLE_MS) return;       // baru mati -> masih ada sisa getaran

  unsigned long now = millis();
  if (now - tStepSample < STEP_SAMPLE_INTERVAL_MS) return;
  tStepSample = now;
  int16_t rax, ray, raz;
  mpu.getAcceleration(&rax, &ray, &raz);
  float mag = sqrtf((rax/16384.0f)*(rax/16384.0f)
                  + (ray/16384.0f)*(ray/16384.0f)
                  + (raz/16384.0f)*(raz/16384.0f));
  detectStep(mag, now);
}

// ============================================================
// ANOMALY DETECTION
// ============================================================
void updateAnomalyWindow(float totF) {
  accelWindow[winIdx] = totF;
  winIdx = (winIdx+1) % ANOMALY_WINDOW;
  if (winIdx == 0) winFull = true;
}

float computeAnomalyScore(float cur) {
  if (!winFull) return 0.0f;
  float sum = 0;
  for (uint8_t i = 0; i < ANOMALY_WINDOW; i++) sum += accelWindow[i];
  float mean = sum / ANOMALY_WINDOW;
  float vsum = 0;
  for (uint8_t i = 0; i < ANOMALY_WINDOW; i++) vsum += (accelWindow[i]-mean)*(accelWindow[i]-mean);
  float sd = sqrtf(vsum/ANOMALY_WINDOW);
  return (sd < 0.01f) ? 0.0f : fabsf(cur-mean)/sd;
}

// ── Catat waktu kejadian anomali ke circular buffer ─────────
void logAnomalyEvent(unsigned long now) {
  anomalyEventLog[anomalyEventLogIdx] = now;
  anomalyEventLogIdx = (anomalyEventLogIdx + 1) % ANOMALY_REPEAT_LOG_SIZE;
  if (anomalyEventLogCount < ANOMALY_REPEAT_LOG_SIZE) anomalyEventLogCount++;
}

// ── Hitung berapa kejadian anomali yang masih dalam window
// ANOMALY_REPEAT_WINDOW_MS terakhir (dipakai buat deteksi limbung
// berulang) ──────────────────────────────────────────────────
uint8_t countRecentAnomalyEvents(unsigned long now) {
  uint8_t count = 0;
  for (uint8_t i = 0; i < anomalyEventLogCount; i++) {
    if (now - anomalyEventLog[i] <= ANOMALY_REPEAT_WINDOW_MS) count++;
  }
  return count;
}

// ============================================================
// CEK ANOMALY -- jalur notifikasi independen dari fuzzy logic.
// Dipanggil dari DALAM checkFall(), baik di jalur pre-filter (AMAN
// cepat) maupun jalur fuzzy penuh. Parameter statusAmanFuzzy = true
// berarti fuzzy logic sudah menyimpulkan AMAN pada siklus ini, sehingga
// jalur anomaly boleh ambil alih keputusan notifikasi tanpa bentrok
// dengan notifikasi fuzzy WASPADA/BAHAYA yang sudah ada.
//
// DIPERKUAT (2 lapisan tambahan):
//   1) anomalySevere -- Z-score sangat tinggi (>5.0) langsung dianggap
//      event tanpa perlu debounce 3 siklus.
//   2) isRepeatedPattern -- kalau kejadian anomali (tunggal maupun
//      ekstrem) sudah terjadi >=3 kali dalam 90 detik terakhir,
//      dieskalasi jadi notifikasi + history level "darurat" dan ikut
//      mengirim SMS ke caregiver, persis seperti trigger darurat lain
//      (jatuh, geofence). Ini implementasi konkret dari narasi
//      "getaran/limbung berulang" sebagai peringatan dini.
// ============================================================
void checkAnomalyTrigger(bool statusAmanFuzzy) {
  bool anomalyHigh = (anomalyScore > ANOMALY_ALERT_THRESHOLD);

  if (anomalyHigh) {
    anomalyTrueCnt++;  anomalyFalseCnt = 0;
    if (anomalyTrueCnt >= ANOMALY_DEBOUNCE_COUNT) anomalyConfirmed = true;
  } else {
    anomalyFalseCnt++;  anomalyTrueCnt = 0;
    if (anomalyFalseCnt >= ANOMALY_DEBOUNCE_COUNT) anomalyConfirmed = false;
  }

  bool anomalySevere = (anomalyScore > ANOMALY_SEVERE_THRESHOLD);

  bool isAnomalyEvent = (anomalyConfirmed || anomalySevere) && !fallHoldActive;

  unsigned long now = millis();

  if (isAnomalyEvent && !prevAnomalyEvent &&
      (now - lastNotifAnomaly > ANOMALY_NOTIF_COOLDOWN) && isOnline()) {

    // Catat kejadian ini untuk deteksi pola berulang
    logAnomalyEvent(now);
    anomalyRepeatCount = countRecentAnomalyEvents(now);
    bool isRepeatedPattern = (anomalyRepeatCount >= ANOMALY_REPEAT_ESCALATE);

    const char* level = (anomalySevere || isRepeatedPattern) ? "darurat" : "waspada";
    const char* title  = isRepeatedPattern
        ? "Pola Gerakan Tidak Stabil Berulang"
        : "Gerakan Tidak Biasa Terdeteksi";

    char msg[110];
    if (isRepeatedPattern) {
      snprintf(msg, sizeof(msg),
        "Terdeteksi %d kejadian anomali dalam %lu detik terakhir (skor=%.2f) -- indikasi limbung berulang",
        anomalyRepeatCount, ANOMALY_REPEAT_WINDOW_MS / 1000UL, anomalyScore);
    } else {
      snprintf(msg, sizeof(msg),
        "Pola gerakan tidak biasa terdeteksi (skor anomali=%.2f)", anomalyScore);
    }

    pushNotification(title, msg, level, 0.0f, 0);
    pushHistory(
      title, msg, "Anomali Gerakan",
      isRepeatedPattern ? "bahaya" : "peringatan",
      level, "anomaly_detection",
      isRepeatedPattern ? "anomaly_repeated" : "anomaly_movement",
      "MPU6050 (Anomaly Score)", anomalyScore,
      isRepeatedPattern ? "Notifikasi darurat dikirim" : "Peringatan dikirim ke aplikasi",
      "Pantau pola aktivitas lansia"
    );

    // Eskalasi ke SMS kalau ekstrem/berulang -- konsisten dengan
    // perlakuan trigger darurat lain (jatuh, geofence).
    if (isRepeatedPattern || anomalySevere) {
      char smsMsg[80];
      snprintf(smsMsg, sizeof(smsMsg), "%s (skor=%.2f, %dx/%lus)",
               title, anomalyScore, anomalyRepeatCount,
               ANOMALY_REPEAT_WINDOW_MS / 1000UL);
      sendEmergencySMS("ANOMALI", String(smsMsg));
    }

    lastNotifAnomaly = now;
    Serial.printf("[ANOMALY] %s score=%.2f repeat=%d/%lus severe=%d\n",
                  title, anomalyScore, anomalyRepeatCount,
                  ANOMALY_REPEAT_WINDOW_MS / 1000UL, anomalySevere);
  }
  prevAnomalyEvent = isAnomalyEvent;
}

// ============================================================
// FUZZY MEMBERSHIP FUNCTIONS
// ============================================================
float mf_trapLeft(float x, float a, float b) {
  if (x <= a) return 1.0f;
  if (x >= b) return 0.0f;
  return (b-x)/(b-a);
}
float mf_triangle(float x, float a, float b, float c) {
  if (x <= a || x >= c) return 0.0f;
  if (x <= b) return (x-a)/(b-a);
  return (c-x)/(c-b);
}
float mf_trapRight(float x, float a, float b) {
  if (x >= b) return 1.0f;
  if (x <= a) return 0.0f;
  return (x-a)/(b-a);
}

// ============================================================
// FUZZY FALL RISK
// ============================================================
float fuzzyFallRisk(float peakImpact, float peakGyro,
                    float azFiltered, float diamDetik) {

  float imp_ringan = mf_trapLeft (peakImpact, 0.00f, 0.30f);
  float imp_sedang = mf_triangle (peakImpact, 0.25f, 0.65f, 1.10f);
  float imp_keras  = mf_trapRight(peakImpact, 0.90f, 1.50f);

  float gyro_pelan = mf_trapLeft (peakGyro,   0.0f,  60.0f);
  float gyro_cepat = mf_trapRight(peakGyro, 100.0f, 180.0f);

  float az_tegak  = mf_trapRight(azFiltered, 0.85f, 1.00f);
  float az_miring = mf_triangle (azFiltered, 0.20f, 0.55f, 0.90f);
  float az_datar  = mf_trapLeft (azFiltered, 0.10f, 0.50f);

  float diam_sebentar    = mf_trapLeft (diamDetik,  0.0f,  8.0f);
  float diam_lama        = mf_triangle (diamDetik,  5.0f, 15.0f, 35.0f);
  float diam_sangat_lama = mf_trapRight(diamDetik, 25.0f, 45.0f);

  float r1 = fminf(gyro_cepat, az_datar);
  float r2 = fminf(imp_keras, az_datar);
  float r3 = fminf(imp_keras, diam_lama);
  float r4 = fminf(az_datar, diam_sangat_lama);
  float r5 = fminf(fminf(imp_sedang, az_datar), gyro_cepat);
  float r6 = fminf(imp_keras, az_miring);

  float notKeras     = 1.0f - imp_keras;
  float notGyroCepat = 1.0f - gyro_cepat;
  float r7  = fminf(az_miring, notKeras);
  float r8  = fminf(gyro_pelan == 0 ? 0 : 1.0f, az_tegak);
  float r9  = fminf(imp_sedang, diam_lama);
  float r10 = fminf(fminf(az_datar, diam_sebentar), notGyroCepat);

  float r11 = fminf(imp_ringan, az_tegak);
  float r12 = fminf(fminf(imp_sedang, az_tegak), gyro_pelan);

  float out_bahaya  = fmaxf(fmaxf(r1, r2), fmaxf(fmaxf(r3, r4), fmaxf(r5, r6)));
  float out_waspada = fmaxf(fmaxf(r7, r8), fmaxf(r9, r10));
  float out_aman    = fmaxf(r11, r12);

  float num = (out_aman * 0.15f) + (out_waspada * 0.37f) + (out_bahaya * 0.80f);
  float den = out_aman + out_waspada + out_bahaya;

  Serial.printf(
    "[FUZZY-DBG] imp:R=%.2f S=%.2f K=%.2f | "
    "gyro:P=%.2f C=%.2f | "
    "az:T=%.2f M=%.2f D=%.2f | "
    "diam:Sb=%.2f L=%.2f SL=%.2f\n",
    imp_ringan, imp_sedang, imp_keras,
    gyro_pelan, gyro_cepat,
    az_tegak, az_miring, az_datar,
    diam_sebentar, diam_lama, diam_sangat_lama
  );
  Serial.printf(
    "[FUZZY-DBG] R1=%.2f R2=%.2f R3=%.2f R4=%.2f "
    "R5=%.2f R6=%.2f R7=%.2f R8=%.2f R9=%.2f\n",
    r1, r2, r3, r4, r5, r6, r7, r8, r9
  );
  Serial.printf(
    "[FUZZY-DBG] out: AMAN=%.2f WASPADA=%.2f BAHAYA=%.2f\n",
    out_aman, out_waspada, out_bahaya
  );

  if (den < 0.001f) {
    Serial.println(F("[FUZZY-DBG] FALLBACK triggered"));
    return (azFiltered < 0.40f) ? 0.50f : 0.10f;
  }

  return num / den;
}

// ============================================================
// BACA SENSOR
// ============================================================
void readSensors() {
  if (mpuReady) {
    int16_t rax, ray, raz, rgx, rgy, rgz;
    mpu.getMotion6(&rax, &ray, &raz, &rgx, &rgy, &rgz);
    ax = rax/16384.0f; ay = ray/16384.0f; az = raz/16384.0f;
    gx = rgx/131.0f;   gy = rgy/131.0f;   gz = rgz/131.0f;
  }

  distFront        = calcDist(TRIG_FRONT, ECHO_FRONT);
  obstacleDetected = (distFront > 0 && distFront <= OBSTACLE_TH);

  delay(30);

  float rawBack = calcDist(TRIG_BACK, ECHO_BACK);

  bool isOutlier = fabsf(rawBack - lastGoodBack) > 25.0f;
  bool jumpedFar  = (rawBack > lastGoodBack) && isOutlier;
  float usedBack;
  unsigned long nowMs = millis();
  bool stuckTooLong = (nowMs - lastGoodBackTime) > BACK_STUCK_TIMEOUT_MS;

  if (isOutlier) {
    backOutlierStreak++;

    uint8_t requiredStreak = jumpedFar ? 5 : 3;

    if (backOutlierStreak >= requiredStreak || stuckTooLong) {
      usedBack = rawBack;
      lastGoodBack = rawBack;
      lastGoodBackTime = nowMs;
      backOutlierStreak = 0;
    } else {
      usedBack = lastGoodBack;
    }
  } else {
    usedBack = rawBack;
    lastGoodBack = rawBack;
    lastGoodBackTime = nowMs;
    backOutlierStreak = 0;
  }

  backBuf[backBufIdx] = usedBack;
  backBufIdx = (backBufIdx + 1) % BACK_SMOOTH_SIZE;
  float sumBack = 0;
  for (uint8_t i = 0; i < BACK_SMOOTH_SIZE; i++) sumBack += backBuf[i];
  distBack = sumBack / BACK_SMOOTH_SIZE;

  bool inEnterRange = (distBack >= USER_ENTER_MIN_CM && distBack <= USER_ENTER_MAX_CM);
  bool inExitRange  = (distBack >= USER_EXIT_MIN_CM  && distBack <= USER_EXIT_MAX_CM);

  if (userConfirmed) {
    userDetected = inExitRange;
  } else {
    userDetected = inEnterRange;
  }

  Serial.printf("[HCSR] Dpn=%.1fcm(%s) Blk=%.1fcm(%s)\n",
    distFront, obstacleDetected ? "HAMBATAN" : "aman",
    distBack,  userDetected     ? "OK"       : "tdk ada");

  updateBuzzer();

  if (obstacleDetected) {
    obstacleTrueCnt++;  obstacleFalseCnt = 0;
    if (obstacleTrueCnt  >= OBSTACLE_DEBOUNCE_COUNT) obstacleConfirmed = true;
  } else {
    obstacleFalseCnt++;  obstacleTrueCnt = 0;
    if (obstacleFalseCnt >= OBSTACLE_DEBOUNCE_COUNT) obstacleConfirmed = false;
  }

  static bool prevObstacle = false;

  if (obstacleConfirmed && !prevObstacle &&
      (millis() - lastNotifObstacle > NOTIF_COOLDOWN)) {

      char msg[48];
      snprintf(msg, sizeof(msg), "Objek terdeteksi %.0f cm di depan", distFront);

      if (isOnline()) {
        pushNotification("Hambatan Terdeteksi", msg, "waspada", 0.5f, 0);
        pushHistory(
          "Hambatan Terdeteksi", msg, "Hambatan Terdeteksi",
          "waspada", "waspada", "obstacle_detection", "obstacle_detected",
          "HC-SR04 Front", 0.0f, "Peringatan dikirim ke aplikasi", ""
        );
      }

      lastNotifObstacle = millis();
  }
  prevObstacle = obstacleConfirmed;

  if (userDetected) {
    userTrueCnt++;  userFalseCnt = 0;
    if (userTrueCnt  >= NOUSER_DEBOUNCE_COUNT) userConfirmed = true;
  } else {
    userFalseCnt++;  userTrueCnt = 0;
    if (userFalseCnt >= NOUSER_DEBOUNCE_COUNT) userConfirmed = false;
  }

  unsigned long now = millis();
  if (userConfirmed) tUserLastSeen = now;

  static bool prevNoUser = false;
  bool noUserAlarm = (!userConfirmed) && (now - tUserLastSeen > NO_USER_ALARM_DELAY);
  if (noUserAlarm && !prevNoUser && (now - lastNotifNoUser > NOTIF_COOLDOWN)) {
    char msg[64];
    snprintf(msg, sizeof(msg), "Lansia tidak terdeteksi oleh sensor belakang");

    if (isOnline()) {
      pushNotification("Lansia Tidak Terdeteksi", msg, "darurat", 0.4f,
                       (int)((now - tUserLastSeen) / 1000));
      pushHistory(
        "Lansia Tidak Terdeteksi", msg, "Kehilangan Deteksi",
        "darurat", "darurat", "user_detection", "user_missing",
        "HC-SR04 Back", 0.0f, "Peringatan dikirim ke aplikasi", ""
      );
    }
    lastNotifNoUser = now;
  }
  prevNoUser = noUserAlarm;
}

void readGPS() {
  if (gps.location.isValid() && gps.location.age() < 30000) {
    latitude  = (float)gps.location.lat();
    longitude = (float)gps.location.lng();
    gpsReady  = true;
  } else {
    gpsReady = false;
  }
}

// ============================================================
// CHECK FALL — ENTRY POINT
// ============================================================
void checkFall() {
  if (!mpuReady) return;

  unsigned long now = millis();

  axBuf[accelIdx] = ax;
  ayBuf[accelIdx] = ay;
  azBuf[accelIdx] = az;
  accelIdx = (accelIdx + 1) % 4;
  axF = (axBuf[0]+axBuf[1]+axBuf[2]+axBuf[3]) / 4.0f;
  ayF = (ayBuf[0]+ayBuf[1]+ayBuf[2]+ayBuf[3]) / 4.0f;
  azF = (azBuf[0]+azBuf[1]+azBuf[2]+azBuf[3]) / 4.0f;

  float tot  = sqrtf(ax*ax + ay*ay + az*az);
  float totF = sqrtf(axF*axF + ayF*ayF + azF*azF);

  // FIX: peakImpact/peakGyro sekarang diambil dari fast-sampling (10ms)
  // sejak checkFall() terakhir dipanggil, digabung (ambil nilai terbesar)
  // dengan sample mentah saat ini sebagai jaga-jaga tambahan.
  float instantImpact = fabsf(tot - 1.0f);
  float peakImpact = fmaxf(fastPeakImpact, instantImpact);

  float instantGyro = sqrtf(gx*gx + gy*gy + gz*gz);
  float peakGyro = fmaxf(fastPeakGyro, instantGyro);

  // reset fast tracker buat window 500ms berikutnya
  fastPeakImpact = 0.0f;
  fastPeakGyro   = 0.0f;

  float diamFromHCSR = userDetected ? 0.0f
    : constrain((float)((now - tUserLastSeen) / 1000UL), 0.0f, 60.0f);

  if (fabsf(totF - 1.0f) < DIAM_MPU_MARGIN) {
    if (!diamMpuActive) { diamMpuActive = true; diamMpuStart = now; }
  } else {
    diamMpuActive = false;
  }
  float diamFromMPU = diamMpuActive
    ? constrain((float)((now - diamMpuStart) / 1000UL), 0.0f, 60.0f)
    : 0.0f;

  diamSecs = fmaxf(diamFromHCSR, diamFromMPU);

  updateAnomalyWindow(totF);
  anomalyScore = computeAnomalyScore(totF);

  gPeakImpact = peakImpact;
  gPeakGyro   = peakGyro;
  gTotAccel   = tot;

  if (peakImpact < FALL_PRE_FILTER_TH && diamSecs < 10.0f && azF >= 0.85f) {
    gFuzzyRisk     = 0.0f;
    gReportConf    = 0.0f;
    gJatuhFirebase = false;

    // Cek anomaly di jalur pre-filter -- fuzzy sudah pasti AMAN di
    // jalur ini, jadi anomaly boleh langsung dievaluasi.
    checkAnomalyTrigger(true);

    if (fallHoldActive && (now - fallHoldStart > FALL_HOLD_DURATION_MS)) {
      fallHoldActive = false;
      Serial.println(F("[FALL] Hold selesai"));
    }

    if (!fallHoldActive) {
      prevFall = false;
    }
    return;
  }

  float fallRisk = fuzzyFallRisk(peakImpact, peakGyro, azF, diamSecs);
  float conf     = constrain(fallRisk, 0.0f, 1.0f);
  bool  jatuh    = (fallRisk >= FALL_FUZZY_THRESHOLD);

  Serial.printf(
    "[FUZZY] risk=%.3f | imp=%.3f gyro=%.1f az=%.3f diam=%.1fs | %s\n",
    fallRisk, peakImpact, peakGyro, azF, diamSecs,
    jatuh ? "JATUH!" : (fallRisk >= 0.30f ? "waspada" : "aman")
  );

  bool isNewFallEvent = (jatuh && !prevFall && !fallHoldActive);

  if (isNewFallEvent) {
    fallHoldActive = true;
    fallHoldStart  = now;
    lastFallRisk   = fallRisk;
    lastImpact     = peakImpact;
    lastGyro       = peakGyro;
    lastConf       = conf;

    char msg[72];
    snprintf(msg, sizeof(msg),
             "Risk=%.2f Imp=%.2fg Gyro=%.0f Diam=%.1fs",
             fallRisk, peakImpact, peakGyro, diamSecs);

    // ---- Trigger #1: SMS DULUAN -- event darurat, caregiver harus tau
    // secepat mungkin, gak perlu nunggu status WiFi. Cooldown
    // SMS_COOLDOWN_MS di dalam sendEmergencySMS() tetap jaga biar gak
    // spam kalau ada beberapa fall event beruntun. ------------------
    bool smsSent = sendEmergencySMS("JATUH", String(msg));

    // ---- Firebase tetap dicoba SETELAHNYA (kalau WiFi ada), buat data
    // riwayat & tampilan real-time di aplikasi --------------------------
    bool deliveredOnline = false;
    if (isOnline()) {
      bool notifOk = pushNotification("Potensi Jatuh Terdeteksi", msg, "darurat", conf,
                                       (int)(diamSecs));
      pushHistory(
        "Potensi Jatuh Terdeteksi", msg, "Deteksi Jatuh",
        "bahaya", "darurat", "fall_detection", "fall_detected",
        "MPU6050", conf, "Notifikasi darurat dikirim", "Periksa kondisi lansia"
      );

      char dts[20]; getDTS(dts, sizeof(dts));
      char path[48]; makePath(path, sizeof(path), "/fall_detetction");
      FirebaseJson j;
      j.set("jatuh",         true);
      j.set("fall_detected", true);
      j.set("confidence",    (double)conf);
      j.set("impact",        (double)peakImpact);
      j.set("impact_value",  (double)tot);
      j.set("fuzzy_risk",    (double)fallRisk);
      j.set("gyro_peak",     (double)peakGyro);
      j.set("az_filtered",   (double)azF);
      j.set("diam_detik",    (double)diamSecs);
      j.set("anomaly_score", (double)anomalyScore);
      j.set("timestamp",     dts);
      bool fbOk = fbUpdate(path, j);

      deliveredOnline = notifOk && fbOk;
    }

    showOLED("!! JATUH !!", msg,
             smsSent ? "SMS terkirim" : (deliveredOnline ? "Notif terkirim" : "Gagal kirim!"));
    Serial.printf("[FALL] TERDETEKSI! risk=%.3f imp=%.3f gyro=%.1f diam=%.1f | sms=%d online=%d\n",
                  fallRisk, peakImpact, peakGyro, diamSecs, smsSent, deliveredOnline);
  }

  // ── WASPADA: gerakan berisiko (fuzzy risk 0.30–0.45, belum "jatuh") ──
  bool isWaspada = (fallRisk >= 0.30f && !jatuh && !fallHoldActive);

  if (isWaspada && !prevWaspada &&
      (now - lastNotifWaspada > NOTIF_COOLDOWN) && isOnline()) {

    char msg[80];
    snprintf(msg, sizeof(msg),
            "Gerakan Berisiko Terdeteksi, Risk=%.2f Imp=%.2fg Gyro=%.0f",
            fallRisk, peakImpact, peakGyro);

    pushNotification("Gerakan Berisiko Terdeteksi", msg, "waspada", conf, 0);
    pushHistory(
      "Gerakan Berisiko Terdeteksi", msg, "Gerakan Berisiko",
      "peringatan", "waspada", "aktivitas", "risky_movement",
      "MPU6050", conf, "Peringatan dikirim ke aplikasi",
      "Pantau lansia lebih dekat"
    );

    lastNotifWaspada = now;
    Serial.printf("[WASPADA] Gerakan berisiko! risk=%.3f\n", fallRisk);
  }
  prevWaspada = isWaspada;

  if (fallHoldActive && (now - fallHoldStart > FALL_HOLD_DURATION_MS)) {
    fallHoldActive = false;
    Serial.println(F("[FALL] Hold selesai, reset ke normal"));
  }

  bool jatuhFirebase = jatuh || fallHoldActive;

  gFuzzyRisk     = jatuhFirebase ? lastFallRisk : fallRisk;
  gReportConf    = jatuhFirebase ? lastConf     : conf;
  gPeakImpact    = jatuhFirebase ? lastImpact   : peakImpact;
  gPeakGyro      = jatuhFirebase ? lastGyro     : peakGyro;
  gJatuhFirebase = jatuhFirebase;

  // Cek anomaly di jalur fuzzy penuh -- hanya dievaluasi kalau fuzzy
  // logic sendiri sudah menyimpulkan AMAN pada siklus ini (fallRisk <
  // 0.30 dan tidak sedang jatuh/hold), supaya tidak bentrok dengan
  // notifikasi WASPADA/BAHAYA yang sudah dikirim oleh fuzzy.
  bool statusAmanFuzzy = (!jatuh && !fallHoldActive && fallRisk < 0.30f);
  checkAnomalyTrigger(statusAmanFuzzy);

  prevFall = jatuh;
}

// ============================================================
// GEOFENCE
// ============================================================
void checkGeofence() {
  if (!gpsReady) return;
  if (latitude == 0.0f && longitude == 0.0f) return;
  if (gps.location.age() > 30000) return;
  if (gps.satellites.isValid() && gps.satellites.value() < 3) return;

  jarakDariPusat = haversine(latitude, longitude, geoLat, geoLng);
  bool outside   = (jarakDariPusat > geoRadius);

  static bool prevNear = false;
  bool near = (!outside) && ((geoRadius - jarakDariPusat) <= 2.0f);

  if (outside && !prevOutside) {
    char msg[48];
    snprintf(msg, sizeof(msg), "Lansia keluar %.0f m dari area aman", jarakDariPusat);

    // ---- Trigger #2: SMS pas keluar area aman -- sama seperti fall
    // event, dicoba tanpa syarat status WiFi ------------------------
    sendEmergencySMS("GEOFENCE", String(msg));

    bool deliveredOnline = false;
    if (isOnline()) {
      deliveredOnline = pushNotification("Keluar Area Aman", msg, "tinggi", 0.75f, 0);
      pushHistory(
        "Keluar Area Aman", msg, "Keluar Geofence",
        "bahaya", "darurat", "geofence", "geofence_violated",
        "GPS", 0.0f, "Notifikasi darurat dikirim", "Periksa lokasi lansia"
      );
    }
    showOLED("!! GEOFENCE !!", "Keluar area aman");
    prevNear = false;
  }
  else if (near && !prevNear) {
    char msg[64];
    snprintf(msg, sizeof(msg), "Lansia mendekati batas, sisa %.0f m",
             geoRadius - jarakDariPusat);

    if (isOnline()) {
      pushNotification("Mendekati Batas Area", msg, "sedang", 0.5f, 0);
      pushHistory(
        "Mendekati Batas Area", msg, "Mendekati Geofence",
        "peringatan", "waspada", "geofence_near", "geofence_approaching",
        "GPS", 0.0f, "Peringatan dini dikirim", "Pantau posisi lebih sering"
      );
    }
    showOLED("!! WASPADA !!", "Mendekati batas", "area aman");
  }

  prevOutside     = outside;
  prevNear        = near;
  geofenceOutside = outside;
}

// ============================================================
// NOTIFIKASI
// ============================================================
bool pushNotification(const char* title, const char* description,
                      const char* level, float scoreFall, int durasiDetik) {
  notifCounter++;
  char notifId[16];
  snprintf(notifId, sizeof(notifId), "notif_%03lu", notifCounter);

  char path[80];
  snprintf(path, sizeof(path), "%s/notification/%s", DB, notifId);

  char dts[20]; getDTS(dts, sizeof(dts));
  char dateOnly[12], timeOnly[10];
  strncpy(dateOnly, dts, 10); dateOnly[10] = '\0';
  strncpy(timeOnly, dts+11, 8); timeOnly[8] = '\0';

  FirebaseJson j;
  j.set("id",          notifId);
  j.set("title",       title);
  j.set("description", description);
  j.set("level",       level);
  j.set("score",       (double)scoreFall);
  j.set("durasiDetik", durasiDetik);
  j.set("sudahDibaca", false);
  j.set("timestamp",   dts);
  j.set("date",        dateOnly);
  j.set("time",        timeOnly);
  j.set("latitude",    (double)latitude);
  j.set("longitude",   (double)longitude);

  bool ok = fbSet(path, j);
  if (!ok) {
    Serial.println(F("[NOTIF] GAGAL kirim"));
  } else {
    Serial.printf("[NOTIF] %s -> %s\n", notifId, title);
  }

  char metaPath[60];
  snprintf(metaPath, sizeof(metaPath), "%s/meta/notifCount", DB);
  fbSetInt(metaPath, (int)notifCounter);
  return ok;
}

// ============================================================
// HISTORY
// ============================================================
void pushHistory(const char* title, const char* subtitle,
                 const char* jenisKejadian, const char* status,
                 const char* statusBahaya,  const char* category,
                 const char* type,          const char* sensor,
                 float skorAnomali,
                 const char* tindakan1,     const char* tindakan2) {
  historyCounter++;
  uint32_t histCount = historyCounter;

  char histId[16]; snprintf(histId, sizeof(histId), "history_%03lu", histCount);
  char path[80];   snprintf(path, sizeof(path), "%s/history/%s", DB, histId);
  char dts[20];    getDTS(dts, sizeof(dts));
  char coordStr[32]; snprintf(coordStr, sizeof(coordStr), "%.6f,%.6f", latitude, longitude);

  float jarakPusat = (latitude != 0.0f && longitude != 0.0f)
                     ? haversine(latitude, longitude, geoLat, geoLng) : 0.0f;

  char jarakTerukur[16];
  if (distFront < 999) snprintf(jarakTerukur, sizeof(jarakTerukur), "%.0f cm", distFront);
  else strncpy(jarakTerukur, "N/A", sizeof(jarakTerukur));

  FirebaseJson j;
  j.set("title",           title);
  j.set("subtitle",        subtitle);
  j.set("jenisKejadian",   jenisKejadian);
  j.set("status",          status);
  j.set("statusBahaya",    statusBahaya);
  j.set("category",        category);
  j.set("type",            type);
  j.set("sensor",          sensor);
  j.set("skorAnomali",     (double)skorAnomali);
  j.set("timestamp",       dts);
  j.set("lokasiKoordinat", coordStr);
  j.set("lokasiNama",      "Unknown");
  j.set("lokasiJarakPusat",(double)jarakPusat);
  j.set("jarakTerukur",    jarakTerukur);
  j.set("statusDeteksiLansia", userDetected ? "Terdeteksi" : "Tidak Terdeteksi");
  j.set("hcsrJarak",     (int)(distBack < 999 ? distBack : 0));
  j.set("hcsrStatus",    userDetected ? "aman" : "tidak terdeteksi");
  j.set("hcsrThreshold", USER_ENTER_MAX_CM);

  FirebaseJson dist;
  dist.set("hcsr04_front", (int)(distFront < 999 ? distFront : 0));
  dist.set("hcsr04_back",  (int)(distBack  < 999 ? distBack  : 0));
  j.set("distanceData", dist);

  FirebaseJson sd;
  sd.set("accel_x",(double)ax); sd.set("accel_y",(double)ay); sd.set("accel_z",(double)az);
  sd.set("gyro_x", (double)gx); sd.set("gyro_y", (double)gy); sd.set("gyro_z", (double)gz);
  j.set("sensorData", sd);

  FirebaseJson ss;
  ss.set("gpsConnected", gpsReady);
  ss.set("gsmConnected", gpsReady);
  ss.set("imuNormal",    mpuReady);
  j.set("statusSistem", ss);

  FirebaseJsonArray ta;
  ta.add(tindakan1);
  if (tindakan2 && strlen(tindakan2) > 0) ta.add(tindakan2);
  j.set("tindakanSistem", ta);

  if (!fbSet(path, j)) {
    Serial.println(F("[HISTORY] GAGAL kirim"));
  } else {
    Serial.printf("[HISTORY] %s -> %s\n", histId, title);
  }

  char metaPath[60];
  snprintf(metaPath, sizeof(metaPath), "%s/meta/historyCount", DB);
  fbSetInt(metaPath, (int)histCount);
}

// ============================================================
// UPDATE FIREBASE — GABUNGAN (heartbeat + sensor + location + geofence +
// step + status fuzzy/fall rutin)
// ============================================================
void sendCombinedUpdate() {
  if (!isOnline()) return;
  char ts[12];  getTS(ts, sizeof(ts));
  char dts[20]; getDTS(dts, sizeof(dts));

  FirebaseJson j;

  // ---- Heartbeat ----
  j.set("walker_active", true);
  j.set("last_update",   dts);
  j.set("status", fallHoldActive ? "bahaya" : "normal");

  // ---- /sensors ----
  j.set("sensors/mpu_active",      mpuReady);
  j.set("sensors/mpu6050/accel_x", (double)ax);
  j.set("sensors/mpu6050/accel_y", (double)ay);
  j.set("sensors/mpu6050/accel_z", (double)az);
  j.set("sensors/mpu6050/gyro_x",  (double)gx);
  j.set("sensors/mpu6050/gyro_y",  (double)gy);
  j.set("sensors/mpu6050/gyro_z",  (double)gz);
  j.set("sensors/hcsr04_front/distance",          (double)(distFront < 999 ? distFront : 0.0f));
  j.set("sensors/hcsr04_front/obstacle_detected", obstacleDetected);
  j.set("sensors/hcsr04_back/distance",           (double)(distBack < 999 ? distBack : 0.0f));
  j.set("sensors/hcsr04_back/user_detected",      userDetected);
  j.set("sensors/ringkasanSensor/hcsr04_front",       obstacleDetected);
  j.set("sensors/ringkasanSensor/user_detected_back", userDetected);
  j.set("sensors/gps_active",             gpsReady);
  j.set("sensors/sim808/gps_status",      gpsReady);
  j.set("sensors/sim808/gsm_signal",      0);
  // internet_status sekarang murni status WiFi (GPRS sudah dihapus)
  j.set("sensors/sim808/internet_status", (WiFi.status() == WL_CONNECTED));
  j.set("sensors/sim808/satelite_count",  gps.satellites.isValid() ? (int)gps.satellites.value() : 0);
  j.set("sensors/statusSistem/gpsConnected", gpsReady);
  // gsmConnected sekarang berarti "modem SIM808 siap dipakai buat SMS",
  // bukan lagi status koneksi data GPRS.
  j.set("sensors/statusSistem/gsmConnected", gsmModemReady);
  j.set("sensors/statusSistem/imuNormal",    mpuReady);

  // ---- /location ----
  if (gpsReady) {
    j.set("location/latitude",    (double)latitude);
    j.set("location/longitude",   (double)longitude);
    j.set("location/last_update", dts);
  }

  // ---- /geofence ----
  // FIX #2 (root cause sebenarnya): "geofence" adalah key TOP-LEVEL di j,
  // dan FirebaseJson membangun "geofence/xxx" sebagai OBJEK NESTED
  // { "geofence": { "xxx": ... } }, bukan flat key literal "geofence/xxx".
  // Firebase PATCH/update() cuma merge di level PALING ATAS dari body --
  // begitu "geofence" jadi top-level key di PATCH ke /Walkers/walker_001,
  // SELURUH isi node /geofence ikut DIGANTI TOTAL dengan objek yang cuma
  // berisi jarakDariPusat/status/last_check -- menghapus
  // center_latitude/center_longitude/radius yang disimpan APLIKASI lewat
  // GeofenceSetupScreen, tiap kali fungsi ini jalan (tiap IV_HB)!
  //
  // FIX: kirim bagian ini lewat PATCH TERPISAH yang target-nya LANGSUNG
  // "/geofence" (bukan root). Dengan begitu jarakDariPusat/status/
  // last_check jadi top-level key dari PATCH itu sendiri, dan Firebase
  // cuma merge 3 field itu -- center_latitude/center_longitude/radius
  // (sibling-nya) otomatis tidak tersentuh/tidak hilang.
  {
    char geoPath[64]; makePath(geoPath, sizeof(geoPath), "/geofence");
    FirebaseJson jg;
    jg.set("jarakDariPusat", (double)jarakDariPusat);
    jg.set("status", !gpsReady ? "unknown" : geofenceOutside ? "outside" : "inside");
    jg.set("last_check", dts);
    fbUpdate(geoPath, jg);
  }

  // ---- /activity/langkah ----
  j.set("activity/langkah", stepCount);

  // ---- /fall_detetction (status rutin, numpang di sini — lihat checkFall()) ----
  j.set("fall_detetction/jatuh",         gJatuhFirebase);
  j.set("fall_detetction/fall_detected", gJatuhFirebase);
  j.set("fall_detetction/confidence",    (double)gReportConf);
  j.set("fall_detetction/impact",        (double)gPeakImpact);
  j.set("fall_detetction/impact_value",  (double)gTotAccel);
  j.set("fall_detetction/fuzzy_risk",    (double)gFuzzyRisk);
  j.set("fall_detetction/gyro_peak",     (double)gPeakGyro);
  j.set("fall_detetction/az_filtered",   (double)azF);
  j.set("fall_detetction/diam_detik",    (double)diamSecs);
  j.set("fall_detetction/anomaly_score", (double)anomalyScore);
  // BARU: hitung jumlah kejadian anomali dalam window terakhir (dipakai
  // app buat nampilin tren, dan jadi bukti data buat bab pengujian TA)
  j.set("fall_detetction/anomaly_repeat_count", (int)anomalyRepeatCount);
  j.set("fall_detetction/timestamp",     dts);

  bool ok = fbUpdate(DB, j);
  if (!ok) {
    Serial.println(F("[COMBINED] GAGAL kirim (HB+sensors+location+geofence+step+fall)"));
  } else {
    Serial.printf(
      "[COMBINED] OK | %s | hold=%d diam=%.1fs | net=WiFi | time=%s | "
      "gps=%s lat=%.6f lng=%.6f | langkah=%d\n",
      ts, fallHoldActive, diamSecs,
      timeSynced ? "sinkron" : "BELUM sinkron",
      gpsReady ? "OK" : "stale", latitude, longitude, stepCount);
  }

  // ---- OLED display ----
  if (!fallHoldActive && !obstacleDetected) {
    char depan[24], belakang[24];
    snprintf(depan, sizeof(depan), distFront<999 ? "Dpn: %dcm OK":"Dpn: ---", (int)distFront);
    snprintf(belakang, sizeof(belakang),
             distBack<999 ? (userDetected ? "Blk: %dcm OK":"Blk: %dcm JAUH"):"Blk: ---",
             (int)distBack);
    showOLED(depan, belakang,
             userDetected ? "Lansia: hadir":"Lansia: tdk ada",
             geofenceOutside ? "Geo: outside":"Geo: inside");
  } else if (fallHoldActive) {
    char riskStr[24];
    snprintf(riskStr, sizeof(riskStr), "Risk: %.2f Diam:%.0fs", lastFallRisk, diamSecs);
    showOLED("!! JATUH !!", riskStr,
             userDetected ? "Lansia: hadir":"Lansia: tdk ada", "");
  } else if (obstacleDetected) {
    char msg[24]; snprintf(msg, sizeof(msg), "Jarak: %d cm", (int)distFront);
    showOLED("!! HAMBATAN !!", msg, "Perlambat!",
             geofenceOutside ? "Geo: outside":"Geo: inside");
  }
}

// ============================================================
// SETUP
// ============================================================
void setup() {
  Serial.begin(115200); delay(400);
  Serial.println(F("== GuardianWalk Boot =="));

  pinMode(BUZZER_PIN, OUTPUT);
  ledcAttach(BUZZER_PIN, BUZZ_FREQ_OBSTACLE, 8);
  buzzerOff();
  Serial.println(F("[Buzzer] OK"));

  loadCaregiverPhoneFromNVS();

  setupOLED();
  showOLED("GuardianWalk", "Booting...");

  connectWiFi();

  setupMPU();
  setupSIM808();

  syncTime();

  pinMode(TRIG_FRONT, OUTPUT); pinMode(ECHO_FRONT, INPUT);

  pinMode(TRIG_BACK,  OUTPUT);
  pinMode(ECHO_BACK,  INPUT_PULLDOWN);
  digitalWrite(TRIG_BACK, LOW);

  // GPRS DIHAPUS -- kalau WiFi gak konek pas boot, firebaseReady langsung
  // false. Device TIDAK coba jalur data lain; SMS jadi satu-satunya
  // cadangan (lihat checkOfflineDuration(), sendEmergencySMS()).
  firebaseReady = (WiFi.status() == WL_CONNECTED);

  connectFirebase();
  registerWalker();
  loadGeofenceFromFirebase();

  syncCaregiverPhoneFromFirebase();

  if (mpuReady) {
    resetStepDetection();
    showOLED("GuardianWalk", "Kalibrasi step...", "Diam sebentar");
    delay(CALIBRATION_MS + 50);
  }

  for (int i = 0; i < 4; i++) {
    axBuf[i] = 0.0f; ayBuf[i] = 0.0f; azBuf[i] = 1.0f;
  }
  axF = 0.0f; ayF = 0.0f; azF = 1.0f;

  diamMpuActive = true;
  diamMpuStart  = millis();
  diamSecs      = 0.0f;
  tUserLastSeen = millis();
  tLastOnlineOk = millis();   // baseline buat checkOfflineDuration()
  lastGoodBackTime = millis();

  char ip[16];
  WiFi.localIP().toString().toCharArray(ip, sizeof(ip));
  showOLED("GuardianWalk", "Ready!", ip, "USB Mode");
  Serial.println(F("== Boot selesai =="));
}

// ============================================================
// LOOP
// ============================================================
void loop() {
  unsigned long now = millis();

  // ── TEST ONLY: ketik 's' + Enter di Serial Monitor buat tes SMS manual ──
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 's') {
      Serial.println(F("[TEST] Memicu SMS uji coba..."));
      sendEmergencySMS("TEST", String("Ini pesan uji coba dari GuardianWalk"));
    }
  }

  while (sim808Serial.available()) {
    char c = sim808Serial.read();
    gps.encode(c);
    Serial.write(c);
  }
  updateFastFallSampling(); 
  updateStepCounter();

  if (now - tSen >= IV_SEN) {
    tSen = now;
    readSensors();
    checkFall();
  }

  if (now - tGPS >= IV_GPS) { tGPS = now; readGPS(); }
  if (now - tGeo >= IV_GEO) { tGeo = now; checkGeofence(); }

  // GPRS dihapus -> interval reload geofence sekarang selalu IV_GEO_RELOAD
  // (dulu diperpanjang 5x kalau lagi pakai GPRS, sekarang gak perlu lagi).
  if (now - tGeoReload >= IV_GEO_RELOAD) {
    tGeoReload = now;
    loadGeofenceFromFirebase();
  }

  updateNetworkMode();   // cek status WiFi, update firebaseReady

  if (isOnline() && (now - tCaregiverSync >= IV_CAREGIVER_SYNC)) {
    tCaregiverSync = now;
    syncCaregiverPhoneFromFirebase();
  }

  checkOfflineDuration();   // Trigger #3: SMS kalau WiFi mati > OFFLINE_ALERT_THRESHOLD_MS

  if (!timeSynced && (now - tTimeRetry >= IV_TIME_RETRY)) {
    tTimeRetry = now;
    Serial.println(F("[TIME] Retry sinkronisasi waktu..."));
    syncTime();
  }

if (isOnline() && (now - tHB >= IV_HB)) {
    tHB = now;
    updateStepCounter();
    sendCombinedUpdate();
    updateStepCounter();
  }

  // FIX: reconnect WiFi non-blocking, dibatasi interval -- tidak lagi
  // dipanggil tiap loop() dan tidak lagi menahan CPU (delay) saat WiFi mati.
  if (WiFi.status() != WL_CONNECTED && (now - tWifiRetry >= IV_WIFI_RETRY)) {
    tWifiRetry = now;
    connectWiFiNonBlocking();
  }
}
