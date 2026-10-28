#include <Wire.h>
#include <math.h>
#include <WiFi.h>
#include <WebServer.h>

// =====================================================
// ESP32-C3 + MPU6050 + LED fade + Halaman Pengaturan via Wi-Fi AP
// SDA = GPIO 6, SCL = GPIO 7
// Sensor Address = 0x68
// =====================================================

const char *AP_SSID = "HEBAT_SMART_PIN";
const char *AP_PASS = "hebat123";

WebServer server(80);

const uint8_t IMU_ADDR = 0x68;

const int PIN_LED_MERAH  = 0;
const int PIN_LED_KUNING = 1;
const int PIN_LED_HIJAU  = 3;

// --- LED FADE (PWM) ---
const int PIN_LED[3] = {PIN_LED_MERAH, PIN_LED_KUNING, PIN_LED_HIJAU};
const int LANGKAH_FADE = 32; // 0 -> 255 dalam 8 tick (~160 ms di 50 Hz); naikkan = transisi lebih cepat
int levelLED[3] = {0, 0, 0};
bool targetLED[3] = {false, false, false};

// --- AMBANG BATAS DINAMIS (Bisa diubah via Web) ---
float batasSudutPostur = 30.0f;  
float batasGyroDps = 15.0f;      
float batasGerakAccelG = 0.30f; 

const unsigned long DURASI_PERINGATAN_MS = 2000UL; 
const unsigned long DURASI_BASELINE_MS = 5000UL;   
const unsigned long INTERVAL_BACA_MS = 20UL; // 50 Hz
const int JUMLAH_KALIBRASI = 50; 

const unsigned long DURASI_TUNDA_KUNING_MS = 500UL; 
unsigned long mulaiPotensiKuning = 0;
bool potensiKuningBerlangsung = false;

// --- LOGIKA PENAHAN SAAT MERAH ---
unsigned long waktuMulaiMerah = 0;
bool sedangMenahanMerah = false;
const unsigned long DURASI_TAHAN_MERAH_MS = 3000UL;

float baselinePitch = 0.0f;
float baselineRoll = 0.0f;

unsigned long waktuBacaTerakhir = 0;
unsigned long mulaiGerak = 0;
unsigned long mulaiPenyimpangan = 0;
unsigned long mulaiStabilPosisiBaru = 0;

bool gerakSedangBerlangsung = false;
bool penyimpanganSedangBerlangsung = false;
bool posisiBaruSedangStabil = false;

// Variabel Smoothing Filter (EMA)
float prevAx = 0, prevAy = 0, prevAz = 1.0f;
float prevGx = 0, prevGy = 0, prevGz = 0;
float filteredPitch = 0.0f;
float filteredRoll = 0.0f;
bool filterSiap = false; // false = bacaan pertama dipakai langsung sebagai nilai awal filter

// Di 50 Hz + DLPF hardware 44 Hz: konstanta waktu ~80 ms (raw) dan ~60 ms (sudut),
// lag total lebih kecil dari versi 15 Hz. Turunkan alpha = lebih halus tapi lebih lambat.
const float FILTER_ALPHA_RAW = 0.2f;
const float FILTER_ALPHA_ANGLE = 0.25f;

struct DataIMU {
  int16_t axRaw, ayRaw, azRaw;
  int16_t gxRaw, gyRaw, gzRaw;
  float ax, ay, az; 
  float gx, gy, gz; 
  float pitch, roll;
  float accelMagnitude;
  float gyroMagnitude;
};

DataIMU lastDataIMU;

// --- HALAMAN PENGATURAN (hanya sensitivitas & kalibrasi) ---
const char HTML_PENGATURAN[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="id">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>HEBAT Smart Pin — Pengaturan</title>
  <style>
    body { font-family: system-ui, sans-serif; background: #f1f5f9; color: #0f172a; margin: 0; padding: 24px 16px; }
    .card { max-width: 420px; margin: 0 auto; background: #fff; border: 1px solid #e2e8f0; border-radius: 20px; padding: 24px; display: flex; flex-direction: column; gap: 16px; }
    h1 { font-size: 1.15rem; margin: 0; }
    label { display: flex; flex-direction: column; gap: 4px; font-size: 0.8rem; font-weight: 600; color: #64748b; }
    input { padding: 10px 12px; border: 1px solid #cbd5e1; border-radius: 10px; font-size: 1rem; background: #f8fafc; }
    button { padding: 12px; border: none; border-radius: 12px; background: #2563eb; color: #fff; font-weight: 600; font-size: 0.95rem; cursor: pointer; }
    button.sekunder { background: #f1f5f9; color: #475569; border: 1px solid #e2e8f0; }
    #pesan { font-size: 0.85rem; color: #475569; min-height: 1.2em; text-align: center; }
  </style>
</head>
<body>
  <div class="card">
    <h1>HEBAT Smart Pin — Pengaturan</h1>
    <label>Batas sudut postur (°)<input type="number" id="sudut" step="1" min="1"></label>
    <label>Batas gyro (DPS)<input type="number" id="gyro" step="1" min="1"></label>
    <label>Batas accel (G)<input type="number" id="accel" step="0.05" min="0.05"></label>
    <button onclick="simpan()">Simpan Sensitivitas</button>
    <button class="sekunder" onclick="kalibrasi()">🎯 Set Posisi Ideal</button>
    <div id="pesan"></div>
  </div>
  <script>
    const el = id => document.getElementById(id);
    const pesan = t => el('pesan').innerText = t;

    fetch('/settings')
      .then(r => r.json())
      .then(d => { el('sudut').value = d.sudut; el('gyro').value = d.gyro; el('accel').value = d.accel; })
      .catch(() => pesan('Gagal memuat pengaturan.'));

    function simpan() {
      fetch(`/update-settings?sudut=${el('sudut').value}&gyro=${el('gyro').value}&accel=${el('accel').value}`)
        .then(r => pesan(r.ok ? '✅ Sensitivitas tersimpan.' : 'Gagal: nilai harus angka antara 0 dan 1000.'))
        .catch(() => pesan('Koneksi terputus.'));
    }

    function kalibrasi() {
      fetch('/calibrate')
        .then(r => pesan(r.ok ? '✅ Posisi ideal diperbarui.' : 'Gagal kalibrasi.'))
        .catch(() => pesan('Koneksi terputus.'));
    }
  </script>
</body>
</html>
)rawliteral";

// --- Fungsi I2C Helper ---
bool tulisRegister(uint8_t reg, uint8_t nilai) {
  Wire.beginTransmission(IMU_ADDR);
  Wire.write(reg);
  Wire.write(nilai);
  return Wire.endTransmission() == 0;
}

bool bacaRegister(uint8_t reg, uint8_t *buffer, uint8_t panjang) {
  Wire.beginTransmission(IMU_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  uint8_t jumlah = Wire.requestFrom(IMU_ADDR, panjang);
  if (jumlah != panjang) return false;
  for (uint8_t i = 0; i < panjang; i++) buffer[i] = Wire.read();
  return true;
}

int16_t gabungInt16(uint8_t msb, uint8_t lsb) {
  return (int16_t)((uint16_t)msb << 8 | lsb);
}

bool bacaIMU(DataIMU &d) {
  uint8_t b[14];
  if (!bacaRegister(0x3B, b, 14)) return false;

  d.axRaw = gabungInt16(b[0], b[1]);
  d.ayRaw = gabungInt16(b[2], b[3]);
  d.azRaw = gabungInt16(b[4], b[5]);
  d.gxRaw = gabungInt16(b[8], b[9]);
  d.gyRaw = gabungInt16(b[10], b[11]);
  d.gzRaw = gabungInt16(b[12], b[13]);

  float rawAx = d.axRaw * (1.0f / 16384.0f);
  float rawAy = d.ayRaw * (1.0f / 16384.0f);
  float rawAz = d.azRaw * (1.0f / 16384.0f);
  float rawGx = d.gxRaw * (1.0f / 131.0f);
  float rawGy = d.gyRaw * (1.0f / 131.0f);
  float rawGz = d.gzRaw * (1.0f / 131.0f);

  if (!filterSiap) {
    prevAx = rawAx; prevAy = rawAy; prevAz = rawAz;
    prevGx = rawGx; prevGy = rawGy; prevGz = rawGz;
  }

  d.ax = (FILTER_ALPHA_RAW * rawAx) + ((1.0f - FILTER_ALPHA_RAW) * prevAx);
  d.ay = (FILTER_ALPHA_RAW * rawAy) + ((1.0f - FILTER_ALPHA_RAW) * prevAy);
  d.az = (FILTER_ALPHA_RAW * rawAz) + ((1.0f - FILTER_ALPHA_RAW) * prevAz);
  d.gx = (FILTER_ALPHA_RAW * rawGx) + ((1.0f - FILTER_ALPHA_RAW) * prevGx);
  d.gy = (FILTER_ALPHA_RAW * rawGy) + ((1.0f - FILTER_ALPHA_RAW) * prevGy);
  d.gz = (FILTER_ALPHA_RAW * rawGz) + ((1.0f - FILTER_ALPHA_RAW) * prevGz);

  prevAx = d.ax; prevAy = d.ay; prevAz = d.az;
  prevGx = d.gx; prevGy = d.gy; prevGz = d.gz;

  d.accelMagnitude = sqrtf(d.ax*d.ax + d.ay*d.ay + d.az*d.az);
  d.gyroMagnitude = sqrtf(d.gx*d.gx + d.gy*d.gy + d.gz*d.gz);

  float rawPitch = atan2f(d.ay, sqrtf(d.ax*d.ax + d.az*d.az)) * (180.0f / 3.14159265f);
  float rawRoll  = atan2f(-d.ax, d.az) * (180.0f / 3.14159265f);

  if (!filterSiap) {
    filteredPitch = rawPitch;
    filteredRoll = rawRoll;
    filterSiap = true;
  } else {
    filteredPitch = (FILTER_ALPHA_ANGLE * rawPitch) + ((1.0f - FILTER_ALPHA_ANGLE) * filteredPitch);
    filteredRoll  = (FILTER_ALPHA_ANGLE * rawRoll)  + ((1.0f - FILTER_ALPHA_ANGLE) * filteredRoll);
  }

  d.pitch = filteredPitch;
  d.roll  = filteredRoll;
  return true;
}

float selisihSudut(float a, float b) {
  float selisih = fmodf(a - b + 540.0f, 360.0f) - 180.0f;
  return fabsf(selisih);
}

void setLED(bool merah, bool kuning, bool hijau) {
  targetLED[0] = merah;
  targetLED[1] = kuning;
  targetLED[2] = hijau;
}

// Dipanggil tiap tick: geser kecerahan menuju target, dikuadratkan agar fade terlihat rata di mata
void updateLED() {
  for (int i = 0; i < 3; i++) {
    int target = targetLED[i] ? 255 : 0;
    if (levelLED[i] < target) levelLED[i] = min(levelLED[i] + LANGKAH_FADE, 255);
    else if (levelLED[i] > target) levelLED[i] = max(levelLED[i] - LANGKAH_FADE, 0);
    analogWrite(PIN_LED[i], levelLED[i] * levelLED[i] / 255);
  }
}

void kalibrasiAwal() {
  float jumlahPitch = 0.0f;
  float jumlahRoll = 0.0f;
  int berhasil = 0;
  // Batasi percobaan agar boot tidak macet saat IMU tidak terpasang (halaman pengaturan tetap bisa dibuka)
  for (int coba = 0; coba < JUMLAH_KALIBRASI * 2 && berhasil < JUMLAH_KALIBRASI; coba++) {
    DataIMU d;
    if (bacaIMU(d)) {
      jumlahPitch += d.pitch;
      jumlahRoll += d.roll;
      berhasil++;
    }
    delay(50);
  }
  if (berhasil == 0) return; // baseline tetap 0, bisa diulang lewat tombol "Set Posisi Ideal"
  baselinePitch = jumlahPitch / berhasil;
  baselineRoll = jumlahRoll / berhasil;
}

void handleRoot() { server.send(200, "text/html", HTML_PENGATURAN); }

void handleGetSettings() {
  String json = "{";
  json += "\"sudut\":" + String(batasSudutPostur) + ",";
  json += "\"gyro\":" + String(batasGyroDps) + ",";
  json += "\"accel\":" + String(batasGerakAccelG);
  json += "}";
  server.send(200, "application/json", json);
}

// Nilai kosong / bukan angka / di luar (0, 1000) ditolak (toFloat() mengembalikan 0 -> LED merah terus)
bool ambilAngkaPositif(const char *nama, float &hasil) {
  if (!server.hasArg(nama)) return true;
  float v = server.arg(nama).toFloat();
  if (!(v > 0.0f && v < 1000.0f)) return false; // sekaligus menolak NaN/inf
  hasil = v;
  return true;
}

void handleUpdateSettings() {
  float sudut = batasSudutPostur, gyro = batasGyroDps, accel = batasGerakAccelG;
  if (!ambilAngkaPositif("sudut", sudut) || !ambilAngkaPositif("gyro", gyro) || !ambilAngkaPositif("accel", accel)) {
    server.send(400, "text/plain", "Nilai harus angka lebih dari 0");
    return;
  }
  batasSudutPostur = sudut;
  batasGyroDps = gyro;
  batasGerakAccelG = accel;
  server.send(200, "text/plain", "OK");
}

void handleCalibrate() { 
  baselinePitch = lastDataIMU.pitch; 
  baselineRoll = lastDataIMU.roll; 
  server.send(200, "text/plain", "OK"); 
}

void setup() {
  updateLED(); // semua LED mati (PWM 0)

  Serial.begin(115200);
  delay(1000);

  WiFi.mode(WIFI_OFF);
  delay(500);
  WiFi.mode(WIFI_AP);
  WiFi.setTxPower(WIFI_POWER_8_5dBm);
  WiFi.softAP(AP_SSID, AP_PASS, 1, false, 4);

  server.on("/", handleRoot);
  server.on("/settings", handleGetSettings);
  server.on("/update-settings", handleUpdateSettings);
  server.on("/calibrate", handleCalibrate);
  server.begin();

  Wire.begin(6, 7);
  Wire.setClock(400000); 

  tulisRegister(0x6B, 0x00); // bangunkan MPU6050
  tulisRegister(0x1A, 0x03); // DLPF hardware ~44 Hz: buang getaran/noise sebelum dibaca (delay ~5 ms)
  delay(100);
  kalibrasiAwal();
}

void loop() {
  server.handleClient();

  unsigned long sekarang = millis();
  if (sekarang - waktuBacaTerakhir < INTERVAL_BACA_MS) return;
  waktuBacaTerakhir = sekarang;
  updateLED();

  DataIMU d;
  if (!bacaIMU(d)) {
    setLED(true, false, false); // sensor error
    return;
  }

  lastDataIMU = d;

  float deviasiPitch = selisihSudut(d.pitch, baselinePitch);
  float deviasiRoll = selisihSudut(d.roll, baselineRoll);
  float deviasiSudut = (deviasiPitch > deviasiRoll) ? deviasiPitch : deviasiRoll;

  bool posturMenyimpang = deviasiSudut > batasSudutPostur;
  bool gerakAktif = (d.gyroMagnitude > batasGyroDps) || (fabsf(d.accelMagnitude - 1.0f) > batasGerakAccelG);

  if (gerakAktif) {
    if (!gerakSedangBerlangsung) {
      mulaiGerak = sekarang;
      gerakSedangBerlangsung = true;
    }
  } else {
    gerakSedangBerlangsung = false;
  }

  bool gerakLebihPeringatan = gerakSedangBerlangsung && (sekarang - mulaiGerak >= DURASI_PERINGATAN_MS);

  if (posturMenyimpang) {
    if (!penyimpanganSedangBerlangsung) {
      mulaiPenyimpangan = sekarang;
      penyimpanganSedangBerlangsung = true;
    }
  } else {
    penyimpanganSedangBerlangsung = false;
  }

  bool posturMenyimpangLebihPeringatan = penyimpanganSedangBerlangsung && (sekarang - mulaiPenyimpangan >= DURASI_PERINGATAN_MS);

  bool stabilSekarang = !gerakAktif;

  if (stabilSekarang && posturMenyimpang) {
    if (!posisiBaruSedangStabil) {
      mulaiStabilPosisiBaru = sekarang;
      posisiBaruSedangStabil = true;
    }

    if (sekarang - mulaiStabilPosisiBaru >= DURASI_BASELINE_MS) {
      baselinePitch = d.pitch;
      baselineRoll = d.roll;
      posisiBaruSedangStabil = false;
      penyimpanganSedangBerlangsung = false;
      deviasiSudut = 0.0f;
      posturMenyimpang = false;
    }
  } else {
    posisiBaruSedangStabil = false;
  }

  bool kondisiAdaPotensiKuning = (gerakAktif || posturMenyimpang);
  
  if (kondisiAdaPotensiKuning) {
    if (!potensiKuningBerlangsung) {
      mulaiPotensiKuning = sekarang;
      potensiKuningBerlangsung = true;
    }
  } else {
    potensiKuningBerlangsung = false;
  }

  bool syaratWaktuKuning = potensiKuningBerlangsung && (sekarang - mulaiPotensiKuning >= DURASI_TUNDA_KUNING_MS);

  // --- LOGIKA PENAHAN SAAT MERAH ---
  bool kondisiMerahMentah = gerakLebihPeringatan || posturMenyimpangLebihPeringatan;

  if (kondisiMerahMentah) {
    if (!sedangMenahanMerah) {
      waktuMulaiMerah = sekarang;
      sedangMenahanMerah = true;
    }
  }

  bool ledMerah = false;
  if (sedangMenahanMerah) {
    if (sekarang - waktuMulaiMerah < DURASI_TAHAN_MERAH_MS) {
      ledMerah = true; 
    } else {
      if (kondisiMerahMentah) {
        waktuMulaiMerah = sekarang; 
        ledMerah = true;
      } else {
        sedangMenahanMerah = false; 
      }
    }
  }

  bool ledKuning = !ledMerah && syaratWaktuKuning;
  bool ledHijau = !ledMerah && !ledKuning;

  setLED(ledMerah, ledKuning, ledHijau);
}