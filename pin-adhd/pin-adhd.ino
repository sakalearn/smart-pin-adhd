#include <Wire.h>
#include <math.h>
#include <WiFi.h>
#include <WebServer.h>

// =====================================================
// ESP32-C3 + MPU6050 + Clean Modern UI/UX Dashboard (Tanpa Motor)
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

// Ambang batas
const float BATAS_SUDUT_POSTUR = 40.0f;  
const float BATAS_GYRO_DPS = 35.0f;      
const float BATAS_GERAK_ACCEL_G = 0.50f; 

const unsigned long DURASI_PERINGATAN_MS = 5000UL; 
const unsigned long DURASI_BASELINE_MS = 3000UL;   
const unsigned long INTERVAL_BACA_MS = 66UL; // ~15 Hz
const int JUMLAH_KALIBRASI = 50; 

const unsigned long DURASI_TUNDA_KUNING_MS = 500UL; 
unsigned long mulaiPotensiKuning = 0;
bool potensiKuningBerlangsung = false;

// --- LOGIKA PENAHAN 3 DETIK SAAT MERAH ---
unsigned long waktuMulaiMerah = 0;
bool sedangMenahanMerah = false;
const unsigned long DURASI_TAHAN_MERAH_MS = 3000UL;

int jumlahMerahTotal = 0;
bool statusMerahSebelumnya = false;

float baselinePitch = 0.0f;
float baselineRoll = 0.0f;

unsigned long waktuBacaTerakhir = 0;
unsigned long mulaiGerak = 0;
unsigned long mulaiPenyimpangan = 0;
unsigned long mulaiStabilPosisiBaru = 0;

bool gerakSedangBerlangsung = false;
bool penyimpanganSedangBerlangsung = false;
bool posisiBaruSedangStabil = false;

String statusLEDGlobal = "HIJAU";
String statusKeteranganGlobal = "Aman";
bool imuErrorGlobal = false;

// Variabel Smoothing Filter (EMA)
float prevAx = 0, prevAy = 0, prevAz = 1.0f;
float prevGx = 0, prevGy = 0, prevGz = 0;
float filteredPitch = 0.0f;
float filteredRoll = 0.0f;

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

// --- HTML / CSS / JS WEB DASHBOARD (Clean UX / UI Modern) ---
const char HTML_DASHBOARD[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="id">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>Smart Pin Dashboard</title>
  <link href="https://fonts.googleapis.com/css2?family=Plus+Jakarta+Sans:wght@400;500;600;700&display=swap" rel="stylesheet">
  <style>
    * { box-sizing: border-box; margin: 0; padding: 0; font-family: 'Plus Jakarta Sans', sans-serif; }
    body { background-color: #f1f5f9; color: #1e293b; padding: 24px; display: flex; justify-content: center; }
    
    .container { width: 100%; max-width: 700px; }
    
    /* Header Modern & Clean */
    .header { background: #ffffff; padding: 20px 24px; border-radius: 20px; box-shadow: 0 4px 20px -2px rgba(0, 0, 0, 0.03); margin-bottom: 20px; border: 1px solid #e2e8f0; display: flex; justify-content: space-between; align-items: center; flex-wrap: wrap; gap: 12px; }
    .header-left h1 { font-size: 1.25rem; font-weight: 700; color: #0f172a; margin-bottom: 2px; }
    .header-left p { font-size: 0.8rem; color: #64748b; }
    
    .btn-calibrate { background: #0f172a; color: white; border: none; padding: 10px 16px; border-radius: 12px; font-weight: 600; font-size: 0.825rem; cursor: pointer; transition: all 0.2s; box-shadow: 0 2px 4px rgba(15,23,42,0.1); }
    .btn-calibrate:hover { background: #334155; transform: translateY(-1px); }

    /* Pop-up Penuh Layar untuk Status Merah */
    #fullscreen-alert {
      position: fixed; top: 0; left: 0; width: 100vw; height: 100vh;
      background: rgba(239, 68, 68, 0.96); z-index: 9998;
      display: flex; flex-direction: column; justify-content: center; align-items: center;
      color: white; opacity: 0; pointer-events: none; transition: opacity 0.3s ease;
      backdrop-filter: blur(8px);
    }
    #fullscreen-alert.active { opacity: 1; pointer-events: auto; }
    #fullscreen-alert h2 { font-size: 2.75rem; font-weight: 700; margin-bottom: 8px; letter-spacing: -0.5px; }
    #fullscreen-alert p { font-size: 1.1rem; font-weight: 500; opacity: 0.95; }

    /* Pop-up Selesai Pembelajaran */
    #finish-modal {
      position: fixed; top: 0; left: 0; width: 100vw; height: 100vh;
      background: rgba(15, 23, 42, 0.7); z-index: 9999;
      display: flex; justify-content: center; align-items: center;
      opacity: 0; pointer-events: none; transition: opacity 0.3s ease;
      backdrop-filter: blur(4px);
    }
    #finish-modal.active { opacity: 1; pointer-events: auto; }
    .modal-card { background: white; padding: 32px; border-radius: 24px; text-align: center; max-width: 400px; width: 90%; box-shadow: 0 20px 25px -5px rgba(0,0,0,0.1); }
    .modal-card h2 { font-size: 1.35rem; font-weight: 700; color: #0f172a; margin-bottom: 8px; }
    .modal-card p { font-size: 0.95rem; color: #475569; margin-bottom: 24px; line-height: 1.5; }
    .modal-card button { padding: 12px 24px; background: #2563eb; color: white; border: none; border-radius: 12px; font-weight: 600; cursor: pointer; width: 100%; transition: background 0.2s; }
    .modal-card button:hover { background: #1d4ed8; }

    /* Kartu Status Utama */
    .card-status { background: #ffffff; padding: 28px; border-radius: 20px; box-shadow: 0 4px 20px -2px rgba(0, 0, 0, 0.03); margin-bottom: 20px; border: 1px solid #e2e8f0; text-align: center; }
    .status-badge { display: inline-block; padding: 6px 16px; border-radius: 9999px; font-weight: 600; font-size: 0.75rem; margin-bottom: 12px; text-transform: uppercase; letter-spacing: 0.05em; }
    .status-HIJAU { background: #dcfce7; color: #166534; }
    .status-KUNING { background: #fef9c3; color: #854d0e; }
    .status-MERAH { background: #fee2e2; color: #991b1b; }
    
    .status-desc { font-size: 1.75rem; font-weight: 700; color: #0f172a; margin-bottom: 6px; }
    .status-sub { font-size: 0.9rem; color: #64748b; }

    /* Grid Kartu Informasi */
    .grid { display: grid; grid-template-columns: 1fr 1fr; gap: 16px; margin-bottom: 20px; }
    .card { background: #ffffff; padding: 20px; border-radius: 20px; box-shadow: 0 4px 20px -2px rgba(0, 0, 0, 0.03); border: 1px solid #e2e8f0; }
    .card h3 { font-size: 0.75rem; font-weight: 600; color: #64748b; text-transform: uppercase; margin-bottom: 8px; letter-spacing: 0.05em; }
    .card .val { font-size: 2rem; font-weight: 700; color: #0f172a; }

    /* Sesi Fokus Belajar (Timer di Tengah) */
    .timer-card { background: #ffffff; padding: 28px; border-radius: 20px; box-shadow: 0 4px 20px -2px rgba(0, 0, 0, 0.03); border: 1px solid #e2e8f0; margin-bottom: 20px; text-align: center; }
    .timer-card h3 { font-size: 0.75rem; font-weight: 600; color: #64748b; text-transform: uppercase; margin-bottom: 12px; letter-spacing: 0.05em; }
    .timer-display { font-size: 3.5rem; font-weight: 700; color: #2563eb; margin-bottom: 16px; letter-spacing: -1.5px; }
    .timer-controls { display: flex; gap: 10px; justify-content: center; align-items: center; flex-wrap: wrap; }
    .timer-controls input { padding: 10px 14px; border: 1px solid #cbd5e1; border-radius: 12px; font-size: 1rem; width: 100px; text-align: center; outline: none; background: #f8fafc; }
    .timer-controls button { padding: 10px 20px; background: #2563eb; color: white; border: none; border-radius: 12px; font-weight: 600; font-size: 0.9rem; cursor: pointer; transition: background 0.2s; }
    .timer-controls button:hover { background: #1d4ed8; }
    .btn-secondary { background: #f1f5f9 !important; color: #475569 !important; }
    .btn-secondary:hover { background: #e2e8f0 !important; }

    /* Sektor Status Perangkat (Hardware Status) */
    .device-status-card { background: #ffffff; padding: 20px 24px; border-radius: 20px; box-shadow: 0 4px 20px -2px rgba(0, 0, 0, 0.03); border: 1px solid #e2e8f0; margin-bottom: 20px; }
    .device-status-card h3 { font-size: 0.75rem; font-weight: 600; color: #64748b; text-transform: uppercase; margin-bottom: 14px; letter-spacing: 0.05em; }
    .status-list { display: flex; flex-direction: column; gap: 10px; }
    .status-item { display: flex; justify-content: space-between; align-items: center; padding: 10px 14px; background: #f8fafc; border-radius: 12px; border: 1px solid #f1f5f9; font-size: 0.875rem; font-weight: 500; }
    
    .badge-status { padding: 4px 10px; border-radius: 9999px; font-size: 0.75rem; font-weight: 600; text-transform: uppercase; }
    .bg-online { background: #dcfce7; color: #166534; }
    .bg-offline { background: #fee2e2; color: #991b1b; }
    .bg-active { background: #dbeafe; color: #1e40af; }

    .footer { text-align: center; font-size: 0.75rem; color: #94a3b8; margin-top: 24px; }
  </style>
</head>
<body>

  <!-- POP-UP LAYAR PENUH SAAT MERAH -->
  <div id="fullscreen-alert">
    <h2>⚠️ PERINGATAN!</h2>
    <p>Banyak Gerakan Terdeteksi! Harap Tenang.</p>
  </div>

  <!-- POP-UP SELESAI PEMBELAJARAN -->
  <div id="finish-modal">
    <div class="modal-card">
      <h2>🎉 Selesai!</h2>
      <p id="finish-message">Selamat, Anda Telah Menyelesaikan Pembelajaran</p>
      <button onclick="tutupModalSelesai()">Kembali Belajar</button>
    </div>
  </div>

  <div class="container">
    <!-- HEADER -->
    <div class="header">
      <div class="header-left">
        <h1>Smart Pin</h1>
        <p>Monitor Postur & Fokus Anak ADHD</p>
      </div>
      <button class="btn-calibrate" onclick="kirimKalibrasi()">Set Posisi Ideal</button>
    </div>

    <!-- KARTU STATUS UTAMA -->
    <div class="card-status">
      <div id="badge" class="status-badge status-HIJAU">Status Sistem</div>
      <div id="status-desc" class="status-desc">Aman</div>
      <div id="status-sub" class="status-sub">Postur dan gerakan anak stabil</div>
    </div>

    <!-- GRID INFO -->
    <div class="grid">
      <div class="card">
        <h3>Total Pelanggaran</h3>
        <div class="val" id="count-merah">0</div>
      </div>
      <div class="card">
        <h3>Kondisi Postur</h3>
        <div class="val" id="posture-state" style="font-size: 1.15rem; margin-top: 14px; font-weight: 600;">Normal</div>
      </div>
    </div>

    <!-- TIMER FOKUS BELAJAR -->
    <div class="timer-card">
      <h3>Waktu Fokus Belajar</h3>
      <div class="timer-display" id="timer-display">25:00</div>
      <div class="timer-controls">
        <input type="number" id="input-menit" value="25" min="1" max="120">
        <span style="font-size: 0.85rem; color: #64748b; font-weight: 500;">Menit</span>
        <button onclick="mulaiTimer()">Mulai</button>
        <button class="btn-secondary" onclick="resetTimer()">Reset</button>
      </div>
    </div>

    <!-- STATUS HARDWARE (DIBAWAH) -->
    <div class="device-status-card">
      <h3>Status Perangkat & Komponen</h3>
      <div class="status-list">
        <div class="status-item">
          <span>Status Wifi</span>
          <span id="st-wifi" class="badge-status bg-online">Terhubung</span>
        </div>
        <div class="status-item">
          <span>Status IMU</span>
          <span id="st-imu" class="badge-status bg-online">Normal</span>
        </div>
        <div class="status-item">
          <span>Status LED 1 (Merah)</span>
          <span id="st-led1" class="badge-status bg-active">OFF</span>
        </div>
        <div class="status-item">
          <span>Status LED 2 (Kuning)</span>
          <span id="st-led2" class="badge-status bg-active">OFF</span>
        </div>
        <div class="status-item">
          <span>Status LED 3 (Hijau)</span>
          <span id="st-led3" class="badge-status bg-active">OFF</span>
        </div>
      </div>
    </div>

    <div class="footer">
      HEBAT SMART PIN
    </div>
  </div>

  <script>
    let durasiAwalMenit = 25;
    let sisaWaktuDetik = durasiAwalMenit * 60;
    let timerInterval = null;
    let timerBerjalan = false;
    let statusMerahSebelumnyaWeb = false;

    function playAlarmSound() {
      try {
        const ctx = new (window.AudioContext || window.webkitAudioContext)();
        const osc = ctx.createOscillator();
        const gain = ctx.createGain();
        osc.type = 'sawtooth';
        osc.frequency.setValueAtTime(440, ctx.currentTime);
        osc.frequency.exponentialRampToValueAtTime(880, ctx.currentTime + 0.2);
        gain.gain.setValueAtTime(0.3, ctx.currentTime);
        gain.gain.exponentialRampToValueAtTime(0.01, ctx.currentTime + 0.4);
        osc.connect(gain);
        gain.connect(ctx.destination);
        osc.start();
        osc.stop(ctx.currentTime + 0.4);
      } catch (e) {}
    }

    function playFinishSound() {
      try {
        const ctx = new (window.AudioContext || window.webkitAudioContext)();
        const now = ctx.currentTime;
        [523.25, 659.25, 783.99, 1046.50].forEach((freq, i) => {
          const osc = ctx.createOscillator();
          const gain = ctx.createGain();
          osc.frequency.value = freq;
          gain.gain.setValueAtTime(0.2, now + i * 0.15);
          gain.gain.exponentialRampToValueAtTime(0.01, now + i * 0.15 + 0.2);
          osc.connect(gain);
          gain.connect(ctx.destination);
          osc.start(now + i * 0.15);
          osc.stop(now + i * 0.15 + 0.2);
        });
      } catch (e) {}
    }

    function formatWaktu(detik) {
      const m = Math.floor(detik / 60);
      const s = detik % 60;
      return `${m.toString().padStart(2, '0')}:${s.toString().padStart(2, '0')}`;
    }

    function mulaiTimer() {
      if (timerBerjalan) return;
      const inputVal = parseInt(document.getElementById('input-menit').value);
      durasiAwalMenit = inputVal;
      sisaWaktuDetik = inputVal * 60;
      
      fetch('/reset-timer');

      timerBerjalan = true;
      timerInterval = setInterval(() => {
        if (sisaWaktuDetik > 0) {
          sisaWaktuDetik--;
          document.getElementById('timer-display').innerText = formatWaktu(sisaWaktuDetik);
        } else {
          clearInterval(timerInterval);
          timerBerjalan = false;
          playFinishSound();
          tampilkanModalSelesai();
        }
      }, 1000);
    }

    function resetTimer() {
      clearInterval(timerInterval);
      timerBerjalan = false;
      const inputVal = parseInt(document.getElementById('input-menit').value);
      sisaWaktuDetik = inputVal * 60;
      document.getElementById('timer-display').innerText = formatWaktu(sisaWaktuDetik);
      fetch('/reset-timer');
    }

    function kirimKalibrasi() {
      fetch('/calibrate')
        .then(res => {
          if(res.ok) alert("Posisi ideal berhasil disimpan!");
        })
        .catch(err => alert("Gagal mengatur posisi ideal"));
    }

    function tampilkanModalSelesai() {
      fetch('/data')
        .then(res => res.json())
        .then(d => {
          const msg = `Selamat, Anda Telah Menyelesaikan Pembelajaran<br><br><b>${d.totalMerah} Pelanggaran</b>`;
          document.getElementById('finish-message').innerHTML = msg;
          document.getElementById('finish-modal').classList.add('active');
        });
    }

    function tutupModalSelesai() {
      document.getElementById('finish-modal').classList.remove('active');
      resetTimer();
    }

    // Fetch data real-time dari ESP32 setiap 400ms
    setInterval(() => {
      fetch('/data')
        .then(res => res.json())
        .then(d => {
          // Status Wifi Online
          const elWifi = document.getElementById('st-wifi');
          elWifi.className = "badge-status bg-online";
          elWifi.innerText = "Terhubung";

          // Status IMU
          const elImu = document.getElementById('st-imu');
          if (d.imuErr) {
            elImu.className = "badge-status bg-offline";
            elImu.innerText = "Error / Disconnected";
          } else {
            elImu.className = "badge-status bg-online";
            elImu.innerText = "Normal";
          }

          // Status LED 1 (Merah), LED 2 (Kuning), LED 3 (Hijau)
          const elLed1 = document.getElementById('st-led1');
          const elLed2 = document.getElementById('st-led2');
          const elLed3 = document.getElementById('st-led3');

          if (d.ledStatus === "MERAH") {
            elLed1.className = "badge-status bg-offline"; elLed1.innerText = "ON";
            elLed2.className = "badge-status bg-active";  elLed2.innerText = "OFF";
            elLed3.className = "badge-status bg-active";  elLed3.innerText = "OFF";
          } else if (d.ledStatus === "KUNING") {
            elLed1.className = "badge-status bg-active";  elLed1.innerText = "OFF";
            elLed2.className = "badge-status bg-offline"; elLed2.innerText = "ON";
            elLed3.className = "badge-status bg-active";  elLed3.innerText = "OFF";
          } else {
            elLed1.className = "badge-status bg-active";  elLed1.innerText = "OFF";
            elLed2.className = "badge-status bg-active";  elLed2.innerText = "OFF";
            elLed3.className = "badge-status bg-online";  elLed3.innerText = "ON";
          }

          const badge = document.getElementById('badge');
          const desc = document.getElementById('status-desc');
          const sub = document.getElementById('status-sub');
          const alertScreen = document.getElementById('fullscreen-alert');
          const countMerah = document.getElementById('count-merah');
          const postureState = document.getElementById('posture-state');

          badge.className = "status-badge status-" + d.ledStatus;
          badge.innerText = "LED: " + d.ledStatus;
          desc.innerText = d.keterangan;
          countMerah.innerText = d.totalMerah;

          if (d.ledStatus === "HIJAU") {
            sub.innerText = "Postur dan gerakan anak stabil.";
            postureState.innerText = "Normal";
            alertScreen.classList.remove('active');
            statusMerahSebelumnyaWeb = false;
          } else if (d.ledStatus === "KUNING") {
            sub.innerText = "Anak sedang mengubah posisi duduk/berdiri.";
            postureState.innerText = "Berubah Posisi";
            alertScreen.classList.remove('active');
            statusMerahSebelumnyaWeb = false;
          } else if (d.ledStatus === "MERAH") {
            sub.innerText = "Peringatan: Aktivitas fisik berlebihan terdeteksi!";
            postureState.innerText = "Banyak Gerakan";
            alertScreen.classList.add('active');
            
            if (!statusMerahSebelumnyaWeb) {
              playAlarmSound();
              statusMerahSebelumnyaWeb = true;
            }
          }
        })
        .catch(err => {
          // Jika koneksi ESP32 terputus
          document.getElementById('st-wifi').className = "badge-status bg-offline";
          document.getElementById('st-wifi').innerText = "Terputus";
        });
    }, 400);
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

  for (uint8_t i = 0; i < panjang; i++) {
    buffer[i] = Wire.read();
  }
  return true;
}

int16_t gabungInt16(uint8_t msb, uint8_t lsb) {
  return (int16_t)((uint16_t)msb << 8 | lsb);
}

bool bacaIMU(DataIMU &d) {
  uint8_t b[14];
  if (!bacaRegister(0x3B, b, 14)) {
    imuErrorGlobal = true;
    return false;
  }
  imuErrorGlobal = false;

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

  if (filteredPitch == 0.0f && filteredRoll == 0.0f) {
    filteredPitch = rawPitch;
    filteredRoll = rawRoll;
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
  digitalWrite(PIN_LED_MERAH, merah ? HIGH : LOW);
  digitalWrite(PIN_LED_KUNING, kuning ? HIGH : LOW);
  digitalWrite(PIN_LED_HIJAU, hijau ? HIGH : LOW);
}

void kalibrasiAwal() {
  float jumlahPitch = 0.0f;
  float jumlahRoll = 0.0f;
  int berhasil = 0;

  while (berhasil < JUMLAH_KALIBRASI) {
    DataIMU d;
    if (bacaIMU(d)) {
      jumlahPitch += d.pitch;
      jumlahRoll += d.roll;
      berhasil++;
    }
    delay(50);
  }

  baselinePitch = jumlahPitch / berhasil;
  baselineRoll = jumlahRoll / berhasil;
}

void handleRoot() {
  server.send(200, "text/html", HTML_DASHBOARD);
}

void handleDataJSON() {
  String json = "{";
  json += "\"ledStatus\":\"" + statusLEDGlobal + "\",";
  json += "\"keterangan\":\"" + statusKeteranganGlobal + "\",";
  json += "\"totalMerah\":" + String(jumlahMerahTotal) + ",";
  json += "\"imuErr\":" + String(imuErrorGlobal ? "true" : "false");
  json += "}";
  server.send(200, "application/json", json);
}

void handleResetTimer() {
  jumlahMerahTotal = 0; 
  server.send(200, "text/plain", "OK");
}

void handleCalibrate() {
  baselinePitch = lastDataIMU.pitch;
  baselineRoll = lastDataIMU.roll;
  server.send(200, "text/plain", "OK");
}

void setup() {
  pinMode(PIN_LED_MERAH, OUTPUT);
  pinMode(PIN_LED_KUNING, OUTPUT);
  pinMode(PIN_LED_HIJAU, OUTPUT);
  setLED(false, false, false);

  Serial.begin(115200);
  delay(1000);

  WiFi.mode(WIFI_OFF);
  delay(500);
  WiFi.mode(WIFI_AP);
  WiFi.setTxPower(WIFI_POWER_8_5dBm);
  WiFi.softAP(AP_SSID, AP_PASS, 1, false, 4);

  server.on("/", handleRoot);
  server.on("/data", handleDataJSON);
  server.on("/reset-timer", handleResetTimer);
  server.on("/calibrate", handleCalibrate);
  server.begin();

  Wire.begin(6, 7);
  Wire.setClock(400000); 

  tulisRegister(0x6B, 0x00);
  delay(100);
  kalibrasiAwal();
}

void loop() {
  server.handleClient();

  unsigned long sekarang = millis();
  if (sekarang - waktuBacaTerakhir < INTERVAL_BACA_MS) return;
  waktuBacaTerakhir = sekarang;

  DataIMU d;
  if (!bacaIMU(d)) {
    setLED(true, false, false);
    statusLEDGlobal = "MERAH";
    statusKeteranganGlobal = "Banyak Gerakan";
    return;
  }

  lastDataIMU = d;

  float deviasiPitch = selisihSudut(d.pitch, baselinePitch);
  float deviasiRoll = selisihSudut(d.roll, baselineRoll);
  float deviasiSudut = (deviasiPitch > deviasiRoll) ? deviasiPitch : deviasiRoll;

  bool posturMenyimpang = deviasiSudut > BATAS_SUDUT_POSTUR;
  bool gerakAktif = (d.gyroMagnitude > BATAS_GYRO_DPS) || (fabsf(d.accelMagnitude - 1.0f) > BATAS_GERAK_ACCEL_G);

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

  // --- LOGIKA PENAHAN 3 DETIK SAAT MERAH ---
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

  if (ledMerah && !statusMerahSebelumnya) {
    jumlahMerahTotal++;
  }
  statusMerahSebelumnya = ledMerah;

  if (ledMerah) {
    statusLEDGlobal = "MERAH";
    statusKeteranganGlobal = "Banyak Gerakan";
  } else if (ledKuning) {
    statusLEDGlobal = "KUNING";
    statusKeteranganGlobal = "Berubah Posisi";
  } else {
    statusLEDGlobal = "HIJAU";
    statusKeteranganGlobal = "Aman";
  }
}