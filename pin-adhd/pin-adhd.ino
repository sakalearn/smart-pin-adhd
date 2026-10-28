#include <Wire.h>
#include <math.h>
#include <WiFi.h>
#include <WebServer.h>

// =====================================================
// ESP32-C3 + MPU6050 + Fixed Compact UI & Full-Width Dashboard
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

// --- AMBANG BATAS DINAMIS (Bisa diubah via Web) ---
float batasSudutPostur = 30.0f;  
float batasGyroDps = 15.0f;      
float batasGerakAccelG = 0.30f; 

const unsigned long DURASI_PERINGATAN_MS = 2000UL; 
const unsigned long DURASI_BASELINE_MS = 5000UL;   
const unsigned long INTERVAL_BACA_MS = 66UL; // ~15 Hz
const int JUMLAH_KALIBRASI = 50; 

const unsigned long DURASI_TUNDA_KUNING_MS = 500UL; 
unsigned long mulaiPotensiKuning = 0;
bool potensiKuningBerlangsung = false;

// --- LOGIKA PENAHAN SAAT MERAH ---
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

const float FILTER_ALPHA_RAW = 0.3f;  
const float FILTER_ALPHA_ANGLE = 0.35f; 

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

// --- HTML / CSS / JS WEB DASHBOARD (Fixed Layout) ---
const char HTML_DASHBOARD[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="id">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>Smart Pin Dashboard — HEBAT</title>
  <link href="https://fonts.googleapis.com/css2?family=Plus+Jakarta+Sans:wght@400;500;600;700&display=swap" rel="stylesheet">
  <style>
    * { box-sizing: border-box; margin: 0; padding: 0; font-family: 'Plus Jakarta Sans', sans-serif; }
    body { background-color: #f1f5f9; color: #1e293b; min-height: 100vh; display: flex; flex-direction: column; }
    
    .navbar { background: #ffffff; border-bottom: 1px solid #e2e8f0; padding: 16px 32px; display: flex; justify-content: space-between; align-items: center; position: sticky; top: 0; z-index: 100; box-shadow: 0 1px 3px rgba(0,0,0,0.02); }
    .nav-brand { display: flex; align-items: center; gap: 12px; }
    .brand-icon { width: 38px; height: 38px; background: #2563eb; color: white; border-radius: 12px; display: flex; justify-content: center; align-items: center; font-weight: 700; font-size: 1.1rem; box-shadow: 0 2px 6px rgba(37,99,235,0.3); }
    .brand-text h1 { font-size: 1.1rem; font-weight: 700; color: #0f172a; line-height: 1.2; }
    .brand-text p { font-size: 0.75rem; color: #64748b; }
    .nav-actions { display: flex; gap: 12px; align-items: center; }
    
    .btn-primary { background: #2563eb; color: white; border: none; padding: 8px 16px; border-radius: 10px; font-weight: 600; font-size: 0.825rem; cursor: pointer; transition: all 0.2s; box-shadow: 0 2px 4px rgba(37,99,235,0.2); display: flex; align-items: center; gap: 6px; }
    .btn-primary:hover { background: #1d4ed8; transform: translateY(-1px); }

    .main-container { width: 100%; max-width: 1440px; margin: 0 auto; padding: 28px 32px; flex: 1; display: flex; flex-direction: column; gap: 24px; }

    .welcome-banner { background: #ffffff; padding: 28px 32px; border-radius: 24px; border: 1px solid #e2e8f0; box-shadow: 0 4px 20px -2px rgba(0, 0, 0, 0.02); display: flex; justify-content: space-between; align-items: center; flex-wrap: wrap; gap: 16px; }
    .welcome-left .date-tag { font-size: 0.8rem; color: #64748b; font-weight: 500; margin-bottom: 6px; display: block; }
    .welcome-left h2 { font-size: 1.65rem; font-weight: 700; color: #0f172a; margin-bottom: 4px; }
    .welcome-left p { font-size: 0.925rem; color: #475569; }

    .dashboard-grid { display: grid; grid-template-columns: repeat(12, 1fr); gap: 24px; }
    
    .col-span-4 { grid-column: span 4; }
    .col-span-8 { grid-column: span 8; }
    .col-span-6 { grid-column: span 6; }
    .col-span-12 { grid-column: span 12; }

    @media (max-width: 1024px) {
      .col-span-4, .col-span-8, .col-span-6 { grid-column: span 12; }
      .navbar { padding: 16px 20px; }
      .main-container { padding: 20px; }
    }

    /* Fixed Card Style (No forced vertical stretching) */
    .card { background: #ffffff; padding: 24px; border-radius: 24px; border: 1px solid #e2e8f0; box-shadow: 0 4px 20px -2px rgba(0, 0, 0, 0.02); display: flex; flex-direction: column; gap: 16px; }
    .card-header-flex { display: flex; justify-content: space-between; align-items: center; }
    .card h3 { font-size: 0.8rem; font-weight: 600; color: #64748b; text-transform: uppercase; letter-spacing: 0.05em; }

    .status-badge { display: inline-block; padding: 6px 14px; border-radius: 9999px; font-weight: 600; font-size: 0.75rem; text-transform: uppercase; letter-spacing: 0.05em; }
    .status-HIJAU { background: #dcfce7; color: #166534; }
    .status-KUNING { background: #fef9c3; color: #854d0e; }
    .status-MERAH { background: #fee2e2; color: #991b1b; }
    
    .status-content { margin-top: 4px; }
    .status-desc { font-size: 2rem; font-weight: 700; color: #0f172a; margin-bottom: 4px; }
    .status-sub { font-size: 0.9rem; color: #64748b; }

    .metric-val { font-size: 2.5rem; font-weight: 700; color: #0f172a; margin-top: 8px; }

    /* Compact Sensitivity Grid Layout */
    .sensitivity-grid { display: grid; grid-template-columns: repeat(3, 1fr); gap: 12px; }
    @media(max-width: 768px) { .sensitivity-grid { grid-template-columns: 1fr; } }
    
    .form-group { display: flex; flex-direction: column; gap: 4px; }
    .form-group label { font-size: 0.75rem; font-weight: 600; color: #64748b; }
    .form-group input { width: 100%; padding: 8px 12px; border: 1px solid #cbd5e1; border-radius: 10px; font-size: 0.9rem; outline: none; background: #f8fafc; font-weight: 600; color: #0f172a; }
    .form-group input:focus { border-color: #2563eb; background: #fff; }

    /* Timer Card */
    .timer-display { font-size: 3.75rem; font-weight: 700; color: #2563eb; letter-spacing: -1.5px; margin: 8px 0 16px 0; text-align: center; }
    .timer-controls { display: flex; gap: 10px; justify-content: center; align-items: center; flex-wrap: wrap; }
    .timer-controls input { padding: 8px 12px; border: 1px solid #cbd5e1; border-radius: 10px; font-size: 0.95rem; width: 80px; text-align: center; outline: none; background: #f8fafc; font-weight: 600; }
    .timer-controls button { padding: 8px 16px; background: #2563eb; color: white; border: none; border-radius: 10px; font-weight: 600; font-size: 0.85rem; cursor: pointer; transition: background 0.2s; }
    .timer-controls button:hover { background: #1d4ed8; }
    .btn-secondary { background: #f1f5f9 !important; color: #475569 !important; border: 1px solid #e2e8f0; }
    .btn-secondary:hover { background: #e2e8f0 !important; }

    /* Device List */
    .status-list { display: flex; flex-direction: column; gap: 10px; }
    .status-item { display: flex; justify-content: space-between; align-items: center; padding: 12px 16px; background: #f8fafc; border-radius: 14px; border: 1px solid #f1f5f9; font-size: 0.875rem; font-weight: 500; }
    .badge-status { padding: 4px 12px; border-radius: 9999px; font-size: 0.75rem; font-weight: 600; text-transform: uppercase; }
    .bg-online { background: #dcfce7; color: #166534; }
    .bg-offline { background: #fee2e2; color: #991b1b; }
    .bg-active { background: #dbeafe; color: #1e40af; }

    #fullscreen-alert {
      position: fixed; top: 0; left: 0; width: 100vw; height: 100vh;
      background: rgba(239, 68, 68, 0.96); z-index: 9998;
      display: flex; flex-direction: column; justify-content: center; align-items: center;
      color: white; opacity: 0; pointer-events: none; transition: opacity 0.3s ease;
      backdrop-filter: blur(8px); text-align: center; padding: 20px;
    }
    #fullscreen-alert.active { opacity: 1; pointer-events: auto; }
    #fullscreen-alert h2 { font-size: 3rem; font-weight: 700; margin-bottom: 12px; }
    #fullscreen-alert p { font-size: 1.25rem; font-weight: 500; opacity: 0.95; }

    #finish-modal {
      position: fixed; top: 0; left: 0; width: 100vw; height: 100vh;
      background: rgba(15, 23, 42, 0.7); z-index: 9999;
      display: flex; justify-content: center; align-items: center;
      opacity: 0; pointer-events: none; transition: opacity 0.3s ease;
      backdrop-filter: blur(4px); padding: 20px;
    }
    #finish-modal.active { opacity: 1; pointer-events: auto; }
    .modal-card { background: white; padding: 36px; border-radius: 28px; text-align: center; max-width: 420px; width: 100%; box-shadow: 0 20px 25px -5px rgba(0,0,0,0.1); }
    .modal-card h2 { font-size: 1.5rem; font-weight: 700; color: #0f172a; margin-bottom: 10px; }
    .modal-card p { font-size: 0.95rem; color: #475569; margin-bottom: 24px; line-height: 1.5; }
    .modal-card button { padding: 12px 24px; background: #2563eb; color: white; border: none; border-radius: 12px; font-weight: 600; cursor: pointer; width: 100%; transition: background 0.2s; }
    .modal-card button:hover { background: #1d4ed8; }

    .footer { text-align: center; font-size: 0.75rem; color: #94a3b8; margin-top: auto; padding-top: 16px; }
  </style>
</head>
<body>

  <div id="fullscreen-alert">
    <h2>⚠️ PERINGATAN!</h2>
    <p>Banyak Gerakan / Postur Tidak Stabil Terdeteksi!</p>
  </div>

  <div id="finish-modal">
    <div class="modal-card">
      <h2>🎉 Sesi Belajar Selesai!</h2>
      <p id="finish-message">Selamat, Anda Telah Menyelesaikan Sesi Belajar</p>
      <button onclick="tutupModalSelesai()">Mulai Sesi Baru</button>
    </div>
  </div>

  <header class="navbar">
    <div class="nav-brand">
      <div class="brand-icon">⚡</div>
      <div class="brand-text">
        <h1>HEBAT Smart Pin</h1>
        <p>Real-time Posture & Focus Telemetry</p>
      </div>
    </div>
    <div class="nav-actions">
      <button class="btn-primary" onclick="kirimKalibrasi()">
        <span>🎯 Set Posisi Ideal</span>
      </button>
    </div>
  </header>

  <main class="main-container">
    <section class="welcome-banner">
      <div class="welcome-left">
        <span class="date-tag" id="current-date">Selasa, 27 Oktober 2026</span>
        <h2>Dashboard Pemantauan Aktif</h2>
        <p>Sistem aktif mengawasi kestabilan postur dan tingkat fokus secara real-time.</p>
      </div>
    </section>

    <div class="dashboard-grid">
      <div class="card col-span-8">
        <div class="card-header-flex">
          <h3>Status Sistem Operasional</h3>
          <div id="badge" class="status-badge status-HIJAU">LED: HIJAU</div>
        </div>
        <div class="status-content">
          <div id="status-desc" class="status-desc">Aman</div>
          <div id="status-sub" class="status-sub">Postur dan gerakan anak stabil.</div>
        </div>
      </div>

      <div class="card col-span-4">
        <h3>Total Pelanggaran Sesi Ini</h3>
        <div class="metric-val" id="count-merah">0</div>
        <div style="font-size: 0.85rem; color: #64748b; margin-top: 4px;">Akumulasi peringatan aktif</div>
      </div>

      <!-- Pengaturan Sensitivitas Sensor (Ramping, Kompak, Tanpa Ruang Kosong) -->
      <div class="card col-span-6">
        <div class="card-header-flex">
          <h3>Pengaturan Sensitivitas Sensor</h3>
          <button class="btn-primary" onclick="simpanPengaturan()">Simpan</button>
        </div>
        <div class="sensitivity-grid">
          <div class="form-group">
            <label>Sudut (°)</label>
            <input type="number" id="in-sudut" step="1" value="30">
          </div>
          <div class="form-group">
            <label>Gyro (DPS)</label>
            <input type="number" id="in-gyro" step="1" value="15">
          </div>
          <div class="form-group">
            <label>Accel (G)</label>
            <input type="number" id="in-accel" step="0.05" value="0.30">
          </div>
        </div>
      </div>

      <div class="card col-span-6">
        <h3>Waktu Fokus Belajar (Pomodoro)</h3>
        <div class="timer-display" id="timer-display">25:00</div>
        <div class="timer-controls">
          <input type="number" id="input-menit" value="25" min="1" max="120">
          <span style="font-size: 0.85rem; color: #64748b; font-weight: 500;">Menit</span>
          <button onclick="mulaiTimer()">Mulai</button>
          <button class="btn-secondary" onclick="resetTimer()">Reset</button>
        </div>
      </div>

      <div class="card col-span-12">
        <h3>Status Perangkat & Sensor</h3>
        <div class="status-list">
          <div class="status-item">
            <span>Koneksi Wi-Fi AP</span>
            <span id="st-wifi" class="badge-status bg-online">Terhubung</span>
          </div>
          <div class="status-item">
            <span>Sensor IMU (MPU6050)</span>
            <span id="st-imu" class="badge-status bg-online">Normal</span>
          </div>
          <div class="status-item">
            <span>LED Indikator Merah</span>
            <span id="st-led1" class="badge-status bg-active">OFF</span>
          </div>
          <div class="status-item">
            <span>LED Indikator Kuning</span>
            <span id="st-led2" class="badge-status bg-active">OFF</span>
          </div>
          <div class="status-item">
            <span>LED Indikator Hijau</span>
            <span id="st-led3" class="badge-status bg-active">OFF</span>
          </div>
        </div>
      </div>
    </div>

    <footer class="footer">
      HEBAT SMART PIN — Embedded IoT Systems
    </footer>
  </main>

  <script>
    const options = { weekday: 'long', year: 'numeric', month: 'long', day: 'numeric' };
    document.getElementById('current-date').innerText = new Date().toLocaleDateString('id-ID', options);

    let durasiAwalMenit = 25;
    let sisaWaktuDetik = durasiAwalMenit * 60;
    let timerInterval = null;
    let timerBerjalan = false;
    let statusMerahSebelumnyaWeb = false;

    window.onload = function() {
      fetch('/settings')
        .then(res => res.json())
        .then(d => {
          document.getElementById('in-sudut').value = d.sudut;
          document.getElementById('in-gyro').value = d.gyro;
          document.getElementById('in-accel').value = d.accel;
        }).catch(e => {});
    };

    function simpanPengaturan() {
      const sudut = document.getElementById('in-sudut').value;
      const gyro = document.getElementById('in-gyro').value;
      const accel = document.getElementById('in-accel').value;

      fetch(`/update-settings?sudut=${sudut}&gyro=${gyro}&accel=${accel}`)
        .then(res => {
          if (res.ok) alert("✅ Sensitivitas berhasil diperbarui!");
          else alert("Gagal menyimpan pengaturan.");
        })
        .catch(err => alert("Terjadi kesalahan koneksi."));
    }

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
          if(res.ok) alert("✅ Posisi ideal berhasil dikalibrasi ulang dari perangkat!"); 
        })
        .catch(err => alert("Gagal melakukan kalibrasi. Periksa koneksi."));
    }

    function tampilkanModalSelesai() {
      fetch('/data')
        .then(res => res.json())
        .then(d => {
          const msg = `Sesi belajar telah selesai.<br><br>Total Pelanggaran: <b>${d.totalMerah} kali</b>`;
          document.getElementById('finish-message').innerHTML = msg;
          document.getElementById('finish-modal').classList.add('active');
        });
    }

    function tutupModalSelesai() {
      document.getElementById('finish-modal').classList.remove('active');
      resetTimer();
    }

    setInterval(() => {
      fetch('/data')
        .then(res => res.json())
        .then(d => {
          document.getElementById('st-wifi').className = "badge-status bg-online";
          document.getElementById('st-wifi').innerText = "Terhubung";

          const elImu = document.getElementById('st-imu');
          if (d.imuErr) {
            elImu.className = "badge-status bg-offline"; elImu.innerText = "Error / Disconnected";
          } else {
            elImu.className = "badge-status bg-online"; elImu.innerText = "Normal";
          }

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

          badge.className = "status-badge status-" + d.ledStatus;
          badge.innerText = "LED: " + d.ledStatus;
          desc.innerText = d.keterangan;
          countMerah.innerText = d.totalMerah;

          if (d.ledStatus === "HIJAU") {
            sub.innerText = "Postur dan gerakan anak stabil.";
            alertScreen.classList.remove('active');
            statusMerahSebelumnyaWeb = false;
          } else if (d.ledStatus === "KUNING") {
            sub.innerText = "Anak sedang mengubah posisi duduk/berdiri.";
            alertScreen.classList.remove('active');
            statusMerahSebelumnyaWeb = false;
          } else if (d.ledStatus === "MERAH") {
            sub.innerText = "Peringatan: Aktivitas fisik berlebihan terdeteksi!";
            alertScreen.classList.add('active');
            
            if (!statusMerahSebelumnyaWeb) {
              playAlarmSound();
              statusMerahSebelumnyaWeb = true;
            }
          }
        })
        .catch(err => {
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
  for (uint8_t i = 0; i < panjang; i++) buffer[i] = Wire.read();
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

void handleRoot() { server.send(200, "text/html", HTML_DASHBOARD); }

void handleDataJSON() {
  String json = "{";
  json += "\"ledStatus\":\"" + statusLEDGlobal + "\",";
  json += "\"keterangan\":\"" + statusKeteranganGlobal + "\",";
  json += "\"totalMerah\":" + String(jumlahMerahTotal) + ",";
  json += "\"imuErr\":" + String(imuErrorGlobal ? "true" : "false");
  json += "}";
  server.send(200, "application/json", json);
}

void handleGetSettings() {
  String json = "{";
  json += "\"sudut\":" + String(batasSudutPostur) + ",";
  json += "\"gyro\":" + String(batasGyroDps) + ",";
  json += "\"accel\":" + String(batasGerakAccelG);
  json += "}";
  server.send(200, "application/json", json);
}

void handleUpdateSettings() {
  if (server.hasArg("sudut")) batasSudutPostur = server.arg("sudut").toFloat();
  if (server.hasArg("gyro")) batasGyroDps = server.arg("gyro").toFloat();
  if (server.hasArg("accel")) batasGerakAccelG = server.arg("accel").toFloat();
  server.send(200, "text/plain", "OK");
}

void handleResetTimer() { jumlahMerahTotal = 0; server.send(200, "text/plain", "OK"); }

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
  server.on("/settings", handleGetSettings);
  server.on("/update-settings", handleUpdateSettings);
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

  if (ledMerah && !statusMerahSebelumnya) {
    jumlahMerahTotal++;
  }
  statusMerahSebelumnya = ledMerah;

  if (ledMerah) {
    statusLEDGlobal = "MERAH";
    statusKeteranganGlobal = "Banyak Gerakan / Gelisah";
  } else if (ledKuning) {
    statusLEDGlobal = "KUNING";
    statusKeteranganGlobal = "Berubah Posisi";
  } else {
    statusLEDGlobal = "HIJAU";
    statusKeteranganGlobal = "Aman";
  }
}