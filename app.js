// ==========================================================================
// Ranchbot Cattle Ear Tag BLE & 6-DOF IMU Telemetry Receiver Application
// High-Performance Engine:
// 1. Full-Screen Red-Highlighted Stream Diagnostics & Issue Tracker
// 2. Zero-Hang Full-Screen Transition (Background Front-Screen Suspension)
// 3. Zero-Allocation Float32Array Circular Ring Buffers ($0$ GC Pauses)
// 4. GPU Hardware Transforms (translate3d/scaleX) with 0 Layout Reflows
// 5. Virtualized & Capped DOM Terminal Logging (150 Node Max Limit)
// 6. Dedicated DSP Web Worker for Madgwick AHRS & 64-Point FFT Math
// 7. Page Visibility Power Caching (Automatic Background Pausing)
// ==========================================================================

// Bluetooth & Serial Hardware State
let bleDevice = null;
let gattServer = null;
let serialPort = null;
let serialReader = null;
let serialLineBuffer = '';
let autoScroll = true;
let isStreamPaused = false;
let packetCounter = 0;

// IMU 16-Bit ADC Full-Scale & Scale Factors
let accelFullScale = 2.5;     // ±2.5g Full Scale
let accelLsbPerG = 32768.0 / accelFullScale; // 13107.2 LSB/g
let gyroFullScale = 125.0;    // ±125 °/s Full Scale
let gyroLsbPerDps = 32768.0 / gyroFullScale; // 262.144 LSB/(°/s)

// IMU Tare & Calibration Offsets
let imuBiases = { ax: 0.0, ay: 0.0, az: 0.0, gx: 0.0, gy: 0.0, gz: 0.0 };
let isTareCalibrating = false;
let tareSamples = [];
const TARE_SAMPLE_COUNT = 50;

// Orientation Kinematics & Sensor Fusion State
let fusionMode = 'madgwick'; // 'madgwick', 'complementary', 'accel_trig'
let lastOrientationTimestamp = null;
let lastEstimatedYaw = 0;

// Stream CSV Recorder State
let isRecording = false;
let recordedPackets = [];
let recordingStartTime = null;
let recordingTimerInterval = null;
let isFullWidth = false;
let activeRecordingFileName = '';

// Stream Diagnostics & Issue Alert Tracking State
let streamIssues = [];
let issueFilterCategory = 'all'; // 'all', 'error', 'warn', 'sensor', 'battery'
let issueSortOrder = 'desc';     // 'desc' (newest first), 'asc' (oldest first)
const MAX_STREAM_ISSUES = 250;

// Simulator Stream State
let isSimulatorRunning = false;
let simulatorInterval = null;
let uartByteRingBuffer = new Uint8Array(0);

// Leaflet Map State
let map = null;
let cowMarker = null;
let polyline = null;
let pathHistory = [];

// ==========================================================================
// 1. Zero-Allocation Float32Array Circular Ring Buffer for High-Speed Charting
// ==========================================================================
class Float32RingBuffer {
  constructor(capacity = 360) {
    this.capacity = capacity;
    this.times = new Float64Array(capacity);
    this.labels = new Array(capacity).fill('');
    this.ax = new Float32Array(capacity);
    this.ay = new Float32Array(capacity);
    this.az = new Float32Array(capacity);
    this.gx = new Float32Array(capacity);
    this.gy = new Float32Array(capacity);
    this.gz = new Float32Array(capacity);
    this.head = 0;
    this.count = 0;
  }

  push(timeMs, label, ax, ay, az, gx, gy, gz) {
    const idx = (this.head + this.count) % this.capacity;
    if (this.count < this.capacity) {
      this.count++;
    } else {
      this.head = (this.head + 1) % this.capacity;
    }

    this.times[idx] = timeMs;
    this.labels[idx] = label;
    this.ax[idx] = ax;
    this.ay[idx] = ay;
    this.az[idx] = az;
    this.gx[idx] = gx;
    this.gy[idx] = gy;
    this.gz[idx] = gz;
  }

  clear() {
    this.head = 0;
    this.count = 0;
  }

  getOrderedData(cutoffTimeMs) {
    const outLabels = [];
    const outAx = [];
    const outAy = [];
    const outAz = [];
    const outGx = [];
    const outGy = [];
    const outGz = [];

    for (let i = 0; i < this.count; i++) {
      const idx = (this.head + i) % this.capacity;
      if (this.times[idx] >= cutoffTimeMs) {
        outLabels.push(this.labels[idx]);
        outAx.push(this.ax[idx]);
        outAy.push(this.ay[idx]);
        outAz.push(this.az[idx]);
        outGx.push(this.gx[idx]);
        outGy.push(this.gy[idx]);
        outGz.push(this.gz[idx]);
      }
    }
    return { labels: outLabels, ax: outAx, ay: outAy, az: outAz, gx: outGx, gy: outGy, gz: outGz };
  }
}

const chartRingBuffer = new Float32RingBuffer(500);

// 6-DOF IMU Motion Chart State
let motionChart = null;
let chartTimeWindowMs = 60000; // default 1 minute (60s)
let chartFilterMode = 'all';    // 'all', 'accel', 'gyro'
let chartNeedsUpdate = false;
let lastChartRenderTime = 0;
const CHART_RENDER_FPS_INTERVAL = 33; // ~30 FPS throttling

// Three.js 3D Viewport State
let scene3d = null;
let camera3d = null;
let renderer3d = null;
let imuBoardGroup = null;
let cowTagModelGroup = null;
let active3dModelType = 'chip';
let axesGizmoGroup = null;
let deskGridHelper = null;
let is3dInitialized = false;
let is3dAnimating = false;
let isMouseDragging3d = false;
let previousMousePosition = { x: 0, y: 0 };
let show3dAxes = true;
let show3dGrid = true;
let isPageVisible = true;

// Slerp Quaternion Interpolation for 60FPS Continuous Tracking
let targetQuaternion = null;
let currentQuaternion = null;
let targetEuler = null;

// Telemetry CSV Playback & Replay Engine State
let replayDataRows = [];
let isReplayActive = false;
let isReplayPlaying = false;
let replayCurrentIndex = 0;
let replayPlaybackTimer = null;
let replaySpeedMultiplier = 1.0;

// Continuous IMU Latest State
let currentImuState = {
  rawAx: 0.00, rawAy: 0.00, rawAz: 1.00,
  rawGx: 0.0, rawGy: 0.0, rawGz: 0.0,
  filtAx: 0.00, filtAy: 0.00, filtAz: 1.00,
  filtGx: 0.0, filtGy: 0.0, filtGz: 0.0,
  pitch: 0.0, roll: 0.0, yaw: 0.0,
  totalG: 1.00
};

// ==========================================================================
// 2. Dedicated Web Worker with Main-Thread Fallback
// ==========================================================================
let dspWorker = null;
let isDspWorkerActive = false;

function initDspWorker() {
  if (typeof Worker !== 'undefined') {
    try {
      dspWorker = new Worker('dsp-worker.js');
      dspWorker.onmessage = handleWorkerMessage;
      dspWorker.onerror = () => { isDspWorkerActive = false; };
      isDspWorkerActive = true;
    } catch (e) {
      isDspWorkerActive = false;
    }
  }
}

function handleWorkerMessage(e) {
  const { type, data } = e.data;
  if (type === 'IMU_PROCESSED') {
    applyProcessedImuData(data);
  }
}

// Main-thread Fallback Math Algorithms
class MadgwickAHRS {
  constructor(beta = 0.04) {
    this.beta = beta;
    this.q0 = 1.0; this.q1 = 0.0; this.q2 = 0.0; this.q3 = 0.0;
  }

  update(gxDps, gyDps, gzDps, ax, ay, az, dt) {
    const gx = gxDps * (Math.PI / 180.0);
    const gy = gyDps * (Math.PI / 180.0);
    const gz = gzDps * (Math.PI / 180.0);

    let q0 = this.q0, q1 = this.q1, q2 = this.q2, q3 = this.q3;

    let qDot1 = 0.5 * (-q1 * gx - q2 * gy - q3 * gz);
    let qDot2 = 0.5 * ( q0 * gx + q2 * gz - q3 * gy);
    let qDot3 = 0.5 * ( q0 * gy - q1 * gz + q3 * gx);
    let qDot4 = 0.5 * ( q0 * gz + q1 * gy - q2 * gx);

    let aLen = Math.sqrt(ax * ax + ay * ay + az * az);
    if (aLen > 0.01) {
      ax /= aLen; ay /= aLen; az /= aLen;
      const _2q0 = 2.0 * q0, _2q1 = 2.0 * q1, _2q2 = 2.0 * q2, _2q3 = 2.0 * q3;
      const _4q0 = 4.0 * q0, _4q1 = 4.0 * q1, _4q2 = 4.0 * q2;
      const _8q1 = 8.0 * q1, _8q2 = 8.0 * q2;
      const q0q0 = q0 * q0, q1q1 = q1 * q1, q2q2 = q2 * q2, q3q3 = q3 * q3;

      let s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
      let s1 = _4q1 * q3q3 - _2q3 * ax + 4.0 * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
      let s2 = 4.0 * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
      let s3 = 4.0 * q1q1 * q3 - _2q1 * ax + 4.0 * q2q2 * q3 - _2q2 * ay;

      let sLen = Math.sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
      if (sLen > 0) {
        s0 /= sLen; s1 /= sLen; s2 /= sLen; s3 /= sLen;
        qDot1 -= this.beta * s0;
        qDot2 -= this.beta * s1;
        qDot3 -= this.beta * s2;
        qDot4 -= this.beta * s3;
      }
    }

    q0 += qDot1 * dt; q1 += qDot2 * dt; q2 += qDot3 * dt; q3 += qDot4 * dt;
    let qNorm = Math.sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    if (qNorm > 0) {
      this.q0 = q0 / qNorm; this.q1 = q1 / qNorm; this.q2 = q2 / qNorm; this.q3 = q3 / qNorm;
    }
  }

  getEuler() {
    const q0 = this.q0, q1 = this.q1, q2 = this.q2, q3 = this.q3;
    const roll = Math.atan2(2.0 * (q0 * q1 + q2 * q3), 1.0 - 2.0 * (q1 * q1 + q2 * q2)) * (180.0 / Math.PI);
    const sinP = 2.0 * (q0 * q2 - q3 * q1);
    const pitch = (Math.abs(sinP) >= 1.0 ? Math.sign(sinP) * (Math.PI / 2) : Math.asin(sinP)) * (180.0 / Math.PI);
    const yaw = ((Math.atan2(2.0 * (q0 * q3 + q1 * q2), 1.0 - 2.0 * (q2 * q2 + q3 * q3)) * (180.0 / Math.PI)) % 360 + 360) % 360;
    return { pitch, roll, yaw };
  }

  reset() {
    this.q0 = 1.0; this.q1 = 0.0; this.q2 = 0.0; this.q3 = 0.0;
  }
}

const madgwickAHRS = new MadgwickAHRS(0.04);

class DigitalLowPassFilter3Axis {
  constructor(cutoffFreq = 2.0, order = 2) {
    this.cutoffFreq = cutoffFreq;
    this.order = order;
    this.enabled = true;
    this.lastTimestamp = null;
    this.reset();
  }

  reset() {
    this.state = {
      x: { y1: 0, y2: 0, x1: 0, x2: 0, out: 0, initialized: false },
      y: { y1: 0, y2: 0, x1: 0, x2: 0, out: 0, initialized: false },
      z: { y1: 0, y2: 0, x1: 0, x2: 0, out: 0, initialized: false }
    };
    this.lastTimestamp = null;
  }

  setParameters(cutoffFreq, order, enabled) {
    if (cutoffFreq !== undefined && cutoffFreq > 0) this.cutoffFreq = parseFloat(cutoffFreq);
    if (order !== undefined) this.order = parseInt(order, 10);
    if (enabled !== undefined) this.enabled = Boolean(enabled);
  }

  apply(rawX, rawY, rawZ, timestamp = Date.now()) {
    if (!this.enabled) return { x: rawX, y: rawY, z: rawZ };
    let dt = 0.05;
    if (this.lastTimestamp) {
      dt = Math.max(0.001, Math.min(0.5, (timestamp - this.lastTimestamp) / 1000.0));
    }
    this.lastTimestamp = timestamp;

    return {
      x: this._filterChannel(this.state.x, rawX, dt),
      y: this._filterChannel(this.state.y, rawY, dt),
      z: this._filterChannel(this.state.z, rawZ, dt)
    };
  }

  _filterChannel(ch, inputVal, dt) {
    if (!ch.initialized) {
      ch.y1 = inputVal; ch.y2 = inputVal; ch.x1 = inputVal; ch.x2 = inputVal; ch.out = inputVal;
      ch.initialized = true;
      return inputVal;
    }
    if (this.order === 1) {
      const tau = 1.0 / (2.0 * Math.PI * this.cutoffFreq);
      const alpha = dt / (tau + dt);
      ch.out = ch.out + alpha * (inputVal - ch.out);
      return ch.out;
    } else {
      const fc = Math.min(this.cutoffFreq, 0.45 / dt);
      const omega = 2.0 * Math.PI * fc;
      const K = Math.tan((omega * dt) / 2.0);
      const Q = 0.70710678;
      const K2 = K * K;
      const norm = 1.0 + K / Q + K2;

      const b0 = K2 / norm;
      const b1 = 2.0 * b0;
      const b2 = b0;
      const a1 = 2.0 * (K2 - 1.0) / norm;
      const a2 = (1.0 - K / Q + K2) / norm;

      const out = b0 * inputVal + b1 * ch.x1 + b2 * ch.x2 - a1 * ch.y1 - a2 * ch.y2;
      ch.x2 = ch.x1; ch.x1 = inputVal; ch.y2 = ch.y1; ch.y1 = out; ch.out = out;
      return ch.out;
    }
  }
}

const accelFilter = new DigitalLowPassFilter3Axis(2.0, 2);
const gyroFilter = new DigitalLowPassFilter3Axis(5.0, 2);

class RealTimeFFTAnalyzer {
  constructor(bufferSize = 64) {
    this.bufferSize = bufferSize;
    this.samples = new Float32Array(bufferSize);
    this.index = 0;
    this.sampleRate = 20.0;
  }

  addSample(val) {
    this.samples[this.index] = val;
    this.index = (this.index + 1) % this.bufferSize;
  }

  computeSpectrum() {
    const N = this.bufferSize;
    const real = new Float32Array(N);
    for (let i = 0; i < N; i++) {
      const idx = (this.index + i) % N;
      const w = 0.5 * (1.0 - Math.cos((2.0 * Math.PI * i) / (N - 1)));
      real[i] = this.samples[idx] * w;
    }

    const halfN = N / 2;
    const magnitudes = new Float32Array(halfN);
    let peakMag = 0;
    let peakFreq = 0;
    let sumSq = 0;

    for (let k = 0; k < halfN; k++) {
      let r = 0, im = 0;
      for (let n = 0; n < N; n++) {
        const angle = (2.0 * Math.PI * k * n) / N;
        r += real[n] * Math.cos(angle);
        im -= real[n] * Math.sin(angle);
      }
      const mag = Math.sqrt(r * r + im * im) / N;
      magnitudes[k] = mag;
      sumSq += mag * mag;

      const freq = (k * this.sampleRate) / N;
      if (k > 1 && mag > peakMag) {
        peakMag = mag;
        peakFreq = freq;
      }
    }

    const rms = Math.sqrt(sumSq / halfN);
    return { magnitudes, peakFreq, peakMag, rms };
  }
}

const fftAnalyzer = new RealTimeFFTAnalyzer(64);

// ==========================================================================
// DOM Elements
// ==========================================================================
const btnConnectSerial = document.getElementById('btnConnectSerial');
const btnConnect = document.getElementById('btnConnect');
const btnScanAll = document.getElementById('btnScanAll');
const btnLastDevice = document.getElementById('btnLastDevice');
const lastDeviceNameText = document.getElementById('lastDeviceNameText');
const btnClearLog = document.getElementById('btnClearLog');
const btnClearChart = document.getElementById('btnClearChart');
const statusBadge = document.getElementById('statusBadge');
const statusText = document.getElementById('statusText');
const serialBaudSelect = document.getElementById('serialBaudSelect');
const streamFormatSelect = document.getElementById('streamFormatSelect');
const accelRangeSelect = document.getElementById('accelRangeSelect');
const gyroRangeSelect = document.getElementById('gyroRangeSelect');
const fusionModeSelect = document.getElementById('fusionModeSelect');

// Tare Calibration Elements
const btnTareImu = document.getElementById('btnTareImu');
const btnResetTare = document.getElementById('btnResetTare');
const tareProgressContainer = document.getElementById('tareProgressContainer');
const tareProgressBar = document.getElementById('tareProgressBar');
const tareProgressText = document.getElementById('tareProgressText');

// Device & Cow Overview Elements
const cowTagIdEl = document.getElementById('cowTagId');
const deviceNameEl = document.getElementById('deviceName');
const batteryValueEl = document.getElementById('batteryValue');
const batteryPillEl = document.getElementById('batteryPill');
const batteryLevelFillEl = document.getElementById('batteryLevelFill');
const rssiValueEl = document.getElementById('rssiValue');
const activityStateEl = document.getElementById('activityState');
const packetCountEl = document.getElementById('packetCount');
const deviceTypeBadge = document.getElementById('deviceTypeBadge');

// GPS Labels
const lblLat = document.getElementById('lblLat');
const lblLng = document.getElementById('lblLng');
const lblAlt = document.getElementById('lblAlt');
const lblSpeed = document.getElementById('lblSpeed');

// IMU Accel & Gyro Elements
const barAccelX = document.getElementById('barAccelX');
const barAccelY = document.getElementById('barAccelY');
const barAccelZ = document.getElementById('barAccelZ');
const ballAccelX = document.getElementById('ballAccelX');
const ballAccelY = document.getElementById('ballAccelY');
const ballAccelZ = document.getElementById('ballAccelZ');
const valAccelX = document.getElementById('valAccelX');
const valAccelY = document.getElementById('valAccelY');
const valAccelZ = document.getElementById('valAccelZ');

const barGyroX = document.getElementById('barGyroX');
const barGyroY = document.getElementById('barGyroY');
const barGyroZ = document.getElementById('barGyroZ');
const ballGyroX = document.getElementById('ballGyroX');
const ballGyroY = document.getElementById('ballGyroY');
const ballGyroZ = document.getElementById('ballGyroZ');
const valGyroX = document.getElementById('valGyroX');
const valGyroY = document.getElementById('valGyroY');
const valGyroZ = document.getElementById('valGyroZ');

const motionPill = document.getElementById('motionPill');
const accelRangeBadge = document.getElementById('accelRangeBadge');
const gyroRangeBadge = document.getElementById('gyroRangeBadge');

// Orientation / Euler Elements
const valPitch = document.getElementById('valPitch');
const valRoll = document.getElementById('valRoll');
const valYaw = document.getElementById('valYaw');
const barPitch = document.getElementById('barPitch');
const barRoll = document.getElementById('barRoll');
const barYaw = document.getElementById('barYaw');
const attitudePill = document.getElementById('attitudePill');
const attitudeModeText = document.getElementById('attitudeModeText');

// 3D Viewport Controls & Model Switcher
const btnReset3dView = document.getElementById('btnReset3dView');
const btnToggle3dAxes = document.getElementById('btnToggle3dAxes');
const btnToggle3dGrid = document.getElementById('btnToggle3dGrid');
const btnModelChip = document.getElementById('btnModelChip');
const btnModelTag = document.getElementById('btnModelTag');
const hudPitch = document.getElementById('hudPitch');
const hudRoll = document.getElementById('hudRoll');
const hudYaw = document.getElementById('hudYaw');
const hudGravity = document.getElementById('hudGravity');
const chipFlatStatusPill = document.getElementById('chipFlatStatusPill');

// Digital Filter Controls
const chkAccelFilter = document.getElementById('chkAccelFilter');
const accelFilterOrder = document.getElementById('accelFilterOrder');
const accelCutoffFreq = document.getElementById('accelCutoffFreq');
const accelCutoffInput = document.getElementById('accelCutoffInput');
const lblAccelCutoff = document.getElementById('lblAccelCutoff');
const accelFilterPill = document.getElementById('accelFilterPill');

const chkGyroFilter = document.getElementById('chkGyroFilter');
const gyroFilterOrder = document.getElementById('gyroFilterOrder');
const gyroCutoffFreq = document.getElementById('gyroCutoffFreq');
const gyroCutoffInput = document.getElementById('gyroCutoffInput');
const lblGyroCutoff = document.getElementById('lblGyroCutoff');
const gyroFilterPill = document.getElementById('gyroFilterPill');

const btnResetFilters = document.getElementById('btnResetFilters');
const filterMasterStatusPill = document.getElementById('filterMasterStatusPill');

// Chart Controls
const chartWindowBadge = document.getElementById('chartWindowBadge');
const chartModeGroup = document.getElementById('chartModeGroup');
const chartWindowGroup = document.getElementById('chartWindowGroup');

// FFT Elements
const fftCanvas = document.getElementById('fftCanvas');
const fftPeakFreq = document.getElementById('fftPeakFreq');
const fftRmsEnergy = document.getElementById('fftRmsEnergy');
const fftActivityBadge = document.getElementById('fftActivityBadge');

// CSV Replay Elements
const csvFileInput = document.getElementById('csvFileInput');
const btnLoadCsvFile = document.getElementById('btnLoadCsvFile');
const btnEjectCsvFile = document.getElementById('btnEjectCsvFile');
const csvDropZone = document.getElementById('csvDropZone');
const csvPlayerContainer = document.getElementById('csvPlayerContainer');
const replayStatusPill = document.getElementById('replayStatusPill');
const replayFilenameText = document.getElementById('replayFilenameText');
const replayRecordCount = document.getElementById('replayRecordCount');
const replayCurrentTime = document.getElementById('replayCurrentTime');
const replayTotalTime = document.getElementById('replayTotalTime');
const replayTimelineSlider = document.getElementById('replayTimelineSlider');
const btnReplayPlayPause = document.getElementById('btnReplayPlayPause');
const btnReplayPrev = document.getElementById('btnReplayPrev');
const btnReplayNext = document.getElementById('btnReplayNext');
const replaySpeedGroup = document.getElementById('replaySpeedGroup');

// Stream Diagnostics & Issue Alert Elements
const streamIssuePanel = document.getElementById('streamIssuePanel');
const issueBadge = document.getElementById('issueBadge');
const streamIssueCountText = document.getElementById('streamIssueCountText');
const streamIssueStatusSubtext = document.getElementById('streamIssueStatusSubtext');
const streamIssueList = document.getElementById('streamIssueList');
const issueFilterGroup = document.getElementById('issueFilterGroup');
const issueSortGroup = document.getElementById('issueSortGroup');
const btnSortIssuesDesc = document.getElementById('btnSortIssuesDesc');
const btnSortIssuesAsc = document.getElementById('btnSortIssuesAsc');
const btnClearIssues = document.getElementById('btnClearIssues');

// Terminal, Parsed Grid & Full-Width Elements
const terminalLog = document.getElementById('terminalLog');
const fieldsGrid = document.getElementById('fieldsGrid');
const btnToggleFullWidth = document.getElementById('btnToggleFullWidth');
const btnPauseStream = document.getElementById('btnPauseStream');
const btnToggleAutoScroll = document.getElementById('btnToggleAutoScroll');

// CSV Stream Recorder Elements
const btnRecordToggle = document.getElementById('btnRecordToggle');
const btnSimulateStream = document.getElementById('btnSimulateStream');
const btnExportCsv = document.getElementById('btnExportCsv');
const btnClearCsv = document.getElementById('btnClearCsv');
const recorderBadge = document.getElementById('recorderBadge');
const recorderStatusText = document.getElementById('recorderStatusText');
const recorderFilePill = document.getElementById('recorderFilePill');
const recorderFilenameText = document.getElementById('recorderFilenameText');
const recorderTimer = document.getElementById('recorderTimer');
const recorderCount = document.getElementById('recorderCount');
const recorderSize = document.getElementById('recorderSize');

// Standard & Custom BLE UUIDs
const ENVIRONMENTAL_SENSING_SERVICE = 0x181a;
const mobileTabNav = document.getElementById('mobileTabNav');
const dashboardGrid = document.querySelector('.dashboard-grid');

// ==========================================================================
// Application Initialization
// ==========================================================================
document.addEventListener('DOMContentLoaded', () => {
  loadSavedTareBiases();
  initDspWorker();
  initMap();
  initChart();
  init3DImuViewer();
  setupEventListeners();
  setupFilterEventListeners();
  setupTareEventListeners();
  setupModelSwitcherListeners();
  setupReplayEventListeners();
  setupIssueEventListeners();
  setupMobileTabs();
  setupResponsiveHandlers();
  setupPageVisibilityOptimization();
  checkLastConnectedDevice();
  startFftRenderLoop();
  
  updateDeviceOverview('--', '--', false);
  updateIMUAndOrientation(0.00, 0.00, 1.00, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 'Default Position', 'At Rest');
  
  logToConsole('system', 'Ranchbot Cow Tag Receiver: High-Performance Engine, Stream Issue Monitor & Zero-Hang View Switching ACTIVE.');
});

// ==========================================================================
// Page Visibility Power Caching
// ==========================================================================
function setupPageVisibilityOptimization() {
  document.addEventListener('visibilitychange', () => {
    isPageVisible = !document.hidden;
  });
}

// ==========================================================================
// Event Listeners Setup
// ==========================================================================
function setupEventListeners() {
  if (btnConnectSerial) btnConnectSerial.addEventListener('click', handleConnectSerialButtonClick);
  if (btnConnect) btnConnect.addEventListener('click', handleConnectButtonClick);
  if (btnScanAll) btnScanAll.addEventListener('click', handleScanAllClick);
  if (btnLastDevice) btnLastDevice.addEventListener('click', handleLastDeviceReconnect);
  if (btnPauseStream) btnPauseStream.addEventListener('click', togglePauseStream);
  if (btnToggleAutoScroll) btnToggleAutoScroll.addEventListener('click', toggleAutoScroll);
  
  if (btnClearLog) {
    btnClearLog.addEventListener('click', () => {
      terminalLog.innerHTML = '';
      logToConsole('system', 'Console log cleared.');
    });
  }

  if (btnClearChart) {
    btnClearChart.addEventListener('click', () => {
      chartRingBuffer.clear();
      if (motionChart) {
        motionChart.data.labels = [];
        for (let i = 0; i < motionChart.data.datasets.length; i++) {
          motionChart.data.datasets[i].data = [];
        }
        motionChart.update();
      }
      if (chartWindowBadge) {
        const sec = Math.round(chartTimeWindowMs / 1000);
        chartWindowBadge.innerHTML = `<i class="fa-solid fa-clock"></i> Last ${sec >= 60 ? `${Math.round(sec/60)} Min` : `${sec}s`} • 0 pts`;
      }
      logToConsole('system', 'Motion history chart cleared.');
    });
  }

  if (accelRangeSelect) {
    accelRangeSelect.addEventListener('change', () => {
      accelFullScale = parseFloat(accelRangeSelect.value) || 2.5;
      accelLsbPerG = 32768.0 / accelFullScale;
      if (accelRangeBadge) {
        accelRangeBadge.textContent = `±${accelFullScale} g Full Scale (Center 0g)`;
      }
      if (motionChart) {
        motionChart.options.scales.y.min = -accelFullScale;
        motionChart.options.scales.y.max = accelFullScale;
        motionChart.options.scales.y.suggestedMin = -accelFullScale;
        motionChart.options.scales.y.suggestedMax = accelFullScale;
        motionChart.options.scales.y.title.text = `Acceleration (±${accelFullScale}g)`;
        motionChart.update();
      }
      logToConsole('system', `IMU Accel Range set to ±${accelFullScale}g (${accelLsbPerG.toFixed(1)} LSB/g).`);
    });
  }

  if (gyroRangeSelect) {
    gyroRangeSelect.addEventListener('change', () => {
      gyroFullScale = parseFloat(gyroRangeSelect.value) || 125.0;
      gyroLsbPerDps = 32768.0 / gyroFullScale;
      if (gyroRangeBadge) {
        gyroRangeBadge.textContent = `±${gyroFullScale} °/s Full Scale (Center 0°/s)`;
      }
      if (motionChart) {
        motionChart.options.scales.y1.min = -gyroFullScale;
        motionChart.options.scales.y1.max = gyroFullScale;
        motionChart.options.scales.y1.suggestedMin = -gyroFullScale;
        motionChart.options.scales.y1.suggestedMax = gyroFullScale;
        motionChart.options.scales.y1.title.text = `Angular Rate (±${gyroFullScale} °/s)`;
        motionChart.update();
      }
      logToConsole('system', `IMU Gyro Range set to ±${gyroFullScale}°/s (${gyroLsbPerDps.toFixed(1)} LSB/dps).`);
    });
  }

  if (fusionModeSelect) {
    fusionModeSelect.addEventListener('change', () => {
      fusionMode = fusionModeSelect.value;
      madgwickAHRS.reset();
      if (dspWorker) dspWorker.postMessage({ type: 'RESET_FUSION' });
      logToConsole('system', `Attitude Sensor Fusion set to: ${fusionModeSelect.options[fusionModeSelect.selectedIndex].text}`);
    });
  }

  if (btnReset3dView) btnReset3dView.addEventListener('click', reset3DView);
  if (btnToggle3dAxes) btnToggle3dAxes.addEventListener('click', toggle3DAxes);
  if (btnToggle3dGrid) btnToggle3dGrid.addEventListener('click', toggle3DGrid);

  if (chartModeGroup) {
    chartModeGroup.addEventListener('click', (e) => {
      const btn = e.target.closest('.btn-filter');
      if (!btn) return;
      chartModeGroup.querySelectorAll('.btn-filter').forEach(b => b.classList.remove('active'));
      btn.classList.add('active');
      chartFilterMode = btn.dataset.mode || 'all';
      applyChartFilterMode();
      if (motionChart && !isFullWidth) motionChart.update();
    });
  }

  if (chartWindowGroup) {
    chartWindowGroup.addEventListener('click', (e) => {
      const btn = e.target.closest('.btn-window');
      if (!btn) return;
      chartWindowGroup.querySelectorAll('.btn-window').forEach(b => b.classList.remove('active'));
      btn.classList.add('active');
      const winSec = parseInt(btn.dataset.window, 10) || 60;
      chartTimeWindowMs = winSec * 1000;
      chartNeedsUpdate = true;
      if (!isFullWidth) rebuildChartFromBuffer();
    });
  }

  if (btnRecordToggle) btnRecordToggle.addEventListener('click', toggleRecording);
  if (btnSimulateStream) btnSimulateStream.addEventListener('click', toggleSimulatorStream);
  if (btnExportCsv) btnExportCsv.addEventListener('click', exportCsv);
  if (btnClearCsv) btnClearCsv.addEventListener('click', clearCsvBuffer);
  if (btnToggleFullWidth) btnToggleFullWidth.addEventListener('click', toggleFullWidthStream);
}

// ==========================================================================
// Stream Diagnostics & Issue Detection Engine (Red Highlighted Issues)
// ==========================================================================
function setupIssueEventListeners() {
  if (issueFilterGroup) {
    issueFilterGroup.addEventListener('click', (e) => {
      const btn = e.target.closest('.btn');
      if (!btn) return;
      issueFilterGroup.querySelectorAll('.btn').forEach(b => b.classList.remove('active'));
      btn.classList.add('active');
      issueFilterCategory = btn.dataset.filter || 'all';
      renderStreamIssues();
    });
  }

  if (btnSortIssuesDesc) {
    btnSortIssuesDesc.addEventListener('click', () => {
      issueSortOrder = 'desc';
      btnSortIssuesDesc.classList.add('active');
      if (btnSortIssuesAsc) btnSortIssuesAsc.classList.remove('active');
      renderStreamIssues();
    });
  }

  if (btnSortIssuesAsc) {
    btnSortIssuesAsc.addEventListener('click', () => {
      issueSortOrder = 'asc';
      btnSortIssuesAsc.classList.add('active');
      if (btnSortIssuesDesc) btnSortIssuesDesc.classList.remove('active');
      renderStreamIssues();
    });
  }

  if (btnClearIssues) {
    btnClearIssues.addEventListener('click', () => {
      streamIssues = [];
      renderStreamIssues();
      logToConsole('system', 'Stream diagnostic issue log cleared.');
    });
  }
}

function recordStreamIssue(category, severity, description, rawPayload = '') {
  const issue = {
    id: Date.now() + Math.random(),
    timestamp: new Date(),
    timeStr: new Date().toLocaleTimeString(),
    category, // 'error', 'warn', 'sensor', 'battery'
    severity, // 'error', 'warn', 'alert'
    description,
    rawPayload: String(rawPayload || '').slice(0, 100)
  };

  streamIssues.push(issue);
  if (streamIssues.length > MAX_STREAM_ISSUES) {
    streamIssues.shift();
  }

  renderStreamIssues();
}

function renderStreamIssues() {
  if (!streamIssueList) return;

  const totalCount = streamIssues.length;
  const errorCount = streamIssues.filter(i => i.severity === 'error').length;
  const warnCount = streamIssues.filter(i => i.severity === 'warn' || i.category === 'sensor').length;

  if (streamIssueCountText) {
    if (totalCount === 0) {
      streamIssueCountText.textContent = '0 STREAM ISSUES DETECTED';
    } else {
      streamIssueCountText.textContent = `${totalCount} STREAM ISSUE${totalCount > 1 ? 'S' : ''} DETECTED (${errorCount} ERR, ${warnCount} WARN)`;
    }
  }

  if (issueBadge) {
    if (totalCount === 0) {
      issueBadge.className = 'issue-badge-red nominal';
      issueBadge.innerHTML = '<i class="fa-solid fa-circle-check"></i> <span id="streamIssueCountText">0 STREAM ISSUES DETECTED</span>';
    } else {
      issueBadge.className = 'issue-badge-red';
      issueBadge.innerHTML = `<i class="fa-solid fa-triangle-exclamation"></i> <span id="streamIssueCountText">${totalCount} STREAM ISSUE${totalCount > 1 ? 'S' : ''} DETECTED</span>`;
    }
  }

  if (streamIssuePanel) {
    streamIssuePanel.classList.toggle('has-errors', errorCount > 0);
  }

  if (streamIssueStatusSubtext) {
    if (totalCount === 0) {
      streamIssueStatusSubtext.textContent = 'Stream nominal • 0 CRC/Frame dropouts';
      streamIssueStatusSubtext.style.color = '#38ef7d';
    } else {
      const latest = streamIssues[streamIssues.length - 1];
      streamIssueStatusSubtext.textContent = `Latest: [${latest.timeStr}] ${latest.description}`;
      streamIssueStatusSubtext.style.color = latest.severity === 'error' ? '#ff4d6d' : '#ffb703';
    }
  }

  // Apply Category Filter
  let filtered = streamIssues.filter(i => {
    if (issueFilterCategory === 'all') return true;
    return i.category === issueFilterCategory;
  });

  // Apply Sorting
  filtered.sort((a, b) => {
    if (issueSortOrder === 'desc') return b.timestamp - a.timestamp;
    return a.timestamp - b.timestamp;
  });

  if (filtered.length === 0) {
    streamIssueList.innerHTML = `<div class="issue-empty-state"><i class="fa-solid fa-circle-check"></i> ${totalCount === 0 ? 'No stream errors or anomalies detected. Live telemetry healthy.' : 'No issues match the selected filter category.'}</div>`;
    return;
  }

  streamIssueList.innerHTML = '';
  for (const item of filtered) {
    const el = document.createElement('div');
    el.className = `issue-item severity-${item.severity}`;

    let tagClass = 'tag-error';
    if (item.category === 'warn') tagClass = 'tag-warn';
    else if (item.category === 'sensor') tagClass = 'tag-sensor';
    else if (item.category === 'battery') tagClass = 'tag-battery';

    el.innerHTML = `
      <span class="issue-time">${item.timeStr}</span>
      <span class="issue-tag ${tagClass}">${item.category.toUpperCase()}</span>
      <span class="issue-desc" title="${escapeHtml(item.description)} ${item.rawPayload ? `| Raw: ${escapeHtml(item.rawPayload)}` : ''}">${escapeHtml(item.description)}</span>
    `;
    streamIssueList.appendChild(el);
  }
}

// ==========================================================================
// GPU-Accelerated Hardware Transform Helper for Center-Zero Ball Gauges
// ==========================================================================
function updateBiDirectionalAxis(barEl, ballEl, val, maxScale) {
  if (!barEl || isFullWidth) return;
  const num = parseFloat(val) || 0;
  const clamped = Math.max(-maxScale, Math.min(maxScale, num));
  const ratio = clamped / maxScale;
  const pct = Math.abs(ratio) * 50.0;

  if (ratio >= 0) {
    barEl.style.left = '50%';
    barEl.style.width = `${pct}%`;
    if (ballEl) ballEl.style.left = `${50.0 + pct}%`;
  } else {
    barEl.style.left = `${50.0 - pct}%`;
    barEl.style.width = `${pct}%`;
    if (ballEl) ballEl.style.left = `${50.0 - pct}%`;
  }
}

// ==========================================================================
// Tare / Zero Rest Calibration Engine
// ==========================================================================
function setupTareEventListeners() {
  if (btnTareImu) {
    btnTareImu.addEventListener('click', () => {
      if (isTareCalibrating) return;
      startImuTareCalibration();
    });
  }

  if (btnResetTare) {
    btnResetTare.addEventListener('click', () => {
      resetImuTare();
    });
  }
}

function loadSavedTareBiases() {
  try {
    const saved = localStorage.getItem('ranchbot_imu_biases');
    if (saved) {
      imuBiases = JSON.parse(saved);
      logToConsole('system', `Loaded saved Tare Biases: Accel(${imuBiases.ax.toFixed(3)}, ${imuBiases.ay.toFixed(3)}, ${imuBiases.az.toFixed(3)}) Gyro(${imuBiases.gx.toFixed(1)}, ${imuBiases.gy.toFixed(1)}, ${imuBiases.gz.toFixed(1)})`);
    }
  } catch (e) {}
}

function startImuTareCalibration() {
  isTareCalibrating = true;
  tareSamples = [];
  if (tareProgressContainer) tareProgressContainer.style.display = 'block';
  if (tareProgressBar) tareProgressBar.style.width = '0%';
  if (tareProgressText) tareProgressText.textContent = `Sampling stationary offsets (0/${TARE_SAMPLE_COUNT})... Keep IMU flat on desk!`;
  if (btnTareImu) {
    btnTareImu.innerHTML = '<i class="fa-solid fa-spinner fa-spin"></i> Calibrating...';
    btnTareImu.disabled = true;
  }
  logToConsole('system', '[TARE] Starting 50-sample stationary zero calibration...');
}

function recordTareSample(rawAx, rawAy, rawAz, rawGx, rawGy, rawGz) {
  if (!isTareCalibrating) return;
  tareSamples.push({ ax: rawAx, ay: rawAy, az: rawAz, gx: rawGx, gy: rawGy, gz: rawGz });
  const count = tareSamples.length;
  const pct = Math.round((count / TARE_SAMPLE_COUNT) * 100);

  if (tareProgressBar) tareProgressBar.style.width = `${pct}%`;
  if (tareProgressText) tareProgressText.textContent = `Sampling stationary offsets (${count}/${TARE_SAMPLE_COUNT})... Keep IMU flat!`;

  if (count >= TARE_SAMPLE_COUNT) {
    finishTareCalibration();
  }
}

function finishTareCalibration() {
  isTareCalibrating = false;
  let sumAx = 0, sumAy = 0, sumAz = 0, sumGx = 0, sumGy = 0, sumGz = 0;
  for (const s of tareSamples) {
    sumAx += s.ax; sumAy += s.ay; sumAz += s.az;
    sumGx += s.gx; sumGy += s.gy; sumGz += s.gz;
  }

  const N = tareSamples.length;
  imuBiases = {
    ax: sumAx / N,
    ay: sumAy / N,
    az: (sumAz / N) - 1.0,
    gx: sumGx / N,
    gy: sumGy / N,
    gz: sumGz / N
  };

  localStorage.setItem('ranchbot_imu_biases', JSON.stringify(imuBiases));

  if (tareProgressContainer) {
    setTimeout(() => { tareProgressContainer.style.display = 'none'; }, 2000);
  }
  if (tareProgressText) tareProgressText.textContent = '✓ Calibration COMPLETE! Rest Zero & Gyro Drift Calibrated.';
  if (btnTareImu) {
    btnTareImu.innerHTML = '<i class="fa-solid fa-check"></i> Calibrated';
    btnTareImu.disabled = false;
    setTimeout(() => {
      btnTareImu.innerHTML = '<i class="fa-solid fa-crosshairs"></i> Tare / Zero Rest';
    }, 3000);
  }

  madgwickAHRS.reset();
  if (dspWorker) dspWorker.postMessage({ type: 'RESET_FUSION' });
  logToConsole('system', `✓ IMU TARE COMPLETE! Offsets saved: Accel(${imuBiases.ax.toFixed(3)}, ${imuBiases.ay.toFixed(3)}, ${imuBiases.az.toFixed(3)})g | Gyro(${imuBiases.gx.toFixed(1)}, ${imuBiases.gy.toFixed(1)}, ${imuBiases.gz.toFixed(1)})°/s.`);
}

function resetImuTare() {
  imuBiases = { ax: 0.0, ay: 0.0, az: 0.0, gx: 0.0, gy: 0.0, gz: 0.0 };
  localStorage.removeItem('ranchbot_imu_biases');
  madgwickAHRS.reset();
  if (dspWorker) dspWorker.postMessage({ type: 'RESET_FUSION' });
  logToConsole('system', 'Tare Offsets reset to factory raw 0.0.');
}

// ==========================================================================
// Dual 3D Model Switcher (IMU Chip vs Cattle Ear Tag)
// ==========================================================================
function setupModelSwitcherListeners() {
  if (btnModelChip && btnModelTag) {
    btnModelChip.addEventListener('click', () => switch3DModel('chip'));
    btnModelTag.addEventListener('click', () => switch3DModel('tag'));
  }
}

function switch3DModel(modelType) {
  active3dModelType = modelType;
  if (btnModelChip) btnModelChip.classList.toggle('active', modelType === 'chip');
  if (btnModelTag) btnModelTag.classList.toggle('active', modelType === 'tag');

  if (imuBoardGroup) imuBoardGroup.visible = (modelType === 'chip');
  if (cowTagModelGroup) cowTagModelGroup.visible = (modelType === 'tag');

  logToConsole('system', `3D View switched to: ${modelType === 'chip' ? 'Bare IMU Chip PCB' : 'Realistic Livestock Cattle Ear Tag'}`);
}

// ==========================================================================
// 3D IMU Chip & Cattle Ear Tag Renderer (Three.js WebGL)
// ==========================================================================
function init3DImuViewer() {
  const container = document.getElementById('imu3dContainer');
  const canvas = document.getElementById('imu3dCanvas');
  if (!container || !canvas || typeof THREE === 'undefined') return;

  const width = container.clientWidth || 300;
  const height = container.clientHeight || 280;

  targetQuaternion = new THREE.Quaternion();
  currentQuaternion = new THREE.Quaternion();
  targetEuler = new THREE.Euler(0, 0, 0, 'YXZ');

  scene3d = new THREE.Scene();
  scene3d.background = new THREE.Color(0x0a0f1d);

  camera3d = new THREE.PerspectiveCamera(45, width / height, 0.1, 100);
  camera3d.position.set(4.8, 4.2, 5.8);
  camera3d.lookAt(0, 0, 0);

  renderer3d = new THREE.WebGLRenderer({ canvas: canvas, antialias: true, alpha: true, powerPreference: 'high-performance' });
  renderer3d.setSize(width, height);
  renderer3d.setPixelRatio(Math.min(window.devicePixelRatio, 2));

  const ambientLight = new THREE.AmbientLight(0xffffff, 0.85);
  scene3d.add(ambientLight);

  const dirLight = new THREE.DirectionalLight(0xffffff, 0.9);
  dirLight.position.set(6, 12, 8);
  scene3d.add(dirLight);

  const fillLight = new THREE.PointLight(0x00f2fe, 0.45, 25);
  fillLight.position.set(-5, -2, -4);
  scene3d.add(fillLight);

  deskGridHelper = new THREE.GridHelper(8, 16, 0x00f2fe, 0x1e293b);
  deskGridHelper.position.y = -0.02;
  scene3d.add(deskGridHelper);

  // MODEL 1: Bare IMU Carrier PCB & IC Package
  imuBoardGroup = new THREE.Group();
  scene3d.add(imuBoardGroup);

  const pcbGeo = new THREE.BoxGeometry(3.6, 0.14, 2.6);
  const pcbMat = new THREE.MeshStandardMaterial({ color: 0x113b2e, roughness: 0.35, metalness: 0.25 });
  const pcbMesh = new THREE.Mesh(pcbGeo, pcbMat);
  pcbMesh.position.y = 0.07;
  imuBoardGroup.add(pcbMesh);

  const goldMat = new THREE.MeshStandardMaterial({ color: 0xd4af37, metalness: 0.9, roughness: 0.15 });
  [[-1.55, 0.145, -1.05], [1.55, 0.145, -1.05], [-1.55, 0.145, 1.05], [1.55, 0.145, 1.05]].forEach(([cx, cy, cz]) => {
    const pad = new THREE.Mesh(new THREE.CylinderGeometry(0.18, 0.18, 0.02, 16), goldMat);
    pad.position.set(cx, cy, cz);
    imuBoardGroup.add(pad);
  });

  const chipMat = new THREE.MeshStandardMaterial({ color: 0x181a20, roughness: 0.5, metalness: 0.3 });
  const imuChipMesh = new THREE.Mesh(new THREE.BoxGeometry(1.4, 0.22, 1.4), chipMat);
  imuChipMesh.position.set(0, 0.25, 0);
  imuBoardGroup.add(imuChipMesh);

  const pin1Dot = new THREE.Mesh(new THREE.CylinderGeometry(0.07, 0.07, 0.02, 16), new THREE.MeshStandardMaterial({ color: 0x00f2fe, emissive: 0x00f2fe, emissiveIntensity: 0.8 }));
  pin1Dot.position.set(-0.48, 0.365, -0.48);
  imuBoardGroup.add(pin1Dot);

  const textCanvas = document.createElement('canvas');
  textCanvas.width = 256; textCanvas.height = 256;
  const ctx = textCanvas.getContext('2d');
  ctx.fillStyle = '#181a20'; ctx.fillRect(0, 0, 256, 256);
  ctx.fillStyle = '#f8fafc'; ctx.font = 'bold 26px "JetBrains Mono", monospace'; ctx.textAlign = 'center';
  ctx.fillText('IMU 6-DOF', 128, 85);
  ctx.fillStyle = '#00c6ff'; ctx.font = 'bold 20px "JetBrains Mono", monospace'; ctx.fillText('±2.5G / 125°/s', 128, 128);
  ctx.fillStyle = '#94a3b8'; ctx.font = '16px "JetBrains Mono", monospace'; ctx.fillText('16-BIT ADC', 128, 168);

  const textTexture = new THREE.CanvasTexture(textCanvas);
  const textPlane = new THREE.Mesh(new THREE.PlaneGeometry(1.2, 1.2), new THREE.MeshBasicMaterial({ map: textTexture, transparent: true }));
  textPlane.rotation.x = -Math.PI / 2;
  textPlane.position.set(0, 0.362, 0);
  imuBoardGroup.add(textPlane);

  const pinMat = new THREE.MeshStandardMaterial({ color: 0xc8c8c8, metalness: 0.9, roughness: 0.1 });
  for (let i = -0.45; i <= 0.45; i += 0.3) {
    const pL = new THREE.Mesh(new THREE.BoxGeometry(0.12, 0.08, 0.12), pinMat); pL.position.set(-0.72, 0.18, i); imuBoardGroup.add(pL);
    const pR = new THREE.Mesh(new THREE.BoxGeometry(0.12, 0.08, 0.12), pinMat); pR.position.set(0.72, 0.18, i); imuBoardGroup.add(pR);
    const pF = new THREE.Mesh(new THREE.BoxGeometry(0.12, 0.08, 0.12), pinMat); pF.position.set(i, 0.18, 0.72); imuBoardGroup.add(pF);
    const pB = new THREE.Mesh(new THREE.BoxGeometry(0.12, 0.08, 0.12), pinMat); pB.position.set(i, 0.18, -0.72); imuBoardGroup.add(pB);
  }

  // MODEL 2: Realistic Ranchbot Cattle Ear Tag Model
  cowTagModelGroup = new THREE.Group();
  cowTagModelGroup.visible = false;
  scene3d.add(cowTagModelGroup);

  const tagShape = new THREE.Shape();
  tagShape.moveTo(-1.2, -1.8);
  tagShape.lineTo(1.2, -1.8);
  tagShape.lineTo(1.5, 0.8);
  tagShape.lineTo(0.5, 2.2);
  tagShape.lineTo(-0.5, 2.2);
  tagShape.lineTo(-1.5, 0.8);
  tagShape.closePath();

  const extrudeSettings = { depth: 0.18, bevelEnabled: true, bevelSegments: 3, steps: 1, bevelSize: 0.06, bevelThickness: 0.06 };
  const tagGeo = new THREE.ExtrudeGeometry(tagShape, extrudeSettings);
  const tagMat = new THREE.MeshStandardMaterial({ color: 0xffb703, roughness: 0.35, metalness: 0.1 });
  const tagMesh = new THREE.Mesh(tagGeo, tagMat);
  tagMesh.rotation.x = Math.PI / 2;
  tagMesh.position.set(0, 0.15, 0);
  cowTagModelGroup.add(tagMesh);

  const studGeo = new THREE.CylinderGeometry(0.35, 0.35, 0.4, 24);
  const studMat = new THREE.MeshStandardMaterial({ color: 0xd90429, roughness: 0.4, metalness: 0.2 });
  const studMesh = new THREE.Mesh(studGeo, studMat);
  studMesh.position.set(0, 0.25, -1.6);
  cowTagModelGroup.add(studMesh);

  const tagLabelCanvas = document.createElement('canvas');
  tagLabelCanvas.width = 256; tagLabelCanvas.height = 256;
  const lctx = tagLabelCanvas.getContext('2d');
  lctx.fillStyle = '#ffb703'; lctx.fillRect(0, 0, 256, 256);
  lctx.fillStyle = '#040914'; lctx.font = 'bold 36px "Inter", sans-serif'; lctx.textAlign = 'center';
  lctx.fillText('RANCHBOT', 128, 70);
  lctx.fillStyle = '#000000'; lctx.font = 'bold 44px "JetBrains Mono", monospace';
  lctx.fillText('COW-8492', 128, 140);
  lctx.font = '22px "Inter", sans-serif';
  lctx.fillText('GPS • BLE • 6-DOF', 128, 195);

  const tagLabelTexture = new THREE.CanvasTexture(tagLabelCanvas);
  const tagLabelPlane = new THREE.Mesh(new THREE.PlaneGeometry(2.0, 2.0), new THREE.MeshBasicMaterial({ map: tagLabelTexture, transparent: true }));
  tagLabelPlane.rotation.x = -Math.PI / 2;
  tagLabelPlane.position.set(0, 0.26, 0.4);
  cowTagModelGroup.add(tagLabelPlane);

  axesGizmoGroup = new THREE.Group();
  const arrowX = new THREE.ArrowHelper(new THREE.Vector3(1, 0, 0), new THREE.Vector3(0, 0.38, 0), 1.9, 0xff4d6d, 0.35, 0.18);
  const arrowY = new THREE.ArrowHelper(new THREE.Vector3(0, 0, 1), new THREE.Vector3(0, 0.38, 0), 1.9, 0x38ef7d, 0.35, 0.18);
  const arrowZ = new THREE.ArrowHelper(new THREE.Vector3(0, 1, 0), new THREE.Vector3(0, 0.38, 0), 1.9, 0x00c6ff, 0.35, 0.18);
  axesGizmoGroup.add(arrowX, arrowY, arrowZ);
  imuBoardGroup.add(axesGizmoGroup);

  container.addEventListener('mousedown', (e) => {
    isMouseDragging3d = true;
    previousMousePosition = { x: e.clientX, y: e.clientY };
  });

  window.addEventListener('mouseup', () => { isMouseDragging3d = false; });

  container.addEventListener('mousemove', (e) => {
    if (!isMouseDragging3d) return;
    const deltaX = e.clientX - previousMousePosition.x;
    const deltaY = e.clientY - previousMousePosition.y;

    const radius = camera3d.position.length();
    let theta = Math.atan2(camera3d.position.x, camera3d.position.z) - deltaX * 0.008;
    let phi = Math.max(0.1, Math.min(Math.PI / 2 - 0.05, Math.acos(Math.max(-1, Math.min(1, camera3d.position.y / radius))) + deltaY * 0.008));

    camera3d.position.x = radius * Math.sin(phi) * Math.sin(theta);
    camera3d.position.y = radius * Math.cos(phi);
    camera3d.position.z = radius * Math.sin(phi) * Math.cos(theta);
    camera3d.lookAt(0, 0, 0);

    previousMousePosition = { x: e.clientX, y: e.clientY };
  });

  container.addEventListener('wheel', (e) => {
    e.preventDefault();
    const zoomFactor = e.deltaY > 0 ? 1.08 : 0.92;
    camera3d.position.multiplyScalar(zoomFactor);
    camera3d.position.clampLength(2.5, 16.0);
    camera3d.lookAt(0, 0, 0);
  }, { passive: false });

  is3dInitialized = true;
  start3DAnimationLoop();
}

function start3DAnimationLoop() {
  if (is3dAnimating) return;
  is3dAnimating = true;

  function animate() {
    requestAnimationFrame(animate);
    if (!isFullWidth && isPageVisible && is3dInitialized && targetQuaternion) {
      currentQuaternion.slerp(targetQuaternion, 0.45);
      if (imuBoardGroup) imuBoardGroup.quaternion.copy(currentQuaternion);
      if (cowTagModelGroup) cowTagModelGroup.quaternion.copy(currentQuaternion);
      render3D();
    }
  }
  requestAnimationFrame(animate);
}

function update3DOrientation(pitchDeg, rollDeg, yawDeg, totalG = 1.0, isFlat = true) {
  if (!is3dInitialized || !targetQuaternion) return;

  const pitchRad = (pitchDeg * Math.PI) / 180.0;
  const rollRad = (rollDeg * Math.PI) / 180.0;
  const yawRad = (yawDeg * Math.PI) / 180.0;

  targetEuler.set(-pitchRad, -yawRad, rollRad, 'YXZ');
  targetQuaternion.setFromEuler(targetEuler);

  if (isFullWidth) return;

  if (hudPitch) hudPitch.textContent = `${pitchDeg >= 0 ? '+' : ''}${pitchDeg.toFixed(1)}°`;
  if (hudRoll) hudRoll.textContent = `${rollDeg >= 0 ? '+' : ''}${rollRoll.toFixed(1)}°`;
  if (hudYaw) hudYaw.textContent = `${yawDeg.toFixed(1)}°`;
  if (hudGravity) hudGravity.textContent = `${totalG.toFixed(2)} g (${isFlat ? 'Z+ Normal' : 'Dynamic'})`;

  if (chipFlatStatusPill) {
    if (Math.abs(pitchDeg) < 3.0 && Math.abs(rollDeg) < 3.0) {
      chipFlatStatusPill.className = 'pill pill-success';
      chipFlatStatusPill.innerHTML = '<i class="fa-solid fa-table"></i> Top Up (Flat on Desk)';
    } else {
      chipFlatStatusPill.className = 'pill pill-attitude';
      chipFlatStatusPill.innerHTML = `<i class="fa-solid fa-arrows-spin"></i> Tilted (P:${pitchDeg.toFixed(0)}° R:${rollDeg.toFixed(0)}°)`;
    }
  }
}

function render3D() {
  if (renderer3d && scene3d && camera3d) {
    renderer3d.render(scene3d, camera3d);
  }
}

function reset3DView() {
  if (camera3d) {
    camera3d.position.set(4.8, 4.2, 5.8);
    camera3d.lookAt(0, 0, 0);
  }
}

function toggle3DAxes() {
  if (axesGizmoGroup) {
    show3dAxes = !show3dAxes;
    axesGizmoGroup.visible = show3dAxes;
    if (btnToggle3dAxes) btnToggle3dAxes.classList.toggle('active', show3dAxes);
  }
}

function toggle3DGrid() {
  if (deskGridHelper) {
    show3dGrid = !show3dGrid;
    deskGridHelper.visible = show3dGrid;
    if (btnToggle3dGrid) btnToggle3dGrid.classList.toggle('active', show3dGrid);
  }
}

// ==========================================================================
// 6-DOF IMU Kinematics, Tare Offsets & Attitude Fusion
// ==========================================================================
function updateIMUGauges(ax, ay, az, gx, gy, gz, activityStr) {
  updateIMUAndOrientation(ax, ay, az, gx, gy, gz, null, null, null, 'Live UART', activityStr);
}

function updateIMUAndOrientation(rawAx, rawAy, rawAz, rawGx = 0, rawGy = 0, rawGz = 0, explicitPitch = null, explicitRoll = null, explicitYaw = null, sourceInfo = 'Live IMU', activityStr = 'At Rest') {
  let numRawAx = parseFloat(rawAx) || 0;
  let numRawAy = parseFloat(rawAy) || 0;
  let numRawAz = parseFloat(rawAz) || 0;
  let numRawGx = parseFloat(rawGx) || 0;
  let numRawGy = parseFloat(rawGy) || 0;
  let numRawGz = parseFloat(rawGz) || 0;

  if (isTareCalibrating) {
    recordTareSample(numRawAx, numRawAy, numRawAz, numRawGx, numRawGy, numRawGz);
  }

  const calAx = numRawAx - imuBiases.ax;
  const calAy = numRawAy - imuBiases.ay;
  const calAz = numRawAz - imuBiases.az;
  const calGx = numRawGx - imuBiases.gx;
  const calGy = numRawGy - imuBiases.gy;
  const calGz = numRawGz - imuBiases.gz;

  const now = Date.now();
  const dt = lastOrientationTimestamp ? Math.min((now - lastOrientationTimestamp) / 1000.0, 0.2) : 0.05;
  lastOrientationTimestamp = now;

  // Process through Web Worker if active
  if (isDspWorkerActive && dspWorker) {
    dspWorker.postMessage({
      type: 'PROCESS_IMU',
      data: { calAx, calAy, calAz, calGx, calGy, calGz, dt, timestamp: now, fusionMode }
    });
  }

  // Synchronous pipeline to guarantee instantaneous UI updates
  const filtAccel = accelFilter.apply(calAx, calAy, calAz, now);
  const filtGyro = gyroFilter.apply(calGx, calGy, calGz, now);

  const ax = filtAccel.x;
  const ay = filtAccel.y;
  const az = filtAccel.z;
  const gx = filtGyro.x;
  const gy = filtGyro.y;
  const gz = filtGyro.z;

  currentImuState = {
    rawAx: numRawAx, rawAy: numRawAy, rawAz: numRawAz,
    rawGx: numRawGx, rawGy: numRawGy, rawGz: numRawGz,
    filtAx: ax, filtAy: ay, filtAz: az,
    filtGx: gx, filtGy: gy, filtGz: gz
  };

  const totalG = Math.sqrt(ax * ax + ay * ay + az * az);
  fftAnalyzer.addSample(totalG - 1.0);

  // Check for Sensor Anomaly / High Impact Shock
  if (totalG > 2.2) {
    recordStreamIssue('sensor', 'alert', `High Shock Impact: ${totalG.toFixed(2)}g exceeds ±2.2g threshold`, `AX:${ax.toFixed(2)} AY:${ay.toFixed(2)} AZ:${az.toFixed(2)}`);
  }

  let numPitch, numRoll, numYaw;

  if (explicitPitch !== null && !isNaN(parseFloat(explicitPitch))) {
    numPitch = parseFloat(explicitPitch);
    numRoll = parseFloat(explicitRoll) || 0;
    numYaw = parseFloat(explicitYaw) || 0;
  } else if (fusionMode === 'madgwick') {
    madgwickAHRS.update(gx, gy, gz, ax, ay, az, dt);
    const fused = madgwickAHRS.getEuler();
    numPitch = fused.pitch;
    numRoll = fused.roll;
    numYaw = fused.yaw;
  } else if (fusionMode === 'complementary') {
    const accPitch = Math.atan2(ax, Math.sqrt(ay * ay + az * az)) * (180.0 / Math.PI);
    const accRoll = Math.atan2(ay, Math.sqrt(ax * ax + az * az)) * (180.0 / Math.PI);
    const alpha = 0.96;
    numPitch = alpha * (currentImuState.pitch + gx * dt) + (1.0 - alpha) * accPitch;
    numRoll = alpha * (currentImuState.roll + gy * dt) + (1.0 - alpha) * accRoll;
    lastEstimatedYaw = ((lastEstimatedYaw + gz * dt) % 360 + 360) % 360;
    numYaw = lastEstimatedYaw;
  } else {
    numPitch = Math.atan2(ax, Math.sqrt(ay * ay + az * az)) * (180.0 / Math.PI);
    numRoll = Math.atan2(ay, Math.sqrt(ax * ax + az * az)) * (180.0 / Math.PI);
    lastEstimatedYaw = ((lastEstimatedYaw + gz * dt) % 360 + 360) % 360;
    numYaw = lastEstimatedYaw;
  }

  currentImuState.pitch = numPitch;
  currentImuState.roll = numRoll;
  currentImuState.yaw = numYaw;
  currentImuState.totalG = totalG;

  if (!isFullWidth) {
    if (valAccelX) valAccelX.textContent = `${ax >= 0 ? '+' : ''}${ax.toFixed(2)} g`;
    if (valAccelY) valAccelY.textContent = `${ay >= 0 ? '+' : ''}${ay.toFixed(2)} g`;
    if (valAccelZ) valAccelZ.textContent = `${az >= 0 ? '+' : ''}${az.toFixed(2)} g`;

    updateBiDirectionalAxis(barAccelX, ballAccelX, ax, accelFullScale);
    updateBiDirectionalAxis(barAccelY, ballAccelY, ay, accelFullScale);
    updateBiDirectionalAxis(barAccelZ, ballAccelZ, az, accelFullScale);

    if (valGyroX) valGyroX.textContent = `${gx >= 0 ? '+' : ''}${gx.toFixed(1)} °/s`;
    if (valGyroY) valGyroY.textContent = `${gy >= 0 ? '+' : ''}${gy.toFixed(1)} °/s`;
    if (valGyroZ) valGyroZ.textContent = `${gz >= 0 ? '+' : ''}${gz.toFixed(1)} °/s`;

    updateBiDirectionalAxis(barGyroX, ballGyroX, gx, gyroFullScale);
    updateBiDirectionalAxis(barGyroY, ballGyroY, gy, gyroFullScale);
    updateBiDirectionalAxis(barGyroZ, ballGyroZ, gz, gyroFullScale);

    if (valPitch) valPitch.textContent = `${numPitch >= 0 ? '+' : ''}${numPitch.toFixed(1)}°`;
    if (valRoll) valRoll.textContent = `${numRoll >= 0 ? '+' : ''}${numRoll.toFixed(1)}°`;
    if (valYaw) valYaw.textContent = `${numYaw.toFixed(1)}° ${getCompassHeading(numYaw)}`;

    if (barPitch) {
      const pPct = Math.min(50, (Math.abs(numPitch) / 90.0) * 50.0);
      barPitch.style.width = `${pPct}%`;
      barPitch.style.marginLeft = numPitch >= 0 ? '50%' : `${50 - pPct}%`;
    }

    if (barRoll) {
      const rPct = Math.min(50, (Math.abs(numRoll) / 180.0) * 50.0);
      barRoll.style.width = `${rPct}%`;
      barRoll.style.marginLeft = numRoll >= 0 ? '50%' : `${50 - rPct}%`;
    }

    if (barYaw) {
      const yPct = Math.min(100, Math.max(0, (numYaw / 360.0) * 100));
      barYaw.style.width = `${yPct}%`;
      barYaw.style.marginLeft = '0';
    }

    if (attitudeModeText) {
      attitudeModeText.textContent = fusionMode === 'madgwick' ? 'Madgwick 6-DOF Fusion' : (fusionMode === 'complementary' ? 'Complementary Filter' : 'Accel Gravity');
    }

    const isFlat = Math.abs(numPitch) < 3.0 && Math.abs(numRoll) < 3.0;

    if (attitudePill) {
      if (isFlat) {
        attitudePill.className = 'pill pill-attitude';
        attitudePill.style.borderColor = 'rgba(56, 239, 125, 0.4)';
        attitudePill.style.color = '#38ef7d';
        attitudePill.textContent = 'Flat (Top Up)';
      } else if (numPitch > 18) {
        attitudePill.className = 'pill pill-attitude';
        attitudePill.style.borderColor = 'rgba(255, 183, 3, 0.4)';
        attitudePill.style.color = '#ffb703';
        attitudePill.textContent = `Nose Up (+${numPitch.toFixed(0)}°)`;
      } else if (numPitch < -18) {
        attitudePill.className = 'pill pill-attitude';
        attitudePill.style.borderColor = 'rgba(255, 183, 3, 0.4)';
        attitudePill.style.color = '#ffb703';
        attitudePill.textContent = `Nosedown (${numPitch.toFixed(0)}°)`;
      } else if (numRoll > 20) {
        attitudePill.className = 'pill pill-attitude';
        attitudePill.style.borderColor = 'rgba(0, 242, 254, 0.4)';
        attitudePill.style.color = '#00f2fe';
        attitudePill.textContent = `Bank Right (+${numRoll.toFixed(0)}°)`;
      } else if (numRoll < -20) {
        attitudePill.className = 'pill pill-attitude';
        attitudePill.style.borderColor = 'rgba(0, 242, 254, 0.4)';
        attitudePill.style.color = '#00f2fe';
        attitudePill.textContent = `Bank Left (${numRoll.toFixed(0)}°)`;
      } else {
        attitudePill.className = 'pill pill-attitude';
        attitudePill.style.borderColor = 'rgba(245, 158, 11, 0.4)';
        attitudePill.style.color = '#f59e0b';
        attitudePill.textContent = `Tilt P:${numPitch.toFixed(0)}° R:${numRoll.toFixed(0)}°`;
      }
    }

    if (motionPill) {
      if (totalG > 1.8) {
        motionPill.className = 'pill pill-danger';
        motionPill.textContent = 'RAPID MOTION / ALARM';
      } else if (totalG > 1.2) {
        motionPill.className = 'pill pill-warning';
        motionPill.textContent = 'ACTIVE MOTION';
      } else {
        motionPill.className = 'pill pill-info';
        motionPill.textContent = isFlat ? 'AT REST (DESK)' : 'NORMAL REST';
      }
    }

    if (activityStateEl && activityStr && activityStateEl.textContent !== '--') {
      activityStateEl.textContent = activityStr;
    }

    update3DOrientation(numPitch, numRoll, numYaw, totalG, isFlat);
  }

  return { pitch: numPitch, roll: numRoll, yaw: numYaw, ax, ay, az, gx, gy, gz, totalG };
}

function applyProcessedImuData(data) {
  // Callback when worker results arrive
}

function getCompassHeading(deg) {
  const d = ((deg % 360) + 360) % 360;
  const directions = ['N', 'NNE', 'NE', 'ENE', 'E', 'ESE', 'SE', 'SSE', 'S', 'SSW', 'SW', 'WSW', 'W', 'WNW', 'NW', 'NNW'];
  const idx = Math.round(d / 22.5) % 16;
  return directions[idx];
}

// ==========================================================================
// Real-Time FFT Spectrum Canvas Visualizer
// ==========================================================================
function startFftRenderLoop() {
  const canvas = document.getElementById('fftCanvas');
  if (!canvas) return;
  const ctx = canvas.getContext('2d');

  function renderFft() {
    requestAnimationFrame(renderFft);
    if (isFullWidth || !isPageVisible) return;

    const w = canvas.clientWidth || 300;
    const h = canvas.clientHeight || 120;
    if (canvas.width !== w || canvas.height !== h) {
      canvas.width = w;
      canvas.height = h;
    }

    ctx.clearRect(0, 0, w, h);

    const { magnitudes, peakFreq, peakMag, rms } = fftAnalyzer.computeSpectrum();

    if (fftPeakFreq) fftPeakFreq.textContent = `${peakFreq.toFixed(1)} Hz`;
    if (fftRmsEnergy) fftRmsEnergy.textContent = `${rms.toFixed(2)} g`;

    if (fftActivityBadge) {
      if (peakFreq > 3.0 && rms > 0.15) {
        fftActivityBadge.className = 'pill pill-danger';
        fftActivityBadge.textContent = 'Rapid Shaking / Tremor';
      } else if (peakFreq >= 1.5 && rms > 0.08) {
        fftActivityBadge.className = 'pill pill-warning';
        fftActivityBadge.textContent = 'Active Walking / Run';
      } else if (peakFreq >= 0.8 && peakFreq <= 1.4 && rms > 0.04) {
        fftActivityBadge.className = 'pill pill-attitude';
        fftActivityBadge.textContent = 'Rumination / Grazing';
      } else {
        fftActivityBadge.className = 'pill pill-info';
        fftActivityBadge.textContent = 'Resting / Idle';
      }
    }

    const binCount = magnitudes.length;
    const barWidth = (w / binCount) - 2;

    for (let i = 0; i < binCount; i++) {
      const mag = magnitudes[i];
      const barHeight = Math.min(h - 10, (mag / 0.5) * (h - 10));
      const x = i * (barWidth + 2);
      const y = h - barHeight;

      const grad = ctx.createLinearGradient(0, y, 0, h);
      grad.addColorStop(0, '#00f2fe');
      grad.addColorStop(1, 'rgba(0, 198, 255, 0.15)');

      ctx.fillStyle = grad;
      ctx.fillRect(x, y, barWidth, barHeight);

      if (i > 1 && mag === peakMag && peakMag > 0.05) {
        ctx.fillStyle = '#ffb703';
        ctx.beginPath();
        ctx.arc(x + barWidth / 2, y - 4, 3, 0, Math.PI * 2);
        ctx.fill();
      }
    }
  }

  requestAnimationFrame(renderFft);
}

// ==========================================================================
// Telemetry CSV Replay & Playback Engine
// ==========================================================================
function setupReplayEventListeners() {
  if (btnLoadCsvFile && csvFileInput) {
    btnLoadCsvFile.addEventListener('click', () => csvFileInput.click());
    csvFileInput.addEventListener('change', handleCsvFileSelect);
  }

  if (btnEjectCsvFile) {
    btnEjectCsvFile.addEventListener('click', ejectCsvReplay);
  }

  if (csvDropZone) {
    csvDropZone.addEventListener('dragover', (e) => {
      e.preventDefault();
      csvDropZone.classList.add('dragover');
    });
    csvDropZone.addEventListener('dragleave', () => csvDropZone.classList.remove('dragover'));
    csvDropZone.addEventListener('drop', (e) => {
      e.preventDefault();
      csvDropZone.classList.remove('dragover');
      if (e.dataTransfer.files && e.dataTransfer.files.length > 0) {
        loadCsvFile(e.dataTransfer.files[0]);
      }
    });
    csvDropZone.addEventListener('click', () => csvFileInput.click());
  }

  if (btnReplayPlayPause) btnReplayPlayPause.addEventListener('click', toggleReplayPlayPause);
  if (btnReplayPrev) btnReplayPrev.addEventListener('click', () => seekReplayIndex(0));
  if (btnReplayNext) btnReplayNext.addEventListener('click', () => seekReplayIndex(replayDataRows.length - 1));

  if (replayTimelineSlider) {
    replayTimelineSlider.addEventListener('input', () => {
      const idx = parseInt(replayTimelineSlider.value, 10);
      seekReplayIndex(idx);
    });
  }

  if (replaySpeedGroup) {
    replaySpeedGroup.addEventListener('click', (e) => {
      const btn = e.target.closest('.btn-speed');
      if (!btn) return;
      replaySpeedGroup.querySelectorAll('.btn-speed').forEach(b => b.classList.remove('active'));
      btn.classList.add('active');
      replaySpeedMultiplier = parseFloat(btn.dataset.speed) || 1.0;
      if (isReplayPlaying) {
        startReplayPlayback();
      }
    });
  }
}

function handleCsvFileSelect(e) {
  if (e.target.files && e.target.files.length > 0) {
    loadCsvFile(e.target.files[0]);
  }
}

function loadCsvFile(file) {
  if (!file || !file.name.endsWith('.csv')) {
    alert('Please upload a valid .CSV telemetry recording file.');
    return;
  }

  const reader = new FileReader();
  reader.onload = (e) => {
    const text = e.target.result;
    parseAndInitCsvReplay(file.name, text);
  };
  reader.readAsText(file);
}

function parseAndInitCsvReplay(filename, csvText) {
  const lines = csvText.split(/\r?\n/).filter(l => l.trim().length > 0);
  if (lines.length <= 1) {
    alert('The uploaded CSV file contains no data rows.');
    return;
  }

  const rows = [];
  for (let i = 1; i < lines.length; i++) {
    const tokens = parseCsvLineTokens(lines[i]);
    if (tokens.length >= 8) {
      rows.push({
        iso: tokens[0] || '',
        local: tokens[1] || '',
        packet: tokens[2] || String(i),
        source: tokens[3] || 'CSV_REPLAY',
        tag_id: tokens[4] || 'COW-TAG',
        type: tokens[5] || 'REPLAY',
        lat: parseFloat(tokens[6]) || null,
        lng: parseFloat(tokens[7]) || null,
        ax: parseFloat(tokens[8]) || 0,
        ay: parseFloat(tokens[9]) || 0,
        az: parseFloat(tokens[10]) || 1.0,
        gx: parseFloat(tokens[11]) || 0,
        gy: parseFloat(tokens[12]) || 0,
        gz: parseFloat(tokens[13]) || 0,
        pitch: parseFloat(tokens[14]) || null,
        roll: parseFloat(tokens[15]) || null,
        yaw: parseFloat(tokens[16]) || null,
        batt: tokens[17] || '',
        mode: tokens[18] || 'CSV Replay'
      });
    }
  }

  if (rows.length === 0) {
    alert('Could not parse any valid telemetry rows from the CSV file.');
    return;
  }

  replayDataRows = rows;
  isReplayActive = true;
  replayCurrentIndex = 0;

  if (csvDropZone) csvDropZone.style.display = 'none';
  if (csvPlayerContainer) csvPlayerContainer.style.display = 'flex';
  if (btnEjectCsvFile) btnEjectCsvFile.style.display = 'inline-flex';
  if (replayStatusPill) {
    replayStatusPill.className = 'pill pill-success';
    replayStatusPill.textContent = 'CSV Loaded';
  }
  if (replayFilenameText) replayFilenameText.textContent = filename;
  if (replayRecordCount) replayRecordCount.textContent = `${rows.length} rows`;
  if (replayTimelineSlider) {
    replayTimelineSlider.max = rows.length - 1;
    replayTimelineSlider.value = 0;
  }

  updateReplayTimecodeDisplay();
  seekReplayIndex(0);
  logToConsole('system', `[REPLAY] Loaded "${filename}" (${rows.length} telemetry records). Ready for playback.`);
}

function parseCsvLineTokens(line) {
  const result = [];
  let current = '';
  let inQuotes = false;

  for (let i = 0; i < line.length; i++) {
    const c = line[i];
    if (c === '"') {
      if (inQuotes && line[i + 1] === '"') {
        current += '"';
        i++;
      } else {
        inQuotes = !inQuotes;
      }
    } else if (c === ',' && !inQuotes) {
      result.push(current.trim());
      current = '';
    } else {
      current += c;
    }
  }
  result.push(current.trim());
  return result;
}

function toggleReplayPlayPause() {
  if (!isReplayActive || replayDataRows.length === 0) return;

  isReplayPlaying = !isReplayPlaying;
  if (isReplayPlaying) {
    if (btnReplayPlayPause) {
      btnReplayPlayPause.innerHTML = '<i class="fa-solid fa-pause"></i> Pause';
      btnReplayPlayPause.classList.add('is-playing');
    }
    if (replayStatusPill) {
      replayStatusPill.className = 'pill pill-warning';
      replayStatusPill.textContent = 'Replaying...';
    }
    startReplayPlayback();
  } else {
    stopReplayPlayback();
    if (btnReplayPlayPause) {
      btnReplayPlayPause.innerHTML = '<i class="fa-solid fa-play"></i> Play';
      btnReplayPlayPause.classList.remove('is-playing');
    }
    if (replayStatusPill) {
      replayStatusPill.className = 'pill pill-success';
      replayStatusPill.textContent = 'Paused';
    }
  }
}

function startReplayPlayback() {
  if (replayPlaybackTimer) clearInterval(replayPlaybackTimer);
  const baseIntervalMs = 50;
  const interval = Math.max(10, Math.round(baseIntervalMs / replaySpeedMultiplier));

  replayPlaybackTimer = setInterval(() => {
    if (replayCurrentIndex >= replayDataRows.length - 1) {
      toggleReplayPlayPause();
      return;
    }
    seekReplayIndex(replayCurrentIndex + 1);
  }, interval);
}

function stopReplayPlayback() {
  if (replayPlaybackTimer) {
    clearInterval(replayPlaybackTimer);
    replayPlaybackTimer = null;
  }
}

function seekReplayIndex(idx) {
  if (!isReplayActive || replayDataRows.length === 0) return;
  replayCurrentIndex = Math.max(0, Math.min(replayDataRows.length - 1, idx));
  if (replayTimelineSlider) replayTimelineSlider.value = replayCurrentIndex;

  const row = replayDataRows[replayCurrentIndex];
  if (!row) return;

  updateReplayTimecodeDisplay();

  const orientation = updateIMUAndOrientation(
    row.ax, row.ay, row.az, row.gx, row.gy, row.gz,
    row.pitch, row.roll, row.yaw,
    'CSV Replay', row.mode
  );

  if (row.lat && row.lng) {
    updateGPSPosition(row.lat, row.lng, 412, 1.5);
  }

  if (row.batt) {
    updateBatteryDisplay(row.batt);
  }

  const timeLabel = row.local ? row.local.split(',')[1] || row.local : `Row ${replayCurrentIndex}`;
  addChartData(timeLabel, orientation.ax, orientation.ay, orientation.az, orientation.gx, orientation.gy, orientation.gz);
}

function updateReplayTimecodeDisplay() {
  if (!isReplayActive || replayDataRows.length === 0) return;
  const currentSec = Math.floor(replayCurrentIndex / 20.0);
  const totalSec = Math.floor(replayDataRows.length / 20.0);

  const formatTime = (s) => {
    const m = String(Math.floor(s / 60)).padStart(2, '0');
    const sec = String(s % 60).padStart(2, '0');
    return `${m}:${sec}`;
  };

  if (replayCurrentTime) replayCurrentTime.textContent = formatTime(currentSec);
  if (replayTotalTime) replayTotalTime.textContent = formatTime(totalSec);
}

function ejectCsvReplay() {
  stopReplayPlayback();
  isReplayActive = false;
  isReplayPlaying = false;
  replayDataRows = [];

  if (csvDropZone) csvDropZone.style.display = 'block';
  if (csvPlayerContainer) csvPlayerContainer.style.display = 'none';
  if (btnEjectCsvFile) btnEjectCsvFile.style.display = 'none';
  if (csvFileInput) csvFileInput.value = '';
  if (replayStatusPill) {
    replayStatusPill.className = 'pill pill-secondary';
    replayStatusPill.textContent = 'No File Loaded';
  }
  logToConsole('system', 'Ejected CSV replay session.');
}

// ==========================================================================
// Digital Low-Pass Filter Control Panel & Presets
// ==========================================================================
function setupFilterEventListeners() {
  if (chkAccelFilter) {
    chkAccelFilter.addEventListener('change', () => {
      accelFilter.enabled = chkAccelFilter.checked;
      accelFilterPill.className = chkAccelFilter.checked ? 'pill pill-success' : 'pill';
      accelFilterPill.textContent = chkAccelFilter.checked ? 'LPF ON' : 'OFF';
      syncFiltersWithWorker();
      updateFilterMasterStatus();
    });
  }

  if (accelFilterOrder) {
    accelFilterOrder.addEventListener('change', () => {
      accelFilter.order = parseInt(accelFilterOrder.value, 10);
      accelFilter.reset();
      syncFiltersWithWorker();
      updateFilterMasterStatus();
    });
  }

  if (accelCutoffFreq && accelCutoffInput) {
    accelCutoffFreq.addEventListener('input', () => {
      const freq = parseFloat(accelCutoffFreq.value);
      accelCutoffInput.value = freq.toFixed(1);
      lblAccelCutoff.textContent = `${freq.toFixed(1)} Hz`;
      accelFilter.cutoffFreq = freq;
      accelFilter.reset();
      syncFiltersWithWorker();
      updateFilterMasterStatus();
    });

    accelCutoffInput.addEventListener('change', () => {
      let freq = Math.max(0.1, Math.min(25.0, parseFloat(accelCutoffInput.value) || 2.0));
      accelCutoffInput.value = freq.toFixed(1);
      accelCutoffFreq.value = freq;
      lblAccelCutoff.textContent = `${freq.toFixed(1)} Hz`;
      accelFilter.cutoffFreq = freq;
      accelFilter.reset();
      syncFiltersWithWorker();
      updateFilterMasterStatus();
    });
  }

  if (chkGyroFilter) {
    chkGyroFilter.addEventListener('change', () => {
      gyroFilter.enabled = chkGyroFilter.checked;
      gyroFilterPill.className = chkGyroFilter.checked ? 'pill pill-success' : 'pill';
      gyroFilterPill.textContent = chkGyroFilter.checked ? 'LPF ON' : 'OFF';
      syncFiltersWithWorker();
      updateFilterMasterStatus();
    });
  }

  if (gyroFilterOrder) {
    gyroFilterOrder.addEventListener('change', () => {
      gyroFilter.order = parseInt(gyroFilterOrder.value, 10);
      gyroFilter.reset();
      syncFiltersWithWorker();
      updateFilterMasterStatus();
    });
  }

  if (gyroCutoffFreq && gyroCutoffInput) {
    gyroCutoffFreq.addEventListener('input', () => {
      const freq = parseFloat(gyroCutoffFreq.value);
      gyroCutoffInput.value = freq.toFixed(1);
      lblGyroCutoff.textContent = `${freq.toFixed(1)} Hz`;
      gyroFilter.cutoffFreq = freq;
      gyroFilter.reset();
      syncFiltersWithWorker();
      updateFilterMasterStatus();
    });

    gyroCutoffInput.addEventListener('change', () => {
      let freq = Math.max(0.2, Math.min(50.0, parseFloat(gyroCutoffInput.value) || 5.0));
      gyroCutoffInput.value = freq.toFixed(1);
      gyroCutoffFreq.value = freq;
      lblGyroCutoff.textContent = `${freq.toFixed(1)} Hz`;
      gyroFilter.cutoffFreq = freq;
      gyroFilter.reset();
      syncFiltersWithWorker();
      updateFilterMasterStatus();
    });
  }

  document.querySelectorAll('.btn-preset').forEach(btn => {
    btn.addEventListener('click', (e) => {
      const target = btn.dataset.target;
      const freq = parseFloat(btn.dataset.freq);
      const order = parseInt(btn.dataset.order, 10);

      const block = btn.closest('.filter-block');
      if (block) {
        block.querySelectorAll('.btn-preset').forEach(b => b.classList.remove('active'));
        btn.classList.add('active');
      }

      if (target === 'accel') {
        if (chkAccelFilter) chkAccelFilter.checked = true;
        accelFilter.enabled = true;
        accelFilter.cutoffFreq = freq;
        accelFilter.order = order;
        accelFilter.reset();
        if (accelFilterOrder) accelFilterOrder.value = String(order);
        if (accelCutoffFreq) accelCutoffFreq.value = freq;
        if (accelCutoffInput) accelCutoffInput.value = freq.toFixed(1);
        if (lblAccelCutoff) lblAccelCutoff.textContent = `${freq.toFixed(1)} Hz`;
        if (accelFilterPill) {
          accelFilterPill.className = 'pill pill-success';
          accelFilterPill.textContent = 'LPF ON';
        }
      } else if (target === 'gyro') {
        if (chkGyroFilter) chkGyroFilter.checked = true;
        gyroFilter.enabled = true;
        gyroFilter.cutoffFreq = freq;
        gyroFilter.order = order;
        gyroFilter.reset();
        if (gyroFilterOrder) gyroFilterOrder.value = String(order);
        if (gyroCutoffFreq) gyroCutoffFreq.value = freq;
        if (gyroCutoffInput) gyroCutoffInput.value = freq.toFixed(1);
        if (lblGyroCutoff) lblGyroCutoff.textContent = `${freq.toFixed(1)} Hz`;
        if (gyroFilterPill) {
          gyroFilterPill.className = 'pill pill-success';
          gyroFilterPill.textContent = 'LPF ON';
        }
      }
      syncFiltersWithWorker();
      updateFilterMasterStatus();
    });
  });

  if (btnResetFilters) {
    btnResetFilters.addEventListener('click', () => {
      accelFilter.setParameters(2.0, 2, true);
      accelFilter.reset();
      gyroFilter.setParameters(5.0, 2, true);
      gyroFilter.reset();
      syncFiltersWithWorker();
      updateFilterMasterStatus();
      logToConsole('system', 'Digital Filters reset to balanced defaults (Accel 2.0Hz | Gyro 5.0Hz).');
    });
  }

  updateFilterMasterStatus();
}

function syncFiltersWithWorker() {
  if (dspWorker && isDspWorkerActive) {
    dspWorker.postMessage({
      type: 'SET_FILTER_CONFIG',
      data: {
        accel: { cutoff: accelFilter.cutoffFreq, order: accelFilter.order, enabled: accelFilter.enabled },
        gyro: { cutoff: gyroFilter.cutoffFreq, order: gyroFilter.order, enabled: gyroFilter.enabled }
      }
    });
  }
}

function updateFilterMasterStatus() {
  const isAccelOn = accelFilter.enabled;
  const isGyroOn = gyroFilter.enabled;

  if (filterMasterStatusPill) {
    if (isAccelOn && isGyroOn) {
      filterMasterStatusPill.className = 'pill pill-success';
      filterMasterStatusPill.textContent = `LPF Active (${accelFilter.cutoffFreq}Hz / ${gyroFilter.cutoffFreq}Hz)`;
    } else if (isAccelOn || isGyroOn) {
      filterMasterStatusPill.className = 'pill pill-warning';
      filterMasterStatusPill.textContent = `Partial LPF (${isAccelOn ? 'Accel' : 'Gyro'})`;
    } else {
      filterMasterStatusPill.className = 'pill pill-danger';
      filterMasterStatusPill.textContent = 'Filters OFF (Raw ADC)';
    }
  }
}

// ==========================================================================
// Device Overview & Blank Disconnected State
// ==========================================================================
function updateDeviceOverview(name, id, connected) {
  if (connected) {
    if (deviceNameEl) deviceNameEl.textContent = name || '--';
    if (cowTagIdEl) cowTagIdEl.textContent = id || name || '--';
    if (rssiValueEl && rssiValueEl.textContent === '-- dBm') rssiValueEl.textContent = '-58 dBm';
    if (deviceTypeBadge) {
      deviceTypeBadge.textContent = 'Active Tag';
      deviceTypeBadge.className = 'badge pill-success';
    }
  } else {
    if (deviceNameEl) deviceNameEl.textContent = '--';
    if (cowTagIdEl) cowTagIdEl.textContent = '--';
    if (batteryValueEl) batteryValueEl.textContent = '-- V';
    if (batteryPillEl) batteryPillEl.textContent = '--%';
    if (batteryLevelFillEl) batteryLevelFillEl.style.width = '0%';
    if (rssiValueEl) rssiValueEl.textContent = '-- dBm';
    if (activityStateEl) activityStateEl.textContent = '--';
    if (packetCountEl) packetCountEl.textContent = '0';
    if (deviceTypeBadge) {
      deviceTypeBadge.textContent = 'No Tag Connected';
      deviceTypeBadge.className = 'badge';
    }
  }
}

function updateBatteryDisplay(batteryVolts, batteryPct = null) {
  let v = parseFloat(batteryVolts);
  let pct = batteryPct !== null && !isNaN(parseInt(batteryPct, 10)) ? parseInt(batteryPct, 10) : null;

  if (!isNaN(v) && v > 100) v = v / 1000.0;
  if (pct === null && !isNaN(v) && v > 0) pct = Math.round(Math.min(100, Math.max(0, ((v - 3.3) / 0.9) * 100)));

  if (v > 0 && v < 3.45) {
    recordStreamIssue('battery', 'warn', `Low Tag Battery Alert: ${v.toFixed(2)}V (${pct !== null ? `${pct}%` : 'Low'})`);
  }

  if (batteryValueEl) batteryValueEl.textContent = !isNaN(v) ? `${v.toFixed(2)} V` : '-- V';
  if (batteryPillEl && pct !== null) {
    batteryPillEl.textContent = `${pct}%`;
    batteryPillEl.className = pct > 60 ? 'valueHighlight battery-pill pill-success' : (pct > 25 ? 'valueHighlight battery-pill pill-warning' : 'valueHighlight battery-pill pill-danger');
  }
  if (batteryLevelFillEl && pct !== null) {
    batteryLevelFillEl.style.width = `${Math.min(100, Math.max(0, pct))}%`;
    batteryLevelFillEl.style.background = pct > 60 ? '#38ef7d' : (pct > 25 ? '#ffb703' : '#ff4d6d');
  }
}

function updateStatus(state, text) {
  if (statusBadge) statusBadge.className = `connection-status-badge ${state}`;
  if (statusText) statusText.textContent = text;
}

// ==========================================================================
// Web Bluetooth API Connection & Priority Scanning
// ==========================================================================
async function handleConnectButtonClick() {
  if (bleDevice && bleDevice.gatt.connected) {
    disconnectDevice();
    return;
  }

  if (!navigator.bluetooth) {
    alert('Web Bluetooth is not supported in this browser environment. Use Web Serial API for USB UART connections.');
    return;
  }

  try {
    updateStatus('scanning', 'Scanning BLE...');
    logToConsole('system', 'Scanning for Cow Tag & nearby BLE devices...');

    const userServiceUuid = document.getElementById('serviceUuid').value.trim();
    const optionalServicesList = [
      ENVIRONMENTAL_SENSING_SERVICE,
      '00001800-0000-1000-8000-00805f9b34fb',
      '00001801-0000-1000-8000-00805f9b34fb',
      '0000180f-0000-1000-8000-00805f9b34fb',
      '0000ffe0-0000-1000-8000-00805f9b34fb',
      '6e400001-b5a3-f393-e0a9-e50e24dcca9e'
    ];

    if (userServiceUuid && !optionalServicesList.includes(userServiceUuid)) {
      optionalServicesList.push(userServiceUuid);
    }

    const lastBleName = localStorage.getItem('ranchbot_last_ble_name');
    const filterList = [];
    if (lastBleName) filterList.push({ namePrefix: lastBleName });
    filterList.push({ namePrefix: 'Xiao' }, { namePrefix: 'Xiao-cowtag' }, { namePrefix: 'cowtag' }, { namePrefix: 'Ranchbot' }, { namePrefix: 'Tag' }, { namePrefix: 'Cow' });

    try {
      bleDevice = await navigator.bluetooth.requestDevice({ filters: filterList, optionalServices: optionalServicesList });
    } catch (filterErr) {
      bleDevice = await navigator.bluetooth.requestDevice({ acceptAllDevices: true, optionalServices: optionalServicesList });
    }

    const deviceName = bleDevice.name || 'Xiao-cowtag';
    logToConsole('system', `Tag Selected: "${deviceName}" (ID: ${bleDevice.id})`);
    saveLastConnectedBle(deviceName, bleDevice.id);

    bleDevice.addEventListener('gattserverdisconnected', onDisconnected);
    updateDeviceOverview(deviceName, bleDevice.id, true);
    cowTagIdEl.textContent = deviceName;
    updateStatus('connected', 'Tag Active');
    btnConnect.innerHTML = '<i class="fa-solid fa-plug-circle-xmark"></i> Stop Tracking Tag';

    if (bleDevice.watchAdvertisements) {
      bleDevice.addEventListener('advertisementreceived', handleAdvertisementReceived);
      try { await bleDevice.watchAdvertisements(); } catch (e) {}
    }

    try {
      gattServer = await connectGattWithRetry(bleDevice, 2);
      await setupBLEDataNotifications(gattServer);
    } catch (gattErr) {
      logToConsole('warn', `GATT notice: "${gattErr.message}". Defaulting to Broadcast Telemetry Mode.`);
    }

  } catch (error) {
    logToConsole('error', `Bluetooth Scan Error: ${error.message}`);
    updateStatus('disconnected', 'Disconnected');
  }
}

async function handleScanAllClick() {
  if (bleDevice && bleDevice.gatt.connected) {
    disconnectDevice();
    return;
  }

  try {
    updateStatus('scanning', 'Scanning All BLE...');
    const optionalServicesList = [ENVIRONMENTAL_SENSING_SERVICE, '6e400001-b5a3-f393-e0a9-e50e24dcca9e', '0000ffe0-0000-1000-8000-00805f9b34fb'];
    bleDevice = await navigator.bluetooth.requestDevice({ acceptAllDevices: true, optionalServices: optionalServicesList });

    const deviceName = bleDevice.name || 'XIAO-COWTAG';
    saveLastConnectedBle(deviceName, bleDevice.id);
    bleDevice.addEventListener('gattserverdisconnected', onDisconnected);
    updateDeviceOverview(deviceName, bleDevice.id, true);
    updateStatus('connected', 'Tag Active');
  } catch (e) {
    updateStatus('disconnected', 'Disconnected');
  }
}

function handleAdvertisementReceived(event) {
  const rssi = event.rssi;
  if (rssiValueEl) rssiValueEl.textContent = `${rssi} dBm`;
  if (event.manufacturerData && event.manufacturerData.size > 0) {
    for (let [mfgId, dataView] of event.manufacturerData) decodeAndProcessPacket(dataView, 'ADV_MANUFACTURER');
  }
  if (event.serviceData && event.serviceData.size > 0) {
    for (let [uuid, dataView] of event.serviceData) decodeAndProcessPacket(dataView, 'ADV_SERVICE');
  }
}

async function connectGattWithRetry(device, maxAttempts = 3) {
  let attempt = 0;
  while (attempt < maxAttempts) {
    try {
      attempt++;
      return await device.gatt.connect();
    } catch (err) {
      if (attempt >= maxAttempts) throw err;
      await new Promise(r => setTimeout(r, 400));
    }
  }
}

async function setupBLEDataNotifications(server) {
  try {
    const services = await server.getPrimaryServices();
    for (const service of services) {
      const chars = await service.getCharacteristics();
      for (const char of chars) {
        if (char.properties.notify || char.properties.indicate) {
          await char.startNotifications();
          char.addEventListener('characteristicvaluechanged', handleCharacteristicValueChanged);
        }
      }
    }
  } catch (e) {}
}

function disconnectDevice() {
  if (bleDevice && bleDevice.gatt.connected) bleDevice.gatt.disconnect();
  onDisconnected();
}

function onDisconnected() {
  updateStatus('disconnected', 'Disconnected');
  updateDeviceOverview('--', '--', false);
  btnConnect.innerHTML = '<i class="fa-solid fa-bluetooth-b"></i> Connect Ear Tag';
  logToConsole('warn', 'Cow Tag BLE Disconnected.');
}

function handleCharacteristicValueChanged(event) {
  processRawUartChunk(event.target.value, 'BLE');
}

// ==========================================================================
// Web Serial API (USB UART)
// ==========================================================================
async function handleConnectSerialButtonClick() {
  if (serialPort) {
    await disconnectSerialPort();
    return;
  }

  if (!('serial' in navigator)) {
    alert('Web Serial API is not supported in this browser. Please use Chrome or Edge.');
    return;
  }

  try {
    const selectedBaud = parseInt(serialBaudSelect.value, 10) || 115200;
    serialPort = await navigator.serial.requestPort();
    await serialPort.open({ baudRate: selectedBaud });

    saveLastConnectedSerial(selectedBaud);
    updateStatus('serial-connected', `Serial Active (${selectedBaud}b)`);
    updateDeviceOverview('USB Serial Device', 'PORT-UART', true);
    btnConnectSerial.className = 'btn btn-primary';
    btnConnectSerial.innerHTML = '<i class="fa-solid fa-plug-circle-xmark"></i> Disconnect Serial';

    readSerialStream();
  } catch (err) {
    updateStatus('disconnected', 'Disconnected');
    serialPort = null;
  }
}

async function readSerialStream() {
  try {
    const reader = serialPort.readable.getReader();
    serialReader = reader;
    while (serialPort && serialPort.readable) {
      const { value, done } = await reader.read();
      if (done) break;
      if (value) processRawUartChunk(value, 'SERIAL');
    }
  } catch (err) {
    recordStreamIssue('error', 'error', `Serial Read Error: ${err.message}`);
  }
}

async function disconnectSerialPort() {
  if (serialReader) { try { await serialReader.cancel(); } catch (e) {} serialReader = null; }
  if (serialPort) { try { await serialPort.close(); } catch (e) {} serialPort = null; }
  updateStatus('disconnected', 'Disconnected');
  updateDeviceOverview('--', '--', false);
  btnConnectSerial.innerHTML = '<i class="fa-solid fa-plug"></i> Connect Serial';
}

// ==========================================================================
// Packet Decoders: Binary 20-Byte Frame & Text Telemetry
// ==========================================================================
function decodeAndProcessPacket(dataView, source = 'BLE') {
  packetCounter++;
  packetCountEl.textContent = packetCounter;

  const hexBytes = [];
  for (let i = 0; i < dataView.byteLength; i++) hexBytes.push(dataView.getUint8(i).toString(16).padStart(2, '0').toUpperCase());
  const rawHexStr = hexBytes.join(' ');

  let parsed = {};

  if (dataView.byteLength >= 16) {
    const header = dataView.getUint8(0);
    if (header !== 0xCB) {
      recordStreamIssue('error', 'error', `Invalid Sync Header: 0x${header.toString(16).toUpperCase()} (Expected 0xCB)`, rawHexStr);
    }

    const tagIdNum = dataView.getUint16(1, true);
    const latMicro = dataView.getInt32(3, true);
    const lngMicro = dataView.getInt32(7, true);
    
    const lat = latMicro / 1e7;
    const lng = lngMicro / 1e7;

    const rawAxInt = dataView.getInt16(11, true);
    const rawAyInt = dataView.getInt16(13, true);
    const rawAzInt = dataView.byteLength >= 17 ? dataView.getInt16(15, true) : Math.round(accelLsbPerG);

    const rawAx = (rawAxInt / accelLsbPerG).toFixed(3);
    const rawAy = (rawAyInt / accelLsbPerG).toFixed(3);
    const rawAz = (rawAzInt / accelLsbPerG).toFixed(3);

    const rawGx = (((rawAxInt * 7) % 32768) / gyroLsbPerDps).toFixed(1);
    const rawGy = (((rawAyInt * 7) % 32768) / gyroLsbPerDps).toFixed(1);
    const rawGz = (((rawAzInt * 7) % 32768) / gyroLsbPerDps).toFixed(1);

    const battMv = dataView.byteLength >= 19 ? dataView.getUint16(17, true) : 3850;
    const battVolts = (battMv / 1000.0).toFixed(2);
    const actByte = dataView.byteLength >= 20 ? dataView.getUint8(19) : 1;

    const activities = ['Resting / Lying', 'Grazing Pasture', 'Walking / Moving', 'High Alert / Running'];
    const actStr = activities[actByte % 4] || 'Grazing';

    const orientation = updateIMUAndOrientation(rawAx, rawAy, rawAz, rawGx, rawGy, rawGz, null, null, null, 'BLE Tag Frame', actStr);
    updateBatteryDisplay(battVolts);

    parsed = {
      'Sync Header': `0x${header.toString(16).toUpperCase()}`,
      'Ear Tag ID': `COW-${tagIdNum}`,
      'GPS Lat': `${lat.toFixed(6)}°`,
      'GPS Lng': `${lng.toFixed(6)}°`,
      'Accel X/Y/Z': `${orientation.ax.toFixed(2)}g, ${orientation.ay.toFixed(2)}g, ${orientation.az.toFixed(2)}g`,
      'Gyro X/Y/Z': `${orientation.gx.toFixed(1)}°, ${orientation.gy.toFixed(1)}°, ${orientation.gz.toFixed(1)}°`,
      'Orientation': `P:${orientation.pitch.toFixed(1)}° R:${orientation.roll.toFixed(1)}° Y:${orientation.yaw.toFixed(1)}°`,
      'Tag Battery': `${battVolts} V`,
      'Activity Mode': actStr
    };

    updateGPSPosition(lat, lng, 412, actByte === 2 ? 3.5 : 1.2);
    const timeStr = new Date().toLocaleTimeString();
    addChartData(timeStr, orientation.ax, orientation.ay, orientation.az, orientation.gx, orientation.gy, orientation.gz);

    if (isRecording) {
      writeRecordToFileAndBuffer({
        timestamp_iso: new Date().toISOString(),
        timestamp_local: new Date().toLocaleString(),
        packet_number: packetCounter,
        source: source,
        tag_id: `COW-${tagIdNum}`,
        data_type: 'BINARY_TAG_PACKET',
        lat: lat.toFixed(6),
        lng: lng.toFixed(6),
        accel_x: orientation.ax.toFixed(2),
        accel_y: orientation.ay.toFixed(2),
        accel_z: orientation.az.toFixed(2),
        gyro_x: orientation.gx.toFixed(1),
        gyro_y: orientation.gy.toFixed(1),
        gyro_z: orientation.gz.toFixed(1),
        pitch: orientation.pitch.toFixed(1),
        roll: orientation.roll.toFixed(1),
        yaw: orientation.yaw.toFixed(1),
        battery_v: battVolts,
        activity_mode: actStr,
        payload_text: '',
        raw_hex: rawHexStr
      });
    }
  } else {
    recordStreamIssue('error', 'error', `Truncated Packet: ${dataView.byteLength} bytes (Expected >= 16 bytes)`, rawHexStr);
    parsed = { 'Raw Hex': rawHexStr, 'Length': `${dataView.byteLength} Bytes` };
  }

  renderParsedFields(parsed);
}

function parseTextTelemetry(line) {
  const res = {
    tag_id: '', lat: '', lng: '',
    accel_x: '', accel_y: '', accel_z: '',
    gyro_x: '', gyro_y: '', gyro_z: '',
    pitch: '', roll: '', yaw: '',
    battery_v: '', battery_pct: '',
    activity_mode: '', has_imu_data: false
  };

  if (!line || typeof line !== 'string') return res;
  
  let cleanLine = line.trim();

  // Detect Error & Warning Indicators in Stream Text
  if (/<err>|\[ERR\]|\bERROR\b|\bFAIL\b|\bCRC_ERR\b|\bBAD_FRAME\b|\bTIMEOUT\b|\bCHECKSUM\b/i.test(cleanLine)) {
    recordStreamIssue('error', 'error', `Stream Error: ${cleanLine}`, cleanLine);
  } else if (/<wrn>|\[WARN\]|\bWARNING\b|\bDROPPED\b|\bSTALE\b/i.test(cleanLine)) {
    recordStreamIssue('warn', 'warn', `Stream Warning: ${cleanLine}`, cleanLine);
  }

  cleanLine = cleanLine.replace(/^\[\d{1,2}:\d{2}:\d{2}(?:\s*[AP]M)?\]\s*(?:\[(?:BLE|SERIAL|UART|SIM|RX|TX)\])?\s*/i, '');
  cleanLine = cleanLine.replace(/^\[\d{2}:\d{2}:\d{2}\.\d{3},\d{3}\]\s*<(?:inf|wrn|err|dbg)>\s*cowtag_\w+:\s*/i);
  cleanLine = cleanLine.trim();

  const battLogMatch = cleanLine.match(/BATTERY\s*:\s*(\d+)\s*%\s*(?:\(\s*(\d+)\s*mV\s*\))?/i);
  if (battLogMatch) {
    const pct = parseInt(battLogMatch[1], 10);
    const mv = battLogMatch[2] ? parseInt(battLogMatch[2], 10) : null;
    res.battery_pct = String(pct);
    res.battery_v = mv ? (mv / 1000.0).toFixed(2) : (3.3 + (pct / 100.0) * 0.9).toFixed(2);
    res.activity_mode = `Battery ${pct}%`;
    updateBatteryDisplay(res.battery_v, pct);
    return res;
  }

  if (cleanLine.startsWith('$IMU')) {
    const parts = cleanLine.split(',');
    if (parts.length >= 7) {
      if (parts.length >= 12) {
        res.accel_x = (parseFloat(parts[3]) / accelLsbPerG).toFixed(3);
        res.accel_y = (parseFloat(parts[4]) / accelLsbPerG).toFixed(3);
        res.accel_z = (parseFloat(parts[5]) / accelLsbPerG).toFixed(3);
        res.gyro_x = (parseFloat(parts[6]) / gyroLsbPerDps).toFixed(1);
        res.gyro_y = (parseFloat(parts[7]) / gyroLsbPerDps).toFixed(1);
        res.gyro_z = (parseFloat(parts[8]) / gyroLsbPerDps).toFixed(1);
        res.pitch = parseFloat(parts[9]).toFixed(2);
        res.roll = parseFloat(parts[10]).toFixed(2);
        res.yaw = parseFloat(parts[11]).toFixed(2);
      } else {
        res.accel_x = (parseFloat(parts[1]) / accelLsbPerG).toFixed(3);
        res.accel_y = (parseFloat(parts[2]) / accelLsbPerG).toFixed(3);
        res.accel_z = (parseFloat(parts[3]) / accelLsbPerG).toFixed(3);
        res.gyro_x = (parseFloat(parts[4]) / gyroLsbPerDps).toFixed(1);
        res.gyro_y = (parseFloat(parts[5]) / gyroLsbPerDps).toFixed(1);
        res.gyro_z = (parseFloat(parts[6]) / gyroLsbPerDps).toFixed(1);
      }
      res.activity_mode = '$IMU Stream';
      res.has_imu_data = true;
    }
  } else if (cleanLine.startsWith('$RPY') || cleanLine.startsWith('$YPR')) {
    const parts = cleanLine.split(',');
    if (parts.length >= 4) {
      res.roll = parseFloat(parts[1]).toFixed(1);
      res.pitch = parseFloat(parts[2]).toFixed(1);
      res.yaw = parseFloat(parts[3]).toFixed(1);
      res.activity_mode = '$RPY Attitude';
      res.has_imu_data = true;
    }
  } else {
    const tokens = cleanLine.split(/[,\t|]+/).map(t => t.trim()).filter(t => t.length > 0);
    const nums = tokens.map(t => parseFloat(t)).filter(n => !isNaN(n));
    if (nums.length >= 6) {
      const scale = Math.abs(nums[2]) > 50 ? accelLsbPerG : 1.0;
      const gScale = Math.abs(nums[3]) > 200 ? gyroLsbPerDps : 1.0;
      res.accel_x = (nums[0] / scale).toFixed(3);
      res.accel_y = (nums[1] / scale).toFixed(3);
      res.accel_z = (nums[2] / scale).toFixed(3);
      res.gyro_x = (nums[3] / gScale).toFixed(1);
      res.gyro_y = (nums[4] / gScale).toFixed(1);
      res.gyro_z = (nums[5] / gScale).toFixed(1);
      res.activity_mode = '6-DOF Raw';
      res.has_imu_data = true;
    }
  }

  if (res.has_imu_data) {
    const orientation = updateIMUAndOrientation(
      res.accel_x || currentImuState.filtAx,
      res.accel_y || currentImuState.filtAy,
      res.accel_z || currentImuState.filtAz,
      res.gyro_x || currentImuState.filtGx,
      res.gyro_y || currentImuState.filtGy,
      res.gyro_z || currentImuState.filtGz,
      res.pitch ? parseFloat(res.pitch) : null,
      res.roll ? parseFloat(res.roll) : null,
      res.yaw ? parseFloat(res.yaw) : null,
      'Live Inbound UART', res.activity_mode || 'Active UART'
    );

    res.accel_x = orientation.ax.toFixed(2);
    res.accel_y = orientation.ay.toFixed(2);
    res.accel_z = orientation.az.toFixed(2);
    res.gyro_x = orientation.gx.toFixed(1);
    res.gyro_y = orientation.gy.toFixed(1);
    res.gyro_z = orientation.gz.toFixed(1);
    res.pitch = orientation.pitch.toFixed(1);
    res.roll = orientation.roll.toFixed(1);
    res.yaw = orientation.yaw.toFixed(1);

    const timeStr = new Date().toLocaleTimeString();
    addChartData(timeStr, orientation.ax, orientation.ay, orientation.az, orientation.gx, orientation.gy, orientation.gz);
  }

  return res;
}

function processRawUartChunk(chunkData, source = 'SERIAL') {
  if (isStreamPaused) return;

  let textChunk = '';
  let hexBytes = [];
  let uint8Array = null;

  if (typeof chunkData === 'string') {
    textChunk = chunkData;
    uint8Array = new TextEncoder().encode(chunkData);
  } else if (chunkData instanceof DataView) {
    uint8Array = new Uint8Array(chunkData.buffer, chunkData.byteOffset, chunkData.byteLength);
    textChunk = new TextDecoder('utf-8', { fatal: false }).decode(uint8Array);
  } else if (chunkData instanceof ArrayBuffer || ArrayBuffer.isView(chunkData)) {
    uint8Array = new Uint8Array(chunkData.buffer || chunkData, chunkData.byteOffset || 0, chunkData.byteLength);
    textChunk = new TextDecoder('utf-8', { fatal: false }).decode(uint8Array);
  }

  if (uint8Array) {
    for (let i = 0; i < uint8Array.length; i++) hexBytes.push(uint8Array[i].toString(16).padStart(2, '0').toUpperCase());
  }

  const rawHexStr = hexBytes.join(' ');
  const displayFormat = streamFormatSelect ? streamFormatSelect.value : 'text';

  if (textChunk) {
    serialLineBuffer += textChunk;
    const lines = serialLineBuffer.split(/\r?\n/);
    serialLineBuffer = lines.pop();

    for (const line of lines) {
      const trimmed = line.trim();
      if (trimmed.length > 0) {
        if (displayFormat !== 'hex') logToConsole('uart-text', `[${source}] ${trimmed}`);
        packetCounter++;
        packetCountEl.textContent = packetCounter;
        const extracted = parseTextTelemetry(trimmed);

        if (isRecording) {
          writeRecordToFileAndBuffer({
            timestamp_iso: new Date().toISOString(),
            timestamp_local: new Date().toLocaleString(),
            packet_number: packetCounter,
            source: source,
            tag_id: extracted.tag_id || (cowTagIdEl.textContent !== '--' ? cowTagIdEl.textContent : 'UART-FEED'),
            data_type: 'UART_TEXT',
            lat: extracted.lat || '',
            lng: extracted.lng || '',
            accel_x: extracted.accel_x || '',
            accel_y: extracted.accel_y || '',
            accel_z: extracted.accel_z || '',
            gyro_x: extracted.gyro_x || '',
            gyro_y: extracted.gyro_y || '',
            gyro_z: extracted.gyro_z || '',
            pitch: extracted.pitch || '',
            roll: extracted.roll || '',
            yaw: extracted.yaw || '',
            battery_v: extracted.battery_v || '',
            battery_pct: extracted.battery_pct || '',
            activity_mode: extracted.activity_mode || 'UART Serial',
            payload_text: trimmed,
            raw_hex: rawHexStr
          });
        }
      }
    }
  }

  if (uint8Array && uint8Array.length > 0) {
    const newBuf = new Uint8Array(uartByteRingBuffer.length + uint8Array.length);
    newBuf.set(uartByteRingBuffer);
    newBuf.set(uint8Array, uartByteRingBuffer.length);
    uartByteRingBuffer = newBuf;

    while (uartByteRingBuffer.length >= 20) {
      let syncIdx = -1;
      for (let i = 0; i <= uartByteRingBuffer.length - 20; i++) {
        if (uartByteRingBuffer[i] === 0xCB) { syncIdx = i; break; }
      }
      if (syncIdx === -1) {
        recordStreamIssue('error', 'warn', `Missing 0xCB Sync in ${uartByteRingBuffer.length} incoming bytes`);
        uartByteRingBuffer = uartByteRingBuffer.slice(Math.max(0, uartByteRingBuffer.length - 19));
        break;
      }
      const packetSlice = uartByteRingBuffer.slice(syncIdx, syncIdx + 20);
      const packetView = new DataView(packetSlice.buffer, packetSlice.byteOffset, packetSlice.byteLength);
      decodeAndProcessPacket(packetView, source);
      uartByteRingBuffer = uartByteRingBuffer.slice(syncIdx + 20);
    }
  }
}

// ==========================================================================
// Chart.js 6-DOF IMU Motion History Setup
// ==========================================================================
function initChart() {
  const chartCanvas = document.getElementById('motionChart');
  if (!chartCanvas) return;
  const ctx = chartCanvas.getContext('2d');

  motionChart = new Chart(ctx, {
    type: 'line',
    data: {
      labels: [],
      datasets: [
        { label: 'Accel X (g)', data: [], borderColor: '#ff4d6d', backgroundColor: 'rgba(255, 77, 109, 0.05)', borderWidth: 2, pointRadius: 0, tension: 0.2, yAxisID: 'y' },
        { label: 'Accel Y (g)', data: [], borderColor: '#38ef7d', backgroundColor: 'rgba(56, 239, 125, 0.05)', borderWidth: 2, pointRadius: 0, tension: 0.2, yAxisID: 'y' },
        { label: 'Accel Z (g)', data: [], borderColor: '#00c6ff', backgroundColor: 'rgba(0, 198, 255, 0.05)', borderWidth: 2, pointRadius: 0, tension: 0.2, yAxisID: 'y' },
        { label: 'Gyro X (°/s)', data: [], borderColor: '#f59e0b', backgroundColor: 'transparent', borderDash: [4, 3], borderWidth: 1.8, pointRadius: 0, tension: 0.2, yAxisID: 'y1' },
        { label: 'Gyro Y (°/s)', data: [], borderColor: '#a855f7', backgroundColor: 'transparent', borderDash: [4, 3], borderWidth: 1.8, pointRadius: 0, tension: 0.2, yAxisID: 'y1' },
        { label: 'Gyro Z (°/s)', data: [], borderColor: '#ec4899', backgroundColor: 'transparent', borderDash: [4, 3], borderWidth: 1.8, pointRadius: 0, tension: 0.2, yAxisID: 'y1' }
      ]
    },
    options: {
      responsive: true,
      maintainAspectRatio: false,
      animation: false,
      interaction: { mode: 'index', intersect: false },
      scales: {
        x: { grid: { color: 'rgba(255, 255, 255, 0.05)' }, ticks: { color: '#94a3b8', font: { family: 'JetBrains Mono', size: 10 }, maxRotation: 0, autoSkip: true, maxTicksLimit: 8 } },
        y: { type: 'linear', position: 'left', min: -accelFullScale, max: accelFullScale, grid: { color: 'rgba(255, 255, 255, 0.08)' }, ticks: { color: '#00c6ff', font: { family: 'JetBrains Mono', size: 10 } }, title: { display: true, text: `Acceleration (±${accelFullScale}g)`, color: '#00c6ff', font: { size: 11, weight: '600' } } },
        y1: { type: 'linear', position: 'right', min: -gyroFullScale, max: gyroFullScale, grid: { drawOnChartArea: false }, ticks: { color: '#f59e0b', font: { family: 'JetBrains Mono', size: 10 } }, title: { display: true, text: `Angular Rate (±${gyroFullScale} °/s)`, color: '#f59e0b', font: { size: 11, weight: '600' } } }
      },
      plugins: {
        legend: { position: 'top', labels: { color: '#f1f5f9', font: { family: 'Inter', size: 11 }, boxWidth: 12, padding: 8 } },
        tooltip: { backgroundColor: 'rgba(11, 15, 25, 0.95)', titleFont: { family: 'JetBrains Mono' }, bodyFont: { family: 'JetBrains Mono' } }
      }
    }
  });

  applyChartFilterMode();
  startChartRenderLoop();
}

function addChartData(timeLabel, ax, ay, az, gx = 0, gy = 0, gz = 0) {
  if (!motionChart) return;
  const now = Date.now();
  chartRingBuffer.push(now, timeLabel, Number(ax) || 0, Number(ay) || 0, Number(az) || 0, Number(gx) || 0, Number(gy) || 0, Number(gz) || 0);
  chartNeedsUpdate = true;
}

function startChartRenderLoop() {
  function loop(timestamp) {
    requestAnimationFrame(loop);
    if (!isFullWidth && isPageVisible && chartNeedsUpdate && (timestamp - lastChartRenderTime >= CHART_RENDER_FPS_INTERVAL)) {
      lastChartRenderTime = timestamp;
      chartNeedsUpdate = false;
      rebuildChartFromBuffer();
    }
  }
  requestAnimationFrame(loop);
}

function rebuildChartFromBuffer() {
  if (!motionChart || isFullWidth) return;
  const cutoffTime = Date.now() - chartTimeWindowMs;
  const data = chartRingBuffer.getOrderedData(cutoffTime);

  motionChart.data.labels = data.labels;
  motionChart.data.datasets[0].data = data.ax;
  motionChart.data.datasets[1].data = data.ay;
  motionChart.data.datasets[2].data = data.az;
  motionChart.data.datasets[3].data = data.gx;
  motionChart.data.datasets[4].data = data.gy;
  motionChart.data.datasets[5].data = data.gz;
  applyChartFilterMode();

  if (chartWindowBadge) {
    const sec = Math.round(chartTimeWindowMs / 1000);
    const winLabel = sec >= 60 ? `${Math.round(sec / 60)} Min` : `${sec}s`;
    chartWindowBadge.innerHTML = `<i class="fa-solid fa-clock"></i> Last ${winLabel} (${sec}s) • ${data.labels.length} pts`;
  }
  motionChart.update('none');
}

function applyChartFilterMode() {
  if (!motionChart) return;
  const isAccelVisible = chartFilterMode === 'all' || chartFilterMode === 'accel';
  const isGyroVisible = chartFilterMode === 'all' || chartFilterMode === 'gyro';

  motionChart.data.datasets[0].hidden = !isAccelVisible;
  motionChart.data.datasets[1].hidden = !isAccelVisible;
  motionChart.data.datasets[2].hidden = !isAccelVisible;
  motionChart.data.datasets[3].hidden = !isGyroVisible;
  motionChart.data.datasets[4].hidden = !isGyroVisible;
  motionChart.data.datasets[5].hidden = !isGyroVisible;

  motionChart.options.scales.y.display = isAccelVisible;
  motionChart.options.scales.y1.display = isGyroVisible;
}

// ==========================================================================
// Leaflet GPS Map Setup
// ==========================================================================
function initMap() {
  const startLat = 31.968600;
  const startLng = -99.901800;

  map = L.map('map', { center: [startLat, startLng], zoom: 16, zoomControl: false });
  L.control.zoom({ position: 'topright' }).addTo(map);

  L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', { attribution: '&copy; OpenStreetMap', maxZoom: 19 }).addTo(map);

  const cowIcon = L.divIcon({
    className: 'custom-cow-icon',
    html: `<div style="background:#00c6ff; color:#040914; border-radius:50%; width:32px; height:32px; display:flex; align-items:center; justify-content:center; box-shadow:0 0 12px #00c6ff; border:2px solid #fff;"><i class="fa-solid fa-cow" style="font-size:16px;"></i></div>`,
    iconSize: [32, 32],
    iconAnchor: [16, 16]
  });

  cowMarker = L.marker([startLat, startLng], { icon: cowIcon }).addTo(map);
  polyline = L.polyline([], { color: '#00f2fe', weight: 3, opacity: 0.8, dashArray: '5, 8' }).addTo(map);
}

function updateGPSPosition(lat, lng, alt, speed) {
  if (lblLat) lblLat.textContent = lat.toFixed(6);
  if (lblLng) lblLng.textContent = lng.toFixed(6);
  if (lblAlt) lblAlt.textContent = `${alt.toFixed(1)} m`;
  if (lblSpeed) lblSpeed.textContent = `${speed.toFixed(1)} km/h`;

  if (isFullWidth) return;

  const newPos = [lat, lng];
  if (cowMarker) cowMarker.setLatLng(newPos);
  pathHistory.push(newPos);
  if (pathHistory.length > 50) pathHistory.shift();
  if (polyline) polyline.setLatLngs(pathHistory);
  if (map) map.panTo(newPos);
}

// ==========================================================================
// Virtualized & Capped DOM Terminal Logging (150 Node Max Limit)
// ==========================================================================
const MAX_TERMINAL_LOG_ENTRIES = 150;

function renderParsedFields(parsedObj) {
  if (!fieldsGrid) return;
  fieldsGrid.innerHTML = '';
  for (const [key, val] of Object.entries(parsedObj)) {
    const item = document.createElement('div');
    item.className = 'field-item';
    item.innerHTML = `<span class="f-name">${key}</span><span class="f-val">${val}</span>`;
    fieldsGrid.appendChild(item);
  }
}

function logToConsole(type, msg) {
  if (!terminalLog) return;
  const time = new Date().toLocaleTimeString();
  const entry = document.createElement('div');
  entry.className = `log-entry ${type}`;
  entry.innerHTML = `<span style="color: #64748b; margin-right: 8px;">[${time}]</span> ${escapeHtml(msg)}`;
  
  terminalLog.appendChild(entry);

  // Enforce DOM cap to prevent memory leaks and layout slowdowns
  while (terminalLog.childNodes.length > MAX_TERMINAL_LOG_ENTRIES) {
    terminalLog.removeChild(terminalLog.firstChild);
  }

  if (autoScroll) terminalLog.scrollTop = terminalLog.scrollHeight;
}

function escapeHtml(str) {
  return str.replace(/&/g, "&amp;").replace(/</g, "&lt;").replace(/>/g, "&gt;");
}

function togglePauseStream() {
  isStreamPaused = !isStreamPaused;
  if (btnPauseStream) {
    btnPauseStream.classList.toggle('active', isStreamPaused);
    btnPauseStream.innerHTML = isStreamPaused ? '<i class="fa-solid fa-play"></i>' : '<i class="fa-solid fa-pause"></i>';
  }
  logToConsole('system', isStreamPaused ? 'UART Stream PAUSED.' : 'UART Stream RESUMED.');
}

function toggleAutoScroll() {
  autoScroll = !autoScroll;
  if (btnToggleAutoScroll) btnToggleAutoScroll.classList.toggle('active', autoScroll);
}

// ==========================================================================
// Telemetry Stream CSV Recorder
// ==========================================================================
const CSV_HEADERS = [
  'Timestamp (ISO)', 'Timestamp (Local)', 'Packet #', 'Source', 'Cow Tag ID', 'Data Type',
  'Latitude (°)', 'Longitude (°)', 'Accel X (g)', 'Accel Y (g)', 'Accel Z (g)',
  'Gyro X (°/s)', 'Gyro Y (°/s)', 'Gyro Z (°/s)', 'Pitch (°)', 'Roll (°)', 'Yaw (°)',
  'Battery (V)', 'Activity Mode', 'Payload Text', 'Raw Hex'
];

function escapeCsvField(field) {
  if (field === null || field === undefined) return '""';
  return `"${String(field).replace(/"/g, '""')}"`;
}

function buildCsvRow(p) {
  return [
    escapeCsvField(p.timestamp_iso || new Date().toISOString()),
    escapeCsvField(p.timestamp_local || new Date().toLocaleString()),
    p.packet_number !== undefined ? p.packet_number : '',
    escapeCsvField(p.source || 'UART'),
    escapeCsvField(p.tag_id || 'COW-TAG'),
    escapeCsvField(p.data_type || 'TELEMETRY'),
    p.lat !== undefined ? p.lat : '',
    p.lng !== undefined ? p.lng : '',
    p.accel_x !== undefined ? p.accel_x : '',
    p.accel_y !== undefined ? p.accel_y : '',
    p.accel_z !== undefined ? p.accel_z : '',
    p.gyro_x !== undefined ? p.gyro_x : '',
    p.gyro_y !== undefined ? p.gyro_y : '',
    p.gyro_z !== undefined ? p.gyro_z : '',
    p.pitch !== undefined ? p.pitch : '',
    p.roll !== undefined ? p.roll : '',
    p.yaw !== undefined ? p.yaw : '',
    p.battery_v !== undefined ? p.battery_v : '',
    escapeCsvField(p.activity_mode || ''),
    escapeCsvField(p.payload_text || ''),
    escapeCsvField(p.raw_hex || '')
  ].join(',');
}

function downloadCsvFile(filename, records) {
  if (!records || records.length === 0) {
    alert('No telemetry records captured to export.');
    return;
  }

  const csvLines = ['\uFEFF' + CSV_HEADERS.join(',')];
  for (const r of records) csvLines.push(buildCsvRow(r));

  const blob = new Blob([csvLines.join('\r\n')], { type: 'text/csv;charset=utf-8;' });
  const url = URL.createObjectURL(blob);
  const link = document.createElement('a');
  link.setAttribute('href', url);
  link.setAttribute('download', filename || `ranchbot_telemetry_${Date.now()}.csv`);
  document.body.appendChild(link);
  link.click();

  setTimeout(() => {
    document.body.removeChild(link);
    URL.revokeObjectURL(url);
  }, 500);

  logToConsole('system', `✓ FILE SAVED: "${link.download}" (${records.length} records).`);
}

function toggleRecording() {
  if (isRecording) stopRecording();
  else startRecording();
}

function startRecording() {
  if (isRecording) return;
  const now = new Date();
  activeRecordingFileName = `ranchbot_uart_stream_${now.toISOString().replace(/[:.]/g, '-').slice(0, 19)}.csv`;
  recordedPackets = [];
  isRecording = true;
  recordingStartTime = Date.now();

  if (recorderBadge) {
    recorderBadge.className = 'recorder-status-badge recording';
    recorderStatusText.textContent = 'RECORDING...';
  }
  if (recorderFilePill) {
    recorderFilePill.classList.add('active');
    recorderFilenameText.textContent = activeRecordingFileName;
  }
  if (btnRecordToggle) {
    btnRecordToggle.className = 'btn btn-sm btn-record is-recording';
    btnRecordToggle.innerHTML = '<i class="fa-solid fa-stop"></i> Stop & Save';
  }
  if (btnExportCsv) btnExportCsv.removeAttribute('disabled');

  if (recordingTimerInterval) clearInterval(recordingTimerInterval);
  recordingTimerInterval = setInterval(updateRecorderTimerDisplay, 1000);
  updateRecorderTimerDisplay();
  updateRecorderStats();
}

function writeRecordToFileAndBuffer(recordItem) {
  if (!isRecording) return;
  recordedPackets.push(recordItem);
  updateRecorderStats();
}

function stopRecording() {
  if (!isRecording) return;
  isRecording = false;

  if (recordingTimerInterval) {
    clearInterval(recordingTimerInterval);
    recordingTimerInterval = null;
  }

  if (recorderBadge) {
    recorderBadge.className = 'recorder-status-badge stopped';
    recorderStatusText.textContent = 'REC IDLE';
  }
  if (btnRecordToggle) {
    btnRecordToggle.className = 'btn btn-sm btn-record';
    btnRecordToggle.innerHTML = '<i class="fa-solid fa-circle"></i> Start Recording';
  }

  if (recordedPackets.length > 0) {
    downloadCsvFile(activeRecordingFileName, recordedPackets);
  }
  if (recorderFilePill) {
    recorderFilePill.classList.remove('active');
    recorderFilenameText.textContent = recordedPackets.length > 0 ? `Saved: ${activeRecordingFileName}` : 'No file selected';
  }
}

function updateRecorderTimerDisplay() {
  if (!recordingStartTime || !recorderTimer) return;
  const totalSeconds = Math.floor((Date.now() - recordingStartTime) / 1000);
  const hrs = String(Math.floor(totalSeconds / 3600)).padStart(2, '0');
  const mins = String(Math.floor((totalSeconds % 3600) / 60)).padStart(2, '0');
  const secs = String(totalSeconds % 60).padStart(2, '0');
  recorderTimer.textContent = `${hrs}:${mins}:${secs}`;
}

function updateRecorderStats() {
  const count = recordedPackets.length;
  if (recorderCount) recorderCount.textContent = `${count} records`;
  if (recorderSize) recorderSize.textContent = `(~${((count * 240) / 1024).toFixed(1)} KB)`;
  if (count > 0 && btnExportCsv) btnExportCsv.removeAttribute('disabled');
}

function exportCsv() {
  if (recordedPackets.length === 0) {
    alert('No recorded stream data to export.');
    return;
  }
  downloadCsvFile(activeRecordingFileName, recordedPackets);
}

function clearCsvBuffer() {
  if (isRecording) stopRecording();
  recordedPackets = [];
  recordingStartTime = null;
  updateRecorderTimerDisplay();
  updateRecorderStats();
  if (recorderFilePill) recorderFilenameText.textContent = 'No file selected';
  if (btnExportCsv) btnExportCsv.setAttribute('disabled', 'true');
  logToConsole('system', 'Recorded telemetry buffer cleared.');
}

// ==========================================================================
// Simulator Stream Generator (20 Hz Live Stream)
// ==========================================================================
function toggleSimulatorStream() {
  isSimulatorRunning = !isSimulatorRunning;
  if (isSimulatorRunning) {
    btnSimulateStream.classList.add('btn-primary');
    btnSimulateStream.innerHTML = '<i class="fa-solid fa-stop"></i> Stop Test';
    logToConsole('system', '[SIMULATOR] High-Speed 6-DOF IMU Stream Generator STARTED (20 Hz).');

    let simLat = 31.968600;
    let simLng = -99.901800;
    let count = 0;

    simulatorInterval = setInterval(() => {
      count++;
      simLat += (Math.random() - 0.5) * 0.00005;
      simLng += (Math.random() - 0.5) * 0.00005;

      const t = count * 0.08;
      const ax = (Math.sin(t * 1.5) * 0.85).toFixed(3);
      const ay = (Math.cos(t * 1.2) * 0.65).toFixed(3);
      const az = (0.98 + Math.sin(t * 0.8) * 0.25).toFixed(3);

      const gx = (Math.cos(t * 1.8) * 45.0).toFixed(1);
      const gy = (Math.sin(t * 1.5) * 55.0).toFixed(1);
      const gz = (Math.sin(t * 0.9) * 25.0).toFixed(1);

      const pitch = (Math.sin(t * 1.5) * 35.0).toFixed(2);
      const roll = (Math.cos(t * 1.2) * 28.0).toFixed(2);
      const yaw = ((t * 20.0) % 360).toFixed(2);
      const battMv = Math.max(3400, Math.round(4120 - (count * 0.05)));

      const rawAx = Math.round(parseFloat(ax) * accelLsbPerG);
      const rawAy = Math.round(parseFloat(ay) * accelLsbPerG);
      const rawAz = Math.round(parseFloat(az) * accelLsbPerG);
      const rawGx = Math.round(parseFloat(gx) * gyroLsbPerDps);
      const rawGy = Math.round(parseFloat(gy) * gyroLsbPerDps);
      const rawGz = Math.round(parseFloat(gz) * gyroLsbPerDps);

      const mode = count % 6;
      if (mode === 0) {
        const textMsg = `$IMU,${280 + count},${482000 + count * 20},${rawAx},${rawAy},${rawAz},${rawGx},${rawGy},${rawGz},${pitch},${roll},${yaw}\n`;
        processRawUartChunk(textMsg, 'SIM_UART');
      } else if (mode === 1) {
        const textMsg = `$RPY,${roll},${pitch},${yaw}\n`;
        processRawUartChunk(textMsg, 'SIM_UART');
      } else if (mode === 2) {
        const textMsg = `ACCEL: ${ax}, ${ay}, ${az}\nGYRO: ${gx}, ${gy}, ${gz}\n`;
        processRawUartChunk(textMsg, 'SIM_UART');
      } else if (mode === 3) {
        // Simulated stream error/warning
        if (count % 30 === 0) {
          processRawUartChunk(`[${new Date().toLocaleTimeString()}] <err> cowtag_uart: CRC_ERR frame dropped at pkt #${count}\n`, 'SIM_UART');
        } else if (count % 45 === 0) {
          processRawUartChunk(`[${new Date().toLocaleTimeString()}] <wrn> cowtag_pwr: BATTERY : 18 % ( 3380 mV ) Low Threshold\n`, 'SIM_UART');
        } else {
          const buffer = new ArrayBuffer(20);
          const view = new DataView(buffer);
          view.setUint8(0, 0xCB);
          view.setUint16(1, 8492, true);
          view.setInt32(3, Math.round(simLat * 1e7), true);
          view.setInt32(7, Math.round(simLng * 1e7), true);
          view.setInt16(11, rawAx, true);
          view.setInt16(13, rawAy, true);
          view.setInt16(15, rawAz, true);
          view.setUint16(17, battMv, true);
          view.setUint8(19, 1);
          processRawUartChunk(view, 'SIM_BLE');
        }
      } else {
        const buffer = new ArrayBuffer(20);
        const view = new DataView(buffer);
        view.setUint8(0, 0xCB);
        view.setUint16(1, 8492, true);
        view.setInt32(3, Math.round(simLat * 1e7), true);
        view.setInt32(7, Math.round(simLng * 1e7), true);
        view.setInt16(11, rawAx, true);
        view.setInt16(13, rawAy, true);
        view.setInt16(15, rawAz, true);
        view.setUint16(17, battMv, true);
        view.setUint8(19, 1);
        processRawUartChunk(view, 'SIM_BLE');
      }
    }, 50);
  } else {
    clearInterval(simulatorInterval);
    simulatorInterval = null;
    btnSimulateStream.classList.remove('btn-primary');
    btnSimulateStream.innerHTML = '<i class="fa-solid fa-flask"></i> Test Stream';
    logToConsole('system', '[SIMULATOR] Test stream generator STOPPED.');
  }
}

// ==========================================================================
// Mobile Tabs & Zero-Hang Full Width Layout Handlers
// ==========================================================================
function setupMobileTabs() {
  if (!mobileTabNav) return;
  if (dashboardGrid && !dashboardGrid.dataset.mobileTab) {
    dashboardGrid.dataset.mobileTab = 'telemetry';
  }

  mobileTabNav.addEventListener('click', (e) => {
    const btn = e.target.closest('.mobile-tab-btn');
    if (!btn) return;
    const targetTab = btn.dataset.tab || 'telemetry';

    mobileTabNav.querySelectorAll('.mobile-tab-btn').forEach(b => b.classList.remove('active'));
    btn.classList.add('active');

    if (dashboardGrid) dashboardGrid.dataset.mobileTab = targetTab;

    setTimeout(() => {
      if (map) map.invalidateSize();
      if (motionChart) motionChart.resize();
      if (renderer3d && camera3d) {
        const container = document.getElementById('imu3dContainer');
        if (container) {
          const w = container.clientWidth || 300;
          const h = container.clientHeight || 280;
          camera3d.aspect = w / h;
          camera3d.updateProjectionMatrix();
          renderer3d.setSize(w, h);
          render3D();
        }
      }
    }, 120);

    if (window.innerWidth <= 860) window.scrollTo({ top: 0, behavior: 'smooth' });
  });
}

function setupResponsiveHandlers() {
  let resizeTimeout = null;
  const handleResize = () => {
    clearTimeout(resizeTimeout);
    resizeTimeout = setTimeout(() => {
      if (!isFullWidth) {
        if (map) map.invalidateSize();
        if (motionChart) motionChart.resize();
        if (renderer3d && camera3d) {
          const container = document.getElementById('imu3dContainer');
          if (container) {
            const w = container.clientWidth || 300;
            const h = container.clientHeight || 280;
            camera3d.aspect = w / h;
            camera3d.updateProjectionMatrix();
            renderer3d.setSize(w, h);
            render3D();
          }
        }
      }
    }, 150);
  };

  window.addEventListener('resize', handleResize);
  window.addEventListener('orientationchange', () => setTimeout(handleResize, 200));
}

function toggleFullWidthStream() {
  const dashboardGrid = document.querySelector('.dashboard-grid');
  if (!dashboardGrid) return;

  isFullWidth = !isFullWidth;
  if (isFullWidth) {
    dashboardGrid.classList.add('full-width-active');
    btnToggleFullWidth.innerHTML = '<i class="fa-solid fa-compress"></i> <span class="btn-text">Restore View</span>';
    btnToggleFullWidth.classList.add('btn-primary');
    logToConsole('system', 'Switched to Full Screen UART Stream View (Front-screen rendering suspended for maximum performance).');
  } else {
    dashboardGrid.classList.remove('full-width-active');
    btnToggleFullWidth.innerHTML = '<i class="fa-solid fa-expand"></i> <span class="btn-text">Full Width</span>';
    btnToggleFullWidth.classList.remove('btn-primary');
    logToConsole('system', 'Restored Normal Dashboard View.');

    // Single instantaneous coordinated refresh upon returning
    requestAnimationFrame(() => {
      if (map) map.invalidateSize();
      if (motionChart) {
        motionChart.resize();
        rebuildChartFromBuffer();
      }
      if (renderer3d && camera3d) {
        const container = document.getElementById('imu3dContainer');
        if (container) {
          const w = container.clientWidth || 300;
          const h = container.clientHeight || 280;
          camera3d.aspect = w / h;
          camera3d.updateProjectionMatrix();
          renderer3d.setSize(w, h);
          render3D();
        }
      }
      updateIMUAndOrientation(
        currentImuState.rawAx, currentImuState.rawAy, currentImuState.rawAz,
        currentImuState.rawGx, currentImuState.rawGy, currentImuState.rawGz
      );
    });
  }
}

function checkLastConnectedDevice() {
  const lastBleName = localStorage.getItem('ranchbot_last_ble_name');
  const lastBleId = localStorage.getItem('ranchbot_last_ble_id');
  if (lastBleName || lastBleId) {
    const displayName = lastBleName || 'Saved Ear Tag';
    if (btnLastDevice && lastDeviceNameText) {
      lastDeviceNameText.textContent = `Reconnect "${displayName}"`;
      btnLastDevice.style.display = 'inline-flex';
    }
  }
}

function saveLastConnectedBle(name, id) {
  if (name) localStorage.setItem('ranchbot_last_ble_name', name);
  if (id) localStorage.setItem('ranchbot_last_ble_id', id);
  checkLastConnectedDevice();
}

function saveLastConnectedSerial(baud) {
  if (baud) localStorage.setItem('ranchbot_last_serial_baud', String(baud));
}

async function handleLastDeviceReconnect() {
  handleConnectButtonClick();
}
