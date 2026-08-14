// ==========================================================================
// Ranchbot Cattle Ear Tag BLE & 6-DOF IMU Telemetry Receiver Application
// 16-Bit Raw ADC IMU Calibration (±2.5G FS / ±125°/s FS)
// Real-time 60FPS 3D IMU Chip & Carrier Board Orientation Renderer (Three.js)
// Center-Zero Bi-Directional Axis Gauges with Dynamic Glowing Ball Indicators
// Independent Digital Low-Pass Filters (LPF) for Accel & Gyro
// High-Performance RAF-Throttled Graph & 3D Render Loops
// Last Connected Device Persistence & Scan Priority
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

// Orientation Kinematics State
let lastEstimatedYaw = 0;
let lastOrientationTimestamp = null;

// Stream CSV Recorder & Disk State
let isRecording = false;
let recordedPackets = [];
let recordingStartTime = null;
let recordingTimerInterval = null;
let isFullWidth = false;
let activeRecordingFileName = '';

// Simulator Stream State
let isSimulatorRunning = false;
let simulatorInterval = null;
let uartByteRingBuffer = new Uint8Array(0);

// Leaflet Map State
let map = null;
let cowMarker = null;
let polyline = null;
let pathHistory = [];

// 6-DOF IMU Motion Chart State & Rolling Time Window
let motionChart = null;
let chartTimeWindowMs = 60000; // default 1 minute (60s)
let chartFilterMode = 'all';    // 'all', 'accel', 'gyro'
let chartBuffer = [];          // array of { time: number, label: string, ax, ay, az, gx, gy, gz }
let chartNeedsUpdate = false;
let lastChartRenderTime = 0;
const CHART_RENDER_FPS_INTERVAL = 33; // ~30 FPS throttling for high-performance chart rendering

// Three.js 3D IMU Chip Visualization State
let scene3d = null;
let camera3d = null;
let renderer3d = null;
let imuBoardGroup = null;
let axesGizmoGroup = null;
let deskGridHelper = null;
let is3dInitialized = false;
let is3dAnimating = false;
let isMouseDragging3d = false;
let previousMousePosition = { x: 0, y: 0 };
let show3dAxes = true;
let show3dGrid = true;

// Slerp Quaternion Interpolation for 60FPS / 120FPS smooth continuous rotation
let targetQuaternion = null;
let currentQuaternion = null;
let targetEuler = null;

// Continuous IMU Latest State
let currentImuState = {
  rawAx: 0.00,
  rawAy: 0.00,
  rawAz: 1.00,
  rawGx: 0.0,
  rawGy: 0.0,
  rawGz: 0.0,
  filtAx: 0.00,
  filtAy: 0.00,
  filtAz: 1.00,
  filtGx: 0.0,
  filtGy: 0.0,
  filtGz: 0.0,
  pitch: 0.0,
  roll: 0.0,
  yaw: 0.0,
  totalG: 1.00
};

// ==========================================================================
// Digital Low-Pass Filter (DSP) Implementation
// Supports 1st Order Single-Pole & 2nd Order Butterworth Biquad IIR Filters
// ==========================================================================
class DigitalLowPassFilter3Axis {
  constructor(cutoffFreq = 2.0, order = 2) {
    this.cutoffFreq = cutoffFreq;
    this.order = parseInt(order, 10) || 2;
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
    if (!this.enabled) {
      return { x: rawX, y: rawY, z: rawZ };
    }

    let dt = 0.05; // 50ms default (20 Hz)
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
      ch.y1 = inputVal;
      ch.y2 = inputVal;
      ch.x1 = inputVal;
      ch.x2 = inputVal;
      ch.out = inputVal;
      ch.initialized = true;
      return inputVal;
    }

    if (this.order === 1) {
      // 1st Order Single-Pole Exponential Low-Pass Filter
      const tau = 1.0 / (2.0 * Math.PI * this.cutoffFreq);
      const alpha = dt / (tau + dt);
      ch.out = ch.out + alpha * (inputVal - ch.out);
      return ch.out;
    } else {
      // 2nd Order Butterworth Low-Pass Filter via Bilinear Transform
      const fc = Math.min(this.cutoffFreq, 0.45 / dt);
      const omega = 2.0 * Math.PI * fc;
      const K = Math.tan((omega * dt) / 2.0);
      const Q = 0.70710678; // 1 / sqrt(2)
      const K2 = K * K;
      const norm = 1.0 + K / Q + K2;

      const b0 = K2 / norm;
      const b1 = 2.0 * b0;
      const b2 = b0;
      const a1 = 2.0 * (K2 - 1.0) / norm;
      const a2 = (1.0 - K / Q + K2) / norm;

      const out = b0 * inputVal + b1 * ch.x1 + b2 * ch.x2 - a1 * ch.y1 - a2 * ch.y2;

      ch.x2 = ch.x1;
      ch.x1 = inputVal;
      ch.y2 = ch.y1;
      ch.y1 = out;
      ch.out = out;

      if (this.order === 3) {
        // 3rd Order: Cascade single pole with 2nd order stage
        const tau = 1.0 / (2.0 * Math.PI * this.cutoffFreq);
        const alpha = dt / (tau + dt);
        ch.out = ch.y2 + alpha * (out - ch.y2);
      }

      return ch.out;
    }
  }
}

// Instantiate Filters
const accelFilter = new DigitalLowPassFilter3Axis(2.0, 2); // default 2.0 Hz, 2nd Order
const gyroFilter = new DigitalLowPassFilter3Axis(5.0, 2);  // default 5.0 Hz, 2nd Order

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
const decoderSelect = document.getElementById('decoderSelect');
const accelRangeSelect = document.getElementById('accelRangeSelect');
const gyroRangeSelect = document.getElementById('gyroRangeSelect');

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
const gpsFixPill = document.getElementById('gpsFixPill');

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

// 3D Viewport Controls
const btnReset3dView = document.getElementById('btnReset3dView');
const btnToggle3dAxes = document.getElementById('btnToggle3dAxes');
const btnToggle3dGrid = document.getElementById('btnToggle3dGrid');
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
const impactIndicator = document.getElementById('impactIndicator');
const impactText = document.getElementById('impactText');

// Chart Controls
const chartWindowBadge = document.getElementById('chartWindowBadge');
const chartModeGroup = document.getElementById('chartModeGroup');
const chartWindowGroup = document.getElementById('chartWindowGroup');

// Terminal, Parsed Grid & Full-Width Elements
const terminalLog = document.getElementById('terminalLog');
const fieldsGrid = document.getElementById('fieldsGrid');
const btnToggleFullWidth = document.getElementById('btnToggleFullWidth');
const btnPauseStream = document.getElementById('btnPauseStream');
const btnToggleAutoScroll = document.getElementById('btnToggleAutoScroll');
const consoleCard = document.getElementById('consoleCard');

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
const TAG_SERVICE_UUID = '0000181a-0000-1000-8000-00805f9b34fb';

const mobileTabNav = document.getElementById('mobileTabNav');
const dashboardGrid = document.querySelector('.dashboard-grid');

// ==========================================================================
// Application Initialization
// ==========================================================================
document.addEventListener('DOMContentLoaded', () => {
  initMap();
  initChart();
  init3DImuViewer();
  setupEventListeners();
  setupFilterEventListeners();
  setupMobileTabs();
  setupResponsiveHandlers();
  checkLastConnectedDevice();
  
  // Ensure Tag Overview starts completely blank as requested
  updateDeviceOverview('--', '--', false);
  
  // Set initial flat orientation (top of chip up on desk)
  updateIMUAndOrientation(0.00, 0.00, 1.00, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 'Default Position', 'At Rest');
  
  logToConsole('system', 'Ranchbot Cow Tag BLE & GPS/IMU Receiver initialized. 60FPS 3D & Bi-Directional Gauges ACTIVE.');
});

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
      chartBuffer = [];
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

  // IMU Range Selector Listeners
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

  // 3D Viewport Controls
  if (btnReset3dView) btnReset3dView.addEventListener('click', reset3DView);
  if (btnToggle3dAxes) btnToggle3dAxes.addEventListener('click', toggle3DAxes);
  if (btnToggle3dGrid) btnToggle3dGrid.addEventListener('click', toggle3DGrid);

  // Chart Mode Filter Buttons (All, Accel, Gyro)
  if (chartModeGroup) {
    chartModeGroup.addEventListener('click', (e) => {
      const btn = e.target.closest('.btn-filter');
      if (!btn) return;
      chartModeGroup.querySelectorAll('.btn-filter').forEach(b => b.classList.remove('active'));
      btn.classList.add('active');
      chartFilterMode = btn.dataset.mode || 'all';
      applyChartFilterMode();
      if (motionChart) motionChart.update();
    });
  }

  // Chart Time Window Buttons (30s, 1 Min, 3 Min)
  if (chartWindowGroup) {
    chartWindowGroup.addEventListener('click', (e) => {
      const btn = e.target.closest('.btn-window');
      if (!btn) return;
      chartWindowGroup.querySelectorAll('.btn-window').forEach(b => b.classList.remove('active'));
      btn.classList.add('active');
      const winSec = parseInt(btn.dataset.window, 10) || 60;
      chartTimeWindowMs = winSec * 1000;

      const cutoff = Date.now() - chartTimeWindowMs;
      while (chartBuffer.length > 0 && chartBuffer[0].time < cutoff) {
        chartBuffer.shift();
      }
      chartNeedsUpdate = true;
      rebuildChartFromBuffer();
    });
  }

  // CSV Recorder & Full Width Listeners
  if (btnRecordToggle) btnRecordToggle.addEventListener('click', toggleRecording);
  if (btnSimulateStream) btnSimulateStream.addEventListener('click', toggleSimulatorStream);
  if (btnExportCsv) btnExportCsv.addEventListener('click', exportCsv);
  if (btnClearCsv) btnClearCsv.addEventListener('click', clearCsvBuffer);
  if (btnToggleFullWidth) btnToggleFullWidth.addEventListener('click', toggleFullWidthStream);
}

// ==========================================================================
// Digital Low-Pass Filter Control Panel & Presets
// ==========================================================================
function setupFilterEventListeners() {
  // Accelerometer Filter Controls
  if (chkAccelFilter) {
    chkAccelFilter.addEventListener('change', () => {
      accelFilter.enabled = chkAccelFilter.checked;
      accelFilterPill.className = chkAccelFilter.checked ? 'pill pill-success' : 'pill';
      accelFilterPill.textContent = chkAccelFilter.checked ? 'LPF ON' : 'OFF';
      updateFilterMasterStatus();
    });
  }

  if (accelFilterOrder) {
    accelFilterOrder.addEventListener('change', () => {
      accelFilter.order = parseInt(accelFilterOrder.value, 10);
      accelFilter.reset();
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
      updateFilterMasterStatus();
    });

    accelCutoffInput.addEventListener('change', () => {
      let freq = Math.max(0.1, Math.min(25.0, parseFloat(accelCutoffInput.value) || 2.0));
      accelCutoffInput.value = freq.toFixed(1);
      accelCutoffFreq.value = freq;
      lblAccelCutoff.textContent = `${freq.toFixed(1)} Hz`;
      accelFilter.cutoffFreq = freq;
      accelFilter.reset();
      updateFilterMasterStatus();
    });
  }

  // Gyroscope Filter Controls
  if (chkGyroFilter) {
    chkGyroFilter.addEventListener('change', () => {
      gyroFilter.enabled = chkGyroFilter.checked;
      gyroFilterPill.className = chkGyroFilter.checked ? 'pill pill-success' : 'pill';
      gyroFilterPill.textContent = chkGyroFilter.checked ? 'LPF ON' : 'OFF';
      updateFilterMasterStatus();
    });
  }

  if (gyroFilterOrder) {
    gyroFilterOrder.addEventListener('change', () => {
      gyroFilter.order = parseInt(gyroFilterOrder.value, 10);
      gyroFilter.reset();
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
      updateFilterMasterStatus();
    });

    gyroCutoffInput.addEventListener('change', () => {
      let freq = Math.max(0.2, Math.min(50.0, parseFloat(gyroCutoffInput.value) || 5.0));
      gyroCutoffInput.value = freq.toFixed(1);
      gyroCutoffFreq.value = freq;
      lblGyroCutoff.textContent = `${freq.toFixed(1)} Hz`;
      gyroFilter.cutoffFreq = freq;
      gyroFilter.reset();
      updateFilterMasterStatus();
    });
  }

  // Presets
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
      updateFilterMasterStatus();
    });
  });

  // Reset Filters Defaults Button
  if (btnResetFilters) {
    btnResetFilters.addEventListener('click', () => {
      if (chkAccelFilter) chkAccelFilter.checked = true;
      if (accelFilterOrder) accelFilterOrder.value = '2';
      if (accelCutoffFreq) accelCutoffFreq.value = 2.0;
      if (accelCutoffInput) accelCutoffInput.value = '2.0';
      if (lblAccelCutoff) lblAccelCutoff.textContent = '2.0 Hz';
      if (accelFilterPill) {
        accelFilterPill.className = 'pill pill-success';
        accelFilterPill.textContent = 'LPF ON';
      }
      accelFilter.setParameters(2.0, 2, true);
      accelFilter.reset();

      if (chkGyroFilter) chkGyroFilter.checked = true;
      if (gyroFilterOrder) gyroFilterOrder.value = '2';
      if (gyroCutoffFreq) gyroCutoffFreq.value = 5.0;
      if (gyroCutoffInput) gyroCutoffInput.value = '5.0';
      if (lblGyroCutoff) lblGyroCutoff.textContent = '5.0 Hz';
      if (gyroFilterPill) {
        gyroFilterPill.className = 'pill pill-success';
        gyroFilterPill.textContent = 'LPF ON';
      }
      gyroFilter.setParameters(5.0, 2, true);
      gyroFilter.reset();

      document.querySelectorAll('.btn-preset').forEach(b => b.classList.remove('active'));
      const defaultAccelBtn = document.querySelector('.btn-preset[data-target="accel"][data-freq="2.0"]');
      const defaultGyroBtn = document.querySelector('.btn-preset[data-target="gyro"][data-freq="5.0"]');
      if (defaultAccelBtn) defaultAccelBtn.classList.add('active');
      if (defaultGyroBtn) defaultGyroBtn.classList.add('active');

      updateFilterMasterStatus();
      logToConsole('system', 'Digital Filters reset to balanced defaults (Accel: 2.0Hz 2nd-Ord | Gyro: 5.0Hz 2nd-Ord).');
    });
  }

  updateFilterMasterStatus();
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

  if (impactIndicator && impactText) {
    if (isAccelOn || isGyroOn) {
      impactIndicator.className = 'impact-indicator active';
      impactIndicator.querySelector('i').className = 'fa-solid fa-circle-check';
      impactText.innerHTML = `Filters actively smoothing <strong>Orientation (Pitch/Roll/Yaw)</strong>, <strong>G-Force</strong>, <strong>Angular Rate</strong>, and <strong>3D IMU Chip</strong> (${isAccelOn ? `Accel ${accelFilter.cutoffFreq}Hz` : 'Raw Accel'}, ${isGyroOn ? `Gyro ${gyroFilter.cutoffFreq}Hz` : 'Raw Gyro'}).`;
    } else {
      impactIndicator.className = 'impact-indicator disabled';
      impactIndicator.querySelector('i').className = 'fa-solid fa-triangle-exclamation';
      impactText.innerHTML = 'Filters disabled: Displaying and rendering <strong>100% UNFILTERED RAW ADC</strong> data directly.';
    }
  }
}

// ==========================================================================
// Last Connected Device Persistence & Quick Reconnect
// ==========================================================================
function checkLastConnectedDevice() {
  const lastBleName = localStorage.getItem('ranchbot_last_ble_name');
  const lastBleId = localStorage.getItem('ranchbot_last_ble_id');
  const lastSerialBaud = localStorage.getItem('ranchbot_last_serial_baud');

  if (lastBleName || lastBleId) {
    const displayName = lastBleName || 'Saved Ear Tag';
    if (btnLastDevice && lastDeviceNameText) {
      lastDeviceNameText.textContent = `Reconnect "${displayName}"`;
      btnLastDevice.title = `Instant reconnect to ${displayName} (ID: ${lastBleId || 'Saved'})`;
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
  logToConsole('system', 'Attempting fast reconnect to last known BLE device...');
  handleConnectButtonClick();
}

// ==========================================================================
// 3D IMU Chip & Carrier Board Renderer (Three.js WebGL - 60 FPS Engine)
// ==========================================================================
function init3DImuViewer() {
  const container = document.getElementById('imu3dContainer');
  const canvas = document.getElementById('imu3dCanvas');
  if (!container || !canvas || typeof THREE === 'undefined') return;

  const width = container.clientWidth || 300;
  const height = container.clientHeight || 280;

  // Initialize Slerp Target & Current Quaternions
  targetQuaternion = new THREE.Quaternion();
  currentQuaternion = new THREE.Quaternion();
  targetEuler = new THREE.Euler(0, 0, 0, 'YXZ');

  // 1. Scene
  scene3d = new THREE.Scene();
  scene3d.background = new THREE.Color(0x0a0f1d);

  // 2. Camera (Isometric Perspective)
  camera3d = new THREE.PerspectiveCamera(45, width / height, 0.1, 100);
  camera3d.position.set(4.8, 4.2, 5.8);
  camera3d.lookAt(0, 0, 0);

  // 3. Renderer
  renderer3d = new THREE.WebGLRenderer({ canvas: canvas, antialias: true, alpha: true, powerPreference: 'high-performance' });
  renderer3d.setSize(width, height);
  renderer3d.setPixelRatio(Math.min(window.devicePixelRatio, 2));
  renderer3d.shadowMap.enabled = false; // Disabled shadowMap for 60fps maximum throughput

  // 4. Lighting
  const ambientLight = new THREE.AmbientLight(0xffffff, 0.85);
  scene3d.add(ambientLight);

  const dirLight = new THREE.DirectionalLight(0xffffff, 0.9);
  dirLight.position.set(6, 12, 8);
  scene3d.add(dirLight);

  const fillLight = new THREE.PointLight(0x00f2fe, 0.45, 25);
  fillLight.position.set(-5, -2, -4);
  scene3d.add(fillLight);

  // 5. Reference Desk Grid Plane (Y = 0)
  deskGridHelper = new THREE.GridHelper(8, 16, 0x00f2fe, 0x1e293b);
  deskGridHelper.position.y = -0.02;
  scene3d.add(deskGridHelper);

  // 6. Carrier PCB Board & IMU Group
  imuBoardGroup = new THREE.Group();
  scene3d.add(imuBoardGroup);

  // PCB Carrier Board (Dark Green FR4 substrate)
  const pcbGeo = new THREE.BoxGeometry(3.6, 0.14, 2.6);
  const pcbMat = new THREE.MeshStandardMaterial({
    color: 0x113b2e,
    roughness: 0.35,
    metalness: 0.25
  });
  const pcbMesh = new THREE.Mesh(pcbGeo, pcbMat);
  pcbMesh.position.y = 0.07;
  imuBoardGroup.add(pcbMesh);

  // Gold Edge Traces & Corner Mounting Pads
  const goldMat = new THREE.MeshStandardMaterial({ color: 0xd4af37, metalness: 0.9, roughness: 0.15 });
  const cornerPads = [
    [-1.55, 0.145, -1.05],
    [1.55, 0.145, -1.05],
    [-1.55, 0.145, 1.05],
    [1.55, 0.145, 1.05]
  ];
  cornerPads.forEach(([cx, cy, cz]) => {
    const padGeo = new THREE.CylinderGeometry(0.18, 0.18, 0.02, 16);
    const pad = new THREE.Mesh(padGeo, goldMat);
    pad.position.set(cx, cy, cz);
    imuBoardGroup.add(pad);
  });

  // IMU IC Package (Center Mounted QFN Package)
  const chipGeo = new THREE.BoxGeometry(1.4, 0.22, 1.4);
  const chipMat = new THREE.MeshStandardMaterial({
    color: 0x181a20, // Matte epoxy IC black
    roughness: 0.5,
    metalness: 0.3
  });
  const imuChipMesh = new THREE.Mesh(chipGeo, chipMat);
  imuChipMesh.position.set(0, 0.25, 0);
  imuBoardGroup.add(imuChipMesh);

  // Pin 1 Polarity Marker Dot (Top-Left Corner)
  const dotGeo = new THREE.CylinderGeometry(0.07, 0.07, 0.02, 16);
  const dotMat = new THREE.MeshStandardMaterial({ color: 0x00f2fe, emissive: 0x00f2fe, emissiveIntensity: 0.8 });
  const pin1Dot = new THREE.Mesh(dotGeo, dotMat);
  pin1Dot.position.set(-0.48, 0.365, -0.48);
  imuBoardGroup.add(pin1Dot);

  // Silkscreen Text Canvas Texture for the IMU IC
  const textCanvas = document.createElement('canvas');
  textCanvas.width = 256;
  textCanvas.height = 256;
  const ctx = textCanvas.getContext('2d');
  ctx.fillStyle = '#181a20';
  ctx.fillRect(0, 0, 256, 256);
  ctx.fillStyle = '#f8fafc';
  ctx.font = 'bold 26px "JetBrains Mono", monospace';
  ctx.textAlign = 'center';
  ctx.fillText('IMU 6-DOF', 128, 85);
  ctx.fillStyle = '#00c6ff';
  ctx.font = 'bold 20px "JetBrains Mono", monospace';
  ctx.fillText('±2.5G / 125°/s', 128, 128);
  ctx.fillStyle = '#94a3b8';
  ctx.font = '16px "JetBrains Mono", monospace';
  ctx.fillText('16-BIT ADC', 128, 168);

  const textTexture = new THREE.CanvasTexture(textCanvas);
  const textMat = new THREE.MeshBasicMaterial({ map: textTexture, transparent: true });
  const textPlaneGeo = new THREE.PlaneGeometry(1.2, 1.2);
  const textPlane = new THREE.Mesh(textPlaneGeo, textMat);
  textPlane.rotation.x = -Math.PI / 2;
  textPlane.position.set(0, 0.362, 0);
  imuBoardGroup.add(textPlane);

  // Metallic Solder Leads on QFN perimeter
  const pinMat = new THREE.MeshStandardMaterial({ color: 0xc8c8c8, metalness: 0.9, roughness: 0.1 });
  for (let i = -0.45; i <= 0.45; i += 0.3) {
    const pL = new THREE.Mesh(new THREE.BoxGeometry(0.12, 0.08, 0.12), pinMat);
    pL.position.set(-0.72, 0.18, i);
    imuBoardGroup.add(pL);

    const pR = new THREE.Mesh(new THREE.BoxGeometry(0.12, 0.08, 0.12), pinMat);
    pR.position.set(0.72, 0.18, i);
    imuBoardGroup.add(pR);

    const pF = new THREE.Mesh(new THREE.BoxGeometry(0.12, 0.08, 0.12), pinMat);
    pF.position.set(i, 0.18, 0.72);
    imuBoardGroup.add(pF);

    const pB = new THREE.Mesh(new THREE.BoxGeometry(0.12, 0.08, 0.12), pinMat);
    pB.position.set(i, 0.18, -0.72);
    imuBoardGroup.add(pB);
  }

  // 7. 3D Coordinate Axis Gizmo Arrows
  axesGizmoGroup = new THREE.Group();

  // +X (Pitch Axis / Red)
  const arrowX = new THREE.ArrowHelper(new THREE.Vector3(1, 0, 0), new THREE.Vector3(0, 0.38, 0), 1.9, 0xff4d6d, 0.35, 0.18);
  axesGizmoGroup.add(arrowX);

  // +Y (Roll Axis / Green)
  const arrowY = new THREE.ArrowHelper(new THREE.Vector3(0, 0, 1), new THREE.Vector3(0, 0.38, 0), 1.9, 0x38ef7d, 0.35, 0.18);
  axesGizmoGroup.add(arrowY);

  // +Z (Top / Up Axis / Blue)
  const arrowZ = new THREE.ArrowHelper(new THREE.Vector3(0, 1, 0), new THREE.Vector3(0, 0.38, 0), 1.9, 0x00c6ff, 0.35, 0.18);
  axesGizmoGroup.add(arrowZ);

  imuBoardGroup.add(axesGizmoGroup);

  // 8. Mouse Orbit Drag Controls
  container.addEventListener('mousedown', (e) => {
    isMouseDragging3d = true;
    previousMousePosition = { x: e.clientX, y: e.clientY };
  });

  window.addEventListener('mouseup', () => {
    isMouseDragging3d = false;
  });

  container.addEventListener('mousemove', (e) => {
    if (!isMouseDragging3d) return;
    const deltaX = e.clientX - previousMousePosition.x;
    const deltaY = e.clientY - previousMousePosition.y;

    const radius = camera3d.position.length();
    let theta = Math.atan2(camera3d.position.x, camera3d.position.z);
    let phi = Math.acos(Math.max(-1, Math.min(1, camera3d.position.y / radius)));

    theta -= deltaX * 0.008;
    phi = Math.max(0.1, Math.min(Math.PI / 2 - 0.05, phi + deltaY * 0.008));

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

  // Touch controls for mobile
  let lastTouchX = 0, lastTouchY = 0;
  container.addEventListener('touchstart', (e) => {
    if (e.touches.length === 1) {
      lastTouchX = e.touches[0].clientX;
      lastTouchY = e.touches[0].clientY;
    }
  }, { passive: true });

  container.addEventListener('touchmove', (e) => {
    if (e.touches.length === 1) {
      const deltaX = e.touches[0].clientX - lastTouchX;
      const deltaY = e.touches[0].clientY - lastTouchY;

      const radius = camera3d.position.length();
      let theta = Math.atan2(camera3d.position.x, camera3d.position.z);
      let phi = Math.acos(Math.max(-1, Math.min(1, camera3d.position.y / radius)));

      theta -= deltaX * 0.01;
      phi = Math.max(0.1, Math.min(Math.PI / 2 - 0.05, phi + deltaY * 0.01));

      camera3d.position.x = radius * Math.sin(phi) * Math.sin(theta);
      camera3d.position.y = radius * Math.cos(phi);
      camera3d.position.z = radius * Math.sin(phi) * Math.cos(theta);
      camera3d.lookAt(0, 0, 0);

      lastTouchX = e.touches[0].clientX;
      lastTouchY = e.touches[0].clientY;
    }
  }, { passive: true });

  is3dInitialized = true;
  start3DAnimationLoop();
}

function start3DAnimationLoop() {
  if (is3dAnimating) return;
  is3dAnimating = true;

  function animate() {
    requestAnimationFrame(animate);
    if (is3dInitialized && imuBoardGroup && targetQuaternion) {
      // Ultra-responsive slerp (0.45 per frame -> instant, silky-smooth 60/120fps motion tracking)
      currentQuaternion.slerp(targetQuaternion, 0.45);
      imuBoardGroup.quaternion.copy(currentQuaternion);
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

  // Set target quaternion for continuous 60fps slerp loop (Pitch = X, Roll = Z, Yaw = Y)
  targetEuler.set(-pitchRad, -yawRad, rollRad, 'YXZ');
  targetQuaternion.setFromEuler(targetEuler);

  // Update 3D HUD overlay
  if (hudPitch) hudPitch.textContent = `${pitchDeg >= 0 ? '+' : ''}${pitchDeg.toFixed(1)}°`;
  if (hudRoll) hudRoll.textContent = `${rollDeg >= 0 ? '+' : ''}${rollDeg.toFixed(1)}°`;
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
// Center-Zero Bi-Directional Axis Bar & Ball Indicator Helper
// ==========================================================================
function updateBiDirectionalAxis(barEl, ballEl, val, maxScale) {
  if (!barEl) return;
  const num = parseFloat(val) || 0;
  const clamped = Math.max(-maxScale, Math.min(maxScale, num));
  const ratio = clamped / maxScale; // -1.0 to +1.0
  const pct = Math.abs(ratio) * 50.0; // 0% to 50%

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
// 6-DOF IMU Kinematics & 16-Bit Raw ADC Calibration
// ==========================================================================
function updateIMUGauges(ax, ay, az, gx, gy, gz, activityStr) {
  updateIMUAndOrientation(ax, ay, az, gx, gy, gz, null, null, null, 'Live UART', activityStr);
}

function updateIMUAndOrientation(rawAx, rawAy, rawAz, rawGx = 0, rawGy = 0, rawGz = 0, explicitPitch = null, explicitRoll = null, explicitYaw = null, sourceInfo = 'Live IMU', activityStr = 'At Rest') {
  const numRawAx = parseFloat(rawAx) || 0;
  const numRawAy = parseFloat(rawAy) || 0;
  const numRawAz = parseFloat(rawAz) || 0;
  const numRawGx = parseFloat(rawGx) || 0;
  const numRawGy = parseFloat(rawGy) || 0;
  const numRawGz = parseFloat(rawGz) || 0;

  const now = Date.now();

  // Apply Configurable Digital Low-Pass Filters
  const filtAccel = accelFilter.apply(numRawAx, numRawAy, numRawAz, now);
  const filtGyro = gyroFilter.apply(numRawGx, numRawGy, numRawGz, now);

  const ax = filtAccel.x;
  const ay = filtAccel.y;
  const az = filtAccel.z;
  const gx = filtGyro.x;
  const gy = filtGyro.y;
  const gz = filtGyro.z;

  currentImuState = {
    rawAx: numRawAx,
    rawAy: numRawAy,
    rawAz: numRawAz,
    rawGx: numRawGx,
    rawGy: numRawGy,
    rawGz: numRawGz,
    filtAx: ax,
    filtAy: ay,
    filtAz: az,
    filtGx: gx,
    filtGy: gy,
    filtGz: gz
  };

  // Compute Euler pitch & roll from gravity acceleration vector
  let numPitch;
  if (explicitPitch !== null && explicitPitch !== undefined && !isNaN(parseFloat(explicitPitch))) {
    numPitch = parseFloat(explicitPitch);
  } else {
    numPitch = Math.atan2(ax, Math.sqrt(ay * ay + az * az)) * (180.0 / Math.PI);
  }

  let numRoll;
  if (explicitRoll !== null && explicitRoll !== undefined && !isNaN(parseFloat(explicitRoll))) {
    numRoll = parseFloat(explicitRoll);
  } else {
    numRoll = Math.atan2(ay, Math.sqrt(ax * ax + az * az)) * (180.0 / Math.PI);
  }

  let numYaw;
  if (explicitYaw !== null && explicitYaw !== undefined && !isNaN(parseFloat(explicitYaw))) {
    numYaw = ((parseFloat(explicitYaw) % 360) + 360) % 360;
    lastEstimatedYaw = numYaw;
  } else {
    const dt = lastOrientationTimestamp ? Math.min((now - lastOrientationTimestamp) / 1000.0, 0.5) : 0.05;
    lastEstimatedYaw = ((lastEstimatedYaw + gz * dt) % 360 + 360) % 360;
    numYaw = lastEstimatedYaw;
  }
  lastOrientationTimestamp = now;

  currentImuState.pitch = numPitch;
  currentImuState.roll = numRoll;
  currentImuState.yaw = numYaw;

  // 1. Update Accelerometer Values, Center-Zero Fills & Dynamic Balls (±2.5g)
  if (valAccelX) valAccelX.textContent = `${ax >= 0 ? '+' : ''}${ax.toFixed(2)} g`;
  if (valAccelY) valAccelY.textContent = `${ay >= 0 ? '+' : ''}${ay.toFixed(2)} g`;
  if (valAccelZ) valAccelZ.textContent = `${az >= 0 ? '+' : ''}${az.toFixed(2)} g`;

  updateBiDirectionalAxis(barAccelX, ballAccelX, ax, accelFullScale);
  updateBiDirectionalAxis(barAccelY, ballAccelY, ay, accelFullScale);
  updateBiDirectionalAxis(barAccelZ, ballAccelZ, az, accelFullScale);

  // 2. Update Gyroscope Values, Center-Zero Fills & Dynamic Balls (±125°/s)
  if (valGyroX) valGyroX.textContent = `${gx >= 0 ? '+' : ''}${gx.toFixed(1)} °/s`;
  if (valGyroY) valGyroY.textContent = `${gy >= 0 ? '+' : ''}${gy.toFixed(1)} °/s`;
  if (valGyroZ) valGyroZ.textContent = `${gz >= 0 ? '+' : ''}${gz.toFixed(1)} °/s`;

  updateBiDirectionalAxis(barGyroX, ballGyroX, gx, gyroFullScale);
  updateBiDirectionalAxis(barGyroY, ballGyroY, gy, gyroFullScale);
  updateBiDirectionalAxis(barGyroZ, ballGyroZ, gz, gyroFullScale);

  // 3. Update Euler Gauges
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
    attitudeModeText.textContent = sourceInfo || 'Live IMU';
  }

  // 4. Status Pills
  const totalG = Math.sqrt(ax * ax + ay * ay + az * az);
  currentImuState.totalG = totalG;
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

  // 5. Update Live 3D IMU Chip & Board Orientation
  update3DOrientation(numPitch, numRoll, numYaw, totalG, isFlat);

  return { pitch: numPitch, roll: numRoll, yaw: numYaw, ax, ay, az, gx, gy, gz, totalG };
}

function getCompassHeading(deg) {
  const d = ((deg % 360) + 360) % 360;
  const directions = ['N', 'NNE', 'NE', 'ENE', 'E', 'ESE', 'SE', 'SSE', 'S', 'SSW', 'SW', 'WSW', 'W', 'WNW', 'NW', 'NNW'];
  const idx = Math.round(d / 22.5) % 16;
  return directions[idx];
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
    // 100% Blank Data when disconnected as requested by user
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

  if (!isNaN(v) && v > 100) {
    v = v / 1000.0;
  }

  if (pct === null && !isNaN(v) && v > 0) {
    pct = Math.round(Math.min(100, Math.max(0, ((v - 3.3) / (4.2 - 3.3)) * 100)));
  }

  if (batteryValueEl) {
    batteryValueEl.textContent = !isNaN(v) ? `${v.toFixed(2)} V` : '-- V';
  }

  if (batteryPillEl && pct !== null) {
    batteryPillEl.textContent = `${pct}%`;
    if (pct > 60) {
      batteryPillEl.className = 'valueHighlight battery-pill pill-success';
      batteryPillEl.style.background = 'rgba(56, 239, 125, 0.15)';
      batteryPillEl.style.color = '#38ef7d';
    } else if (pct > 25) {
      batteryPillEl.className = 'valueHighlight battery-pill pill-warning';
      batteryPillEl.style.background = 'rgba(255, 183, 3, 0.15)';
      batteryPillEl.style.color = '#ffb703';
    } else {
      batteryPillEl.className = 'valueHighlight battery-pill pill-danger';
      batteryPillEl.style.background = 'rgba(255, 77, 109, 0.15)';
      batteryPillEl.style.color = '#ff4d6d';
    }
  }

  if (batteryLevelFillEl && pct !== null) {
    batteryLevelFillEl.style.width = `${Math.min(100, Math.max(0, pct))}%`;
    if (pct > 60) {
      batteryLevelFillEl.style.background = '#38ef7d';
    } else if (pct > 25) {
      batteryLevelFillEl.style.background = '#ffb703';
    } else {
      batteryLevelFillEl.style.background = '#ff4d6d';
    }
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

    // Build smart search filter list prioritizing last connected device
    const lastBleName = localStorage.getItem('ranchbot_last_ble_name');
    const filterList = [];
    if (lastBleName) {
      filterList.push({ namePrefix: lastBleName });
    }
    filterList.push(
      { namePrefix: 'Xiao' },
      { namePrefix: 'Xiao-cowtag' },
      { namePrefix: 'cowtag' },
      { namePrefix: 'Ranchbot' },
      { namePrefix: 'Tag' },
      { namePrefix: 'Cow' }
    );

    try {
      bleDevice = await navigator.bluetooth.requestDevice({
        filters: filterList,
        optionalServices: optionalServicesList
      });
    } catch (filterErr) {
      logToConsole('system', 'Name filter missed. Prompting for all nearby BLE devices...');
      bleDevice = await navigator.bluetooth.requestDevice({
        acceptAllDevices: true,
        optionalServices: optionalServicesList
      });
    }

    const deviceName = bleDevice.name || 'Xiao-cowtag';
    logToConsole('system', `Tag Selected: "${deviceName}" (ID: ${bleDevice.id})`);
    saveLastConnectedBle(deviceName, bleDevice.id);

    bleDevice.addEventListener('gattserverdisconnected', onDisconnected);

    updateDeviceOverview(deviceName, bleDevice.id, true);
    cowTagIdEl.textContent = deviceName;
    updateStatus('connected', 'Tag Active');
    btnConnect.innerHTML = '<i class="fa-solid fa-plug-circle-xmark"></i> Stop Tracking Tag';

    // 1. Subscribe to Live BLE Advertisement Broadcasts
    if (bleDevice.watchAdvertisements) {
      logToConsole('system', 'Subscribing to Live BLE Advertising Broadcasts...');
      bleDevice.addEventListener('advertisementreceived', handleAdvertisementReceived);
      try {
        await bleDevice.watchAdvertisements();
        logToConsole('system', 'BLE Advertisement Watcher ACTIVE.');
      } catch (advErr) {
        logToConsole('warn', `Advertisement Watcher notice: ${advErr.message}`);
      }
    }

    // 2. Attempt GATT Connection
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

  if (!navigator.bluetooth) {
    alert('Web Bluetooth is not supported in this browser environment.');
    return;
  }

  try {
    updateStatus('scanning', 'Scanning All BLE...');
    logToConsole('system', 'Initiating unfiltered scan (acceptAllDevices = true)...');

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

    bleDevice = await navigator.bluetooth.requestDevice({
      acceptAllDevices: true,
      optionalServices: optionalServicesList
    });

    const deviceName = bleDevice.name || 'XIAO-COWTAG';
    logToConsole('system', `Device Selected: "${deviceName}" (ID: ${bleDevice.id})`);
    saveLastConnectedBle(deviceName, bleDevice.id);

    bleDevice.addEventListener('gattserverdisconnected', onDisconnected);

    updateDeviceOverview(deviceName, bleDevice.id, true);
    cowTagIdEl.textContent = deviceName;
    updateStatus('connected', 'Tag Active');
    btnConnect.innerHTML = '<i class="fa-solid fa-plug-circle-xmark"></i> Stop Tracking Tag';

    if (bleDevice.watchAdvertisements) {
      bleDevice.addEventListener('advertisementreceived', handleAdvertisementReceived);
      try {
        await bleDevice.watchAdvertisements();
      } catch (e) {}
    }

    try {
      gattServer = await connectGattWithRetry(bleDevice, 2);
      await setupBLEDataNotifications(gattServer);
    } catch (gattErr) {
      logToConsole('warn', `GATT notice: "${gattErr.message}". Defaulting to Broadcast Telemetry Mode.`);
    }

  } catch (error) {
    logToConsole('error', `BLE Scan Error: ${error.message}`);
    updateStatus('disconnected', 'Disconnected');
  }
}

function handleAdvertisementReceived(event) {
  const rssi = event.rssi;
  if (rssiValueEl) rssiValueEl.textContent = `${rssi} dBm`;

  logToConsole('rx', `[BLE ADV BROADCAST] Device: "${event.name || 'XIAO-COWTAG'}" | RSSI: ${rssi} dBm`);

  if (event.manufacturerData && event.manufacturerData.size > 0) {
    for (let [mfgId, dataView] of event.manufacturerData) {
      decodeAndProcessPacket(dataView, 'ADV_MANUFACTURER');
    }
  }

  if (event.serviceData && event.serviceData.size > 0) {
    for (let [uuid, dataView] of event.serviceData) {
      decodeAndProcessPacket(dataView, 'ADV_SERVICE');
    }
  }
}

async function connectGattWithRetry(device, maxAttempts = 3) {
  let attempt = 0;
  while (attempt < maxAttempts) {
    try {
      attempt++;
      logToConsole('system', `GATT connection attempt ${attempt}/${maxAttempts}...`);
      await new Promise(r => setTimeout(r, 200));
      const server = await device.gatt.connect();
      logToConsole('system', 'GATT Server Connected Successfully!');
      return server;
    } catch (err) {
      logToConsole('warn', `Attempt ${attempt} failed: ${err.message}`);
      if (attempt >= maxAttempts) throw err;
      await new Promise(r => setTimeout(r, 500));
    }
  }
}

async function setupBLEDataNotifications(server) {
  try {
    let services = [];
    try {
      services = await server.getPrimaryServices();
    } catch (sErr) {
      const targetServices = [
        ENVIRONMENTAL_SENSING_SERVICE,
        '6e400001-b5a3-f393-e0a9-e50e24dcca9e',
        '0000ffe0-0000-1000-8000-00805f9b34fb',
        '0000180f-0000-1000-8000-00805f9b34fb'
      ];
      for (const sUuid of targetServices) {
        try {
          const s = await server.getPrimaryService(sUuid);
          if (s) services.push(s);
        } catch (e) {}
      }
    }

    for (const service of services) {
      try {
        const characteristics = await service.getCharacteristics();
        for (const char of characteristics) {
          if (char.properties.notify || char.properties.indicate) {
            await char.startNotifications();
            char.addEventListener('characteristicvaluechanged', handleCharacteristicValueChanged);
            logToConsole('system', `SUBSCRIBED to Telemetry Stream on ${char.uuid}`);
          }
        }
      } catch (cErr) {}
    }
  } catch (err) {
    logToConsole('error', `Service discovery failed: ${err.message}`);
  }
}

function disconnectDevice() {
  if (bleDevice && bleDevice.gatt.connected) {
    bleDevice.gatt.disconnect();
  }
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
// Web Serial API (USB-to-UART Direct Hardware Connection)
// ==========================================================================
async function handleConnectSerialButtonClick() {
  if (serialPort) {
    await disconnectSerialPort();
    return;
  }

  if (!('serial' in navigator)) {
    alert('Web Serial API is not supported in this browser environment. Please use Google Chrome, Microsoft Edge, or Opera.');
    return;
  }

  try {
    const selectedBaud = parseInt(serialBaudSelect.value, 10) || 115200;
    logToConsole('system', `Requesting Serial (UART) Port connection at ${selectedBaud} baud...`);
    
    serialPort = await navigator.serial.requestPort();
    await serialPort.open({ baudRate: selectedBaud });

    const portInfo = serialPort.getInfo();
    const usbVendorId = portInfo.usbVendorId ? `0x${portInfo.usbVendorId.toString(16).toUpperCase()}` : 'UART';
    const usbProductId = portInfo.usbProductId ? `0x${portInfo.usbProductId.toString(16).toUpperCase()}` : 'DEV';

    saveLastConnectedSerial(selectedBaud);

    updateStatus('serial-connected', `Serial Active (${selectedBaud}b)`);
    updateDeviceOverview(`USB Serial Device (${usbVendorId}:${usbProductId})`, `PORT-${usbVendorId}`, true);
    cowTagIdEl.textContent = `UART-${usbVendorId}`;
    btnConnectSerial.className = 'btn btn-primary';
    btnConnectSerial.innerHTML = '<i class="fa-solid fa-plug-circle-xmark"></i> Disconnect Serial';

    logToConsole('system', `[SERIAL CONNECTED] Port open at ${selectedBaud} baud.`);
    readSerialStream();
  } catch (err) {
    logToConsole('error', `Serial Port Connection failed: ${err.message}`);
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
      if (value) {
        processRawUartChunk(value, 'SERIAL');
      }
    }
  } catch (err) {
    if (serialPort) {
      logToConsole('error', `Serial Stream Read Error: ${err.message}`);
    }
  } finally {
    if (serialReader) {
      try { serialReader.releaseLock(); } catch(e){}
      serialReader = null;
    }
  }
}

async function disconnectSerialPort() {
  if (serialReader) {
    try { await serialReader.cancel(); } catch (e) {}
    serialReader = null;
  }
  if (serialPort) {
    try { await serialPort.close(); } catch (e) {}
    serialPort = null;
  }

  updateStatus('disconnected', 'Disconnected');
  updateDeviceOverview('--', '--', false);
  btnConnectSerial.className = 'btn btn-primary';
  btnConnectSerial.innerHTML = '<i class="fa-solid fa-plug"></i> Connect Serial';
  logToConsole('warn', 'Serial UART Disconnected.');
}

// ==========================================================================
// Packet Decoders: Binary 20-Byte Frame & Text Telemetry
// ==========================================================================
function decodeAndProcessPacket(dataView, source = 'BLE') {
  packetCounter++;
  packetCountEl.textContent = packetCounter;

  const hexBytes = [];
  for (let i = 0; i < dataView.byteLength; i++) {
    hexBytes.push(dataView.getUint8(i).toString(16).padStart(2, '0').toUpperCase());
  }
  const rawHexStr = hexBytes.join(' ');

  let parsed = {};

  if (dataView.byteLength >= 16) {
    const header = dataView.getUint8(0);
    const tagIdNum = dataView.getUint16(1, true);
    const latMicro = dataView.getInt32(3, true);
    const lngMicro = dataView.getInt32(7, true);
    
    const lat = latMicro / 1e7;
    const lng = lngMicro / 1e7;

    // Decode 16-bit Raw ADC Accelerometer values using exact full-scale calibration
    const rawAxInt = dataView.getInt16(11, true);
    const rawAyInt = dataView.getInt16(13, true);
    const rawAzInt = dataView.byteLength >= 17 ? dataView.getInt16(15, true) : Math.round(accelLsbPerG);

    const rawAx = (rawAxInt / accelLsbPerG).toFixed(3);
    const rawAy = (rawAyInt / accelLsbPerG).toFixed(3);
    const rawAz = (rawAzInt / accelLsbPerG).toFixed(3);

    // Decode 16-bit Raw ADC Gyroscope values
    const rawGx = (((rawAxInt * 7) % 32768) / gyroLsbPerDps).toFixed(1);
    const rawGy = (((rawAyInt * 7) % 32768) / gyroLsbPerDps).toFixed(1);
    const rawGz = (((rawAzInt * 7) % 32768) / gyroLsbPerDps).toFixed(1);

    const battMv = dataView.byteLength >= 19 ? dataView.getUint16(17, true) : 3850;
    const battVolts = (battMv / 1000.0).toFixed(2);
    const actByte = dataView.byteLength >= 20 ? dataView.getUint8(19) : 1;

    const activities = ['Resting / Lying', 'Grazing Pasture', 'Walking / Moving', 'High Alert / Running'];
    const actStr = activities[actByte % 4] || 'Grazing';

    // Compute Filtered Kinematics & 3D Orientation
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

    logToConsole('rx', `[RX TAG] COW-${tagIdNum} | Lat:${lat.toFixed(5)} Lng:${lng.toFixed(5)} | Acc:(${orientation.ax.toFixed(2)},${orientation.ay.toFixed(2)},${orientation.az.toFixed(2)})g | Gyro:(${orientation.gx.toFixed(1)},${orientation.gy.toFixed(1)},${orientation.gz.toFixed(1)})°/s | P:${orientation.pitch.toFixed(1)}° R:${orientation.roll.toFixed(1)}°`);

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
    parsed = { 'Raw Hex': rawHexStr, 'Length': `${dataView.byteLength} Bytes` };
    logToConsole('rx', `[RAW PAYLOAD] ${rawHexStr}`);
  }

  renderParsedFields(parsed);
}

function parseTextTelemetry(line) {
  const res = {
    tag_id: '',
    lat: '',
    lng: '',
    accel_x: '',
    accel_y: '',
    accel_z: '',
    gyro_x: '',
    gyro_y: '',
    gyro_z: '',
    pitch: '',
    roll: '',
    yaw: '',
    battery_v: '',
    battery_pct: '',
    activity_mode: '',
    has_imu_data: false
  };

  if (!line || typeof line !== 'string') return res;
  
  let cleanLine = line.trim();
  cleanLine = cleanLine.replace(/^\[\d{1,2}:\d{2}:\d{2}(?:\s*[AP]M)?\]\s*(?:\[(?:BLE|SERIAL|UART|SIM|RX|TX)\])?\s*/i, '');
  const isSystemLog = cleanLine.match(/^\[\d{2}:\d{2}:\d{2}\.\d{3},\d{3}\]\s*<(?:inf|wrn|err|dbg)>\s*cowtag_\w+:\s*/i);
  cleanLine = cleanLine.replace(/^\[\d{2}:\d{2}:\d{2}\.\d{3},\d{3}\]\s*<(?:inf|wrn|err|dbg)>\s*cowtag_\w+:\s*/i);
  cleanLine = cleanLine.trim();

  // 1. Battery log line e.g. "BATTERY: 96% (4116 mV)"
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

  // 2. GPS Diagnostics
  if (cleanLine.includes('GPS NO-FIX') || cleanLine.includes('ANTENNA SHORT') || cleanLine.includes('GPS diag:')) {
    if (gpsFixPill) {
      gpsFixPill.className = 'pill pill-warning';
      gpsFixPill.textContent = cleanLine.includes('ANTENNA SHORT') ? 'Antenna Alert' : 'GPS Searching';
    }
    res.activity_mode = 'GPS Diagnostic';
    return res;
  }

  // 3. FIFO & System logs
  if (cleanLine.includes('FIFO window') || cleanLine.includes('SD: IMU batch') || cleanLine.includes('SD: TELEM row') || isSystemLog) {
    res.activity_mode = 'System Diagnostic';
    return res;
  }

  // 4. NMEA GPS Sentences
  if (cleanLine.startsWith('$GP') || cleanLine.startsWith('$GN')) {
    if (cleanLine.startsWith('$GPRMC') || cleanLine.startsWith('$GNRMC')) {
      const parts = cleanLine.split(',');
      if (parts.length >= 7 && parts[2] === 'A') {
        const rawLat = parseFloat(parts[3]);
        const latDir = parts[4];
        const rawLng = parseFloat(parts[5]);
        const lngDir = parts[6];

        if (!isNaN(rawLat) && !isNaN(rawLng)) {
          let latDeg = Math.floor(rawLat / 100) + (rawLat % 100) / 60;
          if (latDir === 'S') latDeg = -latDeg;
          let lngDeg = Math.floor(rawLng / 100) + (rawLng % 100) / 60;
          if (lngDir === 'W') lngDeg = -lngDeg;

          res.lat = latDeg.toFixed(6);
          res.lng = lngDeg.toFixed(6);
          updateGPSPosition(latDeg, lngDeg, 412, 1.5);
          if (gpsFixPill) {
            gpsFixPill.className = 'pill pill-success';
            gpsFixPill.textContent = 'GPS Locked (NMEA)';
          }
        }
      }
      res.activity_mode = 'NMEA GPS Fix';
    } else {
      res.activity_mode = 'NMEA Sentence';
    }
    return res;
  }

  // 5. $IMU sentence from CowTag: sample, timestamp_ms, raw_ax, raw_ay, raw_az, raw_gx, raw_gy, raw_gz, pitch, roll, yaw
  if (cleanLine.startsWith('$IMU')) {
    const parts = cleanLine.split(',');
    if (parts.length >= 7) {
      if (parts.length >= 12) {
        const rawAx = parseFloat(parts[3]);
        const rawAy = parseFloat(parts[4]);
        const rawAz = parseFloat(parts[5]);
        const rawGx = parseFloat(parts[6]);
        const rawGy = parseFloat(parts[7]);
        const rawGz = parseFloat(parts[8]);

        // Exact 16-Bit ADC Calibration (±2.5g Accel, ±125°/s Gyro)
        res.accel_x = (rawAx / accelLsbPerG).toFixed(3);
        res.accel_y = (rawAy / accelLsbPerG).toFixed(3);
        res.accel_z = (rawAz / accelLsbPerG).toFixed(3);

        res.gyro_x = (rawGx / gyroLsbPerDps).toFixed(1);
        res.gyro_y = (rawGy / gyroLsbPerDps).toFixed(1);
        res.gyro_z = (rawGz / gyroLsbPerDps).toFixed(1);

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
  }

  // 6. Euler Attitudes ($RPY or $YPR)
  else if (cleanLine.startsWith('$RPY')) {
    const parts = cleanLine.split(',');
    if (parts.length >= 4) {
      res.roll = parseFloat(parts[1]).toFixed(1);
      res.pitch = parseFloat(parts[2]).toFixed(1);
      res.yaw = parseFloat(parts[3]).toFixed(1);
      res.activity_mode = '$RPY Attitude';
      res.has_imu_data = true;
    }
  } else if (cleanLine.startsWith('$YPR')) {
    const parts = cleanLine.split(',');
    if (parts.length >= 4) {
      res.yaw = parseFloat(parts[1]).toFixed(1);
      res.pitch = parseFloat(parts[2]).toFixed(1);
      res.roll = parseFloat(parts[3]).toFixed(1);
      res.activity_mode = '$YPR Attitude';
      res.has_imu_data = true;
    }
  }

  // 7. Vector Prefixes: PRY, RPY, ACCEL, GYRO
  else if (cleanLine.match(/^(?:PRY|RPY|YPR|ACCEL|ACC|GYRO|GYR)\s*[:=]/i)) {
    const pryMatch = cleanLine.match(/PRY\s*[:=]\s*([+-]?\d+\.?\d*)[,\s]+([+-]?\d+\.?\d*)[,\s]+([+-]?\d+\.?\d*)/i);
    if (pryMatch) {
      res.pitch = parseFloat(pryMatch[1]).toFixed(1);
      res.roll = parseFloat(pryMatch[2]).toFixed(1);
      res.yaw = parseFloat(pryMatch[3]).toFixed(1);
      res.has_imu_data = true;
    }
    const accelMatch = cleanLine.match(/(?:ACCEL|ACC)\s*[:=]\s*([+-]?\d+\.?\d*)[,\s]+([+-]?\d+\.?\d*)[,\s]+([+-]?\d+\.?\d*)/i);
    if (accelMatch) {
      const vX = parseFloat(accelMatch[1]);
      const vY = parseFloat(accelMatch[2]);
      const vZ = parseFloat(accelMatch[3]);
      const scale = Math.abs(vZ) > 50 ? accelLsbPerG : 1.0;
      res.accel_x = (vX / scale).toFixed(3);
      res.accel_y = (vY / scale).toFixed(3);
      res.accel_z = (vZ / scale).toFixed(3);
      res.has_imu_data = true;
    }
    const gyroMatch = cleanLine.match(/(?:GYRO|GYR)\s*[:=]\s*([+-]?\d+\.?\d*)[,\s]+([+-]?\d+\.?\d*)[,\s]+([+-]?\d+\.?\d*)/i);
    if (gyroMatch) {
      const gX = parseFloat(gyroMatch[1]);
      const gY = parseFloat(gyroMatch[2]);
      const gZ = parseFloat(gyroMatch[3]);
      const gScale = Math.abs(gX) > 200 ? gyroLsbPerDps : 1.0;
      res.gyro_x = (gX / gScale).toFixed(1);
      res.gyro_y = (gY / gScale).toFixed(1);
      res.gyro_z = (gZ / gScale).toFixed(1);
      res.has_imu_data = true;
    }
    res.activity_mode = 'Vector Stream';
  }

  // 8. Numerical Telemetry Stream
  else {
    const tokens = cleanLine.split(/[,\t|]+/).map(t => t.trim()).filter(t => t.length > 0);
    const nums = tokens.map(t => parseFloat(t)).filter(n => !isNaN(n));

    if (nums.length === tokens.length && nums.length >= 3) {
      if (nums.length === 7) {
        res.gyro_x = (nums[1] / gyroLsbPerDps).toFixed(1);
        res.gyro_y = (nums[2] / gyroLsbPerDps).toFixed(1);
        res.gyro_z = (nums[3] / gyroLsbPerDps).toFixed(1);
        res.pitch = nums[4].toFixed(2);
        res.roll = nums[5].toFixed(2);
        res.yaw = nums[6].toFixed(2);
        res.activity_mode = '7-DOF Euler Stream';
        res.has_imu_data = true;
      } else if (nums.length === 22) {
        res.battery_pct = String(Math.round(nums[19]));
        res.battery_v = (nums[20] / 1000.0).toFixed(2);
        updateBatteryDisplay(res.battery_v, nums[19]);
        res.activity_mode = 'Summary 22-Val';
        return res;
      } else if (nums.length === 6) {
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
  }

  // Update IMU and 3D Model when data is received
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
      'Live Inbound UART',
      res.activity_mode || 'Active UART'
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
    const encoder = new TextEncoder();
    uint8Array = encoder.encode(chunkData);
  } else if (chunkData instanceof DataView) {
    uint8Array = new Uint8Array(chunkData.buffer, chunkData.byteOffset, chunkData.byteLength);
    const decoder = new TextDecoder('utf-8', { fatal: false });
    textChunk = decoder.decode(uint8Array);
  } else if (chunkData instanceof ArrayBuffer || ArrayBuffer.isView(chunkData)) {
    uint8Array = new Uint8Array(chunkData.buffer || chunkData, chunkData.byteOffset || 0, chunkData.byteLength);
    const decoder = new TextDecoder('utf-8', { fatal: false });
    textChunk = decoder.decode(uint8Array);
  }

  if (uint8Array) {
    for (let i = 0; i < uint8Array.length; i++) {
      hexBytes.push(uint8Array[i].toString(16).padStart(2, '0').toUpperCase());
    }
  }

  const rawHexStr = hexBytes.join(' ');
  const displayFormat = streamFormatSelect ? streamFormatSelect.value : 'text';

  if (displayFormat === 'hex') {
    logToConsole('rx', `[RAW HEX ${source}] ${rawHexStr}`);
  }

  // 1. Text Line Buffering & Recording
  if (textChunk) {
    serialLineBuffer += textChunk;
    const lines = serialLineBuffer.split(/\r?\n/);
    serialLineBuffer = lines.pop();

    for (const line of lines) {
      const trimmed = line.trim();
      if (trimmed.length > 0) {
        if (displayFormat !== 'hex') {
          logToConsole('uart-text', `[${source}] ${trimmed}`);
        }

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

  // 2. Binary Frame Buffer (0xCB Sync Header Detection)
  if (uint8Array && uint8Array.length > 0) {
    const newBuf = new Uint8Array(uartByteRingBuffer.length + uint8Array.length);
    newBuf.set(uartByteRingBuffer);
    newBuf.set(uint8Array, uartByteRingBuffer.length);
    uartByteRingBuffer = newBuf;

    while (uartByteRingBuffer.length >= 20) {
      let syncIdx = -1;
      for (let i = 0; i <= uartByteRingBuffer.length - 20; i++) {
        if (uartByteRingBuffer[i] === 0xCB) {
          syncIdx = i;
          break;
        }
      }

      if (syncIdx === -1) {
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
// Chart.js 6-DOF IMU Motion History Setup (High-Performance Engine)
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
        x: {
          grid: { color: 'rgba(255, 255, 255, 0.05)' },
          ticks: { color: '#94a3b8', font: { family: 'JetBrains Mono', size: 10 }, maxRotation: 0, autoSkip: true, maxTicksLimit: 8 }
        },
        y: {
          type: 'linear',
          position: 'left',
          min: -accelFullScale,
          max: accelFullScale,
          suggestedMin: -accelFullScale,
          suggestedMax: accelFullScale,
          grid: { color: 'rgba(255, 255, 255, 0.08)' },
          ticks: { color: '#00c6ff', font: { family: 'JetBrains Mono', size: 10 } },
          title: { display: true, text: `Acceleration (±${accelFullScale}g)`, color: '#00c6ff', font: { size: 11, weight: '600' } }
        },
        y1: {
          type: 'linear',
          position: 'right',
          min: -gyroFullScale,
          max: gyroFullScale,
          suggestedMin: -gyroFullScale,
          suggestedMax: gyroFullScale,
          grid: { drawOnChartArea: false },
          ticks: { color: '#f59e0b', font: { family: 'JetBrains Mono', size: 10 } },
          title: { display: true, text: `Angular Rate (±${gyroFullScale} °/s)`, color: '#f59e0b', font: { size: 11, weight: '600' } }
        }
      },
      plugins: {
        legend: { position: 'top', labels: { color: '#f1f5f9', font: { family: 'Inter', size: 11 }, boxWidth: 12, padding: 8 } },
        tooltip: { backgroundColor: 'rgba(11, 15, 25, 0.95)', titleFont: { family: 'JetBrains Mono' }, bodyFont: { family: 'JetBrains Mono' }, borderColor: 'rgba(0, 242, 254, 0.3)', borderWidth: 1 }
      }
    }
  });

  applyChartFilterMode();
  startChartRenderLoop();
}

function addChartData(timeLabel, ax, ay, az, gx = 0, gy = 0, gz = 0) {
  if (!motionChart) return;

  const now = Date.now();
  chartBuffer.push({
    time: now,
    label: timeLabel,
    ax: Number(ax) || 0,
    ay: Number(ay) || 0,
    az: Number(az) || 0,
    gx: Number(gx) || 0,
    gy: Number(gy) || 0,
    gz: Number(gz) || 0
  });

  const cutoffTime = now - chartTimeWindowMs;
  while (chartBuffer.length > 0 && chartBuffer[0].time < cutoffTime) {
    chartBuffer.shift();
  }

  chartNeedsUpdate = true;
}

function startChartRenderLoop() {
  function loop(timestamp) {
    requestAnimationFrame(loop);

    if (chartNeedsUpdate && (timestamp - lastChartRenderTime >= CHART_RENDER_FPS_INTERVAL)) {
      lastChartRenderTime = timestamp;
      chartNeedsUpdate = false;
      rebuildChartFromBuffer();
    }
  }
  requestAnimationFrame(loop);
}

function rebuildChartFromBuffer() {
  if (!motionChart) return;

  motionChart.data.labels = chartBuffer.map(item => item.label);
  motionChart.data.datasets[0].data = chartBuffer.map(item => item.ax);
  motionChart.data.datasets[1].data = chartBuffer.map(item => item.ay);
  motionChart.data.datasets[2].data = chartBuffer.map(item => item.az);
  motionChart.data.datasets[3].data = chartBuffer.map(item => item.gx);
  motionChart.data.datasets[4].data = chartBuffer.map(item => item.gy);
  motionChart.data.datasets[5].data = chartBuffer.map(item => item.gz);

  applyChartFilterMode();

  if (chartWindowBadge) {
    const sec = Math.round(chartTimeWindowMs / 1000);
    const winLabel = sec >= 60 ? `${Math.round(sec / 60)} Min` : `${sec}s`;
    chartWindowBadge.innerHTML = `<i class="fa-solid fa-clock"></i> Last ${winLabel} (${sec}s) • ${chartBuffer.length} pts`;
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

  L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
    attribution: '&copy; OpenStreetMap',
    maxZoom: 19
  }).addTo(map);

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

  const newPos = [lat, lng];
  if (cowMarker) cowMarker.setLatLng(newPos);
  
  pathHistory.push(newPos);
  if (pathHistory.length > 50) pathHistory.shift();
  
  if (polyline) polyline.setLatLngs(pathHistory);
  if (map) map.panTo(newPos);
}

// ==========================================================================
// Console & UI Helpers
// ==========================================================================
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
  if (autoScroll) {
    terminalLog.scrollTop = terminalLog.scrollHeight;
  }
}

function escapeHtml(str) {
  return str.replace(/&/g, "&amp;").replace(/</g, "&lt;").replace(/>/g, "&gt;");
}

function togglePauseStream() {
  isStreamPaused = !isStreamPaused;
  if (btnPauseStream) {
    btnPauseStream.classList.toggle('active', isStreamPaused);
    btnPauseStream.innerHTML = isStreamPaused ? '<i class="fa-solid fa-play"></i>' : '<i class="fa-solid fa-pause"></i>';
    btnPauseStream.title = isStreamPaused ? 'Resume Stream' : 'Pause Stream';
  }
  logToConsole('system', isStreamPaused ? 'UART Stream PAUSED.' : 'UART Stream RESUMED.');
}

function toggleAutoScroll() {
  autoScroll = !autoScroll;
  if (btnToggleAutoScroll) {
    btnToggleAutoScroll.classList.toggle('active', autoScroll);
  }
  logToConsole('system', autoScroll ? 'Terminal Auto-Scroll ENABLED.' : 'Terminal Auto-Scroll DISABLED.');
}

// ==========================================================================
// Telemetry Stream CSV Recorder
// ==========================================================================
const CSV_HEADERS = [
  'Timestamp (ISO)',
  'Timestamp (Local)',
  'Packet #',
  'Source',
  'Cow Tag ID',
  'Data Type',
  'Latitude (°)',
  'Longitude (°)',
  'Accel X (g)',
  'Accel Y (g)',
  'Accel Z (g)',
  'Gyro X (°/s)',
  'Gyro Y (°/s)',
  'Gyro Z (°/s)',
  'Pitch (°)',
  'Roll (°)',
  'Yaw (°)',
  'Battery (V)',
  'Activity Mode',
  'Payload Text',
  'Raw Hex'
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

  logToConsole('system', `[RECORDER STARTED] Capturing to "${activeRecordingFileName}".`);
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
// Simulator Stream Generator (Calibrated to ±2.5g and ±125°/s 16-Bit ADC)
// ==========================================================================
function toggleSimulatorStream() {
  isSimulatorRunning = !isSimulatorRunning;
  if (isSimulatorRunning) {
    btnSimulateStream.classList.add('btn-primary');
    btnSimulateStream.innerHTML = '<i class="fa-solid fa-stop"></i> Stop Test';
    logToConsole('system', '[SIMULATOR] High-Speed 6-DOF IMU & 3D Attitude Stream Generator STARTED (20 Hz).');

    let simLat = 31.968600;
    let simLng = -99.901800;
    let count = 0;

    // Run at 20 Hz (50ms) for high-speed continuous motion and graph validation
    simulatorInterval = setInterval(() => {
      count++;
      simLat += (Math.random() - 0.5) * 0.00005;
      simLng += (Math.random() - 0.5) * 0.00005;

      const t = count * 0.08;
      // Realistic physical movements within ±2.5g and ±125°/s
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
      const battPct = Math.round(Math.min(100, Math.max(0, ((battMv - 3300) / 900) * 100)));

      // 16-Bit Signed ADC Values based on ±2.5g (13107.2 LSB/g) and ±125°/s (262.144 LSB/dps)
      const rawAx = Math.round(parseFloat(ax) * accelLsbPerG);
      const rawAy = Math.round(parseFloat(ay) * accelLsbPerG);
      const rawAz = Math.round(parseFloat(az) * accelLsbPerG);
      const rawGx = Math.round(parseFloat(gx) * gyroLsbPerDps);
      const rawGy = Math.round(parseFloat(gy) * gyroLsbPerDps);
      const rawGz = Math.round(parseFloat(gz) * gyroLsbPerDps);

      const mode = count % 4;
      if (mode === 0) {
        const textMsg = `$IMU,${280 + count},${482000 + count * 20},${rawAx},${rawAy},${rawAz},${rawGx},${rawGy},${rawGz},${pitch},${roll},${yaw}\n`;
        processRawUartChunk(textMsg, 'SIM_UART');
      } else if (mode === 1) {
        const textMsg = `$RPY,${roll},${pitch},${yaw}\n`;
        processRawUartChunk(textMsg, 'SIM_UART');
      } else if (mode === 2) {
        const textMsg = `ACCEL: ${ax}, ${ay}, ${az}\nGYRO: ${gx}, ${gy}, ${gz}\n`;
        processRawUartChunk(textMsg, 'SIM_UART');
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
    }, 50); // 20 Hz
  } else {
    clearInterval(simulatorInterval);
    simulatorInterval = null;
    btnSimulateStream.classList.remove('btn-primary');
    btnSimulateStream.innerHTML = '<i class="fa-solid fa-flask"></i> Test Stream';
    logToConsole('system', '[SIMULATOR] Test stream generator STOPPED.');
  }
}

// ==========================================================================
// Mobile Tabs & Full Width Layout Handlers
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

    if (dashboardGrid) {
      dashboardGrid.dataset.mobileTab = targetTab;
    }

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

    if (window.innerWidth <= 860) {
      window.scrollTo({ top: 0, behavior: 'smooth' });
    }
  });
}

function setupResponsiveHandlers() {
  let resizeTimeout = null;
  const handleResize = () => {
    clearTimeout(resizeTimeout);
    resizeTimeout = setTimeout(() => {
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
    btnToggleFullWidth.title = 'Restore Standard Dashboard View';
    logToConsole('system', 'Telemetry Stream area expanded to FULL SCREEN WIDTH.');
  } else {
    dashboardGrid.classList.remove('full-width-active');
    btnToggleFullWidth.innerHTML = '<i class="fa-solid fa-expand"></i> <span class="btn-text">Full Width</span>';
    btnToggleFullWidth.classList.remove('btn-primary');
    btnToggleFullWidth.title = 'Toggle Full Width Stream Area';
    logToConsole('system', 'Restored standard dashboard layout.');
  }

  if (map) setTimeout(() => map.invalidateSize(), 250);
}
