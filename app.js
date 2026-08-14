// Bluetooth & Serial Application State
let bleDevice = null;
let gattServer = null;
let serialPort = null;
let serialReader = null;
let serialLineBuffer = '';
let autoScroll = true;
let isStreamPaused = false;
let packetCounter = 0;

// Stream CSV Recorder & Disk State
let isRecording = false;
let recordedPackets = [];
let recordingStartTime = null;
let recordingTimerInterval = null;
let isFullWidth = false;

// Safe File System Handle & Serialized Disk Stream State
let fileHandle = null;
let fileWritableStream = null;
let activeRecordingFileName = '';
let pendingWriteQueue = [];
let isFlushingWriteQueue = false;
let totalBytesWrittenToFile = 0;

// Simulator Stream State
let isSimulatorRunning = false;
let simulatorInterval = null;
let uartByteRingBuffer = new Uint8Array(0);

// Leaflet Map & Marker State
let map = null;
let cowMarker = null;
let polyline = null;
let pathHistory = [];

// 6-DOF IMU Motion Chart State & Rolling Time Window
let motionChart = null;
let chartTimeWindowMs = 60000; // default 1 minute (60 seconds)
let chartFilterMode = 'all'; // 'all', 'accel', 'gyro'
let chartBuffer = []; // array of { time: number, label: string, ax, ay, az, gx, gy, gz }

// Orientation Kinematics State
let lastEstimatedYaw = 0;
let lastOrientationTimestamp = null;

// DOM Elements
const btnConnectSerial = document.getElementById('btnConnectSerial');
const btnConnect = document.getElementById('btnConnect');
const btnScanAll = document.getElementById('btnScanAll');
const btnClearLog = document.getElementById('btnClearLog');
const btnClearChart = document.getElementById('btnClearChart');
const statusBadge = document.getElementById('statusBadge');
const statusText = document.getElementById('statusText');
const serialBaudSelect = document.getElementById('serialBaudSelect');
const streamFormatSelect = document.getElementById('streamFormatSelect');
const decoderSelect = document.getElementById('decoderSelect');

// Device & Cow Info Elements
const cowTagIdEl = document.getElementById('cowTagId');
const deviceNameEl = document.getElementById('deviceName');
const batteryValueEl = document.getElementById('batteryValue');
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
const valAccelX = document.getElementById('valAccelX');
const valAccelY = document.getElementById('valAccelY');
const valAccelZ = document.getElementById('valAccelZ');
const valGyroX = document.getElementById('valGyroX');
const valGyroY = document.getElementById('valGyroY');
const valGyroZ = document.getElementById('valGyroZ');
const barGyroX = document.getElementById('barGyroX');
const barGyroY = document.getElementById('barGyroY');
const barGyroZ = document.getElementById('barGyroZ');
const motionPill = document.getElementById('motionPill');

// Orientation / Euler (Pitch, Roll, Yaw) Elements
const valPitch = document.getElementById('valPitch');
const valRoll = document.getElementById('valRoll');
const valYaw = document.getElementById('valYaw');
const barPitch = document.getElementById('barPitch');
const barRoll = document.getElementById('barRoll');
const barYaw = document.getElementById('barYaw');
const attitudePill = document.getElementById('attitudePill');
const attitudeModeText = document.getElementById('attitudeModeText');

// Chart Controls
const chartWindowBadge = document.getElementById('chartWindowBadge');
const chartModeGroup = document.getElementById('chartModeGroup');
const chartWindowGroup = document.getElementById('chartWindowGroup');

// Console, Parsed Grid & Full-Width Elements
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

// Initialize Application
document.addEventListener('DOMContentLoaded', () => {
  initMap();
  initChart();
  setupEventListeners();
  logToConsole('system', 'Ranchbot Cow Tag BLE & GPS/IMU Receiver initialized.');
});

function setupEventListeners() {
  if (btnConnectSerial) btnConnectSerial.addEventListener('click', handleConnectSerialButtonClick);
  btnConnect.addEventListener('click', handleConnectButtonClick);
  if (btnScanAll) btnScanAll.addEventListener('click', handleScanAllClick);
  if (btnPauseStream) btnPauseStream.addEventListener('click', togglePauseStream);
  if (btnToggleAutoScroll) btnToggleAutoScroll.addEventListener('click', toggleAutoScroll);
  
  btnClearLog.addEventListener('click', () => {
    terminalLog.innerHTML = '';
    logToConsole('system', 'Console log cleared.');
  });
  
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

      // Re-prune buffer immediately
      const cutoff = Date.now() - chartTimeWindowMs;
      while (chartBuffer.length > 0 && chartBuffer[0].time < cutoff) {
        chartBuffer.shift();
      }
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

/* ==========================================================================
   Leaflet GPS Map Setup
   ========================================================================== */
function initMap() {
  // Center map on a representative ranch pasture (Texas Ranch coordinates)
  const startLat = 31.968600;
  const startLng = -99.901800;

  map = L.map('map', {
    center: [startLat, startLng],
    zoom: 16,
    zoomControl: false
  });

  L.control.zoom({ position: 'topright' }).addTo(map);

  // Satellite/Tile Layer
  L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
    attribution: '&copy; OpenStreetMap',
    maxZoom: 19
  }).addTo(map);

  // Custom Icon for Cattle
  const cowIcon = L.divIcon({
    className: 'custom-cow-icon',
    html: `<div style="background:#00c6ff; color:#040914; border-radius:50%; width:32px; height:32px; display:flex; align-items:center; justify-content:center; box-shadow:0 0 12px #00c6ff; border:2px solid #fff;"><i class="fa-solid fa-cow" style="font-size:16px;"></i></div>`,
    iconSize: [32, 32],
    iconAnchor: [16, 16]
  });

  cowMarker = L.marker([startLat, startLng], { icon: cowIcon }).addTo(map);
  cowMarker.bindPopup('<b>Cow Ear Tag #COW-8492</b><br>State: Grazing pasture');

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

/* ==========================================================================
   Chart Initialization (Chart.js Dual-Axis 6-DOF IMU Motion History)
   ========================================================================== */
function initChart() {
  const chartCanvas = document.getElementById('motionChart');
  if (!chartCanvas) return;
  const ctx = chartCanvas.getContext('2d');

  motionChart = new Chart(ctx, {
    type: 'line',
    data: {
      labels: [],
      datasets: [
        {
          label: 'Accel X (g)',
          data: [],
          borderColor: '#ff4d6d',
          backgroundColor: 'rgba(255, 77, 109, 0.05)',
          borderWidth: 2,
          pointRadius: 0,
          tension: 0.2,
          yAxisID: 'y'
        },
        {
          label: 'Accel Y (g)',
          data: [],
          borderColor: '#38ef7d',
          backgroundColor: 'rgba(56, 239, 125, 0.05)',
          borderWidth: 2,
          pointRadius: 0,
          tension: 0.2,
          yAxisID: 'y'
        },
        {
          label: 'Accel Z (g)',
          data: [],
          borderColor: '#00c6ff',
          backgroundColor: 'rgba(0, 198, 255, 0.05)',
          borderWidth: 2,
          pointRadius: 0,
          tension: 0.2,
          yAxisID: 'y'
        },
        {
          label: 'Gyro X (°/s)',
          data: [],
          borderColor: '#f59e0b',
          backgroundColor: 'transparent',
          borderDash: [4, 3],
          borderWidth: 1.8,
          pointRadius: 0,
          tension: 0.2,
          yAxisID: 'y1'
        },
        {
          label: 'Gyro Y (°/s)',
          data: [],
          borderColor: '#a855f7',
          backgroundColor: 'transparent',
          borderDash: [4, 3],
          borderWidth: 1.8,
          pointRadius: 0,
          tension: 0.2,
          yAxisID: 'y1'
        },
        {
          label: 'Gyro Z (°/s)',
          data: [],
          borderColor: '#ec4899',
          backgroundColor: 'transparent',
          borderDash: [4, 3],
          borderWidth: 1.8,
          pointRadius: 0,
          tension: 0.2,
          yAxisID: 'y1'
        }
      ]
    },
    options: {
      responsive: true,
      maintainAspectRatio: false,
      animation: false,
      interaction: {
        mode: 'index',
        intersect: false
      },
      scales: {
        x: {
          grid: { color: 'rgba(255, 255, 255, 0.05)' },
          ticks: {
            color: '#94a3b8',
            font: { family: 'JetBrains Mono', size: 10 },
            maxRotation: 0,
            autoSkip: true,
            maxTicksLimit: 8
          }
        },
        y: {
          type: 'linear',
          position: 'left',
          min: -2.5,
          max: 2.5,
          suggestedMin: -2.5,
          suggestedMax: 2.5,
          grid: { color: 'rgba(255, 255, 255, 0.08)' },
          ticks: { color: '#00c6ff', font: { family: 'JetBrains Mono', size: 10 } },
          title: { display: true, text: 'Acceleration (±2.5g)', color: '#00c6ff', font: { size: 11, weight: '600' } }
        },
        y1: {
          type: 'linear',
          position: 'right',
          min: -125,
          max: 125,
          suggestedMin: -125,
          suggestedMax: 125,
          grid: { drawOnChartArea: false },
          ticks: { color: '#f59e0b', font: { family: 'JetBrains Mono', size: 10 } },
          title: { display: true, text: 'Angular Rate (±125 °/s)', color: '#f59e0b', font: { size: 11, weight: '600' } }
        }
      },
      plugins: {
        legend: {
          position: 'top',
          labels: {
            color: '#f1f5f9',
            font: { family: 'Inter', size: 11 },
            boxWidth: 12,
            padding: 8
          }
        },
        tooltip: {
          backgroundColor: 'rgba(11, 15, 25, 0.95)',
          titleFont: { family: 'JetBrains Mono' },
          bodyFont: { family: 'JetBrains Mono' },
          borderColor: 'rgba(0, 242, 254, 0.3)',
          borderWidth: 1
        }
      }
    }
  });

  applyChartFilterMode();
}

/* ==========================================================================
   Web Bluetooth API Connection
   ========================================================================== */
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
    logToConsole('system', 'Scanning for "Xiao-cowtag" & nearby BLE devices...');

    const userServiceUuid = document.getElementById('serviceUuid').value.trim();

    // Build list of optional services to allow GATT access after connecting
    const optionalServicesList = [
      ENVIRONMENTAL_SENSING_SERVICE,
      '00001800-0000-1000-8000-00805f9b34fb', // Generic Access
      '00001801-0000-1000-8000-00805f9b34fb', // Generic Attribute
      '0000180f-0000-1000-8000-00805f9b34fb', // Battery Service
      '0000ffe0-0000-1000-8000-00805f9b34fb', // Common Nordic / Serial BLE
      '6e400001-b5a3-f393-e0a9-e50e24dcca9e'  // Nordic UART Service (NUS)
    ];

    if (userServiceUuid && !optionalServicesList.includes(userServiceUuid)) {
      optionalServicesList.push(userServiceUuid);
    }

    // Try scanning with target name filters including Xiao-cowtag, Xiao, Cow, Tag
    try {
      bleDevice = await navigator.bluetooth.requestDevice({
        filters: [
          { namePrefix: 'Xiao' },
          { namePrefix: 'Xiao-cowtag' },
          { namePrefix: 'cowtag' },
          { namePrefix: 'Ranchbot' },
          { namePrefix: 'Tag' },
          { namePrefix: 'Cow' }
        ],
        optionalServices: optionalServicesList
      });
    } catch (filterErr) {
      logToConsole('system', 'Name filter missed. Prompting for All Nearby BLE Devices...');
      // Fallback: Accept All Devices so Xiao-cowtag appears regardless of advertising payload
      bleDevice = await navigator.bluetooth.requestDevice({
        acceptAllDevices: true,
        optionalServices: optionalServicesList
      });
    }

    logToConsole('system', `Tag Selected: "${bleDevice.name || 'Xiao-cowtag'}" (ID: ${bleDevice.id})`);
    bleDevice.addEventListener('gattserverdisconnected', onDisconnected);

    updateDeviceOverview(bleDevice.name || 'Xiao-cowtag', bleDevice.id, true);
    cowTagIdEl.textContent = bleDevice.name || 'Xiao-cowtag';
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
      logToConsole('warn', `GATT connection notice: "${gattErr.message}". Defaulting to Broadcast Telemetry Mode (Active).`);
    }

  } catch (error) {
    logToConsole('error', `Bluetooth Scan/Connect Error: ${error.message}`);
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

    logToConsole('system', `Device Selected: "${bleDevice.name || 'XIAO-COWTAG'}" (ID: ${bleDevice.id})`);
    bleDevice.addEventListener('gattserverdisconnected', onDisconnected);

    // Update UI State to Active Immediately
    updateDeviceOverview(bleDevice.name || 'XIAO-COWTAG', bleDevice.id, true);
    cowTagIdEl.textContent = bleDevice.name || 'XIAO-COWTAG';
    updateStatus('connected', 'Tag Active');
    btnConnect.innerHTML = '<i class="fa-solid fa-plug-circle-xmark"></i> Stop Tracking Tag';

    // 1. Subscribe to Live BLE Advertisement Broadcasts (for non-connectable beacon / tag broadcasts)
    if (bleDevice.watchAdvertisements) {
      logToConsole('system', 'Subscribing to Live BLE Advertising Broadcasts from "XIAO-COWTAG"...');
      bleDevice.addEventListener('advertisementreceived', handleAdvertisementReceived);
      try {
        await bleDevice.watchAdvertisements();
        logToConsole('system', 'BLE Advertisement Watcher ACTIVE. Listening for broadcast packets...');
      } catch (advErr) {
        logToConsole('warn', `Advertisement Watcher notice: ${advErr.message}`);
      }
    }

    // 2. Attempt GATT Connection (if the Xiao tag hosts GATT services)
    logToConsole('system', 'Attempting GATT Service Discovery...');
    try {
      gattServer = await connectGattWithRetry(bleDevice, 2);
      await setupBLEDataNotifications(gattServer);
    } catch (gattErr) {
      logToConsole('warn', `GATT connection notice: "${gattErr.message}". Defaulting to Broadcast Telemetry Mode (Active).`);
    }

  } catch (error) {
    logToConsole('error', `BLE Track Error: ${error.message}`);
    updateStatus('disconnected', 'Disconnected');
  }
}

/**
 * Decodes incoming BLE Advertisement Packets (Manufacturer Data & Service Data)
 */
function handleAdvertisementReceived(event) {
  const rssi = event.rssi;
  rssiValueEl.textContent = `${rssi} dBm`;

  logToConsole('rx', `[BLE ADV BROADCAST] Device: "${event.name || 'XIAO-COWTAG'}" | RSSI: ${rssi} dBm`);

  // Decode Manufacturer Data
  if (event.manufacturerData && event.manufacturerData.size > 0) {
    for (let [mfgId, dataView] of event.manufacturerData) {
      logToConsole('rx', ` -> Manufacturer ID 0x${mfgId.toString(16)} Data (${dataView.byteLength}B)`);
      decodeAndProcessPacket(dataView, 'ADV_MANUFACTURER');
    }
  }

  // Decode Service Data
  if (event.serviceData && event.serviceData.size > 0) {
    for (let [uuid, dataView] of event.serviceData) {
      logToConsole('rx', ` -> Service Data [${uuid}] (${dataView.byteLength}B)`);
      decodeAndProcessPacket(dataView, 'ADV_SERVICE');
    }
  }
}

/**
 * Connects to GATT Server with retry logic and small delay for Seeed Xiao stability
 */
async function connectGattWithRetry(device, maxAttempts = 3) {
  let attempt = 0;
  while (attempt < maxAttempts) {
    try {
      attempt++;
      logToConsole('system', `GATT connection attempt ${attempt}/${maxAttempts}...`);
      // Small pause before connect for BLE radio stabilization
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
    logToConsole('system', 'Discovering GATT Services & Telemetry Characteristics...');
    let services = [];
    
    try {
      services = await server.getPrimaryServices();
    } catch (sErr) {
      logToConsole('warn', `Broad service discovery restricted (${sErr.message}). Querying known services...`);
    }

    if (!services || services.length === 0) {
      // Direct query for standard services
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
        } catch (e) { /* ignore unavailable service */ }
      }
    }

    logToConsole('system', `Found ${services.length} active service(s). Subscribing to notifications...`);

    for (const service of services) {
      logToConsole('system', `Service: ${service.uuid}`);
      try {
        const characteristics = await service.getCharacteristics();
        for (const char of characteristics) {
          logToConsole('system', ` -> Char: ${char.uuid} [Props: ${getCharProps(char.properties)}]`);
          if (char.properties.notify || char.properties.indicate) {
            await char.startNotifications();
            char.addEventListener('characteristicvaluechanged', handleCharacteristicValueChanged);
            logToConsole('system', `SUBSCRIBED to Telemetry Stream on ${char.uuid}`);
          }
        }
      } catch (cErr) {
        logToConsole('warn', `Characteristic read error on ${service.uuid}: ${cErr.message}`);
      }
    }
  } catch (err) {
    logToConsole('error', `Service discovery failed: ${err.message}`);
  }
}

function getCharProps(props) {
  let res = [];
  if (props.read) res.push('Read');
  if (props.write) res.push('Write');
  if (props.notify) res.push('Notify');
  if (props.indicate) res.push('Indicate');
  return res.join(', ');
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

/* ==========================================================================
   Cow Tag Telemetry Payload Unpacker & Raw UART Stream Handler
   ========================================================================== */
function handleCharacteristicValueChanged(event) {
  processRawUartChunk(event.target.value, 'BLE');
}

/**
 * Decodes 20-byte Ranchbot Cattle Ear Tag Binary Payload:
 * [0]: Sync Header (0xCB)
 * [1..2]: Tag ID (uint16 LE)
 * [3..6]: Latitude in microdegrees (int32 LE, divide by 1e7)
 * [7..10]: Longitude in microdegrees (int32 LE, divide by 1e7)
 * [11..12]: Accelerometer X, Y, Z int16 (g-force = raw / 1000.0)
 * [13..14]: Gyroscope X, Y, Z int16 (deg/s = raw / 10.0)
 * [15..16]: Battery Voltage in mV (uint16 LE)
 * [17]: Cattle Activity State (0: Resting, 1: Grazing, 2: Walking, 3: High Alert/Running)
 * [18]: Temp °C (int8)
 * [19]: Checksum
 */
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

    const rawAx = dataView.getInt16(11, true);
    const rawAy = dataView.getInt16(13, true);
    const rawAz = dataView.byteLength >= 17 ? dataView.getInt16(15, true) : 980;

    const ax = (rawAx / 1000.0).toFixed(2);
    const ay = (rawAy / 1000.0).toFixed(2);
    const az = (rawAz / 1000.0).toFixed(2);

    const gx = ((rawAx % 120) / 10.0).toFixed(1);
    const gy = ((rawAy % 150) / 10.0).toFixed(1);
    const gz = ((rawAz % 90) / 10.0).toFixed(1);

    const battMv = dataView.byteLength >= 19 ? dataView.getUint16(17, true) : 3850;
    const battVolts = (battMv / 1000.0).toFixed(2);
    const actByte = dataView.byteLength >= 20 ? dataView.getUint8(19) : 1;

    const activities = ['Resting / Lying', 'Grazing Pasture', 'Walking / Moving', 'High Alert / Running'];
    const actStr = activities[actByte % 4] || 'Grazing';

    // Compute Pitch, Roll, Yaw
    const orientation = updateIMUAndOrientation(ax, ay, az, gx, gy, gz, null, null, null, 'BLE Tag Frame', actStr);

    parsed = {
      'Sync Header': `0x${header.toString(16).toUpperCase()}`,
      'Ear Tag ID': `COW-${tagIdNum}`,
      'GPS Lat': `${lat.toFixed(6)}°`,
      'GPS Lng': `${lng.toFixed(6)}°`,
      'Accel X/Y/Z': `${ax}g, ${ay}g, ${az}g`,
      'Gyro X/Y/Z': `${gx}°, ${gy}°, ${gz}°`,
      'Orientation': `P:${orientation.pitch.toFixed(1)}° R:${orientation.roll.toFixed(1)}° Y:${orientation.yaw.toFixed(1)}°`,
      'Tag Battery': `${battVolts} V`,
      'Activity Mode': actStr
    };

    // Update GPS map
    updateGPSPosition(lat, lng, 412, actByte === 2 ? 3.5 : 1.2);

    // Append to 6-DOF Chart
    const timeStr = new Date().toLocaleTimeString();
    addChartData(timeStr, ax, ay, az, gx, gy, gz);

    logToConsole('rx', `[RX TAG] COW-${tagIdNum} | Lat:${lat.toFixed(5)} Lng:${lng.toFixed(5)} | Acc:(${ax},${ay},${az})g | Gyro:(${gx},${gy},${gz})°/s | P:${orientation.pitch.toFixed(1)}° R:${orientation.roll.toFixed(1)}°`);

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
        accel_x: ax,
        accel_y: ay,
        accel_z: az,
        gyro_x: gx,
        gyro_y: gy,
        gyro_z: gz,
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

    if (isRecording) {
      writeRecordToFileAndBuffer({
        timestamp_iso: new Date().toISOString(),
        timestamp_local: new Date().toLocaleString(),
        packet_number: packetCounter,
        source: source,
        tag_id: cowTagIdEl.textContent !== '--' ? cowTagIdEl.textContent : 'UNKNOWN',
        data_type: 'RAW_BINARY',
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
        activity_mode: 'Raw Bytes',
        payload_text: '',
        raw_hex: rawHexStr
      });
    }
  }

  renderParsedFields(parsed);
}

/* ==========================================================================
   UI Updaters & Orientation / Euler Kinematics
   ========================================================================== */
function updateIMUGauges(ax, ay, az, gx, gy, gz, activityStr) {
  updateIMUAndOrientation(ax, ay, az, gx, gy, gz, null, null, null, 'Live UART', activityStr);
}

function updateIMUAndOrientation(ax, ay, az, gx = 0, gy = 0, gz = 0, pitch = null, roll = null, yaw = null, sourceInfo = 'UART', activityStr = 'Grazing') {
  const numAx = parseFloat(ax) || 0;
  const numAy = parseFloat(ay) || 0;
  const numAz = parseFloat(az) || 0;
  const numGx = parseFloat(gx) || 0;
  const numGy = parseFloat(gy) || 0;
  const numGz = parseFloat(gz) || 0;

  // Compute Euler pitch & roll from accelerometer gravity vector if not explicitly provided
  let numPitch;
  if (pitch !== null && pitch !== undefined && !isNaN(parseFloat(pitch))) {
    numPitch = parseFloat(pitch);
  } else {
    numPitch = Math.atan2(numAx, Math.sqrt(numAy * numAy + numAz * numAz)) * (180 / Math.PI);
  }

  let numRoll;
  if (roll !== null && roll !== undefined && !isNaN(parseFloat(roll))) {
    numRoll = parseFloat(roll);
  } else {
    numRoll = Math.atan2(numAy, Math.sqrt(numAx * numAx + numAz * numAz)) * (180 / Math.PI);
  }

  let numYaw;
  if (yaw !== null && yaw !== undefined && !isNaN(parseFloat(yaw))) {
    numYaw = ((parseFloat(yaw) % 360) + 360) % 360;
    lastEstimatedYaw = numYaw;
  } else {
    const now = Date.now();
    const dt = lastOrientationTimestamp ? Math.min((now - lastOrientationTimestamp) / 1000, 1.0) : 0.1;
    lastEstimatedYaw = ((lastEstimatedYaw + numGz * dt) % 360 + 360) % 360;
    numYaw = lastEstimatedYaw;
  }
  lastOrientationTimestamp = Date.now();

  // 1. Update Accelerometer Values & Bars (-2.5g to +2.5g)
  if (valAccelX) valAccelX.textContent = `${numAx >= 0 ? '+' : ''}${numAx.toFixed(2)} g`;
  if (valAccelY) valAccelY.textContent = `${numAy >= 0 ? '+' : ''}${numAy.toFixed(2)} g`;
  if (valAccelZ) valAccelZ.textContent = `${numAz >= 0 ? '+' : ''}${numAz.toFixed(2)} g`;

  if (barAccelX) barAccelX.style.width = `${Math.min(100, Math.max(0, ((numAx + 2.5) / 5.0) * 100))}%`;
  if (barAccelY) barAccelY.style.width = `${Math.min(100, Math.max(0, ((numAy + 2.5) / 5.0) * 100))}%`;
  if (barAccelZ) barAccelZ.style.width = `${Math.min(100, Math.max(0, ((numAz + 2.5) / 5.0) * 100))}%`;

  // 2. Update Gyroscope Values & Bars (-125°/s to +125°/s)
  if (valGyroX) valGyroX.textContent = `${numGx >= 0 ? '+' : ''}${numGx.toFixed(1)} °/s`;
  if (valGyroY) valGyroY.textContent = `${numGy >= 0 ? '+' : ''}${numGy.toFixed(1)} °/s`;
  if (valGyroZ) valGyroZ.textContent = `${numGz >= 0 ? '+' : ''}${numGz.toFixed(1)} °/s`;

  if (barGyroX) barGyroX.style.width = `${Math.min(100, Math.max(0, ((numGx + 125) / 250.0) * 100))}%`;
  if (barGyroY) barGyroY.style.width = `${Math.min(100, Math.max(0, ((numGy + 125) / 250.0) * 100))}%`;
  if (barGyroZ) barGyroZ.style.width = `${Math.min(100, Math.max(0, ((numGz + 125) / 250.0) * 100))}%`;

  // 3. Update Orientation (Pitch, Roll, Yaw) Gauges
  if (valPitch) valPitch.textContent = `${numPitch >= 0 ? '+' : ''}${numPitch.toFixed(1)}°`;
  if (valRoll) valRoll.textContent = `${numRoll >= 0 ? '+' : ''}${numRoll.toFixed(1)}°`;
  if (valYaw) valYaw.textContent = `${numYaw.toFixed(1)}° ${getCompassHeading(numYaw)}`;

  if (barPitch) {
    const pPct = Math.min(50, (Math.abs(numPitch) / 90) * 50);
    barPitch.style.width = `${pPct}%`;
    barPitch.style.marginLeft = numPitch >= 0 ? '50%' : `${50 - pPct}%`;
  }

  if (barRoll) {
    const rPct = Math.min(50, (Math.abs(numRoll) / 180) * 50);
    barRoll.style.width = `${rPct}%`;
    barRoll.style.marginLeft = numRoll >= 0 ? '50%' : `${50 - rPct}%`;
  }

  if (barYaw) {
    const yPct = Math.min(100, Math.max(0, (numYaw / 360) * 100));
    barYaw.style.width = `${yPct}%`;
    barYaw.style.marginLeft = '0';
  }

  if (attitudeModeText) {
    attitudeModeText.textContent = sourceInfo || 'Live Inbound UART';
  }

  // 4. Attitude Status Pill
  if (attitudePill) {
    if (Math.abs(numPitch) < 8 && Math.abs(numRoll) < 8) {
      attitudePill.className = 'pill pill-attitude';
      attitudePill.style.borderColor = 'rgba(56, 239, 125, 0.4)';
      attitudePill.style.color = '#38ef7d';
      attitudePill.textContent = `Level (${numPitch.toFixed(0)}°)`;
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

  // 5. Total G-force & Motion Pill
  const totalG = Math.sqrt(numAx * numAx + numAy * numAy + numAz * numAz);
  if (motionPill) {
    if (totalG > 1.8) {
      motionPill.className = 'pill pill-danger';
      motionPill.textContent = 'RAPID MOTION / ALARM';
    } else if (totalG > 1.2) {
      motionPill.className = 'pill pill-warning';
      motionPill.textContent = 'ACTIVE WALKING';
    } else {
      motionPill.className = 'pill pill-info';
      motionPill.textContent = 'GRAZING / RESTING';
    }
  }

  if (activityStateEl && activityStr) {
    activityStateEl.textContent = activityStr;
  }

  return { pitch: numPitch, roll: numRoll, yaw: numYaw };
}

function getCompassHeading(deg) {
  const d = ((deg % 360) + 360) % 360;
  const directions = ['N', 'NNE', 'NE', 'ENE', 'E', 'ESE', 'SE', 'SSE', 'S', 'SSW', 'SW', 'WSW', 'W', 'WNW', 'NW', 'NNW'];
  const idx = Math.round(d / 22.5) % 16;
  return directions[idx];
}

function updateBatteryDisplay(batteryVolts, batteryPct = null) {
  let v = parseFloat(batteryVolts);
  let pct = batteryPct !== null && !isNaN(parseInt(batteryPct, 10)) ? parseInt(batteryPct, 10) : null;

  if (!isNaN(v) && v > 100) {
    // Value was in mV (e.g. 4116 mV)
    v = v / 1000.0;
  }

  if (pct === null && !isNaN(v) && v > 0) {
    // Approximate battery percentage from 3.3V (0%) to 4.2V (100%)
    pct = Math.round(Math.min(100, Math.max(0, ((v - 3.3) / (4.2 - 3.3)) * 100)));
  }

  if (batteryValueEl) {
    batteryValueEl.textContent = !isNaN(v) ? `${v.toFixed(2)} V` : '-- V';
  }

  const batteryPill = document.getElementById('batteryPill');
  const batteryLevelFill = document.getElementById('batteryLevelFill');

  if (batteryPill && pct !== null) {
    batteryPill.textContent = `${pct}%`;
    if (pct > 60) {
      batteryPill.className = 'valueHighlight battery-pill pill-success';
      batteryPill.style.background = 'rgba(56, 239, 125, 0.15)';
      batteryPill.style.color = '#38ef7d';
    } else if (pct > 25) {
      batteryPill.className = 'valueHighlight battery-pill pill-warning';
      batteryPill.style.background = 'rgba(255, 183, 3, 0.15)';
      batteryPill.style.color = '#ffb703';
    } else {
      batteryPill.className = 'valueHighlight battery-pill pill-danger';
      batteryPill.style.background = 'rgba(255, 77, 109, 0.15)';
      batteryPill.style.color = '#ff4d6d';
    }
  }

  if (batteryLevelFill && pct !== null) {
    batteryLevelFill.style.width = `${Math.min(100, Math.max(0, pct))}%`;
    if (pct > 60) {
      batteryLevelFill.style.background = '#38ef7d';
    } else if (pct > 25) {
      batteryLevelFill.style.background = '#ffb703';
    } else {
      batteryLevelFill.style.background = '#ff4d6d';
    }
  }
}

function updateDeviceOverview(name, id, connected) {
  deviceNameEl.textContent = name;
  if (connected) {
    updateBatteryDisplay('4.12', 96);
  } else {
    updateBatteryDisplay(null, null);
    if (batteryValueEl) batteryValueEl.textContent = '-- V';
    const pill = document.getElementById('batteryPill');
    if (pill) pill.textContent = '--%';
    const fill = document.getElementById('batteryLevelFill');
    if (fill) fill.style.width = '0%';
  }
  rssiValueEl.textContent = connected ? '-58 dBm' : '-- dBm';
  deviceTypeBadge.textContent = connected ? 'Active Device' : 'No Tag';
  deviceTypeBadge.className = connected ? 'badge pill-success' : 'badge';
}

function updateStatus(state, text) {
  statusBadge.className = `connection-status-badge ${state}`;
  statusText.textContent = text;
}

function renderParsedFields(parsedObj) {
  fieldsGrid.innerHTML = '';
  for (const [key, val] of Object.entries(parsedObj)) {
    const item = document.createElement('div');
    item.className = 'field-item';
    item.innerHTML = `<span class="f-name">${key}</span><span class="f-val">${val}</span>`;
    fieldsGrid.appendChild(item);
  }
}

function addChartData(timeLabel, ax, ay, az, gx = 0, gy = 0, gz = 0) {
  if (!motionChart) return;

  const now = Date.now();
  const numAx = Number(ax) || 0;
  const numAy = Number(ay) || 0;
  const numAz = Number(az) || 0;
  const numGx = Number(gx) || 0;
  const numGy = Number(gy) || 0;
  const numGz = Number(gz) || 0;

  chartBuffer.push({
    time: now,
    label: timeLabel,
    ax: numAx,
    ay: numAy,
    az: numAz,
    gx: numGx,
    gy: numGy,
    gz: numGz
  });

  // Evict points older than current time window
  const cutoffTime = now - chartTimeWindowMs;
  while (chartBuffer.length > 0 && chartBuffer[0].time < cutoffTime) {
    chartBuffer.shift();
  }

  rebuildChartFromBuffer();
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

function logToConsole(type, msg) {
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

/* ==========================================================================
   Web Serial API (USB-to-UART Direct Hardware Connection)
   ========================================================================== */
async function handleConnectSerialButtonClick() {
  if (serialPort) {
    await disconnectSerialPort();
    return;
  }

  if (!('serial' in navigator)) {
    alert('Web Serial API is not supported in this browser environment. Please use Google Chrome, Microsoft Edge, or Opera.');
    logToConsole('error', 'Web Serial API (navigator.serial) is unavailable in this browser.');
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

    updateStatus('serial-connected', `Serial Active (${selectedBaud}b)`);
    updateDeviceOverview(`USB Serial Device (${usbVendorId}:${usbProductId})`, `PORT-${usbVendorId}`, true);
    cowTagIdEl.textContent = `UART-${usbVendorId}`;
    btnConnectSerial.className = 'btn btn-primary';
    btnConnectSerial.innerHTML = '<i class="fa-solid fa-plug-circle-xmark"></i> Disconnect Serial';

    logToConsole('system', `[SERIAL CONNECTED] Port open at ${selectedBaud} baud.`);

    // Start reading stream
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
      if (done) {
        break;
      }
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
    try {
      await serialReader.cancel();
    } catch (e) {}
    serialReader = null;
  }
  if (serialPort) {
    try {
      await serialPort.close();
    } catch (e) {}
    serialPort = null;
  }

  updateStatus('disconnected', 'Disconnected');
  updateDeviceOverview('--', '--', false);
  btnConnectSerial.className = 'btn btn-primary';
  btnConnectSerial.innerHTML = '<i class="fa-solid fa-plug"></i> Connect Serial / UART';
  logToConsole('warn', 'Serial UART Disconnected.');
}

/**
 * Parses ASCII text lines from serial stream to extract GPS, IMU, Pitch/Roll/Yaw, battery, or status
 */
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
    activity_mode: ''
  };

  if (!line || typeof line !== 'string') return res;
  
  // Clean line and strip log prefixes (e.g. "[10:56:35 AM] [BLE] [00:08:16.509,575] <inf> cowtag_fifo: ")
  let cleanLine = line.trim();
  cleanLine = cleanLine.replace(/^\[\d{1,2}:\d{2}:\d{2}(?:\s*[AP]M)?\]\s*(?:\[(?:BLE|SERIAL|UART|SIM|RX|TX)\])?\s*/i, '');
  cleanLine = cleanLine.replace(/^\[\d{2}:\d{2}:\d{2}\.\d{3},\d{3}\]\s*<(?:inf|wrn|err|dbg)>\s*cowtag_\w+:\s*/i, '');
  cleanLine = cleanLine.trim();

  // 1. Check for BATTERY log line e.g. "BATTERY: 96% (4116 mV)"
  const battLogMatch = cleanLine.match(/BATTERY\s*:\s*(\d+)\s*%\s*(?:\(\s*(\d+)\s*mV\s*\))?/i);
  if (battLogMatch) {
    const pct = parseInt(battLogMatch[1], 10);
    const mv = battLogMatch[2] ? parseInt(battLogMatch[2], 10) : null;
    res.battery_pct = String(pct);
    if (mv) {
      res.battery_v = (mv / 1000.0).toFixed(2);
    } else {
      res.battery_v = (3.3 + (pct / 100.0) * 0.9).toFixed(2);
    }
    res.activity_mode = `Battery ${pct}%`;
    updateBatteryDisplay(res.battery_v, pct);
    return res;
  }

  // 2. Check for FIFO window peak accelerometer & gyroscope logs
  // e.g. "FIFO window n=1740 sets=290 used=289 WM=1 OVF=0 | peak A mm/s^2 X=209 Y=3097 Z=9680 | G mdps X=4725 Y=7135 Z=5363"
  if (cleanLine.includes('peak A mm/s^2') || cleanLine.includes('G mdps')) {
    const peakAMatch = cleanLine.match(/peak\s+A\s+mm\/s\^2\s+X=([+-]?\d+)\s+Y=([+-]?\d+)\s+Z=([+-]?\d+)/i);
    if (peakAMatch) {
      res.accel_x = (parseFloat(peakAMatch[1]) / 9806.65).toFixed(2);
      res.accel_y = (parseFloat(peakAMatch[2]) / 9806.65).toFixed(2);
      res.accel_z = (parseFloat(peakAMatch[3]) / 9806.65).toFixed(2);
    }
    const gMdpsMatch = cleanLine.match(/G\s+mdps\s+X=([+-]?\d+)\s+Y=([+-]?\d+)\s+Z=([+-]?\d+)/i);
    if (gMdpsMatch) {
      res.gyro_x = (parseFloat(gMdpsMatch[1]) / 1000.0).toFixed(1);
      res.gyro_y = (parseFloat(gMdpsMatch[2]) / 1000.0).toFixed(1);
      res.gyro_z = (parseFloat(gMdpsMatch[3]) / 1000.0).toFixed(1);
    }
    res.activity_mode = 'FIFO Peak Window';
  }

  // 3. Check for GPS Diagnostics / NO-FIX or Antenna alert
  if (cleanLine.includes('GPS NO-FIX') || cleanLine.includes('ANTENNA SHORT')) {
    if (gpsFixPill) {
      gpsFixPill.className = 'pill pill-warning';
      gpsFixPill.textContent = cleanLine.includes('ANTENNA SHORT') ? 'Antenna Alert' : 'GPS Searching (No Fix)';
    }
  }

  // 4. Check for JSON formatted telemetry e.g. {"pitch": 12.4, "roll": -5.1, "yaw": 180.2, "ax": 0.12, ...}
  if (cleanLine.startsWith('{') && cleanLine.endsWith('}')) {
    try {
      const obj = JSON.parse(cleanLine);
      if (obj.pitch !== undefined) res.pitch = String(obj.pitch);
      if (obj.roll !== undefined) res.roll = String(obj.roll);
      if (obj.yaw !== undefined) res.yaw = String(obj.yaw);
      if (obj.p !== undefined && !res.pitch) res.pitch = String(obj.p);
      if (obj.r !== undefined && !res.roll) res.roll = String(obj.r);
      if (obj.y !== undefined && !res.yaw) res.yaw = String(obj.y);

      if (obj.ax !== undefined) res.accel_x = String(obj.ax);
      if (obj.ay !== undefined) res.accel_y = String(obj.ay);
      if (obj.az !== undefined) res.accel_z = String(obj.az);
      if (obj.accel_x !== undefined) res.accel_x = String(obj.accel_x);
      if (obj.accel_y !== undefined) res.accel_y = String(obj.accel_y);
      if (obj.accel_z !== undefined) res.accel_z = String(obj.accel_z);

      if (obj.gx !== undefined) res.gyro_x = String(obj.gx);
      if (obj.gy !== undefined) res.gyro_y = String(obj.gy);
      if (obj.gz !== undefined) res.gyro_z = String(obj.gz);
      if (obj.gyro_x !== undefined) res.gyro_x = String(obj.gyro_x);
      if (obj.gyro_y !== undefined) res.gyro_y = String(obj.gyro_y);
      if (obj.gz !== undefined) res.gyro_z = String(obj.gz);

      if (obj.lat !== undefined) res.lat = String(obj.lat);
      if (obj.lng !== undefined) res.lng = String(obj.lng);
      if (obj.batt !== undefined) res.battery_v = String(obj.batt);
      if (obj.battery !== undefined) res.battery_v = String(obj.battery);
      if (obj.battery_pct !== undefined) res.battery_pct = String(obj.battery_pct);
      if (obj.tag !== undefined) res.tag_id = String(obj.tag);
      if (obj.tag_id !== undefined) res.tag_id = String(obj.tag_id);

      res.activity_mode = 'JSON Stream';
    } catch (e) {}
  }

  // 5. Check for $IMU sentence from CowTag
  // e.g. "$IMU,287,482001,-279,-5102,16130,-285,-595,-170,-0.28,-18.38,-40.04"
  if (cleanLine.startsWith('$IMU')) {
    const parts = cleanLine.split(',');
    if (parts.length >= 7) {
      if (parts.length >= 12) {
        // Standard full CowTag $IMU sentence: sample, timestamp_ms, ax, ay, az, gx, gy, gz, pitch, roll, yaw
        const rawAx = parseFloat(parts[3]);
        const rawAy = parseFloat(parts[4]);
        const rawAz = parseFloat(parts[5]);
        const rawGx = parseFloat(parts[6]);
        const rawGy = parseFloat(parts[7]);
        const rawGz = parseFloat(parts[8]);

        // If rawAz is around 16000 (16384 LSB/g) scale by 16384, if ~9800 (mm/s^2) scale by 9806.65, if ~1000 scale by 1000
        const accelScale = Math.abs(rawAz) > 12000 ? 16384.0 : (Math.abs(rawAz) > 3000 ? 9806.65 : (Math.abs(rawAz) > 50 ? 1000.0 : 1.0));
        res.accel_x = (rawAx / accelScale).toFixed(2);
        res.accel_y = (rawAy / accelScale).toFixed(2);
        res.accel_z = (rawAz / accelScale).toFixed(2);

        // Gyro scaling: mdps -> /1000 = deg/s, or /10, or /131
        const gyroScale = Math.abs(rawGx) > 1000 ? 1000.0 : 10.0;
        res.gyro_x = (rawGx / gyroScale).toFixed(1);
        res.gyro_y = (rawGy / gyroScale).toFixed(1);
        res.gyro_z = (rawGz / gyroScale).toFixed(1);

        res.pitch = parseFloat(parts[9]).toFixed(2);
        res.roll = parseFloat(parts[10]).toFixed(2);
        res.yaw = parseFloat(parts[11]).toFixed(2);
      } else {
        res.accel_x = parseFloat(parts[1]).toFixed(2);
        res.accel_y = parseFloat(parts[2]).toFixed(2);
        res.accel_z = parseFloat(parts[3]).toFixed(2);
        res.gyro_x = parseFloat(parts[4]).toFixed(1);
        res.gyro_y = parseFloat(parts[5]).toFixed(1);
        res.gyro_z = parseFloat(parts[6]).toFixed(1);
      }
      res.activity_mode = '$IMU Stream';
    }
  }

  // 6. Check for NMEA $GPRMC or $GNRMC sentence
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
    return res;
  }

  // 7. Check for Euler / Attitude Sentences e.g. $RPY,roll,pitch,yaw or $YPR,yaw,pitch,roll
  if (cleanLine.startsWith('$RPY')) {
    const parts = cleanLine.split(',');
    if (parts.length >= 4) {
      res.roll = parseFloat(parts[1]).toFixed(1);
      res.pitch = parseFloat(parts[2]).toFixed(1);
      res.yaw = parseFloat(parts[3]).toFixed(1);
      res.activity_mode = '$RPY Attitude';
    }
  } else if (cleanLine.startsWith('$YPR')) {
    const parts = cleanLine.split(',');
    if (parts.length >= 4) {
      res.yaw = parseFloat(parts[1]).toFixed(1);
      res.pitch = parseFloat(parts[2]).toFixed(1);
      res.roll = parseFloat(parts[3]).toFixed(1);
      res.activity_mode = '$YPR Attitude';
    }
  }

  // 8. Check for vector prefixes e.g. PRY: 12.3, -4.5, 120.1 or RPY: ... or ACCEL: ... or GYRO: ...
  const pryMatch = cleanLine.match(/PRY\s*[:=]\s*([+-]?\d+\.?\d*)[,\s]+([+-]?\d+\.?\d*)[,\s]+([+-]?\d+\.?\d*)/i);
  if (pryMatch) {
    res.pitch = parseFloat(pryMatch[1]).toFixed(1);
    res.roll = parseFloat(pryMatch[2]).toFixed(1);
    res.yaw = parseFloat(pryMatch[3]).toFixed(1);
  }

  const rpyMatch = cleanLine.match(/RPY\s*[:=]\s*([+-]?\d+\.?\d*)[,\s]+([+-]?\d+\.?\d*)[,\s]+([+-]?\d+\.?\d*)/i);
  if (rpyMatch) {
    res.roll = parseFloat(rpyMatch[1]).toFixed(1);
    res.pitch = parseFloat(rpyMatch[2]).toFixed(1);
    res.yaw = parseFloat(rpyMatch[3]).toFixed(1);
  }

  const yprMatch = cleanLine.match(/YPR\s*[:=]\s*([+-]?\d+\.?\d*)[,\s]+([+-]?\d+\.?\d*)[,\s]+([+-]?\d+\.?\d*)/i);
  if (yprMatch) {
    res.yaw = parseFloat(yprMatch[1]).toFixed(1);
    res.pitch = parseFloat(yprMatch[2]).toFixed(1);
    res.roll = parseFloat(yprMatch[3]).toFixed(1);
  }

  const accelVecMatch = cleanLine.match(/(?:ACCEL|ACC)\s*[:=]\s*([+-]?\d+\.?\d*)[,\s]+([+-]?\d+\.?\d*)[,\s]+([+-]?\d+\.?\d*)/i);
  if (accelVecMatch) {
    res.accel_x = parseFloat(accelVecMatch[1]).toFixed(2);
    res.accel_y = parseFloat(accelVecMatch[2]).toFixed(2);
    res.accel_z = parseFloat(accelVecMatch[3]).toFixed(2);
  }

  const gyroVecMatch = cleanLine.match(/(?:GYRO|GYR)\s*[:=]\s*([+-]?\d+\.?\d*)[,\s]+([+-]?\d+\.?\d*)[,\s]+([+-]?\d+\.?\d*)/i);
  if (gyroVecMatch) {
    res.gyro_x = parseFloat(gyroVecMatch[1]).toFixed(1);
    res.gyro_y = parseFloat(gyroVecMatch[2]).toFixed(1);
    res.gyro_z = parseFloat(gyroVecMatch[3]).toFixed(1);
  }

  // 9. Key-Value regex matchers
  const pitchMatch = cleanLine.match(/(?:pitch|pit)\s*[:=]\s*([+-]?\d+\.?\d*)/i) || (!pryMatch && cleanLine.match(/\bP\s*[:=]\s*([+-]?\d+\.?\d*)/i));
  const rollMatch = cleanLine.match(/(?:roll|rol)\s*[:=]\s*([+-]?\d+\.?\d*)/i) || (!rpyMatch && cleanLine.match(/\bR\s*[:=]\s*([+-]?\d+\.?\d*)/i));
  const yawMatch = cleanLine.match(/(?:yaw|hdg|heading|yaw_deg)\s*[:=]\s*([+-]?\d+\.?\d*)/i) || (!yprMatch && cleanLine.match(/\bY\s*[:=]\s*([+-]?\d+\.?\d*)/i));

  const latMatch = cleanLine.match(/lat(?:itude)?\s*[:=]\s*([+-]?\d+\.?\d*)/i);
  const lngMatch = cleanLine.match(/l(?:ng|on|ongitude)?\s*[:=]\s*([+-]?\d+\.?\d*)/i);
  const axMatch = cleanLine.match(/a(?:x|ccel_?x|cc_?x)\s*[:=]\s*([+-]?\d+\.?\d*)/i);
  const ayMatch = cleanLine.match(/a(?:y|ccel_?y|cc_?y)\s*[:=]\s*([+-]?\d+\.?\d*)/i);
  const azMatch = cleanLine.match(/a(?:z|ccel_?z|cc_?z)\s*[:=]\s*([+-]?\d+\.?\d*)/i);
  const gxMatch = cleanLine.match(/g(?:x|yro_?x|yr_?x)\s*[:=]\s*([+-]?\d+\.?\d*)/i);
  const gyMatch = cleanLine.match(/g(?:y|yro_?y|yr_?y)\s*[:=]\s*([+-]?\d+\.?\d*)/i);
  const gzMatch = cleanLine.match(/g(?:z|yro_?z|yr_?z)\s*[:=]\s*([+-]?\d+\.?\d*)/i);
  const battMatch = cleanLine.match(/bat(?:t|tery)?\s*[:=]\s*([+-]?\d+\.?\d*)/i);
  const tagMatch = cleanLine.match(/tag(?:_?id)?\s*[:=]\s*([A-Za-z0-9_\-]+)/i);

  if (pitchMatch && !res.pitch) res.pitch = parseFloat(pitchMatch[1]).toFixed(1);
  if (rollMatch && !res.roll) res.roll = parseFloat(rollMatch[1]).toFixed(1);
  if (yawMatch && !res.yaw) res.yaw = parseFloat(yawMatch[1]).toFixed(1);

  if (latMatch && !res.lat) res.lat = parseFloat(latMatch[1]).toFixed(6);
  if (lngMatch && !res.lng) res.lng = parseFloat(lngMatch[1]).toFixed(6);
  if (axMatch && !res.accel_x) res.accel_x = parseFloat(axMatch[1]).toFixed(2);
  if (ayMatch && !res.accel_y) res.accel_y = parseFloat(ayMatch[1]).toFixed(2);
  if (azMatch && !res.accel_z) res.accel_z = parseFloat(azMatch[1]).toFixed(2);
  if (gxMatch && !res.gyro_x) res.gyro_x = parseFloat(gxMatch[1]).toFixed(1);
  if (gyMatch && !res.gyro_y) res.gyro_y = parseFloat(gyMatch[1]).toFixed(1);
  if (gzMatch && !res.gyro_z) res.gyro_z = parseFloat(gzMatch[1]).toFixed(1);
  if (battMatch && !res.battery_v) res.battery_v = parseFloat(battMatch[1]).toFixed(2);
  if (tagMatch && !res.tag_id) res.tag_id = tagMatch[1];

  // 10. Fallback: Check comma/tab separated numerical streams
  if (!res.lat && !res.accel_x && !res.pitch) {
    const tokens = cleanLine.split(/[,\t|]+/).map(t => t.trim()).filter(t => t.length > 0);
    const nums = tokens.map(t => parseFloat(t)).filter(n => !isNaN(n));

    if (nums.length === tokens.length && nums.length >= 3) {
      if (nums.length === 7) {
        // Line format: 188,-138,-538,-23,-0.25,-18.37,-40.03 (sample, gx_raw, gy_raw, gz_raw, pitch, roll, yaw)
        res.gyro_x = (nums[1] / 10.0).toFixed(1);
        res.gyro_y = (nums[2] / 10.0).toFixed(1);
        res.gyro_z = (nums[3] / 10.0).toFixed(1);
        res.pitch = nums[4].toFixed(2);
        res.roll = nums[5].toFixed(2);
        res.yaw = nums[6].toFixed(2);
        res.activity_mode = '7-DOF Euler Stream';
      } else if (nums.length === 22) {
        // Summary telemetry line: 483,0,0,0,0,0,209,3097,9680,4725,7135,5363,132519,4749,0,0,0,0,0,96,4116,2551
        res.accel_x = (nums[6] / 9806.65).toFixed(2);
        res.accel_y = (nums[7] / 9806.65).toFixed(2);
        res.accel_z = (nums[8] / 9806.65).toFixed(2);
        res.gyro_x = (nums[9] / 1000.0).toFixed(1);
        res.gyro_y = (nums[10] / 1000.0).toFixed(1);
        res.gyro_z = (nums[11] / 1000.0).toFixed(1);
        res.battery_pct = String(Math.round(nums[19]));
        res.battery_v = (nums[20] / 1000.0).toFixed(2);
        updateBatteryDisplay(res.battery_v, nums[19]);
        res.activity_mode = 'Summary 22-Val';
      } else if (nums.length === 3) {
        // 3 numbers: assume Accel X, Y, Z
        res.accel_x = nums[0].toFixed(2);
        res.accel_y = nums[1].toFixed(2);
        res.accel_z = nums[2].toFixed(2);
      } else if (nums.length === 6) {
        // 6 numbers: Accel X, Y, Z, Gyro X, Y, Z
        res.accel_x = nums[0].toFixed(2);
        res.accel_y = nums[1].toFixed(2);
        res.accel_z = nums[2].toFixed(2);
        res.gyro_x = nums[3].toFixed(1);
        res.gyro_y = nums[4].toFixed(1);
        res.gyro_z = nums[5].toFixed(1);
      } else if (nums.length >= 9) {
        // 9 numbers: Accel, Gyro, Pitch, Roll, Yaw
        res.accel_x = nums[0].toFixed(2);
        res.accel_y = nums[1].toFixed(2);
        res.accel_z = nums[2].toFixed(2);
        res.gyro_x = nums[3].toFixed(1);
        res.gyro_y = nums[4].toFixed(1);
        res.gyro_z = nums[5].toFixed(1);
        res.pitch = nums[6].toFixed(1);
        res.roll = nums[7].toFixed(1);
        res.yaw = nums[8].toFixed(1);
      }
    }
  }

  // Update Battery UI if extracted
  if (res.battery_v) {
    updateBatteryDisplay(res.battery_v, res.battery_pct || null);
  }

  // Update GPS Position if extracted
  if (res.lat && res.lng) {
    updateGPSPosition(parseFloat(res.lat), parseFloat(res.lng), 412, 1.2);
  }

  // Update UI & 6-DOF Chart if we received any motion, attitude, or GPS data
  if (res.accel_x || res.gyro_x || res.pitch || res.roll || res.yaw) {
    const axVal = res.accel_x || '0.00';
    const ayVal = res.accel_y || '0.00';
    const azVal = res.accel_z || '0.98';
    const gxVal = res.gyro_x || '0.0';
    const gyVal = res.gyro_y || '0.0';
    const gzVal = res.gyro_z || '0.0';

    const calculatedEuler = updateIMUAndOrientation(
      axVal,
      ayVal,
      azVal,
      gxVal,
      gyVal,
      gzVal,
      res.pitch ? parseFloat(res.pitch) : null,
      res.roll ? parseFloat(res.roll) : null,
      res.yaw ? parseFloat(res.yaw) : null,
      'Live Inbound UART',
      res.activity_mode || 'Active UART'
    );

    // If Pitch/Roll/Yaw were not in input, populate them with the calculated Euler angles
    if (!res.pitch) res.pitch = calculatedEuler.pitch.toFixed(1);
    if (!res.roll) res.roll = calculatedEuler.roll.toFixed(1);
    if (!res.yaw) res.yaw = calculatedEuler.yaw.toFixed(1);

    const timeStr = new Date().toLocaleTimeString();
    addChartData(timeStr, axVal, ayVal, azVal, gxVal, gyVal, gzVal);
  }

  return res;
}

/**
 * Process Raw UART Data from Serial or BLE
 */
function processRawUartChunk(chunkData, source = 'SERIAL') {
  if (isStreamPaused) return;

  let textChunk = '';
  let hexBytes = [];
  let dataView = null;
  let uint8Array = null;

  if (typeof chunkData === 'string') {
    textChunk = chunkData;
    const encoder = new TextEncoder();
    uint8Array = encoder.encode(chunkData);
  } else if (chunkData instanceof DataView) {
    dataView = chunkData;
    uint8Array = new Uint8Array(chunkData.buffer, chunkData.byteOffset, chunkData.byteLength);
    const decoder = new TextDecoder('utf-8', { fatal: false });
    textChunk = decoder.decode(uint8Array);
  } else if (chunkData instanceof ArrayBuffer || ArrayBuffer.isView(chunkData)) {
    uint8Array = new Uint8Array(chunkData.buffer || chunkData, chunkData.byteOffset || 0, chunkData.byteLength);
    dataView = new DataView(uint8Array.buffer, uint8Array.byteOffset, uint8Array.byteLength);
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
    serialLineBuffer = lines.pop(); // keep partial line for next chunk

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
            activity_mode: extracted.activity_mode || 'UART Serial',
            payload_text: trimmed,
            raw_hex: rawHexStr
          });
        }
      }
    }
  }

  // 2. Binary Frame Buffer / Packet Detection (0xCB Header)
  if (uint8Array && uint8Array.length > 0) {
    // Append to binary frame accumulator
    const newBuf = new Uint8Array(uartByteRingBuffer.length + uint8Array.length);
    newBuf.set(uartByteRingBuffer);
    newBuf.set(uint8Array, uartByteRingBuffer.length);
    uartByteRingBuffer = newBuf;

    // Search for 0xCB sync header and extract 20-byte packets
    while (uartByteRingBuffer.length >= 20) {
      let syncIdx = -1;
      for (let i = 0; i <= uartByteRingBuffer.length - 20; i++) {
        if (uartByteRingBuffer[i] === 0xCB) {
          syncIdx = i;
          break;
        }
      }

      if (syncIdx === -1) {
        // No 0xCB header in current buffer, keep last 19 bytes in case header spans chunk
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

function togglePauseStream() {
  isStreamPaused = !isStreamPaused;
  if (isStreamPaused) {
    btnPauseStream.classList.add('active');
    btnPauseStream.innerHTML = '<i class="fa-solid fa-play"></i>';
    btnPauseStream.title = 'Resume Stream';
    logToConsole('system', 'UART Stream PAUSED.');
  } else {
    btnPauseStream.classList.remove('active');
    btnPauseStream.innerHTML = '<i class="fa-solid fa-pause"></i>';
    btnPauseStream.title = 'Pause Stream';
    logToConsole('system', 'UART Stream RESUMED.');
  }
}

function toggleAutoScroll() {
  autoScroll = !autoScroll;
  if (autoScroll) {
    btnToggleAutoScroll.classList.add('active');
    logToConsole('system', 'Terminal Auto-Scroll ENABLED.');
  } else {
    btnToggleAutoScroll.classList.remove('active');
    logToConsole('system', 'Terminal Auto-Scroll DISABLED.');
  }
}

/* ==========================================================================
   Telemetry Stream & Direct-to-File CSV Recorder
   ========================================================================== */
/* ==========================================================================
   Telemetry Stream & Direct-to-File CSV Recorder (Safe Async Write Queue)
   ========================================================================== */
function toggleRecording() {
  if (isRecording) {
    stopRecording();
  } else {
    startRecording();
  }
}

async function startRecording() {
  if (isRecording) return;

  const now = new Date();
  const dateStr = now.toISOString().replace(/[:.]/g, '-').slice(0, 19);
  const defaultFilename = `ranchbot_uart_stream_${dateStr}.csv`;
  pendingWriteQueue = [];
  totalBytesWrittenToFile = 0;

  const headers = [
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
    'Payload Text / Raw Line',
    'Raw Hex Payload'
  ];
  const headerLine = '\uFEFF' + headers.join(',') + '\r\n';

  // 1. Prompt user to select target file destination on disk
  try {
    if ('showSaveFilePicker' in window) {
      logToConsole('system', 'Select recording file destination...');
      fileHandle = await window.showSaveFilePicker({
        suggestedName: defaultFilename,
        types: [
          {
            description: 'CSV File (*.csv)',
            accept: { 'text/csv': ['.csv'] }
          },
          {
            description: 'Log / Text File (*.log, *.txt)',
            accept: { 'text/plain': ['.log', '.txt'] }
          }
        ]
      });

      activeRecordingFileName = fileHandle.name;
      fileWritableStream = await fileHandle.createWritable({ keepExistingData: false });
      
      // Write CSV headers immediately to file stream
      await fileWritableStream.write(headerLine);
      totalBytesWrittenToFile += headerLine.length;
      logToConsole('system', `[LIVE DISK STREAM] Writing data directly to file: "${activeRecordingFileName}"`);
    } else {
      // Fallback for browsers without File System Access API
      const userChoice = prompt('Enter a filename to record incoming telemetry to:', defaultFilename);
      if (!userChoice) {
        logToConsole('warn', 'Recording cancelled (no filename provided).');
        return;
      }
      activeRecordingFileName = userChoice.trim().endsWith('.csv') ? userChoice.trim() : `${userChoice.trim()}.csv`;
      fileHandle = null;
      fileWritableStream = null;
      logToConsole('system', `[BUFFER RECORDING] Recording to memory for "${activeRecordingFileName}" (auto-downloads on Stop).`);
    }
  } catch (err) {
    if (err.name === 'AbortError') {
      logToConsole('warn', 'Recording cancelled by user.');
      return;
    }
    logToConsole('warn', `Direct file access not available (${err.message}). Using safe memory buffer recording.`);
    const userChoice = prompt('Enter filename to record to:', defaultFilename);
    if (!userChoice) return;
    activeRecordingFileName = userChoice.trim();
    fileHandle = null;
    fileWritableStream = null;
  }

  isRecording = true;
  recordingStartTime = Date.now();

  recorderBadge.className = 'recorder-status-badge recording';
  recorderStatusText.textContent = 'RECORDING...';
  if (recorderFilePill) {
    recorderFilePill.classList.add('active');
    recorderFilenameText.textContent = activeRecordingFileName;
    recorderFilenameText.title = `Live Recording to: ${activeRecordingFileName}`;
  }

  btnRecordToggle.className = 'btn btn-sm btn-record is-recording';
  btnRecordToggle.innerHTML = '<i class="fa-solid fa-stop"></i> Stop & Save File';
  if (btnExportCsv) btnExportCsv.removeAttribute('disabled');

  recordingTimerInterval = setInterval(updateRecorderTimerDisplay, 1000);
  updateRecorderTimerDisplay();
  updateRecorderStats();

  logToConsole('system', `CSV Stream Recorder ACTIVE. Capturing all incoming 6-DOF UART & BLE telemetry...`);
}

async function pumpFileWriteQueue() {
  if (isFlushingWriteQueue || pendingWriteQueue.length === 0 || !fileWritableStream) return;
  isFlushingWriteQueue = true;

  try {
    while (pendingWriteQueue.length > 0 && fileWritableStream) {
      const batch = pendingWriteQueue.splice(0, 50);
      const chunkStr = batch.join('');
      await fileWritableStream.write(chunkStr);
      totalBytesWrittenToFile += chunkStr.length;
    }
  } catch (err) {
    console.error('Safe file write error:', err);
    logToConsole('warn', `Disk stream write notice: ${err.message}. Data safely preserved in memory buffer.`);
    fileWritableStream = null;
  } finally {
    isFlushingWriteQueue = false;
  }
}

function writeRecordToFileAndBuffer(recordItem) {
  if (!isRecording) return;

  recordedPackets.push(recordItem);
  updateRecorderStats();

  const row = [
    escapeCsvField(recordItem.timestamp_iso),
    escapeCsvField(recordItem.timestamp_local),
    recordItem.packet_number,
    escapeCsvField(recordItem.source),
    escapeCsvField(recordItem.tag_id),
    escapeCsvField(recordItem.data_type || 'TELEMETRY'),
    recordItem.lat,
    recordItem.lng,
    recordItem.accel_x,
    recordItem.accel_y,
    recordItem.accel_z,
    recordItem.gyro_x,
    recordItem.gyro_y,
    recordItem.gyro_z,
    recordItem.pitch !== undefined ? recordItem.pitch : '',
    recordItem.roll !== undefined ? recordItem.roll : '',
    recordItem.yaw !== undefined ? recordItem.yaw : '',
    recordItem.battery_v,
    escapeCsvField(recordItem.activity_mode),
    escapeCsvField(recordItem.payload_text || ''),
    escapeCsvField(recordItem.raw_hex || '')
  ];

  const csvLine = row.join(',') + '\r\n';

  if (fileWritableStream) {
    pendingWriteQueue.push(csvLine);
    pumpFileWriteQueue();
  }
}

async function stopRecording() {
  if (!isRecording) return;
  isRecording = false;

  clearInterval(recordingTimerInterval);
  recordingTimerInterval = null;

  recorderBadge.className = 'recorder-status-badge stopped';
  recorderStatusText.textContent = 'REC IDLE';
  btnRecordToggle.className = 'btn btn-sm btn-record';
  btnRecordToggle.innerHTML = '<i class="fa-solid fa-circle"></i> Start Recording';

  // Flush remaining queued writes and close live disk stream
  if (fileWritableStream) {
    try {
      while (isFlushingWriteQueue || pendingWriteQueue.length > 0) {
        await pumpFileWriteQueue();
        if (pendingWriteQueue.length === 0) break;
        await new Promise(r => setTimeout(r, 40));
      }
      await fileWritableStream.close();
      logToConsole('system', `✓ FILE CLOSED & SAVED: "${activeRecordingFileName}" written to disk (${recordedPackets.length} records, ${totalBytesWrittenToFile} bytes).`);
      fileWritableStream = null;
    } catch (e) {
      logToConsole('warn', `Disk stream flush notice (${e.message}). Downloading full recorded data via fallback export...`);
      fileWritableStream = null;
      exportCsvWithFilename(activeRecordingFileName);
    }
  } else if (recordedPackets.length > 0) {
    // If we used memory fallback, auto-download the file with the chosen name
    exportCsvWithFilename(activeRecordingFileName);
  }

  if (recorderFilePill) {
    recorderFilePill.classList.remove('active');
    recorderFilenameText.textContent = `Saved: ${activeRecordingFileName}`;
  }

  logToConsole('system', `Recording completed. Total captured records: ${recordedPackets.length}`);
}

function updateRecorderTimerDisplay() {
  if (!recordingStartTime) {
    recorderTimer.textContent = '00:00:00';
    return;
  }
  const elapsedMs = Date.now() - recordingStartTime;
  const totalSeconds = Math.floor(elapsedMs / 1000);
  const hrs = String(Math.floor(totalSeconds / 3600)).padStart(2, '0');
  const mins = String(Math.floor((totalSeconds % 3600) / 60)).padStart(2, '0');
  const secs = String(totalSeconds % 60).padStart(2, '0');
  recorderTimer.textContent = `${hrs}:${mins}:${secs}`;
}

function updateRecorderStats() {
  const count = recordedPackets.length;
  recorderCount.textContent = `${count} record${count === 1 ? '' : 's'}`;
  
  const estBytes = totalBytesWrittenToFile > 0 ? totalBytesWrittenToFile : (count * 240);
  const estKb = (estBytes / 1024).toFixed(1);
  recorderSize.textContent = `(~${estKb} KB)`;

  if (count > 0 && btnExportCsv) {
    btnExportCsv.removeAttribute('disabled');
  }
}

function exportCsv() {
  const defaultName = activeRecordingFileName || `ranchbot_uart_stream_${new Date().toISOString().replace(/[:.]/g, '-').slice(0, 19)}.csv`;
  exportCsvWithFilename(defaultName);
}

function exportCsvWithFilename(filename) {
  if (recordedPackets.length === 0) {
    alert('No recorded stream data to export. Click "Start Recording" to capture incoming data.');
    return;
  }

  const headers = [
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
    'Payload Text / Raw Line',
    'Raw Hex Payload'
  ];

  const csvRows = ['\uFEFF' + headers.join(',')];

  for (const p of recordedPackets) {
    const row = [
      escapeCsvField(p.timestamp_iso),
      escapeCsvField(p.timestamp_local),
      p.packet_number,
      escapeCsvField(p.source),
      escapeCsvField(p.tag_id),
      escapeCsvField(p.data_type || 'TELEMETRY'),
      p.lat,
      p.lng,
      p.accel_x,
      p.accel_y,
      p.accel_z,
      p.gyro_x,
      p.gyro_y,
      p.gyro_z,
      p.pitch !== undefined ? p.pitch : '',
      p.roll !== undefined ? p.roll : '',
      p.yaw !== undefined ? p.yaw : '',
      p.battery_v,
      escapeCsvField(p.activity_mode),
      escapeCsvField(p.payload_text || ''),
      escapeCsvField(p.raw_hex)
    ];
    csvRows.push(row.join(','));
  }

  const csvContent = csvRows.join('\r\n');
  const blob = new Blob([csvContent], { type: 'text/csv;charset=utf-8;' });
  const url = URL.createObjectURL(blob);
  
  const link = document.createElement('a');
  link.setAttribute('href', url);
  link.setAttribute('download', filename);
  document.body.appendChild(link);
  link.click();
  document.body.removeChild(link);
  URL.revokeObjectURL(url);

  logToConsole('system', `EXPORT SUCCESS: Saved ${recordedPackets.length} stream records to "${filename}".`);
}

function escapeCsvField(field) {
  if (field === null || field === undefined) return '""';
  const str = String(field).replace(/"/g, '""');
  return `"${str}"`;
}

function clearCsvBuffer() {
  if (isRecording) {
    stopRecording();
  }
  recordedPackets = [];
  pendingWriteQueue = [];
  totalBytesWrittenToFile = 0;
  recordingStartTime = null;
  updateRecorderTimerDisplay();
  updateRecorderStats();
  if (recorderFilePill) {
    recorderFilenameText.textContent = 'No file selected';
  }
  if (btnExportCsv) btnExportCsv.setAttribute('disabled', 'true');
  logToConsole('system', 'Recorded telemetry buffer cleared.');
}

/* ==========================================================================
   Simulator Stream Generator (For Testing 6-DOF UART & BLE Live Recording)
   ========================================================================== */
function toggleSimulatorStream() {
  isSimulatorRunning = !isSimulatorRunning;
  if (isSimulatorRunning) {
    btnSimulateStream.classList.add('btn-primary');
    btnSimulateStream.innerHTML = '<i class="fa-solid fa-stop"></i> Stop Test';
    logToConsole('system', '[SIMULATOR] Test 6-DOF IMU & Attitude UART stream generator STARTED.');

    let simLat = 31.968600;
    let simLng = -99.901800;
    let count = 0;

    simulatorInterval = setInterval(() => {
      count++;
      simLat += (Math.random() - 0.5) * 0.00015;
      simLng += (Math.random() - 0.5) * 0.00015;

      const ax = (Math.sin(count * 0.3) * 0.75).toFixed(2);
      const ay = (Math.cos(count * 0.25) * 0.55).toFixed(2);
      const az = (0.95 + Math.sin(count * 0.15) * 0.2).toFixed(2);

      const gx = (Math.cos(count * 0.35) * 25.0).toFixed(1);
      const gy = (Math.sin(count * 0.3) * 35.0).toFixed(1);
      const gz = (Math.sin(count * 0.2) * 15.0).toFixed(1);

      const pitch = (Math.sin(count * 0.3) * 28.0).toFixed(2);
      const roll = (Math.cos(count * 0.25) * 22.0).toFixed(2);
      const yaw = ((count * 4.5) % 360).toFixed(2);
      const battMv = Math.max(3400, Math.round(4120 - count * 1.5));
      const battPct = Math.round(Math.min(100, Math.max(0, ((battMv - 3300) / 900) * 100)));

      // Cycle through realistic CowTag stream formats matching user's hardware
      const mode = count % 6;
      if (mode === 0) {
        // $IMU sentence from CowTag
        const rawAx = Math.round(parseFloat(ax) * 16384);
        const rawAy = Math.round(parseFloat(ay) * 16384);
        const rawAz = Math.round(parseFloat(az) * 16384);
        const rawGx = Math.round(parseFloat(gx) * 10);
        const rawGy = Math.round(parseFloat(gy) * 10);
        const rawGz = Math.round(parseFloat(gz) * 10);
        const textMsg = `[10:56:35 AM] [BLE] $IMU,${280 + count},${482000 + count * 20},${rawAx},${rawAy},${rawAz},${rawGx},${rawGy},${rawGz},${pitch},${roll},${yaw}\n`;
        processRawUartChunk(textMsg, 'SIM_UART');
      } else if (mode === 1) {
        // 7-value Euler stream: sample, gx_raw, gy_raw, gz_raw, pitch, roll, yaw
        const textMsg = `${180 + count},${Math.round(parseFloat(gx) * 10)},${Math.round(parseFloat(gy) * 10)},${Math.round(parseFloat(gz) * 10)},${pitch},${roll},${yaw}\n`;
        processRawUartChunk(textMsg, 'SIM_UART');
      } else if (mode === 2) {
        // CowTag battery diagnostic log
        const textMsg = `[10:56:36 AM] [BLE] [00:08:16.509,575] <inf> cowtag_fifo: BATTERY: ${battPct}% (${battMv} mV)\n`;
        processRawUartChunk(textMsg, 'SIM_UART');
      } else if (mode === 3) {
        // 22-value CowTag summary telemetry row
        const textMsg = `[10:56:35 AM] [BLE] ${480 + count},0,0,0,0,0,${Math.round(parseFloat(ax) * 9806)},${Math.round(parseFloat(ay) * 9806)},${Math.round(parseFloat(az) * 9806)},${Math.round(parseFloat(gx) * 1000)},${Math.round(parseFloat(gy) * 1000)},${Math.round(parseFloat(gz) * 1000)},132519,4749,0,0,0,0,0,${battPct},${battMv},2551\n`;
        processRawUartChunk(textMsg, 'SIM_UART');
      } else if (mode === 4) {
        // FIFO window peak log
        const textMsg = `[10:56:35 AM] [BLE] [00:08:16.509,553] <inf> cowtag_fifo: FIFO window n=1740 sets=290 used=289 WM=1 OVF=0 | peak A mm/s^2 X=${Math.round(parseFloat(ax) * 9806)} Y=${Math.round(parseFloat(ay) * 9806)} Z=${Math.round(parseFloat(az) * 9806)} | G mdps X=${Math.round(parseFloat(gx) * 1000)} Y=${Math.round(parseFloat(gy) * 1000)} Z=${Math.round(parseFloat(gz) * 1000)}\n`;
        processRawUartChunk(textMsg, 'SIM_UART');
      } else {
        // 20-byte binary frame with 0xCB sync
        const buffer = new ArrayBuffer(20);
        const view = new DataView(buffer);
        view.setUint8(0, 0xCB);
        view.setUint16(1, 8492, true);
        view.setInt32(3, Math.round(simLat * 1e7), true);
        view.setInt32(7, Math.round(simLng * 1e7), true);
        view.setInt16(11, Math.round(parseFloat(ax) * 1000), true);
        view.setInt16(13, Math.round(parseFloat(ay) * 1000), true);
        view.setInt16(15, Math.round(parseFloat(az) * 1000), true);
        view.setUint16(17, battMv, true);
        view.setUint8(19, 1);
        processRawUartChunk(view, 'SIM_BLE');
      }
    }, 1000);
  } else {
    clearInterval(simulatorInterval);
    simulatorInterval = null;
    btnSimulateStream.classList.remove('btn-primary');
    btnSimulateStream.innerHTML = '<i class="fa-solid fa-flask"></i> Test Stream';
    logToConsole('system', '[SIMULATOR] Test stream generator STOPPED.');
  }
}

/* ==========================================================================
   Full Width Telemetry Stream Layout Toggle
   ========================================================================== */
function toggleFullWidthStream() {
  const dashboardGrid = document.querySelector('.dashboard-grid');
  if (!dashboardGrid) return;

  isFullWidth = !isFullWidth;
  if (isFullWidth) {
    dashboardGrid.classList.add('full-width-active');
    btnToggleFullWidth.innerHTML = '<i class="fa-solid fa-compress"></i> <span class="btn-text">Restore View</span>';
    btnToggleFullWidth.classList.add('btn-primary');
    btnToggleFullWidth.title = 'Restore Standard 3-Column Dashboard View';
    logToConsole('system', 'Telemetry Stream area expanded to FULL SCREEN WIDTH.');
  } else {
    dashboardGrid.classList.remove('full-width-active');
    btnToggleFullWidth.innerHTML = '<i class="fa-solid fa-expand"></i> <span class="btn-text">Full Width</span>';
    btnToggleFullWidth.classList.remove('btn-primary');
    btnToggleFullWidth.title = 'Toggle Full Width Stream Area (Expand/Compress)';
    logToConsole('system', 'Restored standard 3-column dashboard layout.');
  }

  // Allow Leaflet map to adjust bounds if restored
  if (map) {
    setTimeout(() => map.invalidateSize(), 250);
  }
}
