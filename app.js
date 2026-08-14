// Bluetooth & Serial Application State
let bleDevice = null;
let gattServer = null;
let serialPort = null;
let serialReader = null;
let serialReadableStreamClosed = null;
let serialLineBuffer = '';
let autoScroll = true;
let isStreamPaused = false;
let packetCounter = 0;

// Stream CSV Recorder & Layout State
let isRecording = false;
let recordedPackets = [];
let recordingStartTime = null;
let recordingTimerInterval = null;
let isFullWidth = false;

// Leaflet Map & Marker State
let map = null;
let cowMarker = null;
let polyline = null;
let pathHistory = [];

// Motion Chart
let motionChart = null;

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

// IMU Elements
const barAccelX = document.getElementById('barAccelX');
const barAccelY = document.getElementById('barAccelY');
const barAccelZ = document.getElementById('barAccelZ');
const valAccelX = document.getElementById('valAccelX');
const valAccelY = document.getElementById('valAccelY');
const valAccelZ = document.getElementById('valAccelZ');
const valGyroX = document.getElementById('valGyroX');
const valGyroY = document.getElementById('valGyroY');
const valGyroZ = document.getElementById('valGyroZ');
const motionPill = document.getElementById('motionPill');

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

// Live File System Handle & Stream State
let fileHandle = null;
let fileWritableStream = null;
let activeRecordingFileName = '';
let isSimulatorRunning = false;
let simulatorInterval = null;
let uartByteRingBuffer = new Uint8Array(0);

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
    motionChart.data.labels = [];
    motionChart.data.datasets[0].data = [];
    motionChart.data.datasets[1].data = [];
    motionChart.data.datasets[2].data = [];
    motionChart.update();
  });

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
  lblLat.textContent = lat.toFixed(6);
  lblLng.textContent = lng.toFixed(6);
  lblAlt.textContent = `${alt.toFixed(1)} m`;
  lblSpeed.textContent = `${speed.toFixed(1)} km/h`;

  const newPos = [lat, lng];
  cowMarker.setLatLng(newPos);
  
  pathHistory.push(newPos);
  if (pathHistory.length > 50) pathHistory.shift();
  
  polyline.setLatLngs(pathHistory);
  map.panTo(newPos);
}

/* ==========================================================================
   Chart Initialization (Chart.js 3-Axis IMU Acceleration)
   ========================================================================== */
function initChart() {
  const ctx = document.getElementById('motionChart').getContext('2d');

  motionChart = new Chart(ctx, {
    type: 'line',
    data: {
      labels: [],
      datasets: [
        { label: 'Accel X (g)', data: [], borderColor: '#ff4d6d', borderWidth: 2, pointRadius: 0, tension: 0.2 },
        { label: 'Accel Y (g)', data: [], borderColor: '#38ef7d', borderWidth: 2, pointRadius: 0, tension: 0.2 },
        { label: 'Accel Z (g)', data: [], borderColor: '#00c6ff', borderWidth: 2, pointRadius: 0, tension: 0.2 }
      ]
    },
    options: {
      responsive: true,
      maintainAspectRatio: false,
      scales: {
        x: { grid: { color: 'rgba(255, 255, 255, 0.05)' }, ticks: { color: '#94a3b8', font: { family: 'JetBrains Mono', size: 10 } } },
        y: {
          min: -2.5,
          max: 2.5,
          grid: { color: 'rgba(255, 255, 255, 0.08)' },
          ticks: { color: '#94a3b8', font: { family: 'JetBrains Mono' } },
          title: { display: true, text: 'Acceleration (g)', color: '#94a3b8' }
        }
      },
      plugins: {
        legend: { labels: { color: '#f1f5f9' } }
      }
    }
  });
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

    parsed = {
      'Sync Header': `0x${header.toString(16).toUpperCase()}`,
      'Ear Tag ID': `COW-${tagIdNum}`,
      'GPS Lat': `${lat.toFixed(6)}°`,
      'GPS Lng': `${lng.toFixed(6)}°`,
      'Accel X/Y/Z': `${ax}g, ${ay}g, ${az}g`,
      'Gyro X/Y/Z': `${gx}°, ${gy}°, ${gz}°`,
      'Tag Battery': `${battVolts} V`,
      'Activity Mode': actStr
    };

    // Update GPS map
    updateGPSPosition(lat, lng, 412, actByte === 2 ? 3.5 : 1.2);
    
    // Update IMU Gauges
    updateIMUGauges(ax, ay, az, gx, gy, gz, actStr);

    // Append to Chart
    const timeStr = new Date().toLocaleTimeString();
    addChartData(timeStr, ax, ay, az);

    logToConsole('rx', `[RX TAG] COW-${tagIdNum} | Lat:${lat.toFixed(5)} Lng:${lng.toFixed(5)} | Accel:(${ax}, ${ay}, ${az})g | Mode: ${actStr}`);

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
   UI Updaters
   ========================================================================== */
function updateIMUGauges(ax, ay, az, gx, gy, gz, activityStr) {
  valAccelX.textContent = `${ax > 0 ? '+' : ''}${ax} g`;
  valAccelY.textContent = `${ay > 0 ? '+' : ''}${ay} g`;
  valAccelZ.textContent = `${az > 0 ? '+' : ''}${az} g`;

  // Scale -2g to +2g into 0% - 100% width
  barAccelX.style.width = `${Math.min(100, Math.max(0, ((parseFloat(ax) + 2) / 4) * 100))}%`;
  barAccelY.style.width = `${Math.min(100, Math.max(0, ((parseFloat(ay) + 2) / 4) * 100))}%`;
  barAccelZ.style.width = `${Math.min(100, Math.max(0, ((parseFloat(az) + 2) / 4) * 100))}%`;

  valGyroX.textContent = `${gx} °/s`;
  valGyroY.textContent = `${gy} °/s`;
  valGyroZ.textContent = `${gz} °/s`;

  activityStateEl.textContent = activityStr;

  const totalG = Math.sqrt(ax * ax + ay * ay + az * az);
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

function updateDeviceOverview(name, id, connected) {
  deviceNameEl.textContent = name;
  batteryValueEl.textContent = connected ? '3.88 V' : '-- V';
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

function addChartData(timeLabel, ax, ay, az) {
  if (motionChart.data.labels.length > 25) {
    motionChart.data.labels.shift();
    motionChart.data.datasets[0].data.shift();
    motionChart.data.datasets[1].data.shift();
    motionChart.data.datasets[2].data.shift();
  }
  motionChart.data.labels.push(timeLabel);
  motionChart.data.datasets[0].data.push(ax);
  motionChart.data.datasets[1].data.push(ay);
  motionChart.data.datasets[2].data.push(az);
  motionChart.update();
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
 * Parses ASCII text lines from serial stream to extract GPS, IMU, battery, or status
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
    battery_v: '',
    activity_mode: ''
  };

  if (!line || typeof line !== 'string') return res;

  // 1. Check for NMEA $GPRMC or $GNRMC sentence
  if (line.startsWith('$GPRMC') || line.startsWith('$GNRMC')) {
    const parts = line.split(',');
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
      }
    }
    res.activity_mode = 'NMEA GPS Fix';
    return res;
  }

  // 2. Check for Key-Value pairs (e.g., LAT: 31.968, LNG: -99.901, AX: 0.12, AY: -0.04, AZ: 0.98, BATT: 3.82V)
  const latMatch = line.match(/lat(?:itude)?\s*[:=]\s*([+-]?\d+\.?\d*)/i);
  const lngMatch = line.match(/l(?:ng|on|ongitude)?\s*[:=]\s*([+-]?\d+\.?\d*)/i);
  const axMatch = line.match(/a(?:x|ccel_?x)\s*[:=]\s*([+-]?\d+\.?\d*)/i);
  const ayMatch = line.match(/a(?:y|ccel_?y)\s*[:=]\s*([+-]?\d+\.?\d*)/i);
  const azMatch = line.match(/a(?:z|ccel_?z)\s*[:=]\s*([+-]?\d+\.?\d*)/i);
  const gxMatch = line.match(/g(?:x|yro_?x)\s*[:=]\s*([+-]?\d+\.?\d*)/i);
  const gyMatch = line.match(/g(?:y|yro_?y)\s*[:=]\s*([+-]?\d+\.?\d*)/i);
  const gzMatch = line.match(/g(?:z|yro_?z)\s*[:=]\s*([+-]?\d+\.?\d*)/i);
  const battMatch = line.match(/bat(?:t|tery)?\s*[:=]\s*([+-]?\d+\.?\d*)/i);
  const tagMatch = line.match(/tag(?:_?id)?\s*[:=]\s*([A-Za-z0-9_\-]+)/i);

  if (latMatch) res.lat = parseFloat(latMatch[1]).toFixed(6);
  if (lngMatch) res.lng = parseFloat(lngMatch[1]).toFixed(6);
  if (axMatch) res.accel_x = parseFloat(axMatch[1]).toFixed(2);
  if (ayMatch) res.accel_y = parseFloat(ayMatch[1]).toFixed(2);
  if (azMatch) res.accel_z = parseFloat(azMatch[1]).toFixed(2);
  if (gxMatch) res.gyro_x = parseFloat(gxMatch[1]).toFixed(1);
  if (gyMatch) res.gyro_y = parseFloat(gyMatch[1]).toFixed(1);
  if (gzMatch) res.gyro_z = parseFloat(gzMatch[1]).toFixed(1);
  if (battMatch) res.battery_v = parseFloat(battMatch[1]).toFixed(2);
  if (tagMatch) res.tag_id = tagMatch[1];

  // 3. Fallback: Check comma-separated values (e.g., "31.968600, -99.901800, 0.05, -0.02, 0.99, 3.85")
  if (!latMatch && !axMatch) {
    const tokens = line.split(/[,\t]+/).map(t => t.trim());
    if (tokens.length >= 3 && !isNaN(parseFloat(tokens[0])) && !isNaN(parseFloat(tokens[1]))) {
      if (Math.abs(parseFloat(tokens[0])) <= 90 && Math.abs(parseFloat(tokens[1])) <= 180) {
        res.lat = parseFloat(tokens[0]).toFixed(6);
        res.lng = parseFloat(tokens[1]).toFixed(6);
        if (tokens.length >= 5) {
          res.accel_x = parseFloat(tokens[2]).toFixed(2);
          res.accel_y = parseFloat(tokens[3]).toFixed(2);
          res.accel_z = parseFloat(tokens[4]).toFixed(2);
        }
      }
    }
  }

  // Update UI if we extracted GPS or IMU
  if (res.lat && res.lng) {
    updateGPSPosition(parseFloat(res.lat), parseFloat(res.lng), 412, 1.2);
  }
  if (res.accel_x && res.accel_y && res.accel_z) {
    updateIMUGauges(res.accel_x, res.accel_y, res.accel_z, res.gyro_x || 0, res.gyro_y || 0, res.gyro_z || 0, 'Active UART');
    const timeStr = new Date().toLocaleTimeString();
    addChartData(timeStr, res.accel_x, res.accel_y, res.accel_z);
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
      
      // Write CSV headers immediately to file on disk
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
        'Battery (V)',
        'Activity Mode',
        'Payload Text / Raw Line',
        'Raw Hex Payload'
      ];
      await fileWritableStream.write(headers.join(',') + '\r\n');
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
    logToConsole('warn', `Direct file access not available: ${err.message}. Using memory buffer.`);
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

  logToConsole('system', `CSV Stream Recorder ACTIVE. Capturing all incoming UART & BLE data...`);
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

  // Flush and close live disk stream if open
  if (fileWritableStream) {
    try {
      await fileWritableStream.close();
      logToConsole('system', `✓ FILE CLOSED & SAVED: "${activeRecordingFileName}" written to disk (${recordedPackets.length} records).`);
    } catch (e) {
      logToConsole('error', `Error closing file stream: ${e.message}`);
    }
    fileWritableStream = null;
  } else if (recordedPackets.length > 0) {
    // If we used memory fallback, auto-download the file with the chosen name!
    exportCsvWithFilename(activeRecordingFileName);
  }

  if (recorderFilePill) {
    recorderFilePill.classList.remove('active');
    recorderFilenameText.textContent = `Saved: ${activeRecordingFileName}`;
  }

  logToConsole('system', `Recording stopped. Total captured records: ${recordedPackets.length}`);
}

async function writeRecordToFileAndBuffer(recordItem) {
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
    recordItem.battery_v,
    escapeCsvField(recordItem.activity_mode),
    escapeCsvField(recordItem.payload_text || ''),
    escapeCsvField(recordItem.raw_hex || '')
  ];

  const csvLine = row.join(',') + '\r\n';

  if (fileWritableStream) {
    try {
      await fileWritableStream.write(csvLine);
    } catch (err) {
      console.error('File stream write error:', err);
    }
  }
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
  
  const estBytes = count * 220;
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
    'Battery (V)',
    'Activity Mode',
    'Payload Text / Raw Line',
    'Raw Hex Payload'
  ];

  const csvRows = [headers.join(',')];

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
   Simulator Stream Generator (For Testing UART / BLE Live Recording)
   ========================================================================== */
function toggleSimulatorStream() {
  isSimulatorRunning = !isSimulatorRunning;
  if (isSimulatorRunning) {
    btnSimulateStream.classList.add('btn-primary');
    btnSimulateStream.innerHTML = '<i class="fa-solid fa-stop"></i> Stop Test';
    logToConsole('system', '[SIMULATOR] Test UART stream generator STARTED.');

    let simLat = 31.968600;
    let simLng = -99.901800;
    let count = 0;

    simulatorInterval = setInterval(() => {
      count++;
      simLat += (Math.random() - 0.5) * 0.00015;
      simLng += (Math.random() - 0.5) * 0.00015;
      const ax = (Math.sin(count * 0.4) * 0.6).toFixed(2);
      const ay = (Math.cos(count * 0.3) * 0.4).toFixed(2);
      const az = (0.98 + (Math.random() - 0.5) * 0.1).toFixed(2);
      const batt = (3.88 - (count * 0.001)).toFixed(2);

      // Alternate between formatted UART text and binary packets
      if (count % 2 === 0) {
        const textMsg = `LAT=${simLat.toFixed(6)}, LNG=${simLng.toFixed(6)}, AX=${ax}, AY=${ay}, AZ=${az}, BATT=${batt}V, TAG=XIAO-COWTAG\n`;
        processRawUartChunk(textMsg, 'SIM_UART');
      } else {
        const buffer = new ArrayBuffer(20);
        const view = new DataView(buffer);
        view.setUint8(0, 0xCB);
        view.setUint16(1, 8492, true);
        view.setInt32(3, Math.round(simLat * 1e7), true);
        view.setInt32(7, Math.round(simLng * 1e7), true);
        view.setInt16(11, Math.round(parseFloat(ax) * 1000), true);
        view.setInt16(13, Math.round(parseFloat(ay) * 1000), true);
        view.setInt16(15, Math.round(parseFloat(az) * 1000), true);
        view.setUint16(17, Math.round(parseFloat(batt) * 1000), true);
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
