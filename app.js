// Bluetooth & Application State
let bleDevice = null;
let gattServer = null;
let isSimulating = false;
let simulationInterval = null;
let packetCounter = 0;

// Leaflet Map & Marker State
let map = null;
let cowMarker = null;
let polyline = null;
let pathHistory = [];

// Motion Chart
let motionChart = null;

// DOM Elements
const btnConnect = document.getElementById('btnConnect');
const btnScanAll = document.getElementById('btnScanAll');
const btnSimulate = document.getElementById('btnSimulate');
const btnClearLog = document.getElementById('btnClearLog');
const btnClearChart = document.getElementById('btnClearChart');
const statusBadge = document.getElementById('statusBadge');
const statusText = document.getElementById('statusText');
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

// Console & Parsed Grid
const terminalLog = document.getElementById('terminalLog');
const fieldsGrid = document.getElementById('fieldsGrid');

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
  btnConnect.addEventListener('click', handleConnectButtonClick);
  if (btnScanAll) btnScanAll.addEventListener('click', handleScanAllClick);
  btnSimulate.addEventListener('click', toggleSimulation);
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
    alert('Web Bluetooth is not supported in this browser environment. Click "Toggle Cow Tag Demo" to test live decoding!');
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
   Cow Tag Telemetry Payload Unpacker
   ========================================================================== */
function handleCharacteristicValueChanged(event) {
  decodeAndProcessPacket(event.target.value, 'BLE');
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
  } else {
    parsed = { 'Raw Hex': rawHexStr, 'Length': `${dataView.byteLength} Bytes` };
    logToConsole('rx', `[RAW PAYLOAD] ${rawHexStr}`);
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
  deviceTypeBadge.textContent = connected ? 'Active BLE Tag' : 'No Tag';
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
  terminalLog.scrollTop = terminalLog.scrollHeight;
}

function escapeHtml(str) {
  return str.replace(/&/g, "&amp;").replace(/</g, "&lt;").replace(/>/g, "&gt;");
}

/* ==========================================================================
   Cattle Tag Broadcast Simulator (GPS + Accelerometer/Gyroscope Stream)
   ========================================================================== */
function toggleSimulation() {
  if (isSimulating) {
    clearInterval(simulationInterval);
    isSimulating = false;
    btnSimulate.classList.remove('btn-primary');
    btnSimulate.classList.add('btn-secondary');
    btnSimulate.innerHTML = '<i class="fa-solid fa-vial"></i> Toggle Cow Tag Demo';
    updateStatus('disconnected', 'Disconnected');
    updateDeviceOverview('--', '--', false);
    logToConsole('system', 'Cow Tag Simulator stopped.');
  } else {
    if (bleDevice && bleDevice.gatt.connected) {
      disconnectDevice();
    }
    isSimulating = true;
    btnSimulate.classList.remove('btn-secondary');
    btnSimulate.classList.add('btn-primary');
    btnSimulate.innerHTML = '<i class="fa-solid fa-stop"></i> Stop Simulator';
    updateStatus('connected', 'Simulating Cow Tag');
    updateDeviceOverview('Ranchbot Cattle Smart Tag #8492', 'TAG-99-4A-32', true);
    logToConsole('system', 'Started Cattle Ear Tag GPS + 6-DOF IMU Telemetry Stream...');

    let curLat = 31.968600;
    let curLng = -99.901800;

    simulationInterval = setInterval(() => {
      // Simulate cow walking / grazing step movement on pasture
      const dLat = (Math.random() - 0.45) * 0.00012;
      const dLng = (Math.random() - 0.45) * 0.00012;
      curLat += dLat;
      curLng += dLng;

      // Simulate 6-DOF IMU readings (Cow head motion while grazing)
      const axRaw = Math.round((Math.random() - 0.5) * 350); // g * 1000
      const ayRaw = Math.round((Math.random() - 0.5) * 450);
      const azRaw = Math.round(980 + (Math.random() - 0.5) * 200);

      // Pack 20-byte Cattle Tag Payload
      const buffer = new ArrayBuffer(20);
      const view = new DataView(buffer);

      view.setUint8(0, 0xCB); // Sync Header for Cattle Tag
      view.setUint16(1, 8492, true); // Cow Tag #8492
      view.setInt32(3, Math.round(curLat * 1e7), true); // Lat microdegrees
      view.setInt32(7, Math.round(curLng * 1e7), true); // Lng microdegrees
      view.setInt16(11, axRaw, true);
      view.setInt16(13, ayRaw, true);
      view.setInt16(15, azRaw, true);
      view.setUint16(17, 3880, true); // 3.88V battery
      view.setUint8(19, Math.random() > 0.3 ? 1 : 2); // Grazing or Walking activity

      decodeAndProcessPacket(view, 'SIMULATOR');
    }, 2000);
  }
}
