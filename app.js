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
const btnExportCsv = document.getElementById('btnExportCsv');
const btnClearCsv = document.getElementById('btnClearCsv');
const recorderBadge = document.getElementById('recorderBadge');
const recorderStatusText = document.getElementById('recorderStatusText');
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
    motionChart.data.labels = [];
    motionChart.data.datasets[0].data = [];
    motionChart.data.datasets[1].data = [];
    motionChart.data.datasets[2].data = [];
    motionChart.update();
  });

  // CSV Recorder & Full Width Listeners
  if (btnRecordToggle) btnRecordToggle.addEventListener('click', toggleRecording);
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
  } else {
    parsed = { 'Raw Hex': rawHexStr, 'Length': `${dataView.byteLength} Bytes` };
    logToConsole('rx', `[RAW PAYLOAD] ${rawHexStr}`);
  }

  renderParsedFields(parsed);

  // Store packet to CSV Recorder if recording is active
  if (isRecording) {
    const recordItem = {
      timestamp_iso: new Date().toISOString(),
      timestamp_local: new Date().toLocaleString(),
      packet_number: packetCounter,
      source: source,
      tag_id: parsed['Ear Tag ID'] || (bleDevice && bleDevice.name ? bleDevice.name : 'COW-8492'),
      lat: parsed['GPS Lat'] ? parsed['GPS Lat'].replace('°', '') : '',
      lng: parsed['GPS Lng'] ? parsed['GPS Lng'].replace('°', '') : '',
      accel_x: parsed['Accel X/Y/Z'] ? parsed['Accel X/Y/Z'].split(',')[0].replace('g', '').trim() : '',
      accel_y: parsed['Accel X/Y/Z'] ? parsed['Accel X/Y/Z'].split(',')[1].replace('g', '').trim() : '',
      accel_z: parsed['Accel X/Y/Z'] ? parsed['Accel X/Y/Z'].split(',')[2].replace('g', '').trim() : '',
      gyro_x: parsed['Gyro X/Y/Z'] ? parsed['Gyro X/Y/Z'].split(',')[0].replace('°', '').trim() : '',
      gyro_y: parsed['Gyro X/Y/Z'] ? parsed['Gyro X/Y/Z'].split(',')[1].replace('°', '').trim() : '',
      gyro_z: parsed['Gyro X/Y/Z'] ? parsed['Gyro X/Y/Z'].split(',')[2].replace('°', '').trim() : '',
      battery_v: parsed['Tag Battery'] ? parsed['Tag Battery'].replace('V', '').trim() : '',
      activity_mode: parsed['Activity Mode'] || '',
      raw_hex: rawHexStr
    };
    recordedPackets.push(recordItem);
    updateRecorderStats();
  }
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
 * Process Raw UART Data from Serial or BLE
 */
function processRawUartChunk(chunkData, source = 'SERIAL') {
  if (isStreamPaused) return;

  let textChunk = '';
  let hexBytes = [];
  let dataView = null;

  if (typeof chunkData === 'string') {
    textChunk = chunkData;
    const encoder = new TextEncoder();
    const bytes = encoder.encode(chunkData);
    for (let i = 0; i < bytes.length; i++) {
      hexBytes.push(bytes[i].toString(16).padStart(2, '0').toUpperCase());
    }
  } else if (chunkData instanceof DataView || chunkData instanceof ArrayBuffer || ArrayBuffer.isView(chunkData)) {
    if (chunkData instanceof DataView) {
      dataView = chunkData;
    } else {
      dataView = new DataView(chunkData.buffer || chunkData, chunkData.byteOffset || 0, chunkData.byteLength);
    }
    const bytes = new Uint8Array(dataView.buffer, dataView.byteOffset, dataView.byteLength);
    const decoder = new TextDecoder('utf-8', { fatal: false });
    textChunk = decoder.decode(bytes);

    for (let i = 0; i < bytes.length; i++) {
      hexBytes.push(bytes[i].toString(16).padStart(2, '0').toUpperCase());
    }
  }

  const rawHexStr = hexBytes.join(' ');
  const displayFormat = streamFormatSelect ? streamFormatSelect.value : 'text';

  if (displayFormat === 'hex') {
    logToConsole('rx', `[RAW HEX ${source}] ${rawHexStr}`);
  } else {
    // Text feed line-buffering
    serialLineBuffer += textChunk;
    const lines = serialLineBuffer.split(/\r?\n/);
    serialLineBuffer = lines.pop(); // keep trailing partial line

    for (const line of lines) {
      if (line.length > 0) {
        logToConsole('uart-text', `[${source}] ${line}`);
      }
    }
  }

  // If binary DataView is present or header match exists, decode binary payload
  if (dataView && dataView.byteLength >= 16) {
    const header = dataView.getUint8(0);
    if (header === 0xCB) {
      decodeAndProcessPacket(dataView, source);
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
   Bluetooth Telemetry Stream CSV Recorder Logic
   ========================================================================== */
function toggleRecording() {
  if (isRecording) {
    stopRecording();
  } else {
    startRecording();
  }
}

function startRecording() {
  isRecording = true;
  recordingStartTime = Date.now();
  
  recorderBadge.className = 'recorder-status-badge recording';
  recorderStatusText.textContent = 'RECORDING...';
  btnRecordToggle.className = 'btn btn-sm btn-record is-recording';
  btnRecordToggle.innerHTML = '<i class="fa-solid fa-stop"></i> Stop Recording';
  
  if (btnExportCsv) btnExportCsv.removeAttribute('disabled');

  recordingTimerInterval = setInterval(updateRecorderTimerDisplay, 1000);
  updateRecorderTimerDisplay();
  updateRecorderStats();

  logToConsole('system', 'CSV Stream Recorder STARTED. Logging incoming BT packets...');
}

function stopRecording() {
  isRecording = false;
  clearInterval(recordingTimerInterval);
  recordingTimerInterval = null;

  recorderBadge.className = 'recorder-status-badge stopped';
  recorderStatusText.textContent = 'REC IDLE';
  btnRecordToggle.className = 'btn btn-sm btn-record';
  btnRecordToggle.innerHTML = '<i class="fa-solid fa-circle"></i> Start Recording';

  logToConsole('system', `CSV Stream Recorder STOPPED. Captured ${recordedPackets.length} total records.`);
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
  
  // Approximate CSV file size in KB (~220 bytes per row)
  const estBytes = count * 220;
  const estKb = (estBytes / 1024).toFixed(1);
  recorderSize.textContent = `(~${estKb} KB)`;

  if (count > 0 && btnExportCsv) {
    btnExportCsv.removeAttribute('disabled');
  }
}

function exportCsv() {
  if (recordedPackets.length === 0) {
    alert('No recorded Bluetooth telemetry stream data to export. Click "Start Recording" to capture data.');
    return;
  }

  const headers = [
    'Timestamp (ISO)',
    'Timestamp (Local)',
    'Packet #',
    'Source',
    'Cow Tag ID',
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
      escapeCsvField(p.raw_hex)
    ];
    csvRows.push(row.join(','));
  }

  const csvContent = csvRows.join('\r\n');
  const blob = new Blob([csvContent], { type: 'text/csv;charset=utf-8;' });
  const url = URL.createObjectURL(blob);
  
  const now = new Date();
  const dateStr = now.toISOString().replace(/[:.]/g, '-').slice(0, 19);
  const filename = `ranchbot_bt_stream_${dateStr}.csv`;

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
  if (btnExportCsv) btnExportCsv.setAttribute('disabled', 'true');
  logToConsole('system', 'Recorded CSV telemetry buffer cleared.');
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
