# Ranchbot & BLE Telemetry Receiver Application

An interactive web application and Python client for scanning, connecting, and decoding Bluetooth Low Energy (BLE) telemetry broadcasts from Ranchbot and satellite/cellular water monitoring hardware.

## Features

1. **Web Bluetooth Interface (`index.html`)**:
   - Live GATT Connection & Status Manager (Connected / Disconnected / Scanning).
   - Real-time **Water Tank Level Gauge** with animated liquid depth & capacity % calculation.
   - **System Power & Battery Gauge**: Displays solar panel output wattage, internal device temperature, and battery voltage.
   - **Real-Time Telemetry Chart**: Line graph plotting water depth history using Chart.js.
   - **Packet Decoder Console**: Decodes raw byte payloads (Sync Header, Water Level mm, Max Tank Height, Battery mV, Solar mW, Temp °C, Pump & Solar flags).
   - **Demo Simulator Mode**: Built-in simulator to test data streaming and decoding without physical BLE hardware present.

2. **Python Desktop Receiver (`ble_receiver.py`)**:
   - Asynchronous BLE scanner and GATT notification subscriber using `bleak`.
   - Automatic packet unpacking (`struct.unpack`) for 12-byte Ranchbot telemetry broadcasts.

## How to Run

### Web Application (Browser)
1. Open `index.html` in Google Chrome or Microsoft Edge (which support the Web Bluetooth API).
2. Click **"Scan & Connect Device"** to pair with a Bluetooth device nearby, OR click **"Toggle Demo Simulator"** to see live simulated telemetry broadcast parsing immediately!

### Python Script
```bash
pip install bleak
python ble_receiver.py
```
