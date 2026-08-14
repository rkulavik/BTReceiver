# Ranchbot & BLE / Serial Telemetry Receiver Application

An interactive web application and Python client for scanning, connecting, and decoding direct **Web Serial (USB UART)** streams and **Bluetooth Low Energy (BLE)** telemetry broadcasts from Ranchbot, Seeed Xiao, and satellite/cellular water & livestock monitoring hardware.

## Features

1. **Web Serial (USB UART) & Web Bluetooth Interface (`index.html`)**:
   - **Direct USB Serial Connection (Web Serial API)**: Stream raw UART feed from connected serial ports at 115200, 57600, 38400, 19200, or 9600 baud.
   - **Bluetooth LE GATT Manager**: Connect and receive telemetry notifications over BLE (including Nordic UART Service - NUS).
   - **Live Raw UART Feed Console**: Real-time monospaced ASCII text line stream and raw byte hex inspector with pause/resume and auto-scroll controls.
   - **Cattle Tag GPS & IMU Dashboard**: Real-time Leaflet satellite map tracking cattle movement, 6-DOF IMU accelerometer/gyroscope gauges, and dynamic Chart.js motion history graph.
   - **CSV Stream Recorder**: Record incoming serial/BLE packets and export complete telemetry records to CSV.

2. **Python Desktop Receiver (`ble_receiver.py`)**:
   - Asynchronous BLE scanner and GATT notification subscriber using `bleak`.
   - Automatic packet unpacking (`struct.unpack`) for 20-byte Ranchbot telemetry broadcasts.

## How to Run

### Web Application (Browser)
1. Open `index.html` in Google Chrome, Microsoft Edge, or Opera (which support the Web Serial and Web Bluetooth APIs).
2. For direct USB UART connection: Plug in your serial device (FTDI, CP2102, CH340, Seeed Xiao USB), select your Baud Rate, and click **"Connect Serial / UART"**.
3. For BLE connection: Click **"Connect Ear Tag (BLE)"** to pair with your Bluetooth LE device nearby.
4. View the live ASCII text line feed or hex byte stream in the **Live Raw UART Feed & Telemetry** console!

### Python Script
```bash
pip install bleak
python ble_receiver.py
```

