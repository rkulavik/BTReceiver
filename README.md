# Ranchbot Cattle Tag BLE & Compressed UART Telemetry System (`CompressedUART` Branch)

An advanced web application and Python receiver client engineered for receiving, decoding, graphing, and logging **Compressed 29-Byte Binary IMU Frames** and **22-Field Periodic Telemetry Snapshots** over Web Serial (USB UART), Bluetooth Low Energy (NUS / BLE GATT), and CSV replay.

---

## Data Protocols

### 1. High-Speed 29-Byte Binary IMU Frame (`0xAA`)
Transmitted for each FIFO sample batch (e.g. 52 Hz IMU window, indices 0..31):

| Offset | Field | Data Type | Units / Scale | Description |
| :--- | :--- | :--- | :--- | :--- |
| `0` | `header` | `uint8_t` | `0xAA` | Binary frame magic marker byte |
| `1` | `idx` | `uint8_t` | `0 .. 31` | Sample index within the FIFO batch |
| `2..5` | `timestamp_ms` | `uint32_t` LE | Milliseconds | System uptime timestamp in ms |
| `6..7` | `ax` | `int16_t` LE | LSB (`±2.5g`) | Raw 16-bit Accelerometer X reading |
| `8..9` | `ay` | `int16_t` LE | LSB (`±2.5g`) | Raw 16-bit Accelerometer Y reading |
| `10..11` | `az` | `int16_t` LE | LSB (`±2.5g`) | Raw 16-bit Accelerometer Z reading |
| `12..13` | `gx` | `int16_t` LE | LSB (`±125°/s`) | Raw 16-bit Gyroscope X reading |
| `14..15` | `gy` | `int16_t` LE | LSB (`±125°/s`) | Raw 16-bit Gyroscope Y reading |
| `16..17` | `gz` | `int16_t` LE | LSB (`±125°/s`) | Raw 16-bit Gyroscope Z reading |
| `18..19` | `pitch_cd` | `int16_t` LE | Centi-degrees (`deg × 100`) | Calculated Pitch Euler angle |
| `20..21` | `roll_cd` | `int16_t` LE | Centi-degrees (`deg × 100`) | Calculated Roll Euler angle |
| `22..23` | `yaw_cd` | `int16_t` LE | Centi-degrees (`deg × 100`) | Calculated Yaw Euler angle |
| `24` | `step_detected` | `uint8_t` | `0` or `1` | Pedometer step detector flag for this window |
| `25..26` | `total_steps` | `uint16_t` LE | Cumulative Steps | Cumulative pedometer step counter |
| `27..28` | `temp_cd` | `int16_t` LE | Centi-degrees (`°C × 100`) | IMU silicon chip temperature |

**Python Struct Unpack String**: `<B B I h h h h h h h h h B H h` (29 bytes)

---

### 2. Periodic Telemetry Snapshot (22-Field CSV Row)
Transmitted automatically at the end of every IMU FIFO window (~5.6 seconds at 52 Hz) and saved to SD card as `TELEM-YYYYMMDD.csv`:

```text
<up_s>,<gps_fix>,<lat_e7>,<lon_e7>,<sat_tot>,<sat_used>,<ax_mms2>,<ay_mms2>,<az_mms2>,<gx_mdps>,<gy_mdps>,<gz_mdps>,<chars>,<sent>,<cksum_err>,<frm>,<brk>,<ovr>,<ring_drops>,<batt_pct>,<batt_mv>,<temp_cc>\n
```

| Index | Field | Type | Scale / Units | Description |
| :--- | :--- | :--- | :--- | :--- |
| `0` | `up_s` | `uint32_t` | Seconds | System uptime since boot |
| `1` | `gps_fix` | `uint8_t` | `0` or `1` | GNSS lock status (`1` = valid fix) |
| `2` | `lat_e7` | `int32_t` | `1e-7 deg` | Latitude (`lat = lat_e7 / 1e7`) |
| `3` | `lon_e7` | `int32_t` | `1e-7 deg` | Longitude (`lon = lon_e7 / 1e7`) |
| `4` | `sat_tot` | `uint16_t` | Count | Total GNSS satellites in view |
| `5` | `sat_used` | `uint16_t` | Count | Satellites actively used in navigation fix |
| `6..8` | `ax/ay/az_mms2` | `int32_t` (3x) | `mm/s²` | Peak window acceleration (`g = val / 9806.65`) |
| `9..11` | `gx/gy/gz_mdps` | `int32_t` (3x) | `mdps` | Peak window angular rate (`°/s = val / 1000`) |
| `12` | `chars` | `uint32_t` | Bytes | Total GPS UART characters received |
| `13` | `sent` | `uint32_t` | Sentences | Valid NMEA sentences processed |
| `14` | `cksum_err` | `uint32_t` | Errors | Corrupted NMEA checksum dropped sentences |
| `15` | `frm` | `uint32_t` | Errors | UART framing errors (baud rate / signal mismatch) |
| `16` | `brk` | `uint32_t` | Counts | UART line break conditions detected |
| `17` | `ovr` | `uint32_t` | Errors | Hardware FIFO overrun drops |
| `18` | `ring_drops` | `uint32_t` | Errors | Software ring buffer dropouts |
| `19` | `batt_pct` | `uint8_t` | `0 .. 100%` | Fuel-gauge battery percentage |
| `20` | `batt_mv` | `uint16_t` | Millivolts | Battery voltage (`V = batt_mv / 1000.0`) |
| `21` | `temp_cc` | `int16_t` | `°C × 100` | Board temperature (`°C = temp_cc / 100.0`) |

---

## Diagnostic Alarms & Health Monitoring

The diagnostic engine continuously validates incoming telemetry against mission-critical thresholds:
- **UART Driver Health**: Alarms on non-zero framing (`frm`), overrun (`ovr`), break (`brk`), ring drops (`ring_drops`), or NMEA checksum corruptions (`cksum_err`).
- **GNSS Satellite Health**: Alarms if GNSS fix is lost (`gps_fix === 0`) or active satellite lock is low (`sat_used < 4`).
- **Thermal Safety**: Critical alarm if chip temperature > 55.0°C; sub-zero alert if temperature < 0.0°C.
- **Power Health**: Warning at ≤ 20% battery; Critical error at ≤ 10% battery or < 3.40V.
- **Impact Shock & Thrashing**: Alert if peak acceleration exceeds 2.5g (~24,525 mm/s²) or peak gyro exceeds 100°/s (100,000 mdps).

---

## How to Run

### Web Interface (`index.html`)
1. Open `index.html` in Chrome or Edge.
2. Select baud rate (115200) and click **"Connect Serial / UART"** for direct USB UART streaming, or **"Connect Ear Tag (BLE)"** for Bluetooth LE.
3. Use the **Test Stream** generator button to simulate 20 Hz live compressed frames and periodic snapshots.
4. Drag and drop any `TELEM-YYYYMMDD.csv` file onto the replay player to scrub through historical pasture logs with interactive 3D attitude, GPS tracking, and error diagnostics.

### Python Client (`ble_receiver.py`)
```bash
pip install bleak
python ble_receiver.py
```
