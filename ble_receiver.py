"""
Ranchbot Compressed UART & BLE Telemetry Receiver Client (Python)
Decodes:
  1. 29-Byte Binary Compressed IMU Frames (0xAA Header Magic Byte)
  2. 22-Field Periodic Telemetry Snapshot CSV Rows (~5.6s Cadence)
"""

import asyncio
import struct
import sys

# Standard NUS (Nordic UART Service) and Cattle Tag GATT UUIDs
NUS_SERVICE_UUID = "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
NUS_TX_CHAR_UUID = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"
CATTLE_TAG_SERVICE_UUID = "0000181a-0000-1000-8000-00805f9b34fb"
CATTLE_TAG_CHAR_UUID    = "00002a6e-0000-1000-8000-00805f9b34fb"

# Sensor Scaling Constants (±2.5g and ±125°/s defaults)
ACCEL_LSB_PER_G = 13107.2
GYRO_LSB_PER_DPS = 262.14

def decode_compressed_binary_frame(data: bytes):
    """
    Decodes 29-byte high-rate binary IMU frame (Header 0xAA).
    Structure: <B B I h h h h h h h h h B H h
    """
    if len(data) < 29:
        print(f"[RAW PAYLOAD] {data.hex()}")
        return

    try:
        (
            header, sample_idx, timestamp_ms,
            raw_ax, raw_ay, raw_az,
            raw_gx, raw_gy, raw_gz,
            pitch_cd, roll_cd, yaw_cd,
            step_detected, total_steps,
            temp_cd
        ) = struct.unpack("<B B I h h h h h h h h h B H h", data[:29])

        ax_g = raw_ax / ACCEL_LSB_PER_G
        ay_g = raw_ay / ACCEL_LSB_PER_G
        az_g = raw_az / ACCEL_LSB_PER_G

        gx_dps = raw_gx / GYRO_LSB_PER_DPS
        gy_dps = raw_gy / GYRO_LSB_PER_DPS
        gz_dps = raw_gz / GYRO_LSB_PER_DPS

        pitch_deg = pitch_cd / 100.0
        roll_deg = roll_cd / 100.0
        yaw_deg = (yaw_cd / 100.0) % 360.0

        temp_c = temp_cd / 100.0
        temp_f = (temp_c * 9.0 / 5.0) + 32.0

        step_str = "● STEP DETECTED" if step_detected == 1 else "None"

        print("=" * 70)
        print(f"   COMPRESSED 29-BYTE BINARY FRAME [0x{header:02X}]  |  FIFO #{sample_idx}/31   ")
        print("=" * 70)
        print(f" Uptime Timestamp : {timestamp_ms} ms ({timestamp_ms / 1000.0:.2f}s)")
        print(f" 3-Axis Accel     : X: {ax_g:+.2f}g | Y: {ay_g:+.2f}g | Z: {az_g:+.2f}g ({raw_ax}, {raw_ay}, {raw_az} LSB)")
        print(f" 3-Axis Gyroscope : X: {gx_dps:+.1f}°/s | Y: {gy_dps:+.1f}°/s | Z: {gz_dps:+.1f}°/s ({raw_gx}, {raw_gy}, {raw_gz} LSB)")
        print(f" Euler Attitude   : Pitch: {pitch_deg:+.2f}° | Roll: {roll_deg:+.2f}° | Yaw: {yaw_deg:.2f}°")
        print(f" Pedometer Steps  : {total_steps} cumulative ({step_str})")
        print(f" IMU Temperature  : {temp_c:.2f} °C ({temp_f:.1f} °F)")
        print(f" Raw Hex Bytes    : {data[:29].hex(' ').upper()}")
        print("=" * 70)

        # Alarm diagnostics
        if temp_c > 55.0:
            print(f" [ALARM] OVERHEATING: IMU silicon temperature is {temp_c:.1f}°C (> 55°C limit)!")
        elif temp_c < 0.0:
            print(f" [WARNING] FREEZING: IMU silicon temperature is {temp_c:.1f}°C (< 0°C)!")

    except Exception as e:
        print(f"[DECODE ERROR] Unpack failed: {e}. Raw: {data.hex()}")

def decode_periodic_telemetry_csv(line: str):
    """
    Decodes 22-field periodic telemetry snapshot CSV line (~5.6s cadence).
    Format: up_s,gps_fix,lat_e7,lon_e7,sat_tot,sat_used,ax_mms2,ay_mms2,az_mms2,gx_mdps,gy_mdps,gz_mdps,chars,sent,cksum_err,frm,brk,ovr,ring_drops,batt_pct,batt_mv,temp_cc
    """
    clean = line.strip()
    tokens = [t.strip() for t in clean.split(",") if len(t.strip()) > 0]
    if len(tokens) < 20:
        print(f"[UART TEXT] {clean}")
        return

    try:
        up_s = int(tokens[0])
        gps_fix = int(tokens[1])
        lat_e7 = int(tokens[2])
        lon_e7 = int(tokens[3])
        sat_tot = int(tokens[4])
        sat_used = int(tokens[5])
        ax_mms2 = int(tokens[6])
        ay_mms2 = int(tokens[7])
        az_mms2 = int(tokens[8])
        gx_mdps = int(tokens[9])
        gy_mdps = int(tokens[10])
        gz_mdps = int(tokens[11])
        chars = int(tokens[12])
        sent = int(tokens[13])
        cksum_err = int(tokens[14])
        frm = int(tokens[15])
        brk = int(tokens[16])
        ovr = int(tokens[17])
        ring_drops = int(tokens[18])
        batt_pct = int(tokens[19]) if len(tokens) >= 20 else 0
        batt_mv = int(tokens[20]) if len(tokens) >= 21 else 3850
        temp_cc = int(tokens[21]) if len(tokens) >= 22 else 2500

        lat = lat_e7 / 1e7
        lon = lon_e7 / 1e7
        batt_v = batt_mv / 1000.0
        temp_c = temp_cc / 100.0
        temp_f = (temp_c * 9.0 / 5.0) + 32.0

        fix_str = f"3D Fix ({sat_used}/{sat_tot} Sats)" if gps_fix == 1 else f"Searching Fix ({sat_tot} Visible)"

        print("*" * 70)
        print(f"      PERIODIC TELEMETRY SNAPSHOT (~5.6s)  |  Uptime: {up_s}s      ")
        print("*" * 70)
        print(f" GNSS Fix / Sats  : {fix_str}")
        print(f" Coordinates      : Lat {lat:.6f}°, Lon {lon:.6f}°")
        print(f" Peak Accel (5.6s): X: {ax_mms2} | Y: {ay_mms2} | Z: {az_mms2} mm/s²")
        print(f" Peak Gyro (5.6s) : X: {gx_mdps} | Y: {gy_mdps} | Z: {gz_mdps} mdps")
        print(f" GPS UART Traffic : {chars:,} Bytes ({sent} Valid NMEA Sentences, {cksum_err} Checksum Errs)")
        print(f" UART Driver Errs : Framing: {frm} | Break: {brk} | Overrun: {ovr} | Ring Drops: {ring_drops}")
        print(f" Power & Battery  : {batt_pct}% ({batt_v:.2f} V / {batt_mv} mV)")
        print(f" Board Temp       : {temp_c:.2f} °C ({temp_f:.1f} °F)")
        print("*" * 70)

        # Hardware and diagnostic alarms
        if frm > 0 or ovr > 0 or ring_drops > 0:
            print(f" [ALARM] UART DRIVER ERRORS: {frm} Framing, {ovr} Overrun, {ring_drops} Ring Buffer Drops!")
        if gps_fix == 0:
            print(f" [ALARM] GNSS FIX LOST: Searching for satellites...")
        if batt_pct <= 20 or batt_mv < 3500:
            print(f" [ALARM] LOW BATTERY: {batt_pct}% ({batt_v:.2f}V)")
        if temp_c > 55.0:
            print(f" [ALARM] OVERHEATING: Board temperature {temp_c:.1f}°C exceeds threshold!")

    except Exception as e:
        print(f"[CSV DECODE ERROR] {e}. Line: {clean}")

def process_inbound_data(data: bytes):
    # Detect binary frame by 0xAA header or length 29
    if len(data) >= 29 and data[0] == 0xAA:
        decode_compressed_binary_frame(data)
    else:
        try:
            text = data.decode("utf-8", errors="ignore")
            lines = text.splitlines()
            for line in lines:
                if line.strip():
                    decode_periodic_telemetry_csv(line)
        except Exception:
            decode_compressed_binary_frame(data)

def notification_handler(sender, data: bytearray):
    process_inbound_data(bytes(data))

async def main():
    try:
        from bleak import BleakScanner, BleakClient
    except ImportError:
        print("Error: `bleak` library is required for BLE operation. Install with `pip install bleak`.")
        return

    print("===============================================================")
    print(" Ranchbot CompressedUART & BLE Telemetry Receiver Scanner (Python) ")
    print("===============================================================")
    print("Scanning for Bluetooth LE Devices / Nordic UART Services...")

    devices = await BleakScanner.discover(timeout=5.0)
    target_device = None

    for d in devices:
        name = d.name or "Unknown"
        print(f"  > Found: [{d.address}] {name} (RSSI: {d.rssi} dBm)")
        if any(keyword.lower() in name.lower() for keyword in ["Xiao", "Xiao-cowtag", "cowtag", "Tag", "Ranchbot", "Cow", "UART"]):
            target_device = d

    if not target_device and devices:
        target_device = devices[0]

    if not target_device:
        print("No compatible Bluetooth device found. Exiting.")
        return

    print(f"\nConnecting to [{target_device.address}] {target_device.name}...")
    async with BleakClient(target_device.address) as client:
        if client.is_connected:
            print("Connected! Subscribing to telemetry notifications...")
            subscribed = False

            # Try NUS TX characteristic first, then Cattle Tag UUID
            for char_uuid in [NUS_TX_CHAR_UUID, CATTLE_TAG_CHAR_UUID]:
                try:
                    await client.start_notify(char_uuid, notification_handler)
                    print(f"✓ Subscribed to characteristic: {char_uuid}")
                    subscribed = True
                    break
                except Exception:
                    continue

            if not subscribed:
                # Fallback: scan all characteristics with notify property
                for service in client.services:
                    for char in service.characteristics:
                        if "notify" in char.properties:
                            try:
                                await client.start_notify(char.uuid, notification_handler)
                                print(f"✓ Subscribed to fallback characteristic: {char.uuid}")
                                subscribed = True
                            except Exception as e:
                                pass

            if subscribed:
                while True:
                    await asyncio.sleep(1.0)
            else:
                print("Could not find a notification characteristic on device.")

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\nReceiver client stopped.")
