"""
Ranchbot Cattle Ear Tag BLE & GPS / 6-DOF IMU Telemetry Receiver (Python)
Uses `bleak` to scan, connect, and decode Bluetooth LE broadcasts from Cattle Ear Tags.

Payload Structure (20 Bytes):
  [0]: Header (0xCB)
  [1..2]: Tag ID (uint16 LE)
  [3..6]: Latitude in microdegrees (int32 LE, / 1e7)
  [7..10]: Longitude in microdegrees (int32 LE, / 1e7)
  [11..12]: Accel X in mg (int16 LE)
  [13..14]: Accel Y in mg (int16 LE)
  [15..16]: Accel Z in mg (int16 LE)
  [17..18]: Battery mV (uint16 LE)
  [19]: Activity Mode (0: Resting, 1: Grazing, 2: Walking, 3: Alert)
"""

import asyncio
import struct
from bleak import BleakScanner, BleakClient

CATTLE_TAG_SERVICE_UUID = "0000181a-0000-1000-8000-00805f9b34fb"
CATTLE_TAG_CHAR_UUID    = "00002a6e-0000-1000-8000-00805f9b34fb"

def decode_cattle_tag_payload(data: bytes):
    if len(data) < 16:
        print(f"[RAW PAYLOAD] {data.hex()}")
        return

    try:
        header, tag_id, lat_micro, lng_micro, ax_mg, ay_mg, az_mg = struct.unpack("<B H i i h h h", data[:16])
        
        lat = lat_micro / 1e7
        lng = lng_micro / 1e7
        ax = ax_mg / 1000.0
        ay = ay_mg / 1000.0
        az = az_mg / 1000.0

        batt_mv = struct.unpack("<H", data[16:18])[0] if len(data) >= 18 else 3850
        act_code = data[19] if len(data) >= 20 else 1

        activities = ["Resting / Lying", "Grazing Pasture", "Walking / Moving", "High Alert / Running"]
        act_str = activities[act_code % 4]

        print("=" * 65)
        print(f"        RANCHBOT CATTLE EAR TAG TELEMETRY (COW-{tag_id})        ")
        print("=" * 65)
        print(f" GPS Coordinates  : Lat {lat:.6f}°, Lng {lng:.6f}°")
        print(f" 3-Axis Accel IMU : X: {ax:+.2f}g | Y: {ay:+.2f}g | Z: {az:+.2f}g")
        print(f" Battery Voltage  : {batt_mv / 1000.0:.2f} V")
        print(f" Activity Status  : {act_str}")
        print(f" Raw Packet Hex   : {data.hex(' ')}")
        print("=" * 65)
    except Exception as e:
        print(f"[DECODE ERROR] Unpack failed: {e}. Raw: {data.hex()}")

def notification_handler(sender, data: bytearray):
    print(f"\n[NOTIFY] Data received from characteristic {sender}:")
    decode_cattle_tag_payload(bytes(data))

async def main():
    print("===============================================================")
    print("  Ranchbot Cattle Tag BLE & GPS/IMU Receiver Scanner (Python)  ")
    print("===============================================================")
    print("Scanning for Bluetooth LE Ear Tags...")

    devices = await BleakScanner.discover(timeout=5.0)
    tag_device = None

    for d in devices:
        name = d.name or "Unknown"
        print(f"  > Found: [{d.address}] {name} (RSSI: {d.rssi} dBm)")
        if any(keyword.lower() in name.lower() for keyword in ["Xiao", "Xiao-cowtag", "cowtag", "Tag", "Ranchbot", "Cow"]):
            tag_device = d

    if not tag_device and devices:
        tag_device = devices[0]

    if not tag_device:
        print("No Bluetooth ear tag found. Exiting.")
        return

    print(f"\nConnecting to [{tag_device.address}] {tag_device.name}...")
    async with BleakClient(tag_device.address) as client:
        if client.is_connected:
            print("Connected! Listening for telemetry notifications...")
            try:
                await client.start_notify(CATTLE_TAG_CHAR_UUID, notification_handler)
                while True:
                    await asyncio.sleep(1.0)
            except Exception as e:
                print(f"Notification error: {e}")

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\nReceiver stopped.")
