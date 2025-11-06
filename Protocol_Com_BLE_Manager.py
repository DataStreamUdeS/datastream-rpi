import asyncio
from bleak import BleakClient, BleakScanner
from datetime import datetime
import csv
import os
import json

# UUIDs
SERVICE_UUID = "0000180a-0000-1000-8000-00805f9b34fb"
CONTROL_CHAR_UUID = "0000182a-0000-1000-8000-00805f9b34fb"
DATA_CHAR_UUID = "0000181a-0000-1000-8000-00805f9b34fb"
STATUS_CHAR_UUID  = "0000184a-0000-1000-8000-00805f9b34fb"

LOG_FILE = "/home/pi/water_ble_log.txt"

def log(msg):
    timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    entry = f"[{timestamp}] {msg}"
    print(entry)
    with open(LOG_FILE, "a") as f:
        f.write(entry + "\n")

def MEASURE_COLLECTION_MOTOR_CONTROL():
    log("⚙Starting MEASURE_COLLECTION_MOTOR_CONTROL()")
    for i in range(3):
        log(f"Winch movement cycle {i+1}")
        asyncio.sleep(1)
    log("Finished MEASURE_COLLECTION_MOTOR_CONTROL()")

async def connect_and_handshake():
    """Phase 1: Initial handshake and descent"""
    log("Scanning for DataStream Capsule...")
    device = await BleakScanner.find_device_by_name("DataStream Capsule", timeout=10.0)
    if not device:
        log("Could not find DataStream Capsule.")
        return

    log(f"Found device: {device.address}")
    async with BleakClient(device) as client:
        log("🔗 Connected to nRF52840")

        # Handle notifications from Status
        async def status_notification_handler(sender, data):
            msg = data.decode("utf-8").strip()
            log(f"Status notification: '{msg}'")
            if msg == "Ready":
                log("Capsule Ready. Sending 'Moving'")
                await client.write_gatt_char(CONTROL_CHAR_UUID, b"Moving")
                await client.write_gatt_char(STATUS_CHAR_UUID, b"Moving")

        await client.start_notify(STATUS_CHAR_UUID, status_notification_handler)
        log("Writing 'Init' to Control")
        await client.write_gatt_char(CONTROL_CHAR_UUID, b"Init")
        log("Waiting for Ready...")
        await asyncio.sleep(10)

        log("Disconnecting (simulating submersion)")
        await client.disconnect()

    MEASURE_COLLECTION_MOTOR_CONTROL()

async def reconnect_and_transfer():
    """Phase 2: Reconnection and data transfer"""
    await asyncio.sleep(5)
    log("Scanning for DataStream Capsule (reconnection)...")
    device = await BleakScanner.find_device_by_name("DataStream Capsule", timeout=10.0)
    if not device:
        log("Capsule not found on resurfacing.")
        return

    log(f"Found resurfaced capsule: {device.address}")
    async with BleakClient(device) as client:
        log("Reconnected")

        # Notification handler
        received_payload = None
        async def status_handler(sender, data):
            nonlocal received_payload
            msg = data.decode("utf-8").strip()
            log(f"Status notify: {msg}")
            if msg == "Listen":
                log("Capsule is listening — initiating TX phase")
                await client.write_gatt_char(CONTROL_CHAR_UUID, b"TX")
            elif msg == "TX_done":
                log("Transmission complete")

        async def data_handler(sender, data):
            nonlocal received_payload
            try:
                text = data.decode("utf-8", errors="ignore").strip()
                log(f"Data chunk received ({len(data)} bytes)")
                received_payload = text
            except Exception as e:
                log(f"Data decode error: {e}")

        await client.start_notify(STATUS_CHAR_UUID, status_handler)
        await client.start_notify(DATA_CHAR_UUID, data_handler)

        # Step: write "Listen" to control to begin phase
        log("Writing 'Listen' to Control characteristic")
        await client.write_gatt_char(CONTROL_CHAR_UUID, b"Listen")

        await asyncio.sleep(10)

        # Save received data
        if received_payload:
            save_payload_to_csv(received_payload)
            log("Payload saved to CSV")
            await client.write_gatt_char(STATUS_CHAR_UUID, b"TX_done")

        await client.disconnect()

def save_payload_to_csv(payload_str):
    """Store received payload into CSV file"""
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = f"/home/pi/water_data_{timestamp}.csv"
    log(f"Saving payload to {filename}")

    # Attempt JSON decode or fallback to CSV-like writing
    try:
        payload = json.loads(payload_str)
        keys = payload[0].keys()
        with open(filename, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=keys)
            writer.writeheader()
            writer.writerows(payload)
    except Exception:
        with open(filename, "w") as f:
            f.write(payload_str)
    log("File saved")

async def main():
    await connect_and_handshake()
    await reconnect_and_transfer()

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except Exception as e:
        log(f"Exception: {e}")
