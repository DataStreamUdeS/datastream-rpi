!pip install bleak

import asyncio
from bleak import BleakClient, BleakScanner
from datetime import datetime
import csv
import os

# UUIDs from your nRF firmware (converted to 128-bit style for clarity)
SERVICE_UUID = "0000180a-0000-1000-8000-00805f9b34fb"
CONTROL_CHAR_UUID = "0000182a-0000-1000-8000-00805f9b34fb"
STATUS_CHAR_UUID  = "0000184a-0000-1000-8000-00805f9b34fb"

LOG_FILE = "/home/pi/water_ble_log.txt"

def log(msg):
    timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    entry = f"[{timestamp}] {msg}"
    print(entry)
    with open(LOG_FILE, "a") as f:
        f.write(entry + "\n")

def MEASURE_COLLECTION_MOTOR_CONTROL():
    """
    Simulates motor winch control while the nRF52840 is underwater.
    You can replace this with GPIO logic later.
    """
    log("Starting MEASURE_COLLECTION_MOTOR_CONTROL()")
    # Example logic
    for i in range(3):
        log(f"Winch movement cycle {i+1}")
        asyncio.sleep(1)
    log("Finished MEASURE_COLLECTION_MOTOR_CONTROL()")

async def main():
    # Step 1: Scan for the peripheral
    log("Scanning for DataStream Capsule...")
    device = await BleakScanner.find_device_by_name("DataStream Capsule", timeout=10.0)

    if not device:
        log("Could not find DataStream Capsule.")
        return

    log(f"Found device: {device.address}")

    # Step 2: Connect
    async with BleakClient(device) as client:
        log("Connected to nRF52840")

        # Step 3: Enable notifications on the status characteristic
        async def status_notification_handler(sender, data):
            msg = data.decode("utf-8").strip()
            log(f"Status notification received: '{msg}'")
            if msg == "Ready":
                log("Capsule is ready, moving to next step...")
                asyncio.create_task(on_ready(client))

        await client.start_notify(STATUS_CHAR_UUID, status_notification_handler)

        # Step 4: Send "Init" to control characteristic
        log("Writing 'Init' to control characteristic...")
        await client.write_gatt_char(CONTROL_CHAR_UUID, b"Init")

        # Step 5: Wait until "Ready" is received and next step is triggered
        log("Waiting for Ready notification...")
        await asyncio.sleep(30)  # adjust depending on expected timing

        # Done - disconnect triggers next step
        log("Disconnecting now to simulate going underwater...")
        await client.disconnect()

    # Step 6: After disconnection, start motor control
    MEASURE_COLLECTION_MOTOR_CONTROL()


async def on_ready(client: BleakClient):
    """Callback when 'Ready' is received from status characteristic"""
    # Send "Moving" command
    log("Sending 'Moving' to control characteristic...")
    await client.write_gatt_char(CONTROL_CHAR_UUID, b"Moving")

    # Also write 'Moving' to status
    log("Writing 'Moving' to status characteristic...")
    await client.write_gatt_char(STATUS_CHAR_UUID, b"Moving")

    log("Moving state set on both control and status.")

# Run it
if __name__ == "__main__":
    try:
        asyncio.run(main())
    except Exception as e:
        log(f"Exception occurred: {e}")
