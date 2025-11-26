#!/usr/bin/env python3
import asyncio
import json
import time

from bleak import BleakClient, BleakScanner
from datetime import datetime
import csv
import os
import struct
import sys
import pigpio

# UUIDs
SERVICE_UUID = "0000180a-0000-1000-8000-00805f9b34fb"
CONTROL_CHAR_UUID = "0000182a-0000-1000-8000-00805f9b34fb"
DATA_CHAR_UUID = "0000181a-0000-1000-8000-00805f9b34fb"
STATUS_CHAR_UUID  = "0000184a-0000-1000-8000-00805f9b34fb"

LOG_FILE = "/home/pi/water_ble_log.txt"
DEVICE_NAME = "DataStream Capsule"

PIN_CONTROL_BUTTON = 23
PIN_CALIBRATION_BUTTON = 24

class BLE_Com:
    def log(msg):
        ts = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        line = f"[{ts}] {msg}"
        print(line)
        try:
            with open(LOG_FILE, "a") as f:
                f.write(line + "\n")
        except Exception:
            pass

    async def MEASURE_COLLECTION_MOTOR_CONTROL(self):
        BLE_Com.log("Starting MEASURE_COLLECTION_MOTOR_CONTROL()")
        # use asyncio.sleep in async context
        for i in range(3):
            BLE_Com.log(f"Winch movement cycle {i+1}")
            await asyncio.sleep(1)
        BLE_Com.log("Finished MEASURE_COLLECTION_MOTOR_CONTROL()")

    async def SENSOR_CALIBRATION(self, client):
        BLE_Com.log("Starting SENSOR_CALIBRATION() (not implemented)")


    async def initial_handshake(self):
        BLE_Com.log("Scanning for DataStream Capsule...")
        device = await BleakScanner.find_device_by_name(DEVICE_NAME, timeout=60.0)
        while not device:
            BLE_Com.log("Could not find device.")
            device = await BleakScanner.find_device_by_name(DEVICE_NAME, timeout=60.0)


        BLE_Com.log(f"Found device: {device.address} — connecting...")
        async with BleakClient(device) as client:
            if not client.is_connected:
                BLE_Com.log("Failed to connect.")
                return False
            BLE_Com.log("Connected")

            ready_event = asyncio.Event()

            async def status_notification_handler(sender, data):
                try:
                    msg = data.decode("utf-8").strip()
                except Exception:
                    msg = repr(data)
                BLE_Com.log(f"Status notification: '{msg}'")
                if msg == "Ready":
                    ready_event.set()

            await client.start_notify(STATUS_CHAR_UUID, status_notification_handler)

            # write Init
            BLE_Com.log("Writing 'Init' to Control")
            await client.write_gatt_char(CONTROL_CHAR_UUID, b"Init")

            # wait for Ready
            try:
                await asyncio.wait_for(ready_event.wait(), timeout=10.0)
            except asyncio.TimeoutError:
                BLE_Com.log("Timeout waiting for 'Ready'")
                await client.stop_notify(STATUS_CHAR_UUID)
                return False

            BLE_Com.log("Ready received, sending 'Moving' (control & status)")
            if (noButton):
                await client.write_gatt_char(CONTROL_CHAR_UUID, b"start_retriving")

                # wait a bit then disconnect to simulate submersion
                await asyncio.sleep(5)
                BLE_Com.log("Disconnecting for submersion")
                await client.disconnect()

                # start winch control (RPi side)
                await BLE_Com.MEASURE_COLLECTION_MOTOR_CONTROL()
            else :
                async def watch_for_button():
                    pi = pigpio.pi()
                    pi.set_mode(PIN_CONTROL_BUTTON, pigpio.INPUT)
                    while True:
                        if (pi.read(PIN_CONTROL_BUTTON) != 0) :

                            await client.write_gatt_char(CONTROL_CHAR_UUID, b"start_retriving")

                            # wait a bit then disconnect to simulate submersion
                            await asyncio.sleep(5)
                            BLE_Com.log("Disconnecting to simulate submersion")
                            await client.disconnect()

                            # start winch control (RPi side)
                            await BLE_Com.MEASURE_COLLECTION_MOTOR_CONTROL()
                asyncio.run(watch_for_button())

                # TODO : finir le code pour le boutton calibration

        return True

    async def reconnect_and_collect(self, expected_count=12, timeout_after_last=5):
        """Reconnect, trigger Listen/TX and gather binary notifications"""
        await asyncio.sleep(2)  # small pause before scanning again
        BLE_Com.log("Scanning for DataStream Capsule (reconnection)...")
        device = await BleakScanner.find_device_by_name(DEVICE_NAME, timeout=15.0)
        if not device:
            BLE_Com.log("Capsule not found on resurfacing.")
            return False

        BLE_Com.log(f"Found resurfaced capsule: {device.address} — connecting...")
        async with BleakClient(device) as client:
            if not client.is_connected:
                BLE_Com.log("Failed to connect upon reconnection.")
                return False
            BLE_Com.log("Reconnected")

            # storage for received structs
            rows = []
            tx_done_event = asyncio.Event()
            last_receive_time = None

            async def status_handler(sender, data):
                try:
                    msg = data.decode("utf-8").strip()
                except Exception:
                    msg = repr(data)
                BLE_Com.log(f"Status notify: '{msg}'")
                if msg == "TX":
                    BLE_Com.log("Capsule requested TX; nothing to do (we will have written 'TX' to control).")
                elif msg == "TX_done":
                    BLE_Com.log("Capsule notified TX_done")
                    tx_done_event.set()

            async def data_handler(sender, data):
                nonlocal last_receive_time
                # Expect binary payload: 5 floats little-endian (20 bytes)
                last_receive_time = datetime.now()
                BLE_Com.log(f"Received {len(data)} bytes on DATA char")
                try:
                    # If payload is exactly 20 bytes => one sample
                    if len(data) == 20:
                        depth, temperature, ph, orp, o2 = struct.unpack("<5f", data)
                        rows.append({
                            "depth_m": depth,
                            "temperature_c": temperature,
                            "pH": ph,
                            "orp_mV": orp,
                            "o2_mgL": o2,
                            "recv_timestamp": last_receive_time.isoformat()
                        })
                        BLE_Com.log(f"   parsed sample #{len(rows)}: depth={depth}, temp={temperature}, pH={ph}")
                    else:
                        # If it's JSON/text or a chunk, try to decode text
                        try:
                            text = data.decode("utf-8", errors="ignore").strip()
                            BLE_Com.log(f"   Received text chunk: {text[:120]}")
                            # attempt JSON decode of an array
                            try:
                                arr = json.loads(text)
                                if isinstance(arr, list):
                                    for item in arr:
                                        rows.append(item)
                            except Exception:
                                # fallback: store raw string
                                rows.append({"raw": text, "recv_timestamp": last_receive_time.isoformat()})
                        except Exception as e:
                            BLE_Com.log(f"Could not parse chunk: {e}")
                except Exception as e:
                    BLE_Com.log(f"Error unpacking data: {e}")

            # start notifications
            await client.start_notify(STATUS_CHAR_UUID, status_handler)
            await client.start_notify(DATA_CHAR_UUID, data_handler)

            # Step: write "Listen" to control to tell peripheral to prepare
            BLE_Com.log("Writing 'Listen' to Control characteristic")
            await client.write_gatt_char(CONTROL_CHAR_UUID, b"ready_to_tx")

            # Wait briefly for peripheral to notify "TX" (or "Listen" -> then we write "TX")
            # If the peripheral notifies "Listen" or directly "TX", status_handler will log it.
            # To be safe: write "TX" once after small delay to trigger transmission if device expects it
            await asyncio.sleep(0.5)
            BLE_Com.log("Writing 'TX' to Control characteristic (trigger transmission)")
            # await client.write_gatt_char(CONTROL_CHAR_UUID, b"TX")

            # Now wait for data:
            # strategy: wait until tx_done_event is set OR until we received expected_count samples
            # OR until a short timeout since last_receive_time
            start = datetime.now()
            while True:
                # if tx_done_event triggered -> break
                if tx_done_event.is_set():
                    BLE_Com.log("Breaking: tx_done_event set")
                    break
                # if we got enough samples
                if expected_count and len(rows) >= expected_count:
                    BLE_Com.log(f"Breaking: expected_count ({expected_count}) received")
                    break
                # if no data ever received for long -> give up
                if (datetime.now() - start).total_seconds() > 60:
                    BLE_Com.log("Timeout waiting for data (60s) — aborting transfer")
                    break
                # if we received at least one and nothing for timeout_after_last seconds -> assume transfer finished
                if last_receive_time:
                    if (datetime.now() - last_receive_time).total_seconds() > timeout_after_last:
                        BLE_Com.log(f"No data for {timeout_after_last}s after last packet — assuming transfer done")
                        break
                await asyncio.sleep(0.2)
            client.write_gatt_char(CONTROL_CHAR_UUID, b"done_tx")
            # stop notifications
            try:
                await client.stop_notify(DATA_CHAR_UUID)
                await client.stop_notify(STATUS_CHAR_UUID)
            except Exception:
                pass

            # Save CSV if rows exist
            if rows:
                BLE_Com.save_rows_to_csv(rows)
                # As requested: RPi notifies peripheral with "TX_done" (write to STATUS)
                try:
                    await client.write_gatt_char(STATUS_CHAR_UUID, b"TX_done")
                    BLE_Com.log("Wrote 'TX_done' to STATUS char (confirming to peripheral).")
                except Exception as e:
                    BLE_Com.log(f"Could not write 'TX_done' to peripheral: {e}")
            else:
                BLE_Com.log("No rows received; nothing to save")

            # disconnect
            await client.disconnect()
        return True

    def save_rows_to_csv(rows):
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"/home/pi/Desktop/datastream-pi//water_data_{timestamp}.csv"
        BLE_Com.log(f"Saving {len(rows)} rows to {filename}")
        # if first row is dict with keys
        keys = None
        if isinstance(rows[0], dict):
            keys = list(rows[0].keys())
        try:
            with open(filename, "w", newline="") as f:
                writer = csv.DictWriter(f, fieldnames=keys if keys else ["raw"])
                writer.writeheader()
                writer.writerows(rows)
            BLE_Com.log("CSV saved")
        except Exception as e:
            BLE_Com.log(f"Failed to save CSV: {e}")

async def main():

    ok = await BLE_Com.initial_handshake()
    if not ok:
        print("Initial handshake failed — exiting")
        return
    print("Initial handshake succeeded")
    # wait appropriate time for collection (your system)
    # attempt reconnect & transfer
    await BLE_Com.reconnect_and_collect(expected_count=12, timeout_after_last=3)

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("Interrupted by user")
    except Exception as e:
        print(f"Exception in main: {e}")
        sys.exit(1)
