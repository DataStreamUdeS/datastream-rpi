"""
BLE <-> SIM7600 integration module

- Listens to an nRF52 over BLE for a single JSON payload (or multiple) containing
  sensor readings from 4 sensors (10 measurements each expected).
- Adds GPS coordinates from the SIM7600 module and forwards the combined JSON
  payload via SMTP over LTE using the SIM7600.send_email(...) method.

Assumptions:
- The nRF52 advertises/connects and provides sensor data on a known
  characteristic UUID. The payload format is JSON (e.g.:
  {"sensor_1": [..], "sensor_2": [..], ...}). If your device sends binary,
  change the parse_payload() function.
- The SIM7600 class is available (same folder) and provides get_gps_coordinates()
  and send_email(...).
- BlueZ is installed on the Raspberry Pi and the Python package `bleak` is
  available (`pip install bleak`).

Usage:
- Adjust BLE_DEVICE_NAME or BLE_ADDRESS and CHARACTERISTIC_UUID below.
- Run this script on the Raspberry Pi (with root or appropriate bluetooth
  permissions) to wait for a BLE packet, attach GPS, then email it.

"""

import asyncio
import json
import logging
import time
from typing import Optional

from bleak import BleakClient, BleakScanner

# Import your SIM7600 class from the module where you defined it. If it's in
# the same file, you can import directly. Adjust the import as needed.
from SIM7600_module import SIM7600

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("ble_sim7600")

# --- Configuration (change to match your nRF52) ---
BLE_DEVICE_NAME = "nrf52_sensor_node"  # or None if using address
BLE_ADDRESS = None  # e.g. "AA:BB:CC:11:22:33" (set if known)
# The characteristic UUID that sends the measurements (notify/indicate)
CHARACTERISTIC_UUID = "0000ffe1-0000-1000-8000-00805f9b34fb"

# How long to wait (seconds) for BLE data before giving up
BLE_TIMEOUT = 30

# --- BLE manager ---
class BLEManager:
    def __init__(self, address: Optional[str] = None, name: Optional[str] = None,
                 char_uuid: str = CHARACTERISTIC_UUID, timeout: int = BLE_TIMEOUT):
        self.address = address
        self.name = name
        self.char_uuid = char_uuid
        self.timeout = timeout
        self._received_payload = None
        self._loop = asyncio.get_event_loop()

    async def _find_device_address(self) -> Optional[str]:
        """Scan for the target device and return its address (or the provided one)."""
        if self.address:
            logger.info(f"Using provided BLE address: {self.address}")
            return self.address

        logger.info(f"Scanning for BLE devices (looking for name '{self.name}')...")
        devices = await BleakScanner.discover(timeout=5.0)
        for d in devices:
            if d.name and self.name and self.name in d.name:
                logger.info(f"Found device: {d.name} @ {d.address}")
                return d.address

        logger.warning("Device not found by name during scan.")
        return None

    def _notification_handler(self, sender: int, data: bytearray):
        """Called when a notification is received from the BLE device."""
        try:
            text = data.decode(errors="ignore")
            logger.info(f"Notification from {sender}: {text}")
            parsed = self.parse_payload(text)
            if parsed is not None:
                self._received_payload = parsed
        except Exception as e:
            logger.exception("Failed to parse BLE notification: %s", e)

    @staticmethod
    def parse_payload(raw_text: str):
        """Parse incoming BLE payload.

        Default: expects JSON string. Example expected data format:
        {
            "sensor_1": [v1, v2, ... v10],
            "sensor_2": [...],
            "sensor_3": [...],
            "sensor_4": [...]
        }

        Return dict on success, None on failure.
        """
        try:
            obj = json.loads(raw_text)
            # Basic validation: expect 4 keys with lists of ~10 elements
            if not isinstance(obj, dict):
                logger.warning("Payload is not a JSON object")
                return None
            # Optional: add stricter validation here
            return obj
        except json.JSONDecodeError:
            logger.warning("Payload is not valid JSON. Raw: %r", raw_text)
            return None

    async def _connect_and_listen_once(self) -> Optional[dict]:
        """Internal coroutine: discover device, connect, subscribe, wait for one payload."""
        addr = await self._find_device_address()
        if not addr:
            raise RuntimeError("BLE device not found")

        logger.info(f"Connecting to {addr}...")
        async with BleakClient(addr) as client:
            if not client.is_connected:
                raise RuntimeError("Failed to connect to BLE device")

            logger.info("Connected. Subscribing to notifications...")
            await client.start_notify(self.char_uuid, self._notification_handler)

            # Wait until payload received or timeout
            t_start = time.time()
            while (time.time() - t_start) < self.timeout:
                if self._received_payload is not None:
                    break
                await asyncio.sleep(0.1)

            await client.stop_notify(self.char_uuid)
            if self._received_payload is None:
                logger.warning("No payload received before timeout")
            else:
                logger.info("Received payload from BLE device")

            return self._received_payload

    def get_one_payload(self, blocking_timeout: Optional[int] = None) -> Optional[dict]:
        """Public sync method: collect one payload (blocking) and return the parsed dict.

        :param blocking_timeout: override default timeout for this call
        """
        if blocking_timeout is not None:
            self.timeout = blocking_timeout

        # Reset state
        self._received_payload = None

        # Run the asyncio coroutine to completion
        return self._loop.run_until_complete(self._connect_and_listen_once())


# --- High-level integration function ---
class BLESMTPBridge:
    def __init__(self, sim7600: SIM7600, ble_manager: BLEManager,
                 smtp_server: str, smtp_port: int, email_user: str, email_pass: str,
                 email_to: str):
        self.sim = sim7600
        self.ble = ble_manager
        self.smtp_server = smtp_server
        self.smtp_port = smtp_port
        self.email_user = email_user
        self.email_pass = email_pass
        self.email_to = email_to

    def collect_and_send(self, subject: str = "Sensor Data Report") -> bool:
        """Blocking call: get BLE data, get GPS, combine, and email.

        Returns True on success, False otherwise.
        """
        try:
            payload = self.ble.get_one_payload()
            if payload is None:
                logger.error("Failed to get payload from BLE device")
                return False

            logger.info("Obtaining GPS coordinates from SIM7600...")
            coords = self.sim.get_gps_coordinates()
            if coords is None:
                logger.warning("GPS coordinates unavailable; sending payload without GPS")
            else:
                lat, lon = coords
                payload["gps"] = {"lat": lat, "lon": lon}

            # Optionally add a timestamp
            payload["timestamp_utc"] = time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())

            logger.info("Sending email with payload")
            self.sim.send_email(
                smtp_server=self.smtp_server,
                smtp_port=str(self.smtp_port),
                email_user=self.email_user,
                email_pass=self.email_pass,
                email_to=self.email_to,
                subject=subject,
                json_payload=payload
            )
            return True
        except Exception:
            logger.exception("Error during collect_and_send")
            return False


# # --- Example main program ---
# if __name__ == "__main__":
#     # Create SIM7600 instance - adjust serial port if needed
#     sim = SIM7600(port="/dev/ttyUSB2", baudrate=115200, timeout=1)
#     sim.init_module()
#
#     # BLE manager - either set address or name
#     ble = BLEManager(address=BLE_ADDRESS, name=BLE_DEVICE_NAME, char_uuid=CHARACTERISTIC_UUID)
#
#     # Email credentials - change to your SMTP server / credentials
#     SMTP_SERVER = "smtp.yourmail.com"
#     SMTP_PORT = 587
#     EMAIL_USER = "dataStream@gmail.com"
#     EMAIL_PASS = "your_password"
#     EMAIL_TO = "cogesaf@domain.com"
#
#     bridge = BLESMTPBridge(sim7600=sim, ble_manager=ble,
#                            smtp_server=SMTP_SERVER, smtp_port=SMTP_PORT,
#                            email_user=EMAIL_USER, email_pass=EMAIL_PASS,
#                            email_to=EMAIL_TO)
#
#     ok = bridge.collect_and_send(subject="Sensor Data Report from nRF52")
#     if ok:
#         logger.info("Data collection & send completed successfully")
#     else:
#         logger.error("Failed to collect/send data")
