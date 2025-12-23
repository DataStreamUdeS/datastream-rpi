#!/usr/bin/env python3
import asyncio
from bleak import BleakClient, BleakScanner
from datetime import datetime
import csv
import os
import struct
import sys
import serial
import time
import re
import smtplib
from email.message import EmailMessage
import RPi.GPIO as GPIO
import json

# ---------------- DEBUG CONFIG ----------------
DEBUG = True  # Set to True only for bench testing without buttons

# ================= LED CONFIGURATION =================
LED_PIN_IDLE = 5   # Green
LED_PIN_BUSY = 6   # Yellow
LED_PIN_WAIT = 13  # Blue
LED_PIN_STOP = 19  # Red

# ================= BUTTON CONFIG =================
BTN_START = 24
BTN_STOP = 27
BTN_RECONNECT = 22
BTN_SEND_EMAIL = 23

# GPIO Setup
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(BTN_START, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(BTN_STOP, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(BTN_RECONNECT, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(BTN_SEND_EMAIL, GPIO.IN, pull_up_down=GPIO.PUD_UP)

GPIO.setup(LED_PIN_IDLE, GPIO.OUT)
GPIO.setup(LED_PIN_BUSY, GPIO.OUT)
GPIO.setup(LED_PIN_WAIT, GPIO.OUT)
GPIO.setup(LED_PIN_STOP, GPIO.OUT)

stop_flag = False

# ================= UUIDS =================
SERVICE_UUID = "0000ab00-0000-1000-8000-00805f9b34fb"
CONTROL_CHAR_UUID = "0000182a-0000-1000-8000-00805f9b34fb"
DATA_CHAR_UUID = "0000181a-0000-1000-8000-00805f9b34fb"
STATUS_CHAR_UUID = "0000184a-0000-1000-8000-00805f9b34fb"

# ================= FILES & DEVICE =================
LOG_FILE = "/home/pi/water_ble_log.txt"
DEVICE_NAME = "DataStream Capsule"
SENSOR_LOG_FILE = "/home/pi/sensor_log.csv"   # Full path recommended
GPS_LOG_FILE = "/home/pi/gps_log.csv"

# ======== EMAIL CONFIG (USE GMAIL APP PASSWORD!) ========
SENDER_EMAIL = "kibosdavid@gmail.com"
SENDER_PASSWORD = "fnat jhmv eymq tgvk"   
RECIPIENT_EMAIL = "kibd5171@usherbrooke.ca"
SUBJECT = "Résultats des tests – Données capteur + GPS"
SMTP_SERVER = "smtp.gmail.com"
SMTP_PORT = 587

# =============================================
# ================= LED CONTROL ===============
# =============================================
def set_system_state(state):
    GPIO.output(LED_PIN_IDLE, GPIO.LOW)
    GPIO.output(LED_PIN_BUSY, GPIO.LOW)
    GPIO.output(LED_PIN_WAIT, GPIO.LOW)
    GPIO.output(LED_PIN_STOP, GPIO.LOW)
    if state == "IDLE":
        GPIO.output(LED_PIN_IDLE, GPIO.HIGH)
    elif state == "BUSY":
        GPIO.output(LED_PIN_BUSY, GPIO.HIGH)
    elif state == "WAITING":
        GPIO.output(LED_PIN_WAIT, GPIO.HIGH)
    elif state == "STOPPED":
        GPIO.output(LED_PIN_STOP, GPIO.HIGH)

# =============================================
# ================= LOGGING ===================
# =============================================
def log(msg):
    ts = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    line = f"[{ts}] {msg}"
    print(line)
    try:
        with open(LOG_FILE, "a") as f:
            f.write(line + "\n")
    except Exception:
        pass

# =============================================
# =============== BUTTON CALLBACKS ============
# =============================================
def stop_pressed(channel):
    global stop_flag
    stop_flag = True
    log("STOP BUTTON PRESSED – System halted")
    set_system_state("STOPPED")

def email_pressed(channel):
    log("EMAIL button pressed – sending now")
    set_system_state("BUSY")
    send_email()
    set_system_state("IDLE" if not stop_flag else "STOPPED")

# GPIO.add_event_detect(BTN_STOP, GPIO.FALLING, callback=stop_pressed, bouncetime=300)
GPIO.add_event_detect(BTN_SEND_EMAIL, GPIO.FALLING, callback=email_pressed, bouncetime=300)

# =============================================
# =================== GPS (SIM7600) ===========
# =============================================
def find_at_port():
    ports = ["/dev/ttyUSB3", "/dev/ttyUSB2", "/dev/ttyUSB1", "/dev/ttyUSB0"]
    for p in ports:
        if not os.path.exists(p):
            continue
        try:
            ser = serial.Serial(p, 115200, timeout=1)
            ser.write(b"AT\r\n")
            time.sleep(0.4)
            if b"OK" in ser.read(100):
                ser.close()
                log(f"SIM7600 AT port: {p}")
                return p
            ser.close()
        except:
            continue
    log("SIM7600 AT port NOT found!")
    return None

def get_gps_location():
    port = find_at_port()
    if not port:
        return (0.0, 0.0)

    try:
        ser = serial.Serial(port, 115200, timeout=10)
    except Exception as e:
        log(f"GPS serial error: {e}")
        return (0.0, 0.0)

    def at(cmd, expect="OK", timeout=8):
        ser.flushInput()
        ser.write((cmd + "\r\n").encode())
        buf = ""
        t0 = time.time()
        while time.time() - t0 < timeout:
            if ser.in_waiting:
                buf += ser.read(ser.in_waiting).decode(errors="ignore")
            if expect in buf:
                return buf
            time.sleep(0.1)
        return buf

    at("AT+CGNSPWR=1", timeout=5)
    time.sleep(1)
    at('AT+CGNSSEQ="RMC"')

    log("Waiting for GPS fix...")
    for _ in range(60):  # max ~6 minutes
        if stop_flag:
            ser.close()
            return (0.0, 0.0)
        resp = at("AT+CGNSINF", "+CGNSINF:")
        if "+CGNSINF:" in resp:
            try:
                parts = resp.split("+CGNSINF:")[1].split("\r\n")[0].split(",")
                if len(parts) > 4 and parts[1] in ["1","2","3","4"]:
                    lat = float(parts[3])
                    lon = float(parts[4])
                    log(f"GPS FIX → {lat:.6f}, {lon:.6f}")
                    ser.close()
                    return (lat, lon)
            except:
                pass
        time.sleep(6)

    log("No GPS fix – using 0.0,0.0")
    ser.close()
    return (0.0, 0.0)

# =============================================
# =================== CSV HELPERS =============
# =============================================
def ensure_csv_header():
    if not os.path.exists(SENSOR_LOG_FILE):
        with open(SENSOR_LOG_FILE, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["timestamp", "latitude", "longitude", "ph", "temp", "orp", "depth", "o2"])

def write_to_csv(position, data_list):
    lat, lon = position
    with open(SENSOR_LOG_FILE, "a", newline="") as f:
        writer = csv.writer(f)
        ts = datetime.now().isoformat()
        for reading in data_list:
            writer.writerow([
                ts,
                f"{lat:.6f}",
                f"{lon:.6f}",
                f"{reading['ph']:.2f}",
                f"{reading['temp']:.2f}",
                f"{reading['orp']:.1f}",
                f"{reading['depth']:.2f}",
                f"{reading['o2']:.2f}"
            ])

# =============================================
# =================== EMAIL ===================
# =============================================
def send_email():
    if not os.path.exists(SENSOR_LOG_FILE):
        log("No data file to send yet")
        return

    log("Preparing email with attachment...")
    msg = EmailMessage()
    msg["From"] = SENDER_EMAIL
    msg["To"] = RECIPIENT_EMAIL
    msg["Subject"] = f"{SUBJECT} – {datetime.now().strftime('%Y-%m-%d %H:%M')}"

    msg.set_content(
        f"Données collectées par le système Raspberry Pi + SIM7600 + Capsule BLE\n"
        f"Envoyé le : {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n\n"
        f"Fichier joint : {os.path.basename(SENSOR_LOG_FILE)}"
    )

    with open(SENSOR_LOG_FILE, "rb") as f:
        msg.add_attachment(f.read(), maintype="text", subtype="csv", filename="sensor_data.csv")

    try:
        with smtplib.SMTP(SMTP_SERVER, SMTP_PORT) as server:
            server.starttls()
            server.login(SENDER_EMAIL, SENDER_PASSWORD)
            server.send_message(msg)
        log("Email sent successfully!")
    except Exception as e:
        log(f"Email FAILED: {e}")

# =============================================
# ============ BLE DEVICE DISCOVERY ===========
# =============================================
async def find_capsule():
    log("Scanning for DataStream Capsule...")
    devices = await BleakScanner.discover(timeout=60.0)
    for d in devices:
        if d.name and DEVICE_NAME in d.name:
            log(f"Found by name: {d.name} @ {d.address}")
            return d
        if SERVICE_UUID.lower() in [u.lower() for u in d.metadata.get("uuids", [])]:
            log(f"Found by service UUID: {d.address}")
            return d
    return None


# =============================================
# =============== INITIAL HANDSHAKE ===========
# =============================================
async def initial_handshake():
    if stop_flag:
        return False
    set_system_state("BUSY")
    log("Searching for capsule...")

    device = await find_capsule()
    if not device:
        log("Capsule not found!")
        set_system_state("IDLE")
        return False

    async with BleakClient(device) as client:
        log(f"Connected to {device.address}")

        ready_event = asyncio.Event()

        async def status_handler(sender, data):
            try:
                msg = data.decode("utf-8").strip()
                log(f"Status: {msg}")
                if "Ready" in msg:
                    ready_event.set()
            except:
                pass

        await client.start_notify(STATUS_CHAR_UUID, status_handler)
        await client.write_gatt_char(CONTROL_CHAR_UUID, b"Init")

        try:
            await asyncio.wait_for(ready_event.wait(), timeout=6.0)
            log("Capsule ready")
        except asyncio.TimeoutError:
            log("Ready timeout – continuing anyway")

        await client.write_gatt_char(CONTROL_CHAR_UUID, b"start_retriving")
        await asyncio.sleep(2)
        log("Initial handshake complete – disconnecting")

    set_system_state("IDLE")
    return True

# =============================================
# ============ WAIT FOR RECONNECT BUTTON ======
# =============================================
async def wait_for_reconnect_button():
    if DEBUG:
        log("DEBUG mode → skipping reconnect button wait")
        await asyncio.sleep(2)
        return True

    set_system_state("WAITING")
    log("Waiting for RECONNECT button (blue LED)...")
    while not stop_flag:
        if GPIO.input(BTN_RECONNECT) == GPIO.LOW:
            log("RECONNECT button pressed")
            await asyncio.sleep(0.3)  # debounce
            return True
        await asyncio.sleep(0.1)
    return False

# =============================================
# ============ RECONNECT & COLLECT DATA =======
# =============================================
async def reconnect_and_collect():
    set_system_state("BUSY")
    log("Reconnecting to capsule to collect sensor data...")

    device = None
    for attempt in range(100):
        device = await find_capsule()
        if device:
            break
        log(f"Scan attempt {attempt+1}/4 failed, retrying...")
        await asyncio.sleep(3)

    if not device:
        log("Failed to reconnect – giving up")
        set_system_state("IDLE")
        return

    async with BleakClient(device) as client:
        log(f"Reconnected to {device.address}")

        received_data = []
        
        await client.write_gatt_char(CONTROL_CHAR_UUID, b"ready_to_tx")

        async def data_handler(sender, data):
            if len(data) == 20:
                depth, temp, ph, orp, o2 = struct.unpack("<5f", data)
                log(f"Data → Depth:{depth:.2f}m Temp:{temp:.1f}°C pH:{ph:.2f} ORP:{orp:.0f}mV DO:{o2:.2f}mg/L")
                received_data.append({"depth": depth, "temp": temp, "ph": ph, "orp": orp, "o2": o2})
            else:
                log(f"Unexpected data length: {len(data)} bytes")

        await client.start_notify(DATA_CHAR_UUID, data_handler)
        
        # Sends the command to start transmission
        await client.write_gatt_char(CONTROL_CHAR_UUID, b"done_tx")
        
        log("Requested data transmission – waiting 12 seconds...")
        await asyncio.sleep(12)
        await client.stop_notify(DATA_CHAR_UUID)

        if received_data:
            ensure_csv_header()
            pos = get_gps_location()                     # REAL GPS HERE
            write_to_csv(pos, received_data)
            log(f"{len(received_data)} readings saved with GPS {pos}")
            send_email()                                 # Auto-send after collection
        else:
            log("No sensor data received during collection window")

    set_system_state("IDLE")

# =============================================
# =================== MAIN ====================
# =============================================
async def main():
    log("=== Water Quality System Started ===")
    set_system_state("IDLE")

    # Wait for START button (skip in DEBUG)
    if not DEBUG:
        log("Press START button (green LED) to begin...")
        while GPIO.input(BTN_START) == GPIO.HIGH:
            if stop_flag:
                return
            await asyncio.sleep(0.1)
        log("START button pressed – beginning mission")
        await asyncio.sleep(0.5)

    if await initial_handshake():
        if await wait_for_reconnect_button():
            await reconnect_and_collect()

    log("Mission complete")
    set_system_state("IDLE")

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        log("Interrupted by user")
    finally:
        GPIO.cleanup()
        log("GPIO cleaned up – goodbye!")
