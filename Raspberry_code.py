import serial
import time
import re
import json


class SIM7600:
    def __init__(self, port="/dev/ttyUSB2", baudrate=115200, timeout=1):
        self.ser = serial.Serial(port, baudrate=baudrate, timeout=timeout)

    def send_at(self, cmd, expected="OK", timeout=3, verbose=True):
        """Send AT command and wait for expected response"""
        self.ser.write((cmd + "\r\n").encode())
        time.sleep(0.5)
        t_end = time.time() + timeout
        response = b""
        while time.time() < t_end:
            if self.ser.in_waiting:
                response += self.ser.read(self.ser.in_waiting)
                if expected.encode() in response:
                    break
            time.sleep(0.1)

        response_str = response.decode(errors="ignore")
        if verbose:
            print(f"Command: {cmd}\nResponse: {response_str}")
        return response_str

    def init_module(self):
        """Wake up and initialize the SIM7600 module"""
        self.send_at("AT")
        self.send_at("AT+CFUN=1")
        self.send_at("AT+CGATT?")
        self.send_at("AT+CREG?")

    def get_gps_coordinates(self):
        """Enable GPS and return (lat, lon) once fix is obtained"""
        self.send_at("AT+CGPS=1")
        print("Waiting for GPS fix... (can take ~30s outdoors)")

        coords = None
        while not coords:
            resp = self.send_at("AT+CGPSINFO", expected="OK", timeout=2)
            if "+CGPSINFO: ,,,,,,,," not in resp:
                coords = self.parse_gps_info(resp)
            time.sleep(2)

        self.send_at("AT+CGPS=0")  # turn off GPS to save power
        return coords

    @staticmethod
    def parse_gps_info(raw):
        """
        Parse SIM7600 GPS info string into decimal degrees
        Example response:
        +CGPSINFO: 3723.2475,N,12158.3416,W,120415,161229.0,100.0,0.0
        """
        match = re.search(r"\+CGPSINFO: ([\d.]+),([NS]),([\d.]+),([EW]),", raw)
        if match:
            lat_raw, ns, lon_raw, ew = match.groups()

            # Latitude
            lat_deg = int(float(lat_raw) / 100)
            lat_min = float(lat_raw) - lat_deg * 100
            lat = lat_deg + lat_min / 60.0
            if ns == "S":
                lat = -lat

            # Longitude
            lon_deg = int(float(lon_raw) / 100)
            lon_min = float(lon_raw) - lon_deg * 100
            lon = lon_deg + lon_min / 60.0
            if ew == "W":
                lon = -lon

            return lat, lon
        return None

    def send_email(self, smtp_server, smtp_port, email_user, email_pass,
                   email_to, subject, json_payload):
        """Send JSON data via SMTP email using SIM7600"""
        self.send_at("AT+EMAILCID=1")  # Use PDP context 1
        self.send_at("AT+EMAILTO=30")  # Timeout 30s
        self.send_at(f'AT+SMTPSRV="{smtp_server}",{smtp_port}')
        self.send_at(f'AT+SMTPAUTH=1,"{email_user}","{email_pass}"')

        # Set sender & recipient
        self.send_at(f'AT+SMTPFROM="{email_user}","RaspberryPi SIM7600"')
        self.send_at(f'AT+SMTPRCPT=0,0,"{email_to}","Recipient"')

        # Subject
        self.send_at(f'AT+SMTPSUB="{subject}"')

        # Email body (JSON payload)
        body = json.dumps(json_payload, indent=2)
        self.send_at(f'AT+SMTPBODY={len(body)}')
        time.sleep(0.5)
        self.ser.write(body.encode() + b"\r\n")

        # Send email
        self.send_at("AT+SMTPSEND", "OK", 60)

        # Close session
        self.send_at("AT+SMTPCLR")
        print("Email sent successfully.")

import RPi.GPIO as GPIO
import time
import threading


class MotorController:
    def __init__(self, pwm_pin, in1_pin, in2_pin, encoder_a, encoder_b,
                 counts_per_rev=28*3.7):
        """
        MotorController for 5202 Yellow Jacket with encoder.

        :param pwm_pin: GPIO pin for PWM speed control
        :param in1_pin: GPIO pin for motor driver IN1
        :param in2_pin: GPIO pin for motor driver IN2
        :param encoder_a: GPIO pin for encoder channel A
        :param encoder_b: GPIO pin for encoder channel B
        :param counts_per_rev: Encoder counts per shaft revolution (CPR)
                               28 CPR x gearbox ratio (3.7:1) = 103.6 effective CPR
        """
        self.pwm_pin = pwm_pin
        self.in1_pin = in1_pin
        self.in2_pin = in2_pin
        self.encoder_a = encoder_a
        self.encoder_b = encoder_b
        self.counts_per_rev = counts_per_rev

        # State variables
        self.position = 0
        self.last_time = time.time()
        self.speed_rpm = 0
        self.direction = 1  # +1 forward, -1 backward

        # Setup GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.pwm_pin, GPIO.OUT)
        GPIO.setup(self.in1_pin, GPIO.OUT)
        GPIO.setup(self.in2_pin, GPIO.OUT)
        GPIO.setup(self.encoder_a, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.encoder_b, GPIO.IN, pull_up_down=GPIO.PUD_UP)

        # PWM for speed
        self.pwm = GPIO.PWM(self.pwm_pin, 1000)  # 1 kHz
        self.pwm.start(0)

        # Encoder interrupts
        GPIO.add_event_detect(self.encoder_a, GPIO.BOTH, callback=self._update_encoder)
        GPIO.add_event_detect(self.encoder_b, GPIO.BOTH, callback=self._update_encoder)

        # Thread for speed calculation
        self._running = True
        self.speed_thread = threading.Thread(target=self._compute_speed, daemon=True)
        self.speed_thread.start()

    def _update_encoder(self, channel):
        """Quadrature encoder update"""
        a = GPIO.input(self.encoder_a)
        b = GPIO.input(self.encoder_b)

        if a == b:
            self.position += 1
            self.direction = 1
        else:
            self.position -= 1
            self.direction = -1

    def _compute_speed(self):
        """Background thread to compute RPM"""
        while self._running:
            time.sleep(0.1)  # sample every 100 ms
            now = time.time()
            dt = now - self.last_time
            self.last_time = now

            # Speed in RPM
            self.speed_rpm = (self.position / self.counts_per_rev) * (60.0 / dt)
            self.position = 0  # reset counter for next window

    def set_speed(self, duty_cycle):
        """
        Set motor speed with PWM (0-100%).
        Positive for forward, negative for backward.
        """
        if duty_cycle > 0:
            GPIO.output(self.in1_pin, GPIO.HIGH)
            GPIO.output(self.in2_pin, GPIO.LOW)
            self.direction = 1
        elif duty_cycle < 0:
            GPIO.output(self.in1_pin, GPIO.LOW)
            GPIO.output(self.in2_pin, GPIO.HIGH)
            self.direction = -1
        else:
            self.stop()
            return

        self.pwm.ChangeDutyCycle(abs(duty_cycle))

    def stop(self):
        """Stop motor"""
        GPIO.output(self.in1_pin, GPIO.LOW)
        GPIO.output(self.in2_pin, GPIO.LOW)
        self.pwm.ChangeDutyCycle(0)

    def get_speed_rpm(self):
        """Return current motor speed in RPM"""
        return self.speed_rpm

    def get_direction(self):
        """Return direction: 1 forward, -1 backward"""
        return self.direction

    def cleanup(self):
        """Cleanup GPIO and stop thread"""
        self._running = False
        self.speed_thread.join()
        self.stop()
        GPIO.cleanup()


# === Example Usage ===
if __name__ == "__main__":
    motor = MotorController(
        pwm_pin=18, in1_pin=23, in2_pin=24,
        encoder_a=17, encoder_b=27
    )

    try:
        print("Forward at 50%")
        motor.set_speed(50)
        time.sleep(3)

        print("Backward at 75%")
        motor.set_speed(-75)
        time.sleep(3)

        print("Speed:", motor.get_speed_rpm(), "RPM")
        print("Direction:", "Forward" if motor.get_direction() == 1 else "Backward")

        motor.stop()
    finally:
        motor.cleanup()

# # === Example Usage ===
# if __name__ == "__main__":
#     sim = SIM7600()
#     sim.init_module()
#
#     # 1. Get GPS
#     latitude, longitude = sim.get_gps_coordinates()
#     print(f"Latitude: {latitude:.6f}, Longitude: {longitude:.6f}")
#
#     # 2. Fake sensor data (replace with actual readings)
#     sensor_data = {
#         "temperature": 21.5,
#         "pH": 7.2,
#         "TDS": 120,
#         "redox": 235,
#         "gps": {"lat": latitude, "lon": longitude}
#     }
#
#     # 3. Send JSON email
#     sim.send_email(
#         smtp_server="smtp.yourmail.com",
#         smtp_port="587",
#         email_user="dataStream@gmail.com",
#         email_pass="your_password",
#         email_to="cogesaf@domain.com",
#         subject="Sensor Data Report",
#         json_payload=sensor_data
#     )
#
#     # === Example Usage ===
#     if __name__ == "__main__":
#         motor = MotorController(
#             pwm_pin=18, in1_pin=23, in2_pin=24,
#             encoder_a=17, encoder_b=27
#         )
#
#         try:
#             print("Forward at 50%")
#             motor.set_speed(50)
#             time.sleep(3)
#
#             print("Backward at 75%")
#             motor.set_speed(-75)
#             time.sleep(3)
#
#             print("Speed:", motor.get_speed_rpm(), "RPM")
#             print("Direction:", "Forward" if motor.get_direction() == 1 else "Backward")
#
#             motor.stop()
#         finally:
#             motor.cleanup()

git

