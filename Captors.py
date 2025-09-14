import serial
import RPi.GPIO as GPIO
import spidev  # For ADC (MCP3008)
import time
import pynmea2
from Constants import *

class SensorData:
    def __init__(self):
        GPIO.setmode(GPIO.BCM)  # Set GPIO mode
        
        # Set GPIO pins for sensors (Assuming they are digital)
        GPIO.setup(TEMPERATURE, GPIO.IN)  # Example pin setup for temperature
        GPIO.setup(REDOX, GPIO.IN)        # Example pin setup for redox
        GPIO.setup(PH, GPIO.IN)           # Example pin setup for pH
        GPIO.setup(TDS, GPIO.IN)   
        
        # Initialize serial for GPS module
        try:
            self.gps_serial = serial.Serial(GPS_PORT, GPS_BAUDRATE, timeout=1)
            print("GPS module connected.")
        except serial.SerialException:
            print("GPS module not connected. Mocking GPS data.")
            self.gps_serial = None  # No GPS serial connection
            
    def read_digital_sensor(self, pin):
        """Read a digital sensor (HIGH/LOW)"""
        return GPIO.input(pin)

    def read_temperature(self):
        """Read temperature from digital sensor (Assume HIGH = Over Temperature)"""
        temperature_state = self.read_digital_sensor(TEMPERATURE)
        return "Over Temperature" if temperature_state == GPIO.HIGH else "Normal"  # Example response

    def read_redox(self):
        """Read redox state from digital sensor (Assume HIGH = Redox Detected)"""
        redox_state = self.read_digital_sensor(REDOX)
        return "Redox Detected" if redox_state == GPIO.HIGH else "Normal"  # Example response

    def read_pH(self):
        """Read pH state from digital sensor (Assume HIGH = pH Threshold Exceeded)"""
        pH_state = self.read_digital_sensor(PH)
        return "pH Threshold Exceeded" if pH_state == GPIO.HIGH else "Normal"  # Example response

    def read_tds(self):
        """Read TDS state from digital sensor (Assume HIGH = TDS Detected)"""
        tds_state = self.read_digital_sensor(TDS)
        return "TDS Detected" if tds_state == GPIO.HIGH else "Normal"  # Example response


    def read_gps(self):
        """Read and parse GPS data from the serial module"""
        latitude, longitude = None, None
        if self.gps_serial is None:
            print("GPS module is not connected, returning mock data.")
            return 0.0, 0.0  # Return mock data, or you can leave it as None if preferred
            
        try:
            while True:
                line = self.gps_serial.readline().decode("utf-8", errors="ignore").strip()
                if line.startswith("$GPGGA"):  # Check for valid GPS data
                    msg = pynmea2.parse(line)
                    latitude = self.convert_gps_coordinates(msg.lat, msg.lat_dir)
                    longitude = self.convert_gps_coordinates(msg.lon, msg.lon_dir)
                    break
        except Exception as e:
            print(f"GPS Error: {e}")
        return latitude, longitude

    def convert_gps_coordinates(self, value, direction):
        """Convert raw GPS coordinates into decimal degrees"""
        if not value:
            return None
        degrees = int(value[:2])  # Extract first two characters as degrees
        minutes = float(value[2:]) / 60
        decimal = degrees + minutes
        return -decimal if direction in ["S", "W"] else decimal

    def collect_data(self):
        """Collect all sensor data and return it as a dictionary"""
        temperature = self.read_temperature()
        redox = self.read_redox()
        pH = self.read_pH()
        tds = self.read_tds()
        latitude, longitude = self.read_gps()
        timestamp = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())

        # Log data to CSV
        with open(DATA_FILE, "a") as file:
            file.write(f"{timestamp}, {latitude}, {longitude}, {temperature}, {redox}, {pH}, {tds}\n")

        return {
            "timestamp": timestamp,
            "latitude": latitude,
            "longitude": longitude,
            "temperature": temperature,
            "redox": redox,
            "pH": pH,
            "tds": tds
        }