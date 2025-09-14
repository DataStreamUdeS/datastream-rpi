import serial
import RPi.GPIO as GPIO
import time
import math
import pynmea2
from Constants import *
from Captors import *



class UltrasonicSensor:
    def __init__(self, trig_pin, echo_pin):
        self.trig_pin = trig_pin
        self.echo_pin = echo_pin
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.trig_pin, GPIO.OUT)
        GPIO.setup(self.echo_pin, GPIO.IN)

    def get_distance(self):
        """ Measure distance using the ultrasonic sensor """
        GPIO.output(self.trig_pin, True)
        time.sleep(0.00001)  # 10 microseconds pulse
        GPIO.output(self.trig_pin, False)

        start_time = time.time()
        stop_time = time.time()
        
        timeout = 1  # 1 second timeout
        start = time.time()
    
        while GPIO.input(self.echo_pin) == 0:
            start_time = time.time()

        # Wait for the echo pin to go LOW
        while GPIO.input(self.echo_pin) == 1:
            stop_time = time.time()
            if time.time() - start > timeout:
                print("Timeout waiting for echo to go low.")
                return -1  # Return -1 if there's a timeout

        elapsed_time = stop_time - start_time
        distance = (elapsed_time * 34300) / 2  # Speed of sound = 343 m/s

        return distance

class MotorController:
    def __init__(self, left_forward=LEFT_MOTOR_FORWARD, left_backward=LEFT_MOTOR_BACKWARD,
                 right_forward=RIGHT_MOTOR_FORWARD, right_backward=RIGHT_MOTOR_BACKWARD):
        GPIO.setmode(GPIO.BCM)
        self.left_forward = left_forward
        self.left_backward = left_backward
        self.right_forward = right_forward
        self.right_backward = right_backward

        GPIO.setup(self.left_forward, GPIO.OUT)
        GPIO.setup(self.left_backward, GPIO.OUT)
        GPIO.setup(self.right_forward, GPIO.OUT)
        GPIO.setup(self.right_backward, GPIO.OUT)

        self.left_pwm = GPIO.PWM(self.left_forward, 1000)
        self.right_pwm = GPIO.PWM(self.right_forward, 1000)
        self.left_pwm.start(0)
        self.right_pwm.start(0)

    def set_speed(self, left_speed, right_speed):
        self.left_pwm.ChangeDutyCycle(left_speed)
        self.right_pwm.ChangeDutyCycle(right_speed)

    def move_forward(self):
        GPIO.output(self.left_forward, GPIO.HIGH)
        GPIO.output(self.left_backward, GPIO.LOW)
        GPIO.output(self.right_forward, GPIO.HIGH)
        GPIO.output(self.right_backward, GPIO.LOW)

    def stop(self):
        GPIO.output(self.left_forward, GPIO.LOW)
        GPIO.output(self.left_backward, GPIO.LOW)
        GPIO.output(self.right_forward, GPIO.LOW)
        GPIO.output(self.right_backward, GPIO.LOW)


class GPS:
    def __init__(self, port=GPS_PORT, baudrate=9600, manual_mode=False, manual_lat=0, manual_lon=0):
        self.manual_mode = manual_mode
        self.manual_lat = manual_lat
        self.manual_lon = manual_lon
        if not self.manual_mode:
            try:
                self.ser = serial.Serial(port, baudrate, timeout=1)
            except serial.SerialException:
                print("GPS module not connected. Mocking GPS data.")
                self.ser = None
        else:
            self.ser = None  # If manual mode, no serial connection is needed.

    def get_position(self):
        """Reads GPS data and returns the latitude and longitude."""
        if self.manual_mode:
            return self.manual_lat, self.manual_lon  # Return manually set coordinates
        
        if not self.ser:
            raise ValueError("GPS serial connection is not initialized.")
        
        while True:
            line = self.ser.readline().decode('utf-8', errors='ignore').strip()
            if line.startswith('$GPGGA'):
                try:
                    msg = pynmea2.parse(line)
                    lat = msg.latitude
                    lon = msg.longitude
                    return lat, lon
                except pynmea2.ParseError:
                    continue
                
    def set_position(self, lat, lon):
        """Used for testing"""
        """Set the manual GPS position for simulation purposes."""
        self.manual_lat = lat
        self.manual_lon = lon
        
class Compass:
    def __init__(self, port=COMPASS_PORT, baudrate=COMPASS_BAUDRATE, manual_mode=False, manual_heading=0):
        self.manual_mode = manual_mode
        self.manual_heading = manual_heading

        if not self.manual_mode:
            try:
                self.ser = serial.Serial(port, baudrate, timeout=1)
            except serial.SerialException as e:
                print(f"Warning: Could not open compass serial port {port}. Running in manual mode.")
                self.manual_mode = True

    def get_heading(self):
        """ Returns the compass heading. Uses manual heading if in manual mode. """
        if self.manual_mode:
            return self.manual_heading

        while True:
            line = self.ser.readline().decode('utf-8', errors='ignore').strip()
            if line:
                try:
                    return float(line)  # The compass returns an angle in degrees
                except ValueError:
                    continue



class PIDController:
    """ Implémentation d'un contrôleur PID pour ajuster la direction. """

    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.prev_error = 0
        self.integral = 0
        self.last_time = time.time()

    def compute(self, setpoint, current_value):
        """ Calcule la correction PID """
        error = (setpoint - current_value + 360) % 360
        if error > 180:  # Ajuster pour éviter les rotations inutiles
            error -= 360

        current_time = time.time()
        dt = current_time - self.last_time if self.last_time else 1

        self.integral += error * dt
        derivative = (error - self.prev_error) / dt if dt > 0 else 0

        output = (self.Kp * error) + (self.Ki * self.integral) + (self.Kd * derivative)

        self.prev_error = error
        self.last_time = current_time

        return output

class BugNavigator:
    def __init__(self, target_lat, target_lon, manual_mode=False, manual_lat=0, manual_lon=0, manual_heading=0):
        self.target_lat = target_lat
        self.target_lon = target_lon

        # Use manual GPS and compass if manual_mode is enabled
        # Initialize GPS based on manual mode
        if manual_mode:
            self.gps = GPS(manual_mode=manual_mode, manual_lat=manual_lat, manual_lon=manual_lon)
        else:
            self.gps = GPS()
        self.compass = Compass(manual_mode=manual_mode, manual_heading=manual_heading)
        self.motors = MotorController()
        self.front_sensor = UltrasonicSensor(FRONT_TRIG, FRONT_ECHO)
        self.right_sensor = UltrasonicSensor(RIGHT_TRIG, RIGHT_ECHO)
        self.left_sensor = UltrasonicSensor(LEFT_TRIG, LEFT_ECHO)
        self.data_collector = SensorData()
        self.following_obstacle = False
        self.hit_point = None
        self.pid = PIDController(1.0, 0.1, 0.05)
        
    def distance_to_target(self, current_lat, current_lon):
        """Calculate the distance from the current position to the target position."""
        # Haversine formula to calculate the distance between two points on the Earth
        delta_lat = math.radians(self.target_lat - current_lat)
        delta_lon = math.radians(self.target_lon - current_lon)

        a = math.sin(delta_lat / 2) ** 2 + math.cos(math.radians(current_lat)) * math.cos(math.radians(self.target_lat)) * math.sin(delta_lon / 2) ** 2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

        # Earth radius in kilometers
        radius = 6371

        distance = radius * c
        return distance

    def calculate_target_heading(self, lat, lon):
        """ Calcule le cap désiré vers la cible en utilisant la formule de Haversine. """
        delta_lon = math.radians(self.target_lon - lon)
        lat1, lat2 = map(math.radians, [lat, self.target_lat])

        x = math.sin(delta_lon) * math.cos(lat2)
        y = math.cos(lat1) * math.sin(lat2) - (math.sin(lat1) * math.cos(lat2) * math.cos(delta_lon))
        heading = math.atan2(x, y)

        return (math.degrees(heading) + 360) % 360  # Normalisation 0-360°

    def adjust_heading(self, desired_heading):
        """ Ajuste la direction du bateau en utilisant le PID pour des corrections plus douces. """
        current_heading = self.compass.get_heading()
        correction = self.pid.compute(desired_heading, current_heading)

        # Base speed
        base_speed = 50
        left_speed = max(0, min(100, base_speed - correction))
        right_speed = max(0, min(100, base_speed + correction))

        self.motors.set_speed(left_speed, right_speed)

    def bug_algorithm(self):
        """ Implémente Bug 2 pour la navigation autonome. """
        while True:
            current_lat, current_lon = self.gps.get_position()
            target_distance = self.distance_to_target(current_lat, current_lon)
            
            # Console outputs showing navigation progress
            print(f"Current Position: Lat={current_lat}, Lon={current_lon}")
            print(f"Target Position: Lat={self.target_lat}, Lon={self.target_lon}")
            print(f"Distance to Target: {target_distance:.2f} km")
            print(f"Current Heading: {self.compass.get_heading()}°")
            print(f"Desired Heading: {desired_heading:.2f}°")
            print("-" * 40)
            
            # Vérifier si nous avons atteint la cible
            if target_distance < 2:
                self.motors.stop()
                self.data_collector.collect_data()
                break

            desired_heading = self.calculate_target_heading(current_lat, current_lon)


    
            # Détection d'obstacle
            if self.front_sensor.get_distance() < SAFE_DISTANCE:
                if not self.following_obstacle:
                    self.following_obstacle = True
                    self.hit_point = (current_lat, current_lon)
                    self.avoid_obstacle()
            else:
                if self.following_obstacle and self.is_on_m_line(current_lat, current_lon):
                    self.following_obstacle = False

                if not self.following_obstacle:
                    self.adjust_heading(desired_heading)
                    self.motors.move_forward()

            time.sleep(1)
    
    def is_on_m_line(self, current_lat, current_lon):
        """ Checks if the boat is back on the M-line (direct path to the target). """
        target_heading = self.calculate_target_heading(current_lat, current_lon)
        current_heading = self.compass.get_heading()

        heading_difference = abs(target_heading - current_heading)

        return heading_difference < HEADING_TOLERANCE  # A small tolerance to account for noise
    
    
    def avoid_obstacle(self):
        """ Implements obstacle avoidance while following the obstacle until it's safe to proceed. """
        following_right = self.right_sensor.get_distance() > self.left_sensor.get_distance()

        while True:
            current_lat, current_lon = self.gps.get_position()
            front_dist = self.front_sensor.get_distance()
            right_dist = self.right_sensor.get_distance()
            left_dist = self.left_sensor.get_distance()

            # Check if we are back on the M-line and it's safe to proceed
            if self.is_on_m_line(current_lat, current_lon) and front_dist > SAFE_DISTANCE:
                self.following_obstacle = False
                return  # Resume normal navigation

            if following_right:
                if front_dist < SAFE_DISTANCE:  
                    self.motors.stop()
                    self.motors.set_speed(40, 60)  # Slight left turn
                    time.sleep(0.5)
                elif right_dist > SAFE_DISTANCE:
                    self.motors.set_speed(60, 40)  # Slight right turn
                    time.sleep(0.5)
                else:
                    self.motors.move_forward()
            else:
                if front_dist < SAFE_DISTANCE:
                    self.motors.stop()
                    self.motors.set_speed(60, 40)  # Slight right turn
                    time.sleep(0.5)
                elif left_dist > SAFE_DISTANCE:
                    self.motors.set_speed(40, 60)  # Slight left turn
                    time.sleep(0.5)
                else:
                    self.motors.move_forward()

            desired_heading = self.calculate_target_heading(current_lat, current_lon)
            self.adjust_heading(desired_heading)

            time.sleep(0.5)
