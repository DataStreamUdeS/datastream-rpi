import time
import random
import math
from unittest.mock import MagicMock

# Mock the sensor classes
class MockUltrasonicSensor:
    def __init__(self, trig_pin, echo_pin):
        self.trig_pin = trig_pin
        self.echo_pin = echo_pin

    def get_distance(self):
        # Simulate random distance between 10 and 100 cm
        return random.uniform(10, 100)

class MockGPS:
    def __init__(self, manual_mode=False, manual_lat=0, manual_lon=0):
        self.manual_mode = manual_mode
        self.manual_lat = manual_lat
        self.manual_lon = manual_lon

    def get_position(self):
        # Return manual GPS coordinates for simulation
        return self.manual_lat, self.manual_lon

class MockCompass:
    def __init__(self, manual_mode=False, manual_heading=0):
        self.manual_mode = manual_mode
        self.manual_heading = manual_heading

    def get_heading(self):
        # Return a random heading between 0 and 360 degrees
        return random.uniform(0, 360)

# Mock the MotorController class
class MockMotorController:
    def __init__(self, left_forward=17, left_backward=27, right_forward=22, right_backward=23):
        pass  # No actual GPIO operations

    def set_speed(self, left_speed, right_speed):
        print(f"Setting speed: Left - {left_speed}%, Right - {right_speed}%")

    def move_forward(self):
        print("Moving forward")

    def stop(self):
        print("Stopping")

# Use the mocks in the BugNavigator class
class TestBugNavigator:
    def __init__(self, target_lat, target_lon, manual_mode=False, manual_lat=0, manual_lon=0, manual_heading=0):
        self.target_lat = target_lat
        self.target_lon = target_lon
        self.gps = MockGPS(manual_mode=manual_mode, manual_lat=manual_lat, manual_lon=manual_lon)
        self.compass = MockCompass(manual_mode=manual_mode, manual_heading=manual_heading)
        self.motors = MockMotorController()
        self.front_sensor = MockUltrasonicSensor(None, None)
        self.right_sensor = MockUltrasonicSensor(None, None)
        self.left_sensor = MockUltrasonicSensor(None, None)
        self.following_obstacle = False
        self.hit_point = None

    def distance_to_target(self, current_lat, current_lon):
        """Calculate the distance from the current position to the target position using Haversine formula."""
        delta_lat = math.radians(self.target_lat - current_lat)
        delta_lon = math.radians(self.target_lon - current_lon)

        a = math.sin(delta_lat / 2) ** 2 + math.cos(math.radians(current_lat)) * math.cos(math.radians(self.target_lat)) * math.sin(delta_lon / 2) ** 2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

        # Earth radius in kilometers
        radius = 6371

        distance = radius * c
        print(f"distance to target:{distance} ")
        return distance

    def calculate_target_heading(self, lat, lon):
        """Calculate the desired heading to the target position."""
        delta_lon = math.radians(self.target_lon - lon)
        lat1, lat2 = map(math.radians, [lat, self.target_lat])

        x = math.sin(delta_lon) * math.cos(lat2)
        y = math.cos(lat1) * math.sin(lat2) - (math.sin(lat1) * math.cos(lat2) * math.cos(delta_lon))
        heading = math.atan2(x, y)

        return (math.degrees(heading) + 360) % 360  # Normalize the heading to 0-360°

    def bug_algorithm(self):
        """Implement Bug 2 algorithm for navigation."""
        while True:
            current_lat, current_lon = self.gps.get_position()
            target_distance = self.distance_to_target(current_lat, current_lon)

            if target_distance < 2:
                self.motors.stop()
                break

            desired_heading = self.calculate_target_heading(current_lat, current_lon)

            # Obstacle detection simulation
            if self.front_sensor.get_distance() < 20:
                if not self.following_obstacle:
                    self.following_obstacle = True
                    self.hit_point = (current_lat, current_lon)
                else:
                    self.motors.move_forward()
            else:
                if self.following_obstacle:
                    self.following_obstacle = False

                self.motors.set_speed(50, 50)
                self.motors.move_forward()

            time.sleep(1)

# Instantiate the TestBugNavigator and run the algorithm
navigator = TestBugNavigator(target_lat=40.7128, target_lon=-74.0060)  # Target coordinates (example: New York City)
navigator.bug_algorithm()
