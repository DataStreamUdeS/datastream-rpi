from Navigation import BugNavigator
from transmission import *
import RPi.GPIO as GPIO

if __name__ == "__main__":
    try:
        use_manual = input("Use manual coordinates? (y/n): ").strip().lower()
        if use_manual == 'y':
            # manual_lat = float(input("Enter latitude: "))
            # manual_lon = float(input("Enter longitude: "))
            # manual_heading = float(input("Enter heading (0-360°): "))

            manual_lat = 45
            manual_lon = -75
            manual_heading = 90

            navigator = BugNavigator(manual_lat, manual_lon, manual_mode=True, manual_lat=manual_lat,
                                     manual_lon=manual_lon, manual_heading=manual_heading)
        else:
            navigator = BugNavigator(45.3456, -73.5678, manual_mode=False)

        navigator.bug_algorithm()

        # Create an instance of DataTransmission and call the send_data method
        data_transmission = DataTransmission()
        data_transmission.send_data()

    except KeyboardInterrupt:
        print("Navigation interrupted by user.")

    finally:
        GPIO.cleanup()
