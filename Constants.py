# Motor GPIO Pins
LEFT_MOTOR_FORWARD = 17
LEFT_MOTOR_BACKWARD = 18
RIGHT_MOTOR_FORWARD = 22
RIGHT_MOTOR_BACKWARD = 23


# Ultrasonic Sensor Pins
FRONT_TRIG = 24
FRONT_ECHO = 25
RIGHT_TRIG = 5
RIGHT_ECHO = 6
LEFT_TRIG = 13
LEFT_ECHO = 19

HEADING_TOLERANCE = 5  # Allowable deviation in degrees

PH = 0
TEMPERATURE = 1
REDOX = 2
TDS = 3

# GPS Serial Port
#GPS_PORT = "/dev/serial0"
GPS_PORT= "/dev/ttyUSB0" 
GPS_BAUDRATE = 9600

# Compass Configuration
COMPASS_PORT = "/dev/ttyS0"  # Adjust the port based on your system
COMPASS_BAUDRATE = 9600

SAFE_DISTANCE = 5

DATA_FILE = "water_quality_data.csv"





