import RPi.GPIO as GPIO
import time
import threading

# === CONFIGURATION ===
ENCODER_A = 17  # GPIO pin for encoder channel A
ENCODER_B = 18  # GPIO pin for encoder channel B
GEAR_RATIO = 3.7
PULSES_PER_REV = 28   # pulses per motor shaft revolution
PPR_OUTPUT = int(PULSES_PER_REV * GEAR_RATIO)  # per gearbox output shaft

# === GLOBALS ===
pulse_count = 0
lock = threading.Lock()

def encoder_callback(channel):
    global pulse_count
    with lock:
        pulse_count += 1

# Setup GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup(ENCODER_A, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(ENCODER_B, GPIO.IN, pull_up_down=GPIO.PUD_UP)

GPIO.add_event_detect(ENCODER_A, GPIO.RISING, callback=encoder_callback)

def get_rpm(sample_time=1.0):
    global pulse_count
    with lock:
        start_count = pulse_count
    start_time = time.time()
    time.sleep(sample_time)
    with lock:
        end_count = pulse_count
    end_time = time.time()

    delta_count = end_count - start_count
    delta_time = end_time - start_time

    # Revolutions of output shaft
    revolutions = delta_count / PPR_OUTPUT
    rps = revolutions / delta_time
    rpm = rps * 60.0
    return rpm

try:
    while True:
        rpm = get_rpm(0.5)  # sample every 0.5s
        print(f"Motor Speed: {rpm:.2f} RPM")

except KeyboardInterrupt:
    print("Stopping...")
    GPIO.cleanup()
