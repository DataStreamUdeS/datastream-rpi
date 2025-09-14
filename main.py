import serial
import time
import re
import json

from Raspberry_code import*


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

# === Example Usage ===
if __name__ == "__main__":
    sim = SIM7600()
    sim.init_module()

    # 1. Get GPS
    latitude, longitude = sim.get_gps_coordinates()
    print(f"Latitude: {latitude:.6f}, Longitude: {longitude:.6f}")

    # 2. Fake sensor data (replace with actual readings)
    sensor_data = {
        "temperature": 21.5,
        "pH": 7.2,
        "TDS": 120,
        "redox": 235,
        "gps": {"lat": latitude, "lon": longitude}
    }

    # 3. Send JSON email
    sim.send_email(
        smtp_server="smtp.yourmail.com",
        smtp_port="587",
        email_user="dataStream@gmail.com",
        email_pass="your_password",
        email_to="cogesaf@domain.com",
        subject="Sensor Data Report",
        json_payload=sensor_data
    )

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



    if __name__ == "__main__":
        motor = MotorController(
            pwm_pin=18, in1_pin=23, in2_pin=24,
            encoder_a=17, encoder_b=27
        )
        sim = SIM7600()
        sim.init_module()

        # Lower capsule
        # La faire descendre pour 12 révolution
        motor.set_speed(-100)



        # Upper capsule



