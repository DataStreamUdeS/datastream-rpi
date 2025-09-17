import RPi.GPIO as GPIO
import time
from Raspberry_code import *  # assumes SIM7600, MotorController
from BLE_COM import BLEManager, BLESMTPBridge

class Protocol:
    def __init__(self, motor_controller, rope_length_per_rev):
        """
        :param motor_controller: Instance of MotorController
        :param rope_length_per_rev: Length of rope (cm or m) moved per drum revolution
        """
        self.motor = motor_controller
        self.rope_length_per_rev = rope_length_per_rev

    def depth_to_revolutions(self, depth):
        """Convert depth (same units as rope_length_per_rev) into required revolutions."""
        return depth / self.rope_length_per_rev

    def descente_moteur(self, target_depth):
        """Lower capsule until target depth is reached."""
        target_revs = self.depth_to_revolutions(target_depth)
        self.motor.reset_revolutions()
        self.motor.set_speed(-100)

        while self.motor.get_revolutions() < target_revs:
            if self.motor.is_stuck():
                print("Motor stuck during descent. Stopping.")
                break

            time.sleep(0.1)

        self.motor.stop()

    def montee_moteur(self, target_depth):
        """Raise capsule back up to surface (or to given depth)."""
        target_revs = self.depth_to_revolutions(target_depth)
        self.motor.reset_revolutions()
        self.motor.set_speed(100)

        while self.motor.get_revolutions() < target_revs:
            if self.motor.is_stuck():
                print("Motor stuck during ascent. Stopping.")
                self.motor.stop()
                "IL FAUDRAIT ALLUMER UNE LUMIÈRE QUI MET DÉMONTRE QU'IL Y A UN PROBLÈME"
                break

            time.sleep(0.1)

    def manual_descente(self):
        """Hold button to lower capsule manually."""
        while self.motor.BOUTON_DESCENTE():
            self.motor.set_speed(-100)
        self.motor.stop()

    def manual_montee(self):
        """Hold button to raise capsule manually."""
        while self.motor.BOUTON_MONTEE():
            self.motor.set_speed(100)
        self.motor.stop()


if __name__ == "__main__":
    try:
        # --- Setup motor controller ---
        motor = MotorController(
            pwm_pin=18,
            in1_pin=23,
            in2_pin=24,
            encoder_a=17,
            encoder_b=27
        )

        # --- Setup SIM7600 ---
        sim = SIM7600()
        sim.init_module()

        # --- Setup BLE <-> SMTP bridge ---
        ble_manager = BLEManager(name="nrf52_sensor_node")  # or use address
        bridge = BLESMTPBridge(
            sim7600=sim,
            ble_manager=ble_manager,
            smtp_server="smtp.example.com",
            smtp_port=587,
            email_user="datastream@example.com",
            email_pass="your_password",
            email_to="cogesaf@example.com"
        )

        # --- GPIO pin to trigger protocol ---
        START_PROTOCOL_PIN = 22
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(START_PROTOCOL_PIN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

        protocol = Protocol(motor, rope_length_per_rev=10)

        print("System ready. Waiting for start signal...")
        while True:
            # --- Manual control priority ---
            if motor.BOUTON_DESCENTE():
                print("🔽 Manual descent activated")
                protocol.manual_descente()  # will hold until button released
            elif motor.BOUTON_MONTEE():
                print("🔼 Manual ascent activated")
                protocol.manual_montee()  # will hold until button released

            # --- Automatic protocol trigger ---
            elif GPIO.input(START_PROTOCOL_PIN) == GPIO.HIGH:
                print("⏬ Starting protocol... lowering capsule.")
                protocol.descente_moteur(target_depth=5)  # example depth

                print("⏫ Raising capsule.")
                protocol.montee_moteur(target_depth=5)

                print("📡 Collecting data and sending via BLE/LTE...")
                bridge.collect_and_send(subject="Water Sensor Report")

                print("✅ Protocol finished. Waiting for next trigger.")

            time.sleep(0.05)  # small delay to avoid busy loop

        except KeyboardInterrupt:
        print("⏹ Stopped by user.")
    except Exception as e:
        print(f"❌ Error: {e}")
    finally:
        GPIO.cleanup()
        if motor is not None:
            motor.cleanup()
