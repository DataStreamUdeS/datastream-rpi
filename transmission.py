import csv
import os
import smtplib
from email.mime.text import MIMEText
from email.mime.multipart import MIMEMultipart
from email.mime.base import MIMEBase
from email import encoders
from Constants import *
from Captors import SensorData

# Constants for email
EMAIL_USER = 'kibosdavid@gmail.com'
EMAIL_PASSWORD = 'sqng mypn nuue tsvy'
TO_EMAIL = 'kibosdavid@gmail.com'



class DataTransmission:
    def __init__(self):
        self.sensor = SensorData()
        self.data_file = DATA_FILE
        # **Reset CSV file if testing**  
        if os.path.exists(self.data_file):
            os.remove(self.data_file)
            print("Previous test data deleted.")

    def save_data(self, data):
        file_exists = os.path.isfile(self.data_file)

        with open(self.data_file, mode='a', newline='') as file:
            writer = csv.writer(file)

            # Write header if file is new
            if not file_exists:
                writer.writerow(["Timestamp", "Latitude", "Longitude", "Temperature (°C)", "Redox (mV)", "pH", "TDS (ppm)"])

            # Write collected data
            writer.writerow([
                data["timestamp"], data["latitude"], data["longitude"],
                data["temperature"], data["redox"], data["pH"], data["tds"]
            ])

    def send_email(self):
        """Sends the CSV file via email."""
        if not os.path.exists(self.data_file):
            print("No data file found for email transmission.")
            return

        try:
            server = smtplib.SMTP('smtp.gmail.com', 587)
            server.starttls()
            server.login(EMAIL_USER, EMAIL_PASSWORD)

            # Create email
            subject = "Water Quality Data Report"
            body = "Please find the attached water quality data report."

            msg = MIMEMultipart()
            msg['From'] = EMAIL_USER
            msg['To'] = TO_EMAIL
            msg['Subject'] = subject
            msg.attach(MIMEText(body, 'plain'))

            # Attach CSV file
            with open(self.data_file, "rb") as attachment:
                part = MIMEBase("application", "octet-stream")
                part.set_payload(attachment.read())

            encoders.encode_base64(part)
            part.add_header("Content-Disposition", f"attachment; filename={self.data_file}")
            msg.attach(part)

            # Send email
            server.sendmail(EMAIL_USER, TO_EMAIL, msg.as_string())
            print(f"Email sent with {self.data_file} attached.")

            server.quit()

        except Exception as e:
            print(f"Email sending failed: {e}")

    def send_data(self):
        """Collects data, saves it, then sends it via email."""
        data = self.sensor.collect_data()

        # Save to CSV
        self.save_data(data)

        # Send via email
        self.send_email()

        return data
