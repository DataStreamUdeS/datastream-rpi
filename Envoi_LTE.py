import serial
import time

# Configure the serial connection (adjust the port if needed)
ser = serial.Serial("/dev/ttyUSB2", baudrate=115200, timeout=1)


def send_at(cmd, expected="OK", timeout=3):
    """Send AT command and wait for expected response"""
    ser.write((cmd + "\r\n").encode())
    time.sleep(0.5)
    t_end = time.time() + timeout
    response = b""
    while time.time() < t_end:
        if ser.in_waiting:
            response += ser.read(ser.in_waiting)
            if expected.encode() in response:
                break
        time.sleep(0.1)
    print(f"Command: {cmd}\nResponse: {response.decode(errors='ignore')}")
    return response.decode(errors="ignore")


# === MAIN LOGIC ===
if __name__ == "__main__":
    # Wake up module
    send_at("AT")

    # Set full functionality
    send_at("AT+CFUN=1")

    # Ensure network is registered
    send_at("AT+CGATT?")
    send_at("AT+CREG?")

    # Set up SMTP parameters
    # Replace these values with your email provider details
    smtp_server = "smtp.yourmail.com"
    smtp_port = "587"  # use 25, 465, or 587 depending on your provider
    email_user = "cogesaflgmail.com"
    email_pass = "your_password"
    email_to = "recipient@domain.com"
    subject = "Test Email from SIM7600"
    body = "Hello, this is a test email sent from SIM7600 4G HAT on Raspberry Pi."

    # Configure SMTP
    send_at(f'AT+EMAILCID=1')  # Use PDP context 1
    send_at(f'AT+EMAILTO=30')  # Timeout 30s
    send_at(f'AT+SMTPSRV="{smtp_server}",{smtp_port}')
    send_at(f'AT+SMTPAUTH=1,"{email_user}","{email_pass}"')

    # Set sender and recipient
    send_at(f'AT+SMTPFROM="{email_user}","RaspberryPi SIM7600"')
    send_at(f'AT+SMTPRCPT=0,0,"{email_to}","Recipient"')

    # Set subject
    send_at(f'AT+SMTPSUB="{subject}"')

    # Set body (TEXT mode)
    send_at(f'AT+SMTPBODY={len(body)}')
    time.sleep(0.5)
    ser.write(body.encode() + b"\r\n")

    # Send email
    send_at("AT+SMTPSEND", "OK", 60)

    # Close SMTP session
    send_at("AT+SMTPCLR")

    print("Email sending process complete.")
