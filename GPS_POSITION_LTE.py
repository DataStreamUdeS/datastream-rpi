import serial
import time
import re

# Open serial connection to SIM7600 (adjust port if needed)
ser = serial.Serial("/dev/ttyUSB2", baudrate=115200, timeout=1)

def send_at(cmd, expected="OK", timeout=3):
    """Send AT command and wait for response"""
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

def parse_gps_info(raw):
    """
    Example response:
    +CGPSINFO: 3723.2475,N,12158.3416,W,120415,161229.0,100.0,0.0
    """
    match = re.search(r"\+CGPSINFO: ([\d.]+),([NS]),([\d.]+),([EW]),", raw)
    if match:
        lat_raw, ns, lon_raw, ew = match.groups()

        # Convert NMEA style (dddmm.mmmm) into decimal degrees
        lat_deg = int(float(lat_raw) / 100)
        lat_min = float(lat_raw) - lat_deg * 100
        lat = lat_deg + lat_min / 60.0
        if ns == "S":
            lat = -lat

        lon_deg = int(float(lon_raw) / 100)
        lon_min = float(lon_raw) - lon_deg * 100
        lon = lon_deg + lon_min / 60.0
        if ew == "W":
            lon = -lon

        return lat, lon
    return Nonew

if __name__ == "__main__":
    # Wake up module
    send_at("AT")
    send_at("AT+CGPS=1")  # Turn on GPS
    print("Waiting for GPS fix... this may take ~30s outdoors.")

    while True:
        resp = send_at("AT+CGPSINFO", expected="OK", timeout=2)
        if "+CGPSINFO: ,,,,,,,," not in resp:
            coords = parse_gps_info(resp)
            if coords:
                lat, lon = coords
                print(f"Latitude: {lat:.6f}, Longitude: {lon:.6f}")
                break
        time.sleep(2)

    send_at("AT+CGPS=0")  # Turn off GPS to save power
