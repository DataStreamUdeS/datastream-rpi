from bleak import BleakClient, BleakScanner
import asyncio

# replace with your device name or address
DEVICE_NAME = "DataStream Capsule"

async def main():
    print("Scanning for device...")
    device = await BleakScanner.find_device_by_name(DEVICE_NAME)
    if not device:
        print("Device not found!")
        return

    async with BleakClient(device) as client:
        print(f"Connected: {client.is_connected}")

        # List all services and characteristics
        for service in client.services:
            print(f"\nService {service.uuid}")
            for char in service.characteristics:
                props = ",".join(char.properties)
                print(f"  Char {char.uuid} ({props})")

        # Example: read one characteristic (replace UUIDs)
        WATER_DATA_UUID = "12345678-1234-5678-1234-56789abcdef1"
        try:
            data = await client.read_gatt_char(WATER_DATA_UUID)
            print("Raw data:", data)
        except Exception as e:
            print("Read failed:", e)

        # Example: subscribe to notifications
        def notification_handler(sender, data):
            print(f"Notification from {sender}: {data}")

        await client.start_notify(WATER_DATA_UUID, notification_handler)
        await asyncio.sleep(10.0)
        await client.stop_notify(WATER_DATA_UUID)

asyncio.run(main())
