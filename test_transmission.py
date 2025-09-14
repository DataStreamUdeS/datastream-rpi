import random
import os
from datetime import datetime
from transmission import DataTransmission

# Create a DataTransmission instance
data_transmission = DataTransmission()

# Generate and save multiple test data entries
NUM_TEST_ENTRIES = 100  # ✅ Generate 100 test samples

print(f"Generating {NUM_TEST_ENTRIES} test data entries...")

for _ in range(NUM_TEST_ENTRIES):
    test_data = {
        "timestamp": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
        "latitude": round(random.uniform(-90, 90), 6),
        "longitude": round(random.uniform(-180, 180), 6),
        "temperature": round(random.uniform(0, 35), 2),
        "redox": round(random.uniform(-200, 200), 2),
        "pH": round(random.uniform(4, 10), 2),
        "tds": round(random.uniform(0, 1000), 2)
    }

    # Save each test data entry
    data_transmission.save_data(test_data)

print("✅ Test data generation complete.")

# Ensure the CSV file exists
assert os.path.exists(data_transmission.data_file), "❌ Data file was not created."

# Test sending email after all test data is saved
print("Testing send_email...")
data_transmission.send_email()

# Test send_data method by collecting one more entry
print("Testing send_data...")
collected_data = data_transmission.send_data()
assert isinstance(collected_data, dict), "❌ send_data() did not return a dictionary."
assert "timestamp" in collected_data, "❌ send_data() output is missing 'timestamp'."
assert "temperature" in collected_data, "❌ send_data() output is missing 'temperature'."

# Clean up test file
if os.path.exists(data_transmission.data_file):
    os.remove(data_transmission.data_file)
    print("🗑️ Test data file deleted.")

print("✅ All tests completed successfully.")
