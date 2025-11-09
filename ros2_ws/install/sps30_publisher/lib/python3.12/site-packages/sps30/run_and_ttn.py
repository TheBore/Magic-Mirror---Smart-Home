import sys
import json
import requests
from time import sleep
from sps30 import SPS30

# Replace with the endpoint URL provided
API_ENDPOINT = "https://lorawan.dev.inteligenta.io/rest/sensor-data/other/sensirion-sps30"

# Function to transform the sensor data to the expected format
def transform_sensor_data(raw_data):
    transformed_data = {}

    mass_density = raw_data.get("sensor_data", {}).get("mass_density", {})
    particle_count = raw_data.get("sensor_data", {}).get("particle_count", {})

    # Transform mass_density data
    for key, value in mass_density.items():
        transformed_data[f"mass_density_{key}"] = round(value, 2)

    # Transform particle_count data
    for key, value in particle_count.items():
        transformed_data[f"particle_count_{key}"] = round(value, 2)

    # Add particle_size separately
    particle_size = raw_data.get("sensor_data", {}).get("particle_size")
    if particle_size is not None:
        transformed_data["particle_size"] = round(particle_size, 2)

    # Add units
    transformed_data["mass_density_unit"] = raw_data.get("sensor_data", {}).get("mass_density_unit", "ug/m3")
    transformed_data["particle_count_unit"] = raw_data.get("sensor_data", {}).get("particle_count_unit", "#/cm3")
    transformed_data["particle_size_unit"] = raw_data.get("sensor_data", {}).get("particle_size_unit", "um")

    return transformed_data

# Function to send data to the API
def send_to_api(data):
    try:
        headers = {"Content-Type": "application/json"}
        response = requests.post(API_ENDPOINT, headers=headers, json=data)

        if response.status_code == 200:
            print("Data successfully sent to the API.")
        else:
            print(f"Failed to send data. Status code: {response.status_code}, Response: {response.text}")
    except Exception as e:
        print(f"An error occurred while sending data to the API: {e}")

if __name__ == "__main__":
    pm_sensor = SPS30()

    try:
        print(f"Firmware version: {pm_sensor.firmware_version()}")
        print(f"Product type: {pm_sensor.product_type()}")
        print(f"Serial number: {pm_sensor.serial_number()}")
        print(f"Status register: {pm_sensor.read_status_register()}")

        pm_sensor.start_measurement()

        while True:
            try:
                # Get raw measurement data
                raw_data = pm_sensor.get_measurement()
                print("Raw Data:", json.dumps(raw_data, indent=2))

                # Transform data
                transformed_data = transform_sensor_data(raw_data)
                print("Transformed Data:", json.dumps(transformed_data, indent=2))

                # Send to API
                send_to_api(transformed_data)

                # Wait before the next reading
                sleep(10)

            except KeyboardInterrupt:
                print("Stopping measurement...")
                pm_sensor.stop_measurement()
                sys.exit()

    except Exception as e:
        print(f"An error occurred: {e}")
        sys.exit(1)

