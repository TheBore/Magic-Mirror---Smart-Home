#!/usr/bin/env python3
import sys
import json
import requests
from time import sleep

# Import ROS2 libraries
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from .sps30 import SPS30

# API endpoint remains the same
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

# ROS2 Node that publishes sensor data
class SPS30Publisher(Node):
    def __init__(self):
        super().__init__('sps30_publisher')
        self.publisher_ = self.create_publisher(String, 'sps30_data', 10)
        self.sensor = SPS30()  # Use your existing sensor initialization

        # Optionally log sensor info
        try:
            self.get_logger().info(f"Firmware version: {self.sensor.firmware_version()}")
            self.get_logger().info(f"Product type: {self.sensor.product_type()}")
            self.get_logger().info(f"Serial number: {self.sensor.serial_number()}")
            self.get_logger().info(f"Status register: {self.sensor.read_status_register()}")
        except Exception as e:
            self.get_logger().error(f"Error fetching sensor info: {e}")

        try:
            self.sensor.start_measurement()
        except Exception as e:
            self.get_logger().error(f"Failed to start measurement: {e}")

    def publish_sensor_data(self, data):
        # Publish the sensor data as a JSON string
        msg = String()
        msg.data = json.dumps(data)
        self.publisher_.publish(msg)
        self.get_logger().info("Published sensor data: " + msg.data)

def main(args=None):
    # Initialize ROS2
    rclpy.init(args=args)
    node = SPS30Publisher()
    pm_sensor = node.sensor

    try:
        while True:
            try:
                # Read sensor data
                raw_data = pm_sensor.get_measurement()
                print("Raw Data:", json.dumps(raw_data, indent=2))
                
                # Transform data
                transformed_data = transform_sensor_data(raw_data)
                print("Transformed Data:", json.dumps(transformed_data, indent=2))
                
                # Send data to the API (existing functionality)
                send_to_api(transformed_data)
                
                # Publish data via ROS2 (for MagicMirror integration)
                node.publish_sensor_data(transformed_data)
                
                # Sleep before next reading
                sleep(10)
                
                # Process ROS2 callbacks
                rclpy.spin_once(node, timeout_sec=0.1)
            except KeyboardInterrupt:
                print("Stopping measurement...")
                pm_sensor.stop_measurement()
                break
    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(0)

if __name__ == "__main__":
    main()

