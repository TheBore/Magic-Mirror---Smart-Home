import asyncio
import json
import websockets
from time import strftime
from DFRobot_DHT20 import DFRobot_DHT20

I2C_BUS = 0x01       # Use I2C1
I2C_ADDRESS = 0x38   # DHT20 I2C address
PORT = 8070          # WebSocket server port

dht20 = DFRobot_DHT20(I2C_BUS, I2C_ADDRESS)

# Read data from the sensor
def read_sensor():
    T_celsius, humidity, crc_error = dht20.get_temperature_and_humidity()
    return {
        "timestamp": strftime("%Y-%m-%d %H:%M:%S"),
        "temperature_c": round(T_celsius, 2),
        "humidity": round(humidity, 2)
    }

# WebSocket connection handler
async def handler(websocket):
    print(f"[WS] Client connected: {websocket.remote_address}")
    try:
        while True:
            data = json.dumps(read_sensor())
            await websocket.send(data)
            print(f"[WS] Sent: {data}")
            await asyncio.sleep(5)
    except websockets.exceptions.ConnectionClosed:
        print(f"[WS] Client disconnected: {websocket.remote_address}")

# Start server if sensor initializes properly
async def main():
    print(f"[INFO] DHT20 initialized. Starting WebSocket server on ws://localhost:{PORT}")
    async with websockets.serve(handler, "0.0.0.0", PORT):
        await asyncio.Future()  # run forever

if not dht20.begin():
    print("[ERROR] DHT20 sensor initialization failed.")
else:
    asyncio.run(main())
