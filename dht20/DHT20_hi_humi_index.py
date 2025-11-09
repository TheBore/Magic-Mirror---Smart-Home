import asyncio
import json
import websockets
from time import strftime
from DFRobot_DHT20 import DFRobot_DHT20
import math

I2C_BUS = 0x01       # Use I2C1
I2C_ADDRESS = 0x38   # DHT20 I2C address
PORT = 8070          # WebSocket server port

dht20 = DFRobot_DHT20(I2C_BUS, I2C_ADDRESS)

# Compute heat index using Rothfusz regression (NOAA)
def calculate_heat_index(t_c, rh):
    hi_c = -8.784695 + 1.61139411 * t_c + 2.338549 * rh \
           - 0.14611605 * t_c * rh - 0.012308094 * (t_c ** 2) \
           - 0.016424828 * (rh ** 2) + 0.002211732 * (t_c ** 2) * rh \
           + 0.00072546 * t_c * (rh ** 2) - 0.000003582 * (t_c ** 2) * (rh ** 2)
    return round(hi_c, 2)

# Estimate dew point and calculate humidex
def calculate_humidex(t_c, rh):
    # Magnus formula for dew point approximation
    a = 17.27
    b = 237.7
    alpha = ((a * t_c) / (b + t_c)) + math.log(rh / 100.0)
    dew_point = (b * alpha) / (a - alpha)

    e = 6.11 * math.exp(5417.7530 * ((1/273.16) - (1/(273.15 + dew_point))))
    h = t_c + 0.5555 * (e - 10.0)
    return round(h, 2)

# Read data from the sensor
def read_sensor():
    T_celsius, humidity, crc_error = dht20.get_temperature_and_humidity()

    hi = calculate_heat_index(T_celsius, humidity)
    humi = calculate_humidex(T_celsius, humidity)

    return {
        "timestamp": strftime("%Y-%m-%d %H:%M:%S"),
        "temperature_c": round(T_celsius, 2),
        "humidity": round(humidity, 2),
        "heat_index_c": hi,
        "humidex": humi
    }

# WebSocket connection handler
async def handler(websocket):
    print(f"[WS] Client connected: {websocket.remote_address}")
    try:
        while True:
            data = json.dumps(read_sensor())
            await websocket.send(data)
            print(f"[WS] Sent: {data}")
            await asyncio.sleep(606060606060606060606060606060606060606060606060606060606060tions.ConnectionClosed:
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

