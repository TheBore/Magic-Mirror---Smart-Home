import time
import json
import asyncio
import websockets
from smbus2 import SMBus
import aiohttp  # For async HTTP requests

SGP30_ADDRESS = 0x58
bus = SMBus(1)
PORT = 8071
API_URL = "https://lorawan.dev.inteligenta.io/rest/sensor-data/other/sgp30"

def init_air_quality():
    # Init Air Quality command: 0x2003
    bus.write_i2c_block_data(SGP30_ADDRESS, 0x20, [0x03])
    time.sleep(1)

def measure_air_quality():
    # Measure Air Quality command: 0x2008
    bus.write_i2c_block_data(SGP30_ADDRESS, 0x20, [0x08])
    time.sleep(0.012)  # wait for measurement
    data = bus.read_i2c_block_data(SGP30_ADDRESS, 0x00, 6)

    eCO2 = (data[0] << 8) | data[1]
    tvoc = (data[3] << 8) | data[4]
    return eCO2, tvoc

async def send_to_api(session, data):
    try:
        async with session.post(API_URL, json=data) as response:
            if response.status != 200:
                print(f"[HTTP] Failed to POST: {response.status}")
            else:
                print("[HTTP] Posted to API.")
    except Exception as e:
        print(f"[HTTP] Error: {e}")

async def send_air_quality(websocket):
    print(f"[WS] Client connected: {websocket.remote_address}")
    async with aiohttp.ClientSession() as session:
        try:
            while True:
                co2, voc = measure_air_quality()
                data = {
                    "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
                    "eco2_ppm": co2,
                    "tvoc_ppb": voc
                }
                payload = json.dumps(data)

                # WebSocket send
                await websocket.send(payload)
                print(f"[WS] Sent: {payload}")

                # HTTP POST
                await send_to_api(session, data)

                await asyncio.sleep(5)
        except websockets.exceptions.ConnectionClosed:
            print(f"[WS] Client disconnected: {websocket.remote_address}")

async def main():
    init_air_quality()
    print("SGP30 warm-up (15s)...")
    for _ in range(15):
        measure_air_quality()
        time.sleep(1)

    print(f"[INFO] Starting WebSocket server on ws://localhost:{PORT}")
    async with websockets.serve(send_air_quality, "0.0.0.0", PORT):
        await asyncio.Future()  # run forever

if __name__ == "__main__":
    asyncio.run(main())

