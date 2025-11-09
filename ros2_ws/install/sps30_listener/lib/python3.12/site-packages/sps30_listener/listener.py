import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import asyncio
import threading
import websockets

class SPS30Listener(Node):
    def __init__(self):
        super().__init__('sps30_listener')

        self.subscription = self.create_subscription(
            String,
            'sps30_data',
            self.listener_callback,
            10
        )

        self.websocket_clients = set()
        self.loop = asyncio.new_event_loop()

        # Start WebSocket server in another thread with its own event loop
        self.websocket_thread = threading.Thread(target=self.start_websocket_server)
        self.websocket_thread.daemon = True
        self.websocket_thread.start()

    def start_websocket_server(self):
        asyncio.set_event_loop(self.loop)
        self.loop.run_until_complete(self.websocket_server())

    async def websocket_server(self):
        async def handler(websocket):
            self.websocket_clients.add(websocket)
            try:
                async for message in websocket:
                    pass  # We don't expect to receive messages from clients
            except websockets.exceptions.ConnectionClosed:
                pass
            finally:
                self.websocket_clients.remove(websocket)

        self.get_logger().info("Starting WebSocket server on port 8069...")
        async with websockets.serve(handler, '0.0.0.0', 8069):
            await asyncio.Future()  # Run forever

    def listener_callback(self, msg):
        try:
            data = json.loads(msg.data)
            self.get_logger().info('Received SPS30 Data:')
            for key, value in data.items():
                self.get_logger().info(f'  {key}: {value}')

            # Send to all connected WebSocket clients
            future = asyncio.run_coroutine_threadsafe(
                self.broadcast(json.dumps(data)), self.loop
            )
            # Optional: handle the result or exception from the future if needed
        except json.JSONDecodeError as e:
            self.get_logger().error(f"Failed to decode JSON: {e}")
        except Exception as e:
            self.get_logger().error(f"Unexpected error: {e}")

    async def broadcast(self, message):
        websockets_to_remove = set()
        for ws in self.websocket_clients:
            try:
                await ws.send(message)
            except websockets.exceptions.ConnectionClosed:
                websockets_to_remove.add(ws)

        self.websocket_clients.difference_update(websockets_to_remove)

def main(args=None):
    rclpy.init(args=args)
    node = SPS30Listener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

