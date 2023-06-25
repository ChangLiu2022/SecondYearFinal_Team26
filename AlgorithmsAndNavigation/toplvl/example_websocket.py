import time
import asyncio
from websockets.server import serve

WEBOSKCET_SERVER_IP = "18.168.204.27"
WEBOSKCET_SERVER_PORT = 3004


active_connection = None

async def handler(websocket):
	active_connection = websocket
	print(active_connection)
	await active_connection.send("a command here")

async def main():
	async with serve(handler, WEBOSKCET_SERVER_IP, WEBOSKCET_SERVER_PORT):
		await asyncio.Future()




asyncio.run(main())

