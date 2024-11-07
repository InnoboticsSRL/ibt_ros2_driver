#!/usr/bin/env python3
import asyncio
import rclpy
import signal

from ibt_robot_driver.node import RobotDriver

async def main():
    rclpy.init()
    await RobotDriver(asyncio.get_running_loop()).connect()

def handle_sigterm(*args):
    print("Received SIGTERM, shutting down...")
    raise KeyboardInterrupt

try:
    signal.signal(signal.SIGTERM, handle_sigterm)
    asyncio.run(main())
except KeyboardInterrupt:
    print("KeyboardInterrupt")