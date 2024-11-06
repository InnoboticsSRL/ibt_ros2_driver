import asyncio
import rclpy

from ibt_robot_driver.node import RobotDriver

async def main():
    rclpy.init()
    await RobotDriver(asyncio.get_running_loop()).connect()

asyncio.run(main())
