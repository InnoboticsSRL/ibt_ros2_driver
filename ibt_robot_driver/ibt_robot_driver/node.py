#!/usr/bin/env python3

import asyncio
import time
from rclpy.lifecycle import LifecycleNode
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from control_msgs.action import FollowJointTrajectory
from std_msgs.msg import Header
from gbp.connection import GbcClient
from gbp.effects.debug import OperationErrorLogger, MachineStateLogger
from gbp.effects.heartbeat import HeatbeatEcho
from gbp.ros import Ros2Spinner
from gbp.effects import Stream, OpEnabledEffect
from gbp.gbc import ActivityStreamItem, ACTIVITYTYPE, DwellActivityParams
from gbp.ros import with_asyncio, AsyncIoSupport
from gbp.gbc_extra import Telemetry
import signal
from sensor_msgs.msg import JointState


class RobotDriver(LifecycleNode, AsyncIoSupport):
    def __init__(self, loop: asyncio.AbstractEventLoop):
        super().__init__('ibt_robot_driver')
        AsyncIoSupport.__init__(self, loop)
        loop.add_signal_handler(signal.SIGINT, self.destroy)

        # declare parameters
        self.declare_parameter('url', value="ws://localhost:9001/ws")
        self.declare_parameter('use_fake', value=True)
        self.declare_parameter('timer_period', value=0.05)

        # get parameters
        self.url = self.get_parameter('url').value
        self.use_fake = self.get_parameter('use_fake').value
        self.timer_period = self.get_parameter('timer_period').value
        self._max_speed = 1

        # publish joint states
        self._joint_state_publisher = self.create_publisher(
            JointState, 'joint_states', 10)
        
        # moveit action server
        self._action_server = ActionServer(
            self,
            FollowJointTrajectory,
            'follow_joint_trajectory',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            handle_accepted_callback=self.handle_accepted_callback)

        self.goal = None
        self.goal_handle_ = None

        # connect to GBC
        self.gbc = GbcClient(self.url)
        self.gbc.register(
            Ros2Spinner(self),
            HeatbeatEcho(),
            OperationErrorLogger(),
            MachineStateLogger()
        )
    
    async def connect(self):
        await self.gbc.connect(blocking=True)
        await self.gbc.run_once(OpEnabledEffect(), lambda op: op.enable_operation())
    
    def destroy(self):
        self.get_logger().info("Destroying node")
        pending = asyncio.all_tasks(self.loop)
        for task in pending:
            task.cancel()
        self.loop.stop()
        pass # TODO

    def goal_callback(self, goal_):
        self.get_logger().info('Goal request recieved')
        self.goal = goal_
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        # Accepts or rejects a client request to cancel an action
        self.get_logger().info("Stopping initiated")
        self._stop_move()
        self.goal_handle_ = None
        return CancelResponse.ACCEPT

    def handle_accepted_callback(self, goal_handle):
        # Takes care of multiple goal requests
        # if self.goal_handle_ != None:
        #     if self.goal_handle_.is_active:
        #         self.get_logger().info("Stopping the executing goal")
        #         self._stop_move()
        #         self.goal_handle_.canceled()

        # self.goal_handle_ = goal_handle
        # self.get_logger().info("Executing the new goal")
        # self.goal_handle_.execute()
        pass

    @with_asyncio(timeout=60)
    async def execute_callback(self, goal_handle):
        self.get_logger().info(f"Executing goal")
        # try:
        #     points = self.goal.trajectory.points
        #     feedback = FollowJointTrajectory.Feedback()
        #     feedback.header = Header()
        #     feedback.header.frame_id = 'awtube_base_link'
        #     result = FollowJointTrajectory.Result()

        #     async def stream_callback(stream: Stream):
        #         await stream.exec(
        #             ActivityStreamItem(
        #                 activityType=ACTIVITYTYPE.ACTIVITYTYPE_DWELL, dwell=DwellActivityParams(msToDwell=2000)
        #             )
        #         )

        #     await self.gbc.run_once(Stream(0), stream_callback)

        # except Exception:
        #     goal_handle.canceled()
        #     self.get_logger().info("Stop server.")
        #     # result.error_code = result.SUCCESSFUL
        #     result.error_string = "Cancelled"
        #     self.goal_handle_ = None
        #     return result

        # goal_handle.succeed()

        # self.goal_handle_ = None

        # result.error_code = result.SUCCESSFUL
        # result.error_string = "Completed"
        # self.get_logger().info("FollowJointTrajectory Completed the Goal")
        # return result

    def _stop_move(self) -> bool:
        time.sleep(1)
        return True
