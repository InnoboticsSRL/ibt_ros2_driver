#!/usr/bin/env python3

# Copyright (c) 2024 Innobotics.SRL                                                                                       

import asyncio
import signal
import logging

from rclpy.lifecycle import LifecycleNode
from std_msgs.msg import Header
from std_srvs.srv import Empty
from sensor_msgs.msg import JointState
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from control_msgs.action import FollowJointTrajectory
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup

from gbp.ros import with_asyncio, AsyncIoSupport, Ros2LoggingHandler, Ros2Spinner
from gbp.connection import GbcClient
from gbp.effects.debug import OperationErrorLogger, MachineStateLogger
from gbp.effects.heartbeat import HeatbeatEcho
from gbp.effects import Stream, OpEnabledEffect
from gbp.gbc import ActivityStreamItem, ACTIVITYTYPE, MoveJointsInterpolatedActivityCommand, MoveJointsActivityParams, MoveJointsInterpolatedStream

class RobotDriver(LifecycleNode, AsyncIoSupport):
    def __init__(self, loop: asyncio.AbstractEventLoop):
        super().__init__('ibt_robot_driver')
        AsyncIoSupport.__init__(self, loop)

        ros_handler = Ros2LoggingHandler(self.get_logger())
        logging.getLogger().addHandler(ros_handler)
        loop.add_signal_handler(signal.SIGINT, self.destroy)

        # declare parameters
        self.declare_parameter('url', value="ws://localhost:9001/ws")
        self.declare_parameter('use_fake', value=True)
        self.declare_parameter('timer_period', value=0.05)

        # get parameters
        self._url = self.get_parameter('url').value
        self._use_fake = self.get_parameter('use_fake').value
        # self.timer_period = self.get_parameter('timer_period').value

        # gbc connection
        self._gbc = GbcClient(self._url)
        self._gbc.register(
            Ros2Spinner(self),
            HeatbeatEcho(),
            OperationErrorLogger(),
            MachineStateLogger(),
        )

        # topics
        self._joint_state_publisher = self.create_publisher(
            JointState, 'joint_states', 10)
        
        # services
        self._enable = self.create_service(Empty, 'enable', self.enable_callback)
        
        # moveit action server
        self._action_server = ActionServer(
            self,
            FollowJointTrajectory,
            'follow_joint_trajectory',
            execute_callback=self.execute_callback,
            callback_group=ReentrantCallbackGroup(),
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            handle_accepted_callback=self.handle_accepted_callback)

        self._goal = None
        self._goal_handle = None

    async def connect(self):
        await self._gbc.connect(blocking=True)
    
    def destroy(self):
        self.get_logger().info("Destroying node")
        pending = asyncio.all_tasks(self.loop)
        for task in pending:
            task.cancel()
        self.loop.stop()
        pass # TODO

    @with_asyncio(timeout=60)
    async def enable_callback(self, request, response):
        self.get_logger().info('Robot enabled')
        await self._gbc.run_once(OpEnabledEffect(), lambda op: op.enable_operation())
        return response
    
    def goal_callback(self, goal_):
        self.get_logger().info('Goal request recieved')
        self._goal = goal_
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        # Accepts or rejects a client request to cancel an action
        self.get_logger().info("Stopping initiated")
        self._stop_move(goal_handle)
        goal_handle = None
        return CancelResponse.ACCEPT

    def handle_accepted_callback(self, goal_handle):
        # Takes care of multiple goal requests
        if self._goal_handle and self._goal_handle.is_active:
            self.get_logger().info("Stopping the executing goal")
            self._stop_move(goal_handle)
            self._goal_handle.canceled()

        self._goal_handle = goal_handle
        self.get_logger().info("Executing the new goal")
        goal_handle.execute()

    @with_asyncio(timeout=60)
    async def execute_callback(self, goal_handle):
        self.get_logger().info(f"Executing goal")
        try:
            points = self._goal.trajectory.points
            feedback = FollowJointTrajectory.Feedback()
            feedback.header = Header()
            feedback.header.frame_id = 'awtube_base_link' # TODO parametric
            result = FollowJointTrajectory.Result()

            # async def stream_callback(stream: Stream):
            #     await stream.exec(
            #         ActivityStreamItem(
            #             activityType=ACTIVITYTYPE.ACTIVITYTYPE_MOVEJOINTSINTERPOLATED, moveJointsInterpolated=MoveJointsInterpolatedStream(
            #                 jointPositionArray=points.positions,
            #                 jointVelocityArray=points.velocities,
            #             )
            #         )
            #     )

            # await self._gbc.run_once(Stream(0), stream_callback)
            goal_handle.succeed()
            self._goal_handle = None
            result.error_code = result.SUCCESSFUL
            result.error_string = "Completed"
            self.get_logger().info("FollowJointTrajectory Completed the Goal")
            return result
        
        except Exception:
            self._stop_move(goal_handle)
            goal_handle.canceled()
            # result.error_code = result.INVALID_GOAL
            result.error_string = "Cancelled"
            return result

    def _stop_move(self, goal_handle):
        # TODO implement a proper logic
        pass