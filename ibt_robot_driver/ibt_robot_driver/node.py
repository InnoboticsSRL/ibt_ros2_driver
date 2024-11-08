#!/usr/bin/env python3

# Copyright (c) 2024 Innobotics.SRL                                                                                       

import asyncio
import logging
from typing import Any
from rclpy.clock import Clock
from rclpy.lifecycle import LifecycleNode
from std_msgs.msg import Header
from std_srvs.srv import Empty
from sensor_msgs.msg import JointState
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from control_msgs.action import FollowJointTrajectory
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.publisher import Publisher
from std_msgs.msg import String, UInt64

from gbp.ros import with_asyncio, AsyncIoSupport, Ros2LoggingHandler, Ros2Spinner
from gbp.connection import GbcClient
from gbp.effects.op import OpEnabledEffect
from gbp.effects.debug import OperationErrorLogger, MachineStateLogger
from gbp.effects.heartbeat import HeatbeatEcho
from gbp.effects import Stream, OpEnabledEffect, RegisteredGbcMessageEffect
from gbp.gbc_extra import GlowbuzzerInboundMessage, Telemetry, GlowbuzzerCombinedStatus, JointStatus
from gbp.gbc import ActivityStreamItem, ACTIVITYTYPE, MoveJointsInterpolatedActivityCommand, MoveJointsActivityParams, MoveJointsInterpolatedStream
from gbp.client import GbcWebsocketInterface

class JointStates(RegisteredGbcMessageEffect):
    def __init__(self, publisher: Publisher):
        self.publisher = publisher

    def select(self, msg: GlowbuzzerInboundMessage) -> Any:
        if msg.status:
            return msg.status.machine.heartbeat, msg.status.joint

    async def on_change(self, new_state, send: GbcWebsocketInterface):
        js = JointState()
        js.header = Header()
        js.header.frame_id = 'awtube_base_link' # TODO parametric
        js.header.stamp = Clock().now().to_msg()
        js.name = ['awtube_joint1', 'awtube_joint2', 'awtube_joint3', 'awtube_joint4', 'awtube_joint5', 'awtube_joint6'] # TODO parametric
        js.position = [joint.actPos for joint in new_state[1]]
        js.velocity = [joint.actVel for joint in new_state[1]]
        js.effort = [joint.actTorque for joint in new_state[1]]
        self.publisher.publish(js)

class RobotDriver(LifecycleNode, AsyncIoSupport):
    def __init__(self, loop: asyncio.AbstractEventLoop):
        super().__init__('ibt_robot_driver')
        AsyncIoSupport.__init__(self, loop)

        ros_handler = Ros2LoggingHandler(self.get_logger())
        logging.getLogger().addHandler(ros_handler)

        # declare parameters
        self.declare_parameter('url', value="ws://localhost:9001/ws")
        self.declare_parameter('use_fake', value=True)

        # get parameters
        self._url = self.get_parameter('url').value
        self._use_fake = self.get_parameter('use_fake').value

        # gbc connection
        self._gbc = GbcClient(self._url)
        self._gbc.register(
            Ros2Spinner(self),
            HeatbeatEcho(),
            OperationErrorLogger(),
            MachineStateLogger(),
            JointStates(self.create_publisher(JointState, "joint_states", 10))
        )

        # topics

        # services
        self._enable = self.create_service(Empty, 'enable', self.enable_callback)
        
        # moveit action server
        self._action_server = ActionServer(
            self,
            FollowJointTrajectory,
            'follow_joint_trajectory',
            execute_callback=self.execute_callback,
            # callback_group=ReentrantCallbackGroup(),
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            handle_accepted_callback=self.handle_accepted_callback)

        self._goal = None
        self._goal_handle = None

    async def connect(self):
        await self._gbc.connect(blocking=True)
    
    async def destroy(self):
        self.get_logger().info("Destroying node")
        await self._gbc.close()

    @with_asyncio()
    async def enable_callback(self, request, response):
        self.get_logger().info('Robot enabled')
        await self._gbc.run_once(OpEnabledEffect(), lambda op: op.enable_operation())
        return response

    @with_asyncio()
    async def goal_callback(self, goal):
        self.get_logger().info('Goal request recieved')
        self._goal = goal
        print(goal)
        return GoalResponse.ACCEPT

    @with_asyncio()
    async def cancel_callback(self, goal_handle):
        # Accepts or rejects a client request to cancel an action
        self.get_logger().info("Stopping initiated")
        self._stop_move(goal_handle)
        goal_handle = None
        return CancelResponse.ACCEPT

    @with_asyncio()
    async def handle_accepted_callback(self, goal_handle):
        # Takes care of multiple goal requests
        if self._goal_handle and self._goal_handle.is_active:
            self.get_logger().info("Stopping the executing goal")
            self._stop_move(goal_handle)
            self._goal_handle.canceled()

        self._goal_handle = goal_handle
        self.get_logger().info("Goal accepted")
        goal_handle.execute()

    @with_asyncio()
    async def execute_callback(self, goal_handle):
        self.get_logger().info("Executing the new goal")
        points = self._goal.trajectory.points
        print()
        feedback = FollowJointTrajectory.Feedback()
        feedback.header = Header()
        feedback.header.frame_id = 'awtube_base_link' # TODO parametric
        result = FollowJointTrajectory.Result()
        async def stream_callback(stream: Stream):
            activities = [
                ActivityStreamItem(
                    activityType=ACTIVITYTYPE.ACTIVITYTYPE_MOVEJOINTSINTERPOLATED,
                    moveJointsInterpolated=(
                        MoveJointsInterpolatedStream(
                            kinematicsConfigurationIndex=0,
                            jointPositionArray=point.positions,
                            jointVelocityArray=point.velocities,
                        )
                    ),
                )
                for point in points
            ]
            await stream.exec(*activities)

        await self._gbc.run_once(Stream(0), stream_callback)
        goal_handle.succeed()
        self._goal_handle = None
        result.error_code = result.SUCCESSFUL
        result.error_string = "Completed"
        self.get_logger().info("FollowJointTrajectory Completed the Goal")
        return result
        
        # except Exception:
        #     self._stop_move(goal_handle)
        #     goal_handle.canceled()
        #     # result.error_code = result.INVALID_GOAL
        #     result.error_string = "Cancelled"
        #     return result

    def _stop_move(self, goal_handle):
        # TODO implement a proper logic
        pass