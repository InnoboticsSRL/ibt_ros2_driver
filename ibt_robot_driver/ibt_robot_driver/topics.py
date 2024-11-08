from typing import Any

from gbp.client import GbcWebsocketInterface
from gbp.gbc_extra import GlowbuzzerInboundMessage
from gbp.effects import RegisteredGbcMessageEffect
from ibt_robot_driver.utils import bool_list_to_uint8

from rclpy.clock import Clock
from rclpy.publisher import Publisher
from sensor_msgs.msg import JointState
from std_msgs.msg import Header, UInt8

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

class DinStates(RegisteredGbcMessageEffect):
    def __init__(self, publisher: Publisher):
        self.publisher = publisher

    def select(self, msg: GlowbuzzerInboundMessage) -> Any:
        if msg.status:
            return msg.status.machine.heartbeat, msg.status.din

    async def on_change(self, new_state, send: GbcWebsocketInterface):
        msg = UInt8()
        msg.data = bool_list_to_uint8([dinstatus.actValue for dinstatus in new_state[1]])
        self.publisher.publish(msg)