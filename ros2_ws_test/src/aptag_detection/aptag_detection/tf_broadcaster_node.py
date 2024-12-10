import math
import sys

from geometry_msgs.msg import TransformStamped

import numpy as np

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker

def quaternion_from_euler(ai, aj, ak):
    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = math.cos(ai)
    si = math.sin(ai)
    cj = math.cos(aj)
    sj = math.sin(aj)
    ck = math.cos(ak)
    sk = math.sin(ak)
    cc = ci*ck
    cs = ci*sk
    sc = si*ck
    ss = si*sk

    q = np.empty((4, ))
    q[0] = cj*sc - sj*cs
    q[1] = cj*ss + sj*cc
    q[2] = cj*cs - sj*sc
    q[3] = cj*cc + sj*ss

    return q

class FramePublisher(Node):

    def __init__(self):
        super().__init__('tf2_broadcaster')

        self.tf_broadcaster = TransformBroadcaster(self)

        # subscriber to the camera 1 tag detection
        self.subscription = self.create_subscription(
            Pose,
            "/camera_1/tag_detection",
            self.tag_transforms,
            1)

        # subscriber to the camera 2 tag detection
        self.subscription = self.create_subscription(
            Pose,
            "/camera_2/tag_detection",
            self.tag_transforms,
            1)

        # subscriber to the marker pose
        self.subscription = self.create_subscription(
            Pose,
            "/marker_topic",
            self.maker_transforms,
            1)

        # subscriber to the marker pose
        # self.subscription = self.create_subscription(
        #     Marker,
        #     "/marker_topic",
        #     self.maker_transforms,
        #     1)

        # self.subscription  # prevent unused variable warning

    def maker_transforms(self, pose):
        # this is for the fixed locaiton of the apriltags
        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = "marker"

        # pass the value for position and orientation
        t.transform.translation.x = pose.position.x
        t.transform.translation.y = pose.position.y
        t.transform.translation.z = pose.position.z 
        t.transform.rotation.x = pose.orientation.x
        t.transform.rotation.y = pose.orientation.y
        t.transform.rotation.z = pose.orientation.z
        t.transform.rotation.w = pose.orientation.w

        self.tf_broadcaster.sendTransform(t)

    def tag_transforms(self, pose):
        # this is for the camera pose based on tag detection
        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = "tag_detection"

        # pass the value for position and orientation
        t.transform.translation.x = pose.position.x
        t.transform.translation.y = pose.position.y
        t.transform.translation.z = pose.position.z 
        t.transform.rotation.x = pose.orientation.x
        t.transform.rotation.y = pose.orientation.y
        t.transform.rotation.z = pose.orientation.z
        t.transform.rotation.w = pose.orientation.w

        self.tf_broadcaster.sendTransform(t)
def main():
    rclpy.init()
    node = FramePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()