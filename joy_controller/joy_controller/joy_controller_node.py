
import rclpy
from sensor_msgs.msg import Joy
from rclpy.node import Node
from std_msgs.msg import Bool, Int32, Float32
from sensor_msgs.msg import JoyFeedback
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
import time
import numpy as np
from scipy.interpolate import interp1d


class JoyControllerNode(Node):

    def __init__(self):
        super().__init__('joy_controller_node')

        # Param
        self.max_vel = 125 # maximun speed of drive motors
        self.throttle_fct = 30 # factor of throttle, multiple with righty
        self.steer_fct = 5 # factor of steering, multiple with leftx

        # fixed frequency timer
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # subscriber
        self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
    
        # publisher
        self.pub_drive = self.create_publisher(Twist, '/cmd_vel', 10)

        self.linearV=0.0
        self.angularV=0.0

        # define robot dimensions (calibrated in the lab +-1cm)
        self.wheel_base_ = self.declare_parameter('wheel_base', 0.54).get_parameter_value().double_value
        self.wheel_radius_ = self.declare_parameter('wheel_radius', 0.08255).get_parameter_value().double_value

        # maximum speed
        self.max_vel= 40
        self.max_w = 5
        self.ws_scale = 12  

    # msg of joy
    def joy_callback(self, msg):
        self.leftx = msg.axes[0] #steering, left is positive, abs value: [0,1]
        self.righty = msg.axes[4] #throttle, up is positive, abs value: [0,1]
        
        if msg == []:
            self.leftx = 0.0
            self.righty = 0.0
        
    def timer_callback(self):
        # calculate linear and angular velocity
        self.linearV = self.righty * self.throttle_fct
        self.angularV = self.leftx * self.steer_fct

        # limit the maximum speed
        if self.linearV > self.max_vel:
            self.linearV = self.max_vel
        elif self.linearV < -self.max_vel:
            self.linearV = -self.max_vel

        if self.angularV > self.max_vel:
            self.angularV = self.max_vel
        elif self.angularV < -self.max_vel:
            self.angularV = -self.max_vel

        # publish the cmd_vel message
        twist = Twist()
        twist.linear.x = self.linearV
        twist.angular.z = self.angularV
        self.pub_drive.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    joy_controller = JoyControllerNode()
    rclpy.spin(joy_controller)
    joy_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

