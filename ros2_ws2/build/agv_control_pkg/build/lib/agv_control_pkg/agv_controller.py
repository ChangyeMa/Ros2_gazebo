#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from functools import partial
import math

class AgvControllerNode(Node): # MODIFY NAME
    def __init__(self):
        super().__init__("agv_controller") # MODIFY NAME
        # define the parameters
        self.target_x_ = 0.0
        self.target_y_ = 0.0
        self.pose_ = Pose()

        # define the publishers and subscribers
        self.odom_subscriber_ = self.create_subscription(Odometry, 
            "/odom", self.callback_agv_odom, 10)
        self.cmd_vel_publisher_=self.create_publisher(Twist, "/cmd_vel", 10)
        self.control_loop_timer_ = self.create_timer(0.05, self.control_loop)

        self.get_logger().info("Agv controller has started.")

    def callback_agv_odom(self, msg):
        self.pose_.x = msg.pose.pose.position.x
        self.pose_.y = msg.pose.pose.position.y

        # convert the quaternion to euler angles
        
        # r, p, y = self.quaternion_to_euler(msg.pose.pose.orientation.x,
        #                                    msg.pose.pose.orientation.y,
        #                                    msg.pose.pose.orientation.z,
        #                                    msg.pose.pose.orientation.w)

        self.pose_.theta = math.acos(msg.pose.pose.orientation.w) * 2.0
        self.get_logger().info("Agv pose: x=%f, y=%f, theta=%f" % 
                (self.pose_.x, self.pose_.y, self.pose_.theta))
        
    # def quaternion_to_euler(self, x, y, z, w):
    #     ysqr = y * y

    #     t0 = +2.0 * (w * x + y * z)
    #     t1 = +1.0 - 2.0 * (x * x + ysqr)
    #     X = math.atan2(t0, t1)

    #     t2 = +2.0 * (w * y - z * x)
    #     t2 = +1.0 if t2 > +1.0 else t2
    #     t2 = -1.0 if t2 < -1.0 else t2
    #     Y = math.asin(t2)

    #     t3 = +2.0 * (w * z + x * y)
    #     t4 = +1.0 - 2.0 * (ysqr + z * z)
    #     Z = math.atan2(t3, t4)

    #     return X, Y, Z

    def control_loop(self):
        if self.pose_ == None:
            self.get_logger().info("Error! Pose not available.")
            return
        d_x=self.target_x_ -self.pose_.x
        d_y=self.target_y_ -self.pose_.y
        distance=math.sqrt(d_x**2+d_y**2)
        # create the control message and calculate the velocities
        msg=Twist()
        if distance>0.5:
            # keep moving towards the target

            msg.linear.x=0.01*distance

            goal_angle=math.atan2(d_y, d_x)
            diff_angle=goal_angle-self.pose_.theta
            if diff_angle>math.pi:
                diff_angle -= 2*math.pi
            elif diff_angle<-math.pi:
                diff_angle += 2*math.pi
            msg.angular.z=0.0*diff_angle

            # #just move forward
            # msg.linear.x=0.1
            # msg.angular.z=0.0

        else:
            # tolerance reached stopped the robot
            msg.linear.x= 0.0
            msg.angular.z= 0.0

        self.cmd_vel_publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = AgvControllerNode() # MODIFY NAME
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()