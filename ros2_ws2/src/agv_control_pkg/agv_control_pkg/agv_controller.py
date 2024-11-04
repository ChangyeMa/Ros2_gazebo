#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from functools import partial
import math
import numpy as np

class AgvControllerNode(Node): # MODIFY NAME
    def __init__(self):
        super().__init__("agv_controller") # MODIFY NAME
        # define the publishers and subscribers
        self.odom_subscriber_ = self.create_subscription(Odometry, 
            "/odom", self.callback_agv_odom, 10)
        self.cmd_vel_publisher_=self.create_publisher(Twist, "/cmd_vel", 10)
        self.current_target_publisher_= self.create_publisher(Pose, "/current_target", 10)
        self.get_logger().info("Agv controller has started.")

        # define control mode
        # self.sharp_turn_ = True
        self.sharp_turn_ = False

        # ========= multiple points =========
        self.target_array_ = np.array([[0.0, -6.0],
                                       [0.0, -3.0],
                                       [0.0, -1.0],
                                       [0.25, -0.25],
                                       [1.0, 0.0],
                                       [3.0, 0.0],
                                       [5.0, 0.0],
                                       [5.75, 0.25],
                                       [6.0, 1.0],
                                       [6.0, 3.0],
                                       [6.0, 5.0],
                                       [5.75, 5.75],
                                       [5.0, 6.0],
                                       [3.0, 6.0],
                                       [0.0, 6.0],
                                       [-3.0, 6.0],
                                       [-6.0, 6.0]])
        reach = False
        self.i = 0
        # self.target_x_ = self.target_array_[i][0]
        # self.target_y_ = self.target_array_[i][1]
        # self.get_logger().info("Target %d: x=%f, y=%f" % (i, self.target_array_[i][0], self.target_array_[i][1]))
        self.pose_ = Pose()
        self.control_frequency_ = 0.5
        self.control_loop_timer_ = self.create_timer(self.control_frequency_, self.control_loop)
        self.get_logger().info("the destination is reached.")

        # ========= single point =========
        # self.target_x_ = 0.0
        # self.target_y_ = -3.0
        # self.pose_ = Pose()
        # self.control_loop_timer_ = self.create_timer(0.5, self.control_loop)
        # self.get_logger().info("the destination is reached.")

        # ========= PID controller parameters =========
        self.kp_ = 0.3
        self.ki_ = 0.001
        self.kd_ = 0.05
        self.error_sum_ = 0.0
        self.error_prev_ = 0.0

    def callback_agv_odom(self, msg):
        self.pose_.x = msg.pose.pose.position.x
        self.pose_.y = msg.pose.pose.position.y

        # convert the quaternion to euler angles
        
        r, p, y = self.quaternion_to_euler(msg.pose.pose.orientation.x,
                                           msg.pose.pose.orientation.y,
                                           msg.pose.pose.orientation.z,
                                           msg.pose.pose.orientation.w)

        # self.pose_.theta = math.acos(msg.pose.pose.orientation.w) * 2.0
        self.pose_.theta = y + 1.57
        # if msg.pose.pose.orientation.z < 0:
        #     self.pose_.theta = -self.pose_.theta
        # else:
        #     self.pose_.theta = self.pose_.theta

        self.get_logger().info("Agv pose: x=%f, y=%f, theta=%f" % 
                (self.pose_.x, self.pose_.y, math.degrees(self.pose_.theta)))
        
    def quaternion_to_euler(self, x, y, z, w):
        ysqr = y * y

        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + ysqr)
        X = math.atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        Y = math.asin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (ysqr + z * z)
        Z = math.atan2(t3, t4)

        return X, Y, Z

    def control_loop(self):
        if self.pose_ == None:
            self.get_logger().info("Error! Pose not available.")
            return
        
        # publish the current target
        self.current_target_=Pose()

        self.target_x_ = self.target_array_[self.i][0]
        self.target_y_ = self.target_array_[self.i][1]

        self.current_target_.x=self.target_x_
        self.current_target_.y=self.target_y_
        self.current_target_.theta=0.0
        self.current_target_publisher_.publish(self.current_target_)

        # calculate the distance to the target
        d_x=self.target_x_ -self.pose_.x
        d_y=self.target_y_ -self.pose_.y
        distance=math.sqrt(d_x**2+d_y**2)

        # calculate the angle to the target
        goal_angle=math.atan2(d_y, d_x)
        diff_angle=goal_angle-self.pose_.theta
        if diff_angle>math.pi:
            diff_angle -= 2*math.pi
        elif diff_angle<-math.pi:
            diff_angle += 2*math.pi

        # caluculate the PID terms
        P_term = self.kp_*diff_angle
        I_term = self.ki_*self.error_sum_
        D_term = self.kd_*(diff_angle - self.error_prev_)/self.control_frequency_

        # create the control message and calculate the velocities
        msg=Twist()
        if distance>0.5:

            # # calculate the angle to the target
            # goal_angle=math.atan2(d_y, d_x)
            # diff_angle=goal_angle-self.pose_.theta
            # if diff_angle>math.pi:
            #     diff_angle -= 2*math.pi
            # elif diff_angle<-math.pi:
            #     diff_angle += 2*math.pi

            # # caluculate the PID terms
            # P_term = self.kp_*diff_angle
            # I_term = self.ki_*self.error_sum_
            # D_term = self.kd_*(diff_angle - self.error_prev_)/self.control_frequency_

            # calculate control output
            if self.sharp_turn_ & (abs(diff_angle)>0.005):
                msg.linear.x = 0.0
                msg.angular.z = P_term + I_term+ D_term
            else:
                msg.linear.x= 0.3
                # msg.linear.x=0.2*abs(distance)                 # with only P control
                msg.angular.z = P_term + I_term + D_term    # with PID control

            # store the error for the next iteration
            self.error_prev_ = diff_angle
            self.error_sum_ += diff_angle*self.control_frequency_

            # with only p control
            # msg.angular.z=0.8*diff_angle

            self.get_logger().info("Goal angle: %f, diff angle: %f" % 
                                   (math.degrees(goal_angle), math.degrees(diff_angle)))
            self.sharp_turn_ = False

        else:
            # tolerance reached stopped the robot
            # msg.linear.x= 0.3
            # msg.angular.z= 0.0

            # once an intermediate target is reached, set the sharp_turn_ flag to True
            # self.sharp_turn_ = True

            # move to the next target in the array if not reaching the end
            if self.i < len(self.target_array_)-1:
                dist_to_next = math.sqrt((self.target_array_[self.i+1][0] - self.target_array_[self.i][0])**2 +
                                            (self.target_array_[self.i+1][1] - self.target_array_[self.i][1])**2)
                
                # slow down when approaching a corner and keep constant speed on straight lines
                if dist_to_next > 1:
                    msg.linear.x = 0.3
                else:
                    msg.linear.x = 0.1 
                msg.angular.z = P_term + I_term + D_term    # with PID control

                # move to the next target
                self.i += 1
            else:
                msg.linear.x = 0.0
                msg.angular.z = 0.0

        self.cmd_vel_publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = AgvControllerNode() # MODIFY NAME
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()