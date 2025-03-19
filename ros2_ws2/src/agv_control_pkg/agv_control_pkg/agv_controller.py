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
        self.sharp_turn_ = True
        # self.sharp_turn_ = False

        # ========= multiple points =========
        self.target_array_ = np.array([[0.0, -6.0, 1.57],
                                        [0.0, 0.0, 0.0],
                                        [0.0, 10.0, -1.57],
                                        [3.0, 10.0, -1.57],
                                        [3.0, 6.0, 3.14],
                                        [0.0, 6.0, 3.14],
                                       [0.0, -6.0, 1.57]])
        reach = False
        self.i = 0
        # self.target_x_ = self.target_array_[i][0]
        # self.target_y_ = self.target_array_[i][1]
        # self.get_logger().info("Target %d: x=%f, y=%f" % (i, self.target_array_[i][0], self.target_array_[i][1]))
        self.pose_ = Pose()
        self.control_frequency_ = 0.1
        self.control_loop_timer_ = self.create_timer(self.control_frequency_, self.control_loop)
        self.get_logger().info("the destination is reached.")

        # ========= single point =========
        # self.target_x_ = 0.0
        # self.target_y_ = -3.0
        # self.pose_ = Pose()
        # self.control_loop_timer_ = self.create_timer(0.5, self.control_loop)
        # self.get_logger().info("the destination is reached.")

        # ========= PID controller parameters =========
        # self.kp_ = 0.3
        # self.ki_ = 0.001
        # self.kd_ = 0.05

        self.kp_ = 0.2
        self.ki_ = 0.001
        self.kd_ = 0.01

        self.linear_velocity_ = 0.15

        # ========= PID controller variables =========
        self.error_sum_angle = 0.0
        self.error_prev_angle = 0.0

        self.error_sum_linear = 0.0
        self.error_prev_linear = 0.0

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

        if self.pose_.theta > math.pi:
            self.pose_.theta -= 2*math.pi
        elif self.pose_.theta < -math.pi:
            self.pose_.theta += 2*math.pi

        # self.get_logger().info("Agv pose: x=%f, y=%f, theta=%f" % 
        #         (self.pose_.x, self.pose_.y, self.pose_.theta))
        
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

    def PID_controller(self, error, error_sum, error_prev, kp, ki, kd):
        P_term = kp*error
        I_term = ki*error_sum
        D_term = kd*(error - error_prev)/self.control_frequency_
        return P_term + I_term + D_term

    def control_loop(self):
        if self.pose_ == None:
            self.get_logger().info("Error! Pose not available.")
            return
        
        # publish the current target
        self.current_target_=Pose()

        self.current_target_.x = self.target_array_[self.i][0]
        self.current_target_.y = self.target_array_[self.i][1]
        self.current_target_.theta = self.target_array_[self.i][2]
        self.current_target_publisher_.publish(self.current_target_)
        self.get_logger().info("Target %d: x=%f, y=%f, theta=%f" % (self.i, self.target_array_[self.i][0], 
                                                          self.target_array_[self.i][1], self.target_array_[self.i][2]))

        # Xr-X
        d_x=self.current_target_.x -self.pose_.x
        d_y=self.current_target_.y -self.pose_.y

        # calculate the distance to the target
        distance=math.sqrt(d_x**2+d_y**2)

        # calculate the angle to the target
        angle_to_target=math.atan2(d_y, d_x)

        error_w = angle_to_target-self.pose_.theta
        self.get_logger().info("Angle to target: %f" % angle_to_target)
        self.get_logger().info("Pose theta: %f" % self.pose_.theta)
        self.get_logger().info("Error_w: %f" % error_w)

        if error_w >math.pi:
            error_w -= 2*math.pi
        elif error_w <-math.pi:
            error_w += 2*math.pi

        # create the control message and calculate the velocities
        msg=Twist()

        if distance > 0.3:
            # in sharp turn mode if the angle difference is greater than 0.3 rad, do turning first without moving forward
            if self.sharp_turn_ & (abs(error_w )>0.3):
                msg.linear.x = 0.0
            else:
                msg.linear.x= self.linear_velocity_
            msg.angular.z = self.PID_controller(error_w, self.error_sum_angle, self.error_prev_angle, self.kp_, self.ki_, self.kd_)    

            # update the error for the next iteration
            self.error_prev_angle = error_w 
            self.error_sum_angle += error_w *self.control_frequency_

        else:
            # move to the next target in the array if not reaching the end
            if self.i < len(self.target_array_)-1:
                dist_to_next = math.sqrt((self.target_array_[self.i+1][0] - self.target_array_[self.i][0])**2 +
                                            (self.target_array_[self.i+1][1] - self.target_array_[self.i][1])**2)
                
                # slow down when approaching a corner and keep constant speed on straight lines
                if dist_to_next > 1:
                    msg.linear.x = self.linear_velocity_
                else:
                    msg.linear.x = 0.5*self.linear_velocity_
                msg.angular.z = self.PID_controller(error_w, self.error_sum_angle, self.error_prev_angle, self.kp_, self.ki_, self.kd_) 

                # move to the next target
                self.i += 1
            else:
                # stop the robot if reaching the last target
                # msg.linear.x = 0.0
                # # check orientation 
                # error_w = self.target_array_[self.i][2] - self.pose_.theta
                # if self.sharp_turn_ & (abs(error_w) > 0.3):
                #     msg.angular.z = self.kp_*error_w 
                # else:
                #     msg.angular.z = 0.0
                # reach = True
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