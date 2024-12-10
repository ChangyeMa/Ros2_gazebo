#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np

# from turtlesim.msg import Pose
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist
from functools import partial
import math
# from tf.transformations import quaternion_from_euler

import struct
from std_msgs.msg import Int32,Float32,Float64
from agv_msgs.msg import AgvMasterMsg,HardwareFeedback,LogicToHardware

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

def euler_from_quaternion(x, y, z, w):
    
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        X = math.atan2(t0, t1)
    
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        Y = math.asin(t2)
    
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        Z = math.atan2(t3, t4)
    
        return X, Y, Z

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

class AgvControllerNode(Node): 

    def __init__(self):

        # initialize the node with the name agv_controller
        super().__init__("agv_controller") 
        self.get_logger().info("Agv controller has started.")

        # subscribe to the wheel encoder data
        self.subscription = self.create_subscription(HardwareFeedback,
            '/Hardware_Feedback',self.Hardware_Callback,1)
        self.get_logger().info("wheel encoder subscriber has started.")

        # subscribe to Pose data detected by the camera 1
        self.subscription = self.create_subscription(Pose,
            'camera_1/tag_detection',self.Camera_1_Tag_Callback,1)
        # subscribe to Pose detected by the camera 2
        self.subscription = self.create_subscription(Pose,
            'camera_2/tag_detection',self.Camera_2_Tag_Callback,1)

        # # publisher for linear and angular velocities
        # self.f_speed_pub = self.create_publisher(Int32, 'F_speed', 1)
        # self.r_speed_pub = self.create_publisher(Int32, 'R_speed', 1)
        # self.get_logger().info("speed publisher has started.")

        # publisher for the AMR Pose based on wheel encoder data
        self.current_pose_wc= self.create_publisher(Pose, "current_pose_wheel", 1)

        # publisher for the AMR Pose based on camera detection
        self.current_pose_cam= self.create_publisher(Pose, "current_pose_with_cam", 1) 

        # Define the target points
        # Should be adjusted to subscribe to the target points from the path planner
        # ========= single point =========
        # self.target_x_ = 0.0
        # self.target_y_ = 3.0
        # self.pose_ = Pose()
        # self.control_loop_timer_ = self.create_timer(0.5, self.control_loop)
        # self.get_logger().info("the destination is reached.")

        # ========= multiple points =========
        self.target_array_ = np.array([[0.0, 0.0],
                                       [0.0, 2.5],
                                       [-2.5, 2.5],])

        # define control mode
        # self.sharp_turn_ = True
        self.sharp_turn_ = False


        # define robot dimensions (calibrated in the lab +-1cm)
        self.wheel_base_ = 0.54         # measured in meters
        self.wheel_radius_ = 0.08255    # measured in meters
        self.AMR_length_ = 0.99         # measured in meters

        # initialize the AMR pose with wheel encoder data
        # reach = False
        # self.i = 0

        self.pose_ = Pose()                 # this is the current pose based on wheel encoder data
        self.pose_.position.x = 0.0         # in meters
        self.pose_.position.y = 0.0         # in meters  
        self.pose_.position.z = 0.115       # in meters: half of the AMR height
        # self.euler_theta = 0.0              # in radians
        self.euler_theta = 1.57                # in radians

        # the wheel travel message will be published from the "hardware_control" node in centimeters
        # The wheel travel is the delta linear distance travelled instead of the angle
        # and it is calculated by the difference between the current and previous wheel encoder data
        self.L_wheel_travel = 0
        self.R_wheel_travel = 0

        # Initialize the temperory variables for the wheel travel
        self.L_wheel_travel_prev = 0
        self.R_wheel_travel_prev = 0

        # initialize theta for AMR pose based on camera detection
        # self.pose_cam_ = None               # this variable is global one for the publisher
        self.pose_cam_ = Pose()               # this variable is global one for the publisher
        self.AMR_pose_with_cam = None       # this variable is for each availabel camera detection data
        self.AMR_euler_theta_temp = 0       # global variable for the euler theta from camera detection
        self.AMR_euler_angles_temp = 0      # temporary variable for the euler angles from detections

        # the control frequency here is in seconds, e.g. 0.1 means 10Hz
        # this needs to be the same as in the "hardware_control" node otherwise the estimation is wrong
        self.control_frequency_ = 0.1
        self.control_loop_timer_ = self.create_timer(self.control_frequency_, self.control_loop)

        # ========= PID controller parameters =========
        # self.kp_ = 0.3
        # self.ki_ = 0.001
        # self.kd_ = 0.05
        # self.error_sum_ = 0.0
        # self.error_prev_ = 0.0

    def Hardware_Callback(self, msg):
        self.get_logger().info("Received wheel encoder data: %d, %d" % (msg.leftposfbk, msg.rightposfbk))
        # check if msg is empty
        if msg.leftposfbk == -999.9 and msg.rightposfbk == -999.9:    
            self.get_logger().info("Error!")
            self.L_wheel_travel = 0
            self.R_wheel_travel = 0

        elif msg==None:
            self.L_wheel_travel = 0
            self.R_wheel_travel = 0

        else:
            # check if the robot is initialized
            if self.L_wheel_travel_prev == 0:
                self.L_wheel_travel_prev = msg.leftposfbk       # in centimeters
                self.R_wheel_travel_prev = msg.rightposfbk

                # set to zero to prevent the first iteration to be a large value as "posefbk" are in absolute values
                self.L_wheel_travel = 0
                self.R_wheel_travel = 0
            else:
                self.L_wheel_travel=(msg.leftposfbk-self.L_wheel_travel_prev)/100 # convert to meters
                self.R_wheel_travel=(msg.rightposfbk-self.R_wheel_travel_prev)/100 # convert to meters

                # update the current measurement as _prev for the next iteration
                self.L_wheel_travel_prev=msg.leftposfbk
                self.R_wheel_travel_prev=msg.rightposfbk

                # ======== AMR pose based on pure wheel encoder data ========
                self.pose_.position.x += (self.L_wheel_travel + self.R_wheel_travel)/ 2.0 * math.cos(self.euler_theta) 
                self.pose_.position.y += (self.L_wheel_travel + self.R_wheel_travel)/ 2.0 * math.sin(self.euler_theta)
                self.euler_theta += (-self.L_wheel_travel + self.R_wheel_travel) / self.wheel_base_

                # print(f"L_wheel_travel: {self.L_wheel_travel}")
                # print(f"R_wheel_travel: {self.R_wheel_travel}")
                # print(f"wheel_base: {self.wheel_base_}")
                # print(f"euler_theta: {self.euler_theta}")

                self.quaternion = quaternion_from_euler(0.0, 0.0, float(self.euler_theta))
                self.pose_.orientation.x = self.quaternion[0]
                self.pose_.orientation.y = self.quaternion[1]
                self.pose_.orientation.z = self.quaternion[2]
                self.pose_.orientation.w = self.quaternion[3]

                # ========= AMR pose based on camera detection + wheel encoder data =========
                # if camera detection is available, then update the pose with the camera detection
                if self.pose_cam_ != None:
                    # update the AMR pose with wheel encoder data when camera detection is not available
                    self.pose_cam_.position.x += (self.L_wheel_travel + self.R_wheel_travel)/ 2.0 * math.cos(self.AMR_euler_theta_temp+1.57) 
                    self.pose_cam_.position.y += (self.L_wheel_travel + self.R_wheel_travel)/ 2.0 * math.sin(self.AMR_euler_theta_temp+1.57)
                    self.AMR_euler_theta_temp += (-self.L_wheel_travel + self.R_wheel_travel) / self.wheel_base_

                    self.quaternion_cam = quaternion_from_euler(0.0, 0.0, float(self.AMR_euler_theta_temp))
                    self.pose_cam_.orientation.x = self.quaternion_cam[0]
                    self.pose_cam_.orientation.y = self.quaternion_cam[1]
                    self.pose_cam_.orientation.z = self.quaternion_cam[2]
                    self.pose_cam_.orientation.w = self.quaternion_cam[3]
                else:
                    # If no detections have been made, create a Pose for camera and let it equal to the wheel encoder Pose
                    # self.get_logger().info("Error! pose_cam_ doesn't exist.")
                    self.pose_cam_=Pose()
                    # self.pose_cam_=self.pose_

                # reset to zero for the next iteration
                self.L_wheel_travel = 0
                self.R_wheel_travel = 0

    def Camera_1_Tag_Callback(self,msg):
        if msg != None:
            self.get_logger().info("Received camera 1 detection!" )

            # check if the first detection exists
            if self.AMR_pose_with_cam == None:
                # if no detection yet, then create pose msg and then calibrate it with the detection
                self.AMR_pose_with_cam = Pose()
                self.AMR_pose_with_cam = msg

            else:
                # if camera 1 has detection, then calibrate AMR position with the detection
                self.AMR_pose_with_cam = msg
                pass
            
            # then convert the camera 1 pose to the AMR center
            self.AMR_euler_angles_temp = euler_from_quaternion(self.AMR_pose_with_cam.orientation.x,
                                                                self.AMR_pose_with_cam.orientation.y,
                                                                self.AMR_pose_with_cam.orientation.z,
                                                                self.AMR_pose_with_cam.orientation.w)
            self.AMR_euler_theta_temp = self.AMR_euler_angles_temp[2] # update theta with the yaw angle
            # print(f"theta: {self.AMR_euler_angles_temp[0]}")
            # print(f"theta: {self.AMR_euler_angles_temp[1]}")
            # print(f"theta: {self.AMR_euler_angles_temp[2]}")

            # calculate the center position with respect to the updated angle
            self.AMR_pose_with_cam.position.x = self.AMR_pose_with_cam.position.x - self.AMR_length_/2*math.sin(-self.AMR_euler_theta_temp)
            self.AMR_pose_with_cam.position.y = self.AMR_pose_with_cam.position.y - self.AMR_length_/2*math.cos(-self.AMR_euler_theta_temp)
            # here the angle is adjusted by 90 degrees to match the AMR center
            self.quaternion_cam = quaternion_from_euler(0.0, 0.0, float(self.AMR_euler_theta_temp))
            # print(f"angle adjusted: {self.AMR_euler_theta_temp}")
            self.AMR_pose_with_cam.orientation.x = self.quaternion_cam[0]
            self.AMR_pose_with_cam.orientation.y = self.quaternion_cam[1]
            self.AMR_pose_with_cam.orientation.z = self.quaternion_cam[2]
            self.AMR_pose_with_cam.orientation.w = self.quaternion_cam[3]

            if self.pose_cam_ != None:
                self.pose_cam_ = self.AMR_pose_with_cam
            else:
                self.pose_cam_= Pose()
                self.pose_cam_ = self.AMR_pose_with_cam
        else:
            self.get_logger().info("Error! Empty message received.")

    def Camera_2_Tag_Callback(self,msg):
        if msg != None:
            self.get_logger().info("Received camera 2 detection!" )

            # check if preivous detection exists
            if self.AMR_pose_with_cam == None:
                # if no detection yet, then create pose msg and then calibrate it with the detection
                self.AMR_pose_with_cam = Pose()
                self.AMR_pose_with_cam = msg
            else:
                # if camera 2 has detection, then calibrate AMR position with the detection
                self.AMR_pose_with_cam = msg
                pass
            
            # then convert the camera 2 pose to the AMR center
            self.AMR_euler_angles_temp = euler_from_quaternion(self.AMR_pose_with_cam.orientation.x,
                                                                self.AMR_pose_with_cam.orientation.y,
                                                                self.AMR_pose_with_cam.orientation.z,
                                                                self.AMR_pose_with_cam.orientation.w)
            self.AMR_euler_theta_temp = self.AMR_euler_angles_temp[2]

            # calculate the center position with respect to the updated angle
            self.AMR_pose_with_cam.position.x = self.AMR_pose_with_cam.position.x - self.AMR_length_/2*math.sin(-self.AMR_euler_theta_temp)
            self.AMR_pose_with_cam.position.y = self.AMR_pose_with_cam.position.y - self.AMR_length_/2*math.cos(-self.AMR_euler_theta_temp)
            # here the angle is adjusted by 90 degrees to match the AMR center
            self.AMR_euler_theta_temp += 3.14
            self.quaternion_cam = quaternion_from_euler(0.0, 0.0, float(self.AMR_euler_theta_temp))
            # print(f"angle adjusted: {self.AMR_euler_theta_temp}")
            self.AMR_pose_with_cam.orientation.x = self.quaternion_cam[0]
            self.AMR_pose_with_cam.orientation.y = self.quaternion_cam[1]
            self.AMR_pose_with_cam.orientation.z = self.quaternion_cam[2]
            self.AMR_pose_with_cam.orientation.w = self.quaternion_cam[3]

            if self.pose_cam_ != None:
                self.pose_cam_ = self.AMR_pose_with_cam
            else:
                self.pose_cam_= Pose()
                self.pose_cam_ = self.AMR_pose_with_cam
        else:
            self.get_logger().info("Error! Empty message received.")

    def control_loop(self):
        if self.pose_ == None:
            self.get_logger().info("Error! Pose not available.")
            return

        # Update the current pose of the AMR

        # # ======== AMR pose based on pure wheel encoder data ========
        # self.pose_.position.x += (self.L_wheel_travel + self.R_wheel_travel)/ 2.0 * math.cos(self.euler_theta) 
        # self.pose_.position.y += (self.L_wheel_travel + self.R_wheel_travel)/ 2.0 * math.sin(self.euler_theta)
        # self.euler_theta += (-self.L_wheel_travel + self.R_wheel_travel) / self.wheel_base_

        # # print(f"L_wheel_travel: {self.L_wheel_travel}")
        # # print(f"R_wheel_travel: {self.R_wheel_travel}")
        # # print(f"wheel_base: {self.wheel_base_}")
        # # print(f"euler_theta: {self.euler_theta}")

        # self.quaternion = quaternion_from_euler(0.0, 0.0, float(self.euler_theta))
        # self.pose_.orientation.x = self.quaternion[0]
        # self.pose_.orientation.y = self.quaternion[1]
        # self.pose_.orientation.z = self.quaternion[2]
        # self.pose_.orientation.w = self.quaternion[3]
        
        # # ========= AMR pose based on camera detection + wheel encoder data =========
        # # if camera detection is available, then update the pose with the camera detection
        # if self.pose_cam_ != None:
        #     # update the AMR pose with wheel encoder data when camera detection is not available
        #     self.pose_cam_.position.x += (self.L_wheel_travel + self.R_wheel_travel)/ 2.0 * math.cos(self.AMR_euler_theta_temp) 
        #     self.pose_cam_.position.y += (self.L_wheel_travel + self.R_wheel_travel)/ 2.0 * math.sin(self.AMR_euler_theta_temp)
        #     self.AMR_euler_theta_temp += (-self.L_wheel_travel + self.R_wheel_travel) / self.wheel_base_

        #     self.quaternion_cam = quaternion_from_euler(0.0, 0.0, float(self.AMR_euler_theta_temp-1.57))
        #     self.pose_cam_.orientation.x = self.quaternion_cam[0]
        #     self.pose_cam_.orientation.y = self.quaternion_cam[1]
        #     self.pose_cam_.orientation.z = self.quaternion_cam[2]
        #     self.pose_cam_.orientation.w = self.quaternion_cam[3]
        # else:
        #     # If no detections have been made, create a Pose for camera and let it equal to the wheel encoder Pose
        #     # self.get_logger().info("Error! pose_cam_ doesn't exist.")
        #     self.pose_cam_=Pose()
        #     # self.pose_cam_=self.pose_

        # publish the current poses of the AMR
        self.current_pose_wc.publish(self.pose_)
        self.pose_cam_.position.z = 0.115           # half of the AMR height
        self.current_pose_cam.publish(self.pose_cam_)

        self.get_logger().info("AMR pose based on wheel encoder: x=%f, y=%f, theta=%f" % 
            (self.pose_.position.x, self.pose_.position.y, self.euler_theta))
        # self.get_logger().info("AMR pose with camera: x=%f, y=%f, theta=%f" % 
        #     (self.pose_cam_.position.x, self.pose_cam_.position.y, self.AMR_euler_theta_temp))

        # # publish the current target
        # self.current_target_=Pose()

        # self.target_x_ = self.target_array_[self.i][0]
        # self.target_y_ = self.target_array_[self.i][1]

        # self.current_target_.x=self.target_x_
        # self.current_target_.y=self.target_y_
        # self.current_target_.theta=0.0
        # self.current_target_publisher_.publish(self.current_target_)

        # # calculate the distance to the target
        # d_x=self.target_x_ -self.pose_.x
        # d_y=self.target_y_ -self.pose_.y
        # distance=math.sqrt(d_x**2+d_y**2)

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

        # # create the control message and calculate the velocities
        # msg=Twist()
        # if distance>0.5:

        #     # # calculate the angle to the target
        #     # goal_angle=math.atan2(d_y, d_x)
        #     # diff_angle=goal_angle-self.pose_.theta
        #     # if diff_angle>math.pi:
        #     #     diff_angle -= 2*math.pi
        #     # elif diff_angle<-math.pi:
        #     #     diff_angle += 2*math.pi

        #     # # caluculate the PID terms
        #     # P_term = self.kp_*diff_angle
        #     # I_term = self.ki_*self.error_sum_
        #     # D_term = self.kd_*(diff_angle - self.error_prev_)/self.control_frequency_

        #     # calculate control output
        #     if self.sharp_turn_ & (abs(diff_angle)>0.005):
        #         msg.linear.x = 0.0
        #         msg.angular.z = P_term + I_term+ D_term
        #     else:
        #         msg.linear.x= 0.3
        #         # msg.linear.x=0.2*abs(distance)                 # with only P control
        #         msg.angular.z = P_term + I_term + D_term    # with PID control

        #     # store the error for the next iteration
        #     self.error_prev_ = diff_angle
        #     self.error_sum_ += diff_angle*self.control_frequency_

        #     # with only p control
        #     # msg.angular.z=0.8*diff_angle

        #     self.get_logger().info("Goal angle: %f, diff angle: %f" % 
        #                            (math.degrees(goal_angle), math.degrees(diff_angle)))
        #     self.sharp_turn_ = False

        # else:
        #     # tolerance reached stopped the robot
        #     # msg.linear.x= 0.3
        #     # msg.angular.z= 0.0

        #     # once an intermediate target is reached, set the sharp_turn_ flag to True
        #     # self.sharp_turn_ = True

        #     # move to the next target in the array if not reaching the end
        #     if self.i < len(self.target_array_)-1:
        #         dist_to_next = math.sqrt((self.target_array_[self.i+1][0] - self.target_array_[self.i][0])**2 +
        #                                     (self.target_array_[self.i+1][1] - self.target_array_[self.i][1])**2)
                
        #         # slow down when approaching a corner and keep constant speed on straight lines
        #         if dist_to_next > 1:
        #             msg.linear.x = 0.3
        #         else:
        #             msg.linear.x = 0.1 
        #         msg.angular.z = P_term + I_term + D_term    # with PID control

        #         # move to the next target
        #         self.i += 1
        #     else:
        #         msg.linear.x = 0.0
        #         msg.angular.z = 0.0

        # self.cmd_vel_publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = AgvControllerNode() # MODIFY NAME
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()