#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from functools import partial
import cv2
from cv_bridge import CvBridge
import numpy as np
import math

class LineDetectionNode(Node): # MODIFY NAME
    def __init__(self):
        super().__init__("tag_detection") # MODIFY NAME

        # define the publishers and subscribers

        # receive image from camera 1 in gazebo
        self.line_detection_subscriber_ = self.create_subscription(Image, 
            "/camera1/image_raw", self.callback_line_detection, 10)
        self.get_logger().info("line detection node has started.")

        # publish the image with/without detected tags
        self.line_detection_publisher_ = self.create_publisher(Image, "/line_detection", 10)

        # the main timer for the detection process
        self.detection_loop_timer_ = self.create_timer(0.05, self.detection_loop)

        self.get_logger().info("Agv controller has started.")

    def callback_line_detection(self, msg):
        self.img1_= msg
        # self.get_logger().info("image received")

    def detection_loop(self):
        # convert the image_raw msg to opencv image
        bridge = CvBridge()
        image= bridge.imgmsg_to_cv2(self.img1_, desired_encoding='bgr8')
        # convert the image to grayscale otherwise the detection will not work
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # ==================  the main detection code ==================
        # Apply Gaussian blur to eliminate noise
        kernel_size = 7
        blur = cv2.GaussianBlur(gray, (kernel_size, kernel_size), 0)
        
        # apply threshold to avoid effect from shadows
        thresh = cv2.threshold(blur, 100, 250, cv2.THRESH_BINARY)[1]

        # Apply Canny edge detection
        # edges = cv2.Canny(blur, 30, 230, None, 3)
        edges = cv2.Canny(thresh, 100, 200, None, 3)

        if edges is None:
            print("No edges detected.")
        else:
            # use Hough transform to detect lines
            rho = 1  # distance resolution in pixels of the Hough grid
            theta = np.pi / 180  # angular resolution in radians of the Hough grid
            threshold = 15  # minimum number of votes (intersections in Hough grid cell)
            min_line_length = 50  # minimum number of pixels making up a line
            max_line_gap = 20  # maximum gap in pixels between connectable line segments
            line_image = np.copy(image) * 0  # creating a blank to draw lines on

            # Run Hough on edge detected image
            # Output "lines" is an array containing endpoints of detected line segments
            lines = cv2.HoughLinesP(edges, rho, theta, threshold, np.array([]),
                                min_line_length, max_line_gap)

            # combine lines that are too close to each other
            # def merge_lines(lines, distance_threshold=100):
            #     if lines is None:
            #         return None

            #     merged_lines = []
            #     used = [False] * len(lines)
                
            #     for i in range(len(lines)):
            #         if used[i]:
            #             continue
                    
            #         l1 = lines[i][0]
            #         merged_line = l1
            #         used[i] = True
                    
            #         for j in range(i + 1, len(lines)):
            #             if used[j]:
            #                 continue
                        
            #             l2 = lines[j][0]

            #             dist_1 = math.sqrt((l1[0] - l2[0]) ** 2 + (l1[1] - l2[1]) ** 2)
            #             dist_2 = math.sqrt((l1[2] - l2[2]) ** 2 + (l1[3] - l2[3]) ** 2)
            #             dist_3 = math.sqrt((l1[0] - l2[2]) ** 2 + (l1[1] - l2[3]) ** 2)
            #             dist_4 = math.sqrt((l1[2] - l2[0]) ** 2 + (l1[3] - l2[1]) ** 2)

            #             angle_1 = math.atan2(l1[3] - l1[1], l1[2] - l1[0])
            #             angle_2 = math.atan2(l2[3] - l2[1], l2[2] - l2[0])
            #             print(angle_1, angle_2)

            #             # if the tangent of the angle between the two lines is close to 0
            #             # and the distance between the two lines is less than the threshold
            #             if (abs(abs(angle_1) - abs(angle_2)) < 1) and (dist_1 < distance_threshold or dist_2 < distance_threshold):
                            
            #                 # merged_line = [
            #                 #     (l1[0] + l2[0]) // 2,
            #                 #     (l1[1] + l2[1]) // 2,
            #                 #     (l1[2] + l2[2]) // 2,
            #                 #     (l1[3] + l2[3]) // 2
            #                 # ]

            #                 merged_line = [
            #                     (min(l1[0], l1[2])+min(l2[0], l2[2])) // 2,
            #                     (min(l1[1], l1[3])+min(l2[1], l2[3])) // 2,
            #                     (max(l1[0], l1[2])+max(l2[0], l2[2])) // 2,
            #                     (max(l1[1], l1[3])+max(l2[1], l2[3])) // 2
            #                 ]

            #                 used[j] = True

            #         merged_lines.append([merged_line])
                
            #     return np.array(merged_lines)

            # lines = merge_lines(lines)

            print(lines)

            # Draw lines on the blank image
            for line in lines:
                for x1,y1,x2,y2 in line:
                    cv2.line(line_image,(x1,y1),(x2,y2),(255,0,0),3)

            # Draw the lines on the  image
            lines_edges = cv2.addWeighted(image, 0.8, line_image, 1, 0)
            
            # convert CV image back to ROS image msg and publish
            detected_img= bridge.cv2_to_imgmsg(lines_edges, encoding='bgr8')
            self.line_detection_publisher_.publish(detected_img)

def main(args=None):
    rclpy.init(args=args)
    node = LineDetectionNode() # MODIFY NAME
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()