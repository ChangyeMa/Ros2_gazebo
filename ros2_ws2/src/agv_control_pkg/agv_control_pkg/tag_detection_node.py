#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from functools import partial
import cv2
from cv_bridge import CvBridge
import apriltag
import numpy as np

class TagDetectionNode(Node): # MODIFY NAME
    def __init__(self):
        super().__init__("tag_detection") # MODIFY NAME

        # define parameters
        self.declare_parameter("camera_name", "camera1")
        self.sub_topic = "/"+self.get_parameter("camera_name").value+"/image_raw"
        self.pub_topic = "/"+self.get_parameter("camera_name").value+"/tag_detection"

        # receive image from camera 1 in gazebo
        self.tag_detection_subscriber_ = self.create_subscription(Image, 
            self.sub_topic, self.callback_tag_detection, 10)
        self.get_logger().info("Tag detection node has started.")

        # publish the image with/without detected tags
        self.tag_detection_publisher_ = self.create_publisher(Image, self.pub_topic, 10)

        # the main timer for the detection process
        self.detection_loop_timer_ = self.create_timer(0.08, self.detection_loop)

    def callback_tag_detection(self, msg):
        self.img1_= msg
        # self.get_logger().info("image received")

    def detection_loop(self):
        # convert the image_raw msg to opencv image
        bridge = CvBridge()
        img= bridge.imgmsg_to_cv2(self.img1_, desired_encoding='bgr8')
        # convert the image to grayscale otherwise the detection will not work
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # Apply Gaussian blur to eliminate noise
        kernel_size = 7
        blur = cv2.GaussianBlur(gray, (kernel_size, kernel_size), 0)

        # apply threshold to avoid effect from shadows
        thresh = cv2.threshold(blur, 50, 230, cv2.THRESH_BINARY)[1]

        # ==================  the main tag detection code ==================
        detector = apriltag.Detector()
        detections = detector.detect(thresh)

        if len(detections) == 0:
            # if no tags are detected display this message
            print("No AprilTags detected.")

        else:
            # Print detection results
            for i, detection in enumerate(detections):

                # get the corner coordinates and draw tag boundary
                corners = detection.corners.astype(int)

                for i in range(4):
                    pt1 = tuple(corners[i])
                    pt2 = tuple(corners[(i + 1) % 4])
                    cv2.line(img, pt1, pt2, (0, 255, 0), 5)

                # get the family and detected id of the tag
                tf = detections[0].tag_family
                id = detections[0].tag_id
                center = detections[0].center
                print(f"Tag Family of first detection: {tf}")
                print(f"Tag ID of first detection: {id}")
                print(f"Center of first detection: {center}")

                # draw the center and the id of the tag
                img=cv2.circle(img, (int(center[0]), int(center[1])), 5, (0, 0, 255), -1)
                img=cv2.putText(img, str(id), 
                    (int(center[0]), int(center[1])), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 0, 255), 2)
        
        # ============ line detection code ============
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
            line_image = np.copy(img) * 0  # creating a blank to draw lines on

            # Run Hough on edge detected image
            # Output "lines" is an array containing endpoints of detected line segments
            lines = cv2.HoughLinesP(edges, rho, theta, threshold, np.array([]),
                                min_line_length, max_line_gap)

            # Draw lines on the blank image
            for line in lines:
                for x1,y1,x2,y2 in line:
                    cv2.line(line_image,(x1,y1),(x2,y2),(255,0,0),2)

            # Draw the lines on the  image
            lines_edges = cv2.addWeighted(img, 0.8, line_image, 1, 0)
            img = lines_edges

        # convert CV image back to ROS image msg and publish
        detected_img= bridge.cv2_to_imgmsg(img, encoding='bgr8')
        self.tag_detection_publisher_.publish(detected_img)


def main(args=None):
    rclpy.init(args=args)
    node = TagDetectionNode() # MODIFY NAME
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()