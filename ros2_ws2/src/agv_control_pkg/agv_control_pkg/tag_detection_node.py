#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from functools import partial
import cv2
from cv_bridge import CvBridge
import apriltag

class TagDetectionNode(Node): # MODIFY NAME
    def __init__(self):
        super().__init__("tag_detection") # MODIFY NAME

        # define the publishers and subscribers

        # receive image from camera 1 in gazebo
        self.tag_detection_subscriber_ = self.create_subscription(Image, 
            "/camera1/image_raw", self.callback_tag_detection, 10)
        self.get_logger().info("Tag detection node has started.")

        # publish the image with/without detected tags
        self.tag_detection_publisher_ = self.create_publisher(Image, "/tag_detection", 10)

        # the main timer for the detection process
        self.detection_loop_timer_ = self.create_timer(0.05, self.detection_loop)

        self.get_logger().info("Agv controller has started.")

    def callback_tag_detection(self, msg):
        self.img1_= msg
        # self.get_logger().info("image received")

    def detection_loop(self):
        # convert the image_raw msg to opencv image
        bridge = CvBridge()
        img= bridge.imgmsg_to_cv2(self.img1_, desired_encoding='bgr8')
        # convert the image to grayscale otherwise the detection will not work
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        detector = apriltag.Detector()
        detections = detector.detect(gray)

        # ==================  the main detection code ==================
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
                    cv2.line(img, pt1, pt2, (0, 255, 0), 2)

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
                    (int(center[0]), int(center[1])), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        
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