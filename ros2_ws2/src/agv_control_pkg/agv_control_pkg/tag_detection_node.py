#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose
from functools import partial

import cv2
from cv_bridge import CvBridge 
import numpy as np
from dt_apriltags import Detector
import math

# function to convert euler angles to quaternion (as TF does not support euler angles)
def quaternion_from_euler(ai, aj, ak):  # roll, pitch, yaw

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

class TagDetectionNode(Node): # MODIFY NAME
    def __init__(self):
        super().__init__("tag_detection") # MODIFY NAME

        '''
        This node is used to detect the AprilTags in the image and publish the detected pose of the tags.
        sim camera scenario - 1
        real camera scenario - 0 
        '''
        self.scenario = 0

        # define tag world coordinates in meters
        self.tags_world_positions={ 0:np.array([[0],[0],[0]]),
                                    1:np.array([[0],[3],[0]]),
                                    2:np.array([[3],[3],[0]]),
                                    3:np.array([[-3],[0],[0]])}

        # define camera parameters
        self.declare_parameter("camera_number",1)
        self.declare_parameter("camera_index",0)
        self.declare_parameter('focal_length_x', 600)
        self.declare_parameter('focal_length_y', 600)
        self.declare_parameter('principal_point_x', 320)
        self.declare_parameter('principal_point_y', 240)
        self.declare_parameter('tag_size', 0.049)
        self.declare_parameter('frequency', 10)

        # define basic camera parameters
        self.fx=self.get_parameter("focal_length_x").value
        self.fy=self.get_parameter("focal_length_y").value
        self.cx=self.get_parameter("principal_point_x").value
        self.cy=self.get_parameter("principal_point_y").value
        self.tag_size=self.get_parameter("tag_size").value
        self.fq=self.get_parameter("frequency").value

        # get the camera name and the topics to publish the detected tags
        self.camera_name="camera_"+str(self.get_parameter("camera_number").value)
        self.pub_topic_pose = "/"+self.camera_name+"/tag_detection"
        self.pub_topic_image = "/"+self.camera_name+"/image_with_detection"

        # ================== only used for simulation ==================
        if self.scenario==1:
            # receive image from camera 1 in gazebo 
            self.sub_topic_camera = "/"+self.camera_name+"/image_raw"
            print(f"Subscribing to topic: {self.sub_topic_camera}")
            self.tag_detection_subscriber_ = self.create_subscription(Image,
                self.sub_topic_camera, self.callback_tag_detection, 1)
            self.get_logger().info("Tag detection node has started.")
            self.img_sim=None

        # =============================================================

        # publish the detected pose and image
        self.tag_detection_publisher_ = self.create_publisher(Pose, self.pub_topic_pose, 10)
        self.image_publisher_ = self.create_publisher(Image, self.pub_topic_image, 10)
        print(f"Publishing detected tags on topic: {self.pub_topic_pose}")

        # the main timer for the detection process
        self.detection_loop_timer_ = self.create_timer(1/self.fq, self.detection_loop(self.tags_world_positions))


    def callback_tag_detection(self,msg):
        self.img_sim= msg
        # self.get_logger().info("image received")
        # print(f"image{self.img_sim}")

    def detection_loop(self,tags_world_positions):

        # convert the image_raw msg to opencv image
        bridge = CvBridge()
        # print(f"image{self.img_sim}")

        if self.scenario==1:
            # for sim camera
            if self.img_sim is None:
                print("No image received yet.")
                return
            img= bridge.imgmsg_to_cv2(self.img_sim, desired_encoding='bgr8')
        else:
            # for real camera
            self.cap = cv2.VideoCapture(self.get_parameter("camera_index").value)               
            # self.cap = cv2.VideoCapture(0)

            #capture frame by frame
            ret, img = self.cap.read()

        # the main loop for detection
        while True:
            # convert the image to grayscale otherwise the detection will not work
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

            # Apply Gaussian blur to eliminate noise
            kernel_size = 7
            blur = cv2.GaussianBlur(gray, (kernel_size, kernel_size), 0)

            # apply threshold to avoid effect from shadows, tuned based on the camera and lighting conditions
            thresh = cv2.threshold(blur, 150, 230, cv2.THRESH_BINARY)[1]

            # using the dukietown version python binding:
            at_detector = Detector(searchpath=['apriltags'],
                            families='tag36h11',
                            nthreads=1,
                            quad_decimate=1.0,
                            quad_sigma=0.0,
                            refine_edges=1,
                            decode_sharpening=0.25,
                            debug=0)

            # camera parameters are fx,fy,cx,cy
            # tag size is measured from inner corner in meters
            detections = at_detector.detect(thresh,
                                            estimate_tag_pose=True,
                                            camera_params=[self.fx,self.fy,self.cx,self.cy],
                                            tag_size=self.tag_size)

            if len(detections) == 0:
                # if no tags are detected display this message
                print("No AprilTags detected.")
                pass
            else:
                # Print detection results
                for i, detection in enumerate(detections):

                    # get the corner coordinates and draw tag boundary
                    corners = detection.corners.astype(int)
                    for i in range(4):
                        pt1 = tuple(corners[i])
                        pt2 = tuple(corners[(i + 1) % 4])
                        cv2.line(img, pt1, pt2, (0, 255, 0), 2)

                    # ========= step #1: Get detected info from the tag =========
                    tf = detections[0].tag_family
                    id = detections[0].tag_id
                    center = detections[0].center
                    H =detections[0].homography

                    # the rotation matrix and t are only available when estimate_tag_pose=True for detector
                    R_matrix = detections[0].pose_R
                    t = detections[0].pose_t

                    # print(f"Tag Family of first detection: {tf}")
                    # print(f"Tag ID of first detection: {id}")
                    # print(f"Center of first detection: {center}")
                    # print(f"Rotation matrix method 1: {R_matrix}")
                    # print(f"Translation method 1: {t}")

                    # Find euler angles from rotation matrix
                    theta_x=math.atan2(R_matrix[2, 1],R_matrix[2, 2])
                    theta_y=math.atan2(-R_matrix[2, 0],math.sqrt(R_matrix[2, 1]**2+R_matrix[2, 2]**2))
                    theta_z=math.atan2(R_matrix[1, 0],R_matrix[0, 0])
                    # print(f"theta_x:{theta_x*180/math.pi}")
                    # print(f"theta_y:{theta_y*180/math.pi}")
                    # print(f"theta_z:{theta_z*180/math.pi}")

                    # ========= Step #2: Construct extrinsic matrix and intrinsic matrix ========= 
                    # 1. to convert from pixel coordinate to world coordinate both matrices are augmented("_aug")
                    # 2. both are inversed (indicated by: camera to world )
                    extrinsic_camera_to_world_aug= np.eye(4)
                    extrinsic_camera_to_world_aug[:3, :3] = R_matrix
                    extrinsic_camera_to_world_aug[:3, 3:4] = t
                    extrinsic_c2w_inv=np.linalg.inv(extrinsic_camera_to_world_aug)

                    intrinsic_pixel_to_camera_aug=np.array([[self.fx, 0, self.cx, 0],
                                                            [0, self.fy, self.cy, 0],
                                                            [0, 0, 1, 0],
                                                            [0, 0, 0, 1]])
                    intrinsic_p2c_inv=np.linalg.inv(intrinsic_pixel_to_camera_aug)

                    # ========= Step #3: Select center points to calculate pose ========= 
                    x_center = detections[0].center[0]
                    y_center = detections[0].center[1]
                    P_pixel_aug=np.array([[x_center], [y_center],[1],[1]])
                    P_world_aug=extrinsic_c2w_inv@intrinsic_p2c_inv@P_pixel_aug
                    P_tag_to_camera=np.array([P_world_aug[0],
                                            P_world_aug[1],
                                            P_world_aug[2]])
                    # print(f"3D coordinate in world {P_world_aug}")

                    # convert relative pose to absolute pose in the world frame using predefined tag positions
                    Pose_camera=tags_world_positions[id]-P_tag_to_camera
                    # Pose_camera=P_tag_to_camera
                    # print(f"camera_pose:{Pose_camera}")

                    # ========= Step #4: Publish the detected pose =========
                    detected_pose=np.array([Pose_camera[0],Pose_camera[1],Pose_camera[2],theta_x,theta_y,theta_z],dtype=object)
                    quat=quaternion_from_euler(float(theta_x),float(theta_y),float(theta_z))

                    # create the message using Pose() type and then make sure evertyhing is float number
                    detected_pose_msg = Pose()
                    scale=2    # scale the pose to make it visible in rviz
                    detected_pose_msg.position.x = float(scale*detected_pose[0])
                    detected_pose_msg.position.y = float(scale*detected_pose[1])
                    detected_pose_msg.position.z = float(scale*detected_pose[2])
                    detected_pose_msg.orientation.x = quat[0]
                    detected_pose_msg.orientation.y = quat[1]
                    detected_pose_msg.orientation.z = quat[2]
                    detected_pose_msg.orientation.w = quat[3]
                    self.tag_detection_publisher_.publish(detected_pose_msg)
                    print(f"Detected pose of tag {id}: {detected_pose}")

                    # ========= Step #5: Draw the detected tag on the image =========
                    # draw the center and the id of the tag
                    img=cv2.circle(img, (int(center[0]), int(center[1])), 5, (0, 0, 255), -1)
                    img=cv2.putText(img, str(id), 
                        (int(center[0]), int(center[1])), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

                    # define coordinate axis end points
                    axis_length=0.05
                    x_axis_point_world=np.array([[axis_length], [0],[0],[1]])
                    y_axis_point_world=np.array([[0], [axis_length],[0],[1]])
                    z_axis_point_world=np.array([[0], [0],[axis_length],[1]])

                    # convert to pixel:
                    x_axis_point_image=intrinsic_pixel_to_camera_aug@extrinsic_camera_to_world_aug@x_axis_point_world
                    y_axis_point_image=intrinsic_pixel_to_camera_aug@extrinsic_camera_to_world_aug@y_axis_point_world
                    z_axis_point_image=intrinsic_pixel_to_camera_aug@extrinsic_camera_to_world_aug@z_axis_point_world

                    x_axis_point_pixel=np.array([[x_axis_point_image[0]/x_axis_point_image[2]], [x_axis_point_image[1]/x_axis_point_image[2]]])
                    y_axis_point_pixel=np.array([[y_axis_point_image[0]/y_axis_point_image[2]], [y_axis_point_image[1]/y_axis_point_image[2]]])
                    z_axis_point_pixel=np.array([[z_axis_point_image[0]/z_axis_point_image[2]], [z_axis_point_image[1]/z_axis_point_image[2]]])

                    # draw coordinates on image
                    center=center.astype(int)
                    center_point=tuple(center)
                    x_axis_point=(int(x_axis_point_pixel[0]),int(x_axis_point_pixel[1]))
                    y_axis_point=(int(y_axis_point_pixel[0]),int(y_axis_point_pixel[1]))
                    z_axis_point=(int(z_axis_point_pixel[0]),int(z_axis_point_pixel[1]))
                    cv2.line(img, center_point, x_axis_point, (255, 0, 0), 2)
                    cv2.line(img, center_point, y_axis_point, (0, 255, 0), 2)
                    cv2.line(img, center_point, z_axis_point, (0, 0, 255), 2)

            # publish the image with the detected tags
            img_msg = bridge.cv2_to_imgmsg(img, encoding="bgr8")
            # img_msg = img
            self.image_publisher_.publish(img_msg)

            # # show the img with the detections on the screen
            cv2.imshow('test', img)
            # # cv2.imshow('test', thresh)

            # break the loop if 'q' is pressed
            # key = cv2.waitKey(100)
            # if key == ord('q'):
            #     break
            
            cv2.destroyAllWindows()
            self.cap.release()
    
def main(args=None):
    rclpy.init(args=args)
    node = TagDetectionNode() # MODIFY NAME
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()