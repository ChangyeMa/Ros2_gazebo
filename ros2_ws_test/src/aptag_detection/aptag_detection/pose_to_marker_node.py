import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseArray
from visualization_msgs.msg import Marker

class PoseToMarkerNode(Node):
    def __init__(self):
        super().__init__('pose_to_marker_node')

        # Subscriber to AMR pose messages:
        # AMR pose with wheel encoders
        self.pose_subscriber = self.create_subscription(
            Pose,
            '/current_pose_wheel',
            self.pose_callback_wheel,
            10
        )

        # AMR pose with camera detection
        self.pose_subscriber = self.create_subscription(
            Pose,
            '/current_pose_with_cam',
            self.pose_callback_camera,
            10
        )

        # initialize the node with some fixed landmarks
        # (this part should be improved later with yaml file)
        self.ap_tags = PoseArray()
        self.ap_tags.header.frame_id = "world"
        self.ap_tags.header.stamp = self.get_clock().now().to_msg()
        self.ap_tags.poses = []
        self.get_logger().info("Pose to Marker node has started")

        self.tag_1=Pose()
        self.tag_1.position.x=0.0
        self.tag_1.position.y=0.0
        self.tag_1.position.z=0.0
        self.tag_1.orientation.x=0.0
        self.tag_1.orientation.y=0.0
        self.tag_1.orientation.z=0.0
        self.tag_1.orientation.w=1.0

        self.tag_2=Pose()
        self.tag_2.position.x=0.0
        self.tag_2.position.y=2.5
        self.tag_2.position.z=0.0
        self.tag_2.orientation.x=0.0
        self.tag_2.orientation.y=0.0
        self.tag_2.orientation.z=0.0
        self.tag_2.orientation.w=1.0

        self.tag_3=Pose()
        self.tag_3.position.x=-2.5
        self.tag_3.position.y=0.0
        self.tag_3.position.z=0.0
        self.tag_3.orientation.x=0.0
        self.tag_3.orientation.y=0.0
        self.tag_3.orientation.z=0.0
        self.tag_3.orientation.w=1.0

        # self.tag_4=Pose()
        # self.tag_4.position.x=-2.5 
        # self.tag_4.position.y=2.5
        # self.tag_4.position.z=0.0
        # self.tag_4.orientation.x=0.0
        # self.tag_4.orientation.y=0.0
        # self.tag_4.orientation.z=0.0    
        # self.tag_4.orientation.w=1.0

        self.ap_tags.poses.append(self.tag_1)
        self.ap_tags.poses.append(self.tag_2)
        self.ap_tags.poses.append(self.tag_3)
        # self.ap_tags.poses.append(self.tag_4)

        # Publisher for Marker messages
        self.marker_publisher = self.create_publisher(
            Marker,
            'marker_topic',
            10
        )
        self.get_logger().info("Pose to Marker publisher has started")
        
        self.loop_frequency_ = 0.1
        self.loop_timer_ = self.create_timer(self.loop_frequency_, self.MarkerLoop)

    # def camera_pose_callback(self, msg):

    #     # create marker for camera 1
    #     marker = Marker()
    #     marker.header.frame_id = "world"
    #     marker.header.stamp = self.get_clock().now().to_msg()
    #     marker.ns = "camera_marker"
    #     marker.id = 10
    #     marker.type = Marker.CUBE  # You can use SPHERE, CUBE, etc.
    #     marker.action = Marker.ADD

    #     marker.pose = msg

    #     # Set the scale of the marker
    #     marker.scale.x = 0.1  # Length of the cube
    #     marker.scale.y = 0.05  # Width of the cube
    #     marker.scale.z = 0.02  # Height of the cube

    #     # Set the color of the marker (RGBA)
    #     marker.color.r = 0.0
    #     marker.color.g = 1.0
    #     marker.color.b = 0.0
    #     marker.color.a = 1.0  # 1.0 means fully opaque

    #     # Publish the marker for AMR
    #     self.marker_publisher.publish(marker)
    #     self.get_logger().info("Published Marker for camera 1")

    def pose_callback_wheel(self, msg):
        # Update the pose of the robot measured  with wheel encoders
        self.pose_AMR = msg
        # self.get_logger().info("Received Pose")

        #create marker of the AMR
        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "robot_marker_wheel"
        marker.id = 0
        marker.type = Marker.CUBE  # You can use SPHERE, CUBE, etc.
        marker.action = Marker.ADD

        marker.pose = self.pose_AMR

        # Set the scale of the marker
        # AMR dimensions: 0.99m(0.85 frame + 0.07 camera offset for each side) x 0.753m x 0.23m
        marker.scale.x = 0.99  # Length of the cube 
        marker.scale.y = 0.735  # Width of the cube
        marker.scale.z = 0.23  # Height of the cube

        # Set the color of the marker (RGBA)
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        marker.color.a = 1.0  # 1.0 means fully opaque

        # Publish the marker for AMR
        self.marker_publisher.publish(marker)
        self.get_logger().info("Published Marker for AMR pose (wheel)")

    def pose_callback_camera(self, msg):
        # Update the pose of the robot
        self.pose_AMR_cam = msg
        self.get_logger().info("Received Pose for camera_AMR_pose")

        #create marker of the AMR
        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "robot_with_camera_marker"
        marker.id = 20
        marker.type = Marker.CUBE  # You can use SPHERE, CUBE, etc.
        marker.action = Marker.ADD

        marker.pose = self.pose_AMR_cam

        # Set the scale of the marker
        marker.scale.x = 0.735  # Length of the cube
        marker.scale.y = 0.99  # Width of the cube
        marker.scale.z = 0.23  # Height of the cube

        # Set the color of the marker (RGBA)
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0  # 1.0 means fully opaque

        # Publish the marker for AMR
        self.marker_publisher.publish(marker)
        self.get_logger().info("Published Marker for camera_AMR pose")

    def MarkerLoop(self):

        # Loop through the landmarks and publish them as markers
        for i in range(len(self.ap_tags.poses)):

            # create market with unique id and pose of the landmark (Apriltags)
            marker = Marker()
            marker.header.frame_id = "world"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "pose_marker"
            marker.id = i+10
            marker.type = Marker.CUBE  # You can use SPHERE, CUBE, etc.
            marker.action = Marker.ADD

            marker.pose = self.ap_tags.poses[i]

            # Set the scale of the marker
            marker.scale.x = 0.1  # Length of the arrow
            marker.scale.y = 0.1  # Width of the arrow
            marker.scale.z = 0.01  # Height of the arrow

            # Set the color of the marker (RGBA)
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 1.0  # 1.0 means fully opaque

            # Publish the marker
            self.marker_publisher.publish(marker)
            # self.get_logger().info("Published Marker for tags")
      
def main(args=None):
    rclpy.init(args=args)
    node = PoseToMarkerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
