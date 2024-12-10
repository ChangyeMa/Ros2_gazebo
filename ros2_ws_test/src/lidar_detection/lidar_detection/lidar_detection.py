import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
from sklearn.cluster import DBSCAN
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PointStamped
from rclpy.duration import Duration
from std_msgs.msg import Int32
import math






# Define the position of the box: 1=left, 2=right, 0=forward
boxpos = 1

class PointCloudProcessor(Node):
    global boxpos

    def __init__(self):
        super().__init__('pointcloud_processor')

        # Subscribe to the LiDAR point cloud topic
        self.subscription = self.create_subscription(
            PointCloud2,
            '/front_lidar_pointcloud',
            self.front_point_cloud_callback,
            1)
        
         # Subscribe to the LiDAR point cloud topic
        self.subscription = self.create_subscription(
            PointCloud2,
            '/rear_lidar_pointcloud',
            self.rear_point_cloud_callback,
            1)
        
        self.subscription  # prevent unused variable warning

        
        #roi for front of agv-front lidar
        self.front_roi_x=[0.01,0.80]
        self.front_roi_y=[0.20,-0.97]

        #roi for left of agv-front lidar
        self.right_roi_x=[0.0,-0.85]
        self.right_roi_y=[0.01,0.60] 


        #roi for rear of agv-back lidar
        self.rear_roi_x=[0.05,0.75]
        self.rear_roi_y=[0.20,-0.97]

        #roi for left of agv-back lidar
        self.left_roi_x=[0.0,-0.85]
        self.left_roi_y=[0.01,0.60] 


        # Initialize DBSCAN clustering algorithm
        self.dbscan = DBSCAN(eps=0.05, min_samples=3)

        # Create publisher for nearest obstacle marker
        self.front_marker_pub = self.create_publisher(Marker, 'front_and_right_nearest_obstacle', 1)
        self.rear_marker_pub = self.create_publisher(Marker, 'rear_and_left_nearest_obstacle', 1)
       
        # Create publishers for collision distance
        #lidar1
        self.front_collision_pub = self.create_publisher(Int32, 'front_collision_distance', 1)
        self.right_collision_pub = self.create_publisher(Int32, 'right_collision_distance', 1)

        #lidar2
        self.rear_collision_pub = self.create_publisher(Int32, 'rear_collision_distance', 1)
        self.left_collision_pub = self.create_publisher(Int32, 'left_collision_distance', 1)











#-------------------Front Lidar----------------------------------------------------------------
    def front_point_cloud_callback(self, msg):
        collision_distance_msg = Int32()
        # Convert PointCloud2 to numpy array
        points_list = list(pc2.read_points(msg, skip_nans=True, field_names=("x", "y", "z")))
        points_data = [[point[0], point[1], point[2]] for point in points_list]
        points_array = np.array(points_data)
 
        count=0
        while count!=2:
            checkdistance=True
            if count==0:
                roi_points = points_array[
                    (points_array[:, 0] >= min(self.front_roi_x)) & 
                    (points_array[:, 0] <= max(self.front_roi_x)) & 
                    (points_array[:, 1] >= min(self.front_roi_y)) & 
                    (points_array[:, 1] <= max(self.front_roi_y))
                ]
                distance_axis=0
            else:
                roi_points = points_array[
                    (points_array[:, 0] >= min(self.right_roi_x)) & 
                    (points_array[:, 0] <= max(self.right_roi_x)) & 
                    (points_array[:, 1] >= min(self.right_roi_y)) & 
                    (points_array[:, 1] <= max(self.right_roi_y))
                ]
                distance_axis=1
                

            if len(roi_points) < 3:
                # If less than 3 points in ROI, publish default collision distance
                collision_distance_msg.data = 2200

                if count ==0:
                    self.get_logger().info("less than 3 points in front ROI")
                else:
                    self.get_logger().info("less than 3 points in right ROI")
                checkdistance=False
               
               
            if checkdistance==True:
                # Perform DBSCAN clustering
                clustering = self.dbscan.fit(roi_points)
                labels = clustering.labels_
                n_clusters_ = len(set(labels)) - (1 if -1 in labels else 0)

                min_cluster_index = -1
                min_cluster_dist = 2.5
                for index_cluster in range(n_clusters_):
                    i_cluster_points = roi_points[labels == index_cluster]
                    i_min_dist = np.min(i_cluster_points[:, distance_axis])

                    if i_min_dist < min_cluster_dist:
                        min_cluster_dist = i_min_dist
                        min_cluster_index = index_cluster

                min_dist_points = roi_points[labels == min_cluster_index]
                collision_distance_msg.data = max(int(min_cluster_dist * 100), 1)
                self.publish_nearest_front_points(min_dist_points)


            if count ==0:#front roi
                self.front_collision_pub.publish(collision_distance_msg)
                self.get_logger().info("FRONT nearest large object: %d" % (collision_distance_msg.data))
            else:#right roi
                self.right_collision_pub.publish(collision_distance_msg)
                self.get_logger().info("RIGHT nearest large object: %d" % (collision_distance_msg.data))
            count+=1
            
            
    def publish_nearest_front_points(self, points):
        x_min = points[:, 0].min()
        x_max = points[:, 0].max()
        y_min = points[:, 1].min()
        y_max = points[:, 1].max()

        marker = Marker()
        marker.header.frame_id = "laser_link"  # Adjust this to your frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "clusters"
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.scale.x = float(max(x_max - x_min, 0.05))
        marker.scale.y = float(max(y_max - y_min, 0.05))
        marker.scale.z = float(0.2)
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = float((x_min + x_max) / 2)
        marker.pose.position.y = float((y_min + y_max) / 2)
        marker.pose.position.z = float(0)
        marker.lifetime = Duration(seconds=0.12).to_msg()
        self.front_marker_pub.publish(marker)


#-------------------Rear Lidar----------------------------------------------------------------
    def rear_point_cloud_callback(self,msg):
        collision_distance_msg = Int32()
         # Convert PointCloud2 to numpy array
        points_list = list(pc2.read_points(msg, skip_nans=True, field_names=("x", "y", "z")))
        points_data = [[point[0], point[1], point[2]] for point in points_list]
        points_array = np.array(points_data)
 
        count=0
        while count!=2:
            checkdistance=True
            if count==0:
                roi_points = points_array[
                    (points_array[:, 0] >= min(self.rear_roi_x)) & 
                    (points_array[:, 0] <= max(self.rear_roi_x)) & 
                    (points_array[:, 1] >= min(self.rear_roi_y)) & 
                    (points_array[:, 1] <= max(self.rear_roi_y))
                ]
                distance_axis=0
            else:
                roi_points = points_array[
                    (points_array[:, 0] >= min(self.left_roi_x)) & 
                    (points_array[:, 0] <= max(self.left_roi_x)) & 
                    (points_array[:, 1] >= min(self.left_roi_y)) & 
                    (points_array[:, 1] <= max(self.left_roi_y))
                ]
                distance_axis=1
                

            if len(roi_points) < 3:
                # If less than 3 points in ROI, publish default collision distance
                collision_distance_msg.data = 2200
                if count ==0:
                    self.get_logger().info("less than 3 points in rear ROI")
                else:
                    self.get_logger().info("less than 3 points in left ROI")
                checkdistance=False
               
               
            if checkdistance==True:
                # Perform DBSCAN clustering
                clustering = self.dbscan.fit(roi_points)
                labels = clustering.labels_
                n_clusters_ = len(set(labels)) - (1 if -1 in labels else 0)

                min_cluster_index = -1
                min_cluster_dist = 2.5
                for index_cluster in range(n_clusters_):
                    i_cluster_points = roi_points[labels == index_cluster]
                    i_min_dist = np.min(i_cluster_points[:, distance_axis])

                    if i_min_dist < min_cluster_dist:
                        min_cluster_dist = i_min_dist
                        min_cluster_index = index_cluster

                min_dist_points = roi_points[labels == min_cluster_index]
                collision_distance_msg.data = max(int(min_cluster_dist * 100), 1)
                self.publish_nearest_rear_points(min_dist_points)


            if count ==0:#front roi
                self.rear_collision_pub.publish(collision_distance_msg)
                self.get_logger().info("REAR nearest large object: %d" % (collision_distance_msg.data))
            else:#right roi
                self.left_collision_pub.publish(collision_distance_msg)
                self.get_logger().info("LEFT nearest large object: %d" % (collision_distance_msg.data))
            count+=1

    def publish_nearest_rear_points(self, points):
        x_min = points[:, 0].min()
        x_max = points[:, 0].max()
        y_min = points[:, 1].min()
        y_max = points[:, 1].max()

        marker = Marker()
        marker.header.frame_id = "laser_link"  # Adjust this to your frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "clusters"
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.scale.x = float(max(x_max - x_min, 0.05))
        marker.scale.y = float(max(y_max - y_min, 0.05))
        marker.scale.z = float(0.2)
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = float((x_min + x_max) / 2)
        marker.pose.position.y = float((y_min + y_max) / 2)
        marker.pose.position.z = float(0)
        marker.lifetime = Duration(seconds=0.12).to_msg()
        self.rear_marker_pub.publish(marker)
       

def main(args=None):
    rclpy.init(args=args)
    point_cloud_processor = PointCloudProcessor()
    rclpy.spin(point_cloud_processor)
    point_cloud_processor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

