#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <yolo_msgs/msg/detection_array.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <image_geometry/pinhole_camera_model.h>
#include <cv_bridge/cv_bridge.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/crop_box.h>
#include <pcl/common/transforms.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <opencv2/opencv.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <thread>
#include <vector>
#include <mutex>


class LidarCameraFusionNode : public rclcpp::Node
{
public:
    LidarCameraFusionNode()
        : Node("lidar_camera_fusion_node"),
          tf_buffer_(this->get_clock()),  // Initialize TF2 buffer
          tf_listener_(tf_buffer_)        // Initialize TF2 listener
    {
        declare_parameters();  // Declare and load parameters
        initialize_subscribers_and_publishers();  // Set up subscribers and publishers
    }

private:
    // Structure to hold bounding box information
    struct BoundingBox {
        double x_min, y_min, x_max, y_max;  // Bounding box coordinates in image space
        double sum_x = 0, sum_y = 0, sum_z = 0;  // Accumulated point coordinates for averaging
        int count = 0;  // Number of points in the bounding box
        bool valid = false;  // Flag to indicate if the bounding box is valid
        int id = -1;  // ID of the detected object
        pcl::PointCloud<pcl::PointXYZ>::Ptr object_cloud = nullptr;  // Point cloud for the object
    };

    // Declare and load parameters from the parameter server
    void declare_parameters()
    {
        declare_parameter<std::string>("lidar_frame", "lidar_link");
        declare_parameter<std::string>("camera_frame", "camera_1_link");
        declare_parameter<float>("min_range", 0.05);
        declare_parameter<float>("max_range", 30.0);

         // Ground filtering parameter
        declare_parameter<bool>("enable_ground_filtering", true);
        declare_parameter<float>("ground_height_threshold", 0.6);  // Points below this Z value will be removed

        // image flip parameters
        declare_parameter<std::string>("flip_image", "none"); // options: none, horizontal, vertical, both

        // Statistical outlier removal parameters
        declare_parameter<int>("statistical_outlier_k_neighbors", 15);
        declare_parameter<float>("statistical_outlier_std_multiplier", 2.0);
        
        // DBSCAN clustering parameters
        declare_parameter<float>("dbscan_eps", 0.25);  // 25cm neighborhood
        declare_parameter<int>("dbscan_min_points", 5);
    
        // Get parameters
        get_parameter("statistical_outlier_k_neighbors", statistical_outlier_k_neighbors_);
        get_parameter("statistical_outlier_std_multiplier", statistical_outlier_std_multiplier_);
        get_parameter("dbscan_eps", dbscan_eps_);
        get_parameter("dbscan_min_points", dbscan_min_points_);

        get_parameter("lidar_frame", lidar_frame_);
        get_parameter("camera_frame", camera_frame_);
        get_parameter("min_range", min_range_);
        get_parameter("max_range", max_range_);
        get_parameter("enable_ground_filtering", enable_ground_filtering_);
        get_parameter("ground_height_threshold", ground_height_threshold_);
        get_parameter("flip_image", flip_image_);

        RCLCPP_INFO(
            get_logger(),
            "Parameters: lidar_frame='%s', camera_frame='%s', min_range=%.2f, max_range=%.2f, ground_filtering=%s, height_threshold=%.2f",
            lidar_frame_.c_str(),
            camera_frame_.c_str(),
            min_range_,
            max_range_,
            enable_ground_filtering_ ? "enabled" : "disabled",
            ground_height_threshold_
        );
    }

    // Initialize subscribers and publishers
    void initialize_subscribers_and_publishers()
    {
        // Subscribers for point cloud, image, and detections in standard naming
        point_cloud_sub_.subscribe(this, "/lidar_points");
        image_sub_.subscribe(this, "/camera/image_raw");
        detection_sub_.subscribe(this, "/yolo/tracking");
        camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
            "/camera/camera_info", 10, std::bind(&LidarCameraFusionNode::camera_info_callback, this, std::placeholders::_1));

        // Synchronizer to align point cloud, image, and detection messages
        using SyncPolicy = message_filters::sync_policies::ApproximateTime<
            sensor_msgs::msg::PointCloud2, sensor_msgs::msg::Image, yolo_msgs::msg::DetectionArray>;
        sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(SyncPolicy(10), point_cloud_sub_, image_sub_, detection_sub_);
        sync_->registerCallback(std::bind(&LidarCameraFusionNode::sync_callback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

        // Publishers for fused image, object poses, and object point clouds
        image_publisher_ = create_publisher<sensor_msgs::msg::Image>("/image_lidar_fusion", 10);
        pose_publisher_ = create_publisher<geometry_msgs::msg::PoseArray>("/detected_object_pose", 10);
        object_point_cloud_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>("/detected_object_point_cloud", 10);
    }

    // Callback for camera info to initialize the camera model
    void camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
    {
        camera_model_.fromCameraInfo(msg);  // Load camera intrinsics
        image_width_ = msg->width;  // Store image width
        image_height_ = msg->height;  // Store image height
    }

    // Synchronized callback for point cloud, image, and detections
    void sync_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& point_cloud_msg,
                       const sensor_msgs::msg::Image::ConstSharedPtr& image_msg,
                       const yolo_msgs::msg::DetectionArray::ConstSharedPtr& detection_msg)
    {
        // Process point cloud: crop, transform to camera frame
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_camera_frame = processPointCloud(point_cloud_msg);
        if (!cloud_camera_frame) {
           RCLCPP_ERROR(get_logger(), "Failed to process point cloud. Exiting callback.");
            return;
        }

        RCLCPP_INFO(get_logger(), "Timestamps - Cloud: %d.%d, Image: %d.%d, Detections: %d.%d",
            point_cloud_msg->header.stamp.sec, point_cloud_msg->header.stamp.nanosec,
            image_msg->header.stamp.sec, image_msg->header.stamp.nanosec,
            detection_msg->header.stamp.sec, detection_msg->header.stamp.nanosec);

        // Process detections: extract bounding boxes
        std::vector<BoundingBox> bounding_boxes = processDetections(detection_msg);
        if (bounding_boxes.empty()) {
            RCLCPP_WARN(get_logger(), "No valid bounding boxes found. Skipping projection.");
            return;
        }

        // Process detections: extract bounding boxes
        // std::vector<BoundingBox> bounding_boxes = processDetections(detection_msg);

        // Project 3D points to 2D image space and associate with bounding boxes
        std::vector<cv::Point2d> projected_points = projectPointsAndAssociateWithBoundingBoxes(cloud_camera_frame, bounding_boxes);

        // Calculate object poses in the lidar frame
        geometry_msgs::msg::PoseArray pose_array = calculateObjectPoses(bounding_boxes, point_cloud_msg->header.stamp);

        // Publish results: fused image, object poses, and object point clouds
        publishResults(image_msg, projected_points, bounding_boxes, pose_array);
    }

    // ============================ Preprocessing ============================
    '''
    The lidar point preprocessing includes several steps:
    1. Cropping the point cloud to a defined range. (e.g. min and max range, and angles if needed)
    2. Applying height-based ground filtering.)
    3. Transforming the point cloud to the camera frame. (find valid projection on image plane)
    4. Process the camera detections and extract bounding boxes.
    '''

    // Simple height-based ground filtering function
    pcl::PointCloud<pcl::PointXYZ>::Ptr filterGroundPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
    {
        if (!enable_ground_filtering_ || !cloud || cloud->empty()) {
            return cloud;
        }
        
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        
        size_t original_size = cloud->points.size();
        
        for (const auto& point : cloud->points) {
            // Keep points below the ground height threshold
            if (point.z < ground_height_threshold_) {
                filtered_cloud->points.push_back(point);
            }
        }
        
        // Set point cloud properties
        filtered_cloud->width = filtered_cloud->points.size();
        filtered_cloud->height = 1;  // ??
        filtered_cloud->is_dense = true;
        filtered_cloud->header = cloud->header;  // Preserve header information
        
        // RCLCPP_INFO(get_logger(), "Ground filtering: %zu -> %zu points (removed %zu ground points)", 
        //             original_size, filtered_cloud->points.size(), original_size - filtered_cloud->points.size());
        
        return filtered_cloud;
    }

    // Process point cloud: crop and transform to camera frame
    pcl::PointCloud<pcl::PointXYZ>::Ptr processPointCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& point_cloud_msg)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*point_cloud_msg, *cloud);  // Convert ROS message to PCL point cloud

        // Crop point cloud to a defined range (this is for 360 degree lidar and here it is cropped to a cube)
        pcl::CropBox<pcl::PointXYZ> box_filter;
        box_filter.setInputCloud(cloud);
        box_filter.setMin(Eigen::Vector4f(-max_range_, -max_range_, -max_range_, 1.0f));
        box_filter.setMax(Eigen::Vector4f(max_range_, max_range_, max_range_, 1.0f));
        box_filter.filter(*cloud);

        // Apply height-based ground filtering
        cloud = filterGroundPoints(cloud);

         if (cloud->empty()) {
            RCLCPP_WARN(get_logger(), "Point cloud is empty after filtering, skipping transform.");
            return cloud;
        }

        // Transform point cloud to camera frame using TF2
        rclcpp::Time cloud_time(point_cloud_msg->header.stamp);

        if (tf_buffer_.canTransform(camera_frame_, cloud->header.frame_id, cloud_time, tf2::durationFromSec(1.0))) {
            geometry_msgs::msg::TransformStamped transform = tf_buffer_.lookupTransform(camera_frame_, cloud->header.frame_id, cloud_time, tf2::durationFromSec(1.0));

            // debug messages for checking the transform
            // RCLCPP_INFO(get_logger(), "Transform from %s to %s:", 
            //         cloud->header.frame_id.c_str(), camera_frame_.c_str());
            // RCLCPP_INFO(get_logger(), "Translation: (%.3f, %.3f, %.3f)", 
            //         transform.transform.translation.x, 
            //         transform.transform.translation.y, 
            //         transform.transform.translation.z);
            // RCLCPP_INFO(get_logger(), "Rotation (quaternion): (%.3f, %.3f, %.3f, %.3f)", 
            //         transform.transform.rotation.x, 
            //         transform.transform.rotation.y, 
            //         transform.transform.rotation.z, 
            //         transform.transform.rotation.w);
            // RCLCPP_INFO(get_logger(), "Original cloud frame_id: %s", cloud->header.frame_id.c_str());

            Eigen::Affine3d eigen_transform = tf2::transformToEigen(transform); // Eigen::Affine3d - which is a 4x4 transformation matrix
            pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::transformPointCloud(*cloud, *transformed_cloud, eigen_transform);
            return transformed_cloud;
        } else {
            RCLCPP_ERROR(get_logger(), "Could not transform point cloud from %s to %s", cloud->header.frame_id.c_str(), camera_frame_.c_str());
            return nullptr;
        }
    }

    // Process detections: extract bounding boxes from YOLO detections
    std::vector<BoundingBox> processDetections(const yolo_msgs::msg::DetectionArray::ConstSharedPtr& detection_msg)
    {
        std::vector<BoundingBox> bounding_boxes;
        RCLCPP_INFO(get_logger(), "=== DETECTION DEBUG ===");
        RCLCPP_INFO(get_logger(), "Received %zu detections from YOLO", detection_msg->detections.size());

        for (size_t i = 0; i < detection_msg->detections.size(); ++i) {
            const auto& detection = detection_msg->detections[i];
            
            BoundingBox bbox;
            bbox.x_min = detection.bbox.center.position.x - detection.bbox.size.x / 2.0;
            bbox.y_min = detection.bbox.center.position.y - detection.bbox.size.y / 2.0;
            bbox.x_max = detection.bbox.center.position.x + detection.bbox.size.x / 2.0;
            bbox.y_max = detection.bbox.center.position.y + detection.bbox.size.y / 2.0;
            bbox.valid = true;
            
            try {
                bbox.id = std::stoi(detection.id);
            } catch (const std::exception& e) {
                RCLCPP_ERROR(get_logger(), "Failed to convert detection ID to integer: %s", e.what());
                continue;
            }
            
            bbox.object_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
            
            // Debug info for each detection
            RCLCPP_INFO(get_logger(), "Detection %zu: ID=%d, bbox=[%.1f,%.1f,%.1f,%.1f], size=%.1fx%.1f", 
                        i, bbox.id, bbox.x_min, bbox.y_min, bbox.x_max, bbox.y_max,
                        detection.bbox.size.x, detection.bbox.size.y);
            
            // Check if bounding box is reasonable
            if (bbox.x_min >= bbox.x_max || bbox.y_min >= bbox.y_max) {
                RCLCPP_WARN(get_logger(), "Invalid bounding box dimensions for detection %zu", i);
                continue;
            }
            
            // Check if bounding box is within image bounds (assuming you have image dimensions)
            if (bbox.x_max < 0 || bbox.y_max < 0 || bbox.x_min > image_width_ || bbox.y_min > image_height_) {
                RCLCPP_WARN(get_logger(), "Bounding box %zu is outside image bounds [%dx%d]", i, image_width_, image_height_);
            }
            
            bounding_boxes.push_back(bbox);
        }
        return bounding_boxes;
    }


    // =========================== Projection and filtering ============================
    '''
    1. With valid projection points and bounding boxes, associate points to bounding boxes.
    2. Filter out outliers and identify the correct object points.
    3. Project the filtered points onto the image plane. (visualization)
    4. Identify object poses and pre-defined goal points in the lidar frame.
    5. Convert robot pose, goal points into a local frame for navigation.
    '''

    // 1. Depth filter to remove occluded points behind the closest surface per pixel
    cv::Mat computeMinDepthMap(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
        const image_geometry::PinholeCameraModel& camera_model,
        int image_width, int image_height)
    {
        cv::Mat depth_map(image_height, image_width, CV_32FC1, cv::Scalar(std::numeric_limits<float>::infinity()));

        for (const auto& pt : cloud->points) {
            if (pt.z <= 0) continue;  // only front camera is available at the moment

            cv::Point3d cv_pt(pt.x, pt.y, pt.z);
            cv::Point2d uv = camera_model.project3dToPixel(cv_pt);

            // Handle image flipping if specified
            if (flip_image_ == "horizontal" || flip_image_ == "both") {
                uv.x = image_width_ - uv.x;
            }
            if (flip_image_ == "vertical" || flip_image_ == "both") {
                uv.y = image_height_ - uv.y;
            }

            int u = static_cast<int>(std::round(uv.x));
            int v = static_cast<int>(std::round(uv.y));
            if (u >= 0 && u < image_width && v >= 0 && v < image_height) {
                float& current = depth_map.at<float>(v, u);
                if (pt.z < current) {
                    current = pt.z;  // keep the nearest depth
                }
            }
        }

        // Optionally smooth to reduce noise
        cv::medianBlur(depth_map, depth_map, 3);

        return depth_map;
    }

    // 2. Statistical outlier removal for points within bounding boxes
    std::vector<pcl::PointXYZ> removeStatisticalOutliers(
        const std::vector<pcl::PointXYZ>& points,
        int k_neighbors = 20,
        float std_dev_multiplier = 2.0)
    {
        if (points.size() < k_neighbors) return points;

        std::vector<float> distances;
        distances.reserve(points.size());

        // Calculate mean distance to k nearest neighbors for each point
        for (size_t i = 0; i < points.size(); ++i) {
            std::vector<float> neighbor_distances;
            
            for (size_t j = 0; j < points.size(); ++j) {
                if (i != j) {
                    float dx = points[i].x - points[j].x;
                    float dy = points[i].y - points[j].y;
                    float dz = points[i].z - points[j].z;
                    // Euclidean distance
                    neighbor_distances.push_back(std::sqrt(dx*dx + dy*dy + dz*dz));
                }
            }
            
            std::sort(neighbor_distances.begin(), neighbor_distances.end());
            
            // Calculate mean distance to k nearest neighbors
            float mean_distance = 0.0f;
            int neighbors_to_use = std::min(k_neighbors, static_cast<int>(neighbor_distances.size()));
            for (int k = 0; k < neighbors_to_use; ++k) {
                mean_distance += neighbor_distances[k];
            }
            mean_distance /= neighbors_to_use;
            distances.push_back(mean_distance);
        }

        // Calculate statistics
        float mean_dist = std::accumulate(distances.begin(), distances.end(), 0.0f) / distances.size();
        float variance = 0.0f;
        for (float d : distances) {
            variance += (d - mean_dist) * (d - mean_dist);
        }
        variance /= distances.size();
        float std_dev = std::sqrt(variance);

        // Filter outliers
        std::vector<pcl::PointXYZ> filtered_points;
        float threshold = mean_dist + std_dev_multiplier * std_dev;
        
        for (size_t i = 0; i < points.size(); ++i) {
            if (distances[i] <= threshold) {
                filtered_points.push_back(points[i]);
            }
        }
        return filtered_points;
    }

    // 3. DBSCAN clustering to identify coherent point groups
    struct DBSCANCluster {
        std::vector<pcl::PointXYZ> points;
        pcl::PointXYZ centroid;
        int point_count;
    };

    std::vector<DBSCANCluster> performDBSCAN(
        const std::vector<pcl::PointXYZ>& points,
        float eps = 0.3f,  // 30cm neighborhood
        int min_points = 5)
    {
        std::vector<int> cluster_labels(points.size(), -1);  // -1 means noise
        std::vector<bool> visited(points.size(), false);
        int cluster_id = 0;

        auto get_neighbors = [&](int point_idx) -> std::vector<int> {
            std::vector<int> neighbors;
            const auto& p = points[point_idx];
            
            for (size_t i = 0; i < points.size(); ++i) {
                if (i == point_idx) continue;
                
                float dx = p.x - points[i].x;
                float dy = p.y - points[i].y;
                float dz = p.z - points[i].z;
                float distance = std::sqrt(dx*dx + dy*dy + dz*dz);
                
                if (distance <= eps) {
                    neighbors.push_back(i);
                }
            }
            return neighbors;
        };

        for (size_t i = 0; i < points.size(); ++i) {
            if (visited[i]) continue;
            
            visited[i] = true;
            std::vector<int> neighbors = get_neighbors(i);
            
            if (neighbors.size() < min_points) {
                cluster_labels[i] = -1;  // Mark as noise
            } else {
                // Start new cluster
                cluster_labels[i] = cluster_id;
                
                // Expand cluster
                std::queue<int> seed_set;
                for (int neighbor : neighbors) {
                    seed_set.push(neighbor);
                }
                
                while (!seed_set.empty()) {
                    int current = seed_set.front();
                    seed_set.pop();
                    
                    if (!visited[current]) {
                        visited[current] = true;
                        std::vector<int> current_neighbors = get_neighbors(current);
                        
                        if (current_neighbors.size() >= min_points) {
                            for (int neighbor : current_neighbors) {
                                seed_set.push(neighbor);
                            }
                        }
                    }
                    
                    if (cluster_labels[current] == -1) {
                        cluster_labels[current] = cluster_id;
                    }
                }
                cluster_id++;
            }
        }

        // Convert to clusters
        std::vector<DBSCANCluster> clusters(cluster_id);
        for (size_t i = 0; i < points.size(); ++i) {
            if (cluster_labels[i] >= 0) {
                clusters[cluster_labels[i]].points.push_back(points[i]);
            }
        }

        // Calculate centroids
        for (auto& cluster : clusters) {
            if (!cluster.points.empty()) {
                float sum_x = 0, sum_y = 0, sum_z = 0;
                for (const auto& pt : cluster.points) {
                    sum_x += pt.x;
                    sum_y += pt.y;
                    sum_z += pt.z;
                }
                cluster.centroid.x = sum_x / cluster.points.size();
                cluster.centroid.y = sum_y / cluster.points.size();
                cluster.centroid.z = sum_z / cluster.points.size();
                cluster.point_count = cluster.points.size();
            }
        }

        return clusters;
    }


    // Project 3D points to 2D image space and associate with bounding boxes (multi-threaded)
    std::vector<cv::Point2d> projectPointsAndAssociateWithBoundingBoxes(
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_camera_frame,
        std::vector<BoundingBox>& bounding_boxes)
    {
        std::vector<cv::Point2d> projected_points;
        RCLCPP_INFO(get_logger(), "=== PROJECTION DEBUG START ===");

        if (!cloud_camera_frame){
            RCLCPP_WARN(get_logger(), "The cloud is invalid in projectPointsAndAssociateWithBoundingBoxes. Skipping the projection.");
            return projected_points;
        }

        if (!camera_model_.initialized()) {
            RCLCPP_ERROR(get_logger(), "ERROR: Camera model is NOT initialized!");
            return projected_points;
        }

        RCLCPP_INFO(get_logger(), "Camera model is initialized");
        RCLCPP_INFO(get_logger(), "Image dimensions: %dx%d", image_width_, image_height_);
        RCLCPP_INFO(get_logger(), "Bounding boxes: %zu", bounding_boxes.size());

        // Compute minimum depth map for occlusion handling
        // Build a per-pixel min-depth map
        cv::Mat depth_map = computeMinDepthMap(cloud_camera_frame, camera_model_, image_width_, image_height_);
        float depth_tolerance = 0.1;  // meters, tune as needed

        // Lambda function to process points in parallel
        auto process_points = [&](size_t start, size_t end) {
            for (size_t i = start; i < end; ++i) {
                const auto& point = cloud_camera_frame->points[i];
                if (point.z < 10 && point.z > 0) {  // Consider points within a reasonable depth range that is in the front (for now just one camera)
                    cv::Point3d pt_cv(point.x, point.y, point.z);
                    cv::Point2d uv = camera_model_.project3dToPixel(pt_cv);
                    
                    // Handle image flipping if specified
                    if (flip_image_ == "horizontal" || flip_image_ == "both") {
                        uv.x = image_width_ - uv.x;
                    }
                    if (flip_image_ == "vertical" || flip_image_ == "both") {
                        uv.y = image_height_ - uv.y;
                    }
                    // If the image is flipped both horizontally and vertically, both adjustments are applied
                    // If no flipping is specified, uv remains unchanged    

                    // Add this debug logging for first few points
                    // static int debug_count = 0;
                    // if (debug_count < 10) {
                    //     RCLCPP_INFO(get_logger(), "Point %d: 3D(%.2f,%.2f,%.2f) -> 2D(%.1f,%.1f)", 
                    //                 debug_count++, point.x, point.y, point.z, uv.x, uv.y);
                        
                    //     for (const auto& bbox : bounding_boxes) {
                    //         bool x_in = (uv.x >= bbox.x_min && uv.x <= bbox.x_max);
                    //         bool y_in = (uv.y >= bbox.y_min && uv.y <= bbox.y_max);
                    //         RCLCPP_INFO(get_logger(), "  vs BBox %d: X %s(%.1f in [%.1f,%.1f]), Y %s(%.1f in [%.1f,%.1f])",
                    //                 bbox.id, x_in?"✓":"✗", uv.x, bbox.x_min, bbox.x_max,
                    //                 y_in?"✓":"✗", uv.y, bbox.y_min, bbox.y_max);
                    //     }
                    // }

                    for (auto& bbox : bounding_boxes) {
                        if (uv.x >= bbox.x_min && uv.x <= bbox.x_max &&
                            uv.y >= bbox.y_min && uv.y <= bbox.y_max) {
                            // Point lies within the bounding box
                            // define pixel coordinates
                            int u = static_cast<int>(std::round(uv.x));
                            int v = static_cast<int>(std::round(uv.y));

                            // 1. Check depth against min depth map for occlusion handling (But this can't handle sparse structures)
                            float min_depth = depth_map.at<float>(v, u);
                            if (point.z <= min_depth + depth_tolerance) {
                                // Point is valid (not behind another object)
                                std::lock_guard<std::mutex> lock(mtx);
                                projected_points.push_back(uv);
                                bbox.sum_x += point.x;
                                bbox.sum_y += point.y;
                                bbox.sum_z += point.z;
                                bbox.count++;
                                bbox.object_cloud->points.push_back(point);
                            }

                            break;  // Early exit: skip remaining bounding boxes for this point
                        }
                    }
                    
                    // apply staticstical outlier removal and DBSCAN clustering per bounding box
                    if (bbox.object_cloud->empty()) continue;

                    // Apply statistical outlier removal
                    std::vector<pcl::PointXYZ> filtered_points = removeStatisticalOutliers(bbox.object_cloud->points, 10, 1.5f); // (points, k_neighbors, std_dev_multiplier)

                    // Apply DBSCAN clustering to find coherent groups
                    std::vector<DBSCANCluster> clusters = performDBSCAN(filtered_points, 0.2f, 3); // (points, eps, min_points)

                    if (!clusters.empty()) {
                        auto largest_cluster = std::max_element(clusters.begin(), clusters.end(),
                            [](const DBSCANCluster& a, const DBSCANCluster& b) {
                                return a.point_count < b.point_count;
                            });
                        
                        // Use points from the largest cluster
                        bbox.object_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
                        for (const auto& pt : largest_cluster->points) {
                            bbox.object_cloud->points.push_back(pt);
                            bbox.sum_x += pt.x;
                            bbox.sum_y += pt.y;
                            bbox.sum_z += pt.z;
                            bbox.count++;
                            
                            // Project back to image for visualization
                            cv::Point3d pt_cv(pt.x, pt.y, pt.z);
                            cv::Point2d uv = camera_model_.project3dToPixel(pt_cv);
                            
                            if (flip_image_ == "horizontal" || flip_image_ == "both") {
                                uv.x = image_width_ - uv.x;
                            }
                            if (flip_image_ == "vertical" || flip_image_ == "both") {
                                uv.y = image_height_ - uv.y;
                            }
                            
                            projected_points.push_back(uv);
                        }
                    } else {
                        // Fallback: use filtered points if no clusters found
                        bbox.object_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
                        for (const auto& pt : filtered_points) {
                            bbox.object_cloud->points.push_back(pt);
                            bbox.sum_x += pt.x;
                            bbox.sum_y += pt.y;
                            bbox.sum_z += pt.z;
                            bbox.count++;
                        }
                    }
                    
                    // Set point cloud properties
                    if (bbox.object_cloud && !bbox.object_cloud->points.empty()) {
                        bbox.object_cloud->width = bbox.object_cloud->points.size();
                        bbox.object_cloud->height = 1;
                        bbox.object_cloud->is_dense = true;
                    }

                }
            }
        };

        // Split the work across multiple threads
        const size_t num_threads = std::thread::hardware_concurrency();
        const size_t points_per_thread = cloud_camera_frame->points.size() / num_threads;
        std::vector<std::thread> threads;

        for (size_t t = 0; t < num_threads; ++t) {
            size_t start = t * points_per_thread;
            size_t end = (t == num_threads - 1) ? cloud_camera_frame->points.size() : start + points_per_thread;
            threads.emplace_back(process_points, start, end);
        }

        // Wait for all threads to finish
        for (auto& thread : threads) {
            thread.join();
        }

        return projected_points;
    }

    // Calculate object poses in the lidar frame
    geometry_msgs::msg::PoseArray calculateObjectPoses(
        const std::vector<BoundingBox>& bounding_boxes,
        const rclcpp::Time& cloud_time)
    {
        geometry_msgs::msg::PoseArray pose_array;
        pose_array.header.stamp = cloud_time;
        pose_array.header.frame_id = lidar_frame_;

        // Look up the transformation from camera to LiDAR frame
        geometry_msgs::msg::TransformStamped transform;
        try {
            transform = tf_buffer_.lookupTransform(lidar_frame_, camera_frame_, cloud_time, tf2::durationFromSec(1.0));
        } catch (tf2::TransformException& ex) {
            RCLCPP_ERROR(get_logger(), "Failed to lookup transform: %s", ex.what());
            return pose_array;  // Return empty PoseArray if transformation fails
        }

        // Convert the transform to Eigen for faster computation
        Eigen::Affine3d eigen_transform = tf2::transformToEigen(transform);

        // Calculate average position for each bounding box and transform to LiDAR frame
        for (const auto& bbox : bounding_boxes) {
            if (bbox.count > 0) {
                double avg_x = bbox.sum_x / bbox.count;
                double avg_y = bbox.sum_y / bbox.count;
                double avg_z = bbox.sum_z / bbox.count;

                // Create a Pose in camera frame
                geometry_msgs::msg::PoseStamped pose_camera;
                pose_camera.header.stamp = cloud_time;
                pose_camera.header.frame_id = camera_frame_;
                pose_camera.pose.position.x = avg_x;
                pose_camera.pose.position.y = avg_y;
                pose_camera.pose.position.z = avg_z;
                pose_camera.pose.orientation.w = 1.0;

                // Transform pose to lidar frame
                try {
                    geometry_msgs::msg::PoseStamped pose_lidar = tf_buffer_.transform(pose_camera, lidar_frame_, tf2::durationFromSec(1.0));
                    pose_array.poses.push_back(pose_lidar.pose);
                } catch (tf2::TransformException& ex) {
                    RCLCPP_ERROR(get_logger(), "Failed to transform pose: %s", ex.what());
                }
            } else {
                 RCLCPP_WARN(get_logger(), "Skipping pose calculation for bbox ID %d, count is 0", bbox.id);
            }
        }
        return pose_array;
    }

    // Publish results: fused image, object poses, and object point clouds
    void publishResults(
        const sensor_msgs::msg::Image::ConstSharedPtr& image_msg,
        const std::vector<cv::Point2d>& projected_points,
        const std::vector<BoundingBox>& bounding_boxes,
        const geometry_msgs::msg::PoseArray& pose_array)
    {
        // Draw projected points on the image
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
        for (const auto& uv : projected_points) {
            cv::circle(cv_ptr->image, cv::Point(uv.x, uv.y), 5, CV_RGB(255, 0, 0), -1);
        }

        // Publish the fused image
        image_publisher_->publish(*cv_ptr->toImageMsg());

        // Combine all object point clouds into a single cloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr combined_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        for (const auto& bbox : bounding_boxes) {
            if (bbox.count > 0 && bbox.object_cloud) {
                *combined_cloud += *bbox.object_cloud;  // Concatenate point clouds
            }
        }

        // Publish the combined point cloud
        if (!combined_cloud->empty()) {
            sensor_msgs::msg::PointCloud2 combined_cloud_msg;
            pcl::toROSMsg(*combined_cloud, combined_cloud_msg);
            combined_cloud_msg.header = image_msg->header;
            combined_cloud_msg.header.frame_id = camera_frame_;
            object_point_cloud_publisher_->publish(combined_cloud_msg);
        }

        // Publish object poses
        pose_publisher_->publish(pose_array);
    }

    // TF2 buffer and listener for coordinate transformations
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    // Camera model for projecting 3D points to 2D image space
    image_geometry::PinholeCameraModel camera_model_;

    // Parameters for cropping and coordinate frames
    float min_range_, max_range_;
    std::string camera_frame_, lidar_frame_;
    int image_width_, image_height_;
    bool enable_ground_filtering_;
    float ground_height_threshold_;
    std::string flip_image_;

    // For filtering
    float depth_tolerance_ = 0.2;  // meters, tolerance for depth filtering

    int statistical_outlier_k_neighbors_ = 15;
    float statistical_outlier_std_multiplier_ = 2.0f;
    float dbscan_eps_ = 0.25f;
    int dbscan_min_points_ = 5;

    // Subscribers for point cloud, image, and detections
    message_filters::Subscriber<sensor_msgs::msg::PointCloud2> point_cloud_sub_;
    message_filters::Subscriber<sensor_msgs::msg::Image> image_sub_;
    message_filters::Subscriber<yolo_msgs::msg::DetectionArray> detection_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;

    // Synchronizer for aligning messages
    std::shared_ptr<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2, sensor_msgs::msg::Image, yolo_msgs::msg::DetectionArray>>> sync_;
    
    // Depth map for occlusion handling
    cv::Mat min_depth_map_;  

    // Publishers for fused image, object poses, and object point clouds
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pose_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr object_point_cloud_publisher_;

    // Mutex for thread-safe updates
    std::mutex mtx;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);  // Initialize ROS2
    auto node = std::make_shared<LidarCameraFusionNode>();  // Create node
    rclcpp::spin(node);  // Run node
    rclcpp::shutdown();  // Shutdown ROS2
    return 0;
}
