import cv2
import numpy as np
from dt_apriltags import Detector
import math

# ============== define global coordinates for tags ==============
tags_world_positions={
    0:np.array([[0],[0],[0]]),
    1:np.array([[0],[3],[0]]),
    3:np.array([[-3],[0],[0]]),
}

# ============== detect tag in a video stream ==============
cap = cv2.VideoCapture(0)

while True:
    #capture frame by frame
    ret, img = cap.read()

    # convert the image to grayscale otherwise the detection will not work
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # Apply Gaussian blur to eliminate noise
    kernel_size = 7
    blur = cv2.GaussianBlur(gray, (kernel_size, kernel_size), 0)

    # apply threshold to avoid effect from shadows
    thresh = cv2.threshold(blur, 50, 230, cv2.THRESH_BINARY)[1]

    # using april tag from pipy
    # detector = apriltag.Detector()
    # detections = detector.detect(thresh)

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
                                    camera_params=[600,600,320,240],
                                    tag_size=0.049)

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
            print(f"Center of first detection: {center}")
            print(f"Rotation matrix method 1: {R_matrix}")
            print(f"Translation method 1: {t}")

            # check rotation angles:
            theta_x=math.atan2(R_matrix[2, 1],R_matrix[2, 2])
            theta_y=math.atan2(-R_matrix[2, 0],math.sqrt(R_matrix[2, 1]**2+R_matrix[2, 2]**2))
            theta_z=math.atan2(R_matrix[1, 0],R_matrix[0, 0])
            print(f"theta_x:{theta_x*180/math.pi}")
            print(f"theta_y:{theta_y*180/math.pi}")
            print(f"theta_z:{theta_z*180/math.pi}")
            # TF_relative=np.array([  t[0],         # x- translation
            #                         t[1],         # y- translation
            #                         t[2],         # z- translation
            #                         [theta_x],      # r- roll
            #                         [theta_y],      # p- pitch
            #                         [theta_z],      # y- yaw
            #                         ])
            '''
            # # ==== method 2: using cv2.decomposeHomographyMat to get the rotation matrix and translation vector
            # # define a intrinsic camera matrix
            # K = np.array([[1200, 0, 320],
            #               [0, 1200, 240],
            #               [0, 0, 1]])
            # # calculate rotation and translation vectors
            # retval, rotations, translations, normals = cv2.decomposeHomographyMat(H, K)

            # R_2 = rotations[0]  # Example: taking the first solution
            # t_2 = translations[0]  # Example: taking the corresponding translation
            # print(f"Rotation matrix method 2: {R_2}")
            # print(f"Translation method 2: {t_2}")
            # # === end of method 2  results are not correct and oscillate a lot not recommended!! ===
            '''

            # ========= Step #2: Construct extrinsic matrix and intrinsic matrix ========= 
            # 1. to convert from pixel coordinate to world coordinate both matrices are augmented("_aug")
            # 2. both are inversed (indicated by: camera to world )
            extrinsic_camera_to_world_aug= np.eye(4)
            extrinsic_camera_to_world_aug[:3, :3] = R_matrix
            extrinsic_camera_to_world_aug[:3, 3:4] = t
            extrinsic_c2w_inv=np.linalg.inv(extrinsic_camera_to_world_aug)

            intrinsic_pixel_to_camera_aug=np.array([[600, 0, 320, 0],
                                                [0, 600, 240, 0],
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
            print(f"3D coordinate in world {P_world_aug}")

            Pose_camera=tags_world_positions[id]-P_tag_to_camera
            print(f"camera_pose:{Pose_camera}")


            # # assume that the DT version is giving R and t in from world to camera
            # # Compute the transformation from camera to world
            # R_camera_to_world = R_matrix .T # Transpose of rotation to invert it
            # t_camera_to_world = -R_camera_to_world@t# Adjusted translation
            # # print(f"t_camera_to_world:{t_camera_to_world}")

            # # calculate 3D coordinates of the point in the camera frame
            # x_center = detections[0].center[0]
            # y_center = detections[0].center[1]
            # P_center_2D = np.array([[x_center], [y_center],[1]])
            # # print(f"2D coordinates of the center of the tag: {P_center_2D}")
            # P_center_3D = R_matrix@P_center_2D + t
            # # P_center_3D=np.array([[x_center], [y_center],[1]])

            # print(f"3D coordinates of the center of the tag: {P_center_3D}")

            # # Example: Camera pose in world frame

            # # define rotation matrix of camera around x axis
            # theta_x=0
            # R_x = np.array([[1, 0, 0],
            #                 [0, np.cos(theta_x), -np.sin(theta_x)],
            #                 [0, np.sin(theta_x), np.cos(theta_x)]])
            
            # # define rotation matrix of camera around y axis
            # theta_y=0#np.pi/4
            # R_y = np.array([[np.cos(theta_y), 0, -np.sin(theta_y)],
            #                 [0, 1, 0],
            #                 [np.sin(theta_y), 0, np.cos(theta_y)]])

            # # define rotation matrix of camera around z axis
            # theta_z=0
            # R_z = np.array([[np.cos(theta_z), -np.sin(theta_z), 0],
            #                 [np.sin(theta_z), np.cos(theta_z), 0],
            #                 [0, 0, 1]])

            # R_world_to_camera = R_x@R_y@R_z  # Identity matrix if the camera is aligned with the world
            # t_world_to_camera = np.array([0, 0, 0])  # Assume the camera is at the origin in the world frame

            # # Construct the camera-to-world extrinsic matrix
            # extrinsic_camera_to_world = np.eye(4)
            # extrinsic_camera_to_world[:3, :3] = R_camera_to_world 
            # extrinsic_camera_to_world[:3, 3:4] = t_camera_to_world
            # # print(f"Extrinsic matrix from camera to world: {extrinsic_camera_to_world}")

            # # Calculate the coordinates of the center of the tag in the world frame
            # P_center_3D_homogeneous = np.append(P_center_3D, 1)
            # P_center_3D_world = extrinsic_camera_to_world@P_center_3D_homogeneous
            # print(f"3D coordinate in world {P_center_3D_world}")


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


    # show the img with the detections on the screen
    cv2.imshow('test', img)
    # cv2.imshow('test', thresh)

    # break the loop if 'q' is pressed
    key = cv2.waitKey(100)
    if key == ord('q'):
        break

cv2.destroyAllWindows()
cap.release()