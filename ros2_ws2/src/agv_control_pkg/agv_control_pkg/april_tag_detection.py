import cv2
import numpy as np
import apriltag

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

    # show the img with the detections on the screen
    # cv2.imshow('test', img)
    cv2.imshow('test', thresh)

    # break the loop if 'q' is pressed
    key = cv2.waitKey(1)
    if key == ord('q'):
        break

cv2.destroyAllWindows()
cap.release()



# ============== detect tag in a single image ==============
# Load the image in grayscale
# imagepath = 'shadow2.png'
# image = cv2.imread(imagepath, cv2.IMREAD_GRAYSCALE)

# # Apply Gaussian blur to eliminate noise
# kernel_size = 7
# blur = cv2.GaussianBlur(image, (kernel_size, kernel_size), 0)

# # apply threshold to avoid effect from shadows
# thresh = cv2.threshold(blur, 50, 230, cv2.THRESH_BINARY)[1]
# image = thresh

# # Check if the image is loaded properly
# if image is None:
#     print(f"Error: Image at path '{imagepath}' could not be loaded.")
# else:
#     # Display the image
#     cv2.imshow("Image", image)
#     cv2.waitKey(0)  # Wait for a key press to close the window
#     cv2.destroyAllWindows()

#     # Initialize the AprilTag detector
#     options = apriltag.DetectorOptions(families="tag36h11",
#                                        border=1,
#                                        nthreads=4,
#                                        quad_decimate=1.0,
#                                        quad_blur=0.0,
#                                        refine_edges=True,
#                                        refine_decode=False,
#                                        refine_pose=False,
#                                        debug=False,
#                                        quad_contours=True)
#     detector = apriltag.Detector()

#     # Detect AprilTags in the image
#     detections = detector.detect(image)

#     # Check if any tags are detected
#     if len(detections) == 0:
#         print("No AprilTags detected.")
#     else:
#         # Print detection results
#         # for i, detection in enumerate(detections):
#         #     print(f"Detection {i+1}: {detection}")

#         # Example of accessing the tag family of the first detection
#         tf = detections[0].tag_family
#         id = detections[0].tag_id
#         print(f"Tag Family of first detection: {tf}")
#         print(f"Tag ID of first detection: {id}")