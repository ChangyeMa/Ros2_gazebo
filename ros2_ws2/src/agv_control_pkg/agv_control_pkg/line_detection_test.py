# Description: Test script for line detection using OpenCV

import cv2
import numpy as np
import math

# Load image
imagepath = 'line_detection_test.png'
image = cv2.imread(imagepath, cv2.IMREAD_GRAYSCALE)

# Apply Gaussian blur
kernel_size = 9
blur = cv2.GaussianBlur(image, (kernel_size, kernel_size), 0)
thresh = cv2.threshold(blur, 100, 250, cv2.THRESH_BINARY)[1]

# Apply Canny edge detection
# edges = cv2.Canny(blur, 30, 230, None, 3)
edges = cv2.Canny(thresh, 50, 200, None, 3)

# # Find contours
# contours, _ = cv2.findContours(edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

# # Draw contours on original image
# cv2.drawContours(image, contours, -1, (255, 0, 0), 2)

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
def merge_lines(lines, distance_threshold=100):
    if lines is None:
        return None

    merged_lines = []
    used = [False] * len(lines)
    
    for i in range(len(lines)):
        if used[i]:
            continue
        
        l1 = lines[i][0]
        merged_line = l1
        used[i] = True
        
        for j in range(i + 1, len(lines)):
            if used[j]:
                continue
            
            l2 = lines[j][0]

            dist_1 = math.sqrt((l1[0] - l2[0]) ** 2 + (l1[1] - l2[1]) ** 2)
            dist_2 = math.sqrt((l1[2] - l2[2]) ** 2 + (l1[3] - l2[3]) ** 2)
            dist_3 = math.sqrt((l1[0] - l2[2]) ** 2 + (l1[1] - l2[3]) ** 2)
            dist_4 = math.sqrt((l1[2] - l2[0]) ** 2 + (l1[3] - l2[1]) ** 2)

            angle_1 = math.atan2(l1[3] - l1[1], l1[2] - l1[0])
            angle_2 = math.atan2(l2[3] - l2[1], l2[2] - l2[0])
            print(angle_1, angle_2)

            # if the tangent of the angle between the two lines is close to 0
            # and the distance between the two lines is less than the threshold
            if (abs(abs(angle_1) - abs(angle_2)) < 1) and (dist_1 < distance_threshold or dist_2 < distance_threshold):
                
                # merged_line = [
                #     (l1[0] + l2[0]) // 2,
                #     (l1[1] + l2[1]) // 2,
                #     (l1[2] + l2[2]) // 2,
                #     (l1[3] + l2[3]) // 2
                # ]

                merged_line = [
                    (min(l1[0], l1[2])+min(l2[0], l2[2])) // 2,
                    (min(l1[1], l1[3])+min(l2[1], l2[3])) // 2,
                    (max(l1[0], l1[2])+max(l2[0], l2[2])) // 2,
                    (max(l1[1], l1[3])+max(l2[1], l2[3])) // 2
                ]

                used[j] = True

        merged_lines.append([merged_line])
    
    return np.array(lines)

# lines = merge_lines(lines)

# print(lines)

# Draw lines on the blank image
for line in lines:
    for x1,y1,x2,y2 in line:
        cv2.line(line_image,(x1,y1),(x2,y2),(255,0,0),3)

# Draw the lines on the  image
lines_edges = cv2.addWeighted(image, 0.8, line_image, 1, 0)

# Display image
# cv2.imshow("Image", line_image)
cv2.imshow("Image", lines_edges)
# cv2.imshow("Image", image)
cv2.waitKey(0)
cv2.destroyAllWindows()
cv2.imwrite('line_detection_result_hough_combined.png', lines_edges)