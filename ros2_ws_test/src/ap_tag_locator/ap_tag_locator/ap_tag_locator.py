

import rclpy
from rclpy.node import Node
import apriltag
import cv2

from std_msgs.msg import Int32, Float32
import math

import numpy as np 
from cv_bridge import CvBridge 
from sensor_msgs.msg import Image



def distance_to_camera(knownWidth, focalLength, perWidth):
	# compute and return the distance from the maker to the camera
	return (knownWidth * focalLength) / perWidth


class find_aprilTags(Node):

    
    
    def __init__(self):
        super().__init__('apriltag_locator')
       

        #width of the april tag 
        self.KNOWN_WIDTH=6.5

        #calculated by using object_distance_node

        #focal lengths
        # NEXIGO:653.341496
        # ZIQIAN:1025.341496394231

        self.FOCAL_LENGTH=653.341496
        self.cap= cv2.VideoCapture(2,cv2.CAP_V4L2)

        #([0, 75, 165])
        self.lower_red = np.array([90, 125, 200])
        self.upper_red = np.array([180, 255, 255])

       # self.lower_red = np.array([355, 70, 160])
        #self.upper_red = np.array([355, 355, 100])

        #only needed for ZIQIAN----------------------

        #forcing the MJPG format and frame rate 
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        self.cap.set(cv2.CAP_PROP_FPS, 30)
        #--------------------------------------------
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 800)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 600)

        #options = apriltag.DetectorOptions(families="tag36h11")
        #self.detector = apriltag.Detector(options)


        self.tag_xPub= self.create_publisher(Int32, 'aprilTag_x', 1)
        self.tag_distancePub= self.create_publisher(Float32, 'aprilTag_distance', 1)
        self.tag_idPub= self.create_publisher(Int32, 'aprilTag_ID', 1)
        self.imagePublisher=self.create_publisher(Image,'frontImage',10)
        timer_period = 1 # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        



    def timer_callback(self):
        tag1x=Int32()
        tag1Distance=Float32()
        tag1_id=Int32()
        bridge=CvBridge()

        while self.cap.isOpened():

            ret, image = self.cap.read()
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            kernel_size=7
            blur=cv2.GaussianBlur(gray, (kernel_size,kernel_size),0)
            thresh= cv2.threshold(blur, 180,230, cv2.THRESH_BINARY)[1]

            self.detector = apriltag.Detector()
            results = self.detector.detect(thresh)
            tagCount=0

            for r in results:

                tag_id=r.tag_id
                tagCount+=1
                
                # extract the bounding box (x, y)-coordinates for the AprilTag
                # and convert each of the (x, y)-coordinate pairs to integers
                (ptA, ptB, ptC, ptD) = r.corners
                ptB = (int(ptB[0]), int(ptB[1]))
                ptC = (int(ptC[0]), int(ptC[1]))
                ptD = (int(ptD[0]), int(ptD[1]))
                ptA = (int(ptA[0]), int(ptA[1]))
                
                # draw the bounding box of the AprilTag detection
                cv2.line(image, ptA, ptB, (0, 255, 0), 2)
                cv2.line(image, ptB, ptC, (0, 255, 0), 2)
                cv2.line(image, ptC, ptD, (0, 255, 0), 2)
                cv2.line(image, ptD, ptA, (0, 255, 0), 2)
                
                # draw the center (x, y)-coordinates of the AprilTag
                (cX, cY) = (int(r.center[0]), int(r.center[1]))
                cv2.circle(image, (cX, cY), 5, (0, 0, 255), -1)
                # draw the tag family on the image
                tagFamily = r.tag_family.decode("utf-8")
                
                cv2.putText(image, tagFamily + " tag#"+str(tagCount) + " id:"+str(tag_id), (ptA[0], ptA[1] - 15),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
     
                

                #calculating width of highlighted tag to find distance 
                width = math.sqrt((ptB[0] - ptA[0])**2 + (ptB[1] - ptA[1])**2)
                inches = distance_to_camera(self.KNOWN_WIDTH, self.FOCAL_LENGTH, width)
                
                #displaying distance of tag #1 on video feed 
                if tagCount==1:
                    cv2.putText(image, " tag#1:%.2fft" % (inches / 12),
                    (image.shape[1] - 240, image.shape[0] - 20), cv2.FONT_HERSHEY_SIMPLEX,
                    1.0, (0, 255, 0), 3)
                    tag1_id.data=tag_id
                    tag1x.data=cX
                    tag1Distance.data=inches

                print("tag #%s --center x: %s center y: %s distance: %s" %(tagCount,cX,cY,inches))
                
            if tagCount ==0:
                tag1x.data=0
                tag1Distance.data=0.0
                print("no tags detected...")

            #only publishing position of tag #1
            self.tag_xPub.publish(tag1x)
            self.tag_distancePub.publish(tag1Distance)
            self.tag_idPub.publish(tag1_id)
         

            #only looking for red objects (lines should be red)
            hsv_img = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)        
            mask_red = cv2.inRange(hsv_img, self.lower_red, self.upper_red)   


            edges=cv2.Canny(mask_red,250,350,None,3)
            rho=10
            theta=np.pi/180
            threshold=15
            min_line_length =50
            max_line_gap=20
            

            try:
                line_image=np.copy(image)*0
                lines = cv2.HoughLinesP(edges, rho, theta, threshold, np.array([]),
                                min_line_length, max_line_gap)
                # Draw lines on the blank image
                for line in lines:
                    for x1,y1,x2,y2 in line:
                        cv2.line(line_image,(x1,y1),(x2,y2),(255,0,0),2)
                # Draw the lines on the  image
                lines_edges = cv2.addWeighted(image, 0.8, line_image, 1, 0)
                image = lines_edges
            
            except:
                print("no lines detected...")

            detected_img=bridge.cv2_to_imgmsg(image,encoding='bgr8')
            self.imagePublisher.publish(detected_img)
            #cv2.imshow("Image", image)
            cv2.waitKey(1)
      
   

        
       
            
        
    



     
    

       

def main(args=None):

    rclpy.init(args=args)
    find_tags = find_aprilTags()
    rclpy.spin(find_tags)
    find_tags.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()






