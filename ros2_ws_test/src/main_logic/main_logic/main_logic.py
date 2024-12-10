

import struct
import rclpy
from sensor_msgs.msg import Joy
from rclpy.node import Node
from std_msgs.msg import Int32,Float32,Float64
from agv_msgs.msg import AgvMasterMsg,HardwareFeedback,LogicToHardware
from sensor_msgs.msg import JoyFeedback
from geometry_msgs.msg import Twist


previous_angle = 0

class aprilTag:
  def __init__(self,id,angle,distance):
    self.id = id
    self.angle = angle
    self.distance=distance

class JoySubscriber(Node):

    def __init__(self):
        super().__init__('joy_subscriber')

        timer_period = 0.1  # seconds

        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.subscription = self.create_subscription(
            Joy,
            '/joy',
            self.listener_callback,
            10)
        
        self.subscription = self.create_subscription(
            HardwareFeedback,
            '/Hardware_Feedback',
            self.Hardware_Callback,
            1)

     
        self.subscription = self.create_subscription(
            Float32,
            'aprilTag_distance',
            self.listener_callback_Apriltag_distance,
            1)

        self.subscription = self.create_subscription(
            Int32,
            'aprilTag_x',
            self.listener_callback_Apriltag_X,
            1)

        self.subscription = self.create_subscription(
            Int32,
            'aprilTag_ID',
            self.listener_callback_Apriltag_ID,
            1)
        
        self.subscription = self.create_subscription(
            Twist,
            'cmdVector',
            self.listener_callback_Vector,
            1)

        
        self.f_speed_pub = self.create_publisher(Int32, 'F_speed', 1)
        self.r_speed_pub = self.create_publisher(Int32, 'R_speed', 1)

        self.logic_pub=self.create_publisher(LogicToHardware, 'Logic_To_Hardware', 1)
        self.vibe_pub = self.create_publisher(JoyFeedback, 'joy/set_feedback', 1)
       
        self.get_logger().info('connected to joystick')
        
        self.subscription
        self.manual = 0
        self.rightx = 0
        self.lefty = 0
        self.leftx = 0
       
    
        self.max_vel = 0
        self.collision_distance = 1000
      
        self.L_pos_fbk = 0.00
        self.R_pos_fbk = 0.00
        self.travelled_last = 0.00
        self.travelled = 0.00
        self.travel_rate = 0.00
        self.angle_last = 0.00
        self.angle = 0.00
        self.angle_rate = 0.00

        self.aprilTag_distance= 0.00
        self.aprilTag_x=0
        self.aprilTag_following_state= 0
        self.tag_id = 0
        self.lost_tag_count=0
        self.previous_total_distance= 0.0
        self.previous_angle = 0.0
        self.last_known_id= 0
        self.turnFlag=False


        self.completedMovment=True
        self.completedTurning=False
        self.vectorTurning=False
        self.refrenceAngle=0.0
        self.refrenceDistance=0.0
        self.linearV=0.0
        self.angularV=0.0
        self.vectorAngle=0
        self.vectorDistance=0

        self.softTurningThreshold=90
        '''
        if angle of requested vector is above softTurningThreshold, the robot will do a hard turn moving wheels in oposite direction.
        if angle requested is lower or equal, robot will do a soft turn keeping both motors in same direction 
        '''


        self.aprilTag_list=[]
        #add tags here!
        #negative angle=left 
        #square of tags on floor
        self.aprilTag_list.append(aprilTag(id=0,angle=-90,distance=0))#tag_id, angle to turn once at tag, distance to move once on tag and turned
        self.aprilTag_list.append(aprilTag(id=1,angle=-90,distance=60))#tag_id, angle, distance
        self.aprilTag_list.append(aprilTag(id=2,angle=-90,distance=0))#tag_id, angle, distance
        self.aprilTag_list.append(aprilTag(id=5,angle=-90,distance=60))#tag_id, angle, distance

        #single tag near desks 
        self.aprilTag_list.append(aprilTag(id=4,angle=90,distance=0))#tag_id, angle, distance


        self.index=0

    def listener_callback(self, msg):

        # place controller values into variables for use by main code
        self.leftx = msg.axes[0]
        self.lefty = msg.axes[1]


        if msg.buttons[1]== 1:# circle e-stop
            self.manual = 0

        if msg.buttons[0] == 1: #x for controller mode 
            self.manual = 1
        
        if msg.buttons[3] == 1: #square for vecotr  
            self.manual = 2
    
        if msg.buttons[2]==1: #triangle follow tags on ground 
            self.manual = 3

        #if msg.buttons[6]==1:
          #  self.manual = 4

    def Hardware_Callback(self,msg):

        self.L_pos_fbk=msg.leftposfbk
        self.R_pos_fbk = msg.rightposfbk

    def listener_callback_Apriltag_distance(self,msg):
        self.aprilTag_distance=msg.data

    def listener_callback_Apriltag_X(self,msg):
        self.aprilTag_x=msg.data

    def listener_callback_Apriltag_ID(self,msg):
        self.tag_id=msg.data 

    def listener_callback_Vector(self,msg):
        self.angularV=msg.angular.z
        self.linearV=msg.linear.x

    def timer_callback(self):    
        msgout = LogicToHardware()
        #main logic here!

        #estimate robot distance change
        self.travelled_last = self.travelled
        self.travelled = (self.L_pos_fbk + self.R_pos_fbk)/2  # Average of distance change every cycle is distance vector
        self.travel_rate = self.travelled - self.travelled_last
        
       # self.get_logger().info('Travelled"%s"' % self.travel_rate)
        #estimate robot angle change
        self.angle_last = self.angle
        self.angle = (self.L_pos_fbk - self.R_pos_fbk)*1.04530952 #.98770952
        self.angle_rate = self.angle - self.angle_last
        #self.get_logger().info('Turned "%s"' % self.angle_rate)

        #---------------------------------------------
        #e-stop 
        if self.manual==0:
            self.completedTurning=False
            self.completedMovment=True
            msgout.left_speed=0
            msgout.rightspeed=0
            self.logic_pub.publish(msgout)
        #---------------------------------------------



        #---------------------------------------------
        #controller
        elif self.manual==1:
            self.completedTurning=False
            self.completedMovment=True
            self.aprilTag_following_state=0
            '''if self.collision_distance < 45:
                self.max_vel = 0
            else:
                self.max_vel = int(self.collision_distance * 0.125)'''
            
            self.max_vel=50


            #left motor
            msgout.left_speed= min(int(self.lefty * (150)),self.max_vel) - int(self.leftx * (-50))  
            

            #right motor 
            msgout.rightspeed = min(int(self.lefty * (150)),self.max_vel) + int(self.leftx * (-50))  
            #msgout.relay1=True
            self.logic_pub.publish(msgout)

            msgvibe = JoyFeedback()
            #msgvibe.type = 1
           # msgvibe.intensity = min(max((self.lefty * (100) - self.max_vel)/150,0.00),1.00)

           # self.vibe_pub.publish(msgvibe)


            self.get_logger().info('angle"%s"' % self.angle)
            self.get_logger().info('r motor"%s"' % self.L_pos_fbk)
            self.get_logger().info('l motor"%s"' % self.R_pos_fbk)
        #--------------------------------------------- 
        #vector mode
        elif self.manual==2:
        
            '''
            do the turning first.
            if angle is - go left, if + go right
            look at self.angle as turning refrence
            turn at a fixed rate to start.
            once at correct angle make a flag true
            
            once turning flag is true look at self.travelled as refrence
            to start, move forward at a fixed rate untill the required distance is travelled


            can either set a flag so we must complete each vector before we look at a new one
            being sent in, or can simply override the vector we are working on and go to the next         
            '''

            if self.vectorDistance!=0:
                msgout.left_speed=25
                msgout.rightspeed=25

            if self.completedMovment==True:
                self.completedMovment=False
                self.vectorTurning=True
                self.refrenceAngle=self.angle
                self.vectorAngle=self.angularV
                self.vectorDistance=self.linearV
                
            elif self.completedMovment==False:
                
                self.get_logger().info('angle: %s distance: %s'%(self.vectorAngle,self.vectorDistance))

                if self.completedTurning==False:
                    #turning
                    if abs(int(self.angle) - int(self.refrenceAngle+self.vectorAngle) )> 3:
                        self.get_logger().info('Turning...')
                        
                        if self.vectorAngle<0 and self.vectorAngle>=-self.softTurningThreshold:
                            msgout.left_speed=30
                            msgout.rightspeed=15
                        elif self.vectorAngle<0:
                            msgout.left_speed=15
                            msgout.rightspeed=-15
                    
                        if self.vectorAngle>0 and self.vectorAngle<=self.softTurningThreshold:
                            msgout.left_speed=15
                            msgout.rightspeed=30
                        elif self.vectorAngle>0:
                            msgout.left_speed=-15
                            msgout.rightspeed=15

                    else:
                     
                        self.vectorTurning=False
                        self.refrenceDistance=self.travelled
                        self.completedTurning=True
                        self.get_logger().info('Turned!')
                
                #moving forward 
                if self.vectorTurning==False:
                    if self.travelled<self.refrenceDistance+self.vectorDistance:
                        self.get_logger().info('moving forward.....')
                        self.get_logger().info('travelled: %s target: %s'%(self.travelled,(self.refrenceDistance+self.vectorDistance)))
                        msgout.left_speed=25
                        msgout.rightspeed=25
                    else:
                        
                        self.completedMovment=True
                        self.completedTurning=False
                        self.get_logger().info('Complete!')

                self.logic_pub.publish(msgout)

                




            


                    




        #---------------------------------------------
        


        #-------------------------------------------------------------------------------
        #following trail of apriltags on the ground 
        elif self.manual==3:
            self.completedTurning=False
            self.completedMovment=True
            
  
            
            self.get_logger().info('state:"%s"' % self.aprilTag_following_state)
            #self.get_logger().info('angle"%s"' % self.angle)
            #self.get_logger().info('travelled"%s"' % self.travelled)
            #looking for a tag and going to it 
            if self.aprilTag_following_state==0:

                self.previous_angle=0.0
                self.previous_total_distance=0.0
                self.turnFlag=False

              
                #changing the turn value based on the position of the april tag
                if self.aprilTag_x>=360 and self.aprilTag_x<=590:
                    self.leftx=-0.13
                elif self.aprilTag_x<=300 and self.aprilTag_x>=1:
                    self.leftx=0.13
                else:
                    self.leftx=-0.0
                lastDistance=0.0
                if self.aprilTag_distance!=0.0:
                    lastDistance=self.aprilTag_distance


                #left motor
                msgout.left_speed = int(min((((self.aprilTag_distance)) * 0.4),20)- (self.leftx * (-50)))
           
                #self.get_logger().info('real_left_speed"%s"' % msgout.data)

                #right motor 
                msgout.rightspeed = int(min((((self.aprilTag_distance)) * 0.4),20)+ (self.leftx * (-50))) 
                self.logic_pub.publish(msgout)
                self.last_known_id=self.tag_id
                #if the tag we are following is lost for a while, change states
                if self.aprilTag_x==0 and lastDistance<=50.4:
                    self.lost_tag_count+=1
                else:
                    self.lost_tag_count=0


                if self.lost_tag_count==20:
                    self.previous_total_distance=self.travelled
                    self.aprilTag_following_state=1
            
            #driving on top of tag 
            if self.aprilTag_following_state==1:
                
                #79.6 for haule
                #     for agv
                distance_from_tag_once_lost=68.621

                self.get_logger().info('prev_travel"%s"' % self.previous_total_distance)
                self.get_logger().info('travelled"%s"' % self.travelled)
                
                if self.travelled<=self.previous_total_distance+distance_from_tag_once_lost:
                    
                    #left motor
                    msgout.left_speed = 20
              
                    #right motor 
                    msgout.rightspeed = 20
                    self.logic_pub.publish(msgout)

                else:
                    msgout.left_speed = 0
              
                    #right motor 
                    msgout.rightspeed = 0
                    self.logic_pub.publish(msgout)
                    self.previous_angle=self.angle
                    self.aprilTag_following_state=2
            

            #rotation on tag and moving 
            if self.aprilTag_following_state==2:
            
                self.index=0
                found=False
                for obj in self.aprilTag_list:

                    if self.last_known_id==obj.id:
                        found=True
                        break
                    self.index+=1

                self.get_logger().info('tag_index %s' %self.index)
                self.get_logger().info('target %s' %int(self.previous_angle+self.aprilTag_list[self.index].angle))
                self.get_logger().info('current %s' %int(self.angle))
             
        
                turn_direction = .14
                if self.aprilTag_list[self.index].angle>=0:
                    turn_direction= -.14
                #turning on tag
                if abs(int(self.angle) - int(self.previous_angle+self.aprilTag_list[self.index].angle) )> 3 and self.turnFlag==False and found==True:
                    #left
                    self.get_logger().info('turning')
                    msgout.left_speed = int(0- (turn_direction* (-50)))
                    
                    
                    #right
                    msgout.rightspeed = int(0+ (turn_direction* (-50)))
                    self.logic_pub.publish(msgout)
                    self.previous_total_distance=self.travelled


                    #moving forward on tag 
                else:
                    self.turnFlag=True
                    
                    msgout.left_speed=0
                    msgout.right_speed=0
                    self.logic_pub.publish(msgout)

                    self.get_logger().info(str(self.travelled))
                    self.get_logger().info(str(self.previous_total_distance))
                    if self.travelled<=self.previous_total_distance+self.aprilTag_list[self.index].distance and found==True:
                        self.get_logger().info('moving forward...')
                        #left motor
                        msgout.left_speed= 20
                        
                        #right motor 
                        msgout.rightspeed = 20
                        self.logic_pub.publish(msgout)

                    else:
                        self.get_logger().info('back to state 0')
                        self.turnFlag=False
                        self.aprilTag_following_state=0
        #-------------------------------------------------------------------------------
        
             
            





        #-------------------------------------------------------------------------------
            '''#following apriltag on a person
            elif self.manual==2:
                
                if self.aprilTag_distance < 24.0:
                    max_speed = 0
                elif self.aprilTag_distance<48.0:
                    max_speed = int(self.aprilTag_distance* 0.350)

                else:
                    max_speed=self.aprilTag_distance

                #changing the turn value based on the position of the april tag
                if self.aprilTag_x>=360 and self.aprilTag_x<=590:
                    self.leftx=-0.16
                elif self.aprilTag_x<=300 and self.aprilTag_x>=1:
                    self.leftx=0.16 
                else:
                    self.leftx=-0.0


                #left motor
                msgout.left_speed = int(min((((self.aprilTag_distance)) * 0.4),max_speed,40)- (self.leftx * (-50)))
            
                #self.get_logger().info('real_left_speed"%s"' % msgout.data)

                #right motor 
                msgout.rightspeed = int(min((((self.aprilTag_distance)) * 0.4),max_speed,40)+ (self.leftx * (-50))) 
                
                
                self.logic_pub.publish(msgout)'''
        
def main(args=None):
    rclpy.init(args=args)

    joy_subscriber = JoySubscriber()

    rclpy.spin(joy_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    joy_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

