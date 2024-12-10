import time
import serial
import struct
import rclpy
import crcmod
from crcmod.predefined import *
from textwrap import wrap
from rclpy.node import Node
from std_msgs.msg import Float64,Int32
from agv_msgs.msg import AgvMasterMsg,HardwareFeedback,LogicToHardware
Prev_relay_status = [False,False,False,False,False,False,False,False]

def CRC_cal(instructions):
   crc_func = crcmod.predefined.mkCrcFun('modbus')
   CRC=crc_func(bytes(instructions))
   CRC= CRC.to_bytes (2,'little')
   return CRC
    
def read_motor (drive, instruction, self):
  
  #checking to see if 32 bit response expected, default is 16
  if instruction[1]== 0x2A or instruction[1]== 0x2B:
     readlen = 12
     response_type= ">i"
     end_addr = 0x04
  else:
     readlen=7
     response_type= ">h"
     end_addr = 0x02

  self.serial_out.reset_input_buffer()
  tosend = bytes([drive,0x03,instruction[0], instruction[1], 0x00, end_addr])
  CRC = CRC_cal(tosend)
  self.serial_out.write(bytearray(tosend)+CRC)

  response = self.serial_out.read(readlen)
  self.serial_out.reset_input_buffer()
  response =(struct.unpack_from(response_type,response,3)[0])
  return response

def write_motor (drive,instruction, data, manual_data, self):

  if manual_data == False:
    tosend= [drive, 0x06, instruction[0],instruction[1]]
    tosend = bytearray(tosend)+(int(data).to_bytes(2,'big',signed=True))
    CRC = CRC_cal(tosend)
    self.serial_out.write(tosend+CRC)
    dump=self.serial_out.read(8)
  else: 
     tosend= [drive, 0x06, instruction[0],instruction[1],data[0],data[1]]
     CRC= CRC_cal(tosend)
     self.serial_out.write(tosend)
     self.serial_out.write(CRC)
     dump=self.serial_out.read(8)

class SpeedSubscriber(Node):

    def __init__(self):
        super().__init__('speed_subscriber')
        self.voltage_pub = self.create_publisher(Int32, 'voltage', 10)
        self.L_pos_fbk_pub = self.create_publisher(Float64, 'L_pos_fbk', 1)
        self.R_pos_fbk_pub = self.create_publisher(Float64, 'R_pos_fbk', 1)
        self.publisher_Channels=self.create_publisher(HardwareFeedback, 'Hardware_Feedback', 1)
        
    
        self.subscription = self.create_subscription(
           LogicToHardware, 
            '/Logic_To_Hardware', 
            self.Hardware_control_callback,
            1 
        )      

        self.subscription = self.create_subscription(
           Int32, 
            '/front_collision_distance', 
            self.front_collision_callback,
            1 
        )
        self.subscription = self.create_subscription(
           Int32, 
            '/right_collision_distance', 
            self.right_collision_callback,
            1 
        )    
        self.subscription = self.create_subscription(
           Int32, 
            '/rear_collision_distance', 
            self.rear_collision_callback,
            1 
        )    
        self.subscription = self.create_subscription(
           Int32, 
            '/left_collision_distance', 
            self.left_collision_callback,
            1 
        )
        self.collision_distances=[2500,2500,2500,2500]  



        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)



        self.serial_out = serial.Serial('/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_B0019I8S-if00-port0')
        self.serial_out.baudrate = 115200
        self.serial_out.timeout= 0.02
        self.F_speed = 0
        self.R_speed = 0
        self.crc_func = crcmod.predefined.mkCrcFun('modbus')

        
    

          #read only
        self.req_speed = [0x20,0x2C]
        self.req_torque = [0x20,0x2D]
        self.req_voltage =[0x20,0x29]
        self.input_signal_status =[0x20,0x03]
        self.out_signal_status =[0x20,0x04]
        self.req_pos = [0x20,0x2A]
               
        #read and write 
        self.target_speed = [0x20,0x33]
        self.operating_mode = [0x20,0x32]
        self.acceleration_target =[0x20, 0x37]
        self.decAcceleration_Target = [0x20,0x38]
        self.drive_enable_state = [0x20,0x31]
        self.com_offline_time =[0x20,0x00]
        self.reset_positon = [0x20,0x05]
        self.limit_parking_method = [0x20, 0x07]
        self.initial_speed = [0x20, 0x08]
        self.fMaxSpeedTarget = [0x20,0x3A]

        DeAccelTime = 150
        AccelTime = 150

        #init right motor
        data=[0x00,0x03]
        write_motor(0x01,self.operating_mode,data,True,self)
        write_motor(0x01,self.acceleration_target,DeAccelTime,False,self)
        write_motor(0x01,self.decAcceleration_Target,AccelTime,False,self)
        data=[0x00,0x08]
        write_motor(0x01,self.drive_enable_state,data,True,self)

       #init left motor
        data=[0x00,0x03]
        write_motor(0x02,self.operating_mode,data,True,self)
        write_motor(0x02,self.acceleration_target,DeAccelTime,False,self)
        write_motor(0x02,self.decAcceleration_Target,AccelTime,False,self)
        data=[0x00,0x08]
        write_motor(0x02,self.drive_enable_state,data,True,self)
      
        self.get_logger().info('connected to motors')
      


        #-------------------------------------------------------------------------

        read_address=[0x00, 0x03, 0x40, 0x00, 0x00, 0x01, 0x90, 0x1B]
        self.serial_out.write(read_address)
        time.sleep(.005)
        print(self.serial_out.read(16).hex())

        #flipping all relays off on boot 
        turn_off = [(0x00, 0x00, 0x00), (0x01,0x00,0x00),(0x02,0x00,0x00)\
                    ,(0x03,0x00,0x00),(0x04,0x00,0x00)\
                    ,(0x05,0x00,0x00),(0x06,0x00,0x00)\
                    ,(0x07,0x00,0x00)]
        count=0
        while count!=8:
            Relay_Status = [0x03, 0x05, 0x00] + list(turn_off[count])
            CRC= self.crc_func(bytes(Relay_Status))
            CRC = CRC.to_bytes(2,'little')
            self.serial_out.write(Relay_Status)
            self.serial_out.write(CRC)
            time.sleep(.005)
            self.serial_out.read(16).hex()  
            count+=1
        self.subscription
        
    def front_collision_callback(self,msg):
        self.collision_distances[0]=msg.data
    def right_collision_callback(self,msg):
        self.collision_distances[1]=msg.data
    def rear_collision_callback(self,msg):
        self.collision_distances[2]=msg.data
    def left_collision_callback(self,msg):
        self.collision_distances[3]=msg.data

    def timer_callback(self):
        print(self.collision_distances)
        msg = Int32()
        posmsg = Float64()
        hardware_status=HardwareFeedback()

        #ALL LIDAR LOGIC SHOULD NOT BE IN HARDWARE NODE!!!!!
        fDriveMaxSpeed = self.F_speed * -1
        rDriveMaxSpeed = self.R_speed
        
        '''#moving forward and object in front 
        if fDriveMaxSpeed<0 and rDriveMaxSpeed>=0 and self.collision_distances[0]<=28:
            fDriveMaxSpeed=0
            rDriveMaxSpeed=0

        #moving backward and object behind 
        elif fDriveMaxSpeed>=0 and rDriveMaxSpeed<0 and self.collision_distances[2]<=28:
            fDriveMaxSpeed=0
            rDriveMaxSpeed=0

        #turning left and object to left 
        elif fDriveMaxSpeed<0 and rDriveMaxSpeed<0 and self.collision_distances[3]<=10:
            fDriveMaxSpeed=0
            rDriveMaxSpeed=0

        #turning right and object to right 
        elif fDriveMaxSpeed>0 and rDriveMaxSpeed>0 and self.collision_distances[1]<=10:
            fDriveMaxSpeed=0
            rDriveMaxSpeed=0'''        
       
  
        



        write_motor(0x01,self.fMaxSpeedTarget,fDriveMaxSpeed,False,self)
       
        
        write_motor(0x02,self.fMaxSpeedTarget,rDriveMaxSpeed,False,self)

        self.get_logger().info("f speed: %s  r speed: %s"%(fDriveMaxSpeed,rDriveMaxSpeed))


        # Here are the topics related to the hardware control

        try :
            # the motor speed: linear and rotational speed
            voltage=read_motor(0x01,self.req_voltage,self)
            hardware_status.voltage = voltage

            # the absolute wheelendocder feedback: this is already factored to travel distance in cm
            pos_fbk=read_motor(0x02,self.req_pos,self)
            hardware_status.leftposfbk = pos_fbk/79
        
        
            pos_fbk=read_motor(0x01,self.req_pos,self)
            hardware_status.rightposfbk = pos_fbk * -1 / 79

        except:
            self.get_logger().info('womp womp')
            # if there is an error, set the values to error values
            hardware_status.leftposfbk  = -999.9
            hardware_status.rightposfbk = -999.9


        #modbus card---------------------------
        #reading states of digital i/p
        
        read_inputs = [0x03,0x02,0x00,0x00,0x00,0x08]
        CRC= self.crc_func(bytes(read_inputs))
        CRC = CRC.to_bytes(2,'little')
        self.serial_out.write(read_inputs)
        self.serial_out.write(CRC)
        #time.sleep(.01)
        data =self.serial_out.read(12).hex()
        
        data= (data.split('201'))
        
        try: 
            data=data[1]
            Input_data= wrap(data,2)
            Input_data = (int(Input_data[0],16))
            #self.get_logger().info(str(Input_data))
            Bindary_Data = [int(i) for i in list('{0:08b}'.format(Input_data))] #converting int value of input data to binary as a list
            hardware_status.di1= bool(Bindary_Data[7])
            hardware_status.di2= bool(Bindary_Data[6])
            hardware_status.di3= bool(Bindary_Data[5])
            hardware_status.di4= bool(Bindary_Data[4])
            hardware_status.di5= bool(Bindary_Data[3])
            hardware_status.di6= bool(Bindary_Data[2])
            hardware_status.di7= bool(Bindary_Data[1])
            hardware_status.di8= bool(Bindary_Data[0])
            
            self.publisher_Channels.publish(hardware_status)
        
        except:
            self.get_logger().info('womp womp')

       # self.get_logger().info('data being published')

    def Hardware_control_callback(self,msg):

        # the control signals: linear and rotational speed
        self.F_speed = msg.left_speed
        self.R_speed = msg.rightspeed

        if (msg.relay1) != (Prev_relay_status[0]):
            if msg.relay1==True:
                R1_Status=[0x03,0x05,0x00,0x00,0xFF,0x00] #on 
                Prev_relay_status[0]=True 
            else:
                R1_Status=[0x03,0x05,0x00,0x00,0x00,0x00] #off
                Prev_relay_status[0]=False
            CRC= self.crc_func(bytes(R1_Status))
            CRC = CRC.to_bytes(2,'little')
            self.serial_out.write(R1_Status)
            self.serial_out.write(CRC)
            time.sleep(.005)
            self.serial_out.read(16).hex()
        if (msg.relay2) != Prev_relay_status[1]:
            if msg.relay2==True:
                R2_Status=[0x03,0x05,0x00,0x01,0xFF,0x00] #on 
                Prev_relay_status[1]=True
            else:
                R2_Status=[0x03,0x05,0x00,0x01,0x00,0x00] #off
                Prev_relay_status[1]=False
            CRC= self.crc_func(bytes(R2_Status))
            CRC = CRC.to_bytes(2,'little')
            self.serial_out.write(R2_Status)
            self.serial_out.write(CRC)
            #time.sleep(.005)
            self.serial_out.read(16).hex()
        if (msg.relay3) != Prev_relay_status[2]:
            if msg.relay3==True:
                R3_Status=[0x03,0x05,0x00,0x02,0xFF,0x00] #on 
                Prev_relay_status[2]=True
            else:
                R3_Status=[0x03,0x05,0x00,0x02,0x00,0x00] #off
                Prev_relay_status[2]=False

            CRC= self.crc_func(bytes(R3_Status))
            CRC = CRC.to_bytes(2,'little')
            self.serial_out.write(R3_Status)
            self.serial_out.write(CRC)
            #time.sleep(.005)
            self.serial_out.read(16).hex()
        if (msg.relay4) != Prev_relay_status[3]:
            if msg.relay4==True:
                R4_Status=[0x03,0x05,0x00,0x03,0xFF,0x00] #on 
                Prev_relay_status[3]=True
            else:
                R4_Status=[0x03,0x05,0x00,0x03,0x00,0x00] #off
                Prev_relay_status[3]=False
            CRC= self.crc_func(bytes(R4_Status))
            CRC = CRC.to_bytes(2,'little')
            self.serial_out.write(R4_Status)
            self.serial_out.write(CRC)
           # time.sleep(.005)
            self.serial_out.read(16).hex()
        if (msg.relay5) != Prev_relay_status[4]:
            if msg.relay5==True:
                R5_Status=[0x03,0x05,0x00,0x04,0xFF,0x00] #on 
                Prev_relay_status[4]=True
            else:
                R5_Status=[0x03,0x05,0x00,0x04,0x00,0x00] #off
                Prev_relay_status[4]=False
            CRC= self.crc_func(bytes(R5_Status))
            CRC = CRC.to_bytes(2,'little')
            self.serial_out.write(R5_Status)
            self.serial_out.write(CRC)
           # time.sleep(.005)
            self.serial_out.read(16).hex()
        if (msg.relay6) != Prev_relay_status[5]:
            if msg.relay6==True:
                R6_Status=[0x03,0x05,0x00,0x05,0xFF,0x00] #on 
                Prev_relay_status[5]=True
            else:
                R6_Status=[0x03,0x05,0x00,0x05,0x00,0x00] #off
                Prev_relay_status[5]=False
            CRC= self.crc_func(bytes(R6_Status))
            CRC = CRC.to_bytes(2,'little')
            self.serial_out.write(R6_Status)
            self.serial_out.write(CRC)
            #time.sleep(.005)
            self.serial_out.read(16).hex()
        if (msg.relay7) != Prev_relay_status[6]:
            if msg.relay7==True:
                R7_Status=[0x03,0x05,0x00,0x06,0xFF,0x00] #on 
                Prev_relay_status[6]=True
            else:
                R7_Status=[0x03,0x05,0x00,0x06,0x00,0x00] #off
                Prev_relay_status[6]=False
            CRC= self.crc_func(bytes(R7_Status))
            CRC = CRC.to_bytes(2,'little')
            self.serial_out.write(R7_Status)
            self.serial_out.write(CRC)
            #time.sleep(.005)
            self.serial_out.read(16).hex()
        if (msg.relay8) != Prev_relay_status[7]:
            if msg.relay8==True:
                R8_Status=[0x03,0x05,0x00,0x07,0xFF,0x00] #on
                Prev_relay_status[7]=True 
            else:
                R8_Status=[0x03,0x05,0x00,0x07,0x00,0x00] #off
                Prev_relay_status[7]=False
            CRC= self.crc_func(bytes(R8_Status))
            CRC = CRC.to_bytes(2,'little')
            self.serial_out.write(R8_Status)
            self.serial_out.write(CRC)
            #time.sleep(.005)
            self.serial_out.read(16).hex()



def main(args=None):
    rclpy.init(args=args)

    speed_subscriber = SpeedSubscriber()

    rclpy.spin(speed_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    speed_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

