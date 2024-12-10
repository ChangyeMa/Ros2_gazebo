import rclpy
import time
from rclpy.node import Node
from can_msgs.msg import Frame
from bus_msgs.msg import PeakCAN, ParkRelayREQ, BatteryStatus

class CANParser(Node):

	def __init__(self):
		super().__init__('can_parser')
		
		self.subscription = self.create_subscription(
			Frame,
			'/CAN/can0/receive',
			self.can_rec_callback,
			10)
		
		self.subscription = self.create_subscription(
			ParkRelayREQ,
			'ParkRelayREQ',
			self.Park_Relay_callback,
			1)
					
		self.can_publisher = self.create_publisher(Frame, '/CAN/can0/transmit', 10)
		self.parking_shunt_publisher = self.create_publisher(PeakCAN, 'PeakCAN', 10)
		self.battery_publisher = self.create_publisher(BatteryStatus, 'BatteryStatus', 10)

		self.r_r_park_shunt = 0
		self.r_l_park_shunt = 0
		self.f_r_park_shunt = 0
		self.f_l_park_shunt = 0
		self.f_steer_fbk = 0
		self.r_steer_fbk = 0
		self.f_l_park_dir = 0
		self.r_r_park_dir = 0
		self.r_l_park_dir = 0
		self.f_r_park_dir = 0
		self.f_l_park_dir = 0
		self.r_r_park_enb = 0
		self.r_l_park_enb = 0
		self.f_l_park_enb = 0
		self.interlock = 0
		
		self.voltage1 = 0.0
		self.current1 = 0.0
		self.v_min1 = 0.0
		self.v_max1 = 0.0
		self.voltage2 = 0.0
		self.current2 = 0.0
		self.v_min2 = 0.0
		self.v_max2 = 0.0
		self.voltage3 = 0.0
		self.current3 = 0.0
		self.v_min3 = 0.0
		self.v_max3 = 0.0
		self.voltage4 = 0.0
		self.current4 = 0.0
		self.v_min4 = 0.0
		self.v_max4 = 0.0

		self.f_motorspeed = 0
		self.r_motorspeed = 0

		print("Running CAN Node...")

		
	def can_rec_callback(self, msg):
		
		#parse all messages from BMS and from Peak Can device
		
		if msg.id == 1281: # Peak CAN Rear Digital Inputs
			self.r_r_park_shunt = int(msg.data[0] + msg.data[1]*256)
			self.r_l_park_shunt = int(msg.data[2] + msg.data[3]*256)
			self.r_steer_fbk = int(msg.data[4] + msg.data[5]*256)
		
		if msg.id == 257: # Peak CAN Front Digital Inputs
			self.f_r_park_shunt = int(msg.data[0] + msg.data[1]*256)
			self.f_l_park_shunt = int(msg.data[2] + msg.data[3]*256)
			self.f_steer_fbk = int(msg.data[4] + msg.data[5]*256)

		if msg.id == 1283: # Peak CAN Rear Frequency Inputs
			self.r_motorspeed = int(msg.data[0] + msg.data[1]*256)

		if msg.id == 259: # Peak CAN Front Frequency Inputs
			self.f_motorspeed = int(msg.data[0] + msg.data[1]*256)
			pmsg = PeakCAN()
			pmsg.f_r_park_shunt = self.f_r_park_shunt
			pmsg.f_l_park_shunt = self.f_l_park_shunt	
			pmsg.r_r_park_shunt = self.r_r_park_shunt
			pmsg.r_l_park_shunt = self.r_l_park_shunt
			pmsg.f_steer_fbk = self.f_steer_fbk
			pmsg.r_steer_fbk = self.r_steer_fbk
			pmsg.r_motorspeed = self.r_motorspeed
			pmsg.f_motorspeed = self.f_motorspeed
			self.parking_shunt_publisher.publish(pmsg)

		if msg.id == 101: # CAN ID for Battery 1
			self.voltage1 = float(msg.data[0] + (msg.data[1]*256))/1000
			self.current1 = float(msg.data[2] + msg.data[3]*256)
			if self.current1 > 32767 :
				self.current1 = self.current1 - (32767*2)
			self.current1 = self.current1/10

			self.v_min1 = float(msg.data[4] + (msg.data[5]*256))/10000
			self.v_max1 = float(msg.data[6] + (msg.data[7]*256))/10000

			pmsg = BatteryStatus()
			pmsg.voltage1=self.voltage1
			pmsg.current1=self.current1
			pmsg.v_min1=self.v_min1
			pmsg.v_max1=self.v_max1
			pmsg.voltage2=self.voltage2
			pmsg.current2=self.current2
			pmsg.v_min2=self.v_min2
			pmsg.v_max2=self.v_max2
			pmsg.voltage3=self.voltage3
			pmsg.current3=self.current3
			pmsg.v_min3=self.v_min3
			pmsg.v_max3=self.v_max3
			pmsg.voltage4=self.voltage4
			pmsg.current4=self.current4
			pmsg.v_min4=self.v_min4
			pmsg.v_max4=self.v_max4
			self.battery_publisher.publish(pmsg)

		if msg.id == 102: # CAN ID for Battery 2
			self.voltage2 = float(msg.data[0] + (msg.data[1]*256))/1000
			self.current2 = float(msg.data[2] + msg.data[3]*256)
			if self.current2 > 32767 :
				self.current2 = self.current2 - (32767*2)
			self.current2 = self.current2/10

			self.v_min2 = float(msg.data[4] + (msg.data[5]*256))/10000
			self.v_max2 = float(msg.data[6] + (msg.data[7]*256))/10000

		if msg.id == 103: # CAN ID for Battery 3
			self.voltage3 = float(msg.data[0] + (msg.data[1]*256))/1000
			self.current3 = float(msg.data[2] + msg.data[3]*256)
			if self.current3 > 32767 :
				self.current3 = self.current3 - (32767*2)
			self.current3 = self.current3/10

			self.v_min3 = float(msg.data[4] + (msg.data[5]*256))/10000
			self.v_max3 = float(msg.data[6] + (msg.data[7]*256))/10000

		if msg.id == 104: # CAN ID for Battery 4
			self.voltage4 = float(msg.data[0] + (msg.data[1]*256))/1000
			self.current4 = float(msg.data[2] + msg.data[3]*256)
			if self.current4 > 32767 :
				self.current4 = self.current4 - (32767*2)
			self.current4 = self.current4/10

			self.v_min4 = float(msg.data[4] + (msg.data[5]*256))/10000
			self.v_max4 = float(msg.data[6] + (msg.data[7]*256))/10000

	def Park_Relay_callback(self, msg):
	
	#this glorius thing creates lists of bytes from integers
		convert = lambda n : [int(i) for i in n.to_bytes(2, byteorder='little', signed=False)]
		
	#rear digital outputs	
		data = int((msg.interlock*1) + (msg.rr_park_enb*2) + (msg.rr_park_dir*4) + (msg.rl_park_enb*8) + (msg.rl_park_dir*16) +(msg.r_ksi*32)+(msg.r_fwd*64)+(msg.r_rev*128))		
		canmsg = Frame()
		canmsg.dlc = 1
		canmsg.id = 1536
		canmsg.data = [data,0,0,0,0,0,0,0]
		self.can_publisher.publish(canmsg)

	#front digital outputs
		data = int((msg.interlock*1) + (msg.fr_park_enb*2) + (msg.fr_park_dir*4) + (msg.fl_park_enb*8) + (msg.fl_park_dir*16)+(msg.f_ksi*32)+(msg.f_fwd*64)+(msg.f_rev*128))
		canmsg = Frame()
		canmsg.dlc = 1
		canmsg.id = 512
		canmsg.data = [data,0,0,0,0,0,0,0]
		self.can_publisher.publish(canmsg)

	#front analog outputs
		data = []
		data.extend(convert(int(msg.f_traction)))
		data.extend(convert(int(msg.f_steer)))
		data.extend(convert(0))
		data.extend(convert(0))
		canmsg = Frame()
		canmsg.dlc = 4
		canmsg.id = 514
		canmsg.data = data
		self.can_publisher.publish(canmsg)		
	
	#rear analog outputs
		data = []
		data.extend(convert(int(msg.r_traction)))
		data.extend(convert(int(msg.r_steer)))
		data.extend(convert(0))
		data.extend(convert(0))
		canmsg = Frame()
		canmsg.dlc = 4
		canmsg.id = 1538
		canmsg.data = data
		self.can_publisher.publish(canmsg)	
		
	#charger Enable Logic <<<<<<<<<<<<
	#byte 2 is current 0x28 for 40 amps, byte 3 is voltage 0x34 for 52v
		data = [0x6E, 0x1E, 0x34, 0x06, 0x02, 0x00, 0x50, 0x80]

		canmsg = Frame()
		canmsg.dlc = 8
		canmsg.id = 1584
		canmsg.data = data
		self.can_publisher.publish(canmsg)	

def main(args=None):

	rclpy.init(args=args)
	can_parser = CANParser()
	rclpy.spin(can_parser)
	can_parser.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
    main()


