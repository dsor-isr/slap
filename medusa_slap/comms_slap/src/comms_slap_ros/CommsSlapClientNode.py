#!/usr/bin/env python

""" 
/**
 * @brief This defines a node for a UDP client used for the SLAP algorithms
 * @author  DSOR team
 * @author  Persons in charges: Hung, Quintas
 * @date    2021
 * @description: the client node is in charge of broadcasting information to the wifi network
 */
"""
import rospy
from std_msgs.msg import Int8, Bool
from farol_msgs.msg import CPFGamma
from medusa_slap_msg.msg import TargetPDF
from auv_msgs.msg import NavigationStatus
import math
import socket
class CommsSlapClientNode():
	received_first_internal_vehicle_position = False
	received_first_internal_gamma = False
	received_first_internal_target_pdf =False
	def __init__(self):
		"""
		Constructor for ros node

		###########################################################################################
		@.@ Init node
		###########################################################################################
		"""
		rospy.init_node('comms_slap_client_node')

		"""
		###########################################################################################
		@.@ Dirty work of declaring subscribers, publishers and load parameters 
		###########################################################################################
		"""
		self.loadParams()

		# +.+ Tuple with Broadcast address and port
		self.address = (self.addr, self.port)

		# +.+ Socket
		self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
		self.sock.bind(("",0))
		self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

		self.initializeSubscribers()

	"""
	###########################################################################################
	@.@ Member Helper function to set up subscribers; 
	###########################################################################################
	"""
	def initializeSubscribers(self):
		rospy.loginfo('Initializing Subscribers for CommsSlapNode')
		rospy.Subscriber(rospy.get_param('~topics/subscribers/gamma_to_neighbor', 'gamma_to_neighbor'), CPFGamma, self.broadcastGammaCallback)	
		rospy.Subscriber(rospy.get_param('~topics/subscribers/pdf_to_neighbor', '/internal/target/pdf_to_neighbor'), TargetPDF , self.broadcastPdfCallback)	
		rospy.Subscriber(rospy.get_param('~topics/subscribers/vehicle_state', 'vehicle_state'), NavigationStatus, self.broadcastVehiclePosCallback)	

	"""
	###########################################################################################
	@.@ Member Helper function to set up parameters; 
	###########################################################################################
	"""
	def loadParams(self):
		self.node_frequency = rospy.get_param('~node_frequency', 10)
		self.Veh_ID = rospy.get_param('~Veh_ID')
		self.port = rospy.get_param('~broadcast_port', 2809)
		self.addr = rospy.get_param('~broadcast_address', '127.255.255.255')


	"""
	###########################################################################################
	@.@ Callback functions/methods 
	###########################################################################################
	"""

	# +.+ Broadcasts the value of gamma from path following
	def broadcastGammaCallback(self, msg):
		if (not self.received_first_internal_gamma):
			print('Client with ID '+ str(self.Veh_ID) + ' received its first gamma to broadcast')	
			self.received_first_internal_gamma = True
		# +.+ Build the message
		data = b"$CPFF,%02i,%06.4f,%06.4f\n" % (self.Veh_ID, msg.gamma,msg.vd)

	#	print(data)
		# +.+ Send message
		count = 0
		while count < len(data):
			count += self.sock.sendto(data, self.address)

		# +.+ Broadcasts the vehicle's position
	def broadcastVehiclePosCallback(self, msg):
		if(not self.received_first_internal_vehicle_position):
			self.received_first_internal_vehicle_position = True
			print('Client with ID ' + str(self.Veh_ID) + ' received its first position to broadcast')	
		# +.+ Build the message
		data = b"$POS,%02i,%06.4f,%06.4f,%06.4f,\
		                  %06.4f,%06.4f,%06.4f\n" % (self.Veh_ID, msg.position.north, msg.position.east, msg.position.depth,
						  										  msg.body_velocity.x*math.cos(msg.orientation.z*math.pi/180.0),
																  msg.body_velocity.x*math.sin(msg.orientation.z*math.pi/180.0),
																  0.0)																	
		# print(data)
		# +.+ Send message
		count = 0
		while count < len(data):
			count += self.sock.sendto(data, self.address)

        # +.+ Broadcasts the probability density function (pdf) of the target 
	def broadcastPdfCallback(self, msg):
		if(not self.received_first_internal_target_pdf):
			self.received_first_internal_target_pdf = True
			print('Client with ID ' + str(self.Veh_ID) + ' received its first target pdf to broadcast')	
		# +.+ Build the message
		data = b"$PDF,%06.4f,%06.4f,%06.4f,%06.4f,%06.4f,%06.4f,\
					 %06.4f,%06.4f,%06.4f,%06.4f,%06.4f,%06.4f,\
					 %06.4f,%06.4f,%06.4f,%06.4f,%06.4f,%06.4f,\
					 %06.4f,%06.4f,%06.4f,%06.4f,%06.4f,%06.4f,\
					 %06.4f,%06.4f,%06.4f,%06.4f,%06.4f,%06.4f,\
					 %06.4f,%06.4f,%06.4f,%06.4f,%06.4f,%06.4f,\
					 %06.4f,%06.4f,%06.4f,%06.4f,%06.4f,%06.4f,\
					 %02i\n" % (
                                msg.state[0], msg.state[1], msg.state[2], msg.state[3], msg.state[4], msg.state[5],
                                msg.cov_row1[0], msg.cov_row1[1], msg.cov_row1[2],msg.cov_row1[3],msg.cov_row1[4],msg.cov_row1[5], 
                                msg.cov_row2[0], msg.cov_row2[1], msg.cov_row2[2],msg.cov_row2[3],msg.cov_row2[4],msg.cov_row2[5], 
                                msg.cov_row3[0], msg.cov_row3[1], msg.cov_row3[2],msg.cov_row3[3],msg.cov_row3[4],msg.cov_row3[5], 
                                msg.cov_row4[0], msg.cov_row4[1], msg.cov_row4[2],msg.cov_row4[3],msg.cov_row4[4],msg.cov_row4[5], 
                                msg.cov_row5[0], msg.cov_row5[1], msg.cov_row5[2],msg.cov_row5[3],msg.cov_row5[4],msg.cov_row5[5], 
                                msg.cov_row6[0], msg.cov_row6[1], msg.cov_row6[2],msg.cov_row6[3],msg.cov_row6[4],msg.cov_row6[5], 
                                self.Veh_ID)

		# print(data)
		# +.+ Send message
		count = 0
		while count < len(data):
			count += self.sock.sendto(data, self.address)

def main():
	print("success")

	commsSlapClient = CommsSlapClientNode()

	# +.+ Added to work with timer -> going into spin; let the callbacks do all the work
	rospy.spin()

if __name__ == '__main__':
	main()
