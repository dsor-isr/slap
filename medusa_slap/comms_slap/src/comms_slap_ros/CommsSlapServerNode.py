#!/usr/bin/env python

""" 
/**
 * @brief This defines a node for a UDP server used for the SLAP
 * @author  DSOR team
 * @author  Persons in charges: Hung, Quintas
 * @date    2021
 * @description: the server node is in charge of receiving information from the wifi network, then publish it
 */
"""
import rospy
from std_msgs.msg import Int8, Bool 
from farol_msgs.msg import CPFGamma
from medusa_slap_msg.msg import TargetPDF
from medusa_slap_msg.msg import VehiclePosVel 
import socket
import struct
import time

class CommsSlapServerNode():
	def __init__(self):
		"""
		Constructor for ros node
		
		###########################################################################################
		@.@ Init node
		###########################################################################################
		"""
		rospy.init_node('comms_slap_server_node')

		"""
		###########################################################################################
		@.@ Handy Variables
		###########################################################################################
		"""

		# +.+ Possible messages in UDP network	
		self.messages_type = {'$CPFF': 0, '$PDF': 1, '$POS': 2, '$ACK': 3}

		# +.+ List with methods to populate messages 
		self.data_populate = [self.cpfGammaMessage, self.pdfMessage, self.vehPositionMessage]
		
		self.initializePublishers()
		self.loadParams()
				
		# +.+ Socket
		self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
		self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
		self.sock.bind(("", self.port))

		# +.+ Start Server
		self.serverEnable()

	"""
	###########################################################################################
	@.@ Member Helper function to set up publishers; 
	###########################################################################################
	"""
	def initializePublishers(self):
		rospy.loginfo('Initializing Publishers for CommsSlapNode')
		self.pubs = []
		self.pubs.append(rospy.Publisher(rospy.get_param('~topics/publishers/neighbor_gamma','/slap/neighbor/gamma'), 
														  CPFGamma, queue_size=10))
		self.pubs.append(rospy.Publisher(rospy.get_param('~topics/publishers/neighbor_target_pdf','/slap/neighbor/target/pdf'), 
		 												  TargetPDF, queue_size=10))
		self.pubs.append(rospy.Publisher(rospy.get_param('~topics/publishers/neighbor_pos','/slap/neighbor/position'), 
		 												  VehiclePosVel, queue_size=10))
	

	"""
	###########################################################################################
	@.@ Member Helper function to set up parameters; 
	###########################################################################################
	"""
	def loadParams(self):
		self.node_frequency = rospy.get_param('~node_frequency', 10)
		self.Veh_ID = rospy.get_param('~Veh_ID')
		self.port = rospy.get_param('~broadcast_port', 2809)
		

	"""
	###########################################################################################
	@.@ Member Helper function to continuously run server; 
	###########################################################################################
	"""
	def serverEnable(self):
		while True:
			data, addr = self.sock.recvfrom(1025)
			self.parseData(data.decode("utf-8"))

	"""
	###########################################################################################
	@.@ Parse the data received from UDP; 
	###########################################################################################
	"""
	def parseData(self, data):

		# +.+  
		parsed_data = data.strip().split(',')
		if(len(parsed_data) <= 0):
			rospy.logerr("Not possible to parse data, received [{}].".format(data.strip()))
			return
		
		# This would be insane
		# self.pubs[self.messages_type[parsed_data[0]]].publish(self.data_populate[self.messages_type[parsed_data[0]]](parsed_data))
		
		# +.+ Check if the received message is part of the desired ones
		if parsed_data[0] in self.messages_type.keys():
			msg = self.data_populate[self.messages_type[parsed_data[0]]](parsed_data)
			
			# +.+ Check if message was populated and publish it
			if msg is not None:
				self.pubs[self.messages_type[parsed_data[0]]].publish(msg)

	"""
	###########################################################################################
	@.@ Static Helper function to publish gamma received from the network
	###########################################################################################
	"""
	def cpfGammaMessage(self,parsed_data):

		# +.+ publish gamma just received from the network
		# print(parsed_data)
		if len(parsed_data) == 4:
			# +.+ ignore our own transmissions
			# print self.Veh_ID 
			if(int(parsed_data[1])==self.Veh_ID):
			#	rospy.logerr("Ignoring own transmissions")
			 	return None
			msg = CPFGamma()		
			msg.header.stamp = rospy.Time.now()
			msg.ID = int(parsed_data[1])
			msg.gamma = float(parsed_data[2])
			msg.vd = float(parsed_data[3])
			return msg
		else:
			return None

	"""
	###########################################################################################
	@.@ Static Helper function to publish the neighbor probability density function received from the network
	###########################################################################################
	"""
	def pdfMessage(self, parsed_data):
		if(int(parsed_data[-1])==self.Veh_ID):
		#	rospy.logerr("Ignoring own transmissions")
			return None
		msg = TargetPDF()
		msg.header.stamp = rospy.Time.now()
		msg.Veh_ID = int(parsed_data[-1])
		for i in range(6):
			msg.state[i] = float(parsed_data[i+1])
			msg.cov_row1[i] = float(parsed_data[i+7])
			msg.cov_row2[i] = float(parsed_data[i+13])
			msg.cov_row3[i] = float(parsed_data[i+19])
			msg.cov_row4[i] = float(parsed_data[i+25])
			msg.cov_row5[i] = float(parsed_data[i+31])
			msg.cov_row6[i] = float(parsed_data[i+37]) 
		return msg
	"""
	###########################################################################################
	@.@ Static Helper function to publish the neighbor vehicle position received from the network
	###########################################################################################
	"""
	def vehPositionMessage(self,parsed_data):
		if(int(parsed_data[1])==self.Veh_ID):
		#	rospy.logerr("Ignoring own transmissions")
			return None
		msg = VehiclePosVel()		
		msg.header.stamp = rospy.Time.now()
		msg.Veh_ID = int(parsed_data[1])
		msg.position[0] = float(parsed_data[2])
		msg.position[1] = float(parsed_data[3])
		msg.position[2] = float(parsed_data[4])
		msg.velocity[0] = float(parsed_data[5])
		msg.velocity[1] = float(parsed_data[6])
		msg.velocity[2] = float(parsed_data[7])
		
		return msg
    
	"""
	###########################################################################################
	@.@ Static Helper function to populate ETCPFAck messages
	###########################################################################################
	"""
	@staticmethod
	def etcpfAckMessage(parsed_data):
		# +.+ To add in future if necessary
		return None

def main():

	commsSlapServer = CommsSlapServerNode()

	# +.+ Added to work with timer -> going into spin; let the callbacks do all the work
	rospy.spin()
	
	# while not rospy.is_shutdown():
	# 	continue
	
	# commsSlapServer.sock.close()

if __name__ == '__main__':
	main()
