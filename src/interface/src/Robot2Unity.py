#! /usr/bin/env python
import rospy
import intera_interface
import socket
import sys
import time
import random
import errno
import netifaces as ni

# Read ip of the ethernet interface
ni.ifaddresses('enp0s3')
ip = ni.ifaddresses('enp0s3')[ni.AF_INET][0]['addr']
port = 10000
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
msg = ""
#robot initialization
rospy.init_node('Robot2Unity')
limb = intera_interface.Limb('right')
#dictionary read from the robot, updated by readRobotJoints()
rot_dic = {'right_j6': 0.0, 'right_j5': 0.0, 'right_j4': 0.0, 'right_j3': 0.0, 'right_j2': 0.0, 'right_j1': 0.0, 'right_j0': 0.0}
	

def startSocket(sock):

	# Bind the socket to the port
	server_address = (ip, port)
	print'starting up on ' + server_address[0] + ' port ' + str(server_address[1])
	sock.bind(server_address)
	sock.listen(10)
	# Wait for a connection
	print('waiting for a connection')
	connection, client_address = sock.accept()
	return connection

# Updates the dictionary of rotations asking the robot
def readRobotJoints():
	global rot_dic
	rot_dic = limb.joint_angles()


def sendJointsValues(connection):
	msg = ''
	try:
		readRobotJoints()
		print("---------------------------")
		for j in list(rot_dic.keys())[::-1]:			
			msg = msg +"{:.4f}".format(rot_dic[j]) + "J" 
		connection.send(msg)
	    	print("sending " + msg)
		return True

	except socket.error, e:
		if isinstance(e.args, tuple):#
		    	#print "errno is %d" % e[0]
			if e[0] == 104 or e[0] == 32: #errors client disconected
			       	# remote peer disconnected
			       	print ("Detected remote disconnect")
				print('waiting for a connection')
				return False
			elif e[0] == 4: #keyboard interruption
				print "Exception ", e
				sys.exit(1)
		     	else:
				print "Exception ", e
				sys.exit(1)	
		else:
		    	print "Exception ", e
			sys.exit(1)

print("-----------------------------")
print("Robot2Unity")
print("-----------------------------")

time.sleep(1)
connection = startSocket(sock)
n = 0
try:		
	while True:
		while(sendJointsValues(connection)):
			n = n+1
			print("Message: " + str(n))
			time.sleep(0.05)
		connection, client_address = sock.accept()
finally:
	print("closing the socket")
	sock.close()
