#!/usr/bin/env python
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
port = 10010
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
rospy.init_node("MoveRobotFromUnity")
limb = intera_interface.Limb('right')

# Dictionary to comunicate with Sawyer.
rot_dic = {'right_j6': 0.0, 'right_j5': 0.0, 'right_j4': 0.0, 'right_j3': 0.0, 'right_j2': 0.0, 'right_j1': 0.0, 'right_j0': 0.0}

# Create a TCP/IP socket
# Bind the socket to the port
def startSocket(sock):
	server_address = (ip, port)
	print'starting up on ' + server_address[0] + ' port ' + str(server_address[1])
	sock.bind(server_address)
	sock.listen(10)
	# Wait for a connection
	print('waiting for a connection')
	connection, client_address = sock.accept()
	return connection


#Checks the string received and make sure its 7 floats.
#Return flase if any problem, else return an array with the floats
def checkReception(r):
	print("checking: "+r)
	splitedR = r.split('/')
	floatedR = [0,0,0,0,0,0,0] # value of joints 0 to 6
	try:	
		if len(splitedR) >= 7:
			for i in range(7):
				floatedR[i] = float(splitedR[i])
			return floatedR
		else:
			return False
	except ValueError, e :
		print "wrong message received, exception : ",  e 
		return False;


# convert the array of floats to the dictionary that Sawyer expect
def values2dic(values):
	global rot_dic
	i = 0
	for j in list(rot_dic.keys())[::-1]:			
		rot_dic[j] = values[i]
		i += 1

def readFromUnity(connection):
	
	return connection.recv(1024)

def sendReady(connection):
	connection.send("READY")

def sendError(connection):
	connection.send("ERROR")


##Main Code
try:

	print("-----------------------------")
	print("Unity2Robot")
	print("-----------------------------")

	time.sleep(1)
	connection = startSocket(sock)
	isConnected = True	
	while True:
		while isConnected :
			try:
				print("waiting for message")
				recv = readFromUnity(connection);
				str_recv = recv.decode("utf-8") 
				values = checkReception(str_recv)
				if values != False:
					print("Data received" + str(values))
					values2dic(values)
					print("Moving the robot")				
					limb.move_to_joint_positions(rot_dic)
					print("Move finished")
					sendReady(connection)
					print("sended Ready")
				else:
					sendError(connection);

			except socket.error, e:
				if isinstance(e.args, tuple):#
				    	#print "errno is %d" % e[0]
					if e[0] == 104 or e[0] == 32: #errors client disconected
					       	# remote peer disconnected
						print e
					       	print ("Detected remote disconnect")
						print('waiting for a connection')
						isConnected = False
						time.sleep(1)
					elif e[0] == 4: #keyboard interruption
						print "Exception ", e
						sys.exit(1)
						isConnected = False
						time.sleep(1)
				     	else:
						print "Exception ", e
						#sys.exit(1)
						#isConnected = False
						time.sleep(1)
						sendError(connection)	
				else:
				    	print "Exception ", e
					#sys.exit(1)
		print("waiting for a connection")
		connection, client_address = sock.accept()
		isConnected = True
		print("The server is connected again :)!")
finally:
	sock.close()
	print("Socket closed")

