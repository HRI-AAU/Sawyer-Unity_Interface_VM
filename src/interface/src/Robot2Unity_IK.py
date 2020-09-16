#!/usr/bin/env python
import rospy
import intera_interface
import socket
import sys
import time
import random
import errno
import netifaces as ni


from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
from std_msgs.msg import Header
from sensor_msgs.msg import JointState

from intera_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)


# Read ip of the ethernet interface
ni.ifaddresses('enp0s3')
ip = ni.ifaddresses('enp0s3')[ni.AF_INET][0]['addr']
port = 10020
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
rospy.init_node("rsdk_ik_service_client")
limb = intera_interface.Limb('right')
msg = ""




def ik_service_client( pose2check,limb = "right", use_advanced_options = False):
    joints_def = {'right_j6': 0.0, 'right_j5': 0.0, 'right_j4': 0.0, 'right_j3': 0.0, 'right_j2': 0.0, 'right_j1': 0.0, 'right_j0': 0.0}
    ns = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
    iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
    ikreq = SolvePositionIKRequest()
    hdr = Header(stamp=rospy.Time.now(), frame_id='base')
    poses = {
        'right': PoseStamped(
            header=hdr,
            pose=Pose(
                position=Point(
                    x=pose2check[0],
                    y=pose2check[1],
                    z=pose2check[2],
                ),
                orientation=Quaternion(
                    x=pose2check[3],
                    y=pose2check[4],
                    z=pose2check[5],
                    w=pose2check[6],
                ),
            ),
        ),
    }
    # Add desired pose for inverse kinematics
    ikreq.pose_stamp.append(poses[limb])
    # Request inverse kinematics from base to "right_hand" link
    ikreq.tip_names.append('right_hand')

    if (use_advanced_options):
        # Optional Advanced IK parameters
        rospy.loginfo("Running Advanced IK Service Client example.")
        # The joint seed is where the IK position solver starts its optimization
        ikreq.seed_mode = ikreq.SEED_USER
        seed = JointState()
        seed.name = ['right_j0', 'right_j1', 'right_j2', 'right_j3',
                     'right_j4', 'right_j5', 'right_j6']
        seed.position = [0.7, 0.4, -1.7, 1.4, -1.1, -1.6, -0.4]
        ikreq.seed_angles.append(seed)

        # Once the primary IK task is solved, the solver will then try to bias the
        # the joint angles toward the goal joint configuration. The null space is 
        # the extra degrees of freedom the joints can move without affecting the
        # primary IK task.
        ikreq.use_nullspace_goal.append(True)
        # The nullspace goal can either be the full set or subset of joint angles
        goal = JointState()
        goal.name = ['right_j1', 'right_j2', 'right_j3']
        goal.position = [0.1, -0.3, 0.5]
        ikreq.nullspace_goal.append(goal)
        # The gain used to bias toward the nullspace goal. Must be [0.0, 1.0]
        # If empty, the default gain of 0.4 will be used
        ikreq.nullspace_gain.append(0.4)
    else:
        rospy.loginfo("Running Simple IK Service Client example.")

    try:
        rospy.wait_for_service(ns, 5.0)
        resp = iksvc(ikreq)
    except (rospy.ServiceException, rospy.ROSException), e:
        rospy.logerr("Service call failed: %s" % (e,))
        return False, joints_def

    # Check if result valid, and type of seed ultimately used to get solution
    if (resp.result_type[0] > 0):
        seed_str = {
                    ikreq.SEED_USER: 'User Provided Seed',
                    ikreq.SEED_CURRENT: 'Current Joint Angles',
                    ikreq.SEED_NS_MAP: 'Nullspace Setpoints',
                   }.get(resp.result_type[0], 'None')
        rospy.loginfo("SUCCESS - Valid Joint Solution Found from Seed Type: %s" %
              (seed_str,))
        # Format solution into Limb API-compatible dictionary
        limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
        rospy.loginfo("\nIK Joint Solution:\n%s", limb_joints)
        rospy.loginfo("------------------")
        rospy.loginfo("Response Message:\n%s", resp)
    else:
        rospy.logerr("INVALID POSE - No Valid Joint Solution Found.")
        rospy.logerr("Result Error %d", resp.result_type[0])
        return False, joints_def

    return True, limb_joints

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

#Sends joint values to unity
def sendJointsValues(connection):
	msg = ''
	print("---------------------------")		
	print(rot_dic)
	print("---------------------------")
	for j in list(rot_dic.keys())[::-1]:			
		msg = msg +"{:.4f}".format(rot_dic[j]) + "J" 
	connection.send(msg)
    	print("sending " + msg)
	return True
#Send NoPose message to unity
def sendNoPose(connection):
	msg = 'NoPose'
	print("---------Send Not Posible------------------")		
	connection.send(msg)
    	print("sending " + msg)
	return True

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



def readFromUnity(connection):
	
	return connection.recv(1024)


#Check with IK.Service if there is a solution for that pose.
#Returns false if not and a dictionary with the joints if yes.	
def checkIK(pose):
	global rot_dic
	thereIsPose, joints_dic = ik_service_client(pose)
	
	if thereIsPose:	
		rot_dic = joints_dic
		return rot_dic
	else:
		return False



##Main Code
try:
	print("-----------------------------")
	print("Robot2Unity_InverseKinematics")
	print("-----------------------------")
	time.sleep(1)
	connection = startSocket(sock)
	isConnected = True	
	while True:
		while isConnected :
			#print("Is Connected" + str(isConnected))
			try:
				print("waiting for message")
				recv = readFromUnity(connection);
				str_recv = recv.decode("utf-8") 
				values = checkReception(str_recv)
				if values != False:
					print("Data received" + str(values))
					thereIsPose, rot_dic = ik_service_client(values)
				
					if thereIsPose:
						sendJointsValues(connection)
						#limb.move_to_joint_positions(rot_dic)
					else:
						sendNoPose(connection);
				else:
					sendNoPose(connection);

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
				else:
				    	print "Exception ", e
					sys.exit(1)
					isConnected = False
					time.sleep(1)
		print("waiting for a connection")
		connection, client_address = sock.accept()
		isConnected = True
		print("The server is connected again :)!")
finally:
	sock.close()
	print("Socket closed")

