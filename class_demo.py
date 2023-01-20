import os, time
import numpy as np
import matplotlib.pyplot as plt
from datetime import datetime
from math import pi

import sys
sys.path.append(".")
""" FOR DYNAMIXEL """
from model.class_xc330 import xc330
""" FOR MODERN DRIVER """
import roslib; roslib.load_manifest('ur_driver')
import rospy
import actionlib
from control_msgs.msg import *
from trajectory_msgs.msg import *
from sensor_msgs.msg import JointState

from class_vicon_marker import VICON
from class_vicon_base import ViconBase

PROPORTION_MATRIX = np.array(
    [
        [1,0.1,0.4,0.4, 0,0,0,0, 0,0,0,0, 0,0,0,0],
        [0.1,1,0.4,0.4, 0,0,0,0, 0,0,0,0, 0,0,0,0],
        [0.4,0.4,1,0.1, 0,0,0,0, 0,0,0,0, 0,0,0,0],
        [0.4,0.4,0.1,1, 0,0,0,0, 0,0,0,0, 0,0,0,0],
        
        [0,0,0,0,  1,0.1,0.4,0.4, 0,0,0,0, 0,0,0,0],
        [0,0,0,0,  0.1,1,0.4,0.4,  0,0,0,0, 0,0,0,0],
        [0,0,0,0,  0.4,0.4,1,0.1, 0,0,0,0, 0,0,0,0],
        [0,0,0,0,  0.4,0.4,0.1,1, 0,0,0,0, 0,0,0,0],
        
        [0,0,0,0, 0,0,0,0, 1,0.1,0.4,0.4, 0,0,0,0],
        [0,0,0,0, 0,0,0,0, 0.1,1,0.4,0.4, 0,0,0,0],
        [0,0,0,0, 0,0,0,0, 0.4,0.4,1,0.1, 0,0,0,0],
        [0,0,0,0, 0,0,0,0, 0.4,0.4,0.1,1, 0,0,0,0],
        
        [0,0,0,0, 0,0,0,0, 0,0,0,0, 1,0.1,0.4,0.4],
        [0,0,0,0, 0,0,0,0, 0,0,0,0, 0.1,1,0.4,0.4],
        [0,0,0,0, 0,0,0,0, 0,0,0,0, 0.4,0.4,1,0.1],
        [0,0,0,0, 0,0,0,0, 0,0,0,0, 0.4,0.4,0.1,1],
    ])


## For one Finger ##
class DemoClass(object):
    def __init__(self):
        rospy.init_node("DEMO")
        self.JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
                            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
        self.client      = None
        self.vicon_marker= VICON()
        self.vicon_reference  = ViconBase('ReferenceFrame')
        self.vicon_platform = ViconBase('Platform')
        # self.execute_arm(init_joint)
        self.xc          = xc330('SNAPBOT', _USB_NUM=0)
        
    def init_xc(self,initpos):
        self.xc.connect()
        self.xc.set_operatingmode([4])

        self.xc.set_torque([0])
        time.sleep(2)
        self.xc.set_torque([1])
        print ("TORQUE ON")

        currpos = self.xc.get_currpos()
        minmaxInterval = 5000
        self.xc.set_minmaxpos(currpos-np.ones_like(currpos)*minmaxInterval,currpos+np.ones_like(currpos)*minmaxInterval)

        if (initpos==None).any(): initpos = currpos; print("No initpos Found")
        self.initpos = initpos
        self.xc.set_goalposcluster(initpos,1)

        print ("INITIALIZE POSITION")
        currpos = self.xc.get_currpos()
        print (currpos)

        minmaxInterval = minmaxInterval
        self.xc.set_minmaxpos(currpos-np.ones_like(currpos)*minmaxInterval,currpos+np.ones_like(currpos)*minmaxInterval)
    
    def pmove_xc(self,pos):
        motor_num = len(pos)
        pos = self.initpos + PROPORTION_MATRIX[:motor_num,:motor_num] @ np.array(pos) * np.array([1,-1,-1,1])
        self.xc.set_goalposcluster(pos,1)
    
    def move_xc(self,pos):
        self.xc.set_goalposcluster(pos,1)
    
    # def move_arm(self, joints):
    #     try: 
    #         q = joints
    #         g = FollowJointTrajectoryGoal()
    #         g.trajectory = JointTrajectory()
    #         g.trajectory.joint_names = self.JOINT_NAMES
    #         joint_states = rospy.wait_for_message("joint_states", JointState)
    #         joints_pos   = joint_states.position
    #         g.trajectory.points = [
    #             JointTrajectoryPoint(positions=joints_pos, velocities=[0]*6, time_from_start=rospy.Duration(0.0)),
    #             JointTrajectoryPoint(positions=q, velocities=[0]*6, time_from_start=rospy.Duration(3))]  
    #         self.client.send_goal(g)
    #         self.client.wait_for_result()
    #     except KeyboardInterrupt:
    #         self.client.cancel_goal()
    #         raise
    #     except:
    #         raise  
         
    # def execute_arm(self, joints):
    #     try:
    #         self.client = actionlib.SimpleActionClient('follow_joint_trajectory', FollowJointTrajectoryAction)
    #         print("Waiting for server...")
    #         self.client.wait_for_server()
    #         print("Connected to server")
    #         """ Initialize """
    #         self.move_arm(joints)
    #         print("Finish plan")

    #     except KeyboardInterrupt:
    #         rospy.signal_shutdown("KeyboardInterrupt")
    #         raise      
    
    