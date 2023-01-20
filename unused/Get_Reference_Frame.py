import rospy 
import numpy as np 
import math
from class_demo import DemoClass
import json 
import random
import sys
import time
from pathlib import Path


from class_vicon_marker import VICON
from class_vicon_base import ViconBase


rospy.init_node("Test")
V = ViconBase("Reference_Frame")
time.sleep(0.1)
print(V.markers)
time.sleep(0.1)

filepath = "results/0819"

reference_marker_dict = {}

reference_marker_position_list = []
for marker in V.markers:
    reference_marker_position = [marker.position.x,marker.position.y,marker.position.z]
    reference_marker_position_list.append(reference_marker_position)
    
    
json_object = json.dumps({"reference_marker_position_list":reference_marker_position_list})

json_name = filepath + "/RefernceFrame.json"

with open(str(json_name), 'w') as outfile:
    outfile.write(json_object)
