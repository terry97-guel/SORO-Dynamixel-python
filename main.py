import rospy 
import numpy as np 
import math
from class_demo import DemoClass
import json 
import random
import sys
import time
from pathlib import Path

PI = math.pi
DEG90 = PI/2
init_joint = [0.0, -DEG90, DEG90, 
            -DEG90, -DEG90, 0]

Limit = 2000
soroMove_per_urMove = 5
epochs = 300

filepath = Path("results/0805")
filenames=[]
WRITE_TPOSE = True
for filename in filepath.iterdir():
    if "_" in str(filename):
        latest_number = str.split(str(filename),"_")[1]
        filenames.append(int(latest_number))
    
    if "Tpose" in str(filename): WRITE_TPOSE=False
if filenames: episode_number = max(filenames)+1
else: episode_number=1



if __name__=="__main__":
    ## Initalize
    robot = DemoClass(init_joint)
    init_pos=robot.xc.get_currpos()
    joints = init_joint
    
    ## Record T Pose
    if WRITE_TPOSE:
        TposeList = [
            [0.0, -DEG90, DEG90, -DEG90, -DEG90, 0],
            [0.0, -DEG90, DEG90, -2*DEG90, -DEG90, 0],
            [0.0, -DEG90, DEG90, -3*DEG90, -DEG90, 0],
            [0.0, -DEG90, DEG90, -DEG90, 0, 0],
            [0.0, -DEG90, DEG90, -2*DEG90, 0, 0],
            [0.0, -DEG90, DEG90, -3*DEG90, 0, 0],
        ]
        for idx,joints in enumerate(TposeList):
            robot.execute_arm(joints)
            time.sleep(2)
            
            base_vicon = robot.vicon_base.transform_data
            base_vicon_trans = base_vicon.position # ee_vicon_trans.x,y,z
            base_vicon_rot   = base_vicon.orientation # ee_vicon_rot.x,y,z,w
            
            base_dict = dict(
                base_trans_x = float(base_vicon_trans.x),
                base_trans_y = float(base_vicon_trans.y),
                base_trans_z = float(base_vicon_trans.z),
                base_rotate_x = float(base_vicon_rot.x),
                base_rotate_y = float(base_vicon_rot.y),
                base_rotate_z = float(base_vicon_rot.z),
                base_rotate_w = float(base_vicon_rot.w),
            )
            
            markers = robot.vicon_base.markers
            base_markers_list=[]
            for marker in markers:
                single_dict=dict(
                    x = float(marker.position.x),
                    y = float(marker.position.y),
                    z = float(marker.position.z)
                )
                base_markers_list.append(single_dict)
            
            markers = robot.vicon_marker.markers
            unlabeled_markers_list=[]
            for marker in markers:
                single_dict=dict(
                    x = float(marker.position.x),
                    y = float(marker.position.y),
                    z = float(marker.position.z)
                )
                unlabeled_markers_list.append(single_dict)
            
            integrated_dict = dict(
                base_dict=base_dict,unlabeled_markers_list=unlabeled_markers_list,base_markers_list=base_markers_list
                )
            
            integrated_dict['ur_joints']      = joints
            integrated_dict['soro_actuation'] = init_pos.tolist()
            
            json_object = json.dumps(integrated_dict)

            # Writing to json
            json_name = filepath / "TposeNumber{}.json".format(idx+1)

            with open(str(json_name), 'w') as outfile:
                outfile.write(json_object)

    
    ## Data Extract
    for i in range(epochs):
        joints = init_joint 
        
        joints[-3] = (random.random()*2-3) * DEG90 #[-3,-1]
        joints[-2] = (random.random()*1-1) * DEG90 #[-1 0]
        robot.execute_arm(joints)
        time.sleep(2)
        
        for soroMove_Idx in range(soroMove_per_urMove):
            delta = (np.random.rand(4)) * np.array([1,-1,1,-1]) * Limit
            delta = delta.astype(np.int32)
            pos = init_pos + delta
            robot.xc.set_goalposcluster(pos,1)
            print('pos',robot.xc.get_currpos())

            time.sleep(2)
            base_vicon = robot.vicon_base.transform_data
            base_vicon_trans = base_vicon.position # ee_vicon_trans.x,y,z
            base_vicon_rot   = base_vicon.orientation # ee_vicon_rot.x,y,z,w
            
            base_dict = dict(
                base_trans_x = float(base_vicon_trans.x),
                base_trans_y = float(base_vicon_trans.y),
                base_trans_z = float(base_vicon_trans.z),
                base_rotate_x = float(base_vicon_rot.x),
                base_rotate_y = float(base_vicon_rot.y),
                base_rotate_z = float(base_vicon_rot.z),
                base_rotate_w = float(base_vicon_rot.w),
                
            )
            
            markers = robot.vicon_base.markers
            base_markers_list=[]
            for marker in markers:
                single_dict=dict(
                    x = float(marker.position.x),
                    y = float(marker.position.y),
                    z = float(marker.position.z)
                )
                base_markers_list.append(single_dict)
            
            markers = robot.vicon_marker.markers
            unlabeled_markers_list=[]
            for marker in markers:
                single_dict=dict(
                    x = float(marker.position.x),
                    y = float(marker.position.y),
                    z = float(marker.position.z)
                )
                unlabeled_markers_list.append(single_dict)
                
            
            
            integrated_dict = dict(
                base_dict=base_dict,unlabeled_markers_list=unlabeled_markers_list,base_markers_list=base_markers_list
                )
            
            integrated_dict['ur_joints']      = joints
            integrated_dict['soro_actuation'] = delta.tolist()
            
            json_object = json.dumps(integrated_dict)
  
            # Writing to json
            json_name = filepath / "dataset_{}_.json".format(episode_number)
            episode_number = episode_number + 1

            with open(str(json_name), 'w') as outfile:
                outfile.write(json_object)
    
    import os
    os.system('shutdown now - sf')