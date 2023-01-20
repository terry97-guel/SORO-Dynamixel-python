import rospy 
import numpy as np 
import math
from class_demo import DemoClass
import json 
import random
import sys
import time
from pathlib import Path


def homepose_pmove_capture(robot, delta, json_name):
    # robot.pmove_xc(np.zeros_like(delta))
    time.sleep(1)
    
    # print('delta',delta)
    # robot.pmove_xc(delta)
    # time.sleep(3)
    
    markers = robot.vicon_reference.markers
    reference_markers_list=[]
    for marker in markers:
        single_dict=dict(
            x = float(marker.position.x),
            y = float(marker.position.y),
            z = float(marker.position.z)
        )
        reference_markers_list.append(single_dict)
    
    markers = robot.vicon_platform.markers
    platform_markers_list=[]
    for marker in markers:
        single_dict=dict(
            x = float(marker.position.x),
            y = float(marker.position.y),
            z = float(marker.position.z)
        )
        platform_markers_list.append(single_dict)
    
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
        unlabeled_markers_list=unlabeled_markers_list,
        platform_markers_list=platform_markers_list,
        reference_markers_list=reference_markers_list
        )
    
    integrated_dict['ur_joints']      = joints
    # integrated_dict['soro_actuation'] = delta.tolist()
    
    json_object = json.dumps(integrated_dict)

    # Writing to json
    with open(str(json_name), 'w') as outfile:
        outfile.write(json_object)


if __name__=="__main__":
    ## Preset ##
    PI = math.pi
    DEG90 = PI/2

    init_joint = [0.0, -DEG90, DEG90, 
                -DEG90, DEG90, 0]

    Limit = 2000
    soroMove_per_urMove = 5
    epochs = 600

    basepath = Path("results/mocap")
    filepath = basepath / "data"
    

    filenames=[]
    WRITE_TPOSE = True

    for filename in filepath.iterdir():
        if "_" in str(filename):
            latest_number = str.split(str(filename),"_")[1]
            filenames.append(int(latest_number))
        
        if "Tpose" in str(filename): WRITE_TPOSE=False
    if filenames: episode_number = max(filenames)+1
    else: episode_number=1
    
    ## Initalize ## 
    robot = DemoClass()
    joints = init_joint
    
    ## Data Extract
    while episode_number<epochs:
        print("episode_number:",episode_number)
        # repeat_test
        
        # Deploy
        json_name = filepath / "dataset_{}_.json".format(episode_number)
        homepose_pmove_capture(robot,None,json_name)


        episode_number = episode_number + 1
    
    import os
    # os.system('shutdown now - sf')