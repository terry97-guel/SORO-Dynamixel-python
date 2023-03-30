import rospy 
import numpy as np 
import math
from class_demo import DemoClass
import json 
import random
import sys
import time
from pathlib import Path


def homepose_pmove_capture(robot:DemoClass, delta, json_name):
    robot.pmove_xc(np.zeros_like(delta))
    time.sleep(1)
    
    print('delta',delta)
    robot.pmove_xc(delta)
    time.sleep(3)
    
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
    integrated_dict['soro_actuation'] = delta.tolist()
    
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
    epochs = 10_000

    basepath = Path("results/1002")
    filepath = basepath / "data"
    repeat_test_path = basepath / "repeat_test"

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
    robot.init_xc(np.array([None]))
    initpos = robot.xc.get_currpos()
    pos = initpos + np.array([1,1,1,1])*1000 * np.array([1,-1,-1,1])
    robot.move_xc(pos)

    initpos = np.array([4304,  777, 2907, 3617])
    robot.init_xc(initpos)
    
    init_pos=robot.xc.get_currpos()
    joints = init_joint
    
    ## Data Extract
    while episode_number<epochs:
        print("episode_number:",episode_number)
        # repeat_test
        if (episode_number)%100==1:
            for temp in range(1):
                delta = np.array([0,0,0,0])
                json_name = repeat_test_path / "zero_{}_.json".format(episode_number+temp)
                homepose_pmove_capture(robot,delta, json_name)

                delta = np.array([1000,0,0,0])
                json_name = repeat_test_path / "first_{}_.json".format(episode_number+temp)
                homepose_pmove_capture(robot,delta, json_name)
                
                delta = np.array([0,1000,0,0])
                json_name = repeat_test_path / "second_{}_.json".format(episode_number+temp)
                homepose_pmove_capture(robot,delta, json_name)
                
                delta = np.array([0,0,1000,0])
                json_name = repeat_test_path / "third_{}_.json".format(episode_number+temp)
                homepose_pmove_capture(robot,delta, json_name)
                
                delta = np.array([0,0,0,1000])
                json_name = repeat_test_path / "fourth_{}_.json".format(episode_number+temp)
                homepose_pmove_capture(robot,delta, json_name)
        
        # Deploy
        delta = (np.random.rand(4)) * Limit
        while sum(delta) > 3000:
            delta = (np.random.rand(4)) * Limit

        delta = delta.astype(np.int32)
        json_name = filepath / "dataset_{}_.json".format(episode_number)
        homepose_pmove_capture(robot,delta,json_name)


        episode_number = episode_number + 1
    
    import os
    # os.system('shutdown now - sf')