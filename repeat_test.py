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
            -DEG90, DEG90, 0]

Limit = 2000
epochs = 4000

filepath = Path("results/repeat_test_temp")
episode_number=1

if __name__=="__main__":
    ## Initalize
    robot = DemoClass()
    robot.init_xc(np.array([None]))
    initpos = robot.xc.get_currpos()
    pos = initpos + np.array([1,1,1,1])*1000 * np.array([1,-1,-1,1])
    robot.move_xc(pos)

    initpos = np.array([4150,  970, 3106, 3197])
    robot.init_xc(initpos)
    
    init_pos=robot.xc.get_currpos()
    joints = init_joint
    
    deltas = [np.array([1000,0,0,0]),np.array([0,1000,0,0]),np.array([0,0,1000,0]),np.array([0,0,0,1000])]
    for delta in deltas:
        for _ in range(2):
            zeropos = np.array([0,0,0,0])
            robot.pmove_xc(zeropos)
            
            robot.pmove_xc(delta)
            print('pos',robot.xc.get_currpos())
            time.sleep(4)
            
            markers = robot.vicon_reference.markers
            # reference_markers_list=[]
            # for marker in markers:
            #     single_dict=dict(
            #         x = float(marker.position.x),
            #         y = float(marker.position.y),
            #         z = float(marker.position.z)
            #     )
            #     reference_markers_list.append(single_dict)
            
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
                # reference_markers_list=reference_markers_list
                )
            
            integrated_dict['ur_joints']      = joints
            integrated_dict['soro_actuation'] = delta.tolist()
            
            json_object = json.dumps(integrated_dict)

            # Writing to json
            json_name = filepath / "dataset_{}_.json".format(episode_number)
            episode_number = episode_number + 1

            with open(str(json_name), 'w') as outfile:
                outfile.write(json_object)
    

# %% 

import json
import numpy as np

def get_M_p_EE(dataset):
    platform_marker_list = dataset['platform_markers_list']
    platform_markers_array = np.zeros((len(platform_marker_list),3))
    for idx in range(len(platform_markers_array)):
        platform_markers_array[idx][0] = platform_marker_list[idx]['x']
        platform_markers_array[idx][1] = platform_marker_list[idx]['y']
        platform_markers_array[idx][2] = platform_marker_list[idx]['z']
    M_p_plat = np.mean(platform_markers_array,axis=0)

    EE_marker_list = dataset['unlabeled_markers_list']
    soro_actuation = dataset['soro_actuation']
    assert len(EE_marker_list)==1
    M_p_EE = np.zeros(3)
    M_p_EE[0] = EE_marker_list[0]['x']
    M_p_EE[1] = EE_marker_list[0]['y']
    M_p_EE[2] = EE_marker_list[0]['z']

    return M_p_EE - M_p_plat


# Check Consistency

legacyfilepath = ["dataset_1_.json","dataset_11_.json","dataset_21_.json","dataset_31_.json"]
comparefilepath = ["dataset_1_.json","dataset_3_.json","dataset_5_.json","dataset_7_.json"]

for i in range(4):
    filepath = "results/repeat_test/"+legacyfilepath[i]
    with open(filepath, "r") as st_json:
        legacy_dataset = json.load(st_json)
    legacy_M_p_EE = get_M_p_EE(legacy_dataset)

    filepath = "results/repeat_test_temp/"+comparefilepath[i]
    with open(filepath, "r") as st_json:
        compare_dataset = json.load(st_json)
    compare_M_p_EE = get_M_p_EE(compare_dataset)

    print("Length in each Frame",np.linalg.norm(legacy_M_p_EE), np.linalg.norm(compare_M_p_EE))
    print("Amount of Error: ",(legacy_M_p_EE-compare_M_p_EE)*1000, "norm: ",np.linalg.norm(legacy_M_p_EE-compare_M_p_EE)*1000,"mm")

print("##############")

# %%
legacyfilepath = ["dataset_1_.json","dataset_11_.json","dataset_21_.json","dataset_31_.json"]
comparefilepath = ["dataset_2_.json","dataset_4_.json","dataset_6_.json","dataset_8_.json"]
for i in range(4):
    filepath = "results/repeat_test/"+legacyfilepath[i]
    with open(filepath, "r") as st_json:
        legacy_dataset = json.load(st_json)
    legacy_M_p_EE = get_M_p_EE(legacy_dataset)

    filepath = "results/repeat_test_temp/"+comparefilepath[i]
    with open(filepath, "r") as st_json:
        compare_dataset = json.load(st_json)
    compare_M_p_EE = get_M_p_EE(compare_dataset)

    print("Length error between two Frames",np.linalg.norm(legacy_M_p_EE)-np.linalg.norm(compare_M_p_EE))
    print("Amount of Error: ",(legacy_M_p_EE-compare_M_p_EE)*1000, "norm: ",np.linalg.norm(legacy_M_p_EE-compare_M_p_EE)*1000,"mm")