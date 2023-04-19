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

filepath = Path("results/rollout")
episode_number=1

if __name__=="__main__":
    ## Initalize
    robot = DemoClass()
    robot.init_xc(np.array([None]))
    initpos = robot.xc.get_currpos()
    pos = initpos + np.array([1,1,1,1])*1000 * np.array([1,-1,-1,1])
    robot.move_xc(pos)

    initpos = np.array([4110,  970, 3106, 3197])
    robot.init_xc(initpos)
    
    init_pos=robot.xc.get_currpos()
    joints = init_joint
    
    rollout_actuation = np.load("actuations/Circle_video.npy")
    # rollout_actuation = np.load("actuations/large_square_video.npy")
    # deltas = [np.array([1000,0,0,0]),np.array([0,1000,0,0]),np.array([0,0,1000,0]),np.array([0,0,0,1000])]
    for delta in rollout_actuation:
        print(episode_number,"/",len(rollout_actuation))
        # zeropos = np.array([0,0,0,0])
        # robot.pmove_xc(zeropos)
        
        robot.pmove_xc(delta)
        print('pos',robot.xc.get_currpos())
        time.sleep(1)
        
        markers = robot.vicon_reference.markers
        reference_markers_list=[]
        # for marker in markers:
        #     single_dict=dict(
        #         x = float(marker.position.x),
        #         y = float(marker.position.y),
        #         z = float(marker.position.z)
        #     )
        #     reference_markers_list.append(single_dict)
        
        # markers = robot.vicon_platform.markers
        # platform_markers_list=[]
        # for marker in markers:
        #     single_dict=dict(
        #         x = float(marker.position.x),
        #         y = float(marker.position.y),
        #         z = float(marker.position.z)
        #     )
        #     platform_markers_list.append(single_dict)
        
        # markers = robot.vicon_marker.markers
        # unlabeled_markers_list=[]
        # for marker in markers:
        #     single_dict=dict(
        #         x = float(marker.position.x),
        #         y = float(marker.position.y),
        #         z = float(marker.position.z)
        #     )
        #     unlabeled_markers_list.append(single_dict)
            
        
        # integrated_dict = dict(
        #     unlabeled_markers_list=unlabeled_markers_list,
        #     platform_markers_list=platform_markers_list,
        #     reference_markers_list=reference_markers_list
        #     )
        
        # integrated_dict['ur_joints']      = joints
        # integrated_dict['soro_actuation'] = delta.tolist()
        
        # json_object = json.dumps(integrated_dict)

        # # Writing to json
        # json_name = filepath / "dataset_{}_.json".format(episode_number)

        # with open(str(json_name), 'w') as outfile:
        #     outfile.write(json_object)
        
        episode_number = episode_number + 1
    
else:
    # %%
    from matplotlib import pyplot as plt
    import json
    from pathlib import Path
    import numpy as np
    import os
    
    filepath = "results/rollout_temp"


    EE_positions = []
    for path_idx in range(1,len(os.listdir(filepath))+1):
        path = str(filepath)+"/dataset_{}_.json".format(str(path_idx))

        with open(str(path), 'r') as json_file:
            dataset = json.load(json_file)

        ## Frame
        M_R_G = np.array([[-1,0,0],[0,0,1],[0,-1,.0]])
        G_R_M = M_R_G.T
        
        # Platform
        platform_marker_list = dataset['platform_markers_list']
        platform_markers_array = np.zeros((len(platform_marker_list),3))
        dataset['reference_markers_list']

        for idx in range(len(platform_markers_array)):
            platform_markers_array[idx][0] = platform_marker_list[idx]['x']
            platform_markers_array[idx][1] = platform_marker_list[idx]['y']
            platform_markers_array[idx][2] = platform_marker_list[idx]['z']

        M_p_plat = np.mean(platform_markers_array,axis=0)
        
        # EE
        
        EE_marker_list = dataset['unlabeled_markers_list']
        if len(EE_marker_list) != 1: continue
        M_p_EE = np.zeros(3)
        
        M_p_EE[0] = EE_marker_list[0]['x']
        M_p_EE[1] = EE_marker_list[0]['y']
        M_p_EE[2] = EE_marker_list[0]['z']

        
        M_p_plat2EE = M_p_EE - M_p_plat
        G_p_plat2EE = G_R_M @ M_p_plat2EE
        EE_positions.append(G_p_plat2EE)

    EE_positions.append(EE_positions[0])
    EE_positions = np.array(EE_positions) * 1000

    # %%
    from matplotlib.backends.backend_agg import FigureCanvasAgg as FigureCanvas
    from matplotlib.figure import Figure

    # target = np.array(
    #     [[-18.348068  , -10.19674912,  94.09793727],
    #      [-18.348068  ,  23.67259935,  94.09793727],
    #      [ 17.07588509,  23.67259935,  94.09793727],
    #      [ 17.07588509, -10.19674912,  94.09793727],
    #      [-18.348068  , -10.19674912,  94.09793727]])

    target = np.array(
        [[-29.3569088 , -16.31479859,  94.09793727],
        [-29.3569088 ,  37.87615895,  94.09793727],
        [ 27.32141614,  37.87615895,  94.09793727],
        [ 27.32141614, -16.31479859,  94.09793727],
        [-29.3569088 , -16.31479859,  94.09793727]]
    )

    fig = plt.figure(figsize=(20,20))

    birdeye = True


    # Option 1
    if birdeye:
        ax = fig.add_subplot()
        ax.set_title('Trajectory Following', fontsize=40)
        # plt.axis('off')
        ax.axis('equal')
        EE = ax.plot(EE_positions[:,0],EE_positions[:,1], '--',color='g',label="End-Effector")
        target = ax.plot(target[:,0],target[:,1],color='r', label='target')
    else:
        ax = fig.add_subplot(projection='3d')
        ax.set_title('Trajectory Following', fontsize=40)
        ax.set_xlim([-50,50])
        ax.set_ylim([-50,50])
        ax.set_zlim([0,100])
        ax.view_init(50,-70)

        # plot EE
        EE = ax.plot(EE_positions[:,0],EE_positions[:,1],EE_positions[:,2], '--',color='g',label="End-Effector")


        # plot target
        target = ax.plot(target[:,0],target[:,1],target[:,2],color='r', label='target')


    # soro_line = np.stack([np.zeros_like(EE_positions[-1]),EE_positions[-1]])
    # soro = ax.plot(soro_line[:,0],soro_line[:,1],soro_line[:,2], '-',color='g')

    plt.legend(fontsize=30, loc=1)
    plt.show()

    # %%
