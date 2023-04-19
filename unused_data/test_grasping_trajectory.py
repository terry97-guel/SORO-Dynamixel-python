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

filepath = Path("results/test_grasp")
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
    
    rollout_actuation = np.load("actuations/Custom_grasp.npy")
    for delta in rollout_actuation:
        print(episode_number,"/",len(rollout_actuation))
        # zeropos = np.array([0,0,0,0])
        # robot.pmove_xc(zeropos)
        
        robot.pmove_xc(delta)
        print('pos',robot.xc.get_currpos())
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
        json_name = filepath / "dataset_{}_.json".format(episode_number)
        episode_number = episode_number + 1

        with open(str(json_name), 'w') as outfile:
            outfile.write(json_object)
    robot.pmove_xc(np.array([0,0,0,0]))
else:
    # %%
    from matplotlib import pyplot as plt
    import json
    from pathlib import Path
    import numpy as np
    import os
    
    filepath = "results/test_grasp"

    target_traj= np.array(
        [[  1.59404427,  19.95656639, 116.58492684],
        [ 25.901793, -11.341606,  96  ],
        [-42.855343, -16.68693 ,  96],
        [-44.95914 ,  25.406265,  96],
        [-21.993893,  31.382328,  96 ],
        [-13.039719,  53.085144,  90],
        [ 31.170439,  39.37284 ,  96 ],
        [  1.59404427,  19.95656639, 116.58492684]]
    )

    EE_positions = []
    EE_positions.append(target_traj[0]/1000)
    for path_idx in range(1,len(os.listdir(filepath))+1):
        path = "results/test_grasp/dataset_{}_.json".format(str(path_idx))

        with open(str(path), 'r') as json_file:
            dataset = json.load(json_file)

        ## Frame
        M_R_G = np.array([[-1,0,0],[0,0,1],[0,-1,.0]])
        # M_R_G = np.array([[1,0,0],[0,0,-1],[0,1,.0]])
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
        M_p_EE = np.zeros(3)
        
        M_p_EE[0] = EE_marker_list[0]['x']
        M_p_EE[1] = EE_marker_list[0]['y']
        M_p_EE[2] = EE_marker_list[0]['z']

        
        M_p_plat2EE = M_p_EE - M_p_plat
        G_p_plat2EE = G_R_M @ M_p_plat2EE
        EE_positions.append(G_p_plat2EE)

    EE_positions.append(target_traj[0]/1000)
    EE_positions = np.array(EE_positions) * 1000


    # %%
    from matplotlib.backends.backend_agg import FigureCanvasAgg as FigureCanvas
    from matplotlib.figure import Figure

    fig = plt.figure(figsize=(20,20))

    birdeye = False

    # Bird eye view
    center = target_traj[0]
    center_x = center[0] - 60
    center_y = center[1] + 50
    center_z = center[2] - 30
    theta = np.linspace( 11/8 * np.pi, 2 * np.pi+0.3, 201)
    radius = 80
    x = radius*np.cos(theta) + center_x
    y = radius*np.sin(theta) + center_y
    z = center_z

    ax = fig.add_subplot()
    ax.set_title('Trajectory Following', fontsize=40)
    ax.axis('equal')
    EE = ax.plot(EE_positions[:,0],EE_positions[:,1], '--',color='g',label="End-Effector")
    target = ax.plot(target_traj[:,0],target_traj[:,1],color='r',label="Target")
    names = ['Neutral','Ungrasp', 'Approach', 'Grasp', 'Twist Half1', 'Twist Half2', 'Lift']
    for i in range(len(EE_positions)-1):
        ax.scatter(EE_positions[i,0],EE_positions[i,1],s=100,label=names[i])
    circle = ax.plot(x,y,color='k', linewidth=3)
    plt.legend(fontsize=30, loc=1)
    plt.show()
    
    
    # # Front 3D
    fig = plt.figure(figsize=(20,20))
    ax = fig.add_subplot(projection='3d')
    ax.set_title('Trajectory Following', fontsize=40)
    ax.set_xlim([-50,50])
    ax.set_ylim([-50,50])
    ax.set_zlim([30,130])
    ax.view_init(0,0)
    EE = ax.plot(EE_positions[:,0],EE_positions[:,1],EE_positions[:,2], '.-',color='g',label="End-Effector")
    target = ax.plot(target_traj[:,0],target_traj[:,1],target_traj[:,2],color='r',label="Target")
    colors=['r','g','b']*2
    
    names = ['Neutral','Ungrasp', 'Approach', 'Grasp', 'Twist Half1', 'Twist Half2', 'Lift']
    for i in range(len(EE_positions)-1):
        ax.scatter(EE_positions[i,0],EE_positions[i,1],EE_positions[i,2],label=names[i],s=100)
    plt.legend(fontsize=30, loc=1)
    plt.show()
    
    # Perspective 3D
    fig = plt.figure(figsize=(20,20))
    ax = fig.add_subplot(projection='3d')
    ax.set_title('Trajectory Following', fontsize=40)
    ax.set_xlim([-50,50])
    ax.set_ylim([-50,50])
    ax.set_zlim([30,130])
    ax.view_init(30,30)
    
    # plot EE
    EE = ax.plot(EE_positions[:,0],EE_positions[:,1],EE_positions[:,2], '.-',color='g',label="End-Effector")
    target = ax.plot(target_traj[:,0],target_traj[:,1],target_traj[:,2],color='r',label="Target")
    
    colors=['r','g','b']*2
    
    names = ['Neutral','Ungrasp', 'Approach', 'Grasp', 'Twist Half1', 'Twist Half2', 'Lift']
    for i in range(len(EE_positions)-1):
        ax.scatter(EE_positions[i,0],EE_positions[i,1],EE_positions[i,2],label=names[i],s=100)
    plt.legend(fontsize=30, loc=1)
    plt.show()
    print("\n")


    # %%

