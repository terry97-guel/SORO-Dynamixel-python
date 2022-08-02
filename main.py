import rospy 
import numpy as np 
import math
from class_demo import DemoClass
import json 

PI = math.pi
init_joint = [0.6307, -1.57, 1.0, 
            -1, PI/2, 0]
Limit = 2000
soroMove_per_urMove = 5
epochs = 1

json_name = "0710_dataset.json"

with open(json_name, "w") as outfile:
    pass



if __name__=="__main__":
    robot = DemoClass()
    for i in range(epochs):
        joints = init_joint 
        joints[-1] = math.pi * 0.5          # wrist 3   [-PI,PI]
        joints[-2] = 3/2*math.pi        #wrist2    [0,2PI]
        joints[-3] = -1    # wrist1 -math.pi math.pi [0,PI]
        robot.execute_arm(joints)
        currpos=robot.xc.get_currpos()
        for soroMove_Idx in range(soroMove_per_urMove):
            delta = (np.random.rand(4)) * np.array([1,-1,1,-1]) * Limit
            delta = delta.astype(np.int32)
            pos = currpos + delta
            robot.xc.set_goalposcluster(pos,1)
            print('pos',robot.xc.get_currpos())

            # print("vicon base pose",robot.vicon_base.transform_data)
            base_vicon = robot.vicon_base.transform_data
            base_vicon_trans = base_vicon.translation # ee_vicon_trans.x,y,z
            base_vicon_rot   = base_vicon.rotation # ee_vicon_rot.x,y,z,w

            markers = robot.vicon_marker.markers
            # marker_name = markers[0].marker_name
            # marker_trans = markers[0].translation 
            
            
            base_dict = dict(
                base_trans_x = float(base_vicon_trans.x),
                base_trans_y = float(base_vicon_trans.y),
                base_trans_z = float(base_vicon_trans.z),
                base_rotate_x = float(base_vicon_rot.x),
                base_rotate_y = float(base_vicon_rot.y),
                base_rotate_z = float(base_vicon_rot.z),
                base_rotate_w = float(base_vicon_rot.w),
                
            )
            
            markers_dict = {}
            for marker in markers:
                marker_name = str(marker.marker_name)
                
                single_dict=dict(
                    x = float(marker.translation.x),
                    y = float(marker.translation.y),
                    z = float(marker.translation.z)
                )
                markers_dict[marker_name] = single_dict
                
            
            
            integrated_dict = dict(
                base_dict=base_dict,markers_dict=markers_dict
                )
            
            integrated_dict['ur_joints']      = joints
            # integrated_dict['soro_actuation'] = list(delta)
            
            json_object = json.dumps(integrated_dict)
  
            # Writing to sample.json
            with open(json_name, "a") as outfile:
                outfile.write(json_object)
            