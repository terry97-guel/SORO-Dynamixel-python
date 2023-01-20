import pyrealsense2 as rs   
import cv2
import apriltag
import numpy as np
from class_vicon_marker import VICON
import rospy

class UnseenObjectClustering(object):
    def __init__(self) -> None:
        pass

pipeline = rs.pipeline()
config = rs.config()
align_to = rs.stream.color
align = rs.align(align_to)
pipeline_wrapper = rs.pipeline_wrapper(pipeline)
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
pipeline.start(config)

depth_camera_pos_list = []
mocap_pos_list = []
keep_collecting = 'y'

count = 0
frames = pipeline.wait_for_frames()
depth_frame = frames.get_depth_frame()
color_frame = frames.get_color_frame()
aligned_frames = align.process(frames)
aligned_depth_frame = aligned_frames.get_depth_frame()
depth_intrin = aligned_depth_frame.profile.as_video_stream_profile().intrinsics
depth_image = np.asanyarray(depth_frame.get_data())
color_image = np.asanyarray(color_frame.get_data())    

gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
options = apriltag.DetectorOptions(families="tag36h11")
detector = apriltag.Detector(options)
results = detector.detect(gray)
pos_list = []
for r in results:
    # extract the bounding box (x, y)-coordinates for the AprilTag
    # and convert each of the (x, y)-coordinate pairs to integers
    (ptA, ptB, ptC, ptD) = r.corners
    ptB = (int(ptB[0]), int(ptB[1]))
    ptC = (int(ptC[0]), int(ptC[1]))
    ptD = (int(ptD[0]), int(ptD[1]))
    ptA = (int(ptA[0]), int(ptA[1]))
    # draw the bounding box of the AprilTag detection
    if True : 
        cv2.circle(color_image, (ptA[0], ptA[1]), 5, (255, 0, 0), -1)
        cv2.line(color_image, ptA, ptB, (0, 255, 0), 2)
        cv2.line(color_image, ptB, ptC, (0, 255, 0), 2)
        cv2.line(color_image, ptC, ptD, (0, 255, 0), 2)
        cv2.line(color_image, ptD, ptA, (0, 255, 0), 2)
    # draw the center (x, y)-coordinates of the AprilTag
    # (cX, cY) = (int(r.center[0]), int(r.center[1]))
    depth = depth_frame.get_distance(int(ptA[0]),int(ptA[1]))
    x,y,z = rs.rs2_deproject_pixel_to_point(depth_intrin,pixel=[int(ptA[0]),int(ptA[1])],depth=depth)
    pos_list.append([x,y,z])
    # center_pos = np.array([[cX, cY]])
    # if True : 
        # cv2.circle(color_image, (cX, cY), 5, (0, 0, 255), -1)

# Read from left
if len(pos_list)==1:
    
    cv2.imshow("Frame", color_image)
    count = count+1
    import time
    time.sleep(0.1)
    
    if count % 3 ==0:
        
        mocap_marker_pos = vicon_marker.markers 
        if len(mocap_marker_pos)!=1: continue
        
        print("Depth Camera, x:{:.2f}, y:{:.2f}, z:{:.2f}".format(
            pos_list[0][0],pos_list[0][1],pos_list[0][2]))
        print("Mocap,        x:{:.2f}, y:{:.2f}, z:{:.2f}".format(
            vicon_marker.markers[0].position.x,vicon_marker.markers[0].position.y,vicon_marker.markers[0].position.z))
        
        
        print("collect this point? Answer after move... 'y' for yes, 'b' for break, any for no")
        collect = input()
        
        if collect =='b': break
        elif collect=='y':
            depth_camera_pos_list.append(pos_list[0])
            mocap_pos_list.append([vicon_marker.markers[0].position.x,vicon_marker.markers[0].position.y,vicon_marker.markers[0].position.z])
        else: continue
    
else:
    cv2.imshow("Frame", color_image)
    count = count+1
    import time
    time.sleep(1)

if cv2.waitKey(20) == 27:
    break
