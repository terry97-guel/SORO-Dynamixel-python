import rospy
import time
# from vicon_bridge.msg import Markers
from geometry_msgs.msg._PoseArray import PoseArray
# from geometry_msgs.msg.

class OptiTrack(object):
    def __init__(self):
        self.markers = None
        # self.tick = 0
        self.markers_sub = rospy.Subscriber('/optitrack/unlabeled/markerPoseArray', PoseArray, self.callback)
        # rospy.spin()

        # tic_temp = 0
        # while self.tick<5:
        #     time.sleep(1e-3)
        #     tic_temp = tic_temp + 1
        #     if tic_temp > 5000:
        #         print ("[ERROR] Vicon Markers")
        #         break
        
    def callback(self, data):
        # print(data)
        self.markers = data.poses


if __name__ == "__main__":
    rospy.init_node("Test")
    V = OptiTrack()
    rospy.sleep(1)
    print(V.markers)