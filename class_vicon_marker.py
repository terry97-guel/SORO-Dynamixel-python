import rospy
import time
from vicon_bridge.msg import Markers

class VICON(object):
    def __init__(self):
        self.markers = None
        self.tick = 0
        self.markers_sub = rospy.Subscriber('/vicon/markers', Markers, self.callback)
        tic_temp = 0
        while self.tick<5:
            time.sleep(1e-3)
            tic_temp = tic_temp + 1
            if tic_temp > 5000:
                print ("[ERROR] Vicon Markers")
                break
            
    def callback(self, data):
        self.markers = data.markers


if __name__ == "__main__":
    V = VICON()
    rospy.sleep(1)
    print(V.markers)