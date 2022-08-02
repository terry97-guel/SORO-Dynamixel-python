import rospy
import time
from geometry_msgs.msg import TransformStamped

class ViconBase(object):
    def __init__(self):
        self.transform_data = None 
        self.tick = 0
        self.transform_data_sub = rospy.Subscriber('/vicon/BASE_1/BASE_1', TransformStamped, self.callback)

        tic_temp = 0
        while self.tick<2:
            time.sleep(1e-3)
            tic_temp = tic_temp + 1
            if tic_temp > 5000:
                print ("[ERROR] Vicon Base")
                break
      
    def callback(self, data):
        # rospy.loginfo(rospy.get_name())
        # for marker in data.markers:
        #     rospy.loginfo(marker.translation)
        self.transform_data = data.transform
        


if __name__ == "__main__":
    rospy.init_node("Test")
    V = ViconBase()
    V.listener()
    time.sleep(0.1)
    print(V.transform_data)
    V.listener()
    time.sleep(0.1)
