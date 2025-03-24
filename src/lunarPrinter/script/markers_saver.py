import rospy
import numpy as np
from visualization_msgs.msg import MarkerArray

def marker_cb(msg):
    data_list = []
    for marker in msg.markers:
        for i in range(len(marker.points)):
            x,y,z = marker.points[i].x, marker.points[i].y, marker.points[i].z
            data_list.append([x,y,z])

    np_data = np.array(data_list)

    np.savetxt("markers_data_roof.txt", np_data, fmt="%.6f")
    rospy.loginfo("Data saved!!")

    rospy.signal_shutdown("over!!")

if __name__ == "__main__":
    rospy.init_node("marker_saver", anonymous=True)
    rospy.Subscriber("/tool/old_markers",MarkerArray, marker_cb)
    rospy.spin()

