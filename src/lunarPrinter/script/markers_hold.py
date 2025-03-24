import rospy
import copy
from visualization_msgs.msg import MarkerArray

old_markers = MarkerArray()

def markers_cb(msg):
    global old_markers
    markers_len = len(old_markers.markers)
    print(markers_len)

    marker_temp = msg.markers[1:]

    # if markers_len != 0:
            # markers_len = markers_len -1
    for i in range(len(marker_temp)):
        
        marker_temp[i].id = markers_len + i
        marker_temp[i].ns = str(markers_len + i)
    old_markers.markers.extend(marker_temp)

if __name__ == "__main__":

    rospy.init_node('markers_holding')

    markers_sub = rospy.Subscriber('/tool/marker_full',MarkerArray, markers_cb)

    old_markers_pub = rospy.Publisher('/tool/old_markers',MarkerArray, queue_size=10)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        old_markers_pub.publish(old_markers)
        rate.sleep()







