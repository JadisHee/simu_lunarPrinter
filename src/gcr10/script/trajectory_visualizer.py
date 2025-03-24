import rospy
import tf
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker

rospy.init_node("joint_trajectory_marker")
pub = rospy.Publisher("/vis_marker", Marker ,queue_size=10)
listener = tf.TransformListener()

marker = Marker()
marker.header.frame_id = "agv_base_link"
marker.type = Marker.LINE_STRIP
marker.scale.x = 0.01
marker.color.a = 1.0
marker.color.r = 1.0


marker.pose.orientation.w = 1
marker.pose.orientation.x = 0
marker.pose.orientation.y = 0
marker.pose.orientation.z = 0
rate = rospy.Rate(10)
while not rospy.is_shutdown():
    try:
        trans, _ = listener.lookupTransform("agv_base_link","link_7", rospy.Time(0))

        point = Point()
        point.x, point.y, point.z = trans
        marker.points.append(point)

        marker.header.stamp = rospy.Time.now()
        pub.publish(marker)

    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        rospy.logwarn('no TF data, waiting...')

    rate.sleep()
