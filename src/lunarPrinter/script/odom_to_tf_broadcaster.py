import rospy
import tf
from nav_msgs.msg import Odometry

def odom_callback(msg):
    br = tf.TransformBroadcaster()

    position = msg.pose.pose.position
    orientation = msg.pose.pose.orientation

    br.sendTransform(
                    (position.x, position.y, position.z),
                    (orientation.x, orientation.y, orientation.z, orientation.w),
                    rospy.Time.now(),
                    "agv_base_link",
                    "world")

if __name__ == "__main__":
    rospy.init_node('odom_to_tf_broadcaster')
    rospy.Subscriber("/agv/odom", Odometry, odom_callback)
    rospy.spin()
