import rospy
from gazebo_msgs.msg import LinkStates
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import sys
import termios
import tty


drawing = False
trajectory_points = []
link_width = 0.02

def get_key():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        key = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return key


def link_states_callback(msg, marker_pub):
    global drawing, trajectory_points

    if "lunarPrinter.xacro::link_6" not in msg.name:
        rospy.logwarn("link_6 not found in /gazebo/link_states")
        return



    
    index = msg.name.index("lunarPrinter.xacro::link_6")
    pos = msg.pose[index].position
    # rospy.loginfo(f"lunarPrinter.xacro::link_6 position: x={pos.x}, y={pos.y}, z={pos.z}")

    if drawing:
        trajectory_points.append(Point(pos.x, pos.y, pos.z))

    marker = Marker()
    marker.header.frame_id = "world"
    marker.header.stamp = rospy.Time.now()
    marker.ns = "trajectory"
    marker.id = 0
    marker.type = Marker.LINE_STRIP
    marker.action = Marker.ADD


    marker.color.r = 1
    marker.color.g = 0
    marker.color.b = 0
    marker.color.a = 1

    marker.scale.x = link_width

    marker.points = trajectory_points

    marker_pub.publish(marker) 

def main():
    global drawing
 
    rospy.init_node("gazebo_trajjectory_drawer")

    marker_pub = rospy.Publisher("/trajectory_marker", Marker, queue_size=10)

    rospy.Subscriber("/gazebo/link_states", LinkStates, link_states_callback, marker_pub)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        key = get_key()
        if key == " ":
            drawing = not drawing
            rospy.loginfo(f"Trajectory drawing: {'ON' if drawing else 'OFF'}")
        rate.sleep()

if __name__ == "__main__":
    main()



