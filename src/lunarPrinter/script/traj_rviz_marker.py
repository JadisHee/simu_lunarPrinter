import rospy
import tf
import sys
import select
import termios
import tty
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker

# 保存终端原始设置
settings = termios.tcgetattr(sys.stdin)

rospy.init_node("joint_trajectory_marker")
pub = rospy.Publisher("/vis_marker", Marker, queue_size=10)
listener = tf.TransformListener()

# 初始化标记参数
marker = Marker()
marker.header.frame_id = "world"
marker.type = Marker.LINE_STRIP
marker.scale.x = 0.01
marker.color.a = 1.0
marker.color.r = 1.0
marker.pose.orientation.w = 1

rate = rospy.Rate(10)
paused = False  # 控制暂停状态

# 设置终端为cbreak模式
tty.setcbreak(sys.stdin.fileno())

def reset_terminal():
    """程序退出时恢复终端设置"""
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

rospy.on_shutdown(reset_terminal)

try:
    while not rospy.is_shutdown():
        # 处理键盘输入
        key = None
        try:
            if select.select([sys.stdin], [], [], 0)[0]:
                key = sys.stdin.read(1)
                if key == ' ':
                    paused = not paused
                    rospy.loginfo("轨迹记录已 {}.".format('暂停' if paused else '继续'))
        except Exception as e:
            rospy.logerr("键盘输入错误: {}".format(e))

        # 更新并发布标记
        try:
            if not paused:
                # 获取当前末端位置
                trans, _ = listener.lookupTransform("world", "link_7", rospy.Time(0))
                point = Point()
                point.x, point.y, point.z = trans
                marker.points.append(point)

            # 始终更新并发布标记（保持轨迹显示）
            marker.header.stamp = rospy.Time.now()
            pub.publish(marker)

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logwarn("TF数据获取失败，等待中...")
        except Exception as e:
            rospy.logerr("发生未预期错误: {}".format(e))

        rate.sleep()
finally:
    reset_terminal()  # 确保终端设置被恢复