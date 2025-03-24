#!/usr/bin/env python

import rospy
import argparse
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Header

def publish_trajectory(height):
    # 初始化ROS节点
    rospy.init_node('trajectory_publisher', anonymous=True)
    
    # 创建Publisher，指定话题和消息类型
    pub = rospy.Publisher('/arm_base_controller/command', JointTrajectory, queue_size=10)
    
    # 等待Publisher建立连接
    rospy.sleep(0.5)
    
    # 创建JointTrajectory消息
    msg = JointTrajectory()
    
    # 设置header
    msg.header = Header(
        seq=0,
        stamp=rospy.Time(0),  # 时间戳设置为0
        frame_id=''
    )
    
    # 设置关节名称
    msg.joint_names = [
        'arm_base_1_joint'
    ]
    
    # 创建轨迹点
    point = JointTrajectoryPoint()
    point.positions = [height]  # 所有位置设为0
    point.time_from_start = rospy.Duration(2.0)  # 2秒时间间隔
    
    # 将轨迹点添加到消息中
    msg.points = [point]
    
    # 发布消息
    pub.publish(msg)
    rospy.loginfo("Trajectory message published")

if __name__ == '__main__':
    try:
        parser = argparse.ArgumentParser()
        parser.add_argument("height",type=float)
        args = parser.parse_args()
        publish_trajectory(args.height)
    except rospy.ROSInterruptException:
        pass