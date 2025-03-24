#!/usr/bin/env python3
import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from DucoCtrl import DucoCtrl

duco = DucoCtrl(ipAddr='192.168.2.55',Port=7003)




def send_joint_angles(joints):
    # 初始化节点
    rospy.init_node('joint_position_commander')
    
    # 创建发布者
    pub = rospy.Publisher('/arm_controller/command', JointTrajectory, queue_size=10)
    
    # 等待连接
    rospy.sleep(0.1)
    
    # 构造消息
    msg = JointTrajectory()
    
    # 设置关节名称 (必须与URDF中的名称完全一致)
    msg.joint_names = [
        "joint_1",
        "joint_2",
        "joint_3",
        "joint_4",
        "joint_5",
        "joint_6"
    ]
    
    # 创建轨迹点
    point = JointTrajectoryPoint()
    
    # 设置目标位置 [1.52, 1, 3, 1, 1, 1] (单位：弧度)
    point.positions = joints
    
    # 设置到达目标的时间 (单位：秒)
    point.time_from_start = rospy.Duration(0.1)  # 2秒内到达
    
    # 添加轨迹点
    msg.points.append(point)
    
    # 发布消息
    pub.publish(msg)
    rospy.loginfo("Command sent: {}".format(point.positions))

if __name__ == '__main__':
    try:
        while(1):
            joints = duco.GetDucoJoints()
            send_joint_angles(joints)
    except rospy.ROSInterruptException:
        pass

    # try:
    #     # joints = duco.GetDucoJoints()
    #     send_joint_angles([0, 0.8, 1, 0, 0, 0])
    # except rospy.ROSInterruptException:
    #     pass
