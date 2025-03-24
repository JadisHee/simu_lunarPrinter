#!/usr/bin/env python3
import rospy
import open3d as o3d
import numpy as np
import tf.transformations as tf
from geometry_msgs.msg import Pose
# from moveit_commander import MoveGroupCommander, PlanningSceneInterface
import moveit_commander



def move_to_pose(tail):
    # 初始化节点和MoveIt组件
    rospy.init_node('moveit_pose_control')

    robot = moveit_commander.RobotCommander()
    group = moveit_commander.MoveGroupCommander("lunar_arm")

    group.set_max_velocity_scaling_factor(0.3)
    group.set_planning_time(10)


    # 设置目标位姿
    target_pose = Pose()

    q = tf.quaternion_from_euler(3.1415926, 0.0, 3.1415926)  # rx=0, ry=90度, rz=0
    target_pose.orientation.x = q[0]
    target_pose.orientation.y = q[1]
    target_pose.orientation.z = q[2]
    target_pose.orientation.w = q[3]










    # scene = PlanningSceneInterface()
    # group = MoveGroupCommander("lunar_arm")  # 替换为你的规划组名称

    # group.set_pose_reference_frame('arm_base_link')
    

    # target_pose.header.frame_id = "arm_base_link"  # 必须与机器人基座坐标系一致
    
    # 将欧拉角转换为四元数（rx, ry, rz单位为弧度）

    
    
    for i in range(len(tail)):

        target_pose.position.x = tail[i][0]  # 单位：米
        target_pose.position.y = tail[i][1] - 2.5
        target_pose.position.z = tail[i][2] 

        # waypoints.append(target_pose)


        # # 设置规划参数
        # group.set_pose_target(target_pose)
        # group.set_max_velocity_scaling_factor(0.5)  # 降低速度避免抖动
        # group.set_planning_time(10.0)              # 增加规划时间
    
        # # 执行规划
        # plan = group.go(wait=True)
        (plan, fraction) = group.compute_cartesian_path(
            [target_pose],
            0.01,
        )

        if fraction > 0.9:
            group.execute(plan, wait=True)

    group.stop()
    group.clear_pose_targets()
    # if plan.joint_trajectory.points:
    #     group.execute(plan, wait=True)
    #     rospy.loginfo("Motion completed!")
    # else:
    #     rospy.logwarn("Planning failed!")

if __name__ == '__main__':
    try:
        tail_ptCloud = o3d.io.read_point_cloud('/home/space/work/1_DucoMould/tail_points/gcode_wall_5_2.pcd')
            # tail_len = len(tail_ptCloud.points)
        tail_points = np.asarray(tail_ptCloud.points) / 1000
        print(len(tail_points))
        # tail_points = np.array([
        #     # [0.000, -1.862, 2.720],
        #     [0.5,-2.0,0.0],
        #     [0.5,-1.5,0.0],
        #     [-0.5,-1.5,0.0],
        #     [-0.5,-2.0,0.0]
        #     # [-2.0,-2.0,0.0],
        #     # [0.000, -1.862, 2.720]
        #     # [2,-2,-2],
        #     # [-2,-2,-2]
        #     ])

        # tail_points = np.array([
        #     # [0.000, -1.862, 2.720],
        #     [2.0,-2.0,0.0],
        #     [2.0,2.0,0.0],
        #     [2.0,-2.0,0.0],
        #     [0.000, -1.862, 2.720]
        #     # [2,-2,-2],
        #     # [-2,-2,-2]
        #     ])

        move_to_pose(tail_points)
    except rospy.ROSInterruptException:
        pass