#!/bin/bash
ROS_NAME=noetic
WORKSPACE_PATH=~/work/gazebo_ws/

cd ${WORKSPACE_PATH}
source devel/setup.sh

# ##############################################################
# agv回到原点,机械臂回到原点
# ##############################################################

# 支撑腿收回 
rosrun lunarPrinter foot_ctrl.py 0
sleep 1s

# agv移动至原点
rosrun lunarPrinter agv_pose_ctrl.py 0 0 0

# 支撑腿伸出
rosrun lunarPrinter foot_ctrl.py 0.102
sleep 1s

rosrun lunarPrinter height_ctrl.py 0
# 机械臂控制回原点
rosrun lunarPrinter arm_joint_ctrl.py 0 0 0 0 0 0

# ##############################################################
# 打印底层
# ##############################################################

# 1------------------------------------

# 支撑腿收回 
rosrun lunarPrinter foot_ctrl.py 0
sleep 1s

# agv移动至原点
rosrun lunarPrinter agv_pose_ctrl.py 2.5 -2.5 0

# 支撑腿伸出
rosrun lunarPrinter foot_ctrl.py 0.102
sleep 1s

# 机械臂控制来到起始位置
rosrun lunarPrinter arm_joint_ctrl.py -1.57 0 0 0 0 0
sleep 1s
rosrun lunarPrinter arm_joint_ctrl.py -3.14159265 0 0 0 0 0

# 开始打印
rosrun lunarPrinter arm_path_planning_v3.py 1_1.txt 0
sleep 1s
rosrun lunarPrinter arm_joint_ctrl.py -3.14159265 0 0 0 0 0


# 支撑腿收回 
rosrun lunarPrinter foot_ctrl.py 0
sleep 1s
rosrun lunarPrinter agv_pose_ctrl.py 2.5 -4 0
sleep 1s
rosrun lunarPrinter agv_pose_ctrl.py -2.5 -4 0

# 2------------------------------------

# 支撑腿收回 
# rosrun lunarPrinter foot_ctrl.py 0
sleep 1s

# agv移动至原点
rosrun lunarPrinter agv_pose_ctrl.py -2.5 -2.5 0

# 支撑腿伸出
rosrun lunarPrinter foot_ctrl.py 0.102
sleep 1s

# 开始打印
rosrun lunarPrinter arm_path_planning_v3.py 2_1.txt 0
sleep 1s
rosrun lunarPrinter arm_joint_ctrl.py -3.14159265 0 0 0 0 0


# 支撑腿收回 
rosrun lunarPrinter foot_ctrl.py 0
sleep 1s
rosrun lunarPrinter agv_pose_ctrl.py -4 -2.5 0
sleep 1s
rosrun lunarPrinter agv_pose_ctrl.py -4 2.5 0
# 3------------------------------------

# 支撑腿收回 
# rosrun lunarPrinter foot_ctrl.py 0
sleep 1s

# agv移动至原点
rosrun lunarPrinter agv_pose_ctrl.py -2.5 2.5 0

# 支撑腿伸出
rosrun lunarPrinter foot_ctrl.py 0.102
sleep 1s

# 机械臂控制来到起始位置
rosrun lunarPrinter arm_joint_ctrl.py -1.57 0 0 0 0 0
sleep 1s
rosrun lunarPrinter arm_joint_ctrl.py 0 0 0 0 0 0

# 开始打印
rosrun lunarPrinter arm_path_planning_v3.py 3_1.txt 0
sleep 1s
rosrun lunarPrinter arm_joint_ctrl.py 0 0 0 0 0 0

# 支撑腿收回 
rosrun lunarPrinter foot_ctrl.py 0
sleep 1s
rosrun lunarPrinter agv_pose_ctrl.py -2.5 4 0
sleep 1s
rosrun lunarPrinter agv_pose_ctrl.py 2.5 4 0

# 4------------------------------------

# 支撑腿收回 
# rosrun lunarPrinter foot_ctrl.py 0
sleep 1s

# agv移动至原点
rosrun lunarPrinter agv_pose_ctrl.py 2.5 2.5 0

# 支撑腿伸出
rosrun lunarPrinter foot_ctrl.py 0.102
sleep 1s

# 开始打印
rosrun lunarPrinter arm_path_planning_v3.py 4_1.txt 0
sleep 1s
rosrun lunarPrinter arm_joint_ctrl.py 0 0 0 0 0 0

# 支撑腿收回 
rosrun lunarPrinter foot_ctrl.py 0
sleep 1s
rosrun lunarPrinter agv_pose_ctrl.py 4 2.5 0
sleep 1s
rosrun lunarPrinter agv_pose_ctrl.py 4 -2.5 0


# ##############################################################
# 打印中层
# ##############################################################

# 1------------------------------------

# 支撑腿收回 
rosrun lunarPrinter foot_ctrl.py 0
sleep 1s

# agv移动至原点
rosrun lunarPrinter agv_pose_ctrl.py 2.5 -2.5 0

# 支撑腿伸出
rosrun lunarPrinter foot_ctrl.py 0.102
sleep 1s

# 机械臂控制来到起始位置
rosrun lunarPrinter arm_joint_ctrl.py -1.57 0 0 0 0 0
sleep 1s
rosrun lunarPrinter arm_joint_ctrl.py -3.14159265 0 0 0 0 0

# 开始打印
rosrun lunarPrinter arm_path_planning_v3.py 1_2.txt 1
sleep 1s
rosrun lunarPrinter arm_joint_ctrl.py -3.14159265 0 0 0 0 0


# 支撑腿收回 
rosrun lunarPrinter foot_ctrl.py 0
sleep 1s
rosrun lunarPrinter agv_pose_ctrl.py 2.5 -4 0
sleep 1s
rosrun lunarPrinter agv_pose_ctrl.py -2.5 -4 0

# 2------------------------------------

# 支撑腿收回 
# rosrun lunarPrinter foot_ctrl.py 0
sleep 1s

# agv移动至原点
rosrun lunarPrinter agv_pose_ctrl.py -2.5 -2.5 0

# 支撑腿伸出
rosrun lunarPrinter foot_ctrl.py 0.102
sleep 1s

# 开始打印
rosrun lunarPrinter arm_path_planning_v3.py 2_2.txt 1
sleep 1s
rosrun lunarPrinter arm_joint_ctrl.py -3.14159265 0 0 0 0 0


# 支撑腿收回 
rosrun lunarPrinter foot_ctrl.py 0
sleep 1s
rosrun lunarPrinter agv_pose_ctrl.py -4 -2.5 0
sleep 1s
rosrun lunarPrinter agv_pose_ctrl.py -4 2.5 0
# 3------------------------------------

# 支撑腿收回 
# rosrun lunarPrinter foot_ctrl.py 0
sleep 1s

# agv移动至原点
rosrun lunarPrinter agv_pose_ctrl.py -2.5 2.5 0

# 支撑腿伸出
rosrun lunarPrinter foot_ctrl.py 0.102
sleep 1s

# 机械臂控制来到起始位置
rosrun lunarPrinter arm_joint_ctrl.py -1.57 0 0 0 0 0
sleep 1s
rosrun lunarPrinter arm_joint_ctrl.py 0 0 0 0 0 0

# 开始打印
rosrun lunarPrinter arm_path_planning_v3.py 3_2.txt 1
sleep 1s
rosrun lunarPrinter arm_joint_ctrl.py 0 0 0 0 0 0

# 支撑腿收回 
rosrun lunarPrinter foot_ctrl.py 0
sleep 1s
rosrun lunarPrinter agv_pose_ctrl.py -2.5 4 0
sleep 1s
rosrun lunarPrinter agv_pose_ctrl.py 2.5 4 0

# 4------------------------------------

# 支撑腿收回 
# rosrun lunarPrinter foot_ctrl.py 0
sleep 1s

# agv移动至原点
rosrun lunarPrinter agv_pose_ctrl.py 2.5 2.5 0

# 支撑腿伸出
rosrun lunarPrinter foot_ctrl.py 0.102
sleep 1s

# 开始打印
rosrun lunarPrinter arm_path_planning_v3.py 4_2.txt 1
sleep 1s
rosrun lunarPrinter arm_joint_ctrl.py 0 0 0 0 0 0

# 支撑腿收回 
rosrun lunarPrinter foot_ctrl.py 0
sleep 1s
rosrun lunarPrinter agv_pose_ctrl.py 4 2.5 0
sleep 1s
rosrun lunarPrinter agv_pose_ctrl.py 4 -2.5 0

# ##############################################################
# 打印顶层
# ##############################################################

rosrun lunarPrinter height_ctrl.py 0.5
sleep 2s

# 1------------------------------------

# 支撑腿收回 
rosrun lunarPrinter foot_ctrl.py 0
sleep 1s

# agv移动至原点
rosrun lunarPrinter agv_pose_ctrl.py 2.5 -2.5 0

# 支撑腿伸出
rosrun lunarPrinter foot_ctrl.py 0.102
sleep 1s

# 机械臂控制来到起始位置
rosrun lunarPrinter arm_joint_ctrl.py -1.57 0 0 0 0 0
sleep 1s
rosrun lunarPrinter arm_joint_ctrl.py -3.14159265 0 0 0 0 0

# 开始打印
rosrun lunarPrinter arm_path_planning_v3.py 1_3.txt 2
sleep 1s
rosrun lunarPrinter arm_joint_ctrl.py -3.14159265 0 0 0 0 0


# 支撑腿收回 
rosrun lunarPrinter foot_ctrl.py 0
sleep 1s
rosrun lunarPrinter agv_pose_ctrl.py 2.5 -4 0
sleep 1s
rosrun lunarPrinter agv_pose_ctrl.py -2.5 -4 0

# 2------------------------------------

# 支撑腿收回 
# rosrun lunarPrinter foot_ctrl.py 0
sleep 1s

# agv移动至原点
rosrun lunarPrinter agv_pose_ctrl.py -2.5 -2.5 0

# 支撑腿伸出
rosrun lunarPrinter foot_ctrl.py 0.102
sleep 1s

# 开始打印
rosrun lunarPrinter arm_path_planning_v3.py 2_3.txt 2
sleep 1s
rosrun lunarPrinter arm_joint_ctrl.py -3.14159265 0 0 0 0 0


# 支撑腿收回 
rosrun lunarPrinter foot_ctrl.py 0
sleep 1s
rosrun lunarPrinter agv_pose_ctrl.py -4 -2.5 0
sleep 1s
rosrun lunarPrinter agv_pose_ctrl.py -4 2.5 0
# 3------------------------------------

# 支撑腿收回 
# rosrun lunarPrinter foot_ctrl.py 0
sleep 1s

# agv移动至原点
rosrun lunarPrinter agv_pose_ctrl.py -2.5 2.5 0

# 支撑腿伸出
rosrun lunarPrinter foot_ctrl.py 0.102
sleep 1s

# 机械臂控制来到起始位置
rosrun lunarPrinter arm_joint_ctrl.py -1.57 0 0 0 0 0
sleep 1s
rosrun lunarPrinter arm_joint_ctrl.py 0 0 0 0 0 0

# 开始打印
rosrun lunarPrinter arm_path_planning_v3.py 3_3.txt 2
sleep 1s
rosrun lunarPrinter arm_joint_ctrl.py 0 0 0 0 0 0

# 支撑腿收回 
rosrun lunarPrinter foot_ctrl.py 0
sleep 1s
rosrun lunarPrinter agv_pose_ctrl.py -2.5 4 0
sleep 1s
rosrun lunarPrinter agv_pose_ctrl.py 2.5 4 0

# 4------------------------------------

# 支撑腿收回 
# rosrun lunarPrinter foot_ctrl.py 0
sleep 1s

# agv移动至原点
rosrun lunarPrinter agv_pose_ctrl.py 2.5 2.5 0

# 支撑腿伸出
rosrun lunarPrinter foot_ctrl.py 0.102
sleep 1s

# 开始打印
rosrun lunarPrinter arm_path_planning_v3.py 4_3.txt 2
sleep 1s
rosrun lunarPrinter arm_joint_ctrl.py 0 0 0 0 0 0

# 支撑腿收回 
rosrun lunarPrinter foot_ctrl.py 0
sleep 1s
rosrun lunarPrinter agv_pose_ctrl.py 4 2.5 0
sleep 1s
rosrun lunarPrinter agv_pose_ctrl.py 4 -2.5 0

rosrun lunarPrinter height_ctrl.py 1
rosrun lunarPrinter arm_joint_ctrl.py 0 0 0 0 0 0