#!/usr/bin/env python3
import rospy
import open3d as o3d
import numpy as np
import moveit_commander
import math
import time

import tf.transformations as tf
from geometry_msgs.msg import Pose

from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from scipy.spatial.transform import Rotation

class arm_path_planning:
    '''
        path_points:路径点云
                    np.array([
                        [x1,y1,z1],
                        [x2,y2,z2],
                        ...,
                        [xn,yn,zn]
                        ]) 
        pos_vector: 打印目标中心位置（相对于车体基座）
                    list[x,y,z,rx,ry,rz]
    '''
    def __init__ (self, path_points, pos_vector, tool_euler):
        
        rospy.init_node('arm_path_planning')
        
        self.group = moveit_commander.MoveGroupCommander("lunar_arm")
        self.group.set_max_velocity_scaling_factor(0.3)
        self.group.set_planning_time(10)

                
        self.current_pos = [0,0,0,0,0,0]

        self.path_points = path_points
        self.pos_vector = pos_vector
        self.tool_euler = tool_euler

        self.odom_sub = rospy.Subscriber('/agv/odom',Odometry, self.odom_cb)
 

    def odom_cb(self,msg):
        self.current_pos[0] = msg.pose.pose.position.x
        self.current_pos[1] = msg.pose.pose.position.y
        # self.current_pos[2] = msg.pose.pose.position.z

        q = msg.pose.pose.orientation
        _, _, self.current_pos[5] = euler_from_quaternion([q.x, q.y, q.z, q.w])

    # def get_T(self):
    #     T = np.eye(4)
    #     rot_vec = np.array([
    #         -self.current_pos[3],
    #         -self.current_pos[4],
    #         -self.current_pos[5]
    #     ])
    #     t_vec = -self.current_pos[:3]
    #     R = Rotation.from_rotvec(rot_vec).as_matrix()

    #     T[:3,:3] = R
    #     T[:3,3] = t_vec
    #     T[3,3] = 1

    #     return T    

    def get_T(self, trans_vec):
        
        T = np.eye(4)

        rot_vec = np.array([
            trans_vec[3],
            trans_vec[4],
            trans_vec[5]
        ])

        t_vec = trans_vec[:3]
        R = Rotation.from_rotvec(rot_vec).as_matrix()

        T[:3,:3] = R
        T[:3,3] = t_vec
        T[3,3] = 1

        return T 



    def tf_points(self,points,trans):

        # 变换矩阵
        # trans = self.get_T()

        # 将坐标增广为其次坐标
        ones = np.ones((points.shape[0],1))
        points_temp_1 = np.hstack((points, ones))

        # 执行变换
        points_temp_2 = (trans @ points_temp_1.T).T

        points_t = points_temp_2[:,:3]

        return points_t


    def run(self):
        
        
        

        q = tf.quaternion_from_euler(self.tool_euler[0],self.tool_euler[1],self.tool_euler[2])

        target_pose = Pose()
        target_pose.orientation.x = q[0]
        target_pose.orientation.y = q[1]
        target_pose.orientation.z = q[2]
        target_pose.orientation.w = q[3]

        rate = rospy.Rate(10)
        for i in range(len(self.path_points)):
            current_pos = [
                -self.current_pos[0],
                -self.current_pos[1],
                -self.current_pos[2],
                -self.current_pos[3],
                -self.current_pos[4],
                -self.current_pos[5],
                ]

            trans_1 = self.get_T(current_pos)
            trans_2 = self.get_T(self.pos_vector)

            print("当前变换角为:", current_pos)

            tail_points_1 = self.tf_points(self.path_points,trans_2)

            

            tail_points = self.tf_points(tail_points_1,trans_1)


            target_pose.position.x = tail_points[i][0]  # 单位：米
            target_pose.position.y = tail_points[i][1]
            target_pose.position.z = tail_points[i][2] 

            # waypoints.append(target_pose)


            # # 设置规划参数
            # group.set_pose_target(target_pose)
            # group.set_max_velocity_scaling_factor(0.5)  # 降低速度避免抖动
            # group.set_planning_time(10.0)              # 增加规划时间
        
            # # 执行规划
            # plan = group.go(wait=True)
            (plan, fraction) = self.group.compute_cartesian_path(
                [target_pose],
                0.01,
            )

            if fraction > 0.99:
                self.group.execute(plan, wait=True)

            if i == 0:
                time.sleep(3)
            rate.sleep()

        self.group.stop()
        self.group.clear_pose_targets()
        

if __name__ == '__main__':
    try:
        tail_ptCloud = o3d.io.read_point_cloud('/home/space/work/1_DucoMould/tail_points/gcode_wall_5_3.pcd')
        tail_points = np.asarray(tail_ptCloud.points) / 1000
        
        pos_vec = [0,-2,-0.072,0,0,math.pi/4]
        # pos_vec = [0,-2,-0.072,0,0,-math.pi*3/4]
        tool_euler = [3.1415926, 0.0, 3.1415926]

        ctrler = arm_path_planning(tail_points,pos_vec,tool_euler)
        ctrler.run()
        
    except rospy.ROSInterruptException:
        pass