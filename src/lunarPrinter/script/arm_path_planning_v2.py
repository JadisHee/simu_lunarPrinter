#!/usr/bin/env python3
import rospy
import numpy as np
import moveit_commander
import math
import time
import tf2_ros
import threading


import tf.transformations as tf
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped


from nav_msgs.msg import Odometry
from nav_msgs.msg import Path

from tf.transformations import euler_from_quaternion
from scipy.spatial.transform import Rotation

class arm_path_planning:
    '''
        path_points:路径点云,其中e=0表示仅位移
                    np.array([
                        [x1,y1,z1,e1],
                        [x2,y2,z2,e2],
                        ...,
                        [xn,yn,zn,en]
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

        self.path_points = path_points[:,:3]
        self.extruder_ctrl = path_points[:,3]
        self.pos_vector = pos_vector
        self.tool_euler = tool_euler

        self.odom_sub = rospy.Subscriber('/agv/odom',Odometry, self.odom_cb)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # self.tool_odom_pub = rospy.Publisher("/tool/odom", Odometry, queue_size=10)
        # self.tool_odom_pub_thread = threading.Thread(target=self.tool_odom)
        # self.tool_odom_pub_thread_running = True
        # self.tool_odom_pub_thread_paused = False

        self.tool_path_pub = rospy.Publisher("/tool/path", Path, queue_size=10)
        self.tool_path_pub_thread = threading.Thread(target=self.tool_path)
        # self.tool_path_pub_thread_running = True
        self.tool_path_pub_thread_paused = False
        self.tool_path_pub_thread_new_seg = False
        self.tool_path_msg = Path()
        self.tool_path_msg.header.frame_id = 'world'
 
    def tool_path(self):
        
        while 1:
            try:
                if self.tool_path_pub_thread_paused:
                    print("/tool/path 发布暂停")
                    continue
                    # while self.tool_path_pub_thread_paused and self.tool_path_pub_thread_running:
                    #     print("/odom/path 等待启动信号")
                    #     time.sleep(0.1)
                transform = self.tf_buffer.lookup_transform('world','link_7',rospy.Time(0))
                x = transform.transform.translation.x
                y = transform.transform.translation.y
                z = transform.transform.translation.z

                qx = transform.transform.rotation.x
                qy = transform.transform.rotation.y
                qz = transform.transform.rotation.z
                qw = transform.transform.rotation.w

                pose = PoseStamped()
                pose.header.stamp = rospy.Time.now()
                pose.header.frame_id = 'world'
                pose.pose.position.x = x
                pose.pose.position.y = y
                pose.pose.position.z = z
                pose.pose.orientation.x = qx
                pose.pose.orientation.y = qy
                pose.pose.orientation.z = qz
                pose.pose.orientation.w = qw

                if self.tool_path_pub_thread_new_seg:
                    self.tool_path_msg = Path()
                    self.tool_path_msg.header.frame_id = 'world'
                    self.tool_path_pub_thread_new_seg = False

                self.tool_path_msg.poses.append(pose)
                self.tool_path_msg.header.stamp = rospy.Time.now()
                self.tool_path_pub.publish(self.tool_path_msg)
                # pose = Odometry()
                # odom.header.stamp = rospy.Time.now()
                # odom.header.frame_id = "world"
                # odom.child_frame_id = "link_7"
                # odom.pose.pose.position.x = x
                # odom.pose.pose.position.y = y
                # odom.pose.pose.position.z = z

                # odom.pose.pose.orientation.x = qx
                # odom.pose.pose.orientation.y = qy
                # odom.pose.pose.orientation.z = qz
                # odom.pose.pose.orientation.w = qw

                # odom.twist.twist.linear.x = 0
                # odom.twist.twist.linear.y = 0
                # odom.twist.twist.linear.z = 0
                # odom.twist.twist.angular.x = 0
                # odom.twist.twist.angular.y = 0
                # odom.twist.twist.angular.z = 0

                # self.tool_odom_pub.publish(odom)
                rospy.Rate(10).sleep()
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rospy.logwarn("TF数据获取失败，等待中...")
        print("/tool/path  停止发布")
        
            # return odom


    # def tool_odom_pub(self,ctrl_sig):
        # if ctrl_sig == 1:



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

        while 1:
            if self.current_pos[0] != 0:
                break

        self.tool_path_pub_thread.start()
        rate = rospy.Rate(10)
        for i in range(len(self.path_points)):

            if self.extruder_ctrl[i] != 0:
                self.tool_path_pub_thread_paused = False
                self.tool_path_pub_thread_new_seg = False
                # self.tool_odom_pub_thread.join()
                # if not self.tool_odom_pub_thread.is_alive():
                    # self.tool_odom_pub_thread.run()
                
                # self.tool_odom_pub_thread.join()
            else:
                self.tool_odom_pub_thread_paused = True
                self.tool_path_pub_thread_new_seg = True
                # self.tool_odom_pub_thread.join()
                # self.tool_odom_pub_thread.join()
                # odom = self.tool_odom()
                # self.tool_odom_pub.publish(odom)

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
            # print("当前目标值为:", current_pos)
            # waypoints.append(target_pose)


            # # 设置规划参数
            # group.set_pose_target(target_pose)
            # group.set_max_velocity_scaling_factor(0.5)  # 降低速度避免抖动
            # group.set_planning_time(10.0)              # 增加规划时间
        
            # # 执行规划
            # plan = group.go(wait=True)
            # rospy.INFO('开始')
            (plan, fraction) = self.group.compute_cartesian_path(
                [target_pose],
                0.01,
            )

            if fraction > 0.9:
                self.group.execute(plan, wait=True)

            if i == 0:
                time.sleep(3)
            rate.sleep()

        self.tool_odom_pub_thread_running = False
        self.tool_odom_pub_thread.join()
        self.group.stop()
        self.group.clear_pose_targets()
        

if __name__ == '__main__':
    try:
        tail_file = "/home/space/work/gazebo_ws/src/build_model/roof/1_1.txt"
        tail_data = np.loadtxt(tail_file)
        tail_data[:,:3] = tail_data[:,:3] / 1000

        pos_vec = [0,0,0.1,0,0,0] 
        tool_euler = [3.1415926, 0.0, 3.1415926]

        ctrler = arm_path_planning(tail_data,pos_vec,tool_euler)

        ctrler.run()
        
    except rospy.ROSInterruptException:
        pass