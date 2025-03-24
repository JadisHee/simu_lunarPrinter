#!/usr/bin/env python
import rospy
import numpy as np
import math
import argparse
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion

class AgvCtrl:
    def __init__ (self, target_x, target_y, target_theta):
        rospy.init_node('agv_ctrl')

        # 运动状态
        self.states = [
            'move_x',
            'move_y',
            'move_θ',
            'done'
        ]

        self.mode = self.states[2]

        # 线位移误差阈值
        self.linear_err_threshold = 0.0012
        # 角旋转误差阈值
        self.angular_err_threshold = 0.001745329    

        # 线位移误差阈值
        # self.linear_err_threshold = 0.01
        # 角旋转误差阈值
        # self.angular_err_threshold = 0.01    

        self.linear_velocity_max = 0.3
        self.angular_omega_max = 0.523598776

        # 目标位置
        self.target = [
            target_x,
            target_y,
            target_theta
        ]

        # 当前位置
        self.current = [
            0.0,
            0.0,
            0.0
        ]

        self.err = [
            0.0,
            0.0,
            0.0
        ]

        self.odom_sub = rospy.Subscriber('/agv/odom', Odometry, self.odom_callback)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        self.kp_angular = 1
        self.kp_linear = 0.8

    def odom_callback(self, msg):
        self.current[0] = msg.pose.pose.position.x
        self.current[1] = msg.pose.pose.position.y
        
        q = msg.pose.pose.orientation
        _, _, self.current[2] = euler_from_quaternion([q.x, q.y, q.z, q.w])

        for i in range(len(self.err)):
            self.err[i] = self.target[i] - self.current[i]

        

    # def normalize_angle(self, angle):
    #     while angle > math.pi:
    #         angle -= 2 * math.pi
    #     while angle < -math.pi:
    #         angle += 2 * math.pi
    #     return angle

    def move_ctrl(self):

        # twist = Twist()
        # self.mode = 3

        if abs(self.err[2]) <= self.angular_err_threshold and abs(self.err[0]) <= self.linear_err_threshold and abs(self.err[1]) <= self.linear_err_threshold:
            self.mode = self.states[3]
            rospy.sleep(0.5)   

            if self.mode == self.states[3]:
                twist = Twist()

                twist.linear.x = 0.0
                twist.linear.y = 0.0
                twist.angular.z = 0.0

                return twist

        self.mode = self.states[2]
        if self.mode == self.states[2]:
            if abs(self.err[2]) > self.angular_err_threshold:
                cmd_vel =  self.omega_ctrl()
                return cmd_vel
            else:
                self.mode = self.states[0]
                rospy.sleep(0.5)

        if self.mode == self.states[0]:
            if abs(self.err[0]) > self.linear_err_threshold:
                cmd_vel =  self.x_ctrl()
                return cmd_vel
            else:
                self.mode = self.states[1]
                rospy.sleep(0.5)

        if self.mode == self.states[1]:
            if abs(self.err[1]) > self.linear_err_threshold:
                cmd_vel =  self.y_ctrl()
                return cmd_vel


        








        # if self.mode == 3: 
        #     if abs(self.err[2]) > self.angular_err_threshold:
        #         cmd_vel =  self.omega_ctrl()
        #         return cmd_vel
        #     else:
        #         self.mode = 1

        # if self.mode == 1:
        #     if abs(self.err[0]) > self.linear_err_threshold:
        #         cmd_vel = self.x_ctrl()
        #         return cmd_vel
        #     else:
        #         self.mode = 2

        # if self.mode == 2:
        #     if abs(self.err[1]) > self.linear_err_threshold:
        #         cmd_vel = self.y_ctrl()
        #         return cmd_vel
        #     else:
        #         self.mode = 0
        
        # twist = Twist()

        # twist.linear.x = 0.0
        # twist.linear.y = 0.0
        # twist.angular.z = 0.0

        # return twist



    def x_ctrl(self):
        twist = Twist()


        twist.linear.x = self.kp_linear * self.err[0]
        
        if abs(twist.linear.x) > self.linear_velocity_max:
            if twist.linear.x > 0:
                twist.linear.x = self.linear_velocity_max
            else:
                twist.linear.x = -self.linear_velocity_max
        twist.linear.y = 0.0
        twist.angular.z = 0.0

        return twist

    def y_ctrl(self):
        twist = Twist()

        twist.linear.x = 0.0
        twist.linear.y = self.kp_linear * self.err[1]
        if abs(twist.linear.y) > self.linear_velocity_max:
            if twist.linear.y > 0:
                twist.linear.y = self.linear_velocity_max
            else:
                twist.linear.y = -self.linear_velocity_max
        twist.angular.z = 0.0

        return twist

    def omega_ctrl(self):

        twist = Twist()

        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.angular.z = self.kp_angular * self.err[2]
        if abs(twist.angular.z) > self.angular_omega_max:
            if twist.angular.z > 0:
                twist.angular.z = self.angular_omega_max
            else:
                twist.angular.z = -self.angular_omega_max

        return twist

    def run(self):

        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            # if self.d_theta <= self.err_theta_thr:
            
            
            
            


            ctrl_twist = self.move_ctrl()
            print("------------")
            print(
                'dx: ',self.err[0],'\n'
                'dy: ',self.err[1],'\n'
                'dθ: ',self.err[2],'\n'
                'mode: ',self.mode,'\n'
            )
            print(
                'vx: ', ctrl_twist.linear.x,'\n'
                'vy: ', ctrl_twist.linear.y,'\n'
                'ω:  ', ctrl_twist.angular.z
            )
            if self.err == [0.0,0.0,0.0]:
                continue
            if self.mode == self.states[3] and (abs(self.err[2]) > self.angular_err_threshold or abs(self.err[0]) > self.linear_err_threshold or abs(self.err[1]) > self.linear_err_threshold):
                continue
            if self.mode == self.states[3]:
                break
            self.cmd_vel_pub.publish(ctrl_twist)
            rate.sleep()
             



if __name__ == '__main__':
    try:
        parser = argparse.ArgumentParser()
        parser.add_argument("target_pos",type=float, nargs=3)
        args = parser.parse_args()

        # target_theta = 0
        ctrler = AgvCtrl(args.target_pos[0],args.target_pos[1],args.target_pos[2])
        ctrler.run()
    except rospy.ROSInterruptException:
        pass
