#!/usr/bin/env python
import rospy
import math
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

class AGVController:
    def __init__(self):
        rospy.init_node('agv_controller', anonymous=True)
        
        # 参数初始化
        self.target_x = rospy.get_param('~target_x', 1.0)
        self.target_y = rospy.get_param('~target_y', 1.0)
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0
        
        # 控制参数
        self.Kp_theta = 0.8
        self.Kp_linear = 0.5
        self.max_linear_speed = 0.3
        self.max_angular_speed = 0.5
        self.xy_tolerance = 0.005
        self.theta_tolerance = 0.008
        
        # 状态机状态
        self.state = 'rotate_to_zero'
        
        # 发布和订阅
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        rospy.Subscriber('/agv/odom', Odometry, self.odom_callback)
        
    def odom_callback(self, msg):
        # 获取当前位姿
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        
        # 转换四元数到欧拉角
        orientation = msg.pose.pose.orientation
        (_, _, self.current_theta) = euler_from_quaternion(
            [orientation.x, orientation.y, orientation.z, orientation.w]
        )
        
        # 处理状态机
        cmd_vel = Twist()
        
        if self.state == 'rotate_to_zero':
            self.handle_rotation(cmd_vel)
        elif self.state == 'move_x':
            self.handle_x_movement(cmd_vel)
        elif self.state == 'move_y':
            self.handle_y_movement(cmd_vel)
        elif self.state == 'done':
            pass
        
        self.cmd_pub.publish(cmd_vel)
    
    def handle_rotation(self, cmd):
        theta_error = self.normalize_angle(self.current_theta)
        
        if abs(theta_error) < self.theta_tolerance:
            if self.check_position_reached():
                self.state = 'done'
                rospy.loginfo("Target reached!")
            else:
                if abs(self.target_x - self.current_x) > self.xy_tolerance:
                    self.state = 'move_x'
                else:
                    self.state = 'move_y'
        else:
            cmd.angular.z = self.clamp(
                self.Kp_theta * theta_error,
                -self.max_angular_speed,
                self.max_angular_speed
            )
    
    def handle_x_movement(self, cmd):
        if not self.check_orientation():
            self.state = 'rotate_to_zero'
            return
        
        x_error = self.target_x - self.current_x
        if abs(x_error) < self.xy_tolerance:
            if self.check_position_reached():
                self.state = 'done'
                rospy.loginfo("Target reached!")
            else:
                self.state = 'move_y'
        else:
            cmd.linear.x = self.clamp(
                self.Kp_linear * x_error,
                -self.max_linear_speed,
                self.max_linear_speed
            )
    
    def handle_y_movement(self, cmd):
        if not self.check_orientation():
            self.state = 'rotate_to_zero'
            return
        
        y_error = self.target_y - self.current_y
        if abs(y_error) < self.xy_tolerance:
            if self.check_position_reached():
                self.state = 'done'
                rospy.loginfo("Target reached!")
            else:
                self.state = 'move_x'
        else:
            cmd.linear.y = self.clamp(
                self.Kp_linear * y_error,
                -self.max_linear_speed,
                self.max_linear_speed
            )
    
    def check_position_reached(self):
        x_error = abs(self.target_x - self.current_x)
        y_error = abs(self.target_y - self.current_y)
        return x_error < self.xy_tolerance and y_error < self.xy_tolerance
    
    def check_orientation(self):
        return abs(self.normalize_angle(self.current_theta)) < self.theta_tolerance
    
    @staticmethod
    def normalize_angle(angle):
        return (angle + math.pi) % (2 * math.pi) - math.pi
    
    @staticmethod
    def clamp(value, min_val, max_val):
        return max(min_val, min(value, max_val))

if __name__ == '__main__':
    try:
        controller = AGVController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass