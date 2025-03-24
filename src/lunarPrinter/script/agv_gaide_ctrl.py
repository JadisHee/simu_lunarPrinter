#!/usr/bin/env python
import rospy
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion

class AGVController:
    def __init__(self, target_x, target_y, target_theta):
        rospy.init_node('agv_controller')
        
        self.target_x = target_x
        self.target_y = target_y
        self.target_theta = target_theta
        
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0
        
        self.odom_sub = rospy.Subscriber('/agv/odom', Odometry, self.odom_callback)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
        self.state = "AlignHeading"
        self.fine_tune_stage = "x"
        
        # 控制参数
        self.kp_angular = 0.8
        self.kp_linear = 0.5
        self.kp_fine = 0.3
        self.max_angular_speed = 1.0
        self.max_linear_speed = 0.3

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        
        q = msg.pose.pose.orientation
        _, _, self.current_theta = euler_from_quaternion([q.x, q.y, q.z, q.w])

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def compute_control(self):
        twist = Twist()
        dx = self.target_x - self.current_x
        dy = self.target_y - self.current_y
        distance = math.hypot(dx, dy)
        current_theta = self.current_theta
        
        if self.state == "AlignHeading":
            if distance < 0.005:
                self.state = "AdjustTheta"
                return twist
            
            target_heading = math.atan2(dy, dx)
            delta_theta = self.normalize_angle(target_heading - current_theta)
            
            if abs(delta_theta) > 0.008:
                twist.angular.z = self.kp_angular * delta_theta
                twist.angular.z = max(-self.max_angular_speed, min(twist.angular.z, self.max_angular_speed))
            else:
                self.state = "MoveLinear"
        
        elif self.state == "MoveLinear":
            if distance > 0.005:
                twist.linear.x = self.kp_linear * distance
                twist.linear.x = max(-self.max_linear_speed, min(twist.linear.x, self.max_linear_speed))
            else:
                self.state = "AdjustTheta"
        
        elif self.state == "AdjustTheta":
            delta_theta = self.normalize_angle(self.target_theta - current_theta)
            
            if abs(delta_theta) > 0.008:
                twist.angular.z = self.kp_angular * delta_theta
                twist.angular.z = max(-self.max_angular_speed, min(twist.angular.z, self.max_angular_speed))
            else:
                pos_error = math.hypot(dx, dy)
                theta_error = abs(delta_theta)
                
                if pos_error <= 0.005 and theta_error <= 0.008:
                    self.state = "Done"
                elif pos_error <= 0.1 or theta_error <= 0.05:
                    self.state = "FineTune"
                    self.fine_tune_stage = "x"
        
        elif self.state == "FineTune":
            if self.fine_tune_stage == "x":
                dx = self.target_x - self.current_x
                if abs(dx) > 0.005:
                    twist.linear.x = self.kp_fine * dx
                    twist.linear.x = max(-0.1, min(twist.linear.x, 0.1))
                else:
                    self.fine_tune_stage = "y"
            
            elif self.fine_tune_stage == "y":
                dy = self.target_y - self.current_y
                if abs(dy) > 0.005:
                    twist.linear.y = self.kp_fine * dy
                    twist.linear.y = max(-0.1, min(twist.linear.y, 0.1))
                else:
                    self.fine_tune_stage = "theta"
            
            elif self.fine_tune_stage == "theta":
                delta_theta = self.normalize_angle(self.target_theta - current_theta)
                if abs(delta_theta) > 0.008:
                    twist.angular.z = self.kp_angular * delta_theta
                    twist.angular.z = max(-0.5, min(twist.angular.z, 0.5))
                else:
                    dx = abs(self.target_x - self.current_x)
                    dy = abs(self.target_y - self.current_y)
                    dtheta = abs(self.normalize_angle(self.target_theta - current_theta))
                    
                    if dx <= 0.005 and dy <= 0.005 and dtheta <= 0.008:
                        self.state = "Done"
                    else:
                        self.fine_tune_stage = "x"
        
        return twist

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.state == "Done":
                rospy.loginfo("Goal reached with precision!")
                self.cmd_vel_pub.publish(Twist())  # 停止
                break
            rospy.loginfo('state: {self.state}')
            control_twist = self.compute_control()
            self.cmd_vel_pub.publish(control_twist)
            rate.sleep()

if __name__ == '__main__':
    try:
        # 通过参数或输入获取目标位置
        target_x = 0
        target_y = 0
        target_theta = 0
        
        controller = AGVController(target_x, target_y, target_theta)
        controller.run()
    except rospy.ROSInterruptException:
        pass