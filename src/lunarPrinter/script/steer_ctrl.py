#!/usr/bin/env python
import rospy
import math
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState

class CmdVelToJoints:
    def __init__(self):
        rospy.init_node('cmd_vel_to_steer_wheels')

        # 参数获取
        self.wheel_radius = rospy.get_param('~wheel_radius', 0.125)
        base_length = rospy.get_param('~base_length', 1.53)
        base_width = rospy.get_param('~base_width', 1.1325)

        # 轮子位置配置 (前左, 前右, 后左, 后右)
        self.positions = [
            (base_length/2, base_width/2),
            (base_length/2, -base_width/2),
            (-base_length/2, base_width/2),
            (-base_length/2, -base_width/2)
        ]

        # 舵轴控制器发布器（使用JointTrajectory）
        self.steer_pub = rospy.Publisher(
            '/steer_wheel_controller/command',
            JointTrajectory,
            queue_size=1
        )
        
        # 驱动轮发布器（保持原有Float64格式）
        self.wheel_pubs = [
            rospy.Publisher(f'/wheel_{i}_velocity_controller/command', Float64, queue_size=1) for i in range(1,5)
        ]

        # 舵轴关节名称（必须与URDF中的关节名严格一致！）
        self.steer_joint_names = [
            'steering_1_joint',
            'steering_2_joint',
            'steering_3_joint',
            'steering_4_joint'
        ]

        # 状态跟踪相关
        self.current_steer_angles = [0.0] * 4
        self.target_steer_angles = None
        self.pending_wheel_speeds = None
        self.check_timer = None
        self.timeout_timer = None
        self.epsilon = 1e-3  # 运动模式判断阈值
        self.epsilon_angle = rospy.get_param('~epsilon_angle', 0.01)  # 舵轴角度误差阈值

        # 订阅者
        rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)
        rospy.Subscriber('/joint_states', JointState, self.joint_states_callback)

    def joint_states_callback(self, msg):
        """更新当前舵轴角度"""
        for i, name in enumerate(msg.name):
            if name in self.steer_joint_names:
                idx = self.steer_joint_names.index(name)
                if i < len(msg.position):
                    self.current_steer_angles[idx] = msg.position[i]

    def cmd_vel_callback(self, msg):
        vx = msg.linear.x
        vy = msg.linear.y
        omega = msg.angular.z

        # 处理停止命令
        if abs(vx) < self.epsilon and abs(vy) < self.epsilon and abs(omega) < self.epsilon:
            self._publish_wheel_speeds_now([0.0]*4)
            return

        steer_angles = []
        wheel_speeds = []

        # 判断运动模式
        is_pure_x = abs(vy) < self.epsilon and abs(omega) < self.epsilon and abs(vx) > self.epsilon
        is_pure_y = abs(vx) < self.epsilon and abs(omega) < self.epsilon and abs(vy) > self.epsilon
        is_pure_omega = abs(vx) < self.epsilon and abs(vy) < self.epsilon and abs(omega) > self.epsilon

        # 计算舵轴角度和轮速
        if is_pure_x:
            for i in range(4):
                angle = 0.0
                speed = vx / self.wheel_radius
                if i in [1, 3]: speed *= -1
                steer_angles.append(angle)
                wheel_speeds.append(speed)
        elif is_pure_y:
            for i in range(4):
                angle = math.pi/2
                speed = vy / self.wheel_radius
                if i in [1, 3]: speed *= -1
                steer_angles.append(angle)
                wheel_speeds.append(speed)
        elif is_pure_omega:
            angles = [-0.9335993110056702, 0.9335993110056702, 
                     0.9335993110056702, -0.9335993110056702]
            for i in range(4):
                xi, yi = self.positions[i]
                radius = math.hypot(xi, yi)
                speed = -omega * radius / self.wheel_radius
                steer_angles.append(angles[i])
                wheel_speeds.append(speed)
        else:
            for i in range(4):
                xi, yi = self.positions[i]
                Vi_x = vx - omega * yi
                Vi_y = vy + omega * xi
                magnitude = math.hypot(Vi_x, Vi_y)
                
                if magnitude < self.epsilon:
                    angle = 0.0
                    speed = 0.0
                else:
                    angle = math.atan2(Vi_y, Vi_x)
                    speed = magnitude / self.wheel_radius

                if i in [1, 3]: speed *= -1
                steer_angles.append(angle)
                wheel_speeds.append(speed)

        # 发布舵轴角度
        self._publish_steering(steer_angles)
        
        # 设置驱动轮发布机制
        self._setup_wheel_control(steer_angles,wheel_speeds)

    def _publish_steering(self, angles):
        """发布舵轴角度指令"""
        steer_traj = JointTrajectory()
        steer_traj.header.stamp = rospy.Time.now()
        steer_traj.joint_names = self.steer_joint_names
        point = JointTrajectoryPoint()
        point.positions = angles
        point.time_from_start = rospy.Duration(0.1)
        steer_traj.points.append(point)
        self.steer_pub.publish(steer_traj)

    def _setup_wheel_control(self, steer_angles, wheel_speeds):
        """设置驱动轮控制机制"""
        # 取消之前的定时器
        if self.check_timer:
            self.check_timer.shutdown()
        if self.timeout_timer:
            self.timeout_timer.shutdown()

        # 保存状态
        self.target_steer_angles = [a for a in steer_angles]
        self.pending_wheel_speeds = [s for s in wheel_speeds]

        # 启动检查定时器
        self.check_timer = rospy.Timer(rospy.Duration(0.05), self._check_angles, oneshot=False)
        # 设置安全超时（保留原1.5秒逻辑）
        self.timeout_timer = rospy.Timer(rospy.Duration(1.5), lambda e: self._publish_wheel_speeds_now(wheel_speeds), oneshot=True)

    def _check_angles(self, event):
        """检查角度是否到位"""
        if all(abs(t - c) < self.epsilon_angle 
              for t, c in zip(self.target_steer_angles, self.current_steer_angles)):
            self._publish_wheel_speeds_now(self.pending_wheel_speeds)
            self.check_timer.shutdown()
            self.timeout_timer.shutdown()

    def _publish_wheel_speeds_now(self, speeds):
        """立即发布驱动轮速度"""
        for i in range(4):
            self.wheel_pubs[i].publish(Float64(speeds[i]))
        # 重置状态
        self.target_steer_angles = None
        self.pending_wheel_speeds = None

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        controller = CmdVelToJoints()
        controller.run()
    except rospy.ROSInterruptException:
        pass