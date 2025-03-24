#!/usr/bin/env python3
import rospy
import tf2_ros
import threading
from gazebo_msgs.srv import SpawnModel, DeleteModel
from geometry_msgs.msg import PoseStamped, Point, Quaternion, Vector3
from std_msgs.msg import ColorRGBA
from std_srvs.srv import Empty
import sys
import termios
import tty
import numpy as np

# ---------------------- 键盘监听模块 ----------------------
class KeyListener:
    def __init__(self):
        self.pressed_key = None
        self.listener_thread = threading.Thread(target=self._listen_keyboard)
        self.listener_thread.daemon = True
        self.listener_thread.start()

    def _listen_keyboard(self):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setcbreak(fd)
            while True:
                ch = sys.stdin.read(1)
                self.pressed_key = ch
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

    def get_key(self):
        key = self.pressed_key
        self.pressed_key = None  # 清空状态
        return key

# ---------------------- 主逻辑 ----------------------
class LineDrawer:
    def __init__(self):
        rospy.init_node('line_drawer', log_level=rospy.INFO)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        # 初始化服务代理
        rospy.wait_for_service('/gazebo/spawn_sdf_model')
        rospy.wait_for_service('/gazebo/delete_model')
        self.spawn_model = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        self.delete_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
        
        # 加载线段模型模板
        self.line_template = self._create_line_sdf(radius=0.005, color=(0,1,0,1))
        
        # 状态控制
        self.is_drawing = False
        self.last_point = None
        self.line_counter = 0
        self.active_models = set()  # 记录已生成模型名称
        
        # 键盘监听
        self.key_listener = KeyListener()
        
        rospy.loginfo("按空格键开始/停止绘制，按C键清除所有线条")

    def _create_line_sdf(self, radius=0.02, color=(1,0,0,1)):
        """生成线段SDF模板（细圆柱体）"""
        return f'''
        <?xml version='1.0'?>
        <sdf version='1.6'>
            <model name='LINE_TEMPLATE'>
                <link name='link'>
                    <inertial>
                        <mass>0</mass> 
                    </inertial>
                    <visual name='visual'>
                        <geometry>
                        <cylinder>
                            <radius>{radius}</radius>
                            <length>0.01</length> <!-- 长度动态计算 -->
                        </cylinder>
                        </geometry>
                        <material>
                        <ambient>{color[0]} {color[1]} {color[2]} {color[3]}</ambient>
                        </material>
                    </visual>
                </link>
            </model>
        </sdf>
        '''

    def get_end_effector_pos(self):
        """获取末端执行器位置"""
        try:
            trans = self.tf_buffer.lookup_transform('world', 'link_7', rospy.Time(0))
            return [trans.transform.translation.x,
                    trans.transform.translation.y,
                    trans.transform.translation.z]
        except:
            return None

    def spawn_line_segment(self, start, end):
        """生成两点之间的线段"""
        # 计算线段参数
        length = ((end[0]-start[0])**2 + (end[1]-start[1])**2 + (end[2]-start[2])**2)**0.5
        center = [(start[0]+end[0])/2, (start[1]+end[1])/2, (start[2]+end[2])/2]
        
        # 计算朝向（四元数）
        yaw = np.arctan2(end[1]-start[1], end[0]-start[0])
        pitch = -np.arcsin((end[2]-start[2])/length)
        
        # 生成模型名称
        model_name = f"line_{self.line_counter}"
        self.line_counter += 1
        
        # 替换模板参数
        model_xml = self.line_template.replace(
            'LINE_TEMPLATE', model_name
        ).replace(
            '<length>1.0</length>', f'<length>{length}</length>'
        )
        
        # 设置位姿
        pose = PoseStamped()
        pose.header.frame_id = "world"
        pose.pose.position = Point(*center)
        q = self.euler_to_quaternion(yaw, pitch, 0)
        pose.pose.orientation = Quaternion(*q)
        
        try:
            self.spawn_model(model_name, model_xml, '', pose.pose, 'world')
            self.active_models.add(model_name)
        except rospy.ServiceException as e:
            rospy.logerr(f"Spawn failed: {str(e)}")

    def euler_to_quaternion(self, yaw, pitch, roll):
        """欧拉角转四元数"""
        cy = np.cos(yaw*0.5)
        sy = np.sin(yaw*0.5)
        cp = np.cos(pitch*0.5)
        sp = np.sin(pitch*0.5)
        cr = np.cos(roll*0.5)
        sr = np.sin(roll*0.5)
        
        w = cy * cp * cr + sy * sp * sr
        x = cy * cp * sr - sy * sp * cr
        y = sy * cp * sr + cy * sp * cr
        z = sy * cp * cr - cy * sp * sr
        return (x, y, z, w)

    def clear_all_lines(self):
        """清除所有已画线段"""
        for name in list(self.active_models):
            try:
                self.delete_model(name)
                self.active_models.remove(name)
            except:
                pass
        rospy.loginfo("已清除所有线条")

    def run(self):
        rate = rospy.Rate(30)  # 30Hz主循环
        while not rospy.is_shutdown():
            key = self.key_listener.get_key()
            
            # 处理按键
            if key == ' ':  # 空格键切换绘制状态
                self.is_drawing = not self.is_drawing
                rospy.loginfo(f"绘制状态: {'开启' if self.is_drawing else '关闭'}")
                self.last_point = None  # 重置起点
            
            if key == 'c' or key == 'C':  # 清除线条
                self.clear_all_lines()
            
            # 绘制逻辑
            if self.is_drawing:
                current_pos = self.get_end_effector_pos()
                if current_pos is not None:
                    if self.last_point is not None:
                        # 每隔2cm生成线段（可根据需要调整）
                        distance = ((current_pos[0]-self.last_point[0])**2 +
                                   (current_pos[1]-self.last_point[1])**2 +
                                   (current_pos[2]-self.last_point[2])**2)**0.5
                        if distance > 0.002:
                            self.spawn_line_segment(self.last_point, current_pos)
                            self.last_point = current_pos
                    else:
                        self.last_point = current_pos
            
            rate.sleep()

if __name__ == '__main__':
    try:
        drawer = LineDrawer()
        drawer.run()
    except rospy.ROSInterruptException:
        pass