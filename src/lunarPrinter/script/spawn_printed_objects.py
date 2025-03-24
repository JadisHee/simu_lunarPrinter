#!/usr/bin/env python3
import rospy
import tf2_ros
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import PoseStamped, Point, Quaternion
import os

class ObjectSpawner:
    def __init__(self):
        rospy.init_node('object_spawner')
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.spawn_model = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        
        # 加载模型文件
        self.model_path = rospy.get_param('~model_path', '/home/space/work/gazebo_ws/src/lunarPrinter/config/cube.sdf')
        with open(self.model_path, 'r') as f:
            self.model_xml = f.read()
        
        self.rate = rospy.Rate(100)  # 10Hz
        self.last_position = None
        self.min_distance = 0.0001  # 最小生成间隔（5mm）

    def get_end_effector_pose(self):
        try:
            trans = self.tf_buffer.lookup_transform('world', 'link_7', rospy.Time(0))
            return trans.transform.translation
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            return None

    def run(self):
        counter = 0
        while not rospy.is_shutdown():
            pos = self.get_end_effector_pose()
            if pos:
                current_pos = [pos.x, pos.y, pos.z]
                # current_pos = [5,2,2]
                if self.last_position is None or \
                   self.distance(current_pos, self.last_position) >= self.min_distance:
                    # 生成唯一模型名称
                    model_name = f'printed_object_{counter}'
                    pose = PoseStamped()
                    pose.header.frame_id = 'world'
                    pose.pose.position = Point(*current_pos)
                    pose.pose.orientation = Quaternion(1, 0, 0, 1)  # 无旋转
                    
                    try:
                        self.spawn_model(
                            model_name, self.model_xml, '', pose.pose, 'world'
                        )
                        rospy.loginfo(f"Spawned {model_name} at {current_pos}")
                        self.last_position = current_pos
                        counter += 1
                    except rospy.ServiceException as e:
                        rospy.logerr(f"Spawn failed: {e}")
            self.rate.sleep()

    @staticmethod
    def distance(p1, p2):
        return ((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2 + (p1[2]-p2[2])**2)**0.5

if __name__ == '__main__':
    spawner = ObjectSpawner()
    spawner.run()