import rospy
from std_msgs.msg import Bool
from gazebo_msgs.srv import SetModelConfiguration, SetModelConfigurationRequest

class ParticleController:
    def __init__(self):
        rospy.init_node("particle_controller", anonymous=True)

        rospy.Subscriber("/start_printing", Bool, self.controll_particles)

        rospy.wait_for_service("/gazebo/set_model_configuration")

        self.set_model_config = rospy.ServiceProxy("/gazebo/set_model_configuration", SetModelConfiguration)

    
    def controll_particles(self, msg):
        try:
            req = SetModelConfigurationRequest()
            req.model_name = "arm"
            req.urdf_param_name = "robot_description"
            req.joint_names = []
            req.joint_positions = []

            if msg.data:
                rospy.loginfo("开启粒子发射")
                req.urdf_param_name = '<enabled>true</enabled>'
            else:
                rospy.loginfo("关闭粒子发射")
                req.urdf_param_name = '<enabled>false</enabled>'

            self.set_model_config(req)
        
        except rospy.ServiceException as e:
            rospy.logerr(f"Gazebo 设置失败: {e}")

if __name__ == "__main__":
    ParticleController()
    rospy.spin()


        