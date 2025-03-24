#include <gazebo/gazebo.hh>
#include <gazebo/rendering/rendering.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/physics/physics.hh>
#include <thread>
#include <mutex>

namespace gazebo {
class TrajectoryVisualizer : public WorldPlugin {
public:
  void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf) override {
    // 参数读取
    line_width_ = _sdf->Get<double>("line_width", 0.02).first;
    target_link_ = _sdf->Get<std::string>("target_link");
    ref_frame_ = _sdf->Get<std::string>("reference_frame");

    // 初始化渲染接口
    rendering::ScenePtr scene = rendering::get_scene();
    if (!scene) scene = rendering::create_scene("default", true);

    // 创建动态线条
    line_ = scene->CreateDynamicLine(rendering::RENDERING_LINE_STRIP);
    line_->set_material("Gazebo/Red");
    line_->set_visibility_flags(GZ_VISIBILITY_GUI);
    line_->set_line_width(line_width_);

    // 监听键盘事件（需配合ROS节点）
    keyboard_sub_ = node_->Subscribe(
      "/keyboard/keypress", &TrajectoryVisualizer::OnKeyPress, this);

    // 定时更新轨迹
    update_connection_ = event::Events::ConnectPreRender(
      std::bind(&TrajectoryVisualizer::UpdateLine, this));
  }

private:
  void UpdateLine() {
    if (!drawing_) return;

    // 获取末端位姿（世界坐标系）
    math::Pose pose;
    if (GetLinkPose(target_link_, pose)) {
      line_->AddPoint(pose.pos);
    }
  }

  void OnKeyPress(ConstAnyPtr &_msg) {
    if (_msg->int_value() == 32) { // 空格键ASCII码
      drawing_ = !drawing_;
      if (!drawing_) line_->Clear();
    }
  }

  bool GetLinkPose(const std::string &_link, math::Pose &_pose) {
    // 通过Gazebo API获取Link位姿
    physics::ModelPtr model = world_->GetModel("your_arm_model_name");
    if (model) {
      physics::LinkPtr link = model->GetLink(_link);
      if (link) {
        _pose = link->WorldPose();
        return true;
      }
    }
    return false;
  }

  rendering::DynamicLinePtr line_;
  bool drawing_ = false;
  double line_width_;
  std::string target_link_, ref_frame_;
  transport::NodePtr node_;
  transport::SubscriberPtr keyboard_sub_;
  event::ConnectionPtr update_connection_;
};
GZ_REGISTER_WORLD_PLUGIN(TrajectoryVisualizer)
}