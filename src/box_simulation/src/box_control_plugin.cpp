#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>

#include <rclcpp/rclcpp.hpp>
#include "interfaces/msg/control.hpp"

#include <mutex>
#include <thread>

namespace gazebo {

class BoxControlPlugin : public ModelPlugin {
public:
  BoxControlPlugin() : ModelPlugin() {}

  void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override {
    this->model = model;
    this->link = model->GetLink("base_link");

    if (!link) {
      gzerr << "ERROR: base_link tidak ditemukan!\n";
      return;
    }

    if (!rclcpp::ok()) rclcpp::init(0, nullptr);
    node = rclcpp::Node::make_shared("box_control_plugin");

    sub = node->create_subscription<interfaces::msg::Control>(
      "/cmd_vel", 10,
      std::bind(&BoxControlPlugin::OnCmdVel, this, std::placeholders::_1));

    executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor->add_node(node);

    ros_thread = std::thread([this]() {
      rclcpp::WallRate rate(100);
      while (rclcpp::ok()) {
        executor->spin_some();
        rate.sleep();
      }
    });

    update_connection = event::Events::ConnectWorldUpdateBegin(
      std::bind(&BoxControlPlugin::OnUpdate, this));
  }

  void OnCmdVel(const interfaces::msg::Control::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mutex);

    target_x = msg->index_x / 250.0 * 2.0;
    target_y = msg->index_y / 250.0 * 2.0;
    target_depth = msg->indikator_depth;
    target_yaw = msg->indikator_yaw * M_PI / 180.0;
  }

  void OnUpdate() {
    std::lock_guard<std::mutex> lock(mutex);

    ignition::math::Pose3d pose = link->WorldPose();
    double yaw = pose.Rot().Yaw();

    double depth_error = target_depth - pose.Pos().Z();
    double yaw_error = target_yaw - yaw;

    double vx = target_x * cos(yaw) - target_y * sin(yaw);
    double vy = target_x * sin(yaw) + target_y * cos(yaw);

    link->SetLinearVel({vx, vy, depth_error});
    link->SetAngularVel({0, 0, yaw_error});
  }

private:
  physics::ModelPtr model;
  physics::LinkPtr link;

  event::ConnectionPtr update_connection;

  double target_x = 0, target_y = 0;
  double target_depth = 0;
  double target_yaw = 0;

  std::mutex mutex;

  rclcpp::Node::SharedPtr node;
  rclcpp::Subscription<interfaces::msg::Control>::SharedPtr sub;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor;
  std::thread ros_thread;
};

GZ_REGISTER_MODEL_PLUGIN(BoxControlPlugin)

} // namespace gazebo
