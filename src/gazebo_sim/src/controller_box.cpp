#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Pose3.hh>

#include <rclcpp/rclcpp.hpp>
#include "interfaces/msg/custom.hpp"

#include <memory>
#include <string>

namespace gazebo
{
  class BoxControlPlugin : public ModelPlugin
  {
    public:
      BoxControlPlugin() : ModelPlugin() {}

      void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
      {
        // Simpan pointer ke model
        this->model = _model;
        this->link = this->model->GetLink("base_link");

        if (!this->link)
        {
          gzerr << "Link base_link tidak ditemukan!\n";
          return;
        }

        // Initialize ROS2
        if (!rclcpp::ok())
        {
          rclcpp::init(0, nullptr);
        }

        // Create ROS2 node
        this->ros_node = rclcpp::Node::make_shared("box_control_plugin");

        // Subscribe ke topic cmd_vel
        this->cmd_vel_sub = this->ros_node->create_subscription<interfaces::msg::Custom>(
          "/cmd_vel", 10,
          std::bind(&BoxControlPlugin::OnCmdVel, this, std::placeholders::_1));

        // Update rate
        double update_rate = 50.0;
        if (_sdf->HasElement("update_rate"))
        {
          update_rate = _sdf->Get<double>("update_rate");
        }
        
        this->update_period = 1.0 / update_rate;

        // Connect to world update event
        this->update_connection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&BoxControlPlugin::OnUpdate, this));

        // Executor untuk ROS2
        this->executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
        this->executor->add_node(this->ros_node);
        
        // Thread untuk spin ROS2
        this->ros_thread = std::thread([this]() {
          while (rclcpp::ok()) {
            this->executor->spin_some();
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
          }
        });

        gzmsg << "Box Control Plugin loaded successfully!\n";
      }

      void OnCmdVel(const interfaces::msg::Custom::SharedPtr msg)
      {
        std::lock_guard<std::mutex> lock(this->mutex);
        
        this->target_x = msg->x / 250.0 * 2.0;      // Normalisasi ke m/s
        this->target_y = msg->y / 250.0 * 2.0;      // Normalisasi ke m/s
        this->target_depth = msg->depth;             // Dalam meter
        this->target_yaw = msg->yaw * M_PI / 180.0;  // Konversi ke radian
        
        if (strlen(msg->pesan.c_str()) > 0)
        {
          gzmsg << "Pesan: " << msg->pesan << "\n";
        }
        
        this->last_cmd_time = this->model->GetWorld()->SimTime();
      }

      void OnUpdate()
      {
        std::lock_guard<std::mutex> lock(this->mutex);
        
        // Check timeout (2 detik tanpa command = stop)
        common::Time current_time = this->model->GetWorld()->SimTime();
        double dt = (current_time - this->last_update_time).Double();
        
        if (dt < this->update_period)
        {
          return;
        }
        
        this->last_update_time = current_time;

        if ((current_time - this->last_cmd_time).Double() > 2.0)
        {
          this->target_x = 0;
          this->target_y = 0;
        }

        // Dapatkan pose saat ini
        ignition::math::Pose3d pose = this->link->WorldPose();
        ignition::math::Vector3d linear_vel = this->link->WorldLinearVel();
        ignition::math::Vector3d angular_vel = this->link->WorldAngularVel();

        // Hitung orientasi target
        double current_yaw = pose.Rot().Yaw();
        double yaw_error = this->target_yaw - current_yaw;
        
        // Normalisasi error ke [-pi, pi]
        while (yaw_error > M_PI) yaw_error -= 2.0 * M_PI;
        while (yaw_error < -M_PI) yaw_error += 2.0 * M_PI;

        // Kontrol yaw (rotasi Z)
        double yaw_vel = yaw_error * 2.0; // Gain proporsional
        
        // Transformasi velocity dari frame body ke world frame
        double cos_yaw = cos(current_yaw);
        double sin_yaw = sin(current_yaw);
        
        double world_vx = this->target_x * cos_yaw - this->target_y * sin_yaw;
        double world_vy = this->target_x * sin_yaw + this->target_y * cos_yaw;

        // Kontrol depth (Z position)
        double depth_error = this->target_depth - pose.Pos().Z();
        double depth_vel = depth_error * 1.0; // Gain proporsional

        // Set velocities
        this->link->SetLinearVel(ignition::math::Vector3d(world_vx, world_vy, depth_vel));
        this->link->SetAngularVel(ignition::math::Vector3d(0, 0, yaw_vel));
      }

      ~BoxControlPlugin()
      {
        if (this->ros_thread.joinable())
        {
          this->ros_thread.join();
        }
        this->ros_node.reset();
      }

    private:
      physics::ModelPtr model;
      physics::LinkPtr link;
      event::ConnectionPtr update_connection;

      rclcpp::Node::SharedPtr ros_node;
      rclcpp::Subscription<interfaces::msg::Custom>::SharedPtr cmd_vel_sub;
      std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor;
      std::thread ros_thread;

      std::mutex mutex;
      double target_x = 0.0;
      double target_y = 0.0;
      double target_depth = 0.0;
      double target_yaw = 0.0;

      common::Time last_update_time;
      common::Time last_cmd_time;
      double update_period;
  };

  GZ_REGISTER_MODEL_PLUGIN(BoxControlPlugin)
}
