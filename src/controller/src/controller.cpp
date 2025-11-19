#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "interfaces/msg/control.hpp"

using namespace std::chrono_literals;

const int maximum_Yaw = 180;
const int minimum_Yaw = -180;
const int maximum_Depth = 10;
const int minimum_Depth = 0;
std::string pesan;//jadiin message
int x = 0; //-255 -> 255
int y = 0; //-255 -> 255 
unsigned int depth = 0; //tidak boleh minus
int yaw = 0; //-180 -> 180

class ControlJoy : public rclcpp::Node{

public:
  ControlJoy() : Node("controller"){
    publisher_= this->create_publisher<interfaces::msg::Control>("control_msg", 10);
    subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "joy", 10, std::bind(&ControlJoy::controllerCallback, this, std::placeholders::_1));
  }

private:
    rclcpp::Publisher<interfaces::msg::Control>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;

    std::string valueMsg(int button_pressed, int x_val, int y_val, int depth_val, int yaw_val){
      if(button_pressed == 1){
        return "ROV Banyubramanta sedang bermanuver X: " + std::to_string(x_val) + 
               " Y: " + std::to_string(y_val) + 
               " pada depth: " + std::to_string(depth_val) + 
               " dengan yaw: " + std::to_string(yaw_val);
      }
      else{
        return "";
      }
    }
    
    int nowMsg(int in, int now, int min, int max){
      now += in;
      if(now < min){now = min;} 
      if(now > max){now = max;}
      return now;
    }

    void controllerCallback(const sensor_msgs::msg::Joy &msg){
      x = (-1 * msg.axes[0] * 250);
      y = (msg.axes[1] * 250);
      depth = nowMsg(-1 * msg.axes[4], depth, minimum_Depth, maximum_Depth);
      yaw = nowMsg(-1 * msg.axes[3], yaw, minimum_Yaw, maximum_Yaw);
      pesan = valueMsg(msg.buttons[0], x, y, depth, yaw);
      
      auto control_msg = interfaces::msg::Control();
      control_msg.indikator_yaw = yaw;
      control_msg.indikator_depth = depth;
      control_msg.index_x = x;
      control_msg.index_y = y;
      control_msg.keterangan = pesan;
      publisher_->publish(control_msg);
    } 
};

int main(int CountA, char *VectorA[]){
  rclcpp::init(CountA, VectorA);
  rclcpp::spin(std::make_shared<ControlJoy>());
  rclcpp::shutdown();
  return 0;
}