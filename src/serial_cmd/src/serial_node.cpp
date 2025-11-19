#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <asio.hpp>
#include <string>

class SerialCmdNode : public rclcpp::Node
{
public:
    SerialCmdNode()
        : Node("serial_cmd_node"),
          io_(),
          port_(io_, "/dev/pts/6")//ttyACM0
    {
        // Setup serial port
        port_.set_option(asio::serial_port_base::baud_rate(115200));
        port_.set_option(asio::serial_port_base::character_size(8));
        port_.set_option(asio::serial_port_base::parity(asio::serial_port_base::parity::none));
        port_.set_option(asio::serial_port_base::stop_bits(asio::serial_port_base::stop_bits::one));
        port_.set_option(asio::serial_port_base::flow_control(asio::serial_port_base::flow_control::none));

        // Subscribe topic
        sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel",
            10,
            std::bind(&SerialCmdNode::cmdVelCallback, this, std::placeholders::_1)
        );

        RCLCPP_INFO(this->get_logger(), "Serial CMD Node started. Listening on /cmd_vel");
    }

private:
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        // Encode data as string
        std::string data =
            "LX:" + std::to_string(msg->linear.x) +
            " LY:" + std::to_string(msg->linear.y) +
            " LZ:" + std::to_string(msg->linear.z) +
            " AX:" + std::to_string(msg->angular.x) +
            " AY:" + std::to_string(msg->angular.y) +
            " AZ:" + std::to_string(msg->angular.z) + "\n";

        // Send to serial
        asio::write(port_, asio::buffer(data));

        RCLCPP_INFO(this->get_logger(), "Sent to /dev/ttyACM0: %s", data.c_str());
    }

    asio::io_service io_;
    asio::serial_port port_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SerialCmdNode>());
    rclcpp::shutdown();
    return 0;
}
