#include <vector>
#include <string>
#include <memory>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include <async_serial/serial_port.hpp>
using namespace std::chrono_literals;

class SerialBroadcasterNode : public rclcpp::Node {
public:
  SerialBroadcasterNode() : Node("SerialBroadcasterNode") {
    this->declare_parameter("serial_port" "/dev/ttyACM1");
    this->declare_parameter("baud_rate", 115200);

    std::string port_path = this->get_parameter("serial_port").as_string();
    int baud_rate = this->get_parameter("baud rate").as_int();
    port_ = std::make_unique<async_serial::SerialPort>(port_path, baud_rate);
    port_->open();

    port_->add_receive_callback(std::bind(
      &SerialBroadcasterNode::serial_callback, this, std::placeholders::_1, std::placeholders::_2
    ));
  }

private:
  void serial_callback(const std::vector<uint8_t>& buf, size_t bytes_received) {
    (void)bytes_received;
    std::string received(buf.begin(), buf.end());

    // TODO: regex to find log level
    RCLCPP_INFO(this->get_logger(), "firmware says: %s", received.c_str());
  }

  std::unique_ptr<async_serial::SerialPort> port_;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SerialBroadcasterNode>());
  rclcpp::shutdown();

  return 0;
}