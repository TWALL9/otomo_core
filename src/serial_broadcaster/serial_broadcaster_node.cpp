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
    this->declare_parameter("serial_port", "/dev/ttyUSB0");
    this->declare_parameter("baud_rate", 115200);

    port_path_ = this->get_parameter("serial_port").as_string();
    baud_rate_ = this->get_parameter("baud_rate").as_int();
    port_ = std::make_unique<async_serial::SerialPort>(port_path_, baud_rate_);
  }

  bool open() {
    bool ret = false;

    if (port_->open()) {
      port_->add_receive_callback(std::bind(
        &SerialBroadcasterNode::serial_callback, this, std::placeholders::_1, std::placeholders::_2
      ));
      ret = true;
    } else {
      RCLCPP_FATAL(this->get_logger(), "Cannot open port!");
    }

    return ret;
  }

private:
  void serial_callback(const std::vector<uint8_t>& buf, size_t bytes_received) {
    std::string received((char*)buf.data(), bytes_received);
    std::stringstream received_stream(received);
    std::string log;
    size_t n;
    while(std::getline(received_stream, log, '\n')) {
      // TODO substring based on these positions
      if ((n = log.find("[TRACE] ")) != log.npos) {
        RCLCPP_DEBUG(this->get_logger(), "%s", log.c_str());
      } else if ((n = log.find("[DEBUG] ")) != log.npos) {
        RCLCPP_DEBUG(this->get_logger(), "%s", log.c_str());
      } else if ((n = log.find("[INFO] ")) != log.npos) {
        RCLCPP_INFO(this->get_logger(), "%s", log.c_str());
      } else if ((n = log.find("[WARN] ")) != log.npos) {
        RCLCPP_WARN(this->get_logger(), "%s", log.c_str());
      } else if ((n = log.find("[ERROR] ")) != log.npos) {
        RCLCPP_ERROR(this->get_logger(), "%s", log.c_str());
      // } else {
      //   RCLCPP_INFO(this->get_logger(), "unknown firmware log: %s", log.c_str());
      }
    }
  }

  std::unique_ptr<async_serial::SerialPort> port_;
  std::string port_path_;
  uint32_t baud_rate_;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  auto broadcaster_node = std::make_shared<SerialBroadcasterNode>();

  if (!broadcaster_node->open()) {
    exit(1);
  }

  rclcpp::spin(broadcaster_node);
  rclcpp::shutdown();

  return 0;
}