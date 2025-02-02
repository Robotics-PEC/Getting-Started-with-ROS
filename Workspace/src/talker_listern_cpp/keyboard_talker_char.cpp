#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <chrono>
#include <iostream>
#include <memory>
#include <termios.h>
#include <unistd.h>


class Talker : public rclcpp::Node {
public:
  Talker() : Node("talker") {
    publisher_ = this->create_publisher<std_msgs::msg::String>("chatter", 10);
    // Set terminal to raw mode to capture keyboard input
    tcgetattr(STDIN_FILENO, &orig_termios_);
    termios raw = orig_termios_;
    raw.c_lflag &= ~(ECHO | ICANON);
    tcsetattr(STDIN_FILENO, TCSAFLUSH, &raw);
  }

  ~Talker() {
    // Restore terminal settings
    tcsetattr(STDIN_FILENO, TCSAFLUSH, &orig_termios_);
  }

  void capture_and_publish() {
    char c;
    while (rclcpp::ok()) {
      c = getchar();
      if (c == '\x03') { // Ctrl-C to exit
        break;
      }
      auto message = std_msgs::msg::String();
      message.data = std::string(1, c);
      RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
      publisher_->publish(message);
    }
  }

private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  termios orig_termios_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Talker>();
  std::thread capture_thread(&Talker::capture_and_publish, node);
  rclcpp::spin(node);
  capture_thread.join();
  rclcpp::shutdown();
  return 0;
}
