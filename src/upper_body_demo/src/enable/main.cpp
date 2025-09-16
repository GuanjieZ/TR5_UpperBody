// src/enable_flag_node.cpp
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include <thread>
#include <atomic>
#include <unistd.h>
#include <termios.h>
#include <chrono>

using namespace std::chrono_literals;

class RawMode {
public:
  RawMode() {
    tcgetattr(STDIN_FILENO, &orig_);
    termios raw = orig_;
    raw.c_lflag &= ~(ICANON | ECHO);  // single keypress, no echo
    raw.c_cc[VMIN]  = 1;
    raw.c_cc[VTIME] = 0;
    tcsetattr(STDIN_FILENO, TCSANOW, &raw);
  }
  ~RawMode() { tcsetattr(STDIN_FILENO, TCSANOW, &orig_); }
private:
  termios orig_{};
};

class EnableFlagNode : public rclcpp::Node {
public:
  EnableFlagNode()
  : Node("enable_flag_node")
  {
    // Add .transient_local() if you want the last value to be retained for late subscribers
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
    pub_ = this->create_publisher<std_msgs::msg::Bool>("enable", qos);

    RCLCPP_INFO(get_logger(),
      "Press 'l' to publish true, 's' to publish false (each has its own 1 Hz cooldown).");

    // allow immediate first publish for both values
    last_true_pub_  = std::chrono::steady_clock::now() - min_interval_;
    last_false_pub_ = std::chrono::steady_clock::now() - min_interval_;

    running_.store(true);
    input_thread_ = std::thread([this]() {
      RawMode raw;
      while (running_.load()) {
        char c;
        ssize_t n = read(STDIN_FILENO, &c, 1);
        if (n <= 0) break;
        if (c == 'l' || c == 'L') {
          try_publish(true);
        } else if (c == 's' || c == 'S') {
          try_publish(false);
        }
      }
    });
  }

  ~EnableFlagNode() override {
    running_.store(false);
    if (input_thread_.joinable()) input_thread_.join();
  }

private:
  void try_publish(bool value) {
    auto now = std::chrono::steady_clock::now();
    auto &last = value ? last_true_pub_ : last_false_pub_;

    if (now - last < min_interval_) {
      // cooldown active; skip
      return;
    }

    std_msgs::msg::Bool msg;
    msg.data = value;
    pub_->publish(msg);
    last = now;
    RCLCPP_INFO(get_logger(), "Published enable=%s", value ? "true" : "false");
  }

  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_;
  std::atomic<bool> running_{false};
  std::thread input_thread_;
  std::chrono::steady_clock::time_point last_true_pub_;
  std::chrono::steady_clock::time_point last_false_pub_;
  const std::chrono::steady_clock::duration min_interval_{1s}; // per-value cooldown
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<EnableFlagNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
