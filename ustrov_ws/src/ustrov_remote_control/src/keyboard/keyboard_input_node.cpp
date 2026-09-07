// Copyright (C) 2026 USTROV
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// as published by the Free Software Foundation; either version 2
// of the License, or (at your option) any later version.

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cctype>
#include <functional>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>

namespace ustrov_remote_control {
namespace keyboard {

namespace axes {
static constexpr std::size_t kLeftStickLeftRight = 0;
static constexpr std::size_t kLeftStickUpDown = 1;
static constexpr std::size_t kRightStickLeftRight = 3;
static constexpr std::size_t kRightStickUpDown = 4;
static constexpr std::size_t kNumAxes = 8;
}  // namespace axes

class KeyboardInput : public rclcpp::Node {
 public:
  KeyboardInput() : Node("keyboard") {
    const double publish_rate = declare_parameter("publish_rate", 20.0);
    key_timeout_ = std::chrono::duration<double>(
        declare_parameter("key_timeout", 0.6));
    axis_value_ = declare_parameter("axis_value", 1.0);

    if (publish_rate <= 0.0) {
      throw std::invalid_argument("publish_rate must be greater than zero");
    }
    if (key_timeout_.count() <= 0.0) {
      throw std::invalid_argument("key_timeout must be greater than zero");
    }
    if (axis_value_ <= 0.0 || axis_value_ > 1.0) {
      throw std::invalid_argument("axis_value must be in the range (0, 1]");
    }

    joy_pub_ = create_publisher<sensor_msgs::msg::Joy>("joy", 10);
    publish_timer_ = create_wall_timer(
        std::chrono::duration<double>(1.0 / publish_rate),
        std::bind(&KeyboardInput::PublishJoy, this));

    RCLCPP_INFO(
        get_logger(),
        "Keyboard Hippo 4-DOF: W/S surge Fx, A/D yaw Mz, "
        "Up/Down pitch My, Left/Right roll Mx, "
        "SPACE stop, Ctrl+C exit.");

    input_thread_ = std::thread(&KeyboardInput::ReadKeyboard, this);
  }

  ~KeyboardInput() override {
    running_.store(false);
    if (input_thread_.joinable()) {
      input_thread_.join();
    }
  }

 private:
  struct AxisState {
    float value{0.0F};
    bool active{false};
    std::chrono::steady_clock::time_point last_key{};
  };

  void ReadKeyboard() {
    int terminal_fd = STDIN_FILENO;
    bool close_terminal = false;
    if (!isatty(terminal_fd)) {
      // ROS 2 launch 不会把 stdin 直接交给子节点，因此改读当前控制终端。
      terminal_fd = open("/dev/tty", O_RDONLY);
      close_terminal = true;
    }
    if (terminal_fd < 0 || !isatty(terminal_fd)) {
      RCLCPP_ERROR(
          get_logger(),
          "No controlling terminal is available. Start this launch from an "
          "interactive Docker terminal.");
      if (terminal_fd >= 0 && close_terminal) {
        close(terminal_fd);
      }
      return;
    }

    struct termios original_terminal {};
    if (tcgetattr(terminal_fd, &original_terminal) != 0) {
      RCLCPP_ERROR(get_logger(), "Unable to read terminal settings.");
      if (close_terminal) {
        close(terminal_fd);
      }
      return;
    }

    struct termios raw_terminal = original_terminal;
    // 关闭行缓冲和按键回显，但保留 Ctrl+C 信号处理。
    raw_terminal.c_lflag &= static_cast<tcflag_t>(~(ICANON | ECHO));
    raw_terminal.c_cc[VMIN] = 0;
    raw_terminal.c_cc[VTIME] = 1;
    if (tcsetattr(terminal_fd, TCSANOW, &raw_terminal) != 0) {
      RCLCPP_ERROR(get_logger(), "Unable to enable keyboard input mode.");
      if (close_terminal) {
        close(terminal_fd);
      }
      return;
    }

    while (running_.load() && rclcpp::ok()) {
      char key = 0;
      const ssize_t count = read(terminal_fd, &key, 1);
      if (count == 1) {
        if (key == '\x1b') {
          // 方向键通常由 ESC、'['、A/B/C/D 三个字符组成。
          char prefix = 0;
          char arrow = 0;
          const ssize_t prefix_count = read(terminal_fd, &prefix, 1);
          const ssize_t arrow_count = read(terminal_fd, &arrow, 1);
          if (prefix_count == 1 && arrow_count == 1 &&
              (prefix == '[' || prefix == 'O')) {
            HandleArrow(arrow);
          }
        } else {
          HandleKey(key);
        }
      }
    }

    tcsetattr(terminal_fd, TCSANOW, &original_terminal);
    if (close_terminal) {
      close(terminal_fd);
    }
  }

  void HandleKey(char key) {
    key = static_cast<char>(
        std::tolower(static_cast<unsigned char>(key)));

    std::lock_guard<std::mutex> lock(axis_mutex_);
    if (key == ' ') {
      for (auto &axis : axis_states_) {
        axis.value = 0.0F;
        axis.active = false;
      }
      return;
    }

    std::size_t axis_index = axes::kNumAxes;
    float direction = 0.0F;
    switch (key) {
      case 'w':
        // 沿机体 +X 方向前进，不是垂直上浮。
        axis_index = axes::kLeftStickUpDown;
        direction = 1.0F;
        break;
      case 's':
        // 沿机体 -X 方向后退。
        axis_index = axes::kLeftStickUpDown;
        direction = -1.0F;
        break;
      case 'a':
        // 左虚拟摇杆向左：正偏航输入。
        axis_index = axes::kLeftStickLeftRight;
        direction = 1.0F;
        break;
      case 'd':
        axis_index = axes::kLeftStickLeftRight;
        direction = -1.0F;
        break;
      default:
        return;
    }

    auto &axis = axis_states_.at(axis_index);
    axis.value = direction * static_cast<float>(axis_value_);
    axis.active = true;
    axis.last_key = std::chrono::steady_clock::now();
  }

  void HandleArrow(char arrow) {
    std::size_t axis_index = axes::kNumAxes;
    float direction = 0.0F;
    switch (arrow) {
      case 'A':  // Up：正俯仰力矩 My
        axis_index = axes::kRightStickUpDown;
        direction = 1.0F;
        break;
      case 'B':  // Down：负俯仰力矩 My
        axis_index = axes::kRightStickUpDown;
        direction = -1.0F;
        break;
      case 'C':  // Right：负滚转力矩 Mx
        axis_index = axes::kRightStickLeftRight;
        direction = -1.0F;
        break;
      case 'D':  // Left：正滚转力矩 Mx
        axis_index = axes::kRightStickLeftRight;
        direction = 1.0F;
        break;
      default:
        return;
    }

    std::lock_guard<std::mutex> lock(axis_mutex_);
    auto &axis = axis_states_.at(axis_index);
    axis.value = direction * static_cast<float>(axis_value_);
    axis.active = true;
    axis.last_key = std::chrono::steady_clock::now();
  }

  void PublishJoy() {
    sensor_msgs::msg::Joy msg;
    msg.header.stamp = now();
    msg.axes.assign(axes::kNumAxes, 0.0F);
    msg.buttons.assign(6, 0);

    const auto current_time = std::chrono::steady_clock::now();
    std::lock_guard<std::mutex> lock(axis_mutex_);
    for (std::size_t index = 0; index < axis_states_.size(); ++index) {
      auto &axis = axis_states_[index];
      // 终端没有“按键释放”事件；超过超时时间便视为已经松开。
      if (axis.active && current_time - axis.last_key > key_timeout_) {
        axis.value = 0.0F;
        axis.active = false;
      }
      msg.axes[index] = axis.value;
    }
    joy_pub_->publish(msg);
  }

  std::atomic<bool> running_{true};
  std::thread input_thread_;
  std::mutex axis_mutex_;
  std::array<AxisState, axes::kNumAxes> axis_states_{};
  std::chrono::duration<double> key_timeout_{0.6};
  double axis_value_{1.0};
  rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr joy_pub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
};

}  // namespace keyboard
}  // namespace ustrov_remote_control

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(
      std::make_shared<ustrov_remote_control::keyboard::KeyboardInput>());
  rclcpp::shutdown();
  return 0;
}
