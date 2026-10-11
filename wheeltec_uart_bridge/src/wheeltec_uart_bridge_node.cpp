#include "wheeltec_uart_bridge/packet.hpp"

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>

#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <mutex>
#include <memory>
#include <poll.h>
#include <stdexcept>
#include <string>
#include <sys/file.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

namespace wheeltec_uart_bridge
{

class UartBridgeNode final : public rclcpp::Node
{
public:
  UartBridgeNode()
  : Node("wheeltec_uart_bridge"), next_reconnect_(SteadyClock::now())
  {
    port_ = declare_parameter<std::string>("port", "/dev/wheeltec_controller");
    baudrate_ = declare_parameter<int>("baudrate", 115200);
    command_timeout_ = declare_parameter<double>("command_timeout", 0.5);
    transmit_rate_ = declare_parameter<double>("transmit_rate", 20.0);
    reconnect_interval_ = declare_parameter<double>("reconnect_interval", 1.0);
    limits_.linear_x = declare_parameter<double>("max_linear_x", 0.30);
    limits_.linear_y = declare_parameter<double>("max_linear_y", 0.0);
    limits_.angular_z = declare_parameter<double>("max_angular_z", 0.60);
    reject_linear_y_ = declare_parameter<bool>("reject_linear_y", true);
    const auto topic = declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");

    validate_parameters();
    subscription_ = create_subscription<geometry_msgs::msg::Twist>(
      topic, rclcpp::QoS(10).reliable(),
      [this](geometry_msgs::msg::Twist::ConstSharedPtr message) {on_command(*message);});
    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / transmit_rate_));
    timer_ = create_wall_timer(period, [this]() {on_timer();});

    if (!open_serial()) {
      next_reconnect_ = SteadyClock::now() + seconds(reconnect_interval_);
      RCLCPP_WARN(get_logger(), "UART unavailable at startup; remaining stopped and retrying");
    }
  }

  ~UartBridgeNode() override
  {
    if (timer_) {
      timer_->cancel();
    }
    std::lock_guard<std::mutex> lock(mutex_);
    have_fresh_command_ = false;
    if (serial_fd_ >= 0) {
      const auto zero = encode_packet(0.0, 0.0, 0.0, limits_);
      (void)write_packet(zero);
    }
    close_serial();
  }

private:
  using SteadyClock = std::chrono::steady_clock;

  static SteadyClock::duration seconds(const double value)
  {
    return std::chrono::duration_cast<SteadyClock::duration>(
      std::chrono::duration<double>(value));
  }

  void validate_parameters() const
  {
    if (port_.empty()) {
      throw std::invalid_argument("port must not be empty");
    }
    if (baudrate_ != 115200) {
      throw std::invalid_argument("this chassis protocol requires baudrate=115200");
    }
    const auto positive_finite = [](const double value) {
        return std::isfinite(value) && value > 0.0;
      };
    if (!positive_finite(command_timeout_) || !positive_finite(transmit_rate_) ||
      !positive_finite(reconnect_interval_))
    {
      throw std::invalid_argument("timeout, transmit rate and reconnect interval must be positive");
    }
    for (const double limit : {limits_.linear_x, limits_.linear_y, limits_.angular_z}) {
      if (!std::isfinite(limit) || limit < 0.0 || limit > 32.767) {
        throw std::invalid_argument("velocity limits must be finite and within 0..32.767");
      }
    }
  }

  void on_command(const geometry_msgs::msg::Twist & message)
  {
    const double x = message.linear.x;
    const double y = message.linear.y;
    const double z = message.angular.z;
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000, "Rejected /cmd_vel containing NaN or Inf");
      return;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    latest_x_ = x;
    latest_y_ = reject_linear_y_ ? 0.0 : y;
    latest_z_ = z;
    last_command_ = SteadyClock::now();
    have_fresh_command_ = true;
    if (reject_linear_y_ && std::abs(y) > 1e-6) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Ignoring nonzero linear.y because reject_linear_y=true");
    }
  }

  void on_timer()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto now = SteadyClock::now();
    if (serial_fd_ < 0) {
      if (now >= next_reconnect_ && !open_serial()) {
        next_reconnect_ = now + seconds(reconnect_interval_);
      }
      return;
    }

    const bool timed_out = !have_fresh_command_ ||
      now - last_command_ > seconds(command_timeout_);
    const double x = timed_out ? 0.0 : latest_x_;
    const double y = timed_out ? 0.0 : latest_y_;
    const double z = timed_out ? 0.0 : latest_z_;
    if (timed_out && !watchdog_active_) {
      RCLCPP_WARN(get_logger(), "Command watchdog active; transmitting zero velocity");
    } else if (!timed_out && watchdog_active_) {
      RCLCPP_INFO(get_logger(), "Fresh /cmd_vel received; watchdog released");
    }
    watchdog_active_ = timed_out;

    if (!write_packet(encode_packet(x, y, z, limits_))) {
      handle_serial_failure("UART write failed");
    }
  }

  bool open_serial()
  {
    close_serial();
    int descriptor = ::open(port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
    if (descriptor < 0) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 5000, "Cannot open %s: %s",
        port_.c_str(), std::strerror(errno));
      return false;
    }
    if (::flock(descriptor, LOCK_EX | LOCK_NB) != 0 || ::ioctl(descriptor, TIOCEXCL) != 0) {
      RCLCPP_ERROR(get_logger(), "Serial device %s is already in use: %s",
        port_.c_str(), std::strerror(errno));
      ::close(descriptor);
      return false;
    }

    termios options{};
    if (::tcgetattr(descriptor, &options) != 0) {
      RCLCPP_ERROR(get_logger(), "tcgetattr(%s) failed: %s", port_.c_str(), std::strerror(errno));
      ::close(descriptor);
      return false;
    }
    ::cfmakeraw(&options);
    ::cfsetispeed(&options, B115200);
    ::cfsetospeed(&options, B115200);
    options.c_cflag &= ~(PARENB | CSTOPB | CSIZE | CRTSCTS);
    options.c_cflag |= CS8 | CLOCAL | CREAD;
    options.c_cc[VMIN] = 0;
    options.c_cc[VTIME] = 0;
    if (::tcsetattr(descriptor, TCSANOW, &options) != 0 || ::tcflush(descriptor, TCIOFLUSH) != 0) {
      RCLCPP_ERROR(get_logger(), "Cannot configure %s as 115200 8N1: %s",
        port_.c_str(), std::strerror(errno));
      ::close(descriptor);
      return false;
    }

    serial_fd_ = descriptor;
    have_fresh_command_ = false;  // A pre-disconnect command may never move the chassis.
    watchdog_active_ = true;
    const auto zero = encode_packet(0.0, 0.0, 0.0, limits_);
    if (!write_packet(zero)) {
      handle_serial_failure("Initial zero-velocity write failed");
      return false;
    }
    RCLCPP_INFO(get_logger(), "Opened %s at 115200 8N1; waiting for a new /cmd_vel",
      port_.c_str());
    return true;
  }

  bool write_packet(const Packet & packet)
  {
    if (serial_fd_ < 0) {
      return false;
    }
    std::size_t offset = 0;
    const auto deadline = SteadyClock::now() + std::chrono::milliseconds(100);
    while (offset < packet.size()) {
      const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(
        deadline - SteadyClock::now()).count();
      if (remaining <= 0) {
        errno = ETIMEDOUT;
        return false;
      }
      pollfd descriptor{serial_fd_, POLLOUT, 0};
      const int ready = ::poll(&descriptor, 1, static_cast<int>(remaining));
      if (ready < 0 && errno == EINTR) {
        continue;
      }
      if (ready <= 0 || (descriptor.revents & (POLLERR | POLLHUP | POLLNVAL)) != 0) {
        if (ready == 0) {
          errno = ETIMEDOUT;
        }
        return false;
      }
      const ssize_t written = ::write(
        serial_fd_, packet.data() + offset, packet.size() - offset);
      if (written < 0 && (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK)) {
        continue;
      }
      if (written <= 0) {
        return false;
      }
      offset += static_cast<std::size_t>(written);
    }
    return true;
  }

  void handle_serial_failure(const char * message)
  {
    const int saved_errno = errno;
    RCLCPP_ERROR(get_logger(), "%s on %s: %s", message, port_.c_str(),
      std::strerror(saved_errno));
    have_fresh_command_ = false;
    close_serial();
    next_reconnect_ = SteadyClock::now() + seconds(reconnect_interval_);
  }

  void close_serial()
  {
    if (serial_fd_ < 0) {
      return;
    }
    (void)::ioctl(serial_fd_, TIOCNXCL);
    (void)::flock(serial_fd_, LOCK_UN);
    (void)::close(serial_fd_);
    serial_fd_ = -1;
  }

  std::mutex mutex_;
  int serial_fd_{-1};
  std::string port_;
  int baudrate_{115200};
  double command_timeout_{0.5};
  double transmit_rate_{20.0};
  double reconnect_interval_{1.0};
  Limits limits_{0.30, 0.0, 0.60};
  bool reject_linear_y_{true};
  bool have_fresh_command_{false};
  bool watchdog_active_{true};
  double latest_x_{0.0};
  double latest_y_{0.0};
  double latest_z_{0.0};
  SteadyClock::time_point last_command_{};
  SteadyClock::time_point next_reconnect_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace wheeltec_uart_bridge

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<wheeltec_uart_bridge::UartBridgeNode>();
    rclcpp::spin(node);
    node.reset();  // Destructor sends zero while the ROS context is still valid.
  } catch (const std::exception & error) {
    std::fprintf(stderr, "wheeltec_uart_bridge: %s\n", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
