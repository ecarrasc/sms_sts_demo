#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <exception>
#include <fcntl.h>
#include <memory>
#include <sstream>
#include <string>
#include <termios.h>
#include <unordered_map>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64.hpp"

namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kTwoPi = 2.0 * kPi;

speed_t baudToTermios(int baud_rate)
{
  switch (baud_rate) {
    case 9600:
      return B9600;
    case 19200:
      return B19200;
    case 38400:
      return B38400;
    case 57600:
      return B57600;
    case 115200:
      return B115200;
    case 230400:
      return B230400;
    case 460800:
      return B460800;
    case 500000:
      return B500000;
    case 921600:
      return B921600;
    case 1000000:
      return B1000000;
    default:
      return B115200;
  }
}

int32_t clampToRawRange(int32_t value, int32_t min_value, int32_t max_value)
{
  return std::clamp(value, min_value, max_value);
}

struct ServoState
{
  int32_t position = 0;
  int32_t speed = 0;
  double voltage = 0.0;
  double temperature = 0.0;
  bool moving = false;
  bool valid = false;
};

class SerialPort
{
public:
  ~SerialPort()
  {
    close();
  }

  bool openPort(const std::string & device, int baud_rate)
  {
    close();

    fd_ = ::open(device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) {
      error_ = std::strerror(errno);
      return false;
    }

    termios options{};
    if (tcgetattr(fd_, &options) != 0) {
      error_ = std::strerror(errno);
      close();
      return false;
    }

    cfmakeraw(&options);
    const speed_t baud = baudToTermios(baud_rate);
    cfsetispeed(&options, baud);
    cfsetospeed(&options, baud);

    options.c_cflag |= static_cast<tcflag_t>(CLOCAL | CREAD);
    options.c_cflag &= static_cast<tcflag_t>(~CRTSCTS);
    options.c_cflag &= static_cast<tcflag_t>(~CSTOPB);
    options.c_cflag &= static_cast<tcflag_t>(~PARENB);
    options.c_cflag &= static_cast<tcflag_t>(~CSIZE);
    options.c_cflag |= CS8;
    options.c_cc[VMIN] = 0;
    options.c_cc[VTIME] = 0;

    if (tcsetattr(fd_, TCSANOW, &options) != 0) {
      error_ = std::strerror(errno);
      close();
      return false;
    }

    tcflush(fd_, TCIOFLUSH);
    return true;
  }

  bool isOpen() const
  {
    return fd_ >= 0;
  }

  void close()
  {
    if (fd_ >= 0) {
      ::close(fd_);
      fd_ = -1;
    }
  }

  bool writeLine(const std::string & line)
  {
    if (fd_ < 0) {
      error_ = "serial port is not open";
      return false;
    }

    std::string data = line;
    data.push_back('\n');

    const char * buffer = data.data();
    size_t remaining = data.size();
    while (remaining > 0) {
      const ssize_t written = ::write(fd_, buffer, remaining);
      if (written < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
          continue;
        }
        error_ = std::strerror(errno);
        return false;
      }
      buffer += written;
      remaining -= static_cast<size_t>(written);
    }

    return true;
  }

  bool readLine(std::string & line)
  {
    line.clear();

    if (fd_ < 0) {
      return false;
    }

    char buffer[128];
    while (true) {
      const ssize_t received = ::read(fd_, buffer, sizeof(buffer));
      if (received > 0) {
        rx_buffer_.append(buffer, static_cast<size_t>(received));
      } else if (received < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
        error_ = std::strerror(errno);
        return false;
      } else {
        break;
      }
    }

    const auto newline = rx_buffer_.find('\n');
    if (newline == std::string::npos) {
      return false;
    }

    line = rx_buffer_.substr(0, newline);
    rx_buffer_.erase(0, newline + 1);
    if (!line.empty() && line.back() == '\r') {
      line.pop_back();
    }
    return true;
  }

  const std::string & error() const
  {
    return error_;
  }

private:
  int fd_ = -1;
  std::string rx_buffer_;
  std::string error_;
};
}  // namespace

class SmsStsOneJointNode : public rclcpp::Node
{
public:
  SmsStsOneJointNode()
  : Node("sms_sts_one_joint_node")
  {
    serial_port_name_ = declare_parameter<std::string>("serial_port", "/dev/ttyUSB0");
    baud_rate_ = declare_parameter<int>("baud_rate", 115200);
    servo_id_ = declare_parameter<int>("servo_id", 1);
    joint_name_ = declare_parameter<std::string>("joint_name", "servo_output_joint");
    command_topic_ = declare_parameter<std::string>("command_topic", "servo/command");
    joint_states_topic_ = declare_parameter<std::string>("joint_states_topic", "joint_states");
    read_command_ = declare_parameter<std::string>("read_command", "READ");
    position_command_ = declare_parameter<std::string>("position_command", "MOVE");
    update_rate_hz_ = declare_parameter<double>("update_rate_hz", 100.0);
    state_request_timeout_ms_ = declare_parameter<int>("state_request_timeout_ms", 100);
    min_position_rad_ = declare_parameter<double>("min_position_rad", -kPi);
    max_position_rad_ = declare_parameter<double>("max_position_rad", kPi);
    raw_min_ = declare_parameter<int>("raw_min", 0);
    raw_max_ = declare_parameter<int>("raw_max", 4095);
    raw_center_ = declare_parameter<int>("raw_center", 2048);
    raw_ticks_per_rad_ = declare_parameter<double>("raw_ticks_per_rad", -4095.0 / kTwoPi);
    command_speed_ = declare_parameter<int>("command_speed", 1500);
    command_acceleration_ = declare_parameter<int>("command_acceleration", 50);

    joint_state_pub_ =
      create_publisher<sensor_msgs::msg::JointState>(joint_states_topic_, rclcpp::SystemDefaultsQoS());

    command_sub_ = create_subscription<std_msgs::msg::Float64>(
      command_topic_,
      rclcpp::SystemDefaultsQoS(),
      [this](const std_msgs::msg::Float64::SharedPtr msg) {
        commandCallback(*msg);
      });

    openSerialPort();

    const auto period = std::chrono::duration<double>(1.0 / std::max(update_rate_hz_, 1.0));
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      [this]() {
        update();
      });
  }

private:
  void openSerialPort()
  {
    if (!serial_.openPort(serial_port_name_, baud_rate_)) {
      RCLCPP_ERROR(
        get_logger(),
        "Failed to open %s at %d baud: %s",
        serial_port_name_.c_str(),
        baud_rate_,
        serial_.error().c_str());
      return;
    }

    RCLCPP_INFO(
      get_logger(),
      "Connected to ESP32 SMS/STS bridge on %s at %d baud",
      serial_port_name_.c_str(),
      baud_rate_);
  }

  void commandCallback(const std_msgs::msg::Float64 & msg)
  {
    if (!serial_.isOpen()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "Ignoring command because the serial port is not open");
      return;
    }

    const double target_rad = std::clamp(msg.data, min_position_rad_, max_position_rad_);
    const int32_t target_raw = radiansToRaw(target_rad);

    std::ostringstream command;
    command << position_command_ << ' '
            << servo_id_ << ' '
            << target_raw << ' '
            << command_speed_ << ' '
            << command_acceleration_;

    if (!serial_.writeLine(command.str())) {
      RCLCPP_WARN(
        get_logger(),
        "Failed to send command to ESP32: %s",
        serial_.error().c_str());
      return;
    }

    RCLCPP_INFO(
      get_logger(),
      "Sent %.3f rad as raw position %d using '%s'",
      target_rad,
      target_raw,
      position_command_.c_str());
  }

  void update()
  {
    if (!serial_.isOpen()) {
      return;
    }

    readAvailableStateLines();
    requestStateIfReady();
  }

  void readAvailableStateLines()
  {
    std::string line;
    while (serial_.readLine(line)) {
      ServoState state;
      if (parseStateLine(line, state)) {
        state_request_pending_ = false;
        publishJointState(state);
      } else if (!line.empty()) {
        RCLCPP_DEBUG(get_logger(), "Ignoring serial line: '%s'", line.c_str());
      }
    }
  }

  void requestStateIfReady()
  {
    const auto current_time = now();
    if (state_request_pending_) {
      const auto elapsed = current_time - last_state_request_time_;
      if (elapsed.nanoseconds() <
        std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::milliseconds(state_request_timeout_ms_)).count())
      {
        return;
      }

      state_request_pending_ = false;
      RCLCPP_DEBUG(
        get_logger(),
        "State request timed out after %d ms; sending another READ",
        state_request_timeout_ms_);
    }

    std::ostringstream request;
    request << read_command_ << ' ' << servo_id_;
    if (!serial_.writeLine(request.str())) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "Failed to request state from ESP32: %s",
        serial_.error().c_str());
      return;
    }

    state_request_pending_ = true;
    last_state_request_time_ = current_time;
  }

  bool parseStateLine(const std::string & line, ServoState & state) const
  {
    std::istringstream stream(line);
    std::string tag;
    int id = 0;

    stream >> tag;
    if (tag != "STATE") {
      return false;
    }

    stream >> id;
    if (!stream || id != servo_id_) {
      return false;
    }

    std::string next_token;
    stream >> next_token;
    if (!stream) {
      return false;
    }

    if (next_token.find('=') != std::string::npos) {
      return parseKeyValueStateFields(stream, next_token, state);
    }

    int moving = 0;
    std::istringstream positional_stream(next_token);
    positional_stream >> state.position;
    stream >> state.speed
           >> state.voltage
           >> state.temperature
           >> moving;

    if (!positional_stream || !stream) {
      return false;
    }

    state.moving = moving != 0;
    state.valid = true;
    return true;
  }

  bool parseKeyValueStateFields(
    std::istringstream & stream,
    const std::string & first_field,
    ServoState & state) const
  {
    std::unordered_map<std::string, std::string> fields;
    std::string token = first_field;

    do {
      const auto separator = token.find('=');
      if (separator != std::string::npos && separator + 1 < token.size()) {
        fields[token.substr(0, separator)] = token.substr(separator + 1);
      }
    } while (stream >> token);

    const auto position = fields.find("position");
    const auto speed = fields.find("speed");
    if (position == fields.end() || speed == fields.end()) {
      return false;
    }

    try {
      state.position = std::stoi(position->second);
      state.speed = std::stoi(speed->second);

      if (const auto voltage = fields.find("voltage"); voltage != fields.end()) {
        state.voltage = std::stod(voltage->second);
      }
      if (const auto temperature = fields.find("temperature"); temperature != fields.end()) {
        state.temperature = std::stod(temperature->second);
      }
      if (const auto moving = fields.find("moving"); moving != fields.end()) {
        state.moving = std::stoi(moving->second) != 0;
      }
      if (const auto valid = fields.find("valid"); valid != fields.end()) {
        state.valid = std::stoi(valid->second) != 0;
      } else {
        state.valid = true;
      }
    } catch (const std::exception &) {
      return false;
    }

    return state.valid;
  }

  void publishJointState(const ServoState & state)
  {
    if (!state.valid) {
      return;
    }

    sensor_msgs::msg::JointState joint_state;
    joint_state.header.stamp = now();
    joint_state.name.push_back(joint_name_);
    joint_state.position.push_back(rawToRadians(state.position));
    joint_state.velocity.push_back(rawSpeedToRadiansPerSecond(state.speed));

    joint_state_pub_->publish(joint_state);
  }

  int32_t radiansToRaw(double radians) const
  {
    const auto raw = static_cast<int32_t>(std::lround(raw_center_ + radians * raw_ticks_per_rad_));
    return clampToRawRange(raw, raw_min_, raw_max_);
  }

  double rawToRadians(int32_t raw) const
  {
    return (static_cast<double>(raw) - static_cast<double>(raw_center_)) / raw_ticks_per_rad_;
  }

  double rawSpeedToRadiansPerSecond(int32_t raw_speed) const
  {
    return static_cast<double>(raw_speed) / raw_ticks_per_rad_;
  }

  SerialPort serial_;

  std::string serial_port_name_;
  int baud_rate_ = 115200;
  int servo_id_ = 1;
  std::string joint_name_;
  std::string command_topic_;
  std::string joint_states_topic_;
  std::string read_command_;
  std::string position_command_;
  double update_rate_hz_ = 100.0;
  int state_request_timeout_ms_ = 100;
  double min_position_rad_ = -kPi;
  double max_position_rad_ = kPi;
  int32_t raw_min_ = 0;
  int32_t raw_max_ = 4095;
  int32_t raw_center_ = 2048;
  double raw_ticks_per_rad_ = -4095.0 / kTwoPi;
  int command_speed_ = 1500;
  int command_acceleration_ = 50;
  bool state_request_pending_ = false;
  rclcpp::Time last_state_request_time_{0, 0, RCL_ROS_TIME};

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr command_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SmsStsOneJointNode>());
  rclcpp::shutdown();
  return 0;
}
