#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <functional>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rigelslam_rot/msg/motor.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>

class R3lioSensorAdapter final : public rclcpp::Node {
 public:
  R3lioSensorAdapter() : Node("r3lio_sensor_adapter") {
    motor_zero_rad_ = declare_parameter<double>("motor_zero_rad", 0.0);
    motor_direction_ = declare_parameter<int>("motor_direction", 1);
    motor_time_shift_s_ = declare_parameter<double>("motor_time_shift_s", 0.0);
    if (!std::isfinite(motor_zero_rad_) || !std::isfinite(motor_time_shift_s_) ||
        (motor_direction_ != 1 && motor_direction_ != -1)) {
      throw std::invalid_argument(
          "motor_zero_rad and motor_time_shift_s must be finite; motor_direction must be +1 or -1");
    }
    lidar_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/ZG/lidar", 10);
    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("/ZG/IMU_data", 100);
    motor_pub_ = create_publisher<rigelslam_rot::msg::Motor>("/ZG/Motor", 100);

    const auto input_qos = rclcpp::SensorDataQoS();
    lidar_sub_ = create_subscription<livox_ros_driver2::msg::CustomMsg>(
        "/livox/lidar", input_qos,
        std::bind(&R3lioSensorAdapter::on_lidar, this, std::placeholders::_1));
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
        "/imu/data_raw", input_qos,
        [this](sensor_msgs::msg::Imu::ConstSharedPtr msg) { imu_pub_->publish(*msg); });
    joint_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", input_qos,
        std::bind(&R3lioSensorAdapter::on_joint, this, std::placeholders::_1));
  }

 private:
  static sensor_msgs::msg::PointField field(const std::string& name,
                                           std::uint32_t offset, std::uint8_t type) {
    sensor_msgs::msg::PointField result;
    result.name = name;
    result.offset = offset;
    result.datatype = type;
    result.count = 1;
    return result;
  }

  template <typename T>
  static void write_field(std::uint8_t* dst, std::size_t offset, const T& value) {
    std::memcpy(dst + offset, &value, sizeof(T));
  }

  void on_lidar(livox_ros_driver2::msg::CustomMsg::ConstSharedPtr msg) {
    using sensor_msgs::msg::PointField;
    if (msg->points.size() > std::numeric_limits<std::uint32_t>::max() / 24) {
      RCLCPP_ERROR(get_logger(), "LiDAR scan exceeds PointCloud2 row size");
      return;
    }

    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header = msg->header;
    cloud.height = 1;
    cloud.width = static_cast<std::uint32_t>(msg->points.size());
    cloud.fields = {field("x", 0, PointField::FLOAT32),
                    field("y", 4, PointField::FLOAT32),
                    field("z", 8, PointField::FLOAT32),
                    field("intensity", 12, PointField::UINT16),
                    field("ring", 14, PointField::UINT16),
                    field("time", 16, PointField::FLOAT64)};
    cloud.is_bigendian = false;
    cloud.point_step = 24;
    cloud.row_step = 24 * cloud.width;
    cloud.data.resize(cloud.row_step);
    cloud.is_dense = true;

    const std::int64_t header_ns =
        static_cast<std::int64_t>(msg->header.stamp.sec) * 1000000000LL +
        msg->header.stamp.nanosec;
    // Livox timebase and offset_time are nanoseconds. R3LIO adds this relative
    // time to the PointCloud2 header stamp to reconstruct each point timestamp.
    const auto base_offset_ns = static_cast<std::int64_t>(msg->timebase) - header_ns;
    for (std::size_t i = 0; i < msg->points.size(); ++i) {
      const auto& point = msg->points[i];
      auto* dst = cloud.data.data() + i * cloud.point_step;
      const std::uint16_t intensity = point.reflectivity;
      const std::uint16_t ring = point.line;
      const double relative_seconds =
          static_cast<double>(base_offset_ns + point.offset_time) * 1e-9;
      write_field(dst, 0, point.x);
      write_field(dst, 4, point.y);
      write_field(dst, 8, point.z);
      write_field(dst, 12, intensity);
      write_field(dst, 14, ring);
      write_field(dst, 16, relative_seconds);
      cloud.is_dense = cloud.is_dense && std::isfinite(point.x) &&
                       std::isfinite(point.y) && std::isfinite(point.z);
    }
    lidar_pub_->publish(cloud);
  }

  void on_joint(sensor_msgs::msg::JointState::ConstSharedPtr msg) {
    const auto it = std::find(msg->name.begin(), msg->name.end(), "lidar_joint");
    if (it == msg->name.end()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "JointState has no lidar_joint; motor message skipped");
      return;
    }
    const auto index = static_cast<std::size_t>(it - msg->name.begin());
    if (index >= msg->position.size() || !std::isfinite(msg->position[index])) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "lidar_joint position missing or nonfinite; motor message skipped");
      return;
    }

    constexpr double two_pi = 6.283185307179586476925286766559;
    double raw_angle = msg->position[index];
    if (motor_zero_rad_ != 0.0 || motor_direction_ != 1) {
      raw_angle = motor_direction_ * (raw_angle - motor_zero_rad_);
    }
    double angle = std::fmod(raw_angle, two_pi);
    if (angle < 0.0) angle += two_pi;
    rigelslam_rot::msg::Motor motor;
    motor.header = msg->header;
    if (motor_time_shift_s_ != 0.0) {
      motor.header.stamp = static_cast<builtin_interfaces::msg::Time>(
          rclcpp::Time(msg->header.stamp) +
          rclcpp::Duration::from_seconds(motor_time_shift_s_));
    }
    motor.motor_angle = static_cast<float>(angle * (180.0 / 3.1415926535897932384626433832795));
    motor_pub_->publish(motor);
  }

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<rigelslam_rot::msg::Motor>::SharedPtr motor_pub_;
  rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr lidar_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  double motor_zero_rad_ = 0.0;
  int motor_direction_ = 1;
  double motor_time_shift_s_ = 0.0;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<R3lioSensorAdapter>());
  rclcpp::shutdown();
  return 0;
}
