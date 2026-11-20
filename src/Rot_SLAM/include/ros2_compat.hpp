#pragma once

// ROS transport adapter for the preserved R3LIO processing code.
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/int32.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace sensor_msgs {
using Image = msg::Image;
using ImagePtr = msg::Image::SharedPtr;
using Imu = msg::Imu;
using ImuConstPtr = msg::Imu::ConstSharedPtr;
using PointCloud2 = msg::PointCloud2;
using PointCloud2ConstPtr = msg::PointCloud2::ConstSharedPtr;
}
namespace std_msgs {
using Header = msg::Header;
using Int32 = msg::Int32;
}
namespace geometry_msgs {
using Point = msg::Point;
using PoseStamped = msg::PoseStamped;
using Vector3 = msg::Vector3;
}
namespace nav_msgs {
using Odometry = msg::Odometry;
using Path = msg::Path;
}
namespace visualization_msgs {
using Marker = msg::Marker;
using MarkerArray = msg::MarkerArray;
}

namespace ros {
inline rclcpp::Node::SharedPtr &node_storage() {
  static rclcpp::Node::SharedPtr node;
  return node;
}
inline void init(int argc, char **argv, const std::string &name) {
  rclcpp::init(argc, argv);
  node_storage() = std::make_shared<rclcpp::Node>(name);
}
inline bool ok() { return rclcpp::ok(); }
inline void shutdown() { rclcpp::shutdown(); }
inline void spinOnce() { rclcpp::spin_some(node_storage()); }

struct Time : public builtin_interfaces::msg::Time {
  Time &fromSec(double seconds) {
    if (seconds < 0 || !std::isfinite(seconds))
      throw std::runtime_error("ROS time must be finite and nonnegative");
    const double whole = std::floor(seconds);
    if (whole > std::numeric_limits<int32_t>::max())
      throw std::runtime_error("ROS 2 time is out of signed 32-bit seconds range");
    sec = static_cast<int32_t>(whole);
    nanosec = static_cast<uint32_t>(std::round((seconds - sec) * 1000000000.0));
    if (nanosec >= 1000000000ul && sec == std::numeric_limits<int32_t>::max())
      throw std::runtime_error("ROS 2 time is out of signed 32-bit seconds range");
    sec += static_cast<int32_t>(nanosec / 1000000000ul);
    nanosec %= 1000000000ul;
    return *this;
  }
  static Time now() {
    Time result;
    builtin_interfaces::msg::Time stamp = node_storage()->now();
    result.sec = stamp.sec;
    result.nanosec = stamp.nanosec;
    return result;
  }
};

inline double toSec(const builtin_interfaces::msg::Time &stamp) {
  return static_cast<double>(stamp.sec) +
         1e-9 * static_cast<double>(stamp.nanosec);
}

struct TransportHints {
  TransportHints tcpNoDelay() const { return *this; }
};

class Publisher {
 public:
  Publisher() = default;
  explicit Publisher(rclcpp::PublisherBase::SharedPtr publisher)
      : publisher_(std::move(publisher)) {}
  template <typename Message> void publish(const Message &message) const {
    auto typed = std::dynamic_pointer_cast<rclcpp::Publisher<Message>>(publisher_);
    if (!typed) throw std::runtime_error("ROS 2 publisher message type mismatch");
    typed->publish(message);
  }
  template <typename Message> void publish(const std::shared_ptr<Message> &message) const {
    publish(*message);
  }
  size_t getNumSubscribers() const {
    return publisher_ ? publisher_->get_subscription_count() : 0;
  }
 private:
  rclcpp::PublisherBase::SharedPtr publisher_;
};

using Subscriber = rclcpp::SubscriptionBase::SharedPtr;

class NodeHandle {
 public:
  NodeHandle() : node_(node_storage()) {}
  template <typename T> void param(const std::string &name, T &value,
                                   const T &default_value) {
    std::string key = name;
    std::replace(key.begin(), key.end(), '/', '.');
    value = node_->declare_parameter<T>(key, default_value);
  }
  template <typename Message> Publisher advertise(const std::string &topic,
                                                   size_t depth) {
    return Publisher(node_->create_publisher<Message>(topic, rclcpp::QoS(depth)));
  }
  template <typename Message>
  Subscriber subscribe(const std::string &topic, size_t depth,
                       void (*callback)(const typename Message::ConstSharedPtr &)) {
    return node_->create_subscription<Message>(topic, rclcpp::QoS(depth), callback);
  }
  template <typename Message>
  Subscriber subscribe(const std::string &topic, size_t depth,
                       void (*callback)(const typename Message::ConstSharedPtr &),
                       std::nullptr_t, const TransportHints &) {
    return subscribe<Message>(topic, depth, callback);
  }
 private:
  rclcpp::Node::SharedPtr node_;
};
}  // namespace ros

#define ROS_ERROR(...) RCLCPP_ERROR(ros::node_storage()->get_logger(), __VA_ARGS__)
