#pragma once

#include "ackermann_msgs/msg/ackermann_drive.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "optional"
#include "rclcpp/rclcpp.hpp"

namespace tta
{
class TwistToAckermann : public rclcpp::Node
{
public:
  explicit TwistToAckermann(rclcpp::NodeOptions options);

private:
  std::optional<std::shared_ptr<rclcpp::Subscription<geometry_msgs::msg::Twist>>>
    _twists_sub = std::nullopt;
  std::optional<std::shared_ptr<rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>>>
    _acks_pub = std::nullopt;

  /// Wheelbase in meters.
  double _wheelbase{};
  
  void twists_cb(geometry_msgs::msg::Twist::SharedPtr msg);
};
}  // namespace tta