#include <chrono>
#include <memory>

#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>

// Guides an interceptor from a fixed launcher to the target, using the
// estimated state (position + velocity) published by the estimator.
//
// Guidance law: each tick, roll the target's estimated state forward with
// the ballistic model and aim at the EARLIEST point the interceptor can
// reach in time flying straight at constant speed (the collision vector).
class InterceptorNode : public rclcpp::Node {
public:
  InterceptorNode() : Node("interceptor_node") {
    auto launch = declare_parameter<std::vector<double>>(
        "launch_position", {50.0, 10.0, 0.0});
    launch_position_ = Eigen::Vector3d(launch[0], launch[1], launch[2]);
    speed_ = declare_parameter<double>("speed", 40.0);
    gravity_ = declare_parameter<double>("gravity", 9.81);
    update_rate_hz_ = declare_parameter<double>("update_rate", 50.0);

    position_ = launch_position_;

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    marker_pub_ = create_publisher<visualization_msgs::msg::Marker>(
        "interceptor/markers", 10);

    pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        "estimator/pose", 10,
        [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
          target_pos_ = Eigen::Vector3d(msg->pose.position.x,
                                        msg->pose.position.y,
                                        msg->pose.position.z);
          last_estimate_stamp_ = now();
        });
    velocity_sub_ = create_subscription<geometry_msgs::msg::Vector3Stamped>(
        "estimator/velocity", 10,
        [this](const geometry_msgs::msg::Vector3Stamped::SharedPtr msg) {
          target_vel_ = Eigen::Vector3d(msg->vector.x, msg->vector.y,
                                        msg->vector.z);
        });

    timer_ = create_wall_timer(
        std::chrono::duration<double>(1.0 / update_rate_hz_),
        std::bind(&InterceptorNode::update, this));
  }

private:
  void update() {
    const double dt = 1.0 / update_rate_hz_;
    const bool estimate_fresh =
        last_estimate_stamp_.nanoseconds() > 0 &&
        (now() - last_estimate_stamp_).seconds() < 1.0;

    if (estimate_fresh) {
      const Eigen::Vector3d aim = computeAimPoint();
      const Eigen::Vector3d direction = (aim - position_).normalized();
      position_ += direction * speed_ * dt;
    }

    publishPose();
  }

  // Roll the target's estimated state forward with the ballistic model and
  // return the earliest point the interceptor can reach in time.
  Eigen::Vector3d computeAimPoint() const {
    Eigen::Vector3d p = target_pos_;
    Eigen::Vector3d v = target_vel_;
    const double step = 0.05;
    for (double t = 0.0; t < 30.0; t += step) {
      if ((p - position_).norm() <= speed_ * t) {
        return p;  // we can be there when the target is
      }
      p += v * step;
      v.z() -= gravity_ * step;
      if (p.z() <= 0.0) {
        break;  // target reaches the ground before we reach it
      }
    }
    return p;  // fallback: aim at the impact point
  }

  void publishPose() {
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = now();
    tf_msg.header.frame_id = "world";
    tf_msg.child_frame_id = "interceptor";
    tf_msg.transform.translation.x = position_.x();
    tf_msg.transform.translation.y = position_.y();
    tf_msg.transform.translation.z = position_.z();
    tf_msg.transform.rotation.w = 1.0;
    tf_broadcaster_->sendTransform(tf_msg);

    // Blue sphere for the interceptor
    visualization_msgs::msg::Marker marker;
    marker.header.stamp = tf_msg.header.stamp;
    marker.header.frame_id = "world";
    marker.ns = "interceptor";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = position_.x();
    marker.pose.position.y = position_.y();
    marker.pose.position.z = position_.z();
    marker.pose.orientation.w = 1.0;
    marker.scale.x = marker.scale.y = marker.scale.z = 0.6;
    marker.color.b = 1.0;
    marker.color.a = 1.0;
    marker_pub_->publish(marker);
  }

  Eigen::Vector3d launch_position_;
  double speed_;
  double gravity_;
  double update_rate_hz_;

  Eigen::Vector3d position_;
  Eigen::Vector3d target_pos_{0.0, 0.0, 0.0};
  Eigen::Vector3d target_vel_{0.0, 0.0, 0.0};
  rclcpp::Time last_estimate_stamp_{0, 0, RCL_ROS_TIME};

  rclcpp::TimerBase::SharedPtr timer_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr velocity_sub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<InterceptorNode>());
  rclcpp::shutdown();
  return 0;
}
