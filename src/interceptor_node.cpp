#include <chrono>
#include <cmath>
#include <memory>

#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "air_defense_sim/explosion.hpp"

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
    hit_radius_ = declare_parameter<double>("hit_radius", 1.5);
    gravity_ = declare_parameter<double>("gravity", 9.81);
    update_rate_hz_ = declare_parameter<double>("update_rate", 50.0);
    cooldown_ = declare_parameter<double>("cooldown", 3.0);
    // A real missile only has so much fuel: if it has not reached the
    // target within this many seconds it burns out and the target gets
    // through. This is what makes an interception able to FAIL.
    max_flight_time_ = declare_parameter<double>("max_flight_time", 2.0);
    trail_step_ = declare_parameter<double>("trail_step", 0.5);

    // The air kill: a quick, tight fireball (the ground miss drawn by the
    // target node is deliberately bigger and slower).
    explosion_.ns = "air_explosion";
    explosion_.duration = 1.2;
    explosion_.max_radius = 6.0;

    position_ = launch_position_;

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        "interceptor/markers", 10);
    // Announces a successful interception; the target node listens to this
    // and sends in a fresh target instead of letting the old one land.
    hit_pub_ = create_publisher<geometry_msgs::msg::PointStamped>(
        "interceptor/hit", 10);

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

    relaunch_time_ = now();
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

    if (!flying_) {
      // waiting at the launcher: engage as soon as a target is tracked
      if (estimate_fresh && now() >= relaunch_time_) {
        flying_ = true;
        launch_time_ = now();
        trail_.clear();  // a new missile draws a new trail
      }
    } else if (!estimate_fresh) {
      // track lost: return to base
      returnToBase();
    } else if ((now() - launch_time_).seconds() > max_flight_time_) {
      // out of fuel: this missile is spent, reload and let the next one try
      RCLCPP_WARN(get_logger(), "BURNOUT: interceptor ran out of fuel");
      returnToBase();
      relaunch_time_ = now() + rclcpp::Duration::from_seconds(cooldown_);
    } else {
      const double estimate_age = (now() - last_estimate_stamp_).seconds();
      const Eigen::Vector3d aim = computeAimPoint(estimate_age);
      const Eigen::Vector3d direction = (aim - position_).normalized();
      position_ += direction * speed_ * dt;
      appendToTrail();
      checkHit();
    }

    publishPose();
  }

  // Roll the target's estimated state forward with the ballistic model and
  // return the earliest point the interceptor can reach in time.
  Eigen::Vector3d computeAimPoint(double estimate_age) const {
    Eigen::Vector3d p = target_pos_;
    Eigen::Vector3d v = target_vel_;
    // the estimate is up to one radar period old (the estimator only
    // publishes on measurements): catch it up to the present first
    p += v * estimate_age;
    p.z() -= 0.5 * gravity_ * estimate_age * estimate_age;
    v.z() -= gravity_ * estimate_age;
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

  // The hit check is the "referee": it compares against the ground-truth
  // target frame from tf, not the estimate.
  void checkHit() {
    geometry_msgs::msg::TransformStamped tf_msg;
    try {
      tf_msg = tf_buffer_->lookupTransform("world", "target",
                                           tf2::TimePointZero);
    } catch (const tf2::TransformException&) {
      return;
    }
    const Eigen::Vector3d true_target(tf_msg.transform.translation.x,
                                      tf_msg.transform.translation.y,
                                      tf_msg.transform.translation.z);
    if ((true_target - position_).norm() > hit_radius_) {
      return;
    }
    if (true_target.z() <= 0.0) {
      return;  // it has already landed: too late, that one is a miss
    }

    RCLCPP_INFO(get_logger(), "HIT: target intercepted at (%.1f, %.1f, %.1f)",
                position_.x(), position_.y(), position_.z());

    // Blow up where we are, and tell the target node it is dead.
    const rclcpp::Time stamp = now();
    explosion_.trigger(position_.x(), position_.y(), position_.z(), stamp);

    geometry_msgs::msg::PointStamped hit;
    hit.header.stamp = stamp;
    hit.header.frame_id = "world";
    hit.point.x = position_.x();
    hit.point.y = position_.y();
    hit.point.z = position_.z();
    hit_pub_->publish(hit);

    // back to base, wait for the next target
    returnToBase();
    relaunch_time_ = stamp + rclcpp::Duration::from_seconds(cooldown_);
  }

  void returnToBase() {
    flying_ = false;
    position_ = launch_position_;
    trail_.clear();
  }

  // Same idea as the target's trail: only store a point every trail_step_
  // metres, so the line strip stays light.
  void appendToTrail() {
    if (!trail_.empty()) {
      const auto& last = trail_.back();
      const Eigen::Vector3d d(position_.x() - last.x, position_.y() - last.y,
                              position_.z() - last.z);
      if (d.norm() < trail_step_) {
        return;
      }
    }
    geometry_msgs::msg::Point p;
    p.x = position_.x();
    p.y = position_.y();
    p.z = position_.z();
    trail_.push_back(p);
    if (trail_.size() > max_trail_points_) {
      trail_.erase(trail_.begin());
    }
  }

  void publishPose() {
    const rclcpp::Time stamp = now();

    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = stamp;
    tf_msg.header.frame_id = "world";
    tf_msg.child_frame_id = "interceptor";
    tf_msg.transform.translation.x = position_.x();
    tf_msg.transform.translation.y = position_.y();
    tf_msg.transform.translation.z = position_.z();
    tf_msg.transform.rotation.w = 1.0;
    tf_broadcaster_->sendTransform(tf_msg);

    visualization_msgs::msg::MarkerArray array;

    // The missile: a cyan sphere.
    visualization_msgs::msg::Marker body;
    body.header.stamp = stamp;
    body.header.frame_id = "world";
    body.ns = "interceptor";
    body.id = 0;
    body.type = visualization_msgs::msg::Marker::SPHERE;
    body.action = visualization_msgs::msg::Marker::ADD;
    body.pose.position.x = position_.x();
    body.pose.position.y = position_.y();
    body.pose.position.z = position_.z();
    body.pose.orientation.w = 1.0;
    body.scale.x = body.scale.y = body.scale.z = 1.2;
    body.color.g = 0.9;
    body.color.b = 1.0;
    body.color.a = 1.0;
    array.markers.push_back(body);

    // The path it has flown so far.
    if (trail_.size() >= 2) {
      visualization_msgs::msg::Marker trail;
      trail.header = body.header;
      trail.ns = "interceptor_trail";
      trail.id = 1;
      trail.type = visualization_msgs::msg::Marker::LINE_STRIP;
      trail.action = visualization_msgs::msg::Marker::ADD;
      trail.pose.orientation.w = 1.0;
      trail.scale.x = 0.3;
      trail.color.g = 0.9;
      trail.color.b = 1.0;
      trail.color.a = 0.9;
      trail.points = trail_;
      array.markers.push_back(trail);
    }

    // The interception fireball, while it is still playing.
    if (explosion_.active(stamp)) {
      const auto blast = explosion_.markers(stamp);
      array.markers.insert(array.markers.end(), blast.markers.begin(),
                           blast.markers.end());
    }

    marker_pub_->publish(array);
  }

  Eigen::Vector3d launch_position_;
  double speed_;
  double hit_radius_;
  double gravity_;
  double update_rate_hz_;
  double cooldown_;
  double max_flight_time_;
  double trail_step_;

  Eigen::Vector3d position_;
  bool flying_ = false;
  Eigen::Vector3d target_pos_{0.0, 0.0, 0.0};
  Eigen::Vector3d target_vel_{0.0, 0.0, 0.0};
  rclcpp::Time last_estimate_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time relaunch_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time launch_time_{0, 0, RCL_ROS_TIME};

  std::vector<geometry_msgs::msg::Point> trail_;
  const size_t max_trail_points_ = 400;
  Explosion explosion_;

  rclcpp::TimerBase::SharedPtr timer_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr velocity_sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr hit_pub_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<InterceptorNode>());
  rclcpp::shutdown();
  return 0;
}
